#include "esp32_ble_ota.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace esp32_ble_ota {

static const char *const TAG = "esp32_ble_ota";

// ─────────────────────────────────────────────────────────────────────────────
// CRC-32 (ISO 3309 / Ethernet polynomial 0xEDB88320, reflected)
// ─────────────────────────────────────────────────────────────────────────────

static uint32_t s_crc32_table[256];
static bool     s_crc32_ready = false;

static void crc32_init() {
  if (s_crc32_ready)
    return;
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int j = 0; j < 8; j++)
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    s_crc32_table[i] = c;
  }
  s_crc32_ready = true;
}

uint32_t ESP32BLEOTAComponent::crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  crc32_init();
  for (size_t i = 0; i < len; i++)
    crc = s_crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return crc;
}

uint32_t ESP32BLEOTAComponent::crc32_finalize(uint32_t crc) {
  return crc ^ 0xFFFFFFFFu;
}

uint32_t ESP32BLEOTAComponent::crc32_compute(const uint8_t *data, size_t len) {
  return crc32_finalize(crc32_update(0xFFFFFFFFu, data, len));
}

// ─────────────────────────────────────────────────────────────────────────────
// Component lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void ESP32BLEOTAComponent::setup() {
  crc32_init();

  if (esp32_ble_server::global_ble_server == nullptr) {
    ESP_LOGE(TAG, "No BLE server found — add 'esp32_ble_server:' to your config");
    this->mark_failed();
    return;
  }

  // Service and characteristic registration is deferred to loop() so that it
  // runs only after the BLE server reports is_running() == true.  Calling
  // create_service() before the Bluedroid GATTS layer is ready causes
  // esp_ble_gatts_add_char to fail with error 259 (ESP_ERR_INVALID_ARG).
  this->reset_session();
}

void ESP32BLEOTAComponent::loop() {
  // One-time deferred BLE service registration — waits for the BLE stack
  if (!this->ble_setup_done_) {
    auto *server = esp32_ble_server::global_ble_server;
    if (server == nullptr || !server->is_running())
      return;

    // Handle budget per Bluedroid with 128-bit UUIDs:
    //   1  service declaration
    //   3  characteristics x 2 handles (declaration + value) = 6
    //   3  characteristics x 1 extra handle for 128-bit UUID attr = 3
    //   1  CCCD descriptor on STATUS characteristic
    //   = 11 minimum. Use 30 to give definitive headroom; unused slots are free.
    this->service_ = server->create_service(
        esp32_ble::ESPBTUUID::from_raw(this->service_uuid_), true, 30);

    // CONTROL characteristic — write with response
    this->control_char_ = this->service_->create_characteristic(
        this->control_uuid_,
        esp32_ble_server::BLECharacteristic::PROPERTY_WRITE);
    this->control_char_->on_write([this](std::span<const uint8_t> data, uint16_t) {
      this->handle_control(data);
    });

    // DATA characteristic — write without response for higher throughput
    this->data_char_ = this->service_->create_characteristic(
        this->data_uuid_,
        esp32_ble_server::BLECharacteristic::PROPERTY_WRITE_NR);
    this->data_char_->on_write([this](std::span<const uint8_t> data, uint16_t) {
      this->handle_data(data);
    });

    // STATUS characteristic — notify (device -> host)
    this->status_char_ = this->service_->create_characteristic(
        this->status_uuid_,
        esp32_ble_server::BLECharacteristic::PROPERTY_NOTIFY |
        esp32_ble_server::BLECharacteristic::PROPERTY_READ);

    // Manually add the CCCD (0x2902) descriptor — ESPHome does NOT add this
    // automatically when creating characteristics from C++ code (only the YAML
    // path does). Without it clients cannot subscribe to notifications: the
    // descriptor handle is absent from the GATT table and CoreBluetooth /
    // Android will return "attribute not found" on start_notify.
    // Initial value 0x0000 = notifications disabled (client enables on connect).
    auto *cccd = new esp32_ble_server::BLEDescriptor(  // NOLINT
        esp32_ble::ESPBTUUID::from_uint16(ESP_GATT_UUID_CHAR_CLIENT_CONFIG), 2);
    cccd->set_value({0x00, 0x00});
    this->status_char_->add_descriptor(cccd);

    // do_create() was called inside create_service() since the server is already
    // RUNNING. We must not call start() yet — the service handle is assigned
    // asynchronously via ESP_GATTS_CREATE_EVT. Poll is_created() in loop() and
    // call start() only once the handle is valid.
    this->ble_setup_done_ = true;
    

    ESP_LOGD(TAG, "ESP32 BLE OTA Starting...");
    return;
  }

  // send the begin ack once if pending
  if (this->pending_begin_ack_) {
    this->pending_begin_ack_ = false;
    this->send_ack(0xFFFF);
  }

  // Phase 2: wait for the service's async GATTS handle, then start it.
  // create_service() called do_create() which fires esp_ble_gatts_create_service;
  // is_created() becomes true once ESP_GATTS_CREATE_EVT is processed and the
  // handle is assigned. Only then can start() call esp_ble_gatts_start_service.
  if ( !this->service_started_ ){
    if (this->service_ != nullptr && !this->service_->is_running()) {
      if (this->service_->is_created()) {
        this->service_->start(); // TODO: this triggers several times, but eventually works?
      }
      return;
    } else if ( this->service_->is_running() ){
      // track a manual flag to gate the whole loop here, and only print this 'started' statement once
      this->service_started_ = true;
      ESP_LOGI(TAG, "ESP32 BLE OTA service started");
    }
  }


  // Timeout watchdog: abort if host goes silent mid-transfer
  if (this->state_ == ESP32BLEOTAState::IN_PROGRESS) {
    if (millis() - this->last_chunk_time_ > this->chunk_timeout_ms_) {
      ESP_LOGW(TAG, "OTA timeout — no data for %u ms", this->chunk_timeout_ms_);
      this->abort_session(ERR_TIMEOUT);
    }
  }
}

void ESP32BLEOTAComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP32 BLE OTA:");
  ESP_LOGCONFIG(TAG, "  Service UUID : %s", this->service_uuid_.c_str());
  ESP_LOGCONFIG(TAG, "    Control UUID : %s", this->control_uuid_.c_str());
  ESP_LOGCONFIG(TAG, "    Data UUID    : %s", this->data_uuid_.c_str());
  ESP_LOGCONFIG(TAG, "    Status UUID  : %s", this->status_uuid_.c_str());
  ESP_LOGCONFIG(TAG, "  Max chunk    : %u bytes", this->max_chunk_size_);
  ESP_LOGCONFIG(TAG, "  Timeout      : %u ms", this->chunk_timeout_ms_);
}

// ─────────────────────────────────────────────────────────────────────────────
// CONTROL channel handler
//
//   OTA_BEGIN  : [0x01] [4B total_size LE] [4B total_crc32 LE][1B pw_len][pw_len bytes password]
//   OTA_ABORT  : [0x02]
//   OTA_COMMIT : [0x03]
// ─────────────────────────────────────────────────────────────────────────────
void ESP32BLEOTAComponent::handle_control(std::span<const uint8_t> data) {
  if (data.empty())
    return;

  switch (data[0]) {

    case CTRL_BEGIN: {
      if (data.size() < 10) {
        ESP_LOGW(TAG, "Invalid Begin packet, sending error");
        this->send_error(ERR_INTERNAL);
        return;
      }
      if (this->state_ != ESP32BLEOTAState::IDLE) {
        ESP_LOGW(TAG, "ESP32 BLE OTA State != IDLE, aborting with internal error");
        this->abort_session(ERR_INTERNAL);
      }

      uint32_t total_size = (uint32_t) data[1]        |
                            ((uint32_t) data[2] << 8)  |
                            ((uint32_t) data[3] << 16) |
                            ((uint32_t) data[4] << 24);
      uint32_t total_crc  = (uint32_t) data[5]        |
                            ((uint32_t) data[6] << 8)  |
                            ((uint32_t) data[7] << 16) |
                            ((uint32_t) data[8] << 24);

      if (total_size == 0) {
        ESP_LOGW(TAG, "Total size is 0, sending error");
        this->send_error(ERR_INTERNAL);
        return;
      }

      // validate the password in the begin packet
      uint8_t pw_len = data[9];
      if (pw_len > 64 || data.size() < (size_t)(10 + pw_len)) {
        ESP_LOGW(TAG, "OTA BEGIN: malformed password field");
        this->send_error(ERR_BAD_PASSWORD);
        return;
      }
      if (!this->password_.empty()){
        bool ok = (pw_len == this->password_.size()) &&
          (memcmp(data.data() + 10, this->password_.data(), pw_len) == 0);
        if (!ok) {
          ESP_LOGW(TAG, "OTA BEGIN: wrong password - rejecting");
          this->send_error(ERR_BAD_PASSWORD);
          return;
        }
      }


      // hard code the backend to the ESP-IDF backend - it's the only one supported (tested?) for this approach.
      // this _could_ refactor to use the backend_factory and possibly support other devices - which I don't have to test
      this->backend_ = std::make_unique<ota::IDFOTABackend>();
      auto result = this->backend_->begin(total_size);
      if (result != ota::OTA_RESPONSE_OK) {
        ESP_LOGE(TAG, "OTABackend::begin() failed: %u", (unsigned) result);
        this->backend_.reset();
        this->send_error(ERR_BEGIN_FAILED);
        return;
      }

      this->total_size_           = total_size;
      this->total_crc32_expected_ = total_crc;
      this->bytes_written_        = 0;
      this->image_crc_running_    = 0xFFFFFFFFu;
      this->next_seq_             = 0;
      this->last_chunk_time_      = millis();
      this->last_progress_pct_    = 0;
      this->state_                = ESP32BLEOTAState::IN_PROGRESS;

      ESP_LOGI(TAG, "OTA BEGIN  size=%u  crc=0x%08X",
               (unsigned) total_size, (unsigned) total_crc);

      // defer sending ack until next loop() to avoid occasional esp_ble_gatts_send_indicate failed 259 errors
      // ACK begin with sentinel seq=0xFFFF
      // this->send_ack(0xFFFF);
      this->pending_begin_ack_ = true;
      break;
    }

    case CTRL_ABORT: {
      if (this->state_ != ESP32BLEOTAState::IDLE) {
        ESP_LOGW(TAG, "OTA aborted by host");
        this->abort_session(ERR_INTERNAL);
      }
      break;
    }

    case CTRL_COMMIT: {
      if (this->state_ != ESP32BLEOTAState::IN_PROGRESS) {
        this->send_error(ERR_INTERNAL);
        return;
      }
      this->state_ = ESP32BLEOTAState::COMMITTING;

      // Verify byte count
      if (this->bytes_written_ != this->total_size_) {
        ESP_LOGE(TAG, "Size mismatch: written=%u expected=%u",
                 (unsigned) this->bytes_written_, (unsigned) this->total_size_);
        this->abort_session(ERR_SIZE_MISMATCH);
        return;
      }

      // Verify whole-image CRC-32
      uint32_t final_crc = crc32_finalize(this->image_crc_running_);
      if (final_crc != this->total_crc32_expected_) {
        ESP_LOGE(TAG, "Final CRC mismatch: got=0x%08X expected=0x%08X",
                 (unsigned) final_crc, (unsigned) this->total_crc32_expected_);
        this->abort_session(ERR_FINAL_CRC_FAIL);
        return;
      }

      // Finalise the backend — validates and commits the partition
      auto result = this->backend_->end();
      if (result != ota::OTA_RESPONSE_OK) {
        ESP_LOGE(TAG, "OTABackend::end() failed: %u", (unsigned) result);
        this->abort_session(ERR_WRITE_FAILED);
        return;
      }
      this->backend_.reset();

      ESP_LOGI(TAG, "OTA complete — %u bytes, CRC OK (0x%08X) — rebooting",
               (unsigned) this->bytes_written_, (unsigned) final_crc);

      this->send_progress(100);
      this->send_done();

      // Give the BLE stack a moment to flush the notification before reboot
      delay(1000);
      App.safe_reboot();
      break;
    }

    default:
      ESP_LOGW(TAG, "Unknown CONTROL command: 0x%02X", data[0]);
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// DATA channel handler
//
//   Packet layout:
//     [2B seq_num LE] [2B chunk_len LE] [4B chunk_crc32 LE] [<chunk_len> bytes]
// ─────────────────────────────────────────────────────────────────────────────
void ESP32BLEOTAComponent::handle_data(std::span<const uint8_t> data) {
  if (this->state_ != ESP32BLEOTAState::IN_PROGRESS)
    return;

  static const size_t HDR = 8;  // 2B seq + 2B len + 4B crc
  if (data.size() < HDR + 1) {
    this->send_nack(0xFFFF, ERR_INTERNAL);
    return;
  }

  uint16_t seq       = (uint16_t) data[0] | ((uint16_t) data[1] << 8);
  uint16_t chunk_len = (uint16_t) data[2] | ((uint16_t) data[3] << 8);
  uint32_t chunk_crc = (uint32_t) data[4]        |
                       ((uint32_t) data[5] << 8)  |
                       ((uint32_t) data[6] << 16) |
                       ((uint32_t) data[7] << 24);

  // Sanity checks
  if (data.size() != HDR + chunk_len ||
      chunk_len == 0 ||
      chunk_len > this->max_chunk_size_) {
    this->send_nack(seq, ERR_INTERNAL);
    return;
  }

  // Sequence number check
  if (seq != this->next_seq_) {
    ESP_LOGW(TAG, "SEQ error: expected %u got %u",
             (unsigned) this->next_seq_, (unsigned) seq);
    this->send_nack(seq, ERR_SEQ_ERROR);
    return;
  }

  // Per-chunk CRC-32 verification (before writing to flash)
  const uint8_t *payload = data.data() + HDR;
  if (crc32_compute(payload, chunk_len) != chunk_crc) {
    ESP_LOGW(TAG, "Chunk CRC mismatch seq=%u", (unsigned) seq);
    this->send_nack(seq, ERR_CRC_MISMATCH);
    return;
  }

  // Write via the OTABackend — handles platform differences transparently
  auto result = this->backend_->write(const_cast<uint8_t *>(payload), chunk_len);
  if (result != ota::OTA_RESPONSE_OK) {
    ESP_LOGE(TAG, "OTABackend::write() failed at seq=%u: %u",
             (unsigned) seq, (unsigned) result);
    this->abort_session(ERR_WRITE_FAILED);
    return;
  }

  // Accumulate running whole-image CRC
  this->image_crc_running_ = crc32_update(this->image_crc_running_, payload, chunk_len);
  this->bytes_written_    += chunk_len;
  this->next_seq_         += 1;
  this->last_chunk_time_   = millis();

  // Progress reporting every OTA_BLE_PROGRESS_STEP percent
  uint8_t pct = (uint8_t) ((uint64_t) this->bytes_written_ * 100 / this->total_size_);
  if (pct >= this->last_progress_pct_ + this->progress_step_) {
    this->last_progress_pct_ = pct;
    this->send_progress(pct);
    ESP_LOGD(TAG, "OTA progress: %u%% (%u / %u bytes)",
             (unsigned) pct, (unsigned) this->bytes_written_, (unsigned) this->total_size_);
  }

  // ACK — host may now send the next chunk
  this->send_ack(seq);
}

// ─────────────────────────────────────────────────────────────────────────────
// Status notification helpers
// ─────────────────────────────────────────────────────────────────────────────

void ESP32BLEOTAComponent::send_status(std::vector<uint8_t> payload) {
  if (this->status_char_ == nullptr)
    return;
  this->status_char_->set_value(std::move(payload));
  this->status_char_->notify();
}

void ESP32BLEOTAComponent::send_ack(uint16_t seq) {
  this->send_status({STATUS_ACK, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8)});
}

void ESP32BLEOTAComponent::send_nack(uint16_t seq, uint8_t code) {
  this->send_status({STATUS_NACK, (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8), code});
  ESP_LOGW(TAG, "NACK sent: seq=%u code=0x%02X", (unsigned) seq, code);
}

void ESP32BLEOTAComponent::send_progress(uint8_t pct) {
  this->send_status({STATUS_PROGRESS, pct});
}

void ESP32BLEOTAComponent::send_done() {
  this->send_status({STATUS_DONE});
}

void ESP32BLEOTAComponent::send_error(uint8_t code) {
  this->send_status({STATUS_ERROR, code});
  ESP_LOGE(TAG, "ERROR sent: code=0x%02X", code);
}

// ─────────────────────────────────────────────────────────────────────────────
// Session management
// ─────────────────────────────────────────────────────────────────────────────

void ESP32BLEOTAComponent::abort_session(uint8_t code) {
  if (this->backend_) {
    this->backend_->abort();
    this->backend_.reset();
  }
  this->send_error(code);
  this->reset_session();
  ESP_LOGW(TAG, "OTA session aborted (code=0x%02X)", code);
}

void ESP32BLEOTAComponent::reset_session() {
  this->state_                = ESP32BLEOTAState::IDLE;
  this->total_size_           = 0;
  this->total_crc32_expected_ = 0;
  this->bytes_written_        = 0;
  this->image_crc_running_    = 0xFFFFFFFFu;
  this->next_seq_             = 0;
  this->last_chunk_time_      = 0;
  this->last_progress_pct_    = 0;
  this->pending_begin_ack_    = false;
}

}  // namespace esp32_ble_ota
}  // namespace esphome

#endif  // USE_ESP32