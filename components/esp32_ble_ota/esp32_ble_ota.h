#pragma once

/**
 * ESPHome BLE OTA Component
 * ========================
 * Implements Over-The-Air firmware updates via Bluetooth Low Energy.
 * Modeled after the ESPHome WiFi OTA platform, using the same OTAComponent
 * base class so it integrates seamlessly with ESPHome's OTA automation system.
 *
 * Architecture:
 *   - Registers as a GATT service with three characteristics:
 *       CONTROL  (write)  — command/handshake channel
 *       DATA     (write)  — raw firmware chunk channel
 *       STATUS   (notify) — ACK / progress / error responses
 *   - Implements a stop-and-wait ARQ: the host sends one DATA chunk,
 *     the device ACKs (or NACKs with a reason byte), then the host sends
 *     the next chunk.
 *   - CRC-32 is verified per-chunk AND over the whole firmware image.
 *   - On success the device schedules a reboot via App.safe_reboot().
 *
 * BLE UUIDs (128-bit):
 *   Service  : 0000181C-0000-1000-8000-00805F9B34FB
 *   Control  : 00002B10-0000-1000-8000-00805F9B34FB
 *   Data     : 00002B11-0000-1000-8000-00805F9B34FB
 *   Status   : 00002B12-0000-1000-8000-00805F9B34FB
 *
 * Wire protocol (little-endian throughout):
 *
 *   CONTROL writes (host → device):
 *     [0x01] [4B total_size] [4B total_crc32]   — OTA_BEGIN
 *     [0x02]                                     — OTA_ABORT
 *     [0x03]                                     — OTA_COMMIT  (final check + reboot)
 *
 *   DATA writes (host → device):
 *     [2B seq_num] [2B chunk_len] [4B chunk_crc32] [<chunk_len> bytes payload]
 *
 *   STATUS notifications (device → host):
 *     [0x01] [2B seq_num]           — ACK (chunk accepted, send next)
 *     [0x02] [2B seq_num] [1B code] — NACK (error; host should retry or abort)
 *     [0x03] [1B progress_pct]      — PROGRESS (informational, every ~5%)
 *     [0x04]                        — DONE (reboot imminent)
 *     [0x05] [1B code]              — ERROR (fatal; session ended)
 *
 *   NACK / ERROR codes:
 *     0x01 — CRC_MISMATCH    chunk CRC did not match
 *     0x02 — SEQ_ERROR       unexpected sequence number
 *     0x03 — WRITE_FAILED    flash write returned error
 *     0x04 — FINAL_CRC_FAIL  whole-image CRC mismatch after all chunks
 *     0x05 — SIZE_MISMATCH   bytes written ≠ declared total_size
 *     0x06 — BEGIN_FAILED    OTABackend::begin() failed
 *     0x07 — TIMEOUT         host went silent mid-transfer
 *     0x0F — INTERNAL        unexpected internal error
 */

#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/esp32_ble_server/ble_server.h"
#include "esphome/components/esp32_ble_server/ble_service.h"
#include "esphome/components/esp32_ble_server/ble_characteristic.h"
#include "esphome/components/esp32_ble_server/ble_descriptor.h"
#include "esphome/components/ota/ota_backend.h"
#include "esphome/components/ota/ota_backend_esp_idf.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <vector>

namespace esphome {
namespace esp32_ble_ota {

// ── UUIDs ──────────────────────────────────────────────────────────────────
// static const char *const ESP32_BLE_OTA_SERVICE_UUID = "0000181C-0000-1000-8000-00805F9B34FB";
// static const char *const ESP32_BLE_OTA_CONTROL_UUID = "00002B10-0000-1000-8000-00805F9B34FB";
// static const char *const ESP32_BLE_OTA_DATA_UUID    = "00002B11-0000-1000-8000-00805F9B34FB";
// static const char *const ESP32_BLE_OTA_STATUS_UUID  = "00002B12-0000-1000-8000-00805F9B34FB";

// ── Protocol constants ──────────────────────────────────────────────────────
static const uint8_t CTRL_BEGIN  = 0x01;
static const uint8_t CTRL_ABORT  = 0x02;
static const uint8_t CTRL_COMMIT = 0x03;

static const uint8_t STATUS_ACK      = 0x01;
static const uint8_t STATUS_NACK     = 0x02;
static const uint8_t STATUS_PROGRESS = 0x03;
static const uint8_t STATUS_DONE     = 0x04;
static const uint8_t STATUS_ERROR    = 0x05;

static const uint8_t ERR_CRC_MISMATCH   = 0x01;
static const uint8_t ERR_SEQ_ERROR      = 0x02;
static const uint8_t ERR_WRITE_FAILED   = 0x03;
static const uint8_t ERR_FINAL_CRC_FAIL = 0x04;
static const uint8_t ERR_SIZE_MISMATCH  = 0x05;
static const uint8_t ERR_BEGIN_FAILED   = 0x06;
static const uint8_t ERR_TIMEOUT        = 0x07;
static const uint8_t ERR_BAD_PASSWORD   = 0x08;
static const uint8_t ERR_INTERNAL       = 0x0F;

// Maximum time (ms) between consecutive DATA packets before timeout
// static const uint32_t ESP32_BLE_OTA_CHUNK_TIMEOUT_MS = 3000; //30000;

// Maximum firmware chunk payload size
// static const size_t ESP32_BLE_OTA_MAX_CHUNK_SIZE = 500;

// Send a PROGRESS notification every this many percent
// static const uint8_t ESP32_BLE_OTA_PROGRESS_STEP = 5;

// ── Session state machine ────────────────────────────────────────────────────
enum class ESP32BLEOTAState : uint8_t {
  IDLE,         // waiting for OTA_BEGIN
  IN_PROGRESS,  // receiving chunks
  COMMITTING,   // OTA_COMMIT received, verifying & rebooting
};

// ── Main component ────────────────────────────────────────────────────────────
//
// Inherits only from ota::OTAComponent — which already inherits from Component.
// Do NOT also inherit Component directly; that creates a diamond and an
// ambiguous base error.
class ESP32BLEOTAComponent : public ota::OTAComponent {
 public:
  // Component lifecycle
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Called from BLECharacteristic::on_write() lambdas registered in setup()
  void handle_control(std::span<const uint8_t> data);
  void handle_data(std::span<const uint8_t> data);

  void set_password(const std::string &pw){ this->password_ = pw; }
  void set_service_uuid(const std::string &uuid) { this->service_uuid_ = uuid; }
  void set_control_uuid(const std::string &uuid) { this->control_uuid_ = uuid; }
  void set_data_uuid(const std::string &uuid)    { this->data_uuid_    = uuid; }
  void set_status_uuid(const std::string &uuid)  { this->status_uuid_  = uuid; }
  void set_max_chunk_size(size_t size)       { this->max_chunk_size_    = size; }
  void set_chunk_timeout_ms(uint32_t ms)     { this->chunk_timeout_ms_  = ms; }
  void set_progress_step(uint8_t step)       { this->progress_step_     = step; }
  

 protected:
  // CRC-32 (ISO 3309 / Ethernet polynomial 0xEDB88320, reflected)
  static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len);
  static uint32_t crc32_finalize(uint32_t crc);
  static uint32_t crc32_compute(const uint8_t *data, size_t len);

  // Status notification helpers
  void send_status(std::vector<uint8_t> payload);
  void send_ack(uint16_t seq);
  void send_nack(uint16_t seq, uint8_t code);
  void send_progress(uint8_t pct);
  void send_done();
  void send_error(uint8_t code);

  void abort_session(uint8_t code);
  void reset_session();

  // BLE objects — raw pointers, owned by the BLE server stack
  esp32_ble_server::BLEService *service_{nullptr};
  esp32_ble_server::BLECharacteristic *control_char_{nullptr};
  esp32_ble_server::BLECharacteristic *data_char_{nullptr};
  esp32_ble_server::BLECharacteristic *status_char_{nullptr};

  // Session state
  ESP32BLEOTAState state_{ESP32BLEOTAState::IDLE};
  uint32_t    total_size_{0};
  uint32_t    total_crc32_expected_{0};
  uint32_t    bytes_written_{0};
  uint32_t    image_crc_running_{0xFFFFFFFFu};
  uint16_t    next_seq_{0};
  uint32_t    last_chunk_time_{0};
  uint8_t     last_progress_pct_{0};

  std::string password_;

  // Set to true once is_running() was seen and characteristics were registered
  bool ble_setup_done_{false};
  bool service_started_{false};
  bool pending_begin_ack_{false};

  std::string service_uuid_{};
  std::string control_uuid_{};
  std::string data_uuid_   {};
  std::string status_uuid_ {};
  size_t max_chunk_size_{500};
  uint32_t chunk_timeout_ms_{3000};
  uint8_t progress_step_{5};


  // ESP-IDF OTA backend — handles flash partition writes
  std::unique_ptr<ota::IDFOTABackend> backend_;
};

}  // namespace esp32_ble_ota
}  // namespace esphome

#endif  // USE_ESP32