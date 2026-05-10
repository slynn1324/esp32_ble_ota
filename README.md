# ESPHome BLE OTA

An ESPHome external component that enables Over-The-Air firmware updates via Bluetooth Low Energy (BLE) for ESP32 devices. Useful for devices that don't have reliable Wi-Fi access during updates, or where BLE is the only available transport.

## Features

- Firmware upload over BLE using a sliding-window protocol with per-chunk and whole-image CRC-32 verification
- Optional password authentication on the OTA BEGIN handshake
- Configurable BLE service and characteristic UUIDs (useful for deploying multiple independent devices)
- Configurable chunk size, transfer timeout, and progress reporting interval
- Integrates with ESPHome's OTA component system (`on_ota_*` automations work as normal)
- Python uploader script with progress display, retry logic, and pre-flight binary validation

## Requirements

- ESP32 (uses the ESP-IDF OTA backend; ESP8266 is not supported)
- ESPHome 2026.1.0 or later
- `esp32_ble_server:` component enabled in your config
- Python 3.8+ with `bleak` installed (`pip install bleak`) for the host-side uploader

## Installation

Add the component to your ESPHome config as an external component:

```yaml
external_components:
  - source: github://slynn1324/esphome-ble-ota
    components: [esp32_ble_ota]
```

## Basic Configuration

```yaml
esp32_ble:

esp32_ble_server:

ota:
  - platform: esp32_ble_ota
```

## Full Configuration Reference

```yaml
ota:
  - platform: esp32_ble_ota

    # Optional: password checked during OTA BEGIN handshake.
    # Must match the --password argument passed to the uploader.
    # Max 64 bytes, printable ASCII only.
    password: "mysecret"

    # Optional: BLE service and characteristic UUIDs.
    # Override these if you need to avoid conflicts with other BLE services,
    # or to prevent unintended cross-device updates in a multi-device deployment.
    service_uuid: "424F5441-0001-1000-8000-00805F9B34FB"
    control_uuid: "424F5441-0002-1000-8000-00805F9B34FB"
    data_uuid:    "424F5441-0003-1000-8000-00805F9B34FB"
    status_uuid:  "424F5441-0004-1000-8000-00805F9B34FB"

    # Optional: maximum firmware chunk payload size in bytes.
    # Must match --chunk-size passed to the uploader. Default: 500.
    max_chunk_size: 500

    # Optional: how long to wait between chunks before aborting the session.
    # Accepts ESPHome time strings. Default: 3s.
    chunk_timeout: 3s

    # Optional: how often to send PROGRESS notifications to the host.
    # Value is a percentage increment. Default: 5 (every 5%).
    progress_step: 5
```

## Uploading Firmware

Use the included `esp32_ble_ota_upload.py` script to upload a compiled firmware binary.

### Install dependencies

```bash
pip install bleak
```

### Basic usage

```bash
# Connect by device name (scans for the BLE advertisement)
python3 esp32_ble_ota_upload.py --device "my-esp32" firmware.ota.bin

# Connect by MAC address
python3 esp32_ble_ota_upload.py --address AA:BB:CC:DD:EE:FF firmware.ota.bin

# With password
python3 esp32_ble_ota_upload.py --device "my-esp32" --password "mysecret" firmware.ota.bin
```

### All options

```
usage: esp32_ble_ota_upload.py [-h] (--device NAME | --address ADDR)
                         [--password PASSWORD]
                         [--chunk-size BYTES] [--window CHUNKS]
                         [--retries N] [--timeout SECONDS]
                         [--scan-timeout SECONDS] [--verbose]
                         FIRMWARE.bin

  --device, -n      BLE advertisement name to scan for
  --address, -a     BLE MAC address to connect to directly
  --password, -p    OTA password (must match device config)
  --chunk-size, -s  Payload bytes per chunk (default: 500)
  --window, -w      Chunks in-flight before waiting for ACK (default: 8)
  --retries, -r     Per-chunk retry count on NACK (default: 3)
  --timeout, -t     Seconds to wait for any ACK (default: 45)
  --scan-timeout    BLE scan timeout in seconds (default: 10)
  --verbose, -v     Enable debug logging
```

### Which binary file to use

Always use `firmware.ota.bin` — not `firmware.bin` or `firmware.factory.bin`. The uploader validates the binary before connecting and will reject the wrong file type with a clear error message.

## Wire Protocol

The protocol uses three BLE characteristics:

| Characteristic | Direction     | Purpose                        |
|----------------|---------------|--------------------------------|
| CONTROL        | Host → Device | Session commands (BEGIN/ABORT/COMMIT) |
| DATA           | Host → Device | Firmware chunk stream          |
| STATUS         | Device → Host | ACK / NACK / PROGRESS / DONE / ERROR notifications |

All multi-byte integers are little-endian.

### CONTROL packets (host → device)

```
OTA_BEGIN  : [0x01] [4B total_size] [4B total_crc32] [1B pw_len] [pw_len bytes password]
OTA_ABORT  : [0x02]
OTA_COMMIT : [0x03]
```

### DATA packets (host → device)

```
[2B seq_num] [2B chunk_len] [4B chunk_crc32] [<chunk_len> bytes payload]
```

### STATUS notifications (device → host)

```
ACK      : [0x01] [2B seq_num]
NACK     : [0x02] [2B seq_num] [1B error_code]
PROGRESS : [0x03] [1B percent]
DONE     : [0x04]
ERROR    : [0x05] [1B error_code]
```

### Error codes

| Code | Name            | Description                              |
|------|-----------------|------------------------------------------|
| 0x01 | CRC_MISMATCH    | Per-chunk CRC did not match              |
| 0x02 | SEQ_ERROR       | Unexpected sequence number               |
| 0x03 | WRITE_FAILED    | Flash write error                        |
| 0x04 | FINAL_CRC_FAIL  | Whole-image CRC mismatch after all chunks|
| 0x05 | SIZE_MISMATCH   | Bytes written ≠ declared total_size      |
| 0x06 | BEGIN_FAILED    | OTA backend failed to initialise         |
| 0x07 | TIMEOUT         | No data received within chunk_timeout    |
| 0x08 | BAD_PASSWORD    | Password did not match                   |
| 0x0F | INTERNAL        | Unexpected internal error                |

## Default UUIDs

```
Service  : 424F5441-0001-1000-8000-00805F9B34FB
Control  : 424F5441-0002-1000-8000-00805F9B34FB
Data     : 424F5441-0002-1000-8000-00805F9B34FB
Status   : 424F5441-0002-1000-8000-00805F9B34FB
```


## Development Notes

### Performance
 
BLE throughput is limited by connection interval and round-trip latency. The uploader uses a sliding window to keep multiple chunks in flight simultaneously rather than waiting for each ACK before sending the next.
 
Typical transfer times for ~600 KB firmware:
 
| Window size | Approximate time |
|-------------|-----------------|
| 1 (stop-and-wait) | 3–5 min |
| 4 | ~90 s |
| 8 (default) | ~45 s |
| 16 | ~25 s |
 
Window sizes above 8 may overwhelm slower BLE stacks. If you see frequent NACKs, reduce the window size.

### Troubleshooting
 
**`esp_ble_gatts_send_indicate failed 259`** — Occasional error on session start; harmless. The ACK is deferred to the next loop iteration and the transfer proceeds normally.
 
**`esp32_ble took a long time for an operation`** — Expected during BLE stack initialisation. The 30 ms threshold in ESPHome's component watchdog is aggressive for BLE; the warning can be ignored if it only appears at boot.
 
**Garbage characters on serial at power-on** — The ESP ROM bootloader outputs at 74880 baud before the firmware takes over at 115200 baud. Only occurs on a full power cycle, not a warm reset. Benign.
 
**Transfer times out mid-way** — Try reducing `--window` or increasing `chunk_timeout` in your device config.