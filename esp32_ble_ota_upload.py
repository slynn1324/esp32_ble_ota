#!/usr/bin/env python3
"""
ble_ota_upload.py
=================
Host-side uploader for the ESPHome BLE OTA component.

Sends a compiled ESPHome firmware binary to an ESP32 device running the
ota_ble component via Bluetooth Low Energy.

Usage:
    python3 ble_ota_upload.py --device "my-esp32-ble-ota" firmware.bin
    python3 ble_ota_upload.py --address AA:BB:CC:DD:EE:FF firmware.bin

Requirements:
    pip install bleak

Performance notes
-----------------
BLE throughput is limited by connection interval and round-trip latency.
The key optimisation here is a sliding window: up to WINDOW_SIZE chunks
are in-flight simultaneously rather than waiting for an ACK before each
send. The device still ACKs every chunk individually; the window just
allows the pipeline to stay full.

Typical transfer times for ~600 KB firmware:
  Window 1  (stop-and-wait) : 3–5 min
  Window 4                  : ~90 s
  Window 8  (default)       : ~45 s
  Window 16                 : ~25 s   (may overwhelm slower BLE stacks)
"""

import argparse
import asyncio
import logging
import struct
import sys
import time
import zlib
from collections import deque
from pathlib import Path
from typing import Optional

try:
    from bleak import BleakClient, BleakScanner
    from bleak.backends.characteristic import BleakGATTCharacteristic
except ImportError:
    print("ERROR: 'bleak' is not installed.  Run:  pip install bleak", file=sys.stderr)
    sys.exit(1)

# ── UUIDs ─────────────────────────────────────────────────────────────────────
SERVICE_UUID = "424F5441-0001-1000-8000-00805F9B34FB"
CONTROL_UUID = "424F5441-0002-1000-8000-00805F9B34FB"
DATA_UUID    = "424F5442-0003-1000-8000-00805F9B34FB"
STATUS_UUID  = "424F5443-0004-1000-8000-00805F9B34FB"

# ── Protocol constants ────────────────────────────────────────────────────────
CTRL_BEGIN  = 0x01
CTRL_ABORT  = 0x02
CTRL_COMMIT = 0x03

STATUS_ACK      = 0x01
STATUS_NACK     = 0x02
STATUS_PROGRESS = 0x03
STATUS_DONE     = 0x04
STATUS_ERROR    = 0x05

NACK_CODES = {
    0x01: "CRC_MISMATCH",
    0x02: "SEQ_ERROR",
    0x03: "WRITE_FAILED",
    0x04: "FINAL_CRC_FAIL",
    0x05: "SIZE_MISMATCH",
    0x06: "BEGIN_FAILED",
    0x07: "TIMEOUT",
    0x0F: "INTERNAL",
}

CHUNK_SIZE  = 500    # payload bytes per chunk (≤ OTA_BLE_MAX_CHUNK_SIZE)
WINDOW_SIZE = 8      # chunks in flight before waiting for an ACK
MAX_RETRIES = 3      # per-chunk retry count on NACK
TIMEOUT_SEC = 45.0   # seconds to wait for any single ACK

# firmware validation constants
ESP_IMAGE_MAGIC      = 0xE9
ESP_APP_DESC_MAGIC   = 0xABCD5432
APP_DESC_OFFSET      = 32   # esp_image_header_t (8) + esp_image_segment_header_t (8) + load_addr (4) + padding to magic_word alignment = 32
                             # In practice: image_header(8) + segment_header(8) + app_desc starts at offset 32 for IDF5+
MIN_FIRMWARE_SIZE    = 4096  # anything smaller is certainly wrong

log = logging.getLogger("esp32_ble_ota")


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def make_chunk_packet(seq: int, payload: bytes) -> bytes:
    return struct.pack("<HHI", seq, len(payload), crc32(payload)) + payload


class BLEOTAUploader:
    def __init__(self, firmware: bytes, chunk_size: int = CHUNK_SIZE,
                 window_size: int = WINDOW_SIZE, max_retries: int = MAX_RETRIES,
                 timeout: float = TIMEOUT_SEC, password: str = ""):
        self.firmware    = firmware
        self.chunk_size  = chunk_size
        self.window_size = window_size
        self.max_retries = max_retries
        self.timeout     = timeout
        self.password    = password

        self._ack_queue: asyncio.Queue = asyncio.Queue()

    def _on_notify(self, _char: BleakGATTCharacteristic, data: bytearray) -> None:
        log.debug("STATUS ← %s", data.hex())
        self._ack_queue.put_nowait(bytes(data))

    async def _recv(self) -> bytes:
        return await asyncio.wait_for(self._ack_queue.get(), timeout=self.timeout)

    # ── drain any STATUS_PROGRESS notifications, return first non-progress ──
    async def _recv_non_progress(self) -> bytes:
        while True:
            pkt = await self._recv()
            if pkt[0] == STATUS_PROGRESS:
                pct = pkt[1] if len(pkt) > 1 else 0
                log.debug("  [device progress: %d%%]", pct)
                continue
            return pkt

    def parse_err_code(self, code: int): # a single byte
        match code:
            case 0x01:
                return "ERR_CRC_MISMATCH"
            case 0x02:
                return "ERR_SEQ_ERROR"
            case 0x03:
                return "ERR_WRITE_FAILED"
            case 0x04:
                return "ERR_FINAL_CRC_FAIL"
            case 0x05:
                return "ERR_SIZE_MISMATCH"
            case 0x06:
                return "ERR_BEGIN_FAILED"
            case 0x07:
                return "ERR_TIMEOUT"
            case 0x08:
                return "ERR_BAD_PASSWORD"
            case 0x0F:
                return "ERR_INTERNAL"
            case _:
                return "ERR_UNKNOWN"
    
    def validate_firmware(self, data: bytes):
        """
        Raises ValueError with a descriptive message if the binary does not look
        like a valid ESP-IDF OTA application image.
        """
        if len(data) < MIN_FIRMWARE_SIZE:
            raise ValueError(
                f"file too small ({len(data)} bytes) — not a valid firmware image")
        
        # Byte 0: ESP image magic
        if data[0] != ESP_IMAGE_MAGIC:
            raise ValueError(
                f"invalid image magic 0x{data[0]:02X} (expected 0xE9) — "
                f"wrong file? Use firmware.ota.bin, not firmware.bin or firmware.factory.bin")

        # Bytes 32–35: esp_app_desc_t.magic_word
        # Layout: esp_image_header_t (8B) + esp_image_segment_header_t (8B) +
        #         esp_app_desc_t starts here; magic_word is its first field (4B)
        magic_word, = struct.unpack_from("<I", data, 32)
        if magic_word != ESP_APP_DESC_MAGIC:
            raise ValueError(
                f"esp_app_desc magic 0x{magic_word:08X} (expected 0xABCD5AA5) — "
                f"not an IDF application image or file is corrupt")
        
        # Log the app descriptor fields for confirmation — all null-terminated strings
        # Layout after magic_word: secure_version(4), reserv1(8), version(32),
        #                          project_name(32), time(16), date(16), idf_ver(32), ...
        # secure_ver, = struct.unpack_from("<I", data, 36)
        # skip 8 bytes reserv1
        version      = data[32+16 : 32+48].split(b'\x00', 1)[0].decode('ascii', errors='replace')
        project_name = data[32+48 : 32+80].split(b'\x00', 1)[0].decode('ascii', errors='replace')
        # time_str     = data[32+80 : 32+96].split(b'\x00', 1)[0].decode('ascii', errors='replace') # skip date and time fields, they don't necessarily come from the end compile unit and may be misleading
        # date_str     = data[32+96 : 32+112].split(b'\x00', 1)[0].decode('ascii', errors='replace')
        idf_ver      = data[32+112: 32+144].split(b'\x00', 1)[0].decode('ascii', errors='replace')
        log.info("Firmware image validated:")
        log.info("  Project : %s  v%s", project_name, version)
        # log.info("  Compiled: %s %s", date_str, time_str)
        log.info("  IDF     : %s", idf_ver)


    async def upload(self, client: BleakClient) -> None:

        # first, validate the firmware before we even think about uploading
        self.validate_firmware(self.firmware)

        firmware     = self.firmware
        total_size   = len(firmware)
        total_crc    = crc32(firmware)
        total_chunks = (total_size + self.chunk_size - 1) // self.chunk_size
        pw_bytes     = self.password.encode("utf-8") if self.password else b""


        log.info("Firmware : %d bytes  CRC=0x%08X  chunks=%d  window=%d",
                 total_size, total_crc, total_chunks, self.window_size)

        await client.start_notify(STATUS_UUID, self._on_notify)
        # Allow CCCD write to complete before sending OTA_BEGIN
        await asyncio.sleep(2.0) # TODO, see if this can be eliminated with better esp side handling

        # ── OTA_BEGIN ─────────────────────────────────────────────────────
        begin_pkt = struct.pack("<BIIB", CTRL_BEGIN, total_size, total_crc, len(pw_bytes)) + pw_bytes
        log.info("→ OTA_BEGIN")
        await client.write_gatt_char(CONTROL_UUID, begin_pkt, response=True)

        pkt = await self._recv_non_progress()
        if pkt[0] != STATUS_ACK:
            code = pkt[1] if len(pkt) > 1 else 0
            reason = self.parse_err_code(code)
            raise RuntimeError(f"OTA_BEGIN rejected (0x{code:02X}): {pkt.hex()} -- {reason}")
        log.info("← ACK (begin)")

        # ── Sliding-window chunk transfer ─────────────────────────────────
        #
        # sent_up_to  : next seq to send
        # acked_up_to : highest seq confirmed by device + 1
        # in_flight   : deque of (seq, payload, retries_remaining)
        #
        # We fill the window by sending chunks ahead, then process ACKs/NACKs
        # as they arrive.  On NACK we retransmit that chunk immediately and
        # reset the window to stop sending new chunks until it's ACKed.

        sent_up_to  = 0
        acked_up_to = 0
        in_flight: deque = deque()   # (seq, payload)
        retry_count: dict = {}       # seq -> retries used

        start_time = time.monotonic()

        def progress_bar(done: int) -> str:
            pct = int(done * 100 / total_chunks)
            elapsed = time.monotonic() - start_time
            rate    = (done * self.chunk_size) / max(elapsed, 0.001) / 1024
            bar     = "█" * (pct // 5) + "░" * (20 - pct // 5)
            eta     = ((total_chunks - done) / max(done, 1)) * elapsed
            return f"\r  [{bar}] {pct:3d}%  {rate:4.1f} KB/s  ETA {int(eta)}s  "

        while acked_up_to < total_chunks:
            # Fill the window with new chunks
            while sent_up_to < total_chunks and \
                  sent_up_to - acked_up_to < self.window_size:
                offset  = sent_up_to * self.chunk_size
                payload = firmware[offset: offset + self.chunk_size]
                pkt     = make_chunk_packet(sent_up_to, payload)
                log.debug("→ DATA seq=%d", sent_up_to)
                await client.write_gatt_char(DATA_UUID, pkt, response=False)
                in_flight.append((sent_up_to, payload))
                sent_up_to += 1

            # Wait for the oldest in-flight ACK
            resp = await self._recv_non_progress()
            kind = resp[0]

            if kind == STATUS_ACK:
                ack_seq = struct.unpack_from("<H", resp, 1)[0]
                # Slide the window: remove everything up to and including ack_seq
                while in_flight and in_flight[0][0] <= ack_seq:
                    in_flight.popleft()
                acked_up_to = ack_seq + 1
                print(progress_bar(acked_up_to), end="", flush=True)

            elif kind == STATUS_NACK:
                nack_seq  = struct.unpack_from("<H", resp, 1)[0]
                nack_code = resp[3] if len(resp) > 3 else 0xFF
                desc      = NACK_CODES.get(nack_code, f"0x{nack_code:02X}")

                retry_count[nack_seq] = retry_count.get(nack_seq, 0) + 1
                if retry_count[nack_seq] > self.max_retries:
                    raise RuntimeError(
                        f"Chunk seq={nack_seq} permanently rejected: {desc}")

                log.warning("← NACK seq=%d (%s) retry %d/%d",
                            nack_seq, desc, retry_count[nack_seq], self.max_retries)

                # Find and retransmit the NACKed chunk
                for seq, payload in in_flight:
                    if seq == nack_seq:
                        pkt = make_chunk_packet(seq, payload)
                        await client.write_gatt_char(DATA_UUID, pkt, response=False)
                        # Roll back sent_up_to so the window refills from here
                        sent_up_to = nack_seq + 1
                        # Trim in_flight back to just up to the retransmitted chunk
                        while in_flight and in_flight[-1][0] > nack_seq:
                            in_flight.pop()
                        break
                else:
                    raise RuntimeError(
                        f"NACK for seq={nack_seq} not in flight window")

                await asyncio.sleep(0.05)

            elif kind == STATUS_ERROR:
                code = resp[1] if len(resp) > 1 else 0xFF
                raise RuntimeError(
                    f"Device error: {NACK_CODES.get(code, f'0x{code:02X}')}")

            else:
                raise RuntimeError(f"Unexpected STATUS 0x{kind:02X}: {resp.hex()}")

        elapsed = time.monotonic() - start_time
        rate    = total_size / elapsed / 1024
        print(f"\r  [{'█'*20}] 100%  {rate:.1f} KB/s  {elapsed:.0f}s total      ")

        # ── OTA_COMMIT ────────────────────────────────────────────────────
        log.info("→ OTA_COMMIT")
        await client.write_gatt_char(CONTROL_UUID, bytes([CTRL_COMMIT]), response=True)

        resp = await self._recv_non_progress()
        if resp[0] != STATUS_DONE:
            code = resp[1] if len(resp) > 1 else 0
            raise RuntimeError(
                f"Expected DONE after COMMIT; got: {resp.hex()}")
        log.info("← DONE — device rebooting")

    async def abort(self, client: BleakClient) -> None:
        try:
            await client.write_gatt_char(CONTROL_UUID, bytes([CTRL_ABORT]), response=True)
        except Exception:
            pass


async def find_device(name: Optional[str] = None,
                      address: Optional[str] = None,
                      scan_timeout: float = 10.0) -> BleakClient:
    if address:
        log.info("Connecting by address: %s", address)
        return BleakClient(address)
    log.info("Scanning for '%s' (up to %.0fs)…", name, scan_timeout)
    device = await BleakScanner.find_device_by_name(name, timeout=scan_timeout)
    if device is None:
        raise RuntimeError(f"Device '{name}' not found.")
    log.info("Found: %s (%s)", device.name, device.address)
    return BleakClient(device)


async def async_main(args: argparse.Namespace) -> int:
    path = Path(args.firmware)
    if not path.exists():
        log.error("File not found: %s", path)
        return 1

    firmware = path.read_bytes()
    log.info("Loaded: %s (%d bytes)", path, len(firmware))

    uploader = BLEOTAUploader(
        firmware=firmware,
        chunk_size=args.chunk_size,
        window_size=args.window,
        max_retries=args.retries,
        timeout=args.timeout,
        password=args.password or "",
    )

    try:
        client = await find_device(name=args.device, address=args.address,
                                   scan_timeout=args.scan_timeout)
    except RuntimeError as exc:
        log.error("%s", exc)
        return 1

    async with client:
        log.info("Connected to %s", client.address)

        svc = client.services.get_service(SERVICE_UUID)
        if svc is None:
            log.error("BLE OTA service not found — is ota_ble configured?")
            return 1

        try:
            await uploader.upload(client)
        except asyncio.TimeoutError:
            log.error("Timeout waiting for device")
            await uploader.abort(client)
            return 2
        except RuntimeError as exc:
            log.error("Upload failed: %s", exc)
            await uploader.abort(client)
            return 2
        except Exception as exc:
            log.exception("Unexpected error: %s", exc)
            await uploader.abort(client)
            return 2

    print("✔  OTA complete — device is rebooting.")
    return 0


def main() -> None:
    p = argparse.ArgumentParser(
        description="Upload ESPHome firmware over BLE OTA.")

    tgt = p.add_mutually_exclusive_group(required=True)
    tgt.add_argument("--device",  "-n", metavar="NAME")
    tgt.add_argument("--address", "-a", metavar="ADDR")

    p.add_argument("firmware", metavar="FIRMWARE.bin")
    p.add_argument("--password", "-p", metavar="PASSWORD", default=None, help="OTA password (max 64 bytes, must match device config)")
    p.add_argument("--chunk-size", "-s", type=int, default=CHUNK_SIZE,
                   help=f"Payload bytes per chunk (default {CHUNK_SIZE})")
    p.add_argument("--window", "-w", type=int, default=WINDOW_SIZE,
                   help=f"Chunks in-flight before waiting for ACK (default {WINDOW_SIZE})")
    p.add_argument("--retries", "-r", type=int, default=MAX_RETRIES,
                   help=f"Retries on NACK (default {MAX_RETRIES})")
    p.add_argument("--timeout", "-t", type=float, default=TIMEOUT_SEC,
                   help=f"ACK timeout seconds (default {TIMEOUT_SEC})")
    p.add_argument("--scan-timeout", type=float, default=10.0)
    p.add_argument("--verbose", "-v", action="store_true")

    args = p.parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)-8s %(message)s",
    )
    sys.exit(asyncio.run(async_main(args)))


if __name__ == "__main__":
    main()