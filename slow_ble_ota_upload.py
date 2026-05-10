## EXAMPLE CODE -- THIS IS NOT UPDATED TO RUN WITH THE PASSWORD CHANGES


#!/usr/bin/env python3
"""
esphome_ble_ota_upload.py
==========================
Host-side uploader for the ESPHome BLE OTA component.

Sends a compiled ESPHome firmware binary (.bin) to an ESP32 device
running the ota_ble component via Bluetooth Low Energy.

Usage:
    python3 esphome_ble_ota_upload.py --device "My ESP32" firmware.bin
    python3 esphome_ble_ota_upload.py --address AA:BB:CC:DD:EE:FF firmware.bin

Requirements:
    pip install bleak

Protocol summary
----------------
1.  Scan / connect to the device by name or MAC address.
2.  Subscribe to STATUS notifications (0x2B12).
3.  Write OTA_BEGIN to CONTROL (0x2B10):
        [0x01] [total_size: 4B LE] [total_crc32: 4B LE]
4.  Wait for STATUS ACK (0xFF 0xFF sentinel).
5.  For each chunk:
        a. Build packet: [seq: 2B LE] [len: 2B LE] [crc32: 4B LE] [payload]
        b. Write to DATA (0x2B11).
        c. Wait for STATUS ACK (seq must match).
        d. On NACK: retry up to MAX_RETRIES times, then abort.
6.  Write OTA_COMMIT to CONTROL (0x03).
7.  Wait for STATUS DONE (0x04).

Exit codes:
    0 — success
    1 — connection / argument error
    2 — transfer error (NACK, timeout, CRC fail)
"""

import argparse
import asyncio
import logging
import struct
import sys
import zlib
from pathlib import Path
from typing import Optional

try:
    from bleak import BleakClient, BleakScanner
    from bleak.backends.characteristic import BleakGATTCharacteristic
except ImportError:
    print("ERROR: 'bleak' is not installed.  Run:  pip install bleak", file=sys.stderr)
    sys.exit(1)

# ── UUIDs (must match ota_ble.h) ─────────────────────────────────────────────
SERVICE_UUID  = "0000181c-0000-1000-8000-00805f9b34fb"
CONTROL_UUID  = "00002b10-0000-1000-8000-00805f9b34fb"
DATA_UUID     = "00002b11-0000-1000-8000-00805f9b34fb"
STATUS_UUID   = "00002b12-0000-1000-8000-00805f9b34fb"

# ── Protocol constants ────────────────────────────────────────────────────────
CTRL_BEGIN   = 0x01
CTRL_ABORT   = 0x02
CTRL_COMMIT  = 0x03

STATUS_ACK      = 0x01
STATUS_NACK     = 0x02
STATUS_PROGRESS = 0x03
STATUS_DONE     = 0x04
STATUS_ERROR    = 0x05

NACK_CODES = {
    0x01: "CRC_MISMATCH (chunk CRC did not match)",
    0x02: "SEQ_ERROR (unexpected sequence number)",
    0x03: "WRITE_FAILED (flash write error)",
    0x04: "FINAL_CRC_FAIL (whole-image CRC mismatch)",
    0x05: "SIZE_MISMATCH (bytes written ≠ declared size)",
    0x06: "BEGIN_FAILED (OTABackend::begin() failed)",
    0x07: "TIMEOUT (device-side timeout)",
    0x0F: "INTERNAL",
}

CHUNK_SIZE   = 500   # bytes — must not exceed OTA_BLE_MAX_CHUNK_SIZE in firmware
MAX_RETRIES  = 3
TIMEOUT_SEC  = 45.0  # per-chunk ACK wait timeout

log = logging.getLogger("ble_ota")


# ── CRC-32 helper (same polynomial as firmware) ────────────────────────────

def crc32(data: bytes) -> int:
    """Return ISO 3309 CRC-32 (same as zlib.crc32 with mask applied)."""
    return zlib.crc32(data) & 0xFFFFFFFF


# ── Uploader ──────────────────────────────────────────────────────────────────

class BLEOTAUploader:
    def __init__(self, firmware: bytes, chunk_size: int = CHUNK_SIZE,
                 max_retries: int = MAX_RETRIES, timeout: float = TIMEOUT_SEC, password: str = ""):
        self.firmware    = firmware
        self.chunk_size  = chunk_size
        self.max_retries = max_retries
        self.timeout     = timeout
        self.password    = password

        # asyncio synchronisation
        self._status_event: asyncio.Event = asyncio.Event()
        self._last_status: Optional[bytes] = None

    # ── BLE notification callback ─────────────────────────────────────────

    def _on_notify(self, _char: BleakGATTCharacteristic, data: bytearray) -> None:
        self._last_status = bytes(data)
        log.debug("STATUS  ← %s", data.hex())
        self._status_event.set()

    async def _wait_status(self) -> bytes:
        """Wait for the next STATUS notification and return it."""
        self._status_event.clear()
        await asyncio.wait_for(self._status_event.wait(), timeout=self.timeout)
        return self._last_status  # type: ignore[return-value]

    # ── Upload state machine ──────────────────────────────────────────────

    async def upload(self, client: BleakClient) -> None:
        firmware       = self.firmware
        total_size     = len(firmware)
        total_crc      = crc32(firmware)
        total_chunks   = (total_size + self.chunk_size - 1) // self.chunk_size

        log.info("Firmware: %d bytes, CRC-32=0x%08X, %d chunks of ≤%d bytes",
                 total_size, total_crc, total_chunks, self.chunk_size)

        # ── Subscribe to STATUS notifications ─────────────────────────────
        await client.start_notify(STATUS_UUID, self._on_notify)
        log.debug("Subscribed to STATUS notifications")
        await asyncio.sleep(1.0)

        # ── Send OTA_BEGIN ────────────────────────────────────────────────
        begin_pkt = struct.pack("<BII", CTRL_BEGIN, total_size, total_crc)
        log.info("→ OTA_BEGIN  size=%d crc=0x%08X", total_size, total_crc)
        await client.write_gatt_char(CONTROL_UUID, begin_pkt, response=True)

        # Wait for begin ACK (sentinel seq=0xFFFF)
        status = await self._wait_status()
        if not status or status[0] != STATUS_ACK:
            code = status[1] if status and len(status) > 1 else 0
            raise RuntimeError(f"OTA_BEGIN rejected: {status.hex() if status else 'no response'} "
                               f"(error code 0x{code:02X})")
        log.info("← ACK (begin confirmed)")

        # ── Stream chunks ─────────────────────────────────────────────────
        for seq in range(total_chunks):
            offset     = seq * self.chunk_size
            payload    = firmware[offset: offset + self.chunk_size]
            chunk_len  = len(payload)
            chunk_crc  = crc32(payload)

            # header: seq(2) + len(2) + crc(4) = 8 bytes
            pkt = struct.pack("<HHI", seq, chunk_len, chunk_crc) + payload

            for attempt in range(1, self.max_retries + 1):
                log.debug("→ DATA  seq=%d  len=%d  crc=0x%08X  (attempt %d)",
                          seq, chunk_len, chunk_crc, attempt)
                await client.write_gatt_char(DATA_UUID, pkt, response=False)

                status = await self._wait_status()
                if not status:
                    raise RuntimeError(f"No status response for chunk seq={seq}")

                kind = status[0]

                if kind == STATUS_PROGRESS:
                    # Device sent an unsolicited progress notification; wait again
                    pct = status[1] if len(status) > 1 else 0
                    log.info("  [device progress: %d%%]", pct)
                    status = await self._wait_status()
                    kind = status[0] if status else 0

                if kind == STATUS_ACK:
                    ack_seq = struct.unpack_from("<H", status, 1)[0]
                    if ack_seq != seq:
                        raise RuntimeError(
                            f"ACK seq mismatch: expected {seq} got {ack_seq}")
                    # Show progress bar
                    pct = int((seq + 1) * 100 / total_chunks)
                    bar = "█" * (pct // 5) + "░" * (20 - pct // 5)
                    print(f"\r  [{bar}] {pct:3d}%  ({seq+1}/{total_chunks})",
                          end="", flush=True)
                    break  # next chunk

                elif kind == STATUS_NACK:
                    nack_seq  = struct.unpack_from("<H", status, 1)[0]
                    nack_code = status[3] if len(status) > 3 else 0xFF
                    desc      = NACK_CODES.get(nack_code, f"unknown 0x{nack_code:02X}")
                    log.warning("← NACK seq=%d code=0x%02X (%s) attempt %d/%d",
                                nack_seq, nack_code, desc, attempt, self.max_retries)
                    if attempt == self.max_retries:
                        raise RuntimeError(
                            f"Chunk seq={seq} permanently rejected: {desc}")
                    await asyncio.sleep(0.2)  # brief back-off before retry

                elif kind == STATUS_ERROR:
                    code = status[1] if len(status) > 1 else 0xFF
                    desc = NACK_CODES.get(code, f"0x{code:02X}")
                    raise RuntimeError(f"Device sent ERROR during transfer: {desc}")

                else:
                    raise RuntimeError(
                        f"Unexpected STATUS kind 0x{kind:02X}: {status.hex()}")
            else:
                raise RuntimeError(f"Chunk seq={seq} failed after {self.max_retries} attempts")

        print()  # newline after progress bar

        # ── Send OTA_COMMIT ───────────────────────────────────────────────
        log.info("→ OTA_COMMIT")
        await client.write_gatt_char(CONTROL_UUID, bytes([CTRL_COMMIT]), response=True)

        # Wait for DONE (0x04)
        status = await self._wait_status()
        # Device may send a final PROGRESS(100) before DONE; absorb it
        if status and status[0] == STATUS_PROGRESS:
            status = await self._wait_status()

        if not status or status[0] != STATUS_DONE:
            code = status[1] if status and len(status) > 1 else 0
            raise RuntimeError(
                f"Expected DONE after COMMIT; got: {status.hex() if status else 'nothing'}")

        log.info("← DONE — device is rebooting into new firmware")

    async def abort(self, client: BleakClient) -> None:
        """Send OTA_ABORT to cleanly cancel an in-progress session."""
        try:
            await client.write_gatt_char(CONTROL_UUID, bytes([CTRL_ABORT]), response=True)
        except Exception:
            pass


# ── Discovery ─────────────────────────────────────────────────────────────────

async def find_device(name: Optional[str] = None,
                      address: Optional[str] = None,
                      scan_timeout: float = 10.0):
    """Return a BleakClient for the first matching device."""
    if address:
        log.info("Connecting by address: %s", address)
        return BleakClient(address)

    log.info("Scanning for device named '%s' (up to %.0fs)…", name, scan_timeout)
    device = await BleakScanner.find_device_by_name(name, timeout=scan_timeout)
    if device is None:
        raise RuntimeError(
            f"Device '{name}' not found. Ensure BLE is enabled and the device is advertising.")
    log.info("Found: %s (%s)", device.name, device.address)
    return BleakClient(device)


# ── CLI entry point ───────────────────────────────────────────────────────────

async def async_main(args: argparse.Namespace) -> int:
    firmware_path = Path(args.firmware)
    if not firmware_path.exists():
        log.error("Firmware file not found: %s", firmware_path)
        return 1

    firmware = firmware_path.read_bytes()
    log.info("Loaded firmware: %s (%d bytes)", firmware_path, len(firmware))

    uploader = BLEOTAUploader(
        firmware=firmware,
        chunk_size=args.chunk_size,
        max_retries=args.retries,
        timeout=args.timeout,
    )

    try:
        client = await find_device(name=args.device, address=args.address,
                                   scan_timeout=args.scan_timeout)
    except RuntimeError as exc:
        log.error("%s", exc)
        return 1

    async with client:
        log.info("Connected to %s", client.address)

        # Verify the OTA service is present
        services = client.services
        for svc_item in services:
            log.info("Service: %s", svc_item.uuid)
            for char in svc_item.characteristics:
                log.info("  Characteristic: %s [%s]", char.uuid, ",".join(char.properties))
        svc = services.get_service(SERVICE_UUID)
        if svc is None:
            log.error("BLE OTA service (%s) not found on device — "
                      "ensure ota_ble is configured in ESPHome", SERVICE_UUID)
            return 1

        try:
            await uploader.upload(client)
        except asyncio.TimeoutError:
            log.error("Timeout waiting for device response")
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

    print("✔  OTA upload complete — device is rebooting.")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Upload ESPHome firmware to an ESP32 device via BLE OTA.")

    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--device", "-n", metavar="NAME",
                        help="Advertised BLE device name to scan for")
    target.add_argument("--address", "-a", metavar="ADDR",
                        help="BLE MAC address (AA:BB:CC:DD:EE:FF)")

    parser.add_argument("firmware", metavar="FIRMWARE.bin",
                        help="Path to the compiled firmware binary")
    parser.add_argument("--chunk-size", "-s", type=int, default=CHUNK_SIZE,
                        help=f"Payload bytes per chunk (default: {CHUNK_SIZE})")
    parser.add_argument("--retries", "-r", type=int, default=MAX_RETRIES,
                        help=f"Per-chunk retry count on NACK (default: {MAX_RETRIES})")
    parser.add_argument("--timeout", "-t", type=float, default=TIMEOUT_SEC,
                        help=f"Per-chunk ACK timeout in seconds (default: {TIMEOUT_SEC})")
    parser.add_argument("--scan-timeout", type=float, default=10.0,
                        help="BLE scan duration in seconds (default: 10)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable debug logging")

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)-8s %(message)s",
    )

    sys.exit(asyncio.run(async_main(args)))


if __name__ == "__main__":
    main()