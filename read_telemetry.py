#!/usr/bin/env python3
"""
Read TelemetryPayload12 frames from a serial port and print them to stdout.

Frame format:
  sync0  sync1  len  type  payload...  crcLo crcHi
  0xA5   0x5A   LEN  0x01  PAYLOAD     CRC16-CCITT over [type+payload]

Payload format (12 bytes):
  <i i I
  ia_mA, ib_mA, angle_raw
"""

import argparse
import struct
import sys
import time
from typing import Optional

import serial


SYNC = b"\xA5\x5A"
TYPE_TELEMETRY = 0x01
PAYLOAD_STRUCT = struct.Struct("<iiI")
PAYLOAD_SIZE = PAYLOAD_STRUCT.size
FRAME_LENGTH = 1 + PAYLOAD_SIZE
FRAME_SIZE = 2 + 1 + FRAME_LENGTH + 2


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class FrameReader:
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.buf = bytearray()

    def _fill(self, min_bytes: int = 1) -> None:
        chunk = self.ser.read(max(min_bytes, 1))
        if chunk:
            self.buf.extend(chunk)

    def read_frame(self, timeout_s: float = 2.0) -> Optional[bytes]:
        deadline = time.time() + timeout_s

        while time.time() < deadline:
            if len(self.buf) < 4:
                self._fill(4 - len(self.buf))
                continue

            idx = self.buf.find(SYNC)
            if idx < 0:
                self.buf[:] = self.buf[-1:]
                self._fill()
                continue

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 4:
                self._fill(4 - len(self.buf))
                continue

            length = self.buf[2]
            msg_type = self.buf[3]

            if msg_type != TYPE_TELEMETRY or length != FRAME_LENGTH:
                del self.buf[0:1]
                continue

            if len(self.buf) < FRAME_SIZE:
                self._fill(FRAME_SIZE - len(self.buf))
                continue

            frame = bytes(self.buf[:FRAME_SIZE])
            del self.buf[:FRAME_SIZE]

            crc_calc = crc16_ccitt(frame[3 : 3 + length])
            crc_recv = frame[-2] | (frame[-1] << 8)
            if crc_calc != crc_recv:
                self.buf[:] = self.buf[-1:]
                continue

            return frame

        return None


def decode_payload(frame: bytes) -> tuple[int, int, int]:
    payload = frame[4 : 4 + PAYLOAD_SIZE]
    return PAYLOAD_STRUCT.unpack(payload)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0 or COM5)")
    ap.add_argument("--baud", type=int, default=460800, help="Baud rate (default 460800)")
    ap.add_argument("--timeout", type=float, default=0.05, help="Serial read timeout seconds (default 0.05)")
    ap.add_argument("--refresh-hz", type=float, default=20.0, help="Console refresh rate (default 20 Hz)")
    ap.add_argument("--debug", action="store_true", help="Print timeout info")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        reader = FrameReader(ser)
        latest_values: Optional[tuple[int, int, int]] = None
        last_render = 0.0
        refresh_period = 0.0 if args.refresh_hz <= 0.0 else 1.0 / args.refresh_hz

        print("ia_mA\tib_mA\tangle_raw", flush=True)

        while True:
            frame = reader.read_frame(timeout_s=2.0)
            if frame is None:
                if args.debug:
                    print("\n[debug] timeout waiting for frame", flush=True)
                continue

            latest_values = decode_payload(frame)

            now = time.monotonic()
            if latest_values is None:
                continue

            if refresh_period > 0.0 and (now - last_render) < refresh_period:
                continue

            ia_mA, ib_mA, angle_raw = latest_values
            sys.stdout.write(f"\r{ia_mA}\t{ib_mA}\t{angle_raw}   ")
            sys.stdout.flush()
            last_render = now


if __name__ == "__main__":
    raise SystemExit(main())
