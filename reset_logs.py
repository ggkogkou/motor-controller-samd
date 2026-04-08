#!/usr/bin/env python3

import argparse
import serial
import time
import sys
import csv


def parse_args():
    parser = argparse.ArgumentParser(description="Serial reset logger")

    parser.add_argument(
        "--port",
        required=True,
        help="Serial port (e.g. /dev/ttyUSB0 or COM3)"
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )

    parser.add_argument(
        "--output",
        default="reset_logs.csv",
        help="Output file (default: reset_logs.csv)"
    )

    return parser.parse_args()


def main():
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    print(f"Listening on {args.port} @ {args.baud} baud")
    print(f"Saving to {args.output}")

    with open(args.output, "a", newline="") as f:
        writer = csv.writer(f)

        try:
            while True:
                line = ser.readline()

                if not line:
                    continue

                try:
                    message = line.decode("utf-8", errors="replace").strip()
                except Exception:
                    message = "<decode_error>"

                timestamp = int(time.time())

                # CSV-safe write
                writer.writerow([timestamp, message])

                # Console output
                print(f"{timestamp} | {message}")

        except KeyboardInterrupt:
            print("\nStopping logger...")
        finally:
            ser.close()


if __name__ == "__main__":
    main()