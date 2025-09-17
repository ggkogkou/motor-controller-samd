import argparse, time, serial
from pathlib import Path
from datetime import datetime

def parse_args():
    docs_dir   = Path.home() / "Documents" / "Motor-Driver" / "SerialLogs"
    parser     = argparse.ArgumentParser(
        description="Log a serial port to a timestamped text file with auto-reconnect.")
    parser.add_argument("-p", "--port", required=True,
                        help="Serial port (e.g. COM16 or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--dir", type=Path, default=docs_dir,
                        help=f"Log directory (default: {docs_dir})")
    parser.add_argument("--retry", type=int, default=5,
                        help="Seconds to wait before reconnect (default: 5)")
    return parser.parse_args()

def log_serial(port: str, baud: int, log_dir: Path, retry: int):
    log_dir.mkdir(parents=True, exist_ok=True)
    safe_name = port.replace("/", "_")
    log_path  = log_dir / f"{safe_name}_log.txt"

    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser, \
                 open(log_path, "a", buffering=1, encoding="utf-8") as f:
                print(f"[{port}] Connected — logging to {log_path}")
                while True:
                    line = ser.readline()
                    if line:
                        text = line.decode(errors="replace").strip()
                        entry = f"{datetime.now():%Y-%m-%d %H:%M:%S}  {text}"
                        print(f"[{port}] {text}")
                        f.write(entry + "\n")
        except serial.SerialException:
            print(f"[{port}] Disconnected. Retrying in {retry}s…")
            time.sleep(retry)

if __name__ == "__main__":
    cfg = parse_args()
    log_serial(cfg.port, cfg.baud, cfg.dir, cfg.retry)

