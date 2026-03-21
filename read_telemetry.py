#!/usr/bin/env python3
"""
Read SAMD21 telemetry frames from a serial port and optionally live-plot speed + phase currents.

Frame format:
  sync0  sync1  len  type  payload...  crcLo crcHi
  0xA5   0x5A   LEN  0x01  PAYLOAD     CRC16-CCITT over [type+payload]

LEN = 1 + payload_size  (type + payload)

This script auto-detects payload size based on LEN and supports:
- payload 44 bytes (dirSign/offsets/iq_ref/angle/errors)
- payload 88 bytes (older, includes flags)
- payload 92 bytes (newer, includes vd_i_mV/vq_i_mV, no flags)
- payload 116 bytes (adds svpwm, targets, encoder status; no id/iq errors)
"""

import argparse
import struct
import time
from collections import deque
from typing import Optional, Dict, Any

import serial
import os


SYNC = b"\xA5\x5A"
TYPE_TELEMETRY = 0x01


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# Payload parsers keyed by payload size in bytes
PARSERS: Dict[int, Dict[str, Any]] = {
    # 44 bytes: <i I I i i I I i I I I
    44: {
        "name": "v44(dirSign/offsets/iq_ref/angle/errors)",
        "struct": struct.Struct("<iIIiiIIiIII"),
        "header": [
            "dirSign", "ZeroOffsetElectricalAngle", "ThetaEl",
            "ia_mA", "ib_mA",
            "adcOffsetU", "adcOffsetV",
            "iq_ref_mA",
            "angle_raw",
            "runtime_mem_corruption_err", "encoder_err",
        ],
        "idx": {
            "dirSign": 0,
            "theta_el": 2,
            "ia": 3, "ib": 4,
            "iq_ref": 7,
            "angle_raw": 8,
        },
    },
    # 88 bytes: <II 13i II 4i I
    88: {
        "name": "v88(with flags)",
        "struct": struct.Struct("<II13iII4iI"),
        "header": [
            "seq", "t_us",
            "ia_mA", "ib_mA", "ic_mA",
            "vd_mV", "vq_mV",
            "id_mA", "iq_mA", "id_ref_mA", "iq_ref_mA", "id_err_mA", "iq_err_mA",
            "angle_raw", "omega_mrad_s",
            "adc_seq", "missed_pairs",
            "adc_u_raw", "adc_v_raw", "adc_u_off", "adc_v_off",
            "flags",
        ],
        "idx": {
            "seq": 0,
            "omega": 14,
            "ia": 2, "ib": 3, "ic": 4,
            "angle_raw": 13,
            "id": 7, "iq": 8,
        },
    },
    # 92 bytes: <II 15i II 4i   (adds vd_i_mV, vq_i_mV; no flags)
    92: {
        "name": "v92(vd_i/vq_i, no flags)",
        "struct": struct.Struct("<II15iII4i"),
        "header": [
            "seq", "t_us",
            "ia_mA", "ib_mA", "ic_mA",
            "vd_mV", "vq_mV",
            "id_mA", "iq_mA", "id_ref_mA", "iq_ref_mA", "id_err_mA", "iq_err_mA",
            "vd_i_mV", "vq_i_mV",
            "angle_raw", "omega_mrad_s",
            "adc_seq", "missed_pairs",
            "adc_u_raw", "adc_v_raw", "adc_u_off", "adc_v_off",
        ],
        "idx": {
            "seq": 0,
            "omega": 16,
            "ia": 2, "ib": 3, "ic": 4,
            "angle_raw": 15,
            "id": 7, "iq": 8,
        },
    },
    # 116 bytes: <II 13i II 4i 6i 2I
    116: {
        "name": "v116(svpwm/targets/encoder, no id/iq errors)",
        "struct": struct.Struct("<II13iII4i6i2I"),
        "header": [
            "seq", "t_us",
            "ia_mA", "ib_mA", "ic_mA",
            "vd_mV", "vq_mV",
            "id_mA", "iq_mA", "id_ref_mA", "iq_ref_mA",
            "vd_i_mV", "vq_i_mV",
            "angle_raw", "omega_mrad_s",
            "adc_seq", "missed_pairs",
            "adc_u_raw", "adc_v_raw", "adc_u_off", "adc_v_off",
            "svpwm_perA", "svpwm_perB", "svpwm_perC",
            "omega_target_mrad_s", "theta_target_mrad", "theta_target_unwrapped_mrad",
            "zero_offset_electrical_angle_raw", "encoder_error_code",
        ],
        "idx": {
            "seq": 0,
            "omega": 14,
            "ia": 2, "ib": 3, "ic": 4,
            "angle_raw": 13,
            "id": 7, "iq": 8,
        },
    },
}


class FrameReader:
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.buf = bytearray()

    def _fill(self, min_bytes: int = 1) -> None:
        chunk = self.ser.read(max(min_bytes, 512))
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

            # Length includes (type + payload)
            if msg_type != TYPE_TELEMETRY:
                del self.buf[0:1]
                continue

            frame_sz = 2 + 1 + length + 2  # sync + len + (type+payload) + crc
            if length < 1 or frame_sz < 6 or frame_sz > 300:
                del self.buf[0:1]
                continue

            if len(self.buf) < frame_sz:
                self._fill(frame_sz - len(self.buf))
                continue

            frame = bytes(self.buf[:frame_sz])
            del self.buf[:frame_sz]

            crc_calc = crc16_ccitt(frame[3 : 3 + length])  # [type + payload]
            crc_recv = frame[-2] | (frame[-1] << 8)
            if crc_calc != crc_recv:
                # resync: keep last byte
                self.buf[:] = self.buf[-1:]
                continue

            return frame

        return None


def decode_payload(frame: bytes):
    length = frame[2]
    payload_sz = length - 1
    payload = frame[4 : 4 + payload_sz]

    parser = PARSERS.get(payload_sz)
    if not parser:
        return None, payload_sz

    values = parser["struct"].unpack(payload)
    return (parser, values), payload_sz


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0 or COM5)")
    ap.add_argument("--baud", type=int, default=460800, help="Baud rate (default 460800)")
    ap.add_argument("--timeout", type=float, default=0.05, help="Serial read timeout seconds (default 0.05)")
    ap.add_argument("--csv", default="telemetry_log.csv", help="CSV output file path (default telemetry_log.csv)")
    ap.add_argument("--txt", default="telemetry_log.txt", help="TXT output file path (tab-separated, default telemetry_log.txt)")
    ap.add_argument("--overwrite", action="store_true", help="Overwrite output files instead of appending")
    ap.add_argument("--no-header", action="store_true", help="Don't print CSV header")
    ap.add_argument("--plot", action="store_true", help="Live plot omega + phase currents")
    ap.add_argument("--plot-window", type=float, default=10.0, help="Plot window in seconds (default 10)")
    ap.add_argument("--plot-fps", type=float, default=0.0, help="Plot refresh rate FPS (0 = max rate)")
    ap.add_argument("--debug", action="store_true", help="Print debug info (payload size, seq resets)")
    args = ap.parse_args()

    out_csv = None
    out_txt = None

    suppress_header = False
    if args.csv:
        if os.path.exists(args.csv) and not args.overwrite:
            suppress_header = os.path.getsize(args.csv) > 0
            out_csv = open(args.csv, "a", newline="")
        else:
            out_csv = open(args.csv, "w", newline="")

    if args.txt:
        if os.path.exists(args.txt) and not args.overwrite:
            suppress_header = suppress_header or os.path.getsize(args.txt) > 0
            out_txt = open(args.txt, "a", newline="")
        else:
            out_txt = open(args.txt, "w", newline="")

    try:
        with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
            reader = FrameReader(ser)

            active_parser = None
            prev_seq = None

            # Plot setup
            if args.plot:
                import matplotlib.pyplot as plt

                plt.ion()
                fig, (ax_w, ax_i, ax_dq) = plt.subplots(3, 1, sharex=True)
                fig.suptitle("Telemetry")

                ax_w.set_ylabel("omega (mrad/s)")
                (line_w,) = ax_w.plot([], [])
                txt_w = ax_w.text(0.02, 0.9, "", transform=ax_w.transAxes, va="top")

                ax_i.set_ylabel("phase currents (mA)")
                ax_i.set_xlabel("time (s)")
                (line_ia,) = ax_i.plot([], [], label="ia")
                (line_ib,) = ax_i.plot([], [], label="ib")
                ax_i.legend(loc="upper right")

                ax_dq.set_ylabel("dq currents (mA)")
                (line_id,) = ax_dq.plot([], [], label="id")
                (line_iq,) = ax_dq.plot([], [], label="iq")
                ax_dq.legend(loc="upper right")

                maxlen = max(4000, int(args.plot_window * 2000))
                ts = deque(maxlen=maxlen)
                ws = deque(maxlen=maxlen)
                ia = deque(maxlen=maxlen)
                ib = deque(maxlen=maxlen)
                idq_d = deque(maxlen=maxlen)
                idq_q = deque(maxlen=maxlen)

                t0 = time.monotonic()
                last_plot = 0.0

                while True:
                    frame = reader.read_frame(timeout_s=0.2)
                    now = time.monotonic()
                    t = now - t0

                    if frame is not None:
                        decoded, payload_sz = decode_payload(frame)
                        if decoded is None:
                            if args.debug:
                                print(f"[debug] unsupported payload size: {payload_sz} bytes (len={payload_sz+1})")
                            continue

                        parser, values = decoded

                        if active_parser is None:
                            active_parser = parser
                            if args.debug:
                                print(f"[debug] detected telemetry format: {parser['name']} (payload {payload_sz})")

                            if not args.no_header and not suppress_header:
                                header_csv = ",".join(parser["header"])
                                header_txt = "\t".join(parser["header"])
                                print(header_txt)
                                if out_csv:
                                    out_csv.write(header_csv + "\n")
                                    out_csv.flush()
                                if out_txt:
                                    out_txt.write(header_txt + "\n")
                                    out_txt.flush()

                        idx = active_parser["idx"]
                        has_seq = "seq" in idx
                        has_omega = "omega" in idx
                        has_idiq = "id" in idx and "iq" in idx

                        seq = values[idx["seq"]] if has_seq else None
                        omega = values[idx["omega"]] if has_omega else None
                        ia_mA = values[idx["ia"]]
                        ib_mA = values[idx["ib"]]
                        id_mA = values[idx["id"]] if has_idiq else None
                        iq_mA = values[idx["iq"]] if has_idiq else None

                        if has_seq and prev_seq is not None and seq < prev_seq and args.debug:
                            print(f"[debug] seq reset? {prev_seq} -> {seq}")
                        if has_seq:
                            prev_seq = seq


                        line_csv = ",".join(str(x) for x in values)
                        line_txt = "\t".join(str(x) for x in values)
                        print(line_txt)
                        if out_csv:
                            out_csv.write(line_csv + "\n")
                            out_csv.flush()
                        if out_txt:
                            out_txt.write(line_txt + "\n")
                            out_txt.flush()

                        ts.append(t)
                        if has_omega and omega is not None:
                            ws.append(omega)
                        ia.append(ia_mA)
                        ib.append(ib_mA)
                        if has_idiq and id_mA is not None and iq_mA is not None:
                            idq_d.append(id_mA)
                            idq_q.append(iq_mA)

                    if args.plot_fps <= 0.0:
                        should_plot = True
                    else:
                        should_plot = (now - last_plot) >= (1.0 / max(1e-6, args.plot_fps))

                    if should_plot:
                        last_plot = now
                        if len(ts) > 1:
                            if has_omega and ws:
                                line_w.set_data(ts, ws)
                                ax_w.set_xlim(max(0.0, t - args.plot_window), t)

                                ymin = min(ws); ymax = max(ws)
                                if ymin == ymax:
                                    ymin -= 1; ymax += 1
                                pad = 0.05 * (ymax - ymin)
                                ax_w.set_ylim(ymin - pad, ymax + pad)
                                txt_w.set_text(f"omega = {ws[-1]} mrad/s")
                            else:
                                txt_w.set_text("omega = n/a")

                            line_ia.set_data(ts, ia)
                            line_ib.set_data(ts, ib)
                            imin = min(min(ia), min(ib))
                            imax = max(max(ia), max(ib))
                            if imin == imax:
                                imin -= 1; imax += 1
                            ipad = 0.05 * (imax - imin)
                            ax_i.set_ylim(imin - ipad, imax + ipad)

                            if has_idiq and idq_d and idq_q:
                                line_id.set_data(ts, idq_d)
                                line_iq.set_data(ts, idq_q)
                                ax_dq.set_xlim(max(0.0, t - args.plot_window), t)

                                dmin = min(min(idq_d), min(idq_q))
                                dmax = max(max(idq_d), max(idq_q))
                                if dmin == dmax:
                                    dmin -= 1; dmax += 1
                                dpad = 0.05 * (dmax - dmin)
                                ax_dq.set_ylim(dmin - dpad, dmax + dpad)

                        fig.canvas.draw_idle()
                        plt.pause(0.001)

            # No plot: just print frames
            else:
                while True:
                    frame = reader.read_frame(timeout_s=2.0)
                    if frame is None:
                        continue

                    decoded, payload_sz = decode_payload(frame)
                    if decoded is None:
                        if args.debug:
                            print(f"[debug] unsupported payload size: {payload_sz} bytes (len={payload_sz+1})")
                        continue

                    parser, values = decoded
                    if active_parser is None:
                        active_parser = parser
                        if args.debug:
                            print(f"[debug] detected telemetry format: {parser['name']} (payload {payload_sz})")

                        if not args.no_header and not suppress_header:
                            header_csv = ",".join(parser["header"])
                            header_txt = "\t".join(parser["header"])
                            print(header_txt)
                            if out_csv:
                                out_csv.write(header_csv + "\n")
                                out_csv.flush()
                            if out_txt:
                                out_txt.write(header_txt + "\n")
                                out_txt.flush()

                    angle_raw = values[active_parser["idx"]["angle_raw"]]
                    angle_deg = (angle_raw * 360.0) / 16384.0
                    line_csv = ",".join(str(x) for x in values)
                    line_txt = "\t".join(str(x) for x in values)
                    print(line_txt)
                    if out_csv:
                        out_csv.write(line_csv + "\n")
                        out_csv.flush()
                    if out_txt:
                        out_txt.write(line_txt + "\n")
                        out_txt.flush()

    except KeyboardInterrupt:
        pass
    finally:
        if out_csv:
            out_csv.close()
        if out_txt:
            out_txt.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
