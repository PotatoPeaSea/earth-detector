"""Earth detector serial monitor: decodes the ESP32's COBS frames and prints a live summary.

Examples:
  python monitor.py                      # COM10, print 1 s summaries forever
  python monitor.py --seconds 20         # stop after 20 s
  python monitor.py --reset              # pulse EN first to capture boot logs
  python monitor.py --csv run1.csv       # record decimated samples (both sensors)
  python monitor.py --set 0=0.05         # set param 0 (on-threshold, g)
  python monitor.py --rezero --test-alarm
"""
import argparse
import csv
import struct
import sys
import time

import serial

STATES = {0: "INIT", 1: "IDLE", 2: "SHAKING?", 3: "ALARM", 4: "FAULT"}
CHECKS = ["duration", "steady", "freq", "crest", "periodic", "base"]
PARAMS = ["on_thr", "off_thr", "confirm_s", "fmin", "fmax", "crest_max", "periodic_min",
          "decay_min", "base_ratio_min", "latch_s", "quiet_s", "buzzer_en", "lvl1", "lvl2",
          "lvl3", "lvl4", "lvl5", "buzzer_mode"]
OUT_SCALE = 2000.0


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_decode(data: bytes):
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            return None
        out += data[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx, code = 0, 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx, code = len(out), 1
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx, code = len(out), 1
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def make_cmd(ctype: int, payload: bytes = b"") -> bytes:
    body = bytes([ctype, 0, 0]) + payload
    body += struct.pack("<H", crc16(body))
    return cobs_encode(body) + b"\x00"


def checks_str(pass_mask, eval_mask):
    parts = []
    for i, name in enumerate(CHECKS):
        if eval_mask & (1 << i):
            parts.append(("+" if pass_mask & (1 << i) else "-") + name)
    return " ".join(parts)


class Stats:
    def __init__(self):
        self.reset()
        self.last_seq = None
        self.lost = 0
        self.bad_crc = 0
        self.health = None
        self.status = None

    def reset(self):
        self.samples = [0, 0]
        self.frames = 0
        self.gaps = [0, 0]


def parse_health(p):
    uptime, dropped, heap, vbat = struct.unpack_from("<IIIH", p, 0)
    off = 14
    sensors = []
    for _ in range(2):
        (present, dev_first, dev_last, st, d0, d1, d2, odr, raw, ovr, i2c, reinit, noise, tilt,
         zeroed) = struct.unpack_from("<BBBBhhhfIIIIffB", p, off)
        off += struct.calcsize("<BBBBhhhfIIIIffB")
        sensors.append(dict(present=present, dev_first=dev_first, dev_last=dev_last, selftest=st,
                            st_delta=(d0, d1, d2), odr=odr, raw=raw, overruns=ovr, i2c=i2c,
                            reinits=reinit, noise_mg=noise, tilt=tilt, zeroed=zeroed))
    return dict(uptime=uptime, dropped=dropped, heap=heap, vbat=vbat, sensors=sensors)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--reset", action="store_true")
    ap.add_argument("--csv")
    ap.add_argument("--set", action="append", default=[], help="id=value")
    ap.add_argument("--rezero", action="store_true")
    ap.add_argument("--raw", choices=["on", "off"])
    ap.add_argument("--test-alarm", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="only logs, events and the final summary")
    args = ap.parse_args()

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = args.port, args.baud, 0.05
    ser.dtr = False
    ser.rts = False
    ser.open()
    if args.reset:
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        time.sleep(0.05)

    for s in args.set:
        pid, val = s.split("=")
        ser.write(make_cmd(0x80, struct.pack("<Bf", int(pid), float(val))))
    if args.rezero:
        ser.write(make_cmd(0x81, b"\x03"))
    if args.raw:
        ser.write(make_cmd(0x82, b"\x01" if args.raw == "on" else b"\x00"))
    if args.test_alarm:
        ser.write(make_cmd(0x83))
    ser.write(make_cmd(0x86))  # ask for HELLO + PARAMS

    writer = None
    if args.csv:
        fh = open(args.csv, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(["sensor", "index", "t_us", "fs", "ax", "ay", "az", "fx", "fy", "fz", "gap"])

    st = Stats()
    buf = bytearray()
    t0 = time.time()
    last_print = t0
    total = Stats()
    try:
        while True:
            chunk = ser.read(8192)
            if chunk:
                buf += chunk
            while True:
                z = buf.find(b"\x00")
                if z < 0:
                    if len(buf) > 4096:
                        buf.clear()
                    break
                pkt = bytes(buf[:z])
                del buf[:z + 1]
                if not pkt:
                    continue
                f = cobs_decode(pkt)
                if not f or len(f) < 5:
                    st.bad_crc += 1
                    continue
                if crc16(f[:-2]) != struct.unpack("<H", f[-2:])[0]:
                    st.bad_crc += 1
                    continue
                ftype = f[0]
                seq = f[1] | (f[2] << 8)
                if st.last_seq is not None:
                    gap = (seq - st.last_seq - 1) & 0xFFFF
                    if gap < 1000:  # larger jumps are a device restart, not loss
                        st.lost += gap
                st.last_seq = seq
                p = f[3:-2]
                st.frames += 1
                if ftype == 0x01:
                    sid, flags, n, idx, tus, fs = struct.unpack_from("<BBBIIf", p, 0)
                    st.samples[sid] += n
                    total.samples[sid] += n
                    if flags & 1:
                        st.gaps[sid] += 1
                    if writer:
                        vals = struct.unpack_from("<%dh" % (6 * n), p, 15)
                        for i in range(n):
                            row = [v / OUT_SCALE for v in vals[6 * i:6 * i + 6]]
                            writer.writerow([sid + 1, idx + i, (tus + int(i * 1e6 / fs)) & 0xFFFFFFFF,
                                             round(fs, 4)] + [round(v, 5) for v in row] +
                                            [int(flags & 1 and i == 0)])
                elif ftype == 0x02:
                    v = struct.unpack_from("<BBBBB10f", p, 0)
                    st.status = v
                elif ftype == 0x03:
                    (eid, verdict, pm, em, lvl, dur, peak, rms, freq, crest, per, decay, bratio, pga,
                     mmi, disp, tstart) = struct.unpack_from("<IBBBB11fI", p, 0)
                    kind = "EARTHQUAKE" if verdict else "rejected"
                    print(f"[EVENT {eid}] {kind}: {dur:.2f} s, peak {peak:.3f} g, rms {rms:.3f} g, "
                          f"{freq:.2f} Hz, crest {crest:.2f}, periodicity {per:.2f}, decay {decay:.2f},"
                          f" base/top {bratio:.2f}, level {lvl}, PGA {pga:.3f} g, MMI {mmi:.1f}, "
                          f"stroke {disp:.2f} mm | {checks_str(pm, em)}")
                elif ftype == 0x04:
                    st.health = parse_health(p)
                elif ftype == 0x05:
                    print("[LOG]", p.decode("ascii", "replace"))
                elif ftype == 0x06:
                    n = p[0]
                    vals = struct.unpack_from("<%df" % n, p, 1)
                    print("[PARAMS]", ", ".join(f"{i}:{PARAMS[i] if i < len(PARAMS) else i}={v:g}"
                                                for i, v in enumerate(vals)))
                elif ftype == 0x07:
                    proto, nsens, odr, decim, scale = struct.unpack_from("<BBfBf", p, 0)
                    print(f"[HELLO] fw {p[11:].decode(errors='replace')}, protocol {proto}, "
                          f"{nsens} sensors, {odr:g} Hz / {decim}")

            now = time.time()
            if now - last_print >= 1.0:
                dt = now - last_print
                if not args.quiet:
                    line = (f"t={now - t0:5.1f}s  S1 {st.samples[0] / dt:6.1f} sps  "
                            f"S2 {st.samples[1] / dt:6.1f} sps  frames {st.frames / dt:5.1f}/s  "
                            f"lost {st.lost}  bad {st.bad_crc}")
                    if st.status:
                        s = st.status
                        line += (f" | {STATES.get(s[0], s[0])} L{s[1]} rms {s[5] * 1000:6.1f} mg "
                                 f"peak {s[6] * 1000:6.1f} mg f {s[8]:5.2f} Hz q {s[9]:.2f} "
                                 f"base {s[12] * 1000:6.1f} mg")
                    print(line)
                    h = st.health
                    if h:
                        parts = []
                        for i, s in enumerate(h["sensors"]):
                            parts.append(
                                f"S{i + 1}: {'ok' if s['present'] else 'MISSING'} odr {s['odr']:.3f} Hz "
                                f"ovr {s['overruns']} i2c {s['i2c']} re {s['reinits']} "
                                f"noise {s['noise_mg']:.2f} mg tilt {s['tilt']:.1f} "
                                f"st {['?', 'pass', 'FAIL'][s['selftest']]} {s['st_delta']} "
                                f"devid {s['dev_first']:#04x}/{s['dev_last']:#04x}")
                        print("        " + " | ".join(parts) + f" | txdrop {h['dropped']} heap {h['heap']}")
                st.reset()
                last_print = now
            if args.seconds and now - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        el = time.time() - t0
        print(f"SUMMARY {el:.1f} s: S1 {total.samples[0]} samples ({total.samples[0] / el:.1f}/s), "
              f"S2 {total.samples[1]} samples ({total.samples[1] / el:.1f}/s), "
              f"lost frames {st.lost}, bad frames {st.bad_crc}")
        if st.health:
            for i, s in enumerate(st.health["sensors"]):
                print(f"  S{i + 1}: odr {s['odr']:.3f} Hz, overruns {s['overruns']}, i2c errors {s['i2c']}, "
                      f"reinits {s['reinits']}, noise {s['noise_mg']:.2f} mg, tilt {s['tilt']:.1f} deg")
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
