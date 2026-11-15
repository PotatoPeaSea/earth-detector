"""Forward the detector's state to the round ESP32-C3 display (display/ firmware).

  python display_bridge.py                       # auto-detect both boards
  python display_bridge.py --detector COM11 --display COM20
  python display_bridge.py --demo --display COM20   # no detector: cycle idle / bump / alarm levels 1-5

Only one program can open the detector's COM port, so close the dashboard (or monitor.py) first.
Lines sent to the display (see display/src/main.cpp):
  S,<state>,<level>,<peak_g>,<rms_g>,<freq_hz>    every detector STATUS frame (10 Hz)
  B,<peak_g>,<duration_s>                         every rejected event (a bump)
"""
import argparse
import struct
import sys
import time

import serial
import serial.tools.list_ports

from monitor import cobs_decode, crc16

VID_RP2040, VID_ESPRESSIF = 0x2E8A, 0x303A


def find_port(vid):
    for p in serial.tools.list_ports.comports():
        if p.vid == vid:
            return p.device
    return None


def open_port(name, baud, **kw):
    s = serial.Serial()
    s.port, s.baudrate = name, baud
    s.dtr = False  # DTR/RTS toggling can reset an ESP32 or drop the RP2040's USB stream
    s.rts = False
    for k, v in kw.items():
        setattr(s, k, v)
    s.open()
    return s


class Display:
    """Writes lines to the C3; reconnects if the board is unplugged or reset."""

    def __init__(self, name):
        self.name, self.ser, self.next_try = name, None, 0.0

    def send(self, line):
        now = time.time()
        if self.ser is None:
            if now < self.next_try:
                return
            self.next_try = now + 1.0
            name = self.name or find_port(VID_ESPRESSIF)
            if not name:
                return
            try:
                self.ser = open_port(name, 115200, timeout=0, write_timeout=0.2)
                print(f"[display] connected on {name}")
            except (serial.SerialException, OSError):
                return
        try:
            self.ser.write((line + "\n").encode())
        except (serial.SerialException, OSError):
            print("[display] link lost, retrying")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def run_demo(disp):
    plan = [("idle", 4), ("bump", 0), ("idle", 6)] + [("alarm", 3, lvl) for lvl in range(1, 6)] + [("idle", 4)]
    print("[demo] idle -> bump -> alarm levels 1..5 -> idle, repeating (Ctrl+C to stop)")
    while True:
        for step in plan:
            kind = step[0]
            end = time.time() + (step[1] if len(step) > 1 else 0)
            if kind == "bump":
                disp.send("B,0.312,0.18")
                continue
            while time.time() < end:
                if kind == "idle":
                    disp.send("S,1,0,0.002,0.001,0.0")
                else:
                    lvl = step[2]
                    disp.send(f"S,3,{lvl},{0.04 * lvl * lvl:.4f},{0.02 * lvl:.4f},3.2")
                time.sleep(0.1)


def run_bridge(det_name, disp):
    ser = open_port(det_name, 921600, timeout=0.05)
    print(f"[detector] reading {det_name}")
    buf = bytearray()
    last_status = 0.0
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
            if not f or len(f) < 5 or crc16(f[:-2]) != struct.unpack("<H", f[-2:])[0]:
                continue
            p = f[3:-2]
            if f[0] == 0x02:  # STATUS
                state, level, flags, pm, em, rms, peak, crest, freq = struct.unpack_from("<BBBBBfffff", p, 0)
                disp.send(f"S,{state},{level},{peak:.4f},{rms:.4f},{freq:.2f}")
                last_status = time.time()
            elif f[0] == 0x03:  # EVENT
                eid, verdict, pm, em, lvl, dur, peak = struct.unpack_from("<IBBBB2f", p, 0)
                if verdict == 0:
                    print(f"[event {eid}] bump: peak {peak:.3f} g, {dur:.2f} s")
                    disp.send(f"B,{peak:.4f},{dur:.2f}")
                else:
                    print(f"[event {eid}] earthquake: peak {peak:.3f} g, level {lvl}, {dur:.1f} s")
        if last_status and time.time() - last_status > 2.0:
            print("[detector] no STATUS frames for 2 s; display will show NO DATA")
            last_status = 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detector", help="detector COM port (default: auto, RP2040 USB id)")
    ap.add_argument("--display", help="display COM port (default: auto, Espressif USB id)")
    ap.add_argument("--demo", action="store_true", help="drive the display without a detector")
    args = ap.parse_args()
    disp = Display(args.display)
    try:
        if args.demo:
            run_demo(disp)
        else:
            det = args.detector or find_port(VID_RP2040) or "COM10"  # COM10 = the ESP32 DevKit setup
            run_bridge(det, disp)
    except KeyboardInterrupt:
        pass
    except serial.SerialException as e:
        sys.exit(f"detector port error: {e}")


if __name__ == "__main__":
    main()
