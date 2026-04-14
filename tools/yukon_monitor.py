#!/usr/bin/env python3
"""
yukon_monitor.py — Yukon RP2040 voltage & telemetry monitor.

Sends CMD_SENSOR over the Yukon USB serial link and displays voltage,
current, temperatures, and fault flags.

Usage:
    python3 tools/yukon_monitor.py                   # single reading
    python3 tools/yukon_monitor.py --watch           # live display
    python3 tools/yukon_monitor.py --watch --interval 0.5
    python3 tools/yukon_monitor.py --port /dev/ttyACM0
"""

import argparse
import sys
import time

# ── Yukon protocol constants ──────────────────────────────────────────────────

SYNC = 0x7E
ACK  = 0x06
NAK  = 0x15

# CMD_SENSOR encoded frame (command code 5, value 0)
#   cmd    = 5 + 0x20 = 0x25
#   v_high = 0 + 0x40 = 0x40
#   v_low  = 0 + 0x50 = 0x50
#   chk    = 0x25 ^ 0x40 ^ 0x50 = 0x35
CMD_SENSOR_FRAME = bytes([SYNC, 0x25, 0x40, 0x50, 0x35])


# ── Serial query ──────────────────────────────────────────────────────────────

def query_sensor(ser) -> dict | None:
    """Send CMD_SENSOR and collect the response packets.

    Returns a dict of {resp_id: value} or None on timeout / NAK.
    """
    ser.reset_input_buffer()
    ser.write(CMD_SENSOR_FRAME)

    raw     = {}
    pkt     = []
    in_pkt  = False
    deadline = time.monotonic() + 0.5   # 500 ms timeout

    while time.monotonic() < deadline:
        data = ser.read(1)
        if not data:
            continue
        b = data[0]

        if b == ACK:
            return raw

        if b == NAK:
            return None

        if b == SYNC:
            pkt    = [b]
            in_pkt = True
            continue

        if in_pkt:
            pkt.append(b)
            if len(pkt) == 5:
                in_pkt = False
                rtype, v_high, v_low, chk = pkt[1], pkt[2], pkt[3], pkt[4]
                if (0x30 <= rtype <= 0x46 and
                        0x40 <= v_high <= 0x4F and
                        0x50 <= v_low  <= 0x5F and
                        chk == (rtype ^ v_high ^ v_low)):
                    resp_id = rtype - 0x30
                    value   = ((v_high - 0x40) << 4) | (v_low - 0x50)
                    raw[resp_id] = value
                pkt = []

    return None  # timed out


# ── Decode telemetry ──────────────────────────────────────────────────────────

def decode(raw: dict) -> dict:
    """Convert raw sensor dict to human-readable values."""
    voltage = raw.get(0, 0) / 10.0      # V (resolution 0.1 V)
    current = raw.get(1, 0) / 100.0     # A (resolution 0.01 A)

    def temp(rid):
        v = raw.get(rid)
        return None if v is None else round(v / 3.0, 1)

    return dict(
        voltage     = voltage,
        current     = current,
        power       = voltage * current,
        board_temp  = temp(2),
        left_temp   = temp(3),
        right_temp  = temp(4),
        left_fault  = bool(raw.get(5, 0)),
        right_fault = bool(raw.get(6, 0)),
    )


# ── Display ───────────────────────────────────────────────────────────────────

LINES = 8  # number of lines printed per reading (for --watch cursor-up)

def _vfmt(v: float) -> str:
    warn = ""
    if v < 6.0:
        warn = "  *** DEAD ***"
    elif v < 9.0:
        warn = "  ⚠ CRITICAL LOW"
    elif v < 10.5:
        warn = "  ⚠ LOW"
    return f"{v:.1f} V{warn}"

def _tfmt(t: float | None) -> str:
    if t is None:
        return "N/A"
    warn = "  ⚠ HOT" if t > 60.0 else ""
    return f"{t:.1f} °C{warn}"

def print_reading(tel: dict, port: str, ts: str):
    faults = []
    if tel['left_fault']:  faults.append("LEFT motor")
    if tel['right_fault']: faults.append("RIGHT motor")
    fault_str = ", ".join(faults) if faults else "none"

    print(f"[{ts}]  {port}")
    print(f"  Voltage : {_vfmt(tel['voltage'])}")
    print(f"  Current : {tel['current']:.2f} A")
    print(f"  Power   : {tel['power']:.1f} W")
    print(f"  Board   : {_tfmt(tel['board_temp'])}")
    print(f"  Motor L : {_tfmt(tel['left_temp'])}")
    print(f"  Motor R : {_tfmt(tel['right_temp'])}")
    print(f"  Faults  : {fault_str}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Yukon RP2040 voltage & telemetry monitor.")
    parser.add_argument('--port',     default='/dev/yukon',
                        help='Serial port (default: /dev/yukon)')
    parser.add_argument('--baud',     type=int, default=115200)
    parser.add_argument('--watch',    action='store_true',
                        help='Continuously update readings')
    parser.add_argument('--interval', type=float, default=1.0, metavar='SECS',
                        help='Refresh interval for --watch (default: 1.0 s)')
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial not installed — run: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    first = True
    try:
        while True:
            ts  = time.strftime('%H:%M:%S')
            raw = query_sensor(ser)

            if not args.watch:
                if raw is None:
                    print(f"No response from Yukon on {args.port}", file=sys.stderr)
                    sys.exit(1)
                print_reading(decode(raw), args.port, ts)
                break

            # --watch: overwrite previous block after first iteration
            if not first:
                sys.stdout.write(f'\033[{LINES}A')
            first = False

            if raw is None:
                print(f"[{ts}]  {args.port}  — no response")
                # blank remaining lines so cursor lands correctly next time
                for _ in range(LINES - 1):
                    print()
            else:
                print_reading(decode(raw), args.port, ts)

            time.sleep(args.interval)

    except KeyboardInterrupt:
        if args.watch:
            print()
        print("Stopped.")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
