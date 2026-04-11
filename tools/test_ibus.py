#!/usr/bin/env python3
"""
test_ibus.py — iBUS channel live display.

Two input modes:

  yukon   Read RC channels by sending CMD_RC_QUERY to the Yukon RP2040 over
          USB serial.  This is the normal operating path; no iBUS UART needed
          on the Pi.
          Default port: /dev/ttyACM0

  gpio    Read iBUS packets directly from a UART wired to the RX receiver.
          Useful if the Yukon is unavailable or bypassed.
          Default port: /dev/ttyAMA3

Usage:
    python3 tools/test_ibus.py                    # yukon mode, /dev/ttyACM0
    python3 tools/test_ibus.py --yukon            # same, explicit
    python3 tools/test_ibus.py --yukon /dev/ttyACM0
    python3 tools/test_ibus.py --gpio             # GPIO/UART mode, /dev/ttyAMA3
    python3 tools/test_ibus.py --gpio /dev/ttyAMA3
    python3 tools/test_ibus.py --unit-only        # run unit tests and exit

Press Ctrl+C to quit.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
import struct
import queue
import threading

from robot.rc_channels import CHANNEL_NAMES  # noqa: E402

BAR_WIDTH = 28

# ── Offline unit tests (no hardware needed) ───────────────────────────────────

def _make_ibus_packet(channels):
    """Build a valid 32-byte iBUS packet from a list of 14 channel values."""
    body = b'\x20\x40'
    for ch in channels:
        body += struct.pack('<H', ch)
    chk = (0xFFFF - sum(body)) & 0xFFFF
    return body + struct.pack('<H', chk)


def run_unit_tests():
    from drivers.ibus import IBusReader, NUM_CHANNELS, PACKET_LEN

    passed = 0
    failed = 0

    def check(name, result, expected=True):
        nonlocal passed, failed
        ok = result == expected
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        if ok:
            passed += 1
        else:
            failed += 1
            print(f"          got {result!r}, expected {expected!r}")

    print("=== iBUS unit tests ===\n")

    pkt = _make_ibus_packet([1500] * NUM_CHANNELS)
    check("packet is 32 bytes",               len(pkt), PACKET_LEN)
    check("header byte 0 = 0x20",             pkt[0], 0x20)
    check("header byte 1 = 0x40",             pkt[1], 0x40)
    check("checksum valid on neutral packet", IBusReader._verify(pkt))

    bad = bytearray(pkt)
    bad[30] ^= 0xFF
    check("corrupt checksum detected",        IBusReader._verify(bytes(bad)), False)

    channels = [struct.unpack_from('<H', pkt, 2 + i * 2)[0] for i in range(NUM_CHANNELS)]
    check("all channels = 1500 (neutral)",    channels, [1500] * NUM_CHANNELS)

    vals = [1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,1500,1500,1500]
    pkt2 = _make_ibus_packet(vals)
    check("checksum valid on varied packet",  IBusReader._verify(pkt2))
    ch2 = [struct.unpack_from('<H', pkt2, 2 + i * 2)[0] for i in range(NUM_CHANNELS)]
    check("channel values decoded correctly", ch2, vals)

    check("checksum valid on all-min packet", IBusReader._verify(_make_ibus_packet([1000]*NUM_CHANNELS)))
    check("checksum valid on all-max packet", IBusReader._verify(_make_ibus_packet([2000]*NUM_CHANNELS)))

    body = pkt[:30]
    expected_chk = (0xFFFF - sum(body)) & 0xFFFF
    stored_chk   = struct.unpack_from('<H', pkt, 30)[0]
    check("checksum formula correct",         expected_chk, stored_chk)

    print(f"\n{'='*42}")
    print(f"Results: {passed}/{passed+failed} passed", end='')
    print("  (all passed)" if not failed else f"  ({failed} FAILED)")
    print('='*42)
    return failed == 0


# ── Shared display ────────────────────────────────────────────────────────────

def _bar(value, lo=1000, hi=2000):
    frac = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    fill = int(frac * BAR_WIDTH)
    return '█' * fill + '░' * (BAR_WIDTH - fill)


def _render(mode_str, status_str, channels, rc_valid, packets_ok, packets_bad, t_start):
    num_lines = len(CHANNEL_NAMES) + 3  # header + separator + channel lines
    print(f"\033[{num_lines}A", end='')

    print(f"\033[K  {mode_str}  {status_str}")
    print(f"\033[K  {'Name':<14}  {'Value':>5}  {'':^{BAR_WIDTH}}  {'Raw':>5}")
    print(f"\033[K  {'-'*14}  {'-'*5}  {'-'*BAR_WIDTH}  {'-'*5}")
    for name, val in zip(CHANNEL_NAMES, channels):
        print(f"\033[K  {name}  {val:>5}  {_bar(val)}  {val:>5}")


# ── Yukon mode ────────────────────────────────────────────────────────────────

SYNC = 0x7E
ACK  = 0x06
NAK  = 0x15
CMD_RC_QUERY  = 12
RESP_RC_BASE  = 8


def _encode_cmd(cmd_code, byte_value=0):
    cmd    = cmd_code + 0x20
    v_high = (byte_value >> 4)   + 0x40
    v_low  = (byte_value & 0x0F) + 0x50
    chk    = cmd ^ v_high ^ v_low
    return bytes([SYNC, cmd, v_high, v_low, chk])


def _parse_rc(packets):
    """Decode Yukon RC response packets → (channels list, rc_valid bool)."""
    raw = {resp_id: value for resp_id, value in packets}
    channels = [raw.get(RESP_RC_BASE + i, 0) * 5 + 1000 for i in range(14)]
    rc_valid  = bool(raw.get(22, 0))
    return channels, rc_valid


def live_display_yukon(port):
    import serial as _serial

    print(f"\n  Mode: Yukon  ({port})")
    print(f"  Connecting…")
    try:
        ser = _serial.Serial(port, 115200, timeout=0.1, write_timeout=0.5, dsrdtr=False)
    except Exception as e:
        print(f"  Error opening {port}: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    ack_q = queue.Queue()
    rc_q  = queue.Queue()
    stop  = threading.Event()

    def reader():
        pkt = []; in_pkt = False; s_buf = []
        while not stop.is_set():
            try:
                data = ser.read(1)
            except (_serial.SerialException, OSError):
                break
            if not data:
                continue
            b = data[0]
            if b == ACK:
                if s_buf and s_buf[0][0] >= RESP_RC_BASE:
                    rc_q.put(_parse_rc(s_buf))
                s_buf = []; in_pkt = False
                ack_q.put(True)
            elif b == NAK:
                s_buf = []; in_pkt = False; ack_q.put(False)
            elif b == SYNC:
                pkt = [b]; in_pkt = True
            elif in_pkt:
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
                        s_buf.append((resp_id, value))
                    pkt = []

    rx = threading.Thread(target=reader, daemon=True)
    rx.start()

    # Print blank lines to reserve space
    num_lines = len(CHANNEL_NAMES) + 3
    print('\n' * num_lines, end='')

    channels   = [1500] * 14
    rc_valid   = False
    packets_ok = packets_bad = 0
    t_start    = time.time()
    last_ok    = None

    try:
        while True:
            # Send CMD_RC_QUERY
            while True:
                try: ack_q.get_nowait()
                except queue.Empty: break
            ser.write(_encode_cmd(CMD_RC_QUERY, 0))
            # Wait for ACK
            try:
                ack_q.get(timeout=0.1)
            except queue.Empty:
                packets_bad += 1

            # Collect RC response
            try:
                channels, rc_valid = rc_q.get(timeout=0.1)
                packets_ok += 1
                last_ok = time.time()
            except queue.Empty:
                packets_bad += 1

            now = time.time()
            if rc_valid:
                status = f"\033[32mRC OK\033[0m  pkts={packets_ok}  bad={packets_bad}  t={now-t_start:.1f}s"
            elif last_ok:
                status = f"\033[33mRC LOST\033[0m ({now-last_ok:.1f}s ago)  pkts={packets_ok}"
            else:
                status = f"\033[33mWAITING\033[0m  pkts={packets_ok}  bad={packets_bad}"

            _render(f"Yukon  \033[36m{port}\033[0m", status,
                    channels, rc_valid, packets_ok, packets_bad, t_start)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print(f"\n\n  Exiting. OK={packets_ok}  BAD={packets_bad}")
    finally:
        stop.set()
        ser.close()


# ── GPIO/UART mode ────────────────────────────────────────────────────────────

def live_display_gpio(port):
    from drivers.ibus import IBusReader, IBusError

    print(f"\n  Mode: GPIO/UART  ({port})")
    print(f"  Opening at 115200 baud…")
    try:
        reader = IBusReader(port)
    except Exception as e:
        print(f"  Error: {e}")
        sys.exit(1)

    print(f"  Waiting for iBUS signal (Ctrl+C to quit)…")

    num_lines = len(CHANNEL_NAMES) + 3
    print('\n' * num_lines, end='')

    t_start = time.time()
    last_ok = None

    try:
        while True:
            try:
                ch = reader.read()
            except IBusError as e:
                print(f"\r\033[K  Serial error: {e}")
                break

            now = time.time()
            if ch is None:
                age = f" ({now - last_ok:.1f}s ago)" if last_ok else ""
                status = f"\033[33mWAITING\033[0m{age}  bad={reader.packets_bad}"
            else:
                last_ok = now
                status = (f"\033[32mOK\033[0m  pkts={reader.packets_ok}"
                          f"  bad={reader.packets_bad}  t={now-t_start:.1f}s")

            channels = ch if ch else reader.channels
            _render(f"GPIO  \033[36m{port}\033[0m", status,
                    channels, ch is not None, reader.packets_ok, reader.packets_bad, t_start)

    except KeyboardInterrupt:
        print(f"\n\n  Exiting. OK={reader.packets_ok}  BAD={reader.packets_bad}")
    finally:
        reader.close()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    args = sys.argv[1:]

    if '--unit-only' in args or '-u' in args:
        ok = run_unit_tests()
        sys.exit(0 if ok else 1)

    # Always run unit tests first (brief)
    run_unit_tests()
    print()

    # Parse mode and optional port
    mode = 'yukon'
    port = None

    for a in args:
        if a in ('--yukon', '-y'):
            mode = 'yukon'
        elif a in ('--gpio', '-g'):
            mode = 'gpio'
        elif not a.startswith('-'):
            port = a

    if mode == 'yukon':
        live_display_yukon(port or '/dev/ttyACM0')
    else:
        live_display_gpio(port or '/dev/ttyAMA3')


if __name__ == '__main__':
    main()
