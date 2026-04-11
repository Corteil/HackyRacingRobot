#!/usr/bin/env python3
"""
ibus_sim.py — RadioMaster TX-16S iBUS receiver simulator

Creates a virtual serial port (PTY) that emits iBUS packets at ~7 ms intervals,
mimicking a FlySky iBUS receiver connected to a RadioMaster TX-16S transmitter.

Channel layout matches robot.ini defaults:
  CH1   Right X      right stick ← →   Arrow left / right
  CH2   Right Y      right stick ↑ ↓   Arrow up / down
  CH3   Left Y       left  stick ↑ ↓   W / S
  CH4   Left X       left  stick ← →   A / D
  CH5   SF   mode    2-pos              1  (toggle MANUAL / AUTO)
  CH6   SE   speed   3-pos              2  (cycle slow / mid / max)
  CH7   SA   type    3-pos              3  (cycle Camera / GPS / Cam+GPS)
  CH8   SB   GPS log 2-pos              4  (toggle off / on)
  CH10  SD   pause   2-pos              6  (toggle running / paused)
  CH12  SH   Bookmark momentary         5  (momentary high, auto-returns after 300 ms)
  CH9, CH11, CH13–14  unused            held at 1500

TX-16S behaviour notes:
  - Left Y (throttle) starts at 1000 (stick bottom) for safety.
  - Space centres right gimbal + rudder and drops throttle to 1000.
  - V toggles signal: when lost, packet emission stops → robot failsafe triggers.
  - Stick increment is 50 µs per keypress (adjustable with --step).

Usage:
    python3 tools/ibus_sim.py
    python3 rc_drive.py --ibus-port <PTY path printed on stderr>
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pty
import tty
import termios
import struct
import threading
import time
import select
import argparse

from robot.rc_channels import (
    CH_STEER, CH_RIGHT_Y, CH_THROTTLE, CH_LEFT_X,
    CH_MODE, CH_SPEED, CH_AUTO_TYPE, CH_GPS_LOG, CH_PAUSE, CH_BOOKMARK,
    MODE_NAMES, SPEED_NAMES, AUTO_TYPE_NAMES, GPS_LOG_NAMES, PAUSE_NAMES,
)

# ── iBUS protocol constants ───────────────────────────────────────────────────

PACKET_LEN   = 32
NUM_CHANNELS = 14
CH_MIN       = 1000
CH_MAX       = 2000
CH_MID       = 1500
PACKET_HZ    = 143        # ~7 ms per packet, same as real FlySky receiver

# ── Shared state ─────────────────────────────────────────────────────────────

_lock  = threading.Lock()
_state = {
    'channels'       : [CH_MID] * NUM_CHANNELS,
    'rc_valid'       : True,     # False = stop emitting (simulate signal loss)
    'packets_sent'   : 0,
    'running'        : True,
    'bookmark_until' : 0.0,      # monotonic time after which CH10 returns to 1000
}

# Safe power-on defaults
_state['channels'][CH_THROTTLE]  = CH_MIN    # throttle at bottom (safety)
_state['channels'][CH_STEER]   = CH_MID
_state['channels'][CH_RIGHT_Y]  = CH_MID
_state['channels'][CH_LEFT_X]    = CH_MID
_state['channels'][CH_MODE]      = 1000      # SF: MANUAL
_state['channels'][CH_SPEED]     = 1500      # SE: mid
_state['channels'][CH_AUTO_TYPE] = 1000      # SA: Camera
_state['channels'][CH_GPS_LOG]   = 1000      # SB: GPS log off
_state['channels'][CH_PAUSE]     = 1000      # SD: motors running
_state['channels'][CH_BOOKMARK]  = 1000      # SH: low

# ── iBUS packet builder ───────────────────────────────────────────────────────

def _build_packet(channels):
    """Return a 32-byte iBUS packet for the given 14-channel list."""
    data = bytearray([0x20, 0x40])
    for ch in channels:
        data += struct.pack('<H', max(CH_MIN, min(CH_MAX, ch)))
    checksum = (0xFFFF - sum(data)) & 0xFFFF
    data += struct.pack('<H', checksum)
    assert len(data) == PACKET_LEN
    return bytes(data)


# ── Sender thread ─────────────────────────────────────────────────────────────

def _ibus_sender(master_fd, packet_hz):
    """Write iBUS packets to the PTY master at packet_hz."""
    interval  = 1.0 / packet_hz
    next_send = time.monotonic()

    while True:
        with _lock:
            if not _state['running']:
                break
            valid    = _state['rc_valid']
            channels = list(_state['channels'])
            # Auto-release bookmark after its hold time expires
            if _state['bookmark_until'] and time.monotonic() >= _state['bookmark_until']:
                _state['channels'][CH_BOOKMARK] = CH_MIN
                _state['bookmark_until'] = 0.0
                channels = list(_state['channels'])

        now = time.monotonic()
        if now >= next_send:
            if valid:
                try:
                    os.write(master_fd, _build_packet(channels))
                    with _lock:
                        _state['packets_sent'] += 1
                except OSError:
                    break
            next_send += interval
            # Prevent unbounded catch-up after a stall
            if time.monotonic() > next_send + interval * 5:
                next_send = time.monotonic()
        else:
            time.sleep(max(0.0, next_send - time.monotonic() - 0.0005))


# ── Display ───────────────────────────────────────────────────────────────────

def _bar(val, width=28):
    """Render a channel value (1000–2000) as a centred bar."""
    half = width // 2
    bar  = [' '] * width
    bar[half] = '|'
    frac = (val - CH_MID) / (CH_MAX - CH_MID)   # −1.0 … +1.0
    pos  = int(round(frac * half))
    if pos > 0:
        for i in range(half + 1, min(half + 1 + pos, width)):
            bar[i] = '#'
    elif pos < 0:
        for i in range(max(0, half + pos), half):
            bar[i] = '#'
    return '[' + ''.join(bar) + f']  {val:4d} µs'


def draw(pty_path):
    with _lock:
        ch    = list(_state['channels'])
        valid = _state['rc_valid']
        pkts  = _state['packets_sent']

    swf = ch[CH_MODE]
    swe = ch[CH_SPEED]
    swa = ch[CH_AUTO_TYPE]
    swb = ch[CH_GPS_LOG]
    swd = ch[CH_PAUSE]
    bkm = ch[CH_BOOKMARK]

    sig_str = 'OK' if valid else '*** LOST (V to restore) ***'

    lines = [
        '\033[2J\033[H',
        '=== TX-16S iBUS Simulator ' + '=' * 35,
        f'  iBUS PTY   : {pty_path}',
        f'  Packets    : {pkts:>7d}   Signal: {sig_str}',
        '',
        '─── Right gimbal ─── Arrow keys ─────────────────────────────────',
        f'  CH1 Right X   {_bar(ch[CH_STEER])}',
        f'  CH2 Right Y   {_bar(ch[CH_RIGHT_Y])}',
        '',
        '─── Left gimbal  ─── W/S=Left Y  A/D=Left X ─────────────────────',
        f'  CH3 Left Y    {_bar(ch[CH_THROTTLE])}',
        f'  CH4 Left X    {_bar(ch[CH_LEFT_X])}',
        '',
        '─── Switches ────────────────────────────────────────────────────',
        f'  [1] CH5  SF  mode    {MODE_NAMES.get(swf, str(swf)):>10s}  {"●" if swf == 2000 else "○"}',
        f'  [2] CH6  SE  speed   {SPEED_NAMES.get(swe, str(swe)):>10s}',
        f'  [3] CH7  SA  type    {AUTO_TYPE_NAMES.get(swa, str(swa)):>10s}',
        f'  [4] CH8  SB  GPS log {GPS_LOG_NAMES.get(swb, str(swb)):>10s}  {"●" if swb == 2000 else "○"}',
        f'  [6] CH10 SD  pause   {PAUSE_NAMES.get(swd, str(swd)):>10s}  {"■" if swd >= CH_MID else "□"}',
        f'  [5] CH12 SH  Bookmark {"TRIGGERED" if bkm >= CH_MID else "ready":>9s}',
        '',
        '─── Keys ────────────────────────────────────────────────────────',
        '  Arrows  right gimbal (Right X / Right Y)',
        '  W / S   Left Y up / down',
        '  A / D   Left X left / right',
        '  1–6     switches / bookmark (see above)',
        '  Space   centre sticks + throttle to 1000',
        '  V       toggle RC signal (simulate loss)',
        '  Q       quit',
        '=' * 63,
    ]
    sys.stdout.write('\r\n'.join(lines) + '\r\n')
    sys.stdout.flush()


# ── Key helpers ───────────────────────────────────────────────────────────────

def _clamp(val, step):
    return max(CH_MIN, min(CH_MAX, val + step))


def _cycle3(val):
    """Cycle a 3-position switch: 1000 → 1500 → 2000 → 1000."""
    if val <= 1000:
        return 1500
    if val <= 1500:
        return 2000
    return 1000


def _handle_key(ch, step):
    """Apply a single decoded character to _state['channels']."""
    with _lock:
        chs = _state['channels']

        # Sticks
        if   ch in ('w', 'W'):
            chs[CH_THROTTLE]  = _clamp(chs[CH_THROTTLE], +step)
        elif ch in ('s', 'S'):
            chs[CH_THROTTLE]  = _clamp(chs[CH_THROTTLE], -step)
        elif ch in ('a', 'A'):
            chs[CH_LEFT_X]    = _clamp(chs[CH_LEFT_X], -step)
        elif ch in ('d', 'D'):
            chs[CH_LEFT_X]    = _clamp(chs[CH_LEFT_X], +step)

        # Switches
        elif ch == '1':
            chs[CH_MODE]      = 2000 if chs[CH_MODE] == 1000 else 1000
        elif ch == '2':
            chs[CH_SPEED]     = _cycle3(chs[CH_SPEED])
        elif ch == '3':
            chs[CH_AUTO_TYPE] = _cycle3(chs[CH_AUTO_TYPE])
        elif ch == '4':
            chs[CH_GPS_LOG]   = 2000 if chs[CH_GPS_LOG] == 1000 else 1000
        elif ch == '5':
            chs[CH_BOOKMARK]  = CH_MAX
            _state['bookmark_until'] = time.monotonic() + 0.30
        elif ch == '6':
            chs[CH_PAUSE] = 2000 if chs[CH_PAUSE] == 1000 else 1000

        # Centre / kill-switch
        elif ch == ' ':
            chs[CH_STEER]   = CH_MID
            chs[CH_RIGHT_Y]  = CH_MID
            chs[CH_LEFT_X]    = CH_MID
            chs[CH_THROTTLE]  = CH_MIN   # throttle to safe bottom

        # Signal toggle
        elif ch in ('v', 'V'):
            _state['rc_valid'] = not _state['rc_valid']


def _handle_arrow(code, step):
    """Apply an arrow key escape code to _state['channels']."""
    with _lock:
        chs = _state['channels']
        if   code == b'A':   # up
            chs[CH_RIGHT_Y] = _clamp(chs[CH_RIGHT_Y], +step)
        elif code == b'B':   # down
            chs[CH_RIGHT_Y] = _clamp(chs[CH_RIGHT_Y], -step)
        elif code == b'C':   # right
            chs[CH_STEER]  = _clamp(chs[CH_STEER],  +step)
        elif code == b'D':   # left
            chs[CH_STEER]  = _clamp(chs[CH_STEER],  -step)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="TX-16S iBUS receiver simulator — creates a PTY and emits iBUS packets"
    )
    parser.add_argument('--hz',   type=float, default=143,
                        help='Packet rate in Hz (default: 143 ≈ 7 ms/packet)')
    parser.add_argument('--step', type=int,   default=50,
                        help='Stick µs change per keypress (default: 50)')
    args = parser.parse_args()

    # Create PTY: write to master_fd → data readable from slave_fd (the PTY device)
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)   # disable tty byte-mangling on the slave side
    pty_path = os.ttyname(slave_fd)

    print(f'iBUS PTY   : {pty_path}', file=sys.stderr)
    print(f'rc_drive   : python3 rc_drive.py --ibus-port {pty_path}', file=sys.stderr)
    print(f'yukon_sim  : python3 tools/yukon_sim.py --ibus-port {pty_path}', file=sys.stderr)
    print('(press Q to quit)', file=sys.stderr)

    t = threading.Thread(target=_ibus_sender, args=(master_fd, args.hz), daemon=True)
    t.start()

    stdin_fd  = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(stdin_fd)

    try:
        tty.setraw(stdin_fd)
        last_draw = 0.0
        esc_buf   = b''

        while True:
            now = time.monotonic()
            if now - last_draw >= 0.1:
                try:
                    draw(pty_path)
                except Exception:
                    pass
                last_draw = now

            r, _, _ = select.select([stdin_fd], [], [], 0.05)
            if not r:
                continue

            esc_buf += os.read(stdin_fd, 16)

            # Process buffered input byte by byte, handling escape sequences
            while esc_buf:
                if esc_buf.startswith(b'\x1b['):
                    if len(esc_buf) < 3:
                        break                         # wait for rest of sequence
                    _handle_arrow(esc_buf[2:3], args.step)
                    esc_buf = esc_buf[3:]
                    continue

                if esc_buf[0:1] == b'\x1b':
                    if len(esc_buf) < 2:
                        break                         # wait — could be start of CSI
                    esc_buf = esc_buf[1:]             # lone ESC, skip
                    continue

                b  = esc_buf[0]
                esc_buf = esc_buf[1:]

                if b in (ord('q'), ord('Q'), 0x03, 0x04):   # Q / Ctrl+C / Ctrl+D
                    with _lock:
                        _state['running'] = False
                    return

                _handle_key(chr(b), args.step)

    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_attrs)
        with _lock:
            _state['running'] = False
        os.close(master_fd)
        os.close(slave_fd)
        sys.stdout.write('\r\n\r\niBUS simulator stopped.\r\n')
        sys.stdout.flush()


if __name__ == '__main__':
    main()
