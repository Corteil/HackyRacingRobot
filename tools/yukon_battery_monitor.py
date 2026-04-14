#!/usr/bin/env python3
"""
yukon_battery_monitor.py — Pygame battery & telemetry monitor for Yukon RP2040.

Polls CMD_SENSOR over USB serial and displays a live battery gauge, current,
power, motor temperatures, and fault indicators.

Usage:
    python3 tools/yukon_battery_monitor.py
    python3 tools/yukon_battery_monitor.py --port /dev/ttyACM0
    python3 tools/yukon_battery_monitor.py --cells 4     # 4S LiPo thresholds
    python3 tools/yukon_battery_monitor.py --interval 0.5
"""

import argparse
import sys
import threading
import time
import math

# ── Yukon protocol (minimal inline copy — no robot_daemon dependency) ─────────

SYNC              = 0x7E
ACK               = 0x06
NAK               = 0x15
CMD_SENSOR_FRAME  = bytes([SYNC, 0x25, 0x40, 0x50, 0x35])   # CMD_SENSOR, value 0


def _query_sensor(ser) -> dict | None:
    ser.reset_input_buffer()
    ser.write(CMD_SENSOR_FRAME)
    raw, pkt, in_pkt = {}, [], False
    deadline = time.monotonic() + 0.5
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
            pkt, in_pkt = [b], True
            continue
        if in_pkt:
            pkt.append(b)
            if len(pkt) == 5:
                in_pkt = False
                rt, vh, vl, chk = pkt[1], pkt[2], pkt[3], pkt[4]
                if (0x30 <= rt <= 0x46 and 0x40 <= vh <= 0x4F and
                        0x50 <= vl <= 0x5F and chk == (rt ^ vh ^ vl)):
                    raw[rt - 0x30] = ((vh - 0x40) << 4) | (vl - 0x50)
                pkt = []
    return None


def _decode(raw: dict) -> dict:
    v = raw.get(0, 0) / 10.0
    i = raw.get(1, 0) / 100.0

    def _t(k):
        r = raw.get(k)
        return None if r is None else round(r / 3.0, 1)

    return dict(
        voltage     = v,
        current     = i,
        power       = v * i,
        board_temp  = _t(2),
        left_temp   = _t(3),
        right_temp  = _t(4),
        left_fault  = bool(raw.get(5, 0)),
        right_fault = bool(raw.get(6, 0)),
    )


# ── Shared state ──────────────────────────────────────────────────────────────

class _State:
    def __init__(self):
        self.lock        = threading.Lock()
        self.tel         = None      # decoded telemetry dict or None
        self.connected   = False
        self.last_update = 0.0
        self.history     = []        # recent voltage readings for sparkline


_state = _State()


def _poll_thread(port: str, baud: int, interval: float):
    """Background thread: poll Yukon, update _state."""
    import serial as _serial

    ser = None
    while True:
        # (re)connect
        try:
            ser = _serial.Serial(port, baud, timeout=0.1)
            with _state.lock:
                _state.connected = True
        except _serial.SerialException:
            with _state.lock:
                _state.connected = False
                _state.tel = None
            time.sleep(2.0)
            continue

        while True:
            try:
                raw = _query_sensor(ser)
            except (_serial.SerialException, OSError):
                break

            with _state.lock:
                if raw is not None:
                    _state.tel         = _decode(raw)
                    _state.last_update = time.monotonic()
                    hist = _state.history
                    hist.append(_state.tel['voltage'])
                    if len(hist) > 120:
                        hist.pop(0)
                else:
                    _state.tel = None

            time.sleep(interval)

        if ser:
            try:
                ser.close()
            except OSError:
                pass
        with _state.lock:
            _state.connected = False
            _state.tel = None
        time.sleep(2.0)


# ── Pygame UI ─────────────────────────────────────────────────────────────────

W, H = 520, 420

# Palette
BG         = (18,  20,  30)
PANEL      = (30,  33,  48)
BORDER     = (55,  60,  85)
TEXT_HI    = (230, 235, 245)
TEXT_DIM   = (130, 135, 160)
GREEN      = (50,  210, 100)
YELLOW     = (255, 185,  50)
RED        = (220,  55,  55)
ORANGE     = (240, 130,  30)
BLUE_DIM   = ( 70, 110, 200)
WHITE      = (255, 255, 255)
BAT_BG     = ( 45,  48,  70)
SPARKLINE  = ( 80, 160, 255)


def _bar_colour(pct: float) -> tuple:
    """Interpolate green→yellow→red based on charge percentage."""
    if pct >= 0.5:
        t = (pct - 0.5) / 0.5          # 1 at full, 0 at 50 %
        r = int(GREEN[0] + (YELLOW[0] - GREEN[0]) * (1 - t))
        g = int(GREEN[1] + (YELLOW[1] - GREEN[1]) * (1 - t))
        b = int(GREEN[2] + (YELLOW[2] - GREEN[2]) * (1 - t))
    else:
        t = pct / 0.5                   # 1 at 50%, 0 at 0%
        r = int(YELLOW[0] + (RED[0] - YELLOW[0]) * (1 - t))
        g = int(YELLOW[1] + (RED[1] - YELLOW[1]) * (1 - t))
        b = int(YELLOW[2] + (RED[2] - YELLOW[2]) * (1 - t))
    return (r, g, b)


def _temp_colour(t: float | None) -> tuple:
    if t is None:
        return TEXT_DIM
    if t > 70:
        return RED
    if t > 55:
        return ORANGE
    if t > 40:
        return YELLOW
    return GREEN


def _draw_battery(surf, rect, pct: float, voltage: float, v_min: float, v_max: float):
    """Draw a horizontal battery icon with fill level and voltage label."""
    import pygame
    x, y, bw, bh = rect
    nub_w, nub_h = 10, bh // 3

    # Outer shell
    pygame.draw.rect(surf, BORDER, (x, y, bw, bh), border_radius=6)
    pygame.draw.rect(surf, PANEL,  (x + 2, y + 2, bw - 4, bh - 4), border_radius=5)

    # Nub on right
    nx = x + bw
    ny = y + (bh - nub_h) // 2
    pygame.draw.rect(surf, BORDER, (nx, ny, nub_w, nub_h), border_radius=3)

    # Fill
    fill_w = max(0, int((bw - 8) * pct))
    colour  = _bar_colour(pct)
    if fill_w > 0:
        pygame.draw.rect(surf, colour, (x + 4, y + 4, fill_w, bh - 8), border_radius=4)

    # Segment lines
    for seg in range(1, 4):
        sx = x + 4 + (bw - 8) * seg // 4
        pygame.draw.line(surf, BG, (sx, y + 4), (sx, y + bh - 5), 1)


def _draw_sparkline(surf, rect, history: list, v_min: float, v_max: float):
    import pygame
    if len(history) < 2:
        return
    x, y, w, h = rect
    span = max(v_max - v_min, 0.1)
    pts = []
    for i, v in enumerate(history):
        px = x + i * w // max(len(history) - 1, 1)
        py = y + h - int((v - v_min) / span * h)
        py = max(y, min(y + h, py))
        pts.append((px, py))
    if len(pts) >= 2:
        pygame.draw.lines(surf, SPARKLINE, False, pts, 2)


def _draw_stat(surf, font_big, font_sm, label: str, value: str, unit: str,
               colour: tuple, cx: int, cy: int):
    """Draw a labelled stat block centred at (cx, cy)."""
    lbl = font_sm.render(label, True, TEXT_DIM)
    surf.blit(lbl, lbl.get_rect(centerx=cx, bottom=cy - 2))
    val = font_big.render(value, True, colour)
    surf.blit(val, val.get_rect(centerx=cx, top=cy))
    un  = font_sm.render(unit, True, TEXT_DIM)
    surf.blit(un, un.get_rect(left=val.get_rect(centerx=cx, top=cy).right + 3,
                               bottom=cy + val.get_height() - 2))


def run_ui(v_min: float, v_max: float):
    import pygame

    pygame.init()
    pygame.display.set_caption("Yukon Battery Monitor")
    surf = pygame.display.set_mode((W, H))
    clock = pygame.time.Clock()

    # Fonts
    font_title  = pygame.font.SysFont("monospace", 18, bold=True)
    font_huge   = pygame.font.SysFont("monospace", 52, bold=True)
    font_big    = pygame.font.SysFont("monospace", 30, bold=True)
    font_med    = pygame.font.SysFont("monospace", 20)
    font_sm     = pygame.font.SysFont("monospace", 15)

    bat_rect = (40, 80, W - 100, 70)    # x, y, w, h
    spark_rect = (40, 160, W - 80, 40)  # sparkline below battery

    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                return
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                pygame.quit()
                return

        surf.fill(BG)

        with _state.lock:
            tel         = _state.tel
            connected   = _state.connected
            last_update = _state.last_update
            history     = list(_state.history)

        now = time.monotonic()
        stale = (now - last_update) > 3.0 if last_update else True

        # ── Title bar ─────────────────────────────────────────────────────────
        title = font_title.render("YUKON BATTERY MONITOR", True, TEXT_DIM)
        surf.blit(title, (20, 16))

        dot_col = GREEN if (connected and not stale) else RED
        pygame.draw.circle(surf, dot_col, (W - 20, 24), 7)

        ts = time.strftime('%H:%M:%S')
        ts_surf = font_sm.render(ts, True, TEXT_DIM)
        surf.blit(ts_surf, ts_surf.get_rect(right=W - 34, centery=24))

        if not connected:
            msg = font_med.render("No connection", True, RED)
            surf.blit(msg, msg.get_rect(centerx=W // 2, centery=H // 2))
            pygame.display.flip()
            clock.tick(10)
            continue

        if tel is None or stale:
            msg = font_med.render("Waiting for data…", True, YELLOW)
            surf.blit(msg, msg.get_rect(centerx=W // 2, centery=H // 2))
            pygame.display.flip()
            clock.tick(10)
            continue

        voltage = tel['voltage']
        pct     = max(0.0, min(1.0, (voltage - v_min) / max(v_max - v_min, 0.1)))

        # ── Battery bar ───────────────────────────────────────────────────────
        _draw_battery(surf, bat_rect, pct, voltage, v_min, v_max)

        # Voltage label (huge, centred over bar)
        v_col   = _bar_colour(pct)
        v_text  = font_huge.render(f"{voltage:.1f}", True, v_col)
        v_unit  = font_med.render(" V", True, TEXT_DIM)
        total_w = v_text.get_width() + v_unit.get_width()
        bx      = bat_rect[0] + bat_rect[2] // 2 - total_w // 2
        by      = bat_rect[1] + bat_rect[3] + 8
        surf.blit(v_text, (bx, by))
        surf.blit(v_unit, (bx + v_text.get_width(), by + v_text.get_height() - v_unit.get_height() - 2))

        pct_txt = font_sm.render(f"{int(pct * 100)}%", True, TEXT_DIM)
        surf.blit(pct_txt, pct_txt.get_rect(right=bat_rect[0] + bat_rect[2],
                                             top=by))

        # ── Sparkline ─────────────────────────────────────────────────────────
        sp_y = by + font_huge.get_height() + 6
        _draw_sparkline(surf, (40, sp_y, W - 80, 36), history, v_min, v_max)

        # Sparkline axis labels
        hi_lbl = font_sm.render(f"{v_max:.1f}", True, TEXT_DIM)
        lo_lbl = font_sm.render(f"{v_min:.1f}", True, TEXT_DIM)
        surf.blit(hi_lbl, (W - 38, sp_y - 2))
        surf.blit(lo_lbl, (W - 38, sp_y + 36 - lo_lbl.get_height() + 2))

        # ── Current / Power row ───────────────────────────────────────────────
        row_y = sp_y + 52
        cx1, cx2 = W // 4, 3 * W // 4

        # Current
        i_col = ORANGE if tel['current'] > 20 else TEXT_HI
        _draw_stat(surf, font_big, font_sm,
                   "CURRENT", f"{tel['current']:.1f}", "A", i_col, cx1, row_y)

        # Power
        p_col = RED if tel['power'] > 200 else (ORANGE if tel['power'] > 100 else TEXT_HI)
        _draw_stat(surf, font_big, font_sm,
                   "POWER", f"{tel['power']:.0f}", "W", p_col, cx2, row_y)

        # Divider
        pygame.draw.line(surf, BORDER,
                         (W // 2, row_y - 10), (W // 2, row_y + font_big.get_height() + 10))

        # ── Temperatures ──────────────────────────────────────────────────────
        temp_y = row_y + font_big.get_height() + 28
        pygame.draw.line(surf, BORDER, (20, temp_y - 10), (W - 20, temp_y - 10))

        def _temp_str(t):
            return "N/A" if t is None else f"{t:.0f}°C"

        for i, (label, val) in enumerate([
            ("BOARD",   tel['board_temp']),
            ("MOTOR L", tel['left_temp']),
            ("MOTOR R", tel['right_temp']),
        ]):
            tx = 90 + i * (W - 100) // 3
            lbl = font_sm.render(label, True, TEXT_DIM)
            surf.blit(lbl, lbl.get_rect(centerx=tx, top=temp_y))
            tcol = _temp_colour(val)
            tv   = font_med.render(_temp_str(val), True, tcol)
            surf.blit(tv, tv.get_rect(centerx=tx, top=temp_y + lbl.get_height() + 2))

        # ── Faults ────────────────────────────────────────────────────────────
        fault_y = temp_y + font_sm.get_height() + font_med.get_height() + 14
        pygame.draw.line(surf, BORDER, (20, fault_y - 6), (W - 20, fault_y - 6))

        faults = []
        if tel['left_fault']:  faults.append("LEFT MOTOR")
        if tel['right_fault']: faults.append("RIGHT MOTOR")

        if faults:
            for fi, f in enumerate(faults):
                # Pulsing red for active faults
                alpha = int(160 + 95 * math.sin(time.monotonic() * 4))
                fc    = (220, max(0, alpha - 160), max(0, alpha - 160))
                ft    = font_med.render(f"⚠ FAULT: {f}", True, fc)
                surf.blit(ft, ft.get_rect(centerx=W // 2 + (fi - 0.5) * 200, top=fault_y))
        else:
            ok = font_sm.render("No faults", True, GREEN)
            surf.blit(ok, ok.get_rect(centerx=W // 2, top=fault_y))

        pygame.display.flip()
        clock.tick(30)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Pygame battery & telemetry monitor for Yukon RP2040.")
    parser.add_argument('--port',     default='/dev/yukon',
                        help='Serial port (default: /dev/yukon)')
    parser.add_argument('--baud',     type=int, default=115200)
    parser.add_argument('--cells',    type=int, default=3, choices=[2, 3, 4, 6],
                        help='LiPo cell count — sets voltage range (default: 3)')
    parser.add_argument('--interval', type=float, default=1.0, metavar='SECS',
                        help='Sensor poll interval (default: 1.0 s)')
    args = parser.parse_args()

    # Per-cell: 3.3 V depleted → 4.2 V full
    v_min = args.cells * 3.3
    v_max = args.cells * 4.2

    t = threading.Thread(
        target=_poll_thread,
        args=(args.port, args.baud, args.interval),
        daemon=True,
    )
    t.start()

    run_ui(v_min, v_max)


if __name__ == '__main__':
    main()
