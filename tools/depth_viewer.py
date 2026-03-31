#!/usr/bin/env python3
"""
tools/depth_viewer.py — Standalone depth map visualiser.

Connects to a running Robot instance and displays:
  - Left and right rectified camera frames side by side
  - Jet-colourmap depth map (stereo, mono, or fusion)
  - Depth histogram
  - Source and metric status overlay

Does NOT instantiate Robot directly; reads depth over the web stream so it
works even when the robot is on a different machine.

Usage
-----
  # Local (robot running on same machine):
  python3 tools/depth_viewer.py

  # Remote:
  python3 tools/depth_viewer.py --host 192.168.1.42 --port 5000

  Options:
    --host HOST    Dashboard host (default: localhost)
    --port PORT    Dashboard port (default: 5000)
    --max-depth M  Depth range for colour scale in metres (default: 8.0)
    --fps N        Display refresh rate (default: 15)

Controls
--------
  Q / Esc   Quit
"""

import argparse
import sys
import time
import threading
import urllib.request
from io import BytesIO

import cv2
import numpy as np

# ── Pygame import ─────────────────────────────────────────────────────────────
try:
    import pygame
except ImportError:
    print("pygame not installed — pip install pygame", file=sys.stderr)
    sys.exit(1)

# ── Layout constants ──────────────────────────────────────────────────────────

_CAM_W, _CAM_H   = 320, 240    # display size for each camera panel
_DEPTH_W         = 640          # depth map panel width (same aspect as dual camera)
_HIST_H          = 80           # histogram bar height
_MARGIN          = 8
_FONT_SIZE       = 16
_LABEL_H         = 22

_WIN_W  = _CAM_W * 2 + _MARGIN * 3
_WIN_H  = _LABEL_H + _CAM_H + _MARGIN + _CAM_H + _HIST_H + _MARGIN * 3
# Row 0: labels
# Row 1: left cam + right cam (side by side)
# Row 2: depth map (full width, same height as cam row)
# Row 3: histogram

_DEPTH_H = _CAM_H   # depth panel matches camera row height


def _fetch_mjpeg_frame(url: str, timeout: float = 2.0) -> np.ndarray | None:
    """Grab one JPEG frame from an MJPEG stream URL."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            buf = b''
            while True:
                chunk = resp.read(4096)
                if not chunk:
                    break
                buf += chunk
                # Find JPEG boundaries
                start = buf.find(b'\xff\xd8')
                end   = buf.find(b'\xff\xd9', start + 2)
                if start >= 0 and end >= 0:
                    jpeg = buf[start:end + 2]
                    arr  = np.frombuffer(jpeg, dtype=np.uint8)
                    img  = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    buf = buf[end + 2:]
    except Exception:
        return None
    return None


class _StreamThread(threading.Thread):
    """Background thread that continuously fetches the latest frame from a URL."""

    def __init__(self, url: str):
        super().__init__(daemon=True)
        self._url   = url
        self._frame = None
        self._lock  = threading.Lock()
        self._stop  = threading.Event()

    def run(self):
        while not self._stop.is_set():
            frame = _fetch_mjpeg_frame(self._url, timeout=1.0)
            if frame is not None:
                with self._lock:
                    self._frame = frame
            else:
                time.sleep(0.1)

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    def stop(self):
        self._stop.set()


def _draw_histogram(surface: pygame.Surface,
                    depth_rgb: np.ndarray,
                    rect: pygame.Rect,
                    max_depth: float):
    """Draw a simple depth histogram onto *surface* within *rect*."""
    # Reconstruct approximate depth from Jet colour (inverse is not exact but useful for display)
    # Use luminance as a proxy for depth magnitude
    grey = cv2.cvtColor(depth_rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)
    hist, _ = np.histogram(grey, bins=rect.width, range=(0, 255))
    if hist.max() > 0:
        hist = (hist / hist.max() * (rect.height - 4)).astype(int)
    pygame.draw.rect(surface, (30, 30, 30), rect)
    for i, h in enumerate(hist):
        x = rect.left + i
        pygame.draw.line(surface,
                         (0, 200, 100),
                         (x, rect.bottom),
                         (x, rect.bottom - h))
    # Depth scale labels
    font = pygame.font.SysFont(None, 14)
    for d in (0, max_depth / 2, max_depth):
        label = font.render(f"{d:.1f}m", True, (180, 180, 180))
        lx = rect.left + int(d / max_depth * (rect.width - 1)) - label.get_width() // 2
        surface.blit(label, (lx, rect.top))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host',      default='localhost')
    parser.add_argument('--port',      type=int, default=5000)
    parser.add_argument('--max-depth', type=float, default=8.0)
    parser.add_argument('--fps',       type=int, default=15)
    args = parser.parse_args()

    base    = f"http://{args.host}:{args.port}"
    streams = {
        'left':  _StreamThread(f"{base}/stream/front_left"),
        'right': _StreamThread(f"{base}/stream/front_right"),
        'depth': _StreamThread(f"{base}/stream/depth_map"),
    }
    for t in streams.values():
        t.start()

    pygame.init()
    screen = pygame.display.set_mode((_WIN_W, _WIN_H))
    pygame.display.set_caption("HackyRacingRobot — Depth Viewer")
    clock  = pygame.font.SysFont(None, _FONT_SIZE)
    font   = pygame.font.SysFont(None, _FONT_SIZE)

    _dt = 1.0 / args.fps

    try:
        while True:
            # ── events ────────────────────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        return

            screen.fill((20, 20, 20))

            # ── column positions ──────────────────────────────────────────────
            x_left  = _MARGIN
            x_right = _MARGIN * 2 + _CAM_W
            y_cams  = _LABEL_H + _MARGIN
            y_depth = y_cams + _CAM_H + _MARGIN
            y_hist  = y_depth + _DEPTH_H + _MARGIN

            # ── camera labels ─────────────────────────────────────────────────
            for label, x in (("Front Left", x_left), ("Front Right", x_right)):
                surf = font.render(label, True, (180, 180, 180))
                screen.blit(surf, (x, _MARGIN))

            # ── camera frames ─────────────────────────────────────────────────
            for key, x in (('left', x_left), ('right', x_right)):
                frame = streams[key].get_frame()
                if frame is not None:
                    small = cv2.resize(frame, (_CAM_W, _CAM_H))
                    surf  = pygame.surfarray.make_surface(small.swapaxes(0, 1))
                    screen.blit(surf, (x, y_cams))
                else:
                    pygame.draw.rect(screen, (50, 50, 50),
                                     (x, y_cams, _CAM_W, _CAM_H))
                    no_sig = font.render("No signal", True, (120, 120, 120))
                    screen.blit(no_sig, (x + _CAM_W // 2 - no_sig.get_width() // 2,
                                         y_cams + _CAM_H // 2))

            # ── depth map ─────────────────────────────────────────────────────
            depth_frame = streams['depth'].get_frame()
            depth_rect  = pygame.Rect(_MARGIN, y_depth, _WIN_W - _MARGIN * 2, _DEPTH_H)
            if depth_frame is not None:
                scaled = cv2.resize(depth_frame, (depth_rect.width, depth_rect.height))
                surf   = pygame.surfarray.make_surface(scaled.swapaxes(0, 1))
                screen.blit(surf, (depth_rect.left, depth_rect.top))
                # histogram
                hist_rect = pygame.Rect(_MARGIN, y_hist, _WIN_W - _MARGIN * 2, _HIST_H)
                _draw_histogram(screen, depth_frame, hist_rect, args.max_depth)
            else:
                pygame.draw.rect(screen, (50, 50, 50), depth_rect)
                no_depth = font.render("Depth map unavailable", True, (120, 120, 120))
                screen.blit(no_depth, (
                    depth_rect.centerx - no_depth.get_width() // 2,
                    depth_rect.centery,
                ))

            # ── colour scale bar ──────────────────────────────────────────────
            _bar_w = 16
            bar_x  = _WIN_W - _MARGIN - _bar_w
            bar_y  = y_depth
            for i in range(_DEPTH_H):
                t   = 1.0 - (i / _DEPTH_H)
                # Jet colourmap approximation
                r = int(np.clip((1.5 - abs(4 * t - 3)) * 255, 0, 255))
                g = int(np.clip((1.5 - abs(4 * t - 2)) * 255, 0, 255))
                b = int(np.clip((1.5 - abs(4 * t - 1)) * 255, 0, 255))
                pygame.draw.line(screen, (r, g, b),
                                 (bar_x, bar_y + i),
                                 (bar_x + _bar_w, bar_y + i))
            # Labels: 0 m (bottom) and max_depth (top)
            lbl0  = font.render("0m",               True, (200, 200, 200))
            lblmax = font.render(f"{args.max_depth:.0f}m", True, (200, 200, 200))
            screen.blit(lbl0,   (bar_x - lbl0.get_width() - 2,
                                  bar_y + _DEPTH_H - lbl0.get_height()))
            screen.blit(lblmax, (bar_x - lblmax.get_width() - 2, bar_y))

            pygame.display.flip()
            time.sleep(_dt)

    finally:
        for t in streams.values():
            t.stop()
        pygame.quit()


if __name__ == '__main__':
    main()
