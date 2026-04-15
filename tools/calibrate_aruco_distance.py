#!/usr/bin/env python3
"""
calibrate_aruco_distance.py — ArUco distance calibration tool.

Place two ArUco tags exactly GATE_WIDTH apart (default 1.0 m) and position
the robot SETUP_DIST from the gate centre (default 2.0 m).  The tool reads
the tag pixel areas from the camera and computes:

    area_k = D * sqrt(A)

where D is the known distance to each post (sqrt(setup_dist² + (gate_width/2)²))
and A is the detected tag area in pixels².  Multiple captures are averaged.

If a camera calibration file (camera_cal.npz) is present, solvePnP distances
are also shown for comparison — a close match confirms the calibration file is
good and area_k may not be needed.

Controls:
  SPACE  — capture current frame and add to average
  C      — clear all captures
  S      — save recommended area_k to robot.ini [aruco] section
  Q/ESC  — quit

Usage:
  python3 tools/calibrate_aruco_distance.py
  python3 tools/calibrate_aruco_distance.py --setup-dist 2.0 --gate-width 1.0
  python3 tools/calibrate_aruco_distance.py --camera 1   # right front camera
"""

import argparse
import configparser
import math
import os
import re
import sys
import time
import ctypes

import cv2
import numpy as np
import pygame

# ── Suppress GLib noise from picamera2/GStreamer teardown ─────────────────────
try:
    _glib    = ctypes.CDLL('libglib-2.0.so.0')
    _LogFunc = ctypes.CFUNCTYPE(
        None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_void_p)
    _glib.g_log_set_handler(b'GLib-GObject', 1 << 3,
                            _LogFunc(lambda *_: None), None)
except Exception:
    pass

try:
    from picamera2 import Picamera2
    _USE_PICAMERA2 = True
except ImportError:
    _USE_PICAMERA2 = False

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE    = os.path.dirname(os.path.abspath(__file__))
_ROOT    = os.path.normpath(os.path.join(_HERE, '..'))
_INI     = os.path.join(_ROOT, 'robot.ini')

sys.path.insert(0, _ROOT)
from robot.aruco_detector import ArucoDetector, ARUCO_DICT

# ── Colours ───────────────────────────────────────────────────────────────────
_WHITE  = (255, 255, 255)
_BLACK  = (  0,   0,   0)
_GREEN  = (  0, 210,   0)
_RED    = (220,  50,  50)
_YELLOW = (255, 210,   0)
_CYAN   = (  0, 200, 240)
_GREY   = (140, 140, 140)
_ORANGE = (255, 160,   0)

# ── Panel layout ──────────────────────────────────────────────────────────────
_PANEL_W  = 340
_CAM_W    = 640
_CAM_H    = 480
_WIN_W    = _CAM_W + _PANEL_W
_WIN_H    = _CAM_H


# ── Camera wrapper ─────────────────────────────────────────────────────────────

class _Cam:
    def __init__(self, width: int, height: int, cam_index: int = 0, rotation: int = 0):
        self._pc2      = None
        self._cap      = None
        self._rotation = rotation
        if _USE_PICAMERA2:
            print(f"Using picamera2 (cam {cam_index}, {width}×{height})")
            pc2 = Picamera2(cam_index)
            pc2.configure(pc2.create_video_configuration(
                main={"size": (width, height), "format": "RGB888"}
            ))
            pc2.start()
            time.sleep(0.5)
            self._pc2 = pc2
        else:
            print(f"Using OpenCV backend (index {cam_index}, {width}×{height})")
            cap = cv2.VideoCapture(cam_index)
            if not cap.isOpened():
                sys.exit(f"ERROR: cannot open camera {cam_index}")
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._cap = cap

    def read(self):
        if self._pc2 is not None:
            frame = self._pc2.capture_array()
        else:
            ret, frame = self._cap.read()
            if not ret:
                return None
        r = self._rotation
        if r == 90:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if r == 180:
            return cv2.rotate(frame, cv2.ROTATE_180)
        if r == 270:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame

    def release(self):
        if self._pc2 is not None:
            self._pc2.stop()
        if self._cap is not None:
            self._cap.release()


# ── robot.ini helpers ─────────────────────────────────────────────────────────

def _read_ini():
    cp = configparser.ConfigParser(inline_comment_prefixes=('#', ';'))
    cp.read(_INI)
    return cp


def _save_area_k(new_value: float):
    """Update area_k in the [aruco] section of robot.ini, preserving all comments."""
    with open(_INI, 'r') as fh:
        text = fh.read()

    # Replace the area_k line; keep anything after # on same line
    pattern = r'(?m)^(\s*area_k\s*=\s*)[\d.]+(\s*(?:#.*)?)$'
    replacement = rf'\g<1>{new_value:.3f}\g<2>'
    new_text, n = re.subn(pattern, replacement, text)
    if n == 0:
        print("WARNING: could not find 'area_k' line in robot.ini — not saved.")
        return False
    with open(_INI, 'w') as fh:
        fh.write(new_text)
    return True


# ── Panel renderer ─────────────────────────────────────────────────────────────

def _draw_panel(surface: pygame.Surface, font_lg, font_sm,
                setup_dist: float, gate_width: float,
                tag_dist: float,
                tags_info: list,
                captures: list,
                current_area_k: float,
                pose_ok: bool):
    """Draw the right-hand info panel."""
    px, py = _CAM_W + 8, 8
    lh_lg  = font_lg.get_linesize()
    lh_sm  = font_sm.get_linesize()

    def txt(text, colour=_WHITE, big=False):
        nonlocal py
        f = font_lg if big else font_sm
        lh = lh_lg if big else lh_sm
        surface.blit(f.render(text, True, colour), (px, py))
        py += lh

    def sep():
        nonlocal py
        pygame.draw.line(surface, _GREY,
                         (_CAM_W + 4, py + 2), (_WIN_W - 4, py + 2), 1)
        py += 8

    txt("ArUco Distance Cal", _CYAN, big=True)
    sep()

    txt(f"Setup:  {setup_dist:.2f} m from gate centre", _WHITE)
    txt(f"Gate:   {gate_width:.2f} m between posts", _WHITE)
    txt(f"Tag D:  {tag_dist:.3f} m (each post)", _YELLOW)
    sep()

    txt("Visible tags:", _WHITE)
    if not tags_info:
        txt("  (none detected)", _GREY)
    for info in tags_info:
        colour = _GREEN if info['area_k'] else _RED
        txt(f"  Tag {info['id']:>3}  area={info['area']:>6}px²", colour)
        if info['dist_pose'] is not None:
            txt(f"    pose dist = {info['dist_pose']:.3f} m", _CYAN)
        if info['area_k'] is not None:
            txt(f"    area_k   = {info['area_k']:.3f}", _GREEN)
    sep()

    txt(f"Captures: {len(captures)}", _WHITE)
    if captures:
        avg = sum(captures) / len(captures)
        std = math.sqrt(sum((v - avg) ** 2 for v in captures) / len(captures)) \
              if len(captures) > 1 else 0.0
        txt(f"  avg area_k = {avg:.3f}", _GREEN)
        txt(f"  std dev    = {std:.3f}", _GREY)
    sep()

    txt(f"Current robot.ini:", _WHITE)
    txt(f"  area_k = {current_area_k:.3f}", _ORANGE)
    if pose_ok:
        txt("(pose/solvePnP active)", _CYAN)
    sep()

    txt("SPACE  capture", _WHITE)
    txt("C      clear captures", _WHITE)
    txt("S      save to robot.ini", _GREEN if captures else _GREY)
    txt("Q/ESC  quit", _WHITE)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--setup-dist', type=float, default=2.0,
                    help='Distance from robot to gate centre in metres (default 2.0)')
    ap.add_argument('--gate-width', type=float, default=1.0,
                    help='Distance between the two posts in metres (default 1.0)')
    ap.add_argument('--camera', type=int, default=0,
                    help='Camera index (default 0)')
    ap.add_argument('--ini', default=_INI,
                    help=f'Path to robot.ini (default {_INI})')
    ap.add_argument('--width',  type=int, default=_CAM_W)
    ap.add_argument('--height', type=int, default=_CAM_H)
    args = ap.parse_args()

    # Known distance to each post
    half_gate   = args.gate_width / 2.0
    tag_dist    = math.sqrt(args.setup_dist ** 2 + half_gate ** 2)

    # Read robot.ini
    cp  = _read_ini()
    sec = cp['aruco'] if cp.has_section('aruco') else {}
    dict_name      = sec.get('dict',       'DICT_4X4_250')
    tag_size       = float(sec.get('tag_size',  '0.15'))
    current_area_k = float(sec.get('area_k',    '0.0'))
    hfov           = float(sec.get('hfov',      '62.0'))

    # Build calibration file path for this resolution
    calib_file = sec.get('calib_file', 'camera_cal_{width}x{height}.npz')
    calib_file = calib_file.format(width=args.width, height=args.height)
    calib_path = os.path.join(_ROOT, calib_file)
    if not os.path.exists(calib_path):
        calib_path = None
        print(f"No calibration file at {calib_file} — pose estimation disabled.")
    else:
        print(f"Using calibration file: {calib_file}")

    detector = ArucoDetector(
        dict_name  = dict_name,
        draw       = True,
        show_fps   = True,
        calib_file = calib_path,
        tag_size   = tag_size,
        area_k     = 0.0,   # we compute area_k ourselves
        hfov       = hfov,
    )
    pose_ok = calib_path is not None

    cam = _Cam(args.width, args.height, cam_index=args.camera)

    rotation = int(sec.get('rotation', '0') if cp.has_section('camera') else '0')

    pygame.init()
    screen = pygame.display.set_mode((_WIN_W, _WIN_H))
    pygame.display.set_caption("ArUco Distance Calibration")
    clock    = pygame.time.Clock()
    font_lg  = pygame.font.SysFont('monospace', 17, bold=True)
    font_sm  = pygame.font.SysFont('monospace', 14)

    captures: list = []
    running  = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

                elif event.key == pygame.K_SPACE:
                    # Capture: grab area_k from current visible tags
                    frame = cam.read()
                    if frame is not None:
                        state = detector.detect(frame)
                        new_vals = []
                        for tag in state.tags.values():
                            if tag.area > 0:
                                k = tag_dist * math.sqrt(tag.area)
                                new_vals.append(k)
                        if new_vals:
                            avg_k = sum(new_vals) / len(new_vals)
                            captures.append(avg_k)
                            print(f"Captured: area_k = {avg_k:.3f}  "
                                  f"(from {len(new_vals)} tag(s))")
                        else:
                            print("No tags visible — nothing captured.")

                elif event.key == pygame.K_c:
                    captures.clear()
                    print("Captures cleared.")

                elif event.key == pygame.K_s:
                    if not captures:
                        print("No captures yet — nothing to save.")
                    else:
                        best = sum(captures) / len(captures)
                        if _save_area_k(best):
                            print(f"Saved area_k = {best:.3f} to robot.ini")
                            current_area_k = best
                        else:
                            print("Save failed — check robot.ini manually.")

        # ── Grab and detect ───────────────────────────────────────────────────
        frame = cam.read()
        if frame is None:
            continue

        state = detector.detect(frame)

        # Build per-tag info
        tags_info = []
        for tag in sorted(state.tags.values(), key=lambda t: t.id):
            area_k_val = (tag_dist * math.sqrt(tag.area)) if tag.area > 0 else None
            tags_info.append({
                'id':        tag.id,
                'area':      tag.area,
                'dist_pose': tag.distance,   # None unless calib file present
                'area_k':    area_k_val,
            })

        # ── Draw camera frame ─────────────────────────────────────────────────
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        surf = pygame.image.frombuffer(rgb.tobytes(), (rgb.shape[1], rgb.shape[0]), 'RGB')
        screen.fill(_BLACK)
        screen.blit(pygame.transform.scale(surf, (_CAM_W, _CAM_H)), (0, 0))

        # Highlight gate: draw line between the two post centres
        if len(state.tags) == 2:
            t1, t2 = list(state.tags.values())
            sx = frame.shape[1] / _CAM_W
            sy = frame.shape[0] / _CAM_H
            p1 = (int(t1.center_x / sx), int(t1.center_y / sy))
            p2 = (int(t2.center_x / sx), int(t2.center_y / sy))
            pygame.draw.line(screen, _YELLOW, p1, p2, 2)
            mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
            label = font_sm.render(f"{args.gate_width:.2f} m", True, _YELLOW)
            screen.blit(label, (mid[0] - label.get_width() // 2, mid[1] - 20))

        # ── Draw panel ────────────────────────────────────────────────────────
        _draw_panel(screen, font_lg, font_sm,
                    setup_dist     = args.setup_dist,
                    gate_width     = args.gate_width,
                    tag_dist       = tag_dist,
                    tags_info      = tags_info,
                    captures       = captures,
                    current_area_k = current_area_k,
                    pose_ok        = pose_ok)

        pygame.display.flip()
        clock.tick(30)

    cam.release()
    pygame.quit()

    if captures:
        best = sum(captures) / len(captures)
        print(f"\nFinal average area_k = {best:.3f}  (from {len(captures)} capture(s))")
        print(f"Add to robot.ini [aruco]:  area_k = {best:.3f}")


if __name__ == '__main__':
    main()
