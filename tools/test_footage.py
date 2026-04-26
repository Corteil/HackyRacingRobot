#!/usr/bin/env python3
"""
tools/test_footage.py — Run ArUco and/or robot detection on recorded footage.

Plays back an MP4 (or any file cv2.VideoCapture can open) through the same
detector modules used by the live robot, with an annotated window showing
bounding boxes and stats in real time.

Usage
-----
    # ArUco only
    python3 tools/test_footage.py footage.mp4 --aruco

    # Robot detector only (requires trained HEF)
    python3 tools/test_footage.py footage.mp4 --robot-detector models/robot_detector.hef

    # Both together
    python3 tools/test_footage.py footage.mp4 --aruco --robot-detector models/robot_detector.hef

    # Save annotated video
    python3 tools/test_footage.py footage.mp4 --aruco --save annotated.mp4

Controls
--------
    Space       Pause / resume
    →           Step one frame (when paused)
    ←           Step one frame back (when paused)
    + / =       Speed up playback
    - / _       Slow down playback
    S           Save snapshot of current frame to ./snapshot_NNNN.jpg
    Q / Esc     Quit
"""

import sys
import os
import argparse
import configparser
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── Read robot.ini defaults for detector params ───────────────────────────────
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_ini = configparser.ConfigParser(inline_comment_prefixes=('#',))
_ini.read(os.path.join(_REPO_ROOT, 'robot.ini'))
_RD_CONF         = _ini.getfloat('robot_detector', 'conf',         fallback=0.45)
_RD_IOU          = _ini.getfloat('robot_detector', 'iou',          fallback=0.45)
_RD_PERSIST      = _ini.getint  ('robot_detector', 'persist',      fallback=2)
_RD_MATCH_RADIUS = _ini.getint  ('robot_detector', 'match_radius', fallback=60)

import cv2
import numpy as np

# Suppress the harmless GLib-GObject-CRITICAL g_object_unref warning emitted
# by OpenCV's GTK3 backend.  It fires during imshow/destroyAllWindows and
# cannot be caught at the Python level.  Install a no-op GLib log handler for
# that specific domain before any OpenCV window calls are made.
try:
    import ctypes, ctypes.util as _ctu
    _glib = ctypes.CDLL(_ctu.find_library('glib-2.0'))
    _GLogFunc = ctypes.CFUNCTYPE(
        None, ctypes.c_char_p, ctypes.c_uint, ctypes.c_char_p, ctypes.c_void_p)
    _noop_log = _GLogFunc(lambda *_: None)          # keep ref to prevent GC
    _glib.g_log_set_handler(
        b'GLib-GObject', ctypes.c_uint(1 << 3),    # G_LOG_LEVEL_CRITICAL
        _noop_log, None)
except Exception:
    pass

# ── Detector imports (both optional) ─────────────────────────────────────────

def _import_aruco():
    try:
        from robot.aruco_detector import ArucoDetector
        return ArucoDetector
    except ImportError as e:
        print(f"WARNING: cannot import ArucoDetector: {e}")
        return None


def _import_robot_detector():
    try:
        from robot.robot_detector import RobotDetector
        return RobotDetector
    except ImportError as e:
        print(f"WARNING: cannot import RobotDetector: {e}")
        return None


# ── Overlay drawing ───────────────────────────────────────────────────────────

_FONT       = cv2.FONT_HERSHEY_SIMPLEX
_FONT_SMALL = 0.45
_FONT_MED   = 0.55
_WHITE      = (255, 255, 255)
_BLACK      = (0,   0,   0)
_GREEN      = (0,   220, 80)
_ORANGE     = (255, 140, 0)
_GREY       = (180, 180, 180)


def _text(frame, text, x, y, colour=_WHITE, scale=_FONT_SMALL, thickness=1):
    cv2.putText(frame, text, (x, y), _FONT, scale, _BLACK,    thickness + 1, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), _FONT, scale, colour, thickness,     cv2.LINE_AA)


def _draw_overlay(frame, info: dict):
    """Draw semi-transparent stats panel in the top-left corner."""
    lines = info.get('lines', [])
    if not lines:
        return
    pad    = 8
    lh     = 18
    pw     = 220
    ph     = pad * 2 + lh * len(lines)

    # Semi-transparent dark background
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (pw, ph), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    for i, (text, colour) in enumerate(lines):
        _text(frame, text, pad, pad + lh * (i + 1) - 4, colour, _FONT_SMALL)


# ── Speed steps ───────────────────────────────────────────────────────────────

_SPEEDS = [0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 4.0, 8.0]
_DEFAULT_SPEED_IDX = 4   # 1.0×


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Test ArUco / robot detection on recorded footage",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('video',                           help='Path to MP4 footage file')
    parser.add_argument('--aruco',        action='store_true', help='Enable ArUco detection')
    parser.add_argument('--aruco-dict',   default='DICT_4X4_250',
                        help='ArUco dictionary (default: DICT_4X4_250)')
    parser.add_argument('--aruco-calib',  default=None,
                        help='Path to camera calibration .npz (optional)')
    parser.add_argument('--robot-detector', default=None, metavar='HEF',
                        help='Enable robot detector; path to .hef model file')
    parser.add_argument('--conf',  type=float, default=_RD_CONF,
                        help=f'Confidence threshold (default: {_RD_CONF} from robot.ini)')
    parser.add_argument('--iou',   type=float, default=_RD_IOU,
                        help=f'NMS IoU threshold (default: {_RD_IOU} from robot.ini)')
    parser.add_argument('--persist', type=int, default=_RD_PERSIST,
                        help=f'Consecutive frames required to confirm a detection '
                             f'(default: {_RD_PERSIST} from robot.ini; 1=off)')
    parser.add_argument('--match-radius', type=int, default=_RD_MATCH_RADIUS,
                        help=f'Centre-point match radius in pixels between frames '
                             f'(default: {_RD_MATCH_RADIUS} from robot.ini)')
    parser.add_argument('--save',  default=None, metavar='OUTPUT.mp4',
                        help='Save annotated video to file')
    parser.add_argument('--snapshot-dir', default=None, metavar='DIR',
                        help='Directory for S-key snapshots '
                             '(default: ~/Pictures/HackyRacingRobot)')
    parser.add_argument('--start', type=float, default=0.0,
                        help='Start time in seconds (default: 0)')
    args = parser.parse_args()

    snapshot_dir = os.path.expanduser(
        args.snapshot_dir if args.snapshot_dir else '~/Pictures/HackyRacingRobot')
    os.makedirs(snapshot_dir, exist_ok=True)

    if not args.aruco and not args.robot_detector:
        parser.error("Specify at least one of --aruco or --robot-detector")

    # ── Open video ────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"ERROR: cannot open {args.video}")
        sys.exit(1)

    src_fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width      = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if args.start > 0:
        cap.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000)

    print(f"Video   : {args.video}")
    print(f"Size    : {width}×{height}  {src_fps:.1f} fps  {total_frames} frames")

    # ── Build detectors ───────────────────────────────────────────────────────
    aruco_det   = None
    robot_det   = None

    if args.aruco:
        ArucoDetector = _import_aruco()
        if ArucoDetector:
            aruco_det = ArucoDetector(
                dict_name  = args.aruco_dict,
                calib_file = args.aruco_calib,
                draw       = True,
                show_fps   = False,
            )
            print(f"ArUco   : {args.aruco_dict}"
                  f"{f'  calib={args.aruco_calib}' if args.aruco_calib else '  (no calib)'}")

    if args.robot_detector:
        RobotDetector = _import_robot_detector()
        if RobotDetector:
            robot_det = RobotDetector(
                model_path   = args.robot_detector,
                conf         = args.conf,
                iou          = args.iou,
                persist      = args.persist,
                match_radius = args.match_radius,
                draw         = True,
            )
            status = "ready" if robot_det.available else "UNAVAILABLE (HEF missing or Hailo error)"
            print(f"RobotDet: {args.robot_detector}  conf={args.conf}  "
                  f"persist={args.persist}  [{status}]")

    # ── Output video writer ───────────────────────────────────────────────────
    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*'avc1')
        writer = cv2.VideoWriter(args.save, fourcc, src_fps, (width, height))
        if not writer.isOpened():
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(args.save, fourcc, src_fps, (width, height))
        print(f"Saving  : {args.save}")

    print()
    print("Controls: Space=pause  →/←=step  +/-=speed  S=snapshot  Q=quit")
    print()

    # ── Playback state ────────────────────────────────────────────────────────
    paused       = False
    speed_idx    = _DEFAULT_SPEED_IDX
    frame_num    = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
    snapshot_num = 0
    last_aruco   = None
    last_robots  = None
    frame_cache  = None   # hold last decoded frame for redraw when paused

    window = "Footage Test — Q to quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, min(width, 1280), min(height, 720))

    t_frame = time.monotonic()

    while True:
        speed = _SPEEDS[speed_idx]

        # ── Read next frame ───────────────────────────────────────────────────
        if not paused:
            ret, bgr = cap.read()
            if not ret:
                print("\nEnd of footage.")
                break
            frame_num = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
            frame_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            frame_cache = frame_rgb.copy()

            # ── Run detectors ─────────────────────────────────────────────────
            if aruco_det is not None:
                last_aruco = aruco_det.detect(frame_rgb)   # annotates in-place

            if robot_det is not None and robot_det.available:
                last_robots = robot_det.detect(frame_rgb)  # annotates in-place

        else:
            # Paused — redraw last frame without re-running detectors
            if frame_cache is not None:
                frame_rgb = frame_cache.copy()
                if aruco_det is not None and last_aruco is not None:
                    aruco_det.detect(frame_rgb)   # ArUco is stateless; safe to re-run
                if robot_det is not None and last_robots is not None:
                    robot_det._annotate(frame_rgb, last_robots.robots)

        # ── Build stats overlay ───────────────────────────────────────────────
        elapsed_s = frame_num / src_fps
        lines = [
            (f"Frame {frame_num}/{total_frames}  {elapsed_s:.1f}s", _GREY),
            (f"Speed  {speed:.2f}x{'  PAUSED' if paused else ''}", _WHITE),
        ]

        if aruco_det is not None:
            if last_aruco and last_aruco.tags:
                tag_ids = sorted(last_aruco.tags)
                lines.append((f"ArUco  {len(tag_ids)} tag(s): {tag_ids[:4]}", _GREEN))
            else:
                lines.append(("ArUco  no tags", _GREY))

        if robot_det is not None:
            if not robot_det.available:
                lines.append(("Robot  detector unavailable", _GREY))
            elif last_robots and last_robots.count > 0:
                lines.append((f"Robots {last_robots.count} detected", _ORANGE))
            else:
                lines.append(("Robots none", _GREY))

        _draw_overlay(frame_rgb, {'lines': lines})

        # ── Display ───────────────────────────────────────────────────────────
        display_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        cv2.imshow(window, display_bgr)

        if writer and not paused:
            writer.write(display_bgr)

        # ── Console line ──────────────────────────────────────────────────────
        if not paused:
            aruco_info  = (f"tags={sorted(last_aruco.tags)}" if last_aruco and last_aruco.tags else "no tags")
            robot_info  = (f"robots={last_robots.count}" if last_robots else "")
            parts = [f"frame={frame_num}/{total_frames}", aruco_info]
            if robot_info:
                parts.append(robot_info)
            print(f"\r  {' | '.join(parts)}    ", end='', flush=True)

        # ── Timing ───────────────────────────────────────────────────────────
        now    = time.monotonic()
        target = t_frame + (1.0 / src_fps) / speed
        delay  = max(1, int((target - now) * 1000))
        t_frame = max(now, target)

        key = cv2.waitKey(delay if not paused else 30) & 0xFF

        # ── Key handling ──────────────────────────────────────────────────────
        if key in (ord('q'), ord('Q'), 27):         # quit
            break
        elif key == ord(' '):                        # pause / resume
            paused = not paused
            if not paused:
                t_frame = time.monotonic()
        elif key == 83 and paused:                   # → right arrow: step forward
            ret, bgr = cap.read()
            if ret:
                frame_num = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
                frame_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                frame_cache = frame_rgb
        elif key == 81 and paused:                   # ← left arrow: step back
            target_frame = max(0, frame_num - 2)
            cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
            ret, bgr = cap.read()
            if ret:
                frame_num = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
                frame_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                frame_cache = frame_rgb
        elif key in (ord('+'), ord('=')):            # speed up
            speed_idx = min(speed_idx + 1, len(_SPEEDS) - 1)
        elif key in (ord('-'), ord('_')):            # slow down
            speed_idx = max(speed_idx - 1, 0)
        elif key in (ord('s'), ord('S')):            # snapshot
            snap_path = os.path.join(snapshot_dir, f"snapshot_{snapshot_num:04d}.jpg")
            cv2.imwrite(snap_path, display_bgr)
            print(f"\n  Snapshot saved: {snap_path}")
            snapshot_num += 1

    # ── Cleanup ───────────────────────────────────────────────────────────────
    print()
    cap.release()
    if writer:
        writer.release()
        print(f"Saved annotated video: {args.save}")
    if robot_det:
        robot_det.stop()
    cv2.waitKey(1)
    cv2.destroyAllWindows()
    cv2.waitKey(1)


if __name__ == '__main__':
    main()
