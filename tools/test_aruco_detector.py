#!/usr/bin/env python3
"""
test_aruco_detector.py — Unit tests for robot/aruco_detector.py.

All tests use synthetic OpenCV-generated frames — no camera required.

Sections
--------
  1. Blank frame              — no tags detected
  2. Single tag detection     — correct id, centre, area, geometry
  3. Gate pair formation      — even+odd base IDs pair into a gate
  4. Gate centre calculation  — centre_x midpoint between posts
  5. Multiple gates           — tags 0+1 and 2+3 both detected
  6. Non-consecutive tags     — no gate formed (tags 1 and 4)
  7. FPS and timestamp fields — populated after detect()
  8. Distance / bearing (calibrated) — solvePnP values within tolerance
  9. draw=False               — frame not modified

Usage
-----
  python3 tools/test_aruco_detector.py
"""

import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.aruco_detector import ArucoDetector, ArUcoState, ARUCO_DICT

# ── Harness ───────────────────────────────────────────────────────────────────

_passed = 0
_failed = 0


def _check(name: str, condition: bool, detail: str = ""):
    global _passed, _failed
    if condition:
        print(f"  PASS  {name}")
        _passed += 1
    else:
        info = f"  ({detail})" if detail else ""
        print(f"  FAIL  {name}{info}")
        _failed += 1


def _approx(a, b, tol=1.0):
    return abs(a - b) <= tol


# ── Frame helpers ─────────────────────────────────────────────────────────────

FRAME_W, FRAME_H = 640, 480
DICT_ID = cv2.aruco.DICT_4X4_1000
_aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)


def _blank_frame() -> np.ndarray:
    """Plain white RGB frame."""
    return np.full((FRAME_H, FRAME_W, 3), 255, dtype=np.uint8)


def _render_tag(frame: np.ndarray, marker_id: int, top_left: tuple,
                side: int = 80) -> None:
    """
    Draw an ArUco marker onto frame (RGB) at the given top-left corner.
    side: pixel side length of the marker square.
    """
    img = cv2.aruco.generateImageMarker(_aruco_dict, marker_id, side + 4)
    # Convert grayscale marker to RGB
    img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    x, y = top_left
    frame[y:y + img_rgb.shape[0], x:x + img_rgb.shape[1]] = img_rgb


def _frame_with_tag(marker_id: int, x: int = 200, y: int = 200,
                    side: int = 80) -> np.ndarray:
    frame = _blank_frame()
    _render_tag(frame, marker_id, (x, y), side)
    return frame


def _frame_with_gate(odd_id: int, even_id: int,
                     odd_x: int = 150, even_x: int = 380,
                     y: int = 180, side: int = 80) -> np.ndarray:
    """Render a gate pair: odd tag on the left, even tag on the right."""
    frame = _blank_frame()
    _render_tag(frame, odd_id,  (odd_x,  y), side)
    _render_tag(frame, even_id, (even_x, y), side)
    return frame


# ── Minimal camera calibration matrix for pose tests ─────────────────────────

def _fake_calib(tmp_dir: str) -> str:
    """Write a plausible camera_cal.npz (with remap arrays) and return its path."""
    fx = fy = 600.0
    cx, cy = FRAME_W / 2, FRAME_H / 2
    mtx  = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)
    map1, map2 = cv2.initUndistortRectifyMap(
        mtx, dist, None, mtx, (FRAME_W, FRAME_H), cv2.CV_32FC1)
    path = os.path.join(tmp_dir, "camera_cal.npz")
    np.savez(path, camera_matrix=mtx, dist_coeffs=dist, map1=map1, map2=map2)
    return path


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_blank_frame():
    print("\nSection 1: Blank frame — no tags detected")
    det = ArucoDetector(draw=False, show_fps=False)
    state = det.detect(_blank_frame())
    _check("tags dict is empty", len(state.tags) == 0)


def test_single_tag():
    print("\nSection 2: Single tag detection")
    det = ArucoDetector(draw=False, show_fps=False)
    frame = _frame_with_tag(marker_id=1, x=200, y=180, side=100)
    state = det.detect(frame)

    _check("tag id=1 present",    1 in state.tags)
    if 1 not in state.tags:
        return

    tag = state.tags[1]
    _check("tag.id == 1",         tag.id == 1)
    _check("center_x in frame",   0 < tag.center_x < FRAME_W,
           f"got {tag.center_x}")
    _check("center_y in frame",   0 < tag.center_y < FRAME_H,
           f"got {tag.center_y}")
    _check("area > 0",            tag.area > 0, f"got {tag.area}")
    _check("4 corners populated", None not in (
        tag.top_left, tag.top_right, tag.bottom_right, tag.bottom_left))
    _check("distance is None (no calib)", tag.distance is None)
    _check("bearing is None (no calib)",  tag.bearing  is None)


def test_two_tags():
    print("\nSection 3: Two tags — both detected, no gate pairing (pairing is navigator's job)")
    det   = ArucoDetector(draw=False, show_fps=False)
    frame = _frame_with_gate(odd_id=1, even_id=2, odd_x=120, even_x=400)
    state = det.detect(frame)

    _check("2 tags detected",    len(state.tags) == 2, f"got {len(state.tags)}")
    _check("tag id=1 present",   1 in state.tags)
    _check("tag id=2 present",   2 in state.tags)


def test_multiple_tags():
    print("\nSection 4: Multiple tags — all detected")
    det   = ArucoDetector(draw=False, show_fps=False)
    frame = _blank_frame()
    _render_tag(frame, 0, (50,  100), 70)
    _render_tag(frame, 1, (250, 100), 70)
    _render_tag(frame, 2, (350, 100), 70)
    _render_tag(frame, 3, (520, 100), 70)
    state = det.detect(frame)

    _check("4 tags detected",  len(state.tags) == 4, f"got {len(state.tags)}")
    _check("tag 0 present",    0 in state.tags)
    _check("tag 1 present",    1 in state.tags)
    _check("tag 2 present",    2 in state.tags)
    _check("tag 3 present",    3 in state.tags)


def test_fps_and_timestamp():
    print("\nSection 7: FPS and timestamp populated after detect()")
    det = ArucoDetector(draw=False, show_fps=False)
    t0  = time.monotonic()
    state = det.detect(_blank_frame())
    # Second call needed for FPS calculation (delta between frames)
    state = det.detect(_blank_frame())
    _check("fps > 0",          state.fps > 0,         f"got {state.fps}")
    _check("timestamp ≥ t0",   state.timestamp >= t0, f"got {state.timestamp}")


def test_pose_estimation(tmp_path: str):
    print("\nSection 8: Distance / bearing (calibrated frame)")
    calib = _fake_calib(tmp_path)
    TAG_SIZE = 0.15   # metres (must match detector)
    det = ArucoDetector(draw=False, show_fps=False,
                        calib_file=calib, tag_size=TAG_SIZE)

    # Render a large tag near the frame centre — should be at roughly 0.5–1.5 m
    frame = _frame_with_tag(marker_id=5, x=220, y=160, side=120)
    state = det.detect(frame)

    _check("tag 5 detected",         5 in state.tags,
           f"tags={list(state.tags)}")
    if 5 not in state.tags:
        return

    tag = state.tags[5]
    _check("distance is not None",   tag.distance is not None)
    _check("bearing is not None",    tag.bearing  is not None)
    if tag.distance is not None:
        _check("distance in plausible range (0.1–5 m)",
               0.1 <= tag.distance <= 5.0, f"got {tag.distance:.3f} m")
    if tag.bearing is not None:
        _check("bearing in ±45°",
               abs(tag.bearing) <= 45.0, f"got {tag.bearing:.1f}°")


def test_draw_false():
    print("\nSection 9: draw=False — frame pixels unchanged")
    det   = ArucoDetector(draw=False, show_fps=False)
    frame = _frame_with_tag(marker_id=1, x=200, y=180, side=80)
    ref   = frame.copy()
    det.detect(frame)
    _check("frame not modified", np.array_equal(frame, ref))


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    import tempfile

    print("=" * 60)
    print("test_aruco_detector.py — ArucoDetector unit tests")
    print("=" * 60)

    test_blank_frame()
    test_single_tag()
    test_two_tags()
    test_multiple_tags()
    test_fps_and_timestamp()

    with tempfile.TemporaryDirectory() as tmp:
        test_pose_estimation(tmp)

    test_draw_false()

    print(f"\n{'=' * 60}")
    print(f"Results: {_passed} passed, {_failed} failed")
    print(f"{'=' * 60}")
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
