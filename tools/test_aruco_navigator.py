#!/usr/bin/env python3
"""
test_aruco_navigator.py — Unit tests for robot/aruco_navigator.py state machine.

All tests use synthetic ArUcoState inputs — no camera, motors, or IMU required.
The navigator's update() is called directly with crafted states.

Sections
--------
  1.  Initial state           — IDLE, gate_id=0
  2.  IDLE returns zero       — no movement in IDLE
  3.  start() → SEARCHING     — lifecycle
  4.  stop() → IDLE           — lifecycle
  5.  SEARCHING rotates       — non-zero, asymmetric output
  6.  SEARCHING → ALIGNING    — gate becomes visible
  7.  ALIGNING → APPROACHING  — bearing error within deadband
  8.  APPROACHING → PASSING   — distance ≤ pass_distance
  9.  PASSING → SEARCHING     — pass_time elapsed; gate_id advances
 10.  APPROACHING → RECOVERING — gate lost mid-approach
 11.  RECOVERING → SEARCHING  — recover_reverse_time elapsed
 12.  COMPLETE after last gate — max_gates=1 → COMPLETE
 13.  COMPLETE returns zero    — no movement
 14.  Obstacle stop            — LiDAR halts APPROACHING
 15.  _angle_diff wraparound   — corner cases ±180
 16.  from_ini() loading       — reads NavConfig from .ini
 17.  from_ini() missing section — falls back to defaults
 18.  Single-tag fallback      — _resolve_target aims beside post
 19.  IMU search stepping      — heading-controlled rotation steps
 20.  bearing_err attribute    — updated each frame when target visible

Usage
-----
  python3 tests/test_aruco_navigator.py
"""

import math
import os
import sys
import tempfile
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.aruco_navigator import (
    ArucoNavigator, NavConfig, NavState, ArUcoGate, _angle_diff, _clamp, _ramp,
)
from robot.aruco_detector import ArUcoState, ArUcoTag

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


def _approx(a, b, tol=0.05):
    return abs(a - b) <= tol


# ── ArUcoState factory helpers ────────────────────────────────────────────────

FRAME_W = 640


def _empty_state() -> ArUcoState:
    return ArUcoState()


def _tag(tid: int, cx: int = 320, cy: int = 240,
         dist: float = 1.0, bearing: float = 0.0) -> ArUcoTag:
    return ArUcoTag(
        id=tid, center_x=cx, center_y=cy, area=4000,
        top_left=(cx - 40, cy - 40), top_right=(cx + 40, cy - 40),
        bottom_right=(cx + 40, cy + 40), bottom_left=(cx - 40, cy + 40),
        distance=dist, bearing=bearing,
    )


def _state_with_gate(gate_id: int, cx: int = 320, dist: float = 1.0,
                     bearing: float = 0.0) -> ArUcoState:
    """Build an ArUcoState with both posts of gate_id visible.

    Gate N uses outside tag 2N (even) and inside tag 2N+1 (odd).
    Tags are placed symmetrically around cx.
    """
    outside_id = gate_id * 2
    inside_id  = gate_id * 2 + 1
    t_outside = _tag(outside_id, cx=cx + 80, dist=dist, bearing=bearing + 5.0)
    t_inside  = _tag(inside_id,  cx=cx - 80, dist=dist, bearing=bearing - 5.0)
    return ArUcoState(
        tags={outside_id: t_outside, inside_id: t_inside},
        fps=30.0, timestamp=time.monotonic(),
    )


# ── Fake LiDAR scan ───────────────────────────────────────────────────────────

@dataclass
class _FakeScan:
    """Minimal LidarScan-compatible object."""
    angles:    list = field(default_factory=list)
    distances: list = field(default_factory=list)


def _lidar_obstacle(distance_m: float, angle_deg: float = 0.0) -> _FakeScan:
    """Single LiDAR return directly ahead."""
    return _FakeScan(angles=[angle_deg], distances=[distance_m])


def _lidar_clear() -> _FakeScan:
    """LiDAR returns all far away."""
    angles    = list(range(-30, 31, 5))
    distances = [5.0] * len(angles)
    return _FakeScan(angles=angles, distances=distances)


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_initial_state():
    print("\nSection 1: Initial state")
    nav = ArucoNavigator()
    _check("initial state IDLE",  nav.state   == NavState.IDLE)
    _check("initial gate_id=0",   nav.gate_id == 0)


def test_idle_returns_zero():
    print("\nSection 2: IDLE returns (0, 0)")
    nav  = ArucoNavigator()
    l, r = nav.update(_empty_state(), FRAME_W)
    _check("left=0",  _approx(l, 0.0))
    _check("right=0", _approx(r, 0.0))


def test_start_searching():
    print("\nSection 3: start() → SEARCHING")
    nav = ArucoNavigator()
    nav.start()
    _check("state SEARCHING", nav.state   == NavState.SEARCHING)
    _check("gate_id=0",        nav.gate_id == 0)


def test_stop_idle():
    print("\nSection 4: stop() → IDLE")
    nav = ArucoNavigator()
    nav.start()
    nav.stop()
    _check("state IDLE after stop()", nav.state == NavState.IDLE)


def test_searching_rotates():
    print("\nSection 5: SEARCHING rotates (no IMU)")
    nav = ArucoNavigator()
    nav.start()
    time.sleep(0.05)   # allow dt > 0 so ramp produces non-zero output
    l, r = nav.update(_empty_state(), FRAME_W)
    _check("outputs differ (rotation)", l != r, f"l={l:.3f} r={r:.3f}")
    _check("rotation is non-zero",      abs(l) > 0 or abs(r) > 0)


def test_searching_to_aligning():
    print("\nSection 6: SEARCHING → ALIGNING when gate visible")
    nav = ArucoNavigator()
    nav.start()
    # Gate 0 visible, centred, bearing=0
    state = _state_with_gate(0, bearing=10.0)  # outside deadband
    nav.update(state, FRAME_W)
    _check("transitioned to ALIGNING or APPROACHING",
           nav.state in (NavState.ALIGNING, NavState.APPROACHING),
           f"got {nav.state}")


def test_aligning_to_approaching():
    print("\nSection 7: ALIGNING → APPROACHING when bearing within deadband")
    cfg = NavConfig(align_deadband=5.0)
    nav = ArucoNavigator(cfg)
    nav.start()
    nav._set_state(NavState.ALIGNING)

    # Bearing error well within deadband
    state = _state_with_gate(0, bearing=0.0)
    nav.update(state, FRAME_W)
    _check("state APPROACHING",
           nav.state == NavState.APPROACHING, f"got {nav.state}")


def test_approaching_to_passing():
    print("\nSection 8: APPROACHING → PASSING when dist ≤ pass_distance")
    cfg = NavConfig(pass_distance=0.6, pass_time=0.5)
    nav = ArucoNavigator(cfg)
    nav.start()
    nav._set_state(NavState.APPROACHING)

    # Distance exactly at threshold
    state = _state_with_gate(0, dist=0.55, bearing=0.0)
    nav.update(state, FRAME_W)
    _check("state PASSING", nav.state == NavState.PASSING, f"got {nav.state}")


def test_passing_to_next_gate():
    print("\nSection 9: PASSING → SEARCHING; gate_id advances")
    cfg = NavConfig(pass_distance=0.6, pass_time=0.0, max_gates=3)
    nav = ArucoNavigator(cfg)
    nav.start()
    nav._set_state(NavState.APPROACHING)

    # Trigger PASSING
    state = _state_with_gate(0, dist=0.5, bearing=0.0)
    nav.update(state, FRAME_W)
    _check("entered PASSING", nav.state == NavState.PASSING, f"got {nav.state}")

    # Force pass_time elapsed
    nav._pass_start = time.monotonic() - 10.0
    # Inject gate 1 so the navigator can confirm it's visible
    state1 = _state_with_gate(1, dist=2.0, bearing=5.0)
    nav.update(state1, FRAME_W)
    _check("gate_id advanced to 1",   nav.gate_id == 1, f"got {nav.gate_id}")
    _check("state back to SEARCHING", nav.state == NavState.SEARCHING,
           f"got {nav.state}")


def test_approaching_to_recovering():
    print("\nSection 10: APPROACHING → RECOVERING when gate lost")
    cfg = NavConfig(recover_reverse_time=0.3, recover_reverse_speed=0.2)
    nav = ArucoNavigator(cfg)
    nav.start()
    nav._set_state(NavState.APPROACHING)

    # Gate visible once to set up the state properly
    state = _state_with_gate(0, dist=1.0, bearing=0.0)
    nav.update(state, FRAME_W)

    # Simulate elapsed time so the ramp can produce non-zero output
    nav._last_update -= 0.2

    # Now gate is gone
    l, r = nav.update(_empty_state(), FRAME_W)
    _check("state RECOVERING",  nav.state == NavState.RECOVERING,
           f"got {nav.state}")
    _check("reversing (both negative)", l < 0 and r < 0,
           f"l={l:.3f} r={r:.3f}")


def test_recovering_to_searching():
    print("\nSection 11: RECOVERING → SEARCHING after timer")
    cfg = NavConfig(recover_reverse_time=0.01)
    nav = ArucoNavigator(cfg)
    nav.start()
    nav._set_state(NavState.RECOVERING)
    nav._recover_start = time.monotonic() - 1.0   # expired

    nav.update(_empty_state(), FRAME_W)
    _check("state SEARCHING after recovery", nav.state == NavState.SEARCHING,
           f"got {nav.state}")


def test_complete_after_last_gate():
    print("\nSection 12: COMPLETE after last gate passed")
    cfg = NavConfig(max_gates=1, pass_distance=1.0, pass_time=0.0)
    nav = ArucoNavigator(cfg)
    nav.start()
    nav._set_state(NavState.APPROACHING)

    state = _state_with_gate(0, dist=0.8, bearing=0.0)
    nav.update(state, FRAME_W)           # → PASSING

    nav._pass_start = time.monotonic() - 10.0
    nav.update(_empty_state(), FRAME_W)  # → COMPLETE (only 1 gate)
    _check("state COMPLETE", nav.state == NavState.COMPLETE,
           f"got {nav.state}")


def test_complete_returns_zero():
    print("\nSection 13: COMPLETE returns (0, 0)")
    nav = ArucoNavigator()
    nav._state = NavState.COMPLETE
    l, r = nav.update(_empty_state(), FRAME_W)
    _check("left=0",  _approx(l, 0.0))
    _check("right=0", _approx(r, 0.0))


def test_obstacle_stop():
    print("\nSection 14: Obstacle stop during APPROACHING")
    cfg = NavConfig(obstacle_stop_dist=0.5, obstacle_cone_deg=60.0)
    nav = ArucoNavigator(cfg)
    nav.start()
    nav._set_state(NavState.APPROACHING)

    state   = _state_with_gate(0, dist=2.0, bearing=0.0)
    lidar   = _lidar_obstacle(0.3, angle_deg=0.0)   # obstacle at 0.3 m, dead ahead
    l, r    = nav.update(state, FRAME_W, lidar=lidar)
    _check("motors halted (obstacle)", _approx(l, 0.0, tol=0.05)
           and _approx(r, 0.0, tol=0.05),
           f"l={l:.3f} r={r:.3f}")

    # Obstacle outside forward cone — should NOT halt
    lidar_side = _lidar_obstacle(0.3, angle_deg=50.0)  # 50° off-axis, cone=60° total
    nav2 = ArucoNavigator(cfg)
    nav2.start()
    nav2._set_state(NavState.APPROACHING)
    nav2._last_update -= 0.2   # simulate elapsed time so ramp can produce output
    l2, r2 = nav2.update(state, FRAME_W, lidar=lidar_side)
    _check("no halt when obstacle outside cone",
           abs(l2) > 0.01 or abs(r2) > 0.01,
           f"l={l2:.3f} r={r2:.3f}")


def test_angle_diff():
    print("\nSection 15: _angle_diff wraparound")
    cases = [
        (10,  350,  20.0),    # small positive crossing 0
        (350, 10,  -20.0),    # small negative crossing 0
        (180, 0,  -180.0),    # exactly half-circle — formula gives -180 (both signs valid)
        (0,   0,     0.0),    # no difference
        (90,  270, -180.0),   # exactly half-circle (negative)
        (1,   359,   2.0),    # 2° clockwise
        (359,  1,   -2.0),    # 2° anticlockwise
    ]
    for target, current, expected in cases:
        result = _angle_diff(target, current)
        ok = abs(result - expected) < 0.001
        _check(f"_angle_diff({target}, {current}) == {expected}",
               ok, f"got {result}")


def test_from_ini():
    print("\nSection 16: from_ini() loads NavConfig")
    cfg_text = "[navigator]\nmax_gates = 7\nfwd_speed = 0.55\nsteer_kp = 0.9\n"
    with tempfile.NamedTemporaryFile(mode='w', suffix='.ini', delete=False) as f:
        f.write(cfg_text)
        path = f.name
    try:
        nav = ArucoNavigator.from_ini(path)
        _check("max_gates=7",        nav.cfg.max_gates  == 7)
        _check("fwd_speed≈0.55",     _approx(nav.cfg.fwd_speed, 0.55, 0.001))
        _check("steer_kp≈0.9",       _approx(nav.cfg.steer_kp,  0.90, 0.001))
    finally:
        os.unlink(path)


def test_from_ini_missing_section():
    print("\nSection 17: from_ini() — missing section uses defaults")
    with tempfile.NamedTemporaryFile(mode='w', suffix='.ini', delete=False) as f:
        f.write("[other]\nfoo = bar\n")
        path = f.name
    try:
        nav = ArucoNavigator.from_ini(path)
        _check("max_gates == default",
               nav.cfg.max_gates == NavConfig.max_gates)
    finally:
        os.unlink(path)


def test_single_tag_fallback():
    print("\nSection 18: Single-tag fallback — _resolve_target offsets aim")
    nav = ArucoNavigator()
    nav.start()

    # Tags without bearing/distance → pixel-offset path (tx is modified directly).
    # Gate 0: outside post = tag 0 (even), inside post = tag 1 (odd).

    # Only outside tag visible (tag 0) — gate is to the RIGHT → aim right (tx > cx)
    outside_tag = ArUcoTag(
        id=0, center_x=280, center_y=240, area=4000,
        top_left=(240, 200), top_right=(320, 200),
        bottom_right=(320, 280), bottom_left=(240, 280),
        distance=None, bearing=None,
    )
    state = ArUcoState(tags={0: outside_tag}, fps=30.0, timestamp=time.monotonic())
    tx, dist, bear, _ = nav._resolve_target(state, FRAME_W // 2)

    _check("target_x returned",      tx is not None)
    _check("target_x > tag centre_x (aim right of outside post)",
           tx is not None and tx > outside_tag.center_x,
           f"tx={tx}, tag.cx={outside_tag.center_x}")

    # Only inside tag visible (tag 1) — gate is to the LEFT → aim left (tx < cx)
    inside_tag = ArUcoTag(
        id=1, center_x=360, center_y=240, area=4000,
        top_left=(320, 200), top_right=(400, 200),
        bottom_right=(400, 280), bottom_left=(320, 280),
        distance=None, bearing=None,
    )
    state2 = ArUcoState(tags={1: inside_tag}, fps=30.0, timestamp=time.monotonic())
    tx2, _, _, _ = nav._resolve_target(state2, FRAME_W // 2)

    _check("target_x < tag centre_x (aim left of inside post)",
           tx2 is not None and tx2 < inside_tag.center_x,
           f"tx={tx2}, tag.cx={inside_tag.center_x}")


def test_imu_search_step():
    print("\nSection 19: IMU search stepping")
    cfg = NavConfig(search_step_deg=45.0, search_step_pause=0.0)
    nav = ArucoNavigator(cfg)
    nav.start()
    # Simulate heading before step complete
    nav._search_origin = 10.0
    time.sleep(0.02)
    l, r = nav.update(_empty_state(), FRAME_W, heading=20.0)
    _check("still rotating (10° < 45° step)", l != r, f"l={l:.3f} r={r:.3f}")

    # Simulate step complete — heading has moved 45°
    nav._search_origin = 10.0
    time.sleep(0.02)
    l2, r2 = nav.update(_empty_state(), FRAME_W, heading=56.0)  # 46° turned
    _check("pausing after step complete (outputs near zero)",
           _approx(abs(l2), 0.0, tol=0.3) and _approx(abs(r2), 0.0, tol=0.3),
           f"l={l2:.3f} r={r2:.3f}")


def test_bearing_err_attribute():
    print("\nSection 20: bearing_err updated when target visible")
    cfg = NavConfig(align_deadband=2.0)
    nav = ArucoNavigator(cfg)
    nav.start()
    nav._set_state(NavState.ALIGNING)

    state = _state_with_gate(0, bearing=12.0)
    nav.update(state, FRAME_W)
    _check("bearing_err attribute set",
           hasattr(nav, 'bearing_err') and nav.bearing_err is not None)
    _check("bearing_err ≈ 12° (±5° tolerance)",
           hasattr(nav, 'bearing_err') and abs(nav.bearing_err - 12.0) <= 5.0,
           f"got {getattr(nav, 'bearing_err', None)}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("test_aruco_navigator.py — ArucoNavigator state machine tests")
    print("=" * 60)

    test_initial_state()
    test_idle_returns_zero()
    test_start_searching()
    test_stop_idle()
    test_searching_rotates()
    test_searching_to_aligning()
    test_aligning_to_approaching()
    test_approaching_to_passing()
    test_passing_to_next_gate()
    test_approaching_to_recovering()
    test_recovering_to_searching()
    test_complete_after_last_gate()
    test_complete_returns_zero()
    test_obstacle_stop()
    test_angle_diff()
    test_from_ini()
    test_from_ini_missing_section()
    test_single_tag_fallback()
    test_imu_search_step()
    test_bearing_err_attribute()

    print(f"\n{'=' * 60}")
    print(f"Results: {_passed} passed, {_failed} failed")
    print(f"{'=' * 60}")
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
