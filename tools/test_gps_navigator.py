#!/usr/bin/env python3
"""
test_gps_navigator.py — Unit tests for robot/gps_navigator.py state machine.

All tests use synthetic GpsState inputs — no GPS, motors, or IMU hardware
required.  The navigator's update() is called directly with crafted states.

Sections
--------
  1.  Geometry helpers         — _haversine, _bearing, _angle_diff sanity checks
  2.  Initial state            — IDLE, empty waypoints
  3.  start(), no waypoints    — transitions to ERROR
  4.  start(), with waypoints  — transitions to WAITING_FIX
  5.  stop() → IDLE            — lifecycle
  6.  IDLE / COMPLETE / ERROR  — return (0, 0)
  7.  WAITING_FIX, no fix      — holds still, stays WAITING_FIX
  8.  WAITING_FIX → NAVIGATING — valid fix acquired
  9.  WAITING_FIX min_fix_quality — low-quality fix rejected
 10.  NAVIGATING drives        — non-zero motor output toward waypoint
 11.  NAVIGATING arrival       — within radius → ARRIVED, holds still
 12.  ARRIVED advances         — pause=0 → moves to next waypoint
 13.  Single WP → COMPLETE     — lifecycle end
 14.  COMPLETE returns (0, 0)
 15.  Multi-WP sequencing      — 3 waypoints visited in order
 16.  Loop mode                — COMPLETE restarts from WAITING_FIX
 17.  Spin on spot             — |error| > spin_threshold → asymmetric output
 18.  Proportional steering    — moderate error → fwd_speed ± steer
 19.  LiDAR obstacle stop      — forward obstacle → halts
 20.  Look-ahead blending      — near WP, next WP exists → bearing blended
 21.  IMU heading priority     — IMU used in preference to GPS course
 22.  from_ini() loading       — reads GpsNavConfig values from .ini
 23.  from_ini() missing section — falls back to defaults

Usage:
    python3 tools/test_gps_navigator.py
"""

import math
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot.gps_navigator import (
    GpsNavigator, GpsNavState, GpsNavConfig,
    _haversine, _bearing, _angle_diff,
)

# ── Test harness ──────────────────────────────────────────────────────────────

_passed = 0
_failed = 0


def _check(name, condition, detail=""):
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


# ── Synthetic GPS state ───────────────────────────────────────────────────────

class _Gps:
    """Minimal mock GPS state matching the fields read by GpsNavigator.update()."""
    def __init__(self, lat=None, lon=None, fix=False, fix_quality=0,
                 heading=None, ts=0.0):
        self.latitude         = lat
        self.longitude        = lon
        self.fix              = fix
        self.fix_quality      = fix_quality
        self.fix_quality_name = ("RTK Fixed" if fix_quality == 4 else
                                 "GPS"        if fix_quality == 1 else "Invalid")
        self.heading          = heading
        self.timestamp        = ts


_ts = [0.0]  # unique timestamp counter — navigator ignores duplicate timestamps


def _gps(lat=None, lon=None, fix=True, fix_quality=1, heading=None):
    """Valid GPS fix with a unique timestamp so the navigator processes it."""
    _ts[0] += 1.0
    return _Gps(lat=lat, lon=lon, fix=fix, fix_quality=fix_quality,
                heading=heading, ts=_ts[0])


def _no_fix():
    _ts[0] += 1.0
    return _Gps(fix=False, fix_quality=0, ts=_ts[0])


# ── Synthetic LiDAR ───────────────────────────────────────────────────────────

class _FakeScan:
    def __init__(self, angles, distances):
        self.angles    = angles
        self.distances = distances


def _lidar_obstacle(dist_m=0.3, angle_deg=0.0):
    """Single obstacle dead ahead (0° = forward in the navigator's convention)."""
    return _FakeScan([float(angle_deg)], [dist_m * 1000.0])


def _lidar_clear():
    return _FakeScan([], [])


# ── Navigator factory ─────────────────────────────────────────────────────────

def _nav(arrival_radius=2.0, min_fix_quality=1, ramp_rate=1000.0,
         arrival_pause=0.0, loop=False, lookahead_m=5.0,
         spin_threshold=45.0, fwd_speed=0.5, steer_kp=0.8, steer_max=0.7):
    """Return a GpsNavigator tuned for unit tests (instant ramp, zero pause)."""
    cfg = GpsNavConfig(
        arrival_radius     = arrival_radius,
        min_fix_quality    = min_fix_quality,
        ramp_rate          = ramp_rate,
        arrival_pause      = arrival_pause,
        loop               = loop,
        lookahead_m        = lookahead_m,
        spin_threshold     = spin_threshold,
        fwd_speed          = fwd_speed,
        steer_kp           = steer_kp,
        steer_max          = steer_max,
        imu_kp             = 1.0,
        spin_speed         = 0.3,
        obstacle_stop_dist = 0.5,
        obstacle_cone_deg  = 60.0,
    )
    return GpsNavigator(cfg)


def _run(nav, gps, imu=None, yukon=None, lidar=None, n=200):
    """Call update() n times with the same inputs; return last (left, right)."""
    left = right = 0.0
    for _ in range(n):
        left, right = nav.update(gps, imu, yukon, lidar)
    return left, right


# ── Reference positions ───────────────────────────────────────────────────────

# Cambridge area base point
_LAT0 = 52.200000
_LON0 =  0.100000

# ~100 m due north  (1° lat ≈ 111,000 m)
_LAT_N = _LAT0 + 0.000900
_LON_N = _LON0

# ~100 m due east  (1° lon @ 52° ≈ 68,000 m)
_LAT_E = _LAT0
_LON_E = _LON0 + 0.001470


# ── Test sections ─────────────────────────────────────────────────────────────

def test_geometry():
    print("\n1. Geometry helpers:")

    d = _haversine(_LAT0, _LON0, _LAT_N, _LON_N)
    _check("haversine: ~100 m north ≈ 100 m (±5 m)", _approx(d, 100.0, tol=5.0),
           f"got {d:.1f} m")
    _check("haversine: same point = 0 m",
           _haversine(_LAT0, _LON0, _LAT0, _LON0) == 0.0)

    b_n = _bearing(_LAT0, _LON0, _LAT_N, _LON_N)
    _check("bearing: due north ≈ 0° or 360° (±2°)",
           _approx(b_n, 0.0, tol=2.0) or _approx(b_n, 360.0, tol=2.0),
           f"got {b_n:.1f}°")
    b_e = _bearing(_LAT0, _LON0, _LAT_E, _LON_E)
    _check("bearing: due east ≈ 90° (±2°)",  _approx(b_e, 90.0, tol=2.0),
           f"got {b_e:.1f}°")

    _check("angle_diff(10, 350) =  +20° (short CW arc)",
           _approx(_angle_diff(10.0, 350.0),  20.0, tol=0.01))
    _check("angle_diff(350, 10) = −20° (short CCW arc)",
           _approx(_angle_diff(350.0, 10.0), -20.0, tol=0.01))
    _check("angle_diff(0, 0) = 0°",
           _angle_diff(0.0, 0.0) == 0.0)
    _check("angle_diff result always in −180..+180",
           all(abs(_angle_diff(float(t), 0.0)) <= 180.0 for t in range(0, 361, 15)))


def test_initial_state():
    print("\n2. Initial state:")
    nav = _nav()
    _check("state = IDLE",           nav.state is GpsNavState.IDLE)
    _check("waypoint_index = 0",     nav.waypoint_index == 0)
    _check("current_waypoint = None",nav.current_waypoint is None)
    _check("waypoints list empty",   nav.waypoints == [])
    _check("bearing_to_wp = None",   nav.bearing_to_wp  is None)
    _check("distance_to_wp = None",  nav.distance_to_wp is None)


def test_start_no_waypoints():
    print("\n3. start() with no waypoints → ERROR:")
    nav = _nav()
    nav.start()
    _check("state = ERROR",          nav.state is GpsNavState.ERROR)
    l, r = nav.update(_no_fix())
    _check("ERROR: left = 0",        l == 0.0, f"got {l}")
    _check("ERROR: right = 0",       r == 0.0, f"got {r}")


def test_start_with_waypoints():
    print("\n4. start() with waypoints → WAITING_FIX:")
    nav = _nav()
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    _check("state = WAITING_FIX",   nav.state is GpsNavState.WAITING_FIX)
    _check("waypoint_index = 0",    nav.waypoint_index == 0)


def test_stop():
    print("\n5. stop() → IDLE:")
    nav = _nav()
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    nav.stop()
    _check("state = IDLE after stop", nav.state is GpsNavState.IDLE)


def test_idle_complete_error_zero():
    print("\n6. IDLE / COMPLETE / ERROR return (0, 0):")

    nav_idle = _nav()
    l, r = _run(nav_idle, _gps(_LAT0, _LON0))
    _check("IDLE: left = 0",  l == 0.0, f"got {l}")
    _check("IDLE: right = 0", r == 0.0, f"got {r}")

    # Reach COMPLETE via a single-waypoint route
    nav_c = _nav(arrival_pause=0.0)
    nav_c.load_waypoints([(_LAT0, _LON0)])
    nav_c.start()
    nav_c.update(_gps(_LAT0, _LON0))   # WAITING_FIX → NAVIGATING → ARRIVED (dist=0)
    nav_c.update(_gps(_LAT0, _LON0))   # ARRIVED (pause=0) → COMPLETE
    _check("COMPLETE state reached", nav_c.state is GpsNavState.COMPLETE)
    l, r = _run(nav_c, _gps(_LAT0, _LON0))
    _check("COMPLETE: left = 0",  l == 0.0, f"got {l}")
    _check("COMPLETE: right = 0", r == 0.0, f"got {r}")


def test_waiting_fix_no_fix():
    print("\n7. WAITING_FIX with no fix — holds still:")
    nav = _nav()
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    l, r = _run(nav, _no_fix())
    _check("stays WAITING_FIX",       nav.state is GpsNavState.WAITING_FIX)
    _check("left ≈ 0 (no movement)",  abs(l) < 0.01, f"got {l:.4f}")
    _check("right ≈ 0 (no movement)", abs(r) < 0.01, f"got {r:.4f}")


def test_waiting_fix_acquired():
    print("\n8. WAITING_FIX → NAVIGATING on valid fix:")
    nav = _nav()
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    nav.update(_gps(_LAT0, _LON0))
    _check("valid fix → NAVIGATING", nav.state is GpsNavState.NAVIGATING)


def test_min_fix_quality():
    print("\n9. WAITING_FIX respects min_fix_quality:")
    nav = _nav(min_fix_quality=4)   # require RTK Fixed
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()

    nav.update(_gps(_LAT0, _LON0, fix_quality=1))  # plain GPS — not enough
    _check("fix_quality=1 < 4 → stays WAITING_FIX",
           nav.state is GpsNavState.WAITING_FIX)

    nav.update(_gps(_LAT0, _LON0, fix_quality=4))  # RTK Fixed
    _check("fix_quality=4 ≥ 4 → NAVIGATING",
           nav.state is GpsNavState.NAVIGATING)


def test_navigating_drives():
    print("\n10. NAVIGATING drives toward waypoint:")
    nav = _nav()
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    l, r = _run(nav, _gps(_LAT0, _LON0))  # 100 m away — will not arrive
    _check("state = NAVIGATING",           nav.state is GpsNavState.NAVIGATING)
    _check("distance_to_wp set",           nav.distance_to_wp is not None)
    _check("distance_to_wp > arrival_radius",
           nav.distance_to_wp is not None and nav.distance_to_wp > 2.0,
           f"got {nav.distance_to_wp:.1f} m")
    _check("motors non-zero (driving)",    abs(l) > 0.01 or abs(r) > 0.01,
           f"l={l:.3f} r={r:.3f}")
    _check("both motors forward",          l > 0 and r > 0,
           f"l={l:.3f} r={r:.3f}")


def test_arrival():
    print("\n11. NAVIGATING → ARRIVED within arrival_radius:")
    nav = _nav(arrival_radius=2.0, arrival_pause=0.05)
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    # Robot at exact waypoint coordinates → distance = 0
    nav.update(_gps(_LAT_N, _LON_N))
    _check("distance=0 → ARRIVED",       nav.state is GpsNavState.ARRIVED)
    l, r = nav.update(_gps(_LAT_N, _LON_N))
    _check("ARRIVED: left ≈ 0",          abs(l) < 0.01, f"got {l:.4f}")
    _check("ARRIVED: right ≈ 0",         abs(r) < 0.01, f"got {r:.4f}")


def test_arrived_advances():
    print("\n12. ARRIVED advances after pause:")
    nav = _nav(arrival_pause=0.0)
    nav.load_waypoints([(_LAT_N, _LON_N), (_LAT_E, _LON_E)])
    nav.start()
    nav.update(_gps(_LAT_N, _LON_N))   # arrive at WP0
    _check("arrived at WP0 → ARRIVED",  nav.state is GpsNavState.ARRIVED)
    _check("waypoint_index still 0",    nav.waypoint_index == 0)
    nav.update(_gps(_LAT_N, _LON_N))   # pause=0 → advance
    _check("after pause → NAVIGATING",  nav.state is GpsNavState.NAVIGATING)
    _check("waypoint_index = 1",        nav.waypoint_index == 1)


def test_single_wp_complete():
    print("\n13. Single waypoint → COMPLETE:")
    nav = _nav(arrival_pause=0.0)
    nav.load_waypoints([(_LAT0, _LON0)])
    nav.start()
    nav.update(_gps(_LAT0, _LON0))   # → ARRIVED (dist=0)
    nav.update(_gps(_LAT0, _LON0))   # → COMPLETE
    _check("single WP → COMPLETE",  nav.state is GpsNavState.COMPLETE)


def test_complete_zero():
    print("\n14. COMPLETE returns (0, 0):")
    nav = _nav(arrival_pause=0.0)
    nav.load_waypoints([(_LAT0, _LON0)])
    nav.start()
    nav.update(_gps(_LAT0, _LON0))
    nav.update(_gps(_LAT0, _LON0))
    l, r = _run(nav, _gps(_LAT0, _LON0))
    _check("COMPLETE: left = 0",  l == 0.0, f"got {l}")
    _check("COMPLETE: right = 0", r == 0.0, f"got {r}")


def test_multi_wp_sequencing():
    print("\n15. Multi-waypoint sequencing (3 WPs visited in order):")
    nav = _nav(arrival_pause=0.0)
    nav.load_waypoints([(_LAT_N, _LON_N), (_LAT_E, _LON_E), (_LAT0, _LON0)])
    nav.start()

    for idx, (lat, lon) in enumerate([(_LAT_N, _LON_N),
                                       (_LAT_E, _LON_E),
                                       (_LAT0,  _LON0)]):
        nav.update(_gps(lat, lon))   # arrive
        nav.update(_gps(lat, lon))   # advance / complete

    _check("all 3 WPs visited → COMPLETE", nav.state is GpsNavState.COMPLETE)


def test_loop_mode():
    print("\n16. Loop mode → restarts from WAITING_FIX:")
    nav = _nav(arrival_pause=0.0, loop=True)
    nav.load_waypoints([(_LAT0, _LON0)])
    nav.start()
    nav.update(_gps(_LAT0, _LON0))
    nav.update(_gps(_LAT0, _LON0))   # loop=True → WAITING_FIX, not COMPLETE
    _check("loop: state = WAITING_FIX (not COMPLETE)",
           nav.state is GpsNavState.WAITING_FIX)
    _check("loop: waypoint_index reset to 0", nav.waypoint_index == 0)


def test_spin_on_spot():
    print("\n17. Spin on spot for large heading error:")
    nav = _nav(spin_threshold=45.0, ramp_rate=1000.0)
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    # Enter NAVIGATING and set _imu_target ≈ 0° (due north)
    nav.update(_gps(_LAT0, _LON0))

    # IMU heading = 180° → error = ±180° > spin_threshold=45°
    l, r = _run(nav, _gps(_LAT0, _LON0), imu=180.0, n=300)

    _check("spin: motors have opposite signs",
           (l > 0.01 and r < -0.01) or (l < -0.01 and r > 0.01),
           f"l={l:.3f} r={r:.3f}")
    _check("spin: magnitudes approximately equal (symmetric)",
           _approx(abs(l), abs(r), tol=0.05),
           f"|l|={abs(l):.3f} |r|={abs(r):.3f}")


def test_proportional_steering():
    print("\n18. Proportional steering for moderate heading error:")
    # Target ≈ 0° (north); IMU heading = 20° → error = −20° → steer left → right > left
    nav = _nav(spin_threshold=45.0, fwd_speed=0.5, steer_kp=0.8, steer_max=0.7,
               ramp_rate=1000.0)
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    nav.update(_gps(_LAT0, _LON0))   # set _imu_target ≈ 0°

    l, r = _run(nav, _gps(_LAT0, _LON0), imu=20.0, n=300)

    # heading_err = _angle_diff(0, 20) = −20° → steer negative
    # left = fwd − steer (larger), right = fwd + steer (smaller) → left > right
    # left > right pivots the robot counter-clockwise (toward 0°). Correct.
    _check("steering: both motors forward",                     l > 0 and r > 0,
           f"l={l:.3f} r={r:.3f}")
    _check("steering: left > right (counter-clockwise to 0°)",  l > r,
           f"l={l:.3f} r={r:.3f}")


def test_lidar_obstacle_stop():
    print("\n19. LiDAR obstacle stop:")
    nav = _nav()
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    nav.update(_gps(_LAT0, _LON0))   # enter NAVIGATING

    # Obstacle 0.3 m dead ahead — below 0.5 m threshold → halt
    l, r = _run(nav, _gps(_LAT0, _LON0), lidar=_lidar_obstacle(0.3, 0.0), n=100)
    _check("obstacle @ 0.3 m: left ≈ 0",  abs(l) < 0.05, f"got {l:.4f}")
    _check("obstacle @ 0.3 m: right ≈ 0", abs(r) < 0.05, f"got {r:.4f}")

    # Obstacle 2.0 m ahead — above threshold → should drive
    l2, r2 = _run(nav, _gps(_LAT0, _LON0), lidar=_lidar_obstacle(2.0, 0.0), n=200)
    _check("obstacle @ 2.0 m: drives (not halted)",
           abs(l2) > 0.01 or abs(r2) > 0.01, f"l={l2:.3f} r={r2:.3f}")

    # No obstacle → drives
    l3, r3 = _run(nav, _gps(_LAT0, _LON0), lidar=_lidar_clear(), n=200)
    _check("no obstacle: drives",
           abs(l3) > 0.01 or abs(r3) > 0.01, f"l={l3:.3f} r={r3:.3f}")


def test_lookahead_blending():
    print("\n20. Look-ahead blending toward next waypoint:")
    # WP0 is ~100 m north; WP1 is ~100 m east.  When the robot is within
    # lookahead_m of WP0, _imu_target is blended toward the WP0→WP1 bearing
    # (roughly south-east).  bearing_to_wp always reflects the direct bearing
    # to WP0 — the blend only affects _imu_target, which drives heading_err.
    #
    # Test: provide IMU heading = direct bearing to WP0 (≈ 0°).
    # Far away (no blend): heading_err ≈ 0° (IMU aligned with _imu_target).
    # Close in (blend active): _imu_target swings toward WP1 → heading_err grows.

    nav = _nav(lookahead_m=50.0, arrival_radius=0.1, ramp_rate=1000.0,
               spin_threshold=170.0)   # don't trigger spin so we get heading_err
    nav.load_waypoints([(_LAT_N, _LON_N), (_LAT_E, _LON_E)])
    nav.start()

    # Far position (100 m from WP0 — outside 50 m lookahead)
    nav.update(_gps(_LAT0, _LON0), imu_heading=0.0)
    err_far = nav.heading_err   # _imu_target ≈ 0° (no blend), IMU = 0° → err ≈ 0°

    # Near position (~11 m south of WP0 — inside 50 m lookahead)
    near_lat = _LAT_N - 0.000100
    nav.update(_gps(near_lat, _LON_N), imu_heading=0.0)
    err_near = nav.heading_err  # _imu_target blended toward WP0→WP1 (≠ 0°) → err > 0

    _check("look-ahead far: heading_err ≈ 0° (no blend)",
           err_far is not None and abs(err_far) < 3.0, f"got {err_far}")
    _check("look-ahead near: heading_err larger (blend toward WP1 active)",
           err_near is not None and abs(err_near) > 5.0, f"got {err_near}")


def test_imu_priority():
    print("\n21. IMU heading used in preference to GPS course:")
    nav = _nav(ramp_rate=1000.0, spin_threshold=45.0)
    nav.load_waypoints([(_LAT_N, _LON_N)])
    nav.start()
    nav.update(_gps(_LAT0, _LON0))   # set _imu_target ≈ 0° (north)

    # IMU aligned with target → small heading error
    _run(nav, _gps(_LAT0, _LON0), imu=0.0, n=100)
    err_imu = nav.heading_err

    # No IMU; GPS course = 180° (opposite direction) → large error
    _run(nav, _gps(_LAT0, _LON0, heading=180.0), imu=None, n=100)
    err_gps = nav.heading_err

    _check("with IMU aligned: heading_err ≈ 0° (±5°)",
           err_imu is not None and abs(err_imu) < 5.0, f"got {err_imu}")
    _check("without IMU: GPS course used → non-zero heading_err",
           err_gps is not None and abs(err_gps) > 10.0, f"got {err_gps}")


def test_from_ini_loading():
    print("\n22. from_ini() loading:")
    ini = """
[gps_navigator]
arrival_radius    = 3.5
min_fix_quality   = 4
fwd_speed         = 0.6
steer_kp          = 1.2
steer_max         = 0.8
arrival_pause     = 2.0
ramp_rate         = 3.0
imu_kp            = 1.5
spin_speed        = 0.4
spin_threshold    = 30.0
loop              = true
lookahead_m       = 8.0
obstacle_stop_dist = 0.75
obstacle_cone_deg  = 45.0
"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.ini', delete=False) as f:
        f.write(ini)
        path = f.name

    try:
        nav = GpsNavigator.from_ini(path)
        c   = nav.cfg
        _check("arrival_radius = 3.5",          c.arrival_radius     == 3.5)
        _check("min_fix_quality = 4",            c.min_fix_quality    == 4)
        _check("fwd_speed = 0.6",                c.fwd_speed          == 0.6)
        _check("steer_kp = 1.2",                 c.steer_kp           == 1.2)
        _check("steer_max = 0.8",                c.steer_max          == 0.8)
        _check("arrival_pause = 2.0",            c.arrival_pause      == 2.0)
        _check("loop = True",                    c.loop               is True)
        _check("lookahead_m = 8.0",              c.lookahead_m        == 8.0)
        _check("spin_threshold = 30.0",          c.spin_threshold     == 30.0)
        _check("obstacle_stop_dist = 0.75",      c.obstacle_stop_dist == 0.75)
        _check("obstacle_cone_deg = 45.0",       c.obstacle_cone_deg  == 45.0)
    finally:
        os.unlink(path)


def test_from_ini_defaults():
    print("\n23. from_ini() missing section → defaults:")
    with tempfile.NamedTemporaryFile(mode='w', suffix='.ini', delete=False) as f:
        f.write("[other]\nfoo = bar\n")
        path = f.name

    try:
        nav = GpsNavigator.from_ini(path)
        d   = GpsNavConfig()
        _check("arrival_radius default",   nav.cfg.arrival_radius  == d.arrival_radius)
        _check("fwd_speed default",         nav.cfg.fwd_speed       == d.fwd_speed)
        _check("loop default = False",      nav.cfg.loop            is False)
        _check("lookahead_m default",       nav.cfg.lookahead_m     == d.lookahead_m)
    finally:
        os.unlink(path)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 55)
    print("GPS navigator unit tests (no hardware required)")
    print("=" * 55)

    test_geometry()
    test_initial_state()
    test_start_no_waypoints()
    test_start_with_waypoints()
    test_stop()
    test_idle_complete_error_zero()
    test_waiting_fix_no_fix()
    test_waiting_fix_acquired()
    test_min_fix_quality()
    test_navigating_drives()
    test_arrival()
    test_arrived_advances()
    test_single_wp_complete()
    test_complete_zero()
    test_multi_wp_sequencing()
    test_loop_mode()
    test_spin_on_spot()
    test_proportional_steering()
    test_lidar_obstacle_stop()
    test_lookahead_blending()
    test_imu_priority()
    test_from_ini_loading()
    test_from_ini_defaults()

    print(f"\n{'=' * 55}")
    print(f"GPS navigator tests: {_passed} passed, {_failed} failed")
    print(f"{'=' * 55}")
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
