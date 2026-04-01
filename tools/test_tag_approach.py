#!/usr/bin/env python3
"""
test_tag_approach.py — Fluid tag approach tests.

Three tests that verify the robot can find a single ArUco tag and
approach it smoothly, stopping 30 cm away.

Test 1 — Direct approach
  Robot starts anywhere within ~3 m of the tag.
  Rotates slowly until the tag appears, then drives a smooth curved
  approach to the stop distance.

Test 2 — Arc from left
  Robot starts to the LEFT of the tag.
  Drives a continuous rightward arc until the tag enters the camera,
  then curves in and stops.

Test 3 — Arc from right
  Mirror of Test 2.

Fluid motion philosophy
-----------------------
  The robot is always moving — there are no stop-and-pivot phases.

  A single control law runs every tick:

    steer  = steer_kp  * (bearing / 45°)          clamped to ±1
    fwd    = fwd_speed * cos(bearing)^cos_power    slows in large turns
    left   = fwd - steer * fwd                     differential mix
    right  = fwd + steer * fwd

  cos_power controls how much forward speed is traded for turning authority.
  At cos_power=1 the robot slows naturally on large bearing errors and
  recovers speed as it straightens — like a car on a bend, not a tank pivot.

  When the tag is not visible the robot curves gently (arc) or rotates
  slowly (direct search) but never stops dead.

  Motor speeds are ramped at ramp_rate units/second to prevent jerks.

Usage
-----
  python3 tools/test_tag_approach.py              # all three tests
  python3 tools/test_tag_approach.py --test 1
  python3 tools/test_tag_approach.py --test 2
  python3 tools/test_tag_approach.py --test 3
  python3 tools/test_tag_approach.py --no-motors  # camera/detection only
  python3 tools/test_tag_approach.py --calib camera_cal.npz
  python3 tools/test_tag_approach.py --stop-dist 0.30 --fwd-speed 0.40

Tuning
------
  --fwd-speed    Top speed while approaching (default 0.40)
  --steer-kp     Steering gain: higher = faster turns, more oscillation (default 0.8)
  --cos-power    Forward-speed taper in turns: 1=gentle, 2=aggressive (default 1.0)
  --ramp-rate    Max speed change per second: lower = smoother (default 1.5)
  --arc-outer    Faster wheel speed during arc search (default 0.35)
  --arc-inner    Slower wheel speed during arc search (default 0.15)
  --search-speed Slow rotation speed for direct search (default 0.22)
"""

import argparse
import configparser
import logging
import math
import os
import signal
import socket
import sys
import time

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _REPO)

from robot_daemon import Robot, RobotMode, setup_logging

log = logging.getLogger("test_tag_approach")

# ── Emergency stop — registered as soon as robot is created ──────────────────
#
# SIGINT (Ctrl+C) and SIGTERM both call robot.estop() immediately, before any
# Python stack unwinding.  This sends CMD_KILL directly to the Yukon over
# serial — the fastest possible motor stop — then exits.
#
# Without this a KeyboardInterrupt raised inside time.sleep() would unwind
# through try/except/finally blocks before motors are stopped.

_robot_ref: Robot | None = None


def _emergency_stop(signum, frame):
    print("\n\n  *** EMERGENCY STOP ***", flush=True)
    if _robot_ref is not None:
        try:
            _robot_ref.estop()         # CMD_KILL direct to Yukon
        except Exception:
            pass
    sys.exit(1)

# ── Distance estimation fallback (no calibration) ────────────────────────────

AREA_K = 1600.0   # pixel²·m²  — tune with a ruler if needed

# ── Visualiser telemetry emitter ──────────────────────────────────────────────
#
# Broadcasts a small JSON datagram every tick so nav_visualiser.py --udp can
# display the robot's dead-reckoned position, heading, visible tags, and
# bearing error in real time — without sharing a Robot instance.
#
# Usage:
#   Terminal 1:  python3 tools/test_tag_approach.py
#   Terminal 2:  python3 tools/nav_visualiser.py --udp
#
# Set --vis-port 0 to disable emission entirely.

VIS_PORT_DEFAULT = 5005
WHEELBASE        = 0.20   # metres  (match robot geometry)
VEL_SCALE        = 0.70   # m/s at motor speed 1.0  (tune to match robot)


class VisEmitter:
    """
    Dead-reckons robot position from motor commands and broadcasts
    VisFrame JSON over UDP to nav_visualiser.py --udp.

    Thread-safe: emit() can be called from the control loop.
    """

    def __init__(self, port: int = VIS_PORT_DEFAULT,
                 host: str = "127.0.0.1"):
        self._host    = host
        self._port    = port
        self._enabled = port > 0
        self._sock    = None
        if self._enabled:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Dead-reckoned pose
        self._rx  = 0.0
        self._ry  = 0.0
        self._hdg = 0.0   # degrees, North=0 clockwise
        self._last_t = time.monotonic()

    def reset(self):
        """Call before each test to reset the dead-reckoned position."""
        self._rx = self._ry = self._hdg = 0.0
        self._last_t = time.monotonic()

    def emit(self, left: float, right: float,
             bearing: float | None, distance: float | None,
             nav_state: str, tag_id: int, tag_size: float):
        """Call once per control tick with current motor outputs and tag info."""
        now = time.monotonic()
        dt  = min(now - self._last_t, 0.1)
        self._last_t = now

        # Unicycle dead-reckoning
        v_l = left  * VEL_SCALE
        v_r = right * VEL_SCALE
        v   = (v_l + v_r) / 2.0
        w   = (v_r - v_l) / WHEELBASE
        rad = math.radians(self._hdg)
        self._rx  += v * math.sin(rad) * dt
        self._ry  += v * math.cos(rad) * dt
        self._hdg  = (self._hdg + math.degrees(w) * dt) % 360

        if not self._enabled or self._sock is None:
            return

        # Build world-coordinate tag entry if we have bearing + distance
        tags = {}
        if bearing is not None and distance is not None:
            world_ang = (self._hdg + bearing) % 360
            r2 = math.radians(world_ang)
            wx = self._rx + math.sin(r2) * distance
            wy = self._ry + math.cos(r2) * distance
            tags[str(tag_id)] = [round(wx, 3), round(wy, 3),
                                  round(distance, 3), round(bearing, 2)]

        pkt = {
            "rx":    round(self._rx,  3),
            "ry":    round(self._ry,  3),
            "hdg":   round(self._hdg, 2),
            "lspd":  round(left,  3),
            "rspd":  round(right, 3),
            "state": nav_state,
            "bear":  round(bearing, 2) if bearing is not None else None,
            "tags":  tags,
            "ts":    round(now, 3),
        }
        try:
            self._sock.sendto(
                (str(pkt).replace("'", '"')        # quick JSON-safe conversion
                         .replace("None", "null")
                         .replace("True", "true")
                         .replace("False", "false")
                         .encode()),
                (self._host, self._port),
            )
        except OSError:
            pass

    def close(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass


def _area_distance(area: float, tag_size: float) -> float:
    if area <= 0:
        return 99.0
    return math.sqrt(AREA_K * (tag_size / 0.15) ** 2 / area)


def _tag_info(aruco, tag_id: int, tag_size: float):
    """(bearing_deg, distance_m) or (None, None) if tag not visible."""
    if aruco is None:
        return None, None
    tag = aruco.tags.get(tag_id)
    if tag is None:
        return None, None
    dist = tag.distance if tag.distance is not None \
        else _area_distance(tag.area, tag_size)
    bear = tag.bearing if tag.bearing is not None \
        else (tag.center_x - 320) / 320.0 * 30.0
    return bear, dist


# ── Motor ramp ────────────────────────────────────────────────────────────────

class _Ramp:
    """Smooth first-order ramp on a single motor channel."""
    def __init__(self, rate: float):
        self._rate = rate
        self._val  = 0.0

    def step(self, target: float, dt: float) -> float:
        delta = target - self._val
        limit = self._rate * dt
        self._val += max(-limit, min(limit, delta))
        return self._val

    def reset(self, val: float = 0.0):
        self._val = val


# ── Core fluid control law ────────────────────────────────────────────────────

def _fluid_motors(bearing: float, fwd_speed: float,
                  steer_kp: float, cos_power: float):
    """
    Always-moving control: forward speed tapers with bearing error,
    steering differential keeps the robot curving toward the target.

    Returns (left, right) in -1..+1.
    """
    # Normalise bearing to ±1 over a ±45° window, clamped beyond
    steer = max(-1.0, min(1.0, steer_kp * (bearing / 45.0)))

    # Forward speed tapers as abs(bearing) grows: cos(bearing)^cos_power
    # At bearing=0° fwd=fwd_speed; at ±45° fwd≈0.71*fwd_speed (cos_power=1)
    taper = math.cos(math.radians(bearing)) ** cos_power
    fwd   = fwd_speed * max(0.0, taper)

    # Differential mix: outer wheel = fwd + steer*fwd, inner = fwd - steer*fwd
    left  = max(-1.0, min(1.0, fwd * (1.0 - steer)))
    right = max(-1.0, min(1.0, fwd * (1.0 + steer)))
    return left, right


# ── Result ────────────────────────────────────────────────────────────────────

class TestResult:
    def __init__(self, name: str):
        self.name    = name
        self.passed  = False
        self.reason  = ""
        self.metrics = {}

    def ok(self, **kw):
        self.passed = True
        self.reason = "all criteria met"
        self.metrics.update(kw)
        return self

    def fail(self, reason: str, **kw):
        self.passed = False
        self.reason = reason
        self.metrics.update(kw)
        return self

    def print_summary(self):
        ok    = self.passed
        color = "\033[32m" if ok else "\033[31m"
        reset = "\033[0m"
        tag   = "PASS" if ok else "FAIL"
        print(f"\n  {color}[{tag}]{reset}  {self.name}")
        print(f"         {self.reason}")
        for k, v in self.metrics.items():
            val = f"{v:.3f}" if isinstance(v, float) else str(v)
            print(f"           {k}: {val}")


# ── Generic run loop ──────────────────────────────────────────────────────────

def _run(robot, args, result: TestResult,
         search_left: float, search_right: float,
         emitter: VisEmitter | None = None) -> TestResult:
    """
    Single unified control loop for all three tests.

    While the tag is not visible the robot uses (search_left, search_right)
    — a slow rotation for Test 1, a gentle arc for Tests 2/3.

    Once the tag is visible the fluid control law takes over immediately;
    there is no phase switch or pause.

    Stops when distance ≤ stop_dist and confirms the final distance.
    """
    dt        = 1.0 / 20          # 20 Hz
    ramp_l    = _Ramp(args.ramp_rate)
    ramp_r    = _Ramp(args.ramp_rate)
    test_start  = time.monotonic()
    tag_found_at  = None
    stop_tolerance = args.stop_tol

    def _drive(l: float, r: float):
        l2, r2 = ramp_l.step(l, dt), ramp_r.step(r, dt)
        if not args.no_motors:
            robot.drive(l2, r2)

    def _stop():
        # Ramp down to zero over ~0.4 s, then hard kill
        for _ in range(int(0.4 / dt)):
            l2, r2 = ramp_l.step(0.0, dt), ramp_r.step(0.0, dt)
            if not args.no_motors:
                robot.drive(l2, r2)
            time.sleep(dt)
        if not args.no_motors:
            robot.estop()   # CMD_KILL — zeroes both motors on the Yukon

    try:
        while True:
            t0      = time.monotonic()
            elapsed = t0 - test_start

            if elapsed > args.run_timeout:
                _stop()
                return result.fail(f"run timeout at {elapsed:.1f} s",
                                   elapsed_s=elapsed)

            if tag_found_at is None and elapsed > args.search_timeout:
                _stop()
                return result.fail(
                    f"tag {args.tag_id} not found within "
                    f"{args.search_timeout:.0f} s",
                    elapsed_s=elapsed)

            bearing, distance = _tag_info(
                robot.get_aruco_state(), args.tag_id, args.tag_size)

            # ── Tag visible ────────────────────────────────────────────────────
            if bearing is not None:
                if tag_found_at is None:
                    tag_found_at = elapsed
                    print(f"\n    Tag found  t={elapsed:.1f}s"
                          + (f"  dist={distance:.2f}m" if distance else "")
                          + f"  bear={bearing:+.1f}°")

                # Stop condition
                if distance is not None and distance <= args.stop_dist:
                    _stop()
                    time.sleep(0.3)   # settle
                    # Fresh reading
                    aruco2 = robot.get_aruco_state()
                    _, final_dist = _tag_info(
                        aruco2, args.tag_id, args.tag_size)
                    if final_dist is None:
                        final_dist = distance
                    print(f"    Stopped    t={elapsed:.1f}s"
                          f"  final={final_dist:.3f}m")

                    # Evaluate
                    failures = []
                    lo = args.stop_dist - stop_tolerance
                    hi = args.stop_dist + stop_tolerance
                    if final_dist < lo:
                        failures.append(
                            f"overshot: {final_dist:.3f}m < {lo:.3f}m")
                    elif final_dist > hi:
                        failures.append(
                            f"too far: {final_dist:.3f}m > {hi:.3f}m")

                    metrics = dict(
                        tag_found_s  = tag_found_at,
                        final_dist_m = final_dist,
                        stop_target_m= args.stop_dist,
                        elapsed_s    = elapsed,
                    )
                    return result.fail("; ".join(failures), **metrics) \
                        if failures else result.ok(**metrics)

                # Fluid drive toward tag
                l, r = _fluid_motors(
                    bearing, args.fwd_speed, args.steer_kp, args.cos_power)
                _drive(l, r)
                if emitter:
                    emitter.emit(ramp_l._val, ramp_r._val,
                                 bearing, distance, "APPROACH",
                                 args.tag_id, args.tag_size)

                status = (f"    bearing={bearing:+5.1f}°  "
                          f"dist={distance:.2f}m  "
                          f"L={ramp_l._val:+.2f}  R={ramp_r._val:+.2f}")
                print(status, end="\r", flush=True)

            # ── Tag not visible — search motion ────────────────────────────────
            else:
                _drive(search_left, search_right)
                if emitter:
                    emitter.emit(ramp_l._val, ramp_r._val,
                                 None, None, "SEARCH",
                                 args.tag_id, args.tag_size)
                elapsed_str = f"t={elapsed:.1f}s"
                print(f"    searching  {elapsed_str}", end="\r", flush=True)

            time.sleep(max(0.0, dt - (time.monotonic() - t0)))

    except KeyboardInterrupt:
        # Should not normally reach here — SIGINT handler fires first.
        # Belt-and-braces in case signal fires between handler registration.
        if not args.no_motors:
            robot.estop()
        return result.fail("interrupted by operator",
                           tag_found_s=tag_found_at or -1.0)
    finally:
        if not args.no_motors:
            robot.estop()   # CMD_KILL — always zeroes motors on exit


# ── Three tests ───────────────────────────────────────────────────────────────

def test_direct(robot, args, emitter=None) -> TestResult:
    """Rotate slowly on the spot until tag visible, then fluid approach."""
    result = TestResult("Test 1 — Direct approach")
    if emitter:
        emitter.reset()
    return _run(robot, args, result,
                search_left=-args.search_speed,
                search_right=args.search_speed,
                emitter=emitter)


def test_arc_left(robot, args, emitter=None) -> TestResult:
    """
    Robot starts to the LEFT of the tag.
    Arc rightward (left=outer, right=inner) until tag appears.
    """
    result = TestResult("Test 2 — Arc from left")
    if emitter:
        emitter.reset()
    return _run(robot, args, result,
                search_left=args.arc_outer,
                search_right=args.arc_inner,
                emitter=emitter)


def test_arc_right(robot, args, emitter=None) -> TestResult:
    """
    Robot starts to the RIGHT of the tag.
    Arc leftward (right=outer, left=inner) until tag appears.
    """
    result = TestResult("Test 3 — Arc from right")
    if emitter:
        emitter.reset()
    return _run(robot, args, result,
                search_left=args.arc_inner,
                search_right=args.arc_outer,
                emitter=emitter)


# ── Setup instructions ────────────────────────────────────────────────────────

SETUPS = {
    1: (
        "Test 1 — Direct approach",
        [
            "Place tag ID {tag_id} anywhere within ~3 m of the robot.",
            "It can be off to one side — the robot rotates slowly to find it.",
            "Once acquired it curves in and stops {stop_cm} cm away.",
        ],
    ),
    2: (
        "Test 2 — Arc from left",
        [
            "Place the robot to the LEFT of tag ID {tag_id},",
            "facing away or side-on so the tag is off to the robot's right.",
            "The robot drives a continuous rightward arc until it sees the tag,",
            "then curves in and stops {stop_cm} cm away.",
        ],
    ),
    3: (
        "Test 3 — Arc from right",
        [
            "Place the robot to the RIGHT of tag ID {tag_id},",
            "facing away or side-on so the tag is off to the robot's left.",
            "The robot drives a continuous leftward arc until it sees the tag,",
            "then curves in and stops {stop_cm} cm away.",
        ],
    ),
}


def _print_setup(num: int, args):
    title, lines = SETUPS[num]
    subs = dict(tag_id=args.tag_id, stop_cm=int(args.stop_dist * 100))
    print(f"\n  ── {title} ──")
    for line in lines:
        print(f"    {line.format(**subs)}")


# ── Robot init ────────────────────────────────────────────────────────────────

def _make_robot(args) -> Robot:
    from robot_utils import _cfg
    cfg = configparser.ConfigParser()
    if os.path.exists(args.config):
        cfg.read(args.config)

    bool_val = lambda x: x.lower() == 'true'
    port_val = lambda x: None if x.lower() in ('auto', '') else x

    yukon_port = args.yukon_port or _cfg(cfg, 'robot', 'yukon_port', None, port_val)

    return Robot(
        yukon_port     = yukon_port,
        ibus_port      = _cfg(cfg, 'robot',  'ibus_port',   '/dev/null'),
        enable_camera  = not _cfg(cfg, 'camera', 'disabled', False, bool_val),
        cam_width      = _cfg(cfg, 'camera', 'width',       640,   int),
        cam_height     = _cfg(cfg, 'camera', 'height',      480,   int),
        cam_fps        = _cfg(cfg, 'camera', 'fps',         30,    int),
        cam_rotation   = _cfg(cfg, 'camera', 'rotation',    180,   int),
        enable_aruco   = True,
        aruco_dict     = _cfg(cfg, 'aruco',  'dict',        'DICT_4X4_1000'),
        aruco_calib    = args.calib,
        aruco_tag_size = args.tag_size,
        enable_lidar   = False,
        enable_gps     = False,
        no_motors      = args.no_motors,
    )


def _wait_ready(robot, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        s = robot.get_state()
        if s.telemetry.voltage > 0 and s.camera_ok:
            break
        time.sleep(0.2)
    s = robot.get_state()
    return s.telemetry.voltage > 0, s.camera_ok


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Fluid tag approach tests: direct, arc-left, arc-right")
    p.add_argument("--test",           type=int, choices=[1, 2, 3],
                   help="Run only test 1, 2, or 3 (default: all)")
    p.add_argument("--tag-id",         type=int,   default=1)
    p.add_argument("--stop-dist",      type=float, default=0.30,
                   help="Stop distance in metres (default 0.30)")
    p.add_argument("--stop-tol",       type=float, default=0.08,
                   help="Pass/fail tolerance in metres (default 0.08)")
    p.add_argument("--tag-size",       type=float, default=0.15,
                   help="Physical tag side length in metres (default 0.15)")
    p.add_argument("--calib",          default="camera_cal.npz")
    p.add_argument("--fwd-speed",      type=float, default=0.40,
                   help="Forward speed during approach (default 0.40)")
    p.add_argument("--steer-kp",       type=float, default=0.8,
                   help="Steering gain (default 0.8)")
    p.add_argument("--cos-power",      type=float, default=1.0,
                   help="Forward taper exponent in turns (default 1.0)")
    p.add_argument("--ramp-rate",      type=float, default=1.5,
                   help="Max speed change per second (default 1.5)")
    p.add_argument("--arc-outer",      type=float, default=0.35,
                   help="Faster wheel during arc search (default 0.35)")
    p.add_argument("--arc-inner",      type=float, default=0.15,
                   help="Slower wheel during arc search (default 0.15)")
    p.add_argument("--search-speed",   type=float, default=0.22,
                   help="Rotation speed for direct search (default 0.22)")
    p.add_argument("--search-timeout", type=float, default=15.0)
    p.add_argument("--run-timeout",    type=float, default=60.0)
    p.add_argument("--no-motors",      action="store_true")
    p.add_argument("--vis-port",       type=int, default=VIS_PORT_DEFAULT,
                   help=f"UDP port for nav_visualiser (0=off, default {VIS_PORT_DEFAULT})")
    p.add_argument("--vis-host",       default="127.0.0.1",
                   help="UDP host for nav_visualiser (default 127.0.0.1)")
    p.add_argument("--yukon-port",     default=None,
                   help="Override Yukon serial port (e.g. /dev/ttyACM0)")
    p.add_argument("--config",         default="robot.ini")
    p.add_argument("--log-level",      default="WARNING")
    args = p.parse_args()

    setup_logging(level=args.log_level)

    tests_to_run = [args.test] if args.test else [1, 2, 3]

    print("=" * 55)
    print("  HackyRacingRobot — Fluid Tag Approach Tests")
    print("=" * 55)
    if args.no_motors:
        print("  *** --no-motors: drive commands suppressed ***")
    print(f"  Tag ID      : {args.tag_id}")
    print(f"  Stop dist   : {args.stop_dist*100:.0f} cm  (±{args.stop_tol*100:.0f} cm)")
    print(f"  Fwd speed   : {args.fwd_speed:.2f}  steer_kp={args.steer_kp:.2f}"
          f"  cos_power={args.cos_power:.1f}  ramp={args.ramp_rate:.1f}/s")
    print(f"  Arc speeds  : outer={args.arc_outer:.2f}  inner={args.arc_inner:.2f}")

    robot = _make_robot(args)

    # Register emergency stop as early as possible so Ctrl+C always kills motors
    global _robot_ref
    _robot_ref = robot
    signal.signal(signal.SIGINT,  _emergency_stop)
    signal.signal(signal.SIGTERM, _emergency_stop)

    robot.start()

    print("\n  Starting subsystems...", end="", flush=True)
    yukon_ok, camera_ok = _wait_ready(robot)
    print(f"  Yukon={'OK' if yukon_ok else 'MISSING'}"
          f"  Camera={'OK' if camera_ok else 'MISSING'}")

    if not camera_ok:
        print("\n  ERROR: camera not running.")
        robot.stop()
        sys.exit(1)

    if not yukon_ok and not args.no_motors:
        print("\n  ERROR: Yukon not responding. Use --no-motors for dry run.")
        robot.stop()
        sys.exit(1)

    robot.set_mode(RobotMode.AUTO)

    # Create visualiser emitter (no-op if --vis-port 0)
    emitter = VisEmitter(port=args.vis_port, host=args.vis_host)
    if args.vis_port > 0:
        print(f"  Visualiser : broadcasting to {args.vis_host}:{args.vis_port}")
        print(f"               run:  python3 tools/nav_visualiser.py --udp")
    else:
        print("  Visualiser : disabled (--vis-port 0)")

    results = []

    try:
        for num in tests_to_run:
            _print_setup(num, args)
            print()
            try:
                input("  Press ENTER when ready (Ctrl+C to skip)... ")
            except KeyboardInterrupt:
                print("\n  Skipped.")
                continue
            print()

            if num == 1:
                r = test_direct(robot, args, emitter=emitter)
            elif num == 2:
                r = test_arc_left(robot, args, emitter=emitter)
            else:
                r = test_arc_right(robot, args, emitter=emitter)

            r.print_summary()
            results.append(r)

            if num != tests_to_run[-1]:
                print()
                try:
                    input("  Press ENTER for next test... ")
                except KeyboardInterrupt:
                    print("\n  Stopping.")
                    break

    finally:
        emitter.close()
        if not args.no_motors:
            robot.estop()
        robot.set_mode(RobotMode.MANUAL)
        robot.stop()

    passed  = sum(1 for r in results if r.passed)
    failed  = sum(1 for r in results if not r.passed)
    skipped = len(tests_to_run) - len(results)

    print(f"\n{'=' * 55}")
    print(f"  Results: {passed} passed  {failed} failed  {skipped} skipped")
    print(f"{'=' * 55}\n")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
