#!/usr/bin/env python3
"""
test_robot.py — Integration tests for robot_daemon.py using the Yukon simulator.

Uses a PTY (pseudo-terminal) to connect Robot to yukon_sim without real
hardware.  Camera, LiDAR and GPS are all disabled so only the Yukon serial
link and its subsystem threads are exercised.

Tests:
  1.  Lifecycle      — start() connects to PTY; stop() shuts down cleanly
  2.  get_state()    — returns RobotState with correct types and defaults
  3.  Drive commands — drive() sends correct wire bytes to the simulator
  4.  Kill           — kill() zeros both motor bytes in the simulator
  5.  LED            — set_led_a / set_led_b update simulator state
  6.  Telemetry      — 1 Hz thread populates voltage, current, temps correctly
  7.  IMU heading    — heading decoded from RESP_HEADING matches sim value
  8.  Estop          — estop() does not raise; mode becomes ESTOP
  9.  Reset ESTOP    — reset_estop() returns mode to MANUAL
 10.  AUTO mode      — mode transitions, drive() clamping
 11.  Data logging   — start/stop creates valid JSONL with expected fields
 12.  Bearing hold   — CMD_BEARING sets sim target; sim drifts to target heading
 13.  RC SC dlog     — CH9 high starts data logging; low stops it
 14.  RC SD pause    — CH10 mid/high activates no-motors; edge-triggered not level
 15.  RC SH ESTOP    — rising edge on CH12 resets ESTOP; no-op in MANUAL

Usage:
    python3 tools/test_robot.py
"""

import os
import sys
import pty
import tty
import time
import threading

_REPO  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _REPO)
sys.path.insert(0, _TOOLS)

from robot_daemon import Robot, RobotMode
from yukon_sim import (yukon_server, _state, _lock,
                        SIM_VOLTAGE, SIM_CURRENT, SIM_TEMP,
                        RC_MANUAL, RC_AUTO)


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


# ── PTY / simulator helpers ───────────────────────────────────────────────────

_SIM_HEADING = 45.0   # degrees — what the sim reports as current IMU heading


def _start_sim():
    """Open a PTY, reset sim state, start yukon_server thread.

    Returns (master_fd, slave_fd, slave_path).
    """
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    slave_path = os.ttyname(slave_fd)

    with _lock:
        _state['left_byte']      = None
        _state['right_byte']     = None
        _state['led_a']          = False
        _state['led_b']          = False
        _state['cmds_rx']        = 0
        _state['running']        = True
        _state['rc_mode']        = RC_MANUAL
        _state['rc_channels']    = [1500] * 14
        _state['rc_valid']       = True
        _state['imu_present']    = True
        _state['imu_heading']    = _SIM_HEADING
        _state['bearing_target'] = None
        _state['last_imu_tick']  = time.monotonic()

    t = threading.Thread(target=yukon_server, args=(master_fd,), daemon=True)
    t.start()
    return master_fd, slave_fd, slave_path


def _stop_sim(master_fd, slave_fd):
    with _lock:
        _state['running'] = False
    for fd in (master_fd, slave_fd):
        try:
            os.close(fd)
        except OSError:
            pass


def _make_robot(yukon_port):
    return Robot(
        yukon_port      = yukon_port,
        ibus_port       = '/tmp/no_ibus_for_tests',  # rc_thread logs warning + retries
        enable_camera   = False,
        enable_lidar    = False,
        enable_gps      = False,
        no_motors       = False,
        dlog_ch         = 9,   # SC
        rec_ch          = 11,  # SG
        pause_ch        = 10,  # SD
        gps_bookmark_ch = 12,  # SH
    )


def _set_rc(ch_1based, value):
    """Set a single RC channel (1-based) in the simulator."""
    with _lock:
        _state['rc_channels'][ch_1based - 1] = value


def _wait_rc_poll():
    """Wait long enough for two RC poll cycles (RC is queried at 10 Hz)."""
    time.sleep(0.25)


# ── Individual test sections ──────────────────────────────────────────────────

def test_lifecycle(robot):
    print("\nLifecycle:")
    _check("robot object created",   robot is not None)
    _check("initial mode is MANUAL", robot.get_state().mode is RobotMode.MANUAL)


def test_get_state(robot):
    print("\nget_state():")
    state = robot.get_state()
    _check("returns RobotState",        state is not None)
    _check("drive.left is float",       isinstance(state.drive.left, float))
    _check("drive.right is float",      isinstance(state.drive.right, float))
    _check("initial left speed = 0",    state.drive.left  == 0.0)
    _check("initial right speed = 0",   state.drive.right == 0.0)
    _check("nav_state is str",          isinstance(state.nav_state, str))
    _check("nav_gate is int",           isinstance(state.nav_gate, int))
    _check("speed_scale > 0",           state.speed_scale > 0.0)
    _check("camera_ok is bool",          isinstance(state.camera_ok, bool))
    _check("lidar_ok = False",          state.lidar_ok  is False)
    _check("gps_ok = False",            state.gps_ok    is False)


def test_drive(robot):
    print("\nDrive commands:")

    # Drive commands are only applied by the sim in AUTO mode (matches firmware behaviour).
    # Put the robot in AUTO and use robot.drive() so the control thread sends the values.
    # Note: control thread applies speed_scale, so we test sign/symmetry not exact bytes.
    robot.set_mode(RobotMode.AUTO)
    robot.drive(0.5, -0.5)
    time.sleep(0.15)   # wait for control thread (50 Hz) to deliver to sim

    with _lock:
        lb = _state['left_byte']
        rb = _state['right_byte']

    # left = forward (0–99), right = reverse (101–200), equal magnitudes
    _check("drive(0.5, -0.5): left byte forward (0<lb<100)",   0 < lb < 100,    f"got {lb}")
    _check("drive(0.5, -0.5): right byte reverse (100<rb≤200)", 100 < rb <= 200, f"got {rb}")
    _check("drive(0.5, -0.5): symmetric magnitudes (lb == rb-100)", lb == rb - 100, f"lb={lb} rb={rb}")

    # drive(0, 0): trigger another AUTO pulse to send zero bytes
    robot.set_mode(RobotMode.AUTO)
    robot.drive(0.0, 0.0)
    time.sleep(0.15)
    with _lock:
        lb = _state['left_byte']
        rb = _state['right_byte']
    _check("drive(0, 0): left byte = 0",   lb == 0, f"got {lb}")
    _check("drive(0, 0): right byte = 0",  rb == 0, f"got {rb}")

    robot.set_mode(RobotMode.MANUAL)   # restore
    time.sleep(0.05)


def test_kill(robot):
    print("\nKill:")
    robot._yukon.drive(0.75, 0.75)
    time.sleep(0.1)
    robot._yukon.kill()
    time.sleep(0.1)

    with _lock:
        lb = _state['left_byte']
        rb = _state['right_byte']
    _check("kill: left byte = 0",   lb == 0, f"got {lb}")
    _check("kill: right byte = 0",  rb == 0, f"got {rb}")


def test_led(robot):
    print("\nLED:")
    robot._yukon.set_led_a(True)
    time.sleep(0.1)
    with _lock:
        led_a = _state['led_a']
    _check("set_led_a(True): led_a = True",  led_a is True)

    robot._yukon.set_led_b(True)
    time.sleep(0.1)
    with _lock:
        led_b = _state['led_b']
    _check("set_led_b(True): led_b = True",  led_b is True)

    robot._yukon.set_led_a(False)
    time.sleep(0.1)
    with _lock:
        led_a = _state['led_a']
    _check("set_led_a(False): led_a = False", led_a is False)


def test_telemetry(robot):
    print("\nTelemetry (waiting up to 2 s for 1 Hz thread):")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        t = robot.get_state().telemetry
        if t.voltage > 0:
            break
        time.sleep(0.1)

    t = robot.get_state().telemetry
    _check(f"voltage ≈ {SIM_VOLTAGE} V",
           _approx(t.voltage, SIM_VOLTAGE, tol=0.5),
           f"got {t.voltage}")
    _check(f"current ≈ {SIM_CURRENT} A",
           _approx(t.current, SIM_CURRENT, tol=0.1),
           f"got {t.current}")
    _check(f"board_temp ≈ {SIM_TEMP} °C",
           _approx(t.board_temp, SIM_TEMP, tol=2.0),
           f"got {t.board_temp}")
    _check("left_fault = False",  t.left_fault  is False)
    _check("right_fault = False", t.right_fault is False)
    _check("fl_fault = False",    t.fl_fault is False)
    _check("fr_fault = False",    t.fr_fault is False)
    _check("rl_fault = False",    t.rl_fault is False)
    _check("rr_fault = False",    t.rr_fault is False)
    _check("fl_temp present",     t.fl_temp >= 0.0)
    _check("fr_temp present",     t.fr_temp >= 0.0)
    _check("telemetry timestamp set", t.timestamp > 0)


_SIM_PITCH =   0.0   # degrees — sim default pitch
_SIM_ROLL  =   0.0   # degrees — sim default roll


def test_imu_heading(robot):
    print("\nIMU heading / pitch / roll:")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        hdg = robot.get_heading()
        if hdg is not None:
            break
        time.sleep(0.1)

    hdg = robot.get_heading()
    # Heading is encoded/decoded with ~1.4° resolution; allow 3° tolerance
    _check(f"heading ≈ {_SIM_HEADING}° (±3°)",
           hdg is not None and _approx(hdg, _SIM_HEADING, tol=3.0),
           f"got {hdg}")

    state = robot.get_state()
    t = state.telemetry
    # Pitch: ~0.7° resolution; sim default 0.0°; allow 2° tolerance
    _check(f"pitch ≈ {_SIM_PITCH}° (±2°)",
           t.pitch is not None and _approx(t.pitch, _SIM_PITCH, tol=2.0),
           f"got {t.pitch}")
    # Roll: ~1.4° resolution; sim default 0.0°; allow 2° tolerance
    _check(f"roll ≈ {_SIM_ROLL}° (±2°)",
           t.roll is not None and _approx(t.roll, _SIM_ROLL, tol=2.0),
           f"got {t.roll}")

    _check("nav_bearing_err is None (navigator not running)",
           state.nav_bearing_err is None)


def test_estop(robot):
    print("\nEstop:")
    try:
        robot.estop()
        ok = True
    except Exception as e:
        ok = False
    _check("estop() does not raise", ok)
    _check("mode after estop is ESTOP",
           robot.get_state().mode is RobotMode.ESTOP)


def test_reset_estop(robot):
    print("\nReset ESTOP:")
    robot.estop()
    _check("mode is ESTOP before reset",
           robot.get_state().mode is RobotMode.ESTOP)
    robot.reset_estop()
    _check("mode returns to MANUAL after reset_estop()",
           robot.get_state().mode is RobotMode.MANUAL)


def test_auto_mode(robot):
    print("\nAUTO mode:")
    robot.reset_estop()   # ensure clean MANUAL state

    robot.set_mode(RobotMode.AUTO)
    _check("set_mode(AUTO) → mode is AUTO",
           robot.get_state().mode is RobotMode.AUTO)

    robot.set_mode(RobotMode.MANUAL)
    _check("set_mode(MANUAL) → mode is MANUAL",
           robot.get_state().mode is RobotMode.MANUAL)

    raised = False
    try:
        robot.set_mode(RobotMode.ESTOP)
    except ValueError:
        raised = True
    _check("set_mode(ESTOP) raises ValueError", raised)

    # drive() stores values regardless of mode (applied by control thread in AUTO)
    robot.set_mode(RobotMode.AUTO)
    robot.drive(0.6, -0.4)
    with robot._mode_lock:
        al = robot._auto_left
        ar = robot._auto_right
    _check("drive(0.6, -0.4): _auto_left stored correctly",
           _approx(al, 0.6), f"got {al}")
    _check("drive(0.6, -0.4): _auto_right stored correctly",
           _approx(ar, -0.4), f"got {ar}")

    # Clamp check
    robot.drive(2.0, -2.0)
    with robot._mode_lock:
        al = robot._auto_left
        ar = robot._auto_right
    _check("drive(2.0, …) clamped to 1.0",  _approx(al,  1.0), f"got {al}")
    _check("drive(…, -2.0) clamped to -1.0", _approx(ar, -1.0), f"got {ar}")

    robot.set_mode(RobotMode.MANUAL)


def test_data_log(robot):
    import tempfile, os
    print("\nData logging:")

    _check("not logging before start", not robot.is_data_logging())

    with tempfile.NamedTemporaryFile(suffix='.jsonl', delete=False) as f:
        tmp_path = f.name

    try:
        started = robot.start_data_log(path=tmp_path, hz=5.0)
        _check("start_data_log() returns True", started is True)
        _check("is_data_logging() True while active", robot.is_data_logging())

        # Second call while active should return False
        started2 = robot.start_data_log(path=tmp_path, hz=5.0)
        _check("start_data_log() returns False when already active",
               started2 is False)

        time.sleep(0.5)   # let at least 2 records be written

        returned_path = robot.stop_data_log()
        _check("stop_data_log() returns the log path",
               returned_path == tmp_path)
        _check("is_data_logging() False after stop", not robot.is_data_logging())

        size = os.path.getsize(tmp_path)
        _check("log file has content (> 0 bytes)", size > 0, f"size={size}")

        with open(tmp_path) as fh:
            import json
            lines = [l.strip() for l in fh if l.strip()]
        _check("log file has at least 1 record", len(lines) >= 1,
               f"got {len(lines)}")
        if lines:
            rec = json.loads(lines[0])
            _check("record has 'ts' field",   'ts'   in rec)
            _check("record has 'mode' field", 'mode' in rec)
            _check("record has 'drive' field",'drive' in rec)
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass


def test_bearing_hold(robot):
    print("\nBearing hold (CMD_BEARING):")

    target = 135.0

    # Reset sim heading to 0° so we can observe drift toward target
    with _lock:
        _state['imu_heading']    = 0.0
        _state['bearing_target'] = None
        _state['last_imu_tick']  = time.monotonic()

    robot._yukon.set_bearing(target)
    time.sleep(0.15)   # let command reach simulator

    with _lock:
        sim_target = _state['bearing_target']

    _check("set_bearing() → sim bearing_target set",
           sim_target is not None, f"got {sim_target!r}")
    # CMD_BEARING is encoded 0–254 → 0–359° with ~1.4° resolution
    _check(f"sim bearing_target ≈ {target}° (±3°)",
           sim_target is not None and _approx(sim_target, target, tol=3.0),
           f"got {sim_target}")

    # Sim drifts at 90°/s; starting from 0° it should reach ~135° in ≤2 s
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        hdg = robot.get_heading()
        if hdg is not None and _approx(hdg, target, tol=10.0):
            break
        time.sleep(0.1)

    hdg = robot.get_heading()
    _check(f"heading converges to {target}° after bearing hold (±10°)",
           hdg is not None and _approx(hdg, target, tol=10.0),
           f"got {hdg}")

    # Clear bearing
    robot._yukon.clear_bearing()
    time.sleep(0.15)

    with _lock:
        cleared = _state['bearing_target']
    _check("clear_bearing() → sim bearing_target = None",
           cleared is None, f"got {cleared!r}")


def test_rc_sc_dlog(robot):
    """SC (CH9): data logging toggled by RC switch."""
    print("\nRC SC — dlog switch:")

    # Ensure switch starts LOW (off) so first-poll baseline is off
    _set_rc(9, 1000)
    _wait_rc_poll()
    _check("dlog off when SC low", not robot.is_data_logging())

    # Move SC to HIGH → should start data logging
    _set_rc(9, 2000)
    _wait_rc_poll()
    _check("dlog on when SC high", robot.is_data_logging())

    # Move SC back to LOW → should stop data logging
    _set_rc(9, 1000)
    _wait_rc_poll()
    _check("dlog off when SC returns low", not robot.is_data_logging())


def test_rc_sd_pause(robot):
    """SD (CH10): no-motors edge-triggered; startup does not activate it."""
    print("\nRC SD — no-motors switch:")

    # Start with switch HIGH — after first poll the baseline is synced without acting
    _set_rc(10, 2000)
    _wait_rc_poll()
    _check("no_motors not set at startup even when SD high",
           not robot.get_state().no_motors)

    # Drop to LOW so baseline becomes 0
    _set_rc(10, 1000)
    _wait_rc_poll()
    _check("no_motors still off after dropping to low", not robot.get_state().no_motors)

    # Move to MID (1500 > 1333) → transition 0→1 should activate no-motors
    _set_rc(10, 1500)
    _wait_rc_poll()
    _check("no_motors on when SD moves to mid (1500)", robot.get_state().no_motors)

    # Move to LOW → should deactivate no-motors
    _set_rc(10, 1000)
    _wait_rc_poll()
    _check("no_motors off when SD returns low", not robot.get_state().no_motors)

    # Dashboard toggle while SD is low — must not be overridden by next RC poll
    robot.set_no_motors(True)
    _wait_rc_poll()
    _check("dashboard no_motors toggle not overridden by SD=low",
           robot.get_state().no_motors)
    robot.set_no_motors(False)  # restore


def test_rc_sh_estop_reset(robot):
    """SH (CH12): rising edge resets ESTOP; no effect in MANUAL."""
    print("\nRC SH — ESTOP reset via switch:")

    # Initialise SH to LOW so _prev_bookmark_ch is established
    _set_rc(12, 1000)
    _wait_rc_poll()

    # Trigger ESTOP via API
    robot.estop()
    _check("mode is ESTOP after estop()", robot.get_state().mode is RobotMode.ESTOP)

    # Rising edge on SH → should clear ESTOP
    _set_rc(12, 2000)
    _wait_rc_poll()
    _check("mode returns to MANUAL after SH rising edge in ESTOP",
           robot.get_state().mode is RobotMode.MANUAL)

    # Reset SH to LOW then rise again in MANUAL — should NOT change mode
    _set_rc(12, 1000)
    _wait_rc_poll()
    _set_rc(12, 2000)
    _wait_rc_poll()
    _check("SH rising edge in MANUAL does not change mode",
           robot.get_state().mode is RobotMode.MANUAL)

    # Leave SH low for subsequent tests
    _set_rc(12, 1000)
    _wait_rc_poll()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 50)
    print("robot.py integration tests (Yukon simulator)")
    print("=" * 50)

    master_fd, slave_fd, slave_path = _start_sim()
    print(f"\nSim PTY: {slave_path}")

    robot = _make_robot(slave_path)
    try:
        robot.start()
        time.sleep(0.3)   # let subsystem threads settle

        test_lifecycle(robot)
        test_get_state(robot)
        test_drive(robot)
        test_kill(robot)
        test_led(robot)
        test_telemetry(robot)
        test_imu_heading(robot)
        test_estop(robot)
        test_reset_estop(robot)
        test_auto_mode(robot)
        test_data_log(robot)
        test_bearing_hold(robot)
        test_rc_sc_dlog(robot)
        test_rc_sd_pause(robot)
        test_rc_sh_estop_reset(robot)

    finally:
        robot.stop()
        _stop_sim(master_fd, slave_fd)

    print(f"\n{'=' * 50}")
    print(f"Integration tests: {_passed} passed, {_failed} failed")
    print(f"{'=' * 50}")
    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
