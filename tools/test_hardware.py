#!/usr/bin/env python3
"""
test_hardware.py — Real-world hardware tests for Yukon board and ArUco tags.

Runs a sequence of interactive tests that require physical hardware.
Each test pauses and asks you to confirm what you observe on the robot.

Tests
-----
  Section 1: Yukon connection
    1.1  Serial port found and opens
    1.2  Telemetry packet received (voltage, current, temps)
    1.3  Voltage in expected range (7–13 V)

  Section 2: Motors
    2.1  Left motor forward  (operator confirms wheel spins forward)
    2.2  Left motor reverse
    2.3  Right motor forward
    2.4  Right motor reverse
    2.5  Both motors forward together (robot drives straight)
    2.6  Kill command stops both motors immediately

  Section 3: IMU heading (BNO085)
    3.1  Heading response received (IMU fitted and responding)
    3.2  Heading changes when robot is rotated by hand
    3.3  Heading is stable when robot is held still (drift < 2° over 3 s)

  Section 4: ArUco tag detection (requires Pi camera + printed tags)
    4.1  Camera opens
    4.2  Tag detected when held in front of camera
    4.3  Distance estimate reasonable (within 20% of measured)
    4.4  Bearing is near zero when tag centred in frame
    4.5  Both gate posts detected when tags 0+1 both visible

Usage
-----
  python3 tools/test_hardware.py
  python3 tools/test_hardware.py --port /dev/ttyACM0
  python3 tools/test_hardware.py --skip-motors      # skip Section 2
  python3 tools/test_hardware.py --skip-camera      # skip Section 4
  python3 tools/test_hardware.py --tag-dist 0.5     # expected tag distance (m)
  python3 tools/test_hardware.py --tag-size 0.15    # physical tag size (m)
  python3 tools/test_hardware.py --calib camera_cal.npz

Operator prompts
----------------
  [ENTER]  Test passed — I saw the expected behaviour
  n        Test failed — something was wrong
  s        Skip this test
"""

import argparse
import glob
import math
import os
import sys
import time

# ── Path setup ────────────────────────────────────────────────────────────────
_REPO  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _REPO)
sys.path.insert(0, _TOOLS)

# ── Harness ───────────────────────────────────────────────────────────────────

_passed = 0
_failed = 0
_skipped = 0


def _check(name: str, condition: bool, detail: str = ""):
    global _passed, _failed
    if condition:
        print(f"  PASS  {name}")
        _passed += 1
    else:
        info = f"  — {detail}" if detail else ""
        print(f"  FAIL  {name}{info}")
        _failed += 1
    return condition


def _ask(prompt: str) -> str:
    """Prompt the operator and return 'pass', 'fail', or 'skip'."""
    global _passed, _failed, _skipped
    print(f"\n  >>> {prompt}")
    print("      [ENTER]=pass  n=fail  s=skip  : ", end="", flush=True)
    try:
        ans = input().strip().lower()
    except (KeyboardInterrupt, EOFError):
        print("\nAborted.")
        sys.exit(1)
    if ans == "n":
        _failed += 1
        print("  FAIL  (operator)")
        return "fail"
    elif ans == "s":
        _skipped += 1
        print("  SKIP")
        return "skip"
    else:
        _passed += 1
        print("  PASS  (operator confirmed)")
        return "pass"


def _section(title: str):
    print(f"\n{'─' * 55}")
    print(f"  {title}")
    print(f"{'─' * 55}")


# ── Serial helpers ────────────────────────────────────────────────────────────

def _find_port() -> str | None:
    candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    return candidates[0] if candidates else None


def _open_serial(port: str, baud: int = 115200):
    import serial
    ser = serial.Serial(port, baud, timeout=1.0, dsrdtr=False)
    time.sleep(0.2)   # let Yukon boot / settle
    ser.reset_input_buffer()
    return ser


# ── Yukon protocol (inline — no dependency on robot_daemon) ──────────────────

SYNC        = 0x7E
ACK         = 0x06
NAK         = 0x15

CMD_LED     = 1
CMD_LEFT    = 2
CMD_RIGHT   = 3
CMD_KILL    = 4
CMD_SENSOR  = 5

RESP_VOLTAGE = 0
RESP_CURRENT = 1
RESP_TEMP    = 2
RESP_TEMP_L  = 3
RESP_TEMP_R  = 4
RESP_FAULT_L = 5
RESP_FAULT_R = 6
RESP_HEADING = 7

SENSOR_SCALE = {
    RESP_VOLTAGE: 10.0,
    RESP_CURRENT: 100.0,
    RESP_TEMP:    3.0,
    RESP_TEMP_L:  3.0,
    RESP_TEMP_R:  3.0,
    RESP_FAULT_L: 1.0,
    RESP_FAULT_R: 1.0,
}


def _encode(cmd_code: int, value: int) -> bytes:
    cmd    = cmd_code + 0x20
    v_high = (value >> 4)   + 0x40
    v_low  = (value & 0x0F) + 0x50
    chk    = cmd ^ v_high ^ v_low
    return bytes([SYNC, cmd, v_high, v_low, chk])


def _speed_to_byte(speed: float) -> int:
    """Convert -1.0..+1.0 to Yukon motor byte 0-200 (100=stop)."""
    clamped = max(-1.0, min(1.0, speed))
    return int(round(clamped * 100 + 100))


def _send_cmd(ser, cmd_code: int, value: int, timeout: float = 0.5) -> bool:
    """Send a command and wait for ACK. Returns True on ACK."""
    ser.reset_input_buffer()
    ser.write(_encode(cmd_code, value))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        b = ser.read(1)
        if b:
            if b[0] == ACK:
                return True
            if b[0] == NAK:
                return False
    return False   # timeout


def _motor(ser, left: float, right: float):
    _send_cmd(ser, CMD_LEFT,  _speed_to_byte(left))
    _send_cmd(ser, CMD_RIGHT, _speed_to_byte(right))


def _kill(ser):
    _send_cmd(ser, CMD_KILL, 0)


def _request_telemetry(ser, timeout: float = 1.0) -> dict | None:
    """
    Send CMD_SENSOR and collect all RESP_* packets until ACK.
    Returns dict of {resp_id: value} or None on failure.
    """
    ser.reset_input_buffer()
    ser.write(_encode(CMD_SENSOR, 0))

    data   = {}
    buf    = bytearray()
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        raw = ser.read(64)
        if not raw:
            continue
        buf.extend(raw)

        # Scan for 5-byte response packets: [SYNC, resp_type+0x30, V_HIGH, V_LOW, CHK]
        i = 0
        while i <= len(buf) - 5:
            if buf[i] == SYNC:
                pkt = buf[i:i + 5]
                resp_raw = pkt[1]
                v_high   = pkt[2]
                v_low    = pkt[3]
                chk      = pkt[4]

                if chk == (resp_raw ^ v_high ^ v_low):
                    resp_id = resp_raw - 0x30
                    if 0 <= resp_id <= 7:
                        raw_val = ((v_high - 0x40) << 4) | (v_low - 0x50)
                        scale   = SENSOR_SCALE.get(resp_id, 1.0)
                        data[resp_id] = raw_val / scale
                        i += 5
                        continue
                    elif resp_raw == ACK:
                        return data   # end of sensor stream
            i += 1

        # Check for bare ACK byte (not framed)
        if ACK in buf:
            return data

    return None


def _read_heading(ser, samples: int = 5, interval: float = 0.25) -> list[float]:
    """
    Request telemetry 'samples' times and collect any RESP_HEADING values.
    Returns list of decoded headings (degrees).
    """
    headings = []
    for _ in range(samples):
        telem = _request_telemetry(ser)
        if telem and RESP_HEADING in telem:
            raw = telem[RESP_HEADING]
            if raw != 255.0:   # 255 = IMU absent
                headings.append(raw * (359.0 / 254.0))
        time.sleep(interval)
    return headings


# ── Section 1: Yukon connection ───────────────────────────────────────────────

def test_yukon_connection(port: str) -> object | None:
    """Returns open serial object on success, None on failure."""
    _section("Section 1: Yukon connection")

    if not _check("Serial port specified or found", port is not None,
                  "no /dev/ttyACM* found — is Yukon plugged in?"):
        return None

    try:
        ser = _open_serial(port)
        _check("Port opens without error", True)
    except Exception as e:
        _check("Port opens without error", False, str(e))
        return None

    telem = _request_telemetry(ser)
    if not _check("Telemetry packet received", telem is not None,
                  "no response — check firmware is running"):
        ser.close()
        return None

    voltage = telem.get(RESP_VOLTAGE, 0.0)
    _check(f"Voltage in range 7–13 V (got {voltage:.1f} V)",
           7.0 <= voltage <= 13.0,
           f"got {voltage:.1f} V — check battery / USB power")

    if RESP_FAULT_L in telem:
        fl = telem[RESP_FAULT_L]
        _check(f"Left motor fault = 0 (got {fl:.0f})",  fl == 0.0)
    if RESP_FAULT_R in telem:
        fr = telem[RESP_FAULT_R]
        _check(f"Right motor fault = 0 (got {fr:.0f})", fr == 0.0)

    return ser


# ── Section 2: Motors ─────────────────────────────────────────────────────────

def test_motors(ser):
    _section("Section 2: Motors  *** ROBOT WILL MOVE — clear the area ***")

    SPEED = 0.35
    HOLD  = 1.0   # seconds per direction

    tests = [
        ("2.1  Left motor FORWARD  — left wheel spins forward",
         SPEED, 0.0),
        ("2.2  Left motor REVERSE  — left wheel spins backward",
         -SPEED, 0.0),
        ("2.3  Right motor FORWARD — right wheel spins forward",
         0.0, SPEED),
        ("2.4  Right motor REVERSE — right wheel spins backward",
         0.0, -SPEED),
        ("2.5  Both FORWARD        — robot drives straight forward",
         SPEED, SPEED),
    ]

    print(f"\n  Motor speed = {SPEED:.0%}, hold = {HOLD:.1f} s each")

    for label, l, r in tests:
        print(f"\n  Running: {label}")
        _motor(ser, l, r)
        time.sleep(HOLD)
        _kill(ser)
        time.sleep(0.3)
        _ask(f"Did you see: {label}?")

    # Kill command test
    print("\n  Testing KILL command...")
    _motor(ser, SPEED, SPEED)
    time.sleep(0.5)
    ok = _send_cmd(ser, CMD_KILL, 0)
    _check("2.6  Kill command ACK'd by Yukon", ok)
    _ask("2.6  Did both motors stop immediately?")


# ── Section 3: IMU heading ────────────────────────────────────────────────────

def test_imu(ser):
    _section("Section 3: IMU heading (BNO085)")

    print("\n  Reading telemetry for IMU heading...")
    headings = _read_heading(ser, samples=5, interval=0.3)

    imu_present = len(headings) > 0
    _check("3.1  RESP_HEADING received (IMU present)", imu_present,
           "no heading packets — is BNO085 fitted on the Yukon I2C port?")

    if not imu_present:
        print("  Skipping IMU tests — no IMU.")
        return

    h0 = headings[-1]
    print(f"\n  Current heading: {h0:.1f}°")
    print("  Rotate the robot roughly 90° in either direction, then hold still.")
    input("  Press ENTER when done rotating... ")

    headings2 = _read_heading(ser, samples=3, interval=0.2)
    if headings2:
        h1 = headings2[-1]
        delta = abs((h1 - h0 + 180) % 360 - 180)
        _check(f"3.2  Heading changed after rotation (Δ={delta:.1f}°, expect > 45°)",
               delta > 45.0, f"Δ={delta:.1f}°")
    else:
        _check("3.2  Heading changed after rotation", False, "no heading received")

    print("\n  Hold the robot completely still for 3 seconds...")
    time.sleep(0.5)
    h_start = _read_heading(ser, samples=1)
    time.sleep(3.0)
    h_end   = _read_heading(ser, samples=1)

    if h_start and h_end:
        drift = abs((h_end[0] - h_start[0] + 180) % 360 - 180)
        _check(f"3.3  Heading stable at rest (drift={drift:.1f}°, expect < 2°)",
               drift < 2.0, f"drift={drift:.1f}°")
    else:
        _check("3.3  Heading stable at rest", False, "no heading readings")


# ── Section 4: ArUco tag detection ───────────────────────────────────────────

def test_aruco(calib_file: str | None, tag_size: float, expected_dist: float):
    _section("Section 4: ArUco tag detection")

    # ── Camera ──────────────────────────────────────────────────────────────
    try:
        from picamera2 import Picamera2
        use_picamera = True
    except ImportError:
        use_picamera = False

    import cv2
    import numpy as np
    from robot.aruco_detector import ArucoDetector

    calib = calib_file if (calib_file and os.path.exists(calib_file)) else None
    if calib:
        print(f"  Using calibration: {calib}")
    else:
        print("  No calibration file — distance/bearing not available.")

    det = ArucoDetector(draw=True, show_fps=True, calib_file=calib, tag_size=tag_size)

    # Open camera
    cam = None
    if use_picamera:
        try:
            from picamera2 import Picamera2
            cam = Picamera2()
            config = cam.create_video_configuration(
                main={"format": "RGB888", "size": (640, 480)})
            cam.configure(config)
            cam.start()
            time.sleep(1.0)
            _check("4.1  Camera opens (Picamera2)", True)
        except Exception as e:
            _check("4.1  Camera opens (Picamera2)", False, str(e))
            cam = None

    if cam is None:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            _check("4.1  Camera opens (OpenCV /dev/video0)", True)
        else:
            _check("4.1  Camera opens", False, "no camera found")
            print("  Skipping camera tests.")
            return
    else:
        cap = None

    def _grab_frame():
        if cam is not None:
            return cam.capture_array()
        ret, bgr = cap.read()
        if ret:
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return None

    def _show_live(seconds: float, label: str):
        """Show annotated live feed for 'seconds' in a cv2 window."""
        deadline = time.monotonic() + seconds
        last_state = None
        while time.monotonic() < deadline:
            frame = _grab_frame()
            if frame is None:
                break
            state = det.detect(frame)
            last_state = state
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            remaining = max(0, deadline - time.monotonic())
            cv2.putText(bgr, f"{label}  {remaining:.0f}s",
                        (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 60), 2)
            cv2.imshow("Hardware Test — ArUco", bgr)
            cv2.waitKey(1)
        return last_state

    # ── 4.2  Basic detection ─────────────────────────────────────────────────
    print("\n  Hold Tag 1 (or any tag) about 50 cm in front of the camera.")
    input("  Press ENTER when ready... ")
    state = _show_live(5.0, "Tag detection")
    _check("4.2  At least one tag detected",
           state is not None and len(state.tags) > 0,
           f"tags seen: {list(state.tags) if state else []}")

    # ── 4.3  Distance estimate ───────────────────────────────────────────────
    if calib and state and state.tags:
        dists = [t.distance for t in state.tags.values() if t.distance is not None]
        if dists:
            measured = dists[0]
            err_pct  = abs(measured - expected_dist) / expected_dist * 100
            _check(f"4.3  Distance ≈ {expected_dist:.2f} m "
                   f"(got {measured:.2f} m, err={err_pct:.0f}%, expect ≤ 20%)",
                   err_pct <= 20.0, f"got {measured:.2f} m")
        else:
            print("  SKIP  4.3  Distance estimate — no calibrated tag in last frame")
            global _skipped
            _skipped += 1
    else:
        print("  SKIP  4.3  Distance estimate — calibration not loaded")
        _skipped += 1

    # ── 4.4  Bearing near zero when centred ─────────────────────────────────
    print("\n  Centre a tag in the middle of the camera frame and hold still.")
    input("  Press ENTER when ready... ")
    state2 = _show_live(4.0, "Centre tag for bearing")
    if state2 and state2.tags and calib:
        bears = [t.bearing for t in state2.tags.values() if t.bearing is not None]
        if bears:
            b = bears[0]
            _check(f"4.4  Bearing near 0° when centred (got {b:+.1f}°, expect ±10°)",
                   abs(b) <= 10.0, f"got {b:+.1f}°")
        else:
            print("  SKIP  4.4  Bearing — no calibrated reading")
            _skipped += 1
    else:
        print("  SKIP  4.4  Bearing — calibration not loaded or no tag")
        _skipped += 1

    # ── 4.5  Both gate posts detected ────────────────────────────────────────
    print("\n  Now hold Tag 0 (outside post) and Tag 1 (inside post) side by side,")
    print("  both facing the camera.")
    input("  Press ENTER when ready... ")
    state3 = _show_live(6.0, "Gate posts: T0 + T1")
    _check("4.5  Both gate posts detected (tags 0+1)",
           state3 is not None and 0 in state3.tags and 1 in state3.tags,
           f"tags: {list(state3.tags) if state3 else []}")

    cv2.destroyAllWindows()
    if cam:
        cam.stop()
        cam.close()
    elif cap:
        cap.release()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="HackyRacingRobot real-hardware tests")
    parser.add_argument("--port",         default=None,
                        help="Yukon serial port (default: auto-detect)")
    parser.add_argument("--skip-motors",  action="store_true",
                        help="Skip motor movement tests")
    parser.add_argument("--skip-imu",     action="store_true",
                        help="Skip IMU heading tests")
    parser.add_argument("--skip-camera",  action="store_true",
                        help="Skip ArUco camera tests")
    parser.add_argument("--tag-dist",     type=float, default=0.5,
                        help="Expected distance to tag during detection test (m, default 0.5)")
    parser.add_argument("--tag-size",     type=float, default=0.15,
                        help="Physical ArUco tag side length in metres (default 0.15)")
    parser.add_argument("--calib",        default="camera_cal.npz",
                        help="Camera calibration file (default: camera_cal.npz)")
    args = parser.parse_args()

    print("=" * 55)
    print("  HackyRacingRobot — Hardware Tests")
    print("=" * 55)
    print()
    print("  Sections:")
    print("    1. Yukon connection & telemetry")
    if not args.skip_motors:
        print("    2. Motors  *** ROBOT WILL MOVE ***")
    if not args.skip_imu:
        print("    3. IMU heading (BNO085)")
    if not args.skip_camera:
        print("    4. ArUco tag detection")
    print()
    print("  At each manual prompt:")
    print("    [ENTER] = pass    n = fail    s = skip")
    print()
    input("  Press ENTER to begin... ")

    # ── Section 1 ─────────────────────────────────────────────────────────────
    port = args.port or _find_port()
    ser  = test_yukon_connection(port)

    # ── Section 2 ─────────────────────────────────────────────────────────────
    if ser and not args.skip_motors:
        test_motors(ser)
    elif args.skip_motors:
        print("\n  Section 2: Motors — SKIPPED")

    # ── Section 3 ─────────────────────────────────────────────────────────────
    if ser and not args.skip_imu:
        test_imu(ser)
    elif args.skip_imu:
        print("\n  Section 3: IMU — SKIPPED")

    # ── Section 4 ─────────────────────────────────────────────────────────────
    if not args.skip_camera:
        test_aruco(
            calib_file=args.calib,
            tag_size=args.tag_size,
            expected_dist=args.tag_dist,
        )
    else:
        print("\n  Section 4: ArUco — SKIPPED")

    # ── Tidy up ───────────────────────────────────────────────────────────────
    if ser:
        try:
            _kill(ser)
            ser.close()
        except Exception:
            pass

    print(f"\n{'=' * 55}")
    print(f"  Results: {_passed} passed, {_failed} failed, {_skipped} skipped")
    print(f"{'=' * 55}")

    if _failed:
        print("\n  Some tests FAILED — review output above.")
    else:
        print("\n  All tests passed.")

    return 0 if _failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
