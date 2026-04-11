# Planned Improvements

Concrete issues found by code review, ordered by effort. Pick up from here next session.

---

## Quick (15–30 min each)

### ~~1. Remove `or 0` guards for tag IDs~~ ✓ done
~~**File:** `tools/serial_telemetry_v2.py:377–380`~~
~~`state.nav_outside_tag or 0` uses Python's falsy `0` to guard against `None`, but the field is typed `int = 0` so `None` can never occur. The pattern would silently corrupt tag ID 0 if it were ever used.~~
~~**Fix:** Remove all four `or 0` guards on the nav tag ID fields.~~

### 2. Log zlib decompression failures
**File:** `robot/telemetry_proto.py` — `FrameDecoder._try_decode()`
LIDAR frames that fail CRC or zlib decompression are silently dropped with no logging. Over a noisy radio link this is the most common failure mode and is invisible.
**Fix:** Add `log.debug("LIDAR frame decompression failed: type=0x%02x len=%d", ptype, len(payload))` before returning `None`.

### 3. Validate non-empty waypoints in GPS navigator
**File:** `robot/gps_navigator.py` — `start()` method
If the waypoint JSON file is empty or missing the navigator starts with `_waypoints = []` and stays in `WAITING_FIX` forever. No log, no alarm.
**Fix:** At the top of `start()`, if `len(self._waypoints) == 0`: log an error, optionally raise, and ensure the dashboard/telemetry shows a meaningful state.

---

## Medium (1–2 hours each)

### ~~4. Transmit `next_gate` index in TYPE_NAV packet~~ ✓ done
~~**Files:** `robot/telemetry_proto.py`, `tools/serial_telemetry_v2.py`, `tools/ground_station_v2.py`~~
~~`next_outside_tag` and `next_inside_tag` are transmitted but not the actual `next_gate` index. The ground station derives it as `gate + 1`, which is wrong for non-sequential tracks (e.g. sequence `[0, 3, 1]`).~~
~~**Fix:** Add `next_gate: u8` to `_FMT_NAV_EXT` (bump to 15 bytes), pass `state.nav_next_gate` in `serial_telemetry_v2.py`, and use it in `ground_station_v2.py:handle_nav` instead of `gate + 1`.~~

### 5. NTRIP socket not guaranteed closed on exception
**File:** `gnss/ntrip.py` — `_connect_and_stream()`
The inner streaming loop has no `try/finally`, so a mid-stream exception (broken pipe, malformed header) can leave the socket open. The reconnect then opens a second socket.
**Fix:** Wrap the loop body in `try/finally: self._close_socket()`.

### 6. Add `scan_age` to LidarScan and staleness check in navigator
**Files:** `drivers/ld06.py`, `robot/aruco_navigator.py`
LD06 scans are only published when the angle wraps through 0°. If packets are lost the navigator's obstacle detection acts on a scan that may be several seconds old with no indication it's stale.
**Fix:** Add a `timestamp: float` field to `LidarScan` (set to `time.monotonic()` on publish). In `ArucoNavigator._check_obstacle()`, skip the obstacle check and log a warning if `scan.timestamp` is more than 1 second old.

### 7. Wire up `NavState.ERROR` in the ArUco navigator
**File:** `robot/aruco_navigator.py`
`NavState.ERROR` is defined but never transitioned to. Configuration failures (missing track gates, bad `track_file`) raise uncaught exceptions or silently stay in IDLE.
**Fix:** Catch configuration errors in `start()` and `update()`, transition to `NavState.ERROR`, and surface it in `RobotState.nav_state` so the ground station alarm panel can flag it.

### 8. Join recording flush thread on shutdown
**File:** `robot_daemon.py` — `Robot.stop()`
The thread that flushes final video frames on `stop_cam_recording()` is a daemon thread. If the main process exits before it finishes the `.mp4` can be corrupted.
**Fix:** Store the flush thread reference in the `_Camera` object and `join(timeout=5)` it inside `Robot.stop()` before the process exits.

---

## Harder (half day each)

### 9. Optimistic ground station state can drift if a command packet is lost
**File:** `tools/ground_station_v2.py` — `apply_cmd()`
`apply_cmd()` updates local state immediately. If the SiK packet is lost the robot never executes the command and the UI shows the wrong state until the next robot packet arrives.
**Fix:** In `apply_cmd()`, record `_optimistic_ts = time.monotonic()` for the changed field. In `get()`, revert the optimistic value if no confirming STATE packet has arrived within ~1 second. Requires tracking which fields were optimistically set and when.

### 10. Decouple CMD_RC_QUERY from the 50 Hz control loop
**File:** `robot_daemon.py` — `_control_thread()`
`CMD_RC_QUERY` triggers 15 response packets that must all drain before the next command. This blocks `_cmd_lock` for ~2–3 ms inside the 20 ms control loop deadline. On a slow serial day a single stall can cause the heartbeat or drive command to miss its slot.
**Fix:** Move RC polling to a dedicated thread at 10 Hz that writes results into a shared `_rc_channels` dict under `_cmd_lock`. The 50 Hz loop reads from the dict without touching the serial port. Needs careful testing on hardware to verify timing before and after.

---

## Notes
- Items 1–3 are safe to do in any order with no hardware needed.
- Items 4–8 each touch the protocol or threading but are self-contained.
- Items 9–10 change runtime behaviour that's currently working — test on hardware after each.
- After fixing item 4, update `docs/PROTOCOL.md` NAV packet spec (add `next_gate: u8` to the fixed header table).
