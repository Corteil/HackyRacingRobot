# HackyRacingRobot — To-Do List

---

## For the AI Assistant

Suggested improvements based on codebase analysis, roughly ordered by priority.

### Code & Architecture
- [ ] Commit the currently uncommitted changes (README.md, robot.ini, robot_daemon.py, robot_web.py, robot_mobile.py, camera_*.py, gnss/*.py, yukon firmware) — git status shows a large batch of unstaged modifications
- [ ] Decide whether `camera_cal.npz` should be committed (large binary, 9.1 MB) or added to `.gitignore`
- [ ] `camera_monitor.py` and `camera_web.py` share a large amount of logic (exposure control, ArUco toggle, resolution switching) — extract shared code into a `robot/camera_controls.py` module to reduce duplication
- [ ] NTRIP credentials in `robot.ini` are stored in plaintext — add support for reading them from environment variables or a separate `secrets.ini` (already in `.gitignore`) with a fallback

### Testing
- [ ] Add unit tests for the `gnss/` package — currently has no dedicated test file unlike every other subsystem
- [ ] Expand `tools/test_robot.py` integration tests to cover AUTO mode and navigator state transitions using the PTY simulator
- [ ] Add a `--dry-run` / `--no-hardware` flag to `tools/test_gps.py` so it can run in CI without a physical receiver

### Navigation & Autonomy
- [ ] LiDAR data (`LidarScan`) is collected but not used by either navigator — add basic obstacle-stop logic to `aruco_navigator.py` and `gps_navigator.py` (halt if any point inside a configurable forward cone is closer than N metres)
- [ ] `gps_navigator.py` drives straight between waypoints — add a simple look-ahead / smoothing step to reduce oscillation on longer straights
- [ ] The ArUco gate navigator has no recovery behaviour when it loses sight of the target gate — add a slow rotation search state

### Documentation
- [ ] `docs/ARCHITECTURE.md` and `docs/SETUP.md` may be out of date following the rename from Yukon to HackyRacingRobot — review and update device paths, file names, and wiring notes
- [ ] Add a `docs/CALIBRATION.md` walk-through covering the full camera calibration → ArUco detection pipeline

### Minor / Quality-of-Life
- [ ] Video recordings have no maximum duration or size cap — long runs could fill the SD card; add a rolling max-minutes config option to `[output]`
- [ ] `robot_web.py` and `robot_mobile.py` both start their own `Robot` instance; if both are launched together they fight over serial ports — document clearly that only one frontend should run at a time, or add a shared-daemon IPC layer
- [ ] `tools/gps_route_builder.py` saves routes as plain JSON with no schema version field — add a `"version": 1` key now before the format evolves

---

## For the Owner

_Add your own tasks below._

- [ ]
- [ ]
- [ ]
- [ ]
- [ ]
