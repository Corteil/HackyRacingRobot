# Hacky Racing Robot Tools & Tests

Helper scripts, calibration tools, simulators, and host-side tests.
All scripts run on the Raspberry Pi host unless noted otherwise.
Run from the repo root.

---

## Tests

### test_main.py

Host-side tests for the Yukon `main.py` serial protocol.
Connects over USB serial and exercises motor control, LED, kill, and sensor commands.

```
python3 tools/test_main.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | auto-detect | Serial port (e.g. `/dev/ttyACM0`) |
| `--baud N` | `115200` | Baud rate |
| `--dry-run` | off | Run protocol encoder tests only — no hardware required |
| `--ramp` | off | Run 30-second motor ramp test instead of the standard suite |

**Examples**
```
python3 tools/test_main.py --dry-run
python3 tools/test_main.py --port /dev/ttyACM0
python3 tools/test_main.py --port /dev/ttyACM0 --ramp
```

---

### test_ibus.py

Unit tests and live channel display for the iBUS RC receiver via Pi UART.
Shows a live bar graph of all 14 channels, updating in-place.

> **Note:** The main robot stack no longer reads iBUS on the Pi — RC input is handled by Yukon GP26. `test_ibus.py` is useful when testing `rc_drive.py` or `drivers/ibus.py` standalone (requires the `uart3-pi5` overlay and iBUS wired to GPIO 9).

```
python3 tools/test_ibus.py [PORT]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `PORT` (positional) | `/dev/ttyAMA3` | Serial port for the iBUS receiver |

Press `Ctrl+C` to exit the live display.

**Examples**
```
python3 tools/test_ibus.py
python3 tools/test_ibus.py /dev/ttyAMA3
```

---

### test_ld06.py

Unit tests and live display for the LD06 LiDAR driver.
Shows 16 compass sectors (22.5° each) as a bar graph of minimum distance per sector.

```
python3 tools/test_ld06.py [PORT]
python3 tools/test_ld06.py -u
```

| Argument | Default | Description |
|----------|---------|-------------|
| `PORT` (positional) | `/dev/ttyAMA0` | Serial port for the LD06 |
| `-u` | off | Run unit tests only — no hardware required |

Press `Ctrl+C` to exit the live display.

**Examples**
```
python3 tools/test_ld06.py -u
python3 tools/test_ld06.py /dev/ttyAMA0
```

---

### test_gps.py

TAU1308 RTK GNSS rover with a pygame GUI.
Settings are read from `tools/config.ini` if it exists; CLI flags override them.
Create `tools/config.ini` from the `[gnss]` and `[ntrip]` sections of `robot.ini` as a starting point, or use `--no-config` to skip module configuration.

```
python3 tools/test_gps.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--config PATH` | `tools/config.ini` (optional) | Path to config file |
| `--port PORT` | from config | Serial port for the TAU1308 |
| `--baud N` | from config | Baud rate |
| `--host HOST` | from config | NTRIP caster hostname |
| `--ntrip-port N` | from config | NTRIP caster port |
| `--mount MOUNT` | from config | NTRIP mountpoint |
| `--user USER` | from config | NTRIP username |
| `--password PASS` | from config | NTRIP password |
| `--lat N` | from config | Approximate latitude (decimal degrees) |
| `--lon N` | from config | Approximate longitude (decimal degrees) |
| `--height N` | from config | Approximate height above ellipsoid (m) |
| `--hz N` | from config | GNSS update rate in Hz |
| `--no-config` | off | Skip module configuration on startup |
| `--debug` | off | Print raw NMEA / RTCM output |
| `--dry-run` | off | NMEA parsing unit tests only — no hardware or display required |

**Examples**
```
python3 tools/test_gps.py --dry-run
python3 tools/test_gps.py
python3 tools/test_gps.py --port /dev/ttyUSB0 --debug
python3 tools/test_gps.py --no-config --hz 5
```

---

### test_gnss.py

Unit tests for the `gnss/` package — no hardware required.
Tests NMEA parsing, coordinate conversions, checksum functions, binary helpers, and GNSSBase sentence handling.

```
python3 tools/test_gnss.py
```

Tests covered:

| # | Section | What is checked |
|---|---------|-----------------|
| 1 | Checksum | `_nmea_checksum` XOR correctness; `_verify_nmea_checksum` good/bad/missing markers |
| 2 | Coordinates | `_parse_lat` / `_parse_lon` N/S/E/W sign, empty and non-numeric inputs |
| 3 | Numeric helpers | `_safe_float` / `_safe_int` valid, empty, and non-numeric inputs |
| 4 | Binary helpers | `_pack_u8/u16/u32` little-endian; `_fletcher8` checksum |
| 5 | Initial state | `GNSSBase(None)` default fields all None/zero/False |
| 6 | GGA parsing | Lat, lon, fix quality, satellites, HDOP, altitude, `has_fix`, `position`, `stats` |
| 7 | RMC parsing | `rmc_valid`, `speed_knots`, `speed_ms` conversion, `course` |
| 8 | GST parsing | `std_lat`, `std_lon` field positions; `h_error_m` calculation |
| 9 | RTK fix | Quality=4 → `has_rtk_fixed`, `fix_quality_name` = "RTK Fixed" |
| 10 | Bad sentences | Unknown type, missing `$`, empty string all return False |
| 11 | NTRIPClient | `status == NTRIP_DISCONNECTED` on construction |

---

### test_bno085.py

Unit tests and hardware verification for the BNO085 IMU.
Tests quaternion/heading math and bearing protocol without hardware; hardware flags require the Yukon connected with the IMU fitted.

```
python3 tools/test_bno085.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | auto-detect | Yukon serial port |
| `--hardware` | off | Run hardware protocol round-trip tests (requires Yukon + IMU) |
| `--live` | off | Live heading display (requires Yukon + IMU) |

**Examples**
```
python3 tools/test_bno085.py
python3 tools/test_bno085.py --hardware
python3 tools/test_bno085.py --port /dev/ttyACM0 --live
```

---

### test_robot.py

Integration tests for `robot_daemon.py` using the Yukon PTY simulator.
No hardware required — a pseudo-terminal connects `Robot` to `yukon_sim` in-process.

```
python3 tools/test_robot.py
```

Tests covered:

| # | Test | What is checked |
|---|------|-----------------|
| 1 | Lifecycle | `Robot` object created; initial mode is `MANUAL` |
| 2 | `get_state()` | Returns `RobotState`; correct types and zero defaults |
| 3 | Drive commands | `drive(0.5, −0.5)` → correct wire bytes in simulator |
| 4 | Kill | `kill()` zeroes both motor bytes |
| 5 | LED | `set_led_a` / `set_led_b` update simulator LED state |
| 6 | Telemetry | 1 Hz thread populates voltage, current, temperature |
| 7 | IMU heading | `get_heading()` decodes `RESP_HEADING` within 3° tolerance |
| 8 | Estop | `estop()` does not raise; mode becomes `ESTOP` |
| 9 | Reset ESTOP | `reset_estop()` returns mode to `MANUAL` |
| 10 | AUTO mode | `set_mode(AUTO/MANUAL)` transitions; `set_mode(ESTOP)` raises; `drive()` clamps and stores values |
| 11 | Data logging | `start_data_log()` / `stop_data_log()` create valid JSONL with `ts`, `mode`, `drive` fields |

The iBUS port is set to a non-existent path — this is expected and harmless; the main robot stack no longer uses Pi-side iBUS (RC input moved to Yukon GP26).

---

### test_leds.py

Host-side tests for the NeoPixel LED strip module (SLOT3).
Tests all colour presets, individual pixel control, `set_pixels()`, and all built-in patterns.

```
python3 tools/test_leds.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--yukon-port PORT` | `/dev/ttyACM0` | Yukon serial port |
| `--baud N` | `115200` | Baud rate |
| `--dry-run` | off | Protocol encoding tests only — no hardware required |

**Examples**
```
python3 tools/test_leds.py --dry-run
python3 tools/test_leds.py --yukon-port /dev/ttyACM0
```

---

### test_aruco.py

Unit tests and live camera test for ArUco marker detection and gate navigation.
Tests detector initialisation, synthetic frame detection, navigator helper functions,
and state-machine transitions without hardware; `--live` requires a camera.

```
python3 tools/test_aruco.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--live` | off | Run live camera detection view |
| `--camera N` | `0` | Camera index for live mode |

**Examples**
```
python3 tools/test_aruco.py
python3 tools/test_aruco.py --live
python3 tools/test_aruco.py --live --camera 0
```

---

### test_aruco_detector.py

Unit tests for `robot/aruco_detector.py` using synthetic OpenCV-generated frames — no camera required.

```
python3 tools/test_aruco_detector.py
```

Tests: blank frame, single tag, gate pair formation, wrong-direction gate, gate centre calculation, multiple gates, non-consecutive tags, FPS/timestamp fields, calibrated distance/bearing, `draw=False`.

---

### test_aruco_navigator.py

Unit tests for `robot/aruco_navigator.py` state machine using synthetic `ArUcoState` inputs — no camera, motors, or IMU required.

```
python3 tools/test_aruco_navigator.py
```

Tests all state transitions (IDLE → SEARCHING → ALIGNING → APPROACHING → PASSING → RECOVERING → COMPLETE), obstacle stop, single-tag fallback, IMU search stepping, and `from_ini()` config loading.

---

### test_hardware.py

Interactive real-world hardware tests requiring physical robot hardware.
Guides the operator through verifying the Yukon connection, telemetry, motors (forward/reverse per side), IMU heading, and ArUco tag detection.

```
python3 tools/test_hardware.py [--port PORT]
```

---

### test_tag_approach.py

Fluid ArUco tag approach tests.  Requires a running robot with camera and motors.
Three tests: direct approach, arc from left, arc from right.  The robot uses a continuous
steering law — no stop-and-pivot phases.

```
python3 tools/test_tag_approach.py [flags]
```

---

### nav_visualiser.py

Real-time overhead navigation view — shows robot position, visible ArUco tags, gate lines, camera bearing cone, planned aim point, navigator state, motor bars, IMU compass, and LiDAR returns.

```bash
python3 tools/nav_visualiser.py --live      # poll running robot_dashboard.py at :5000
python3 tools/nav_visualiser.py --sim       # closed-loop simulation (no hardware)
python3 tools/nav_visualiser.py --udp       # listen for UDP telemetry broadcast
python3 tools/nav_visualiser.py --live --url http://pi-ip:5000
```

---

### yukon_firmware_and_software/test_bno085_yukon.py

BNO085 IMU driver tests — runs directly on the Yukon RP2040 (MicroPython).

Upload this file **and** `lib/bno085.py` to the Yukon with Thonny, then run from the Thonny shell. Do **not** save as `main.py`.

Tests:
1. I2C scan — BNO085 visible at expected address
2. Driver init — `BNO085()` constructs without error
3. Data flow — `update()` delivers fresh quaternion data within 500 ms

---

## Calibration & Generation

### calibrate_camera.py

Interactive lens calibration tool for the IMX296 global shutter camera.
Captures checkerboard images across a 3×3 zone grid and outputs `camera_cal.npz`.

```bash
python3 tools/calibrate_camera.py
python3 tools/calibrate_camera.py --width 640 --height 480
python3 tools/calibrate_camera.py --config robot.ini
```

| Flag | Default | Description |
|------|---------|-------------|
| `--width N` | from `robot.ini` `[camera]` (640) | Capture width in pixels |
| `--height N` | from `robot.ini` `[camera]` (480) | Capture height in pixels |
| `--config PATH` | `../robot.ini` | Config file to read resolution from |

**Keys**

| Key | Action |
|-----|--------|
| `Space` | Manual capture |
| `A` | Toggle auto-capture (default: on — holds still 1.2 s then captures) |
| `M` | Toggle mirror display (useful when holding the board yourself) |
| `R` | Rotate image 90° |
| `C` | Compute calibration from current captures (≥ 6 needed) |
| `U` | Toggle undistort preview (after calibration) |
| `Q` / `Esc` | Save and quit |

**Workflow**

1. Print `docs/checkerboard_9x6.pdf` (25 mm squares, 9×6 inner corners).
2. Run the tool. Cover all 9 zones shown on screen — corners first, then edges, then centre.
3. Hold the board still in each zone; auto-capture triggers after 1.2 s.
4. Press `C` to preview calibration quality (RMS < 0.5 = excellent, < 1.0 = acceptable).
5. Press `Q` to save `camera_cal.npz`.

Output: `camera_cal_{width}x{height}.npz` in the repo root (e.g. `camera_cal_640x480.npz`).
Used by `robot/aruco_detector.py` for lens undistortion and pose estimation.

**Note:** calibrate at the highest resolution the camera supports for best accuracy,
then use `derive_calibrations.py` to generate files for all other resolutions.
The tool reads `[camera] width/height` from `robot.ini` automatically, so running
without flags will always match your configured resolution.

---

### derive_calibrations.py

Derives per-resolution calibration `.npz` files from a master calibration.
Calibrate once at full sensor resolution, then run this tool to generate
files for every other resolution automatically.

```bash
# Resolutions from [camera_calibrations] resolutions in robot.ini
python3 tools/derive_calibrations.py camera_cal_1456x1088.npz

# Explicit resolutions
python3 tools/derive_calibrations.py camera_cal_1456x1088.npz 640x480 1280x720
```

| Argument | Description |
|----------|-------------|
| `master` | Source `.npz` produced by `calibrate_camera.py` |
| `WxH ...` | Target resolutions (optional — defaults to `[camera_calibrations] resolutions` in `robot.ini`) |
| `--config PATH` | Path to `robot.ini` (default: `../robot.ini`) |

Output files are written alongside the master (e.g. `camera_cal_640x480.npz`).
`robot_daemon.py` selects the right file automatically using the `{width}x{height}` template in `[aruco] calib_file`.

---

### generate_aruco_tags.py

Generate ArUco marker PDFs — one tag per page at 300 DPI.

```bash
python3 tools/generate_aruco_tags.py                    # IDs 1–4, 4X4_50, A4
python3 tools/generate_aruco_tags.py 1 2 3 4 5 6       # specific IDs
python3 tools/generate_aruco_tags.py --dict 6X6_100 7 8
python3 tools/generate_aruco_tags.py --paper LETTER --size 150 1 2
```

**Options**

| Option | Default | Description |
|--------|---------|-------------|
| `IDs` (positional) | `1 2 3 4` | Tag IDs to generate |
| `--dict NAME` | `4X4_50` | ArUco dictionary (`4X4_50`, `4X4_100`, `5X5_100`, `6X6_100`, …) |
| `--paper SIZE` | `A4` | Paper size (`A3`, `A4`, `A5`, `LETTER`, `LEGAL`, `HALF`) |
| `--size MM` | 80% of page width | Printed tag size in mm |
| `--out PATH` | `docs/aruco_tags_<ids>.pdf` | Output PDF path |

---

### make_checkerboard_pdf.py

Generate a printable checkerboard calibration target.

```bash
python3 tools/make_checkerboard_pdf.py
```

Output: `docs/checkerboard_9x6.pdf`
- 9×6 inner corners (10×7 squares)
- 25 mm squares (matches `calibrate_camera.py`)
- Centred on A4 at 300 DPI

---

## Simulators

### pygame_gamepad_ibus_rx.py

Physical gamepad → iBUS PTY simulator. Uses a pygame joystick/gamepad to drive iBUS packets on a PTY, replacing the keyboard-based `ibus_sim.py` for use with a physical controller.

Channel layout matches `robot.ini` defaults: CH1 aileron, CH2 elevator, CH3 throttle, CH4 rudder, CH5–CH8 mode switches, CH10 bookmark.

```bash
python3 tools/pygame_gamepad_ibus_rx.py                          # default joystick 0
python3 tools/pygame_gamepad_ibus_rx.py --list-joysticks         # list available devices
python3 tools/pygame_gamepad_ibus_rx.py --joystick 1 --hz 143
python3 tools/pygame_gamepad_ibus_rx.py --axis-aileron 3 --axis-elevator 4  # Xbox layout
```

| Flag | Default | Description |
|------|---------|-------------|
| `--list-joysticks` | off | Print available joystick devices and exit |
| `--joystick N` | `0` | Joystick device index |
| `--hz N` | `143` | iBUS packet rate in Hz |
| `--deadzone N` | `0.05` | Axis deadzone (0.0–1.0) |
| `--axis-rudder N` | `0` | Axis index for CH4 rudder (left stick H) |
| `--axis-throttle N` | `1` | Axis index for CH3 throttle (left stick V, inverted) |
| `--axis-aileron N` | `2` | Axis index for CH1 aileron (right stick H) |
| `--axis-elevator N` | `3` | Axis index for CH2 elevator (right stick V) |
| `--btn-mode N` | `0` | Button for CH5 mode toggle |
| `--btn-speed N` | `1` | Button for CH6 speed cycle |
| `--btn-type N` | `2` | Button for CH7 type cycle |
| `--btn-gpslog N` | `3` | Button for CH8 GPS log toggle |
| `--btn-bookmark N` | `4` | Button for CH10 momentary bookmark |
| `--btn-centre N` | `6` | Button to centre sticks and drop throttle |
| `--btn-signal N` | `7` | Button to toggle RC signal loss |

On startup the PTY path is printed to stderr. Pass it to `yukon_sim.py` or `rc_drive.py` via `--ibus-port`.

---

### ibus_sim.py

Interactive iBUS RC receiver simulator — creates a PTY and emits iBUS packets at ~143 Hz, mimicking a FlySky receiver connected to a RadioMaster TX-16S.

```bash
python3 tools/ibus_sim.py
python3 tools/ibus_sim.py --hz 143 --step 50
```

| Flag | Default | Description |
|------|---------|-------------|
| `--hz N` | `143` | Packet rate in Hz (~7 ms/packet, matching real hardware) |
| `--step N` | `50` | Stick µs change per keypress |

**Keys**

| Key | Action |
|-----|--------|
| `W` / `S` | Throttle up / down (CH3) |
| `A` / `D` | Rudder left / right (CH4) |
| Arrow keys | Aileron / elevator (CH1 / CH2) |
| `1` | Toggle SwA — MANUAL / AUTO (CH5) |
| `2` | Cycle SwB — slow 25% / mid / max (CH6) |
| `3` | Cycle SwC — Camera / GPS / Cam+GPS (CH7) |
| `4` | Toggle SwD — GPS log off / on (CH8) |
| `5` | Momentary bookmark pulse (CH10, 300 ms) |
| `Space` | Centre sticks and drop throttle to 1000 |
| `V` | Toggle RC signal loss (stops emitting packets) |
| `Q` | Quit |

On startup the PTY path is printed to stderr. Pass it to any consumer:

```bash
# Pipe into rc_drive.py
python3 rc_drive.py --ibus-port /dev/pts/3

# Pipe into yukon_sim so CMD_RC_QUERY returns live stick values
python3 tools/yukon_sim_gui.py --ibus-port /dev/pts/3
```

---

### yukon_sim.py

Consolidated PTY-based Yukon USB serial simulator for offline development and testing.
Supports three display modes selected via `--mode` (default: gui).

```bash
python3 tools/yukon_sim.py                        # GUI mode (default)
python3 tools/yukon_sim.py --mode gui
python3 tools/yukon_sim.py --mode web [--port 5002]
python3 tools/yukon_sim.py --mode headless
python3 tools/yukon_sim.py --ibus-port /dev/pts/3  # pipe ibus_sim into RC channels
```

Creates a virtual serial port (PTY) that emulates `/dev/ttyACM0`:
- Parses 5-byte command packets and responds with ACK/NAK.
- Returns simulated sensor data for `CMD_SENSOR` and RC channel data for `CMD_RC_QUERY`.
- Tracks `--no-motors` mode (sees `CMD_RC_QUERY` but no `CMD_MODE` heartbeat).
- Optionally reads live RC channels from `ibus_sim.py` via `--ibus-port`.

| Flag | Default | Description |
|------|---------|-------------|
| `--mode MODE` | `gui` | Display mode: `gui` \| `web` \| `headless` |
| `--port N` | `5002` | HTTP port (web mode only) |
| `--ibus-port DEV` | none | iBUS PTY/device to read RC channels from (e.g. `ibus_sim.py` PTY) |

Point any client (`robot_daemon.py`, `tools/test_main.py`) at the PTY path printed on startup.

---

## Utilities

### upload.py

Upload a MicroPython file to the Yukon RP2040 over USB serial.

```bash
python3 tools/upload.py yukon_firmware_and_software/main.py
python3 tools/upload.py yukon_firmware_and_software/main.py main.py
```

| Argument | Default | Description |
|----------|---------|-------------|
| `SRC` (positional, required) | — | Source file to upload |
| `DEST` (positional) | basename of `SRC` | Destination filename on device |

Port is hardcoded to `/dev/ttyACM0` at 115200 baud.

Use this instead of `mpremote` or `ampy` — those fail because `yukon.reset()` drops the USB connection on every Ctrl+C. This tool handles the double USB reconnect before entering the raw REPL.

---

### gps_route_builder.py

Pygame GUI for recording and editing GPS waypoint routes.
Saves and loads the versioned JSON format: `{"version": 1, "waypoints": [...]}`.

```bash
python3 tools/gps_route_builder.py
python3 tools/gps_route_builder.py --waypoints route.json --live
```

| Flag | Default | Description |
|------|---------|-------------|
| `--waypoints PATH` | none | Load waypoints JSON on start |
| `--live` | off | Connect to robot.py on start |

---

### gps_route_builder_web.py

Web-based GPS route builder — browser UI for recording and editing waypoints.
Exports the versioned JSON format: `{"version": 1, "waypoints": [...]}`.

```bash
python3 tools/gps_route_builder_web.py
python3 tools/gps_route_builder_web.py --waypoints route.json --live
```

| Flag | Default | Description |
|------|---------|-------------|
| `--host HOST` | `0.0.0.0` | Bind address |
| `--port N` | `5003` | HTTP port |
| `--waypoints PATH` | none | Load waypoints JSON on start |
| `--live` | off | Auto-connect to robot.py |

---

### read_data_log.py

Web viewer for JSONL data logs produced by `robot_daemon.py`.
Opens log files from `~/Documents/HackyRacingRobot/` and serves an interactive
browser dashboard on port 5004.

```bash
python3 tools/read_data_log.py
python3 tools/read_data_log.py --dir /path/to/logs --port 5004
```

| Flag | Default | Description |
|------|---------|-------------|
| `--dir PATH` | `~/Documents/HackyRacingRobot/` | Directory containing `.jsonl` files |
| `--host HOST` | `0.0.0.0` | Bind address |
| `--port N` | `5004` | HTTP port |

**Tabs**

| Tab | Content |
|-----|---------|
| Drive | Mode timeline (colour-coded), left/right motor commands, RC steer/throttle |
| Telemetry | Voltage, IMU heading, board/motor temps, CPU % and temp |
| GPS | Leaflet map with track coloured by mode, speed-over-time chart |
| LiDAR | Polar scatter plot for any frame via slider |
| Inspector | Raw JSON for any frame |

Charts downsample long runs to 800 points; full LiDAR data is fetched on demand per frame.

---

### depth_viewer.py

Standalone depth map visualiser. Connects to a running `robot_dashboard.py` and displays the left/right rectified camera frames, a Jet-colourmap depth map (stereo, mono, or fusion), a depth histogram, and source/metric status overlay.

Does not instantiate `Robot` directly — reads over the web stream, so it works against a remote Pi.

```bash
python3 tools/depth_viewer.py                       # local robot at :5000
python3 tools/depth_viewer.py --host 192.168.1.42
python3 tools/depth_viewer.py --max-depth 5.0 --fps 20
```

| Flag | Default | Description |
|------|---------|-------------|
| `--host HOST` | `localhost` | Dashboard host |
| `--port N` | `5000` | Dashboard port |
| `--max-depth M` | `8.0` | Depth range for colour scale in metres |
| `--fps N` | `15` | Display refresh rate in Hz |

**Keys:** `Q` / `Esc` — quit.

---

### setup_depth_model.py

Download and verify the `scdepthv3` HEF model for the Hailo-8L depth accelerator.
Downloads from the Hailo Model Zoo S3 bucket and places the file at `models/scdepthv3.hef` relative to the project root. Also verifies the Hailo device is reachable.

```bash
python3 tools/setup_depth_model.py
python3 tools/setup_depth_model.py --model-dir /path/to/models/
python3 tools/setup_depth_model.py --force         # re-download even if file exists
python3 tools/setup_depth_model.py --no-verify     # skip Hailo device check
```

| Flag | Default | Description |
|------|---------|-------------|
| `--model-dir DIR` | `<project_root>/models/` | Destination directory |
| `--force` | off | Re-download even if the file already exists |
| `--no-verify` | off | Skip Hailo device verification after download |

Enable depth in `robot.ini` (`[depth] enabled = true`) then configure `mode` and `hailo_model` as needed.

---

### yukon_firmware_and_software/i2c_scan.py

I2C bus scanner — runs directly on the Yukon RP2040 (MicroPython).

Upload with Thonny and run from the Thonny shell. Do **not** save as `main.py`.

```
# In Thonny shell after uploading:
import i2c_scan
```

Scans the Yukon's built-in Qw/ST I2C bus (I2C0, 400 kHz) and prints found device addresses.
