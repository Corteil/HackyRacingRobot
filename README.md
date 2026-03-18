# Yukon Robot Controller

Raspberry Pi–hosted controller for a Pimoroni Yukon robot. A Pi reads an RC transmitter via iBUS and sends motor commands to the Yukon over USB serial using a compact 5-byte protocol.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Pimoroni Yukon | RP2040-based motor controller board |
| Left motors | `DualMotorModule` in SLOT2 |
| Right motors | `DualMotorModule` in SLOT5 |
| RC receiver | FlySky iBUS on GPIO 9 / `/dev/ttyAMA3` (uart3-pi5 overlay) |
| Host ↔ Yukon | USB serial `/dev/ttyACM0` at 115200 baud |
| Camera | IMX296 (global shutter, fixed focus) via picamera2 |
| LiDAR | LD06 on `/dev/ttyAMA0`; PWM motor drive on GPIO 18 |

---

## Files

### `main.py` — Yukon firmware (MicroPython)

Runs on the Yukon (RP2040). Responsibilities:

- Registers and enables two `DualMotorModule`s (SLOT2 = left, SLOT5 = right)
- **Core 1:** continuously applies the latest left/right motor speeds
- **Core 0:** reads 5-byte serial commands from the host, runs Yukon health monitoring, and periodically logs sensor data
- On `FaultError`: zeros speeds, re-enables output and modules, then continues
- Exits cleanly on boot button press (`yukon.reset()`)

### `ibus.py` — FlySky iBUS receiver library

Reads the iBUS RC protocol from a serial port on the Pi.

- 32-byte packets at ~7 ms intervals (115200 8N1)
- Syncs to `0x20 0x40` header, verifies checksum (`0xFFFF − sum(bytes[0:30])`)
- Decodes 14 channels as 16-bit little-endian values (range 1000–2000, mid 1500)
- API: `IBusReader(port)` as context manager; `.read()` returns a list of 14 ints or `None`

### `test_ibus.py` — iBUS tests and live display

- **Unit tests** (no hardware): packet construction, header, checksum, channel decode
- **Live display**: bar graph of all 14 channels, updating in-place; defaults to `/dev/ttyAMA3`

```
python3 test_ibus.py              # unit tests + live display
python3 test_ibus.py --unit-only  # unit tests only
```

Channel mapping (transmitter must assign switches in its channel menu):

| Channel | Function |
|---------|----------|
| CH1 | Aileron |
| CH2 | Elevator |
| CH3 | Throttle |
| CH4 | Rudder |
| CH5 | SwA |
| CH6 | SwB |

### `robot.py` — Robot controller daemon

Manages all Pi-side hardware: camera, LiDAR, GPS/NTRIP, RC receiver and Yukon serial link.

- `SystemState` dataclass: single snapshot of all sensor/status data shared with GUIs
- `Robot` class: starts/stops subsystem threads; exposes `get_state()`, `get_frame()`, `get_aruco_state()`
- Camera thread captures frames, rotates, runs ArUco detection; corrects BGR→RGB channel order from picamera2
- LiDAR (`_Lidar`): reads LD06, exports latest `LidarScan`; PWM motor control via GPIO 18 sysfs
- GPS (`_GPS`): NMEA parsing, optional NTRIP RTK injection
- Configuration via `robot.ini` (all sections editable at runtime through `robot_gui.py`)

### `robot_gui.py` — 4-panel pygame monitor

Full-screen robot dashboard.

```
python3 robot_gui.py [--config robot.ini] [--no-camera] [--no-lidar]
```

Panels: Drive bars, Telemetry (voltage/current/temps/faults), GPS, Camera+LiDAR.

Key bindings:

| Key | Action |
|-----|--------|
| E | Emergency stop |
| R | Reset |
| `[` / `]` | Cycle camera |
| T | Toggle ArUco detection |
| C | Config overlay (edit `robot.ini` live) |
| Q / Esc | Quit |

Config overlay: ↑↓ navigate, Enter edit value, Left/Right toggle booleans, S save, Esc cancel.

### `lidar_gui.py` — Standalone LiDAR visualiser

360° polar plot of LD06 scan data.

```
python3 lidar_gui.py [PORT]    # default /dev/ttyAMA0
```

| Key | Action |
|-----|--------|
| Scroll / `+` / `-` | Zoom (500 mm – 12 m range) |
| F | Freeze / unfreeze scan |
| G | Toggle grid |
| Q / Esc | Quit |

Bottom bar shows RPM, scan rate, point count, range, and nearest obstacle.

### `camera_monitor.py` — Live camera monitor

Focus-assist and colour-mode tool for the IMX296 camera.

```
python3 camera_monitor.py
```

| Key | Action |
|-----|--------|
| M | Cycle colour mode (Colour / Greyscale / False Colour / Edges / Focus Peak) |
| S | Cycle capture resolution |
| R | Rotate 90° |
| H | Toggle histogram |
| A | Toggle auto-exposure |
| ↑ / ↓ | Adjust exposure (manual mode) |
| `[` / `]` | Adjust gain |
| T | Toggle ArUco detection |
| D | Cycle ArUco dictionary |
| F | Freeze frame (↑↓ scroll tag list while frozen) |
| Q / Esc | Quit |

Right panel shows detected ArUco tags (ID, distance, bearing, corners). Bottom bar shows sharpness metric (Laplacian variance).

### `ld06.py` — LD06 LiDAR driver

- Parses 47-byte packets: CRC8 (poly 0x4D), 12 points/packet, speed in °/s×100, distances in mm
- Background reader thread; `get_scan()` returns latest `LidarScan(angles, distances, rpm)`
- PWM motor speed exported via `/sys/class/pwm/pwmchip0` on GPIO 18 (requires `dtoverlay=pwm,pin=18,func=2`)
- PWM permissions: install `/etc/udev/rules.d/99-pwm.rules` (see below)

### `test_ld06.py` — LD06 tests and live display

- **Unit tests** (no hardware): 16 tests covering packet structure, CRC, angle interpolation, RPM, zero-intensity clamp, wrap-around
- **Live display**: 16 compass sectors (22.5° each), min-distance bar graph

```
python3 test_ld06.py [PORT]        # unit tests + live display
python3 test_ld06.py -u            # unit tests only
```

### `aruco_detector.py` — ArUco tag detector

Wraps OpenCV `ArucoDetector`. Returns per-tag corners, ID, estimated distance and bearing. Shared by `robot.py` and `camera_monitor.py`.

### `test_main.py` — Host-side protocol tester

Tests the Yukon serial protocol from the Pi.

- **Encoder tests** (no hardware): round-trip encode/decode, printable-ASCII validation, checksum, speed encoding
- **Hardware tests**: LED, left/right motors, kill, sensor readback, error detection (bad checksum, out-of-range fields)
- **Ramp test**: 30-second motor speed profile with live sensor logging

```
python3 test_main.py              # encoder tests + hardware tests (auto-detect port)
python3 test_main.py --dry-run    # encoder tests only
python3 test_main.py --ramp       # 30-second ramp test
python3 test_main.py --port /dev/ttyACM0
```

### `upload.py` — MicroPython file uploader

Uploads a file to the Yukon via raw REPL, handling the double USB reconnect caused by `yukon.reset()` on each Ctrl+C interrupt.

```
python3 upload.py main.py
```

---

## Serial Protocol (Host → Yukon)

5-byte packets, all bytes printable ASCII (no REPL interference):

```
[SYNC, CMD, V_HIGH, V_LOW, CHK]
```

| Field  | Encoding | Range |
|--------|----------|-------|
| SYNC   | `0x7E` (`~`) | fixed |
| CMD    | `cmd_code + 0x20` | `0x21–0x25` |
| V_HIGH | `(value >> 4) + 0x40` | `0x40–0x4F` |
| V_LOW  | `(value & 0xF) + 0x50` | `0x50–0x5F` |
| CHK    | `CMD ^ V_HIGH ^ V_LOW` | never equals SYNC |

Response: `ACK` (0x06) on success, `NAK` (0x15) on any error.

### Commands

| Command | Code | Value |
|---------|------|-------|
| `CMD_LED` | 1 | 0=LED_A off, 1=LED_A on, 2=LED_B off, 3=LED_B on |
| `CMD_LEFT` | 2 | motor speed byte (see below) |
| `CMD_RIGHT` | 3 | motor speed byte |
| `CMD_KILL` | 4 | ignored — zeros both motors |
| `CMD_SENSOR` | 5 | ignored — replies with sensor data then ACK |

### Motor speed encoding

| Speed | Byte value |
|-------|-----------|
| 0% (stop) | 0 |
| +50% (fwd) | 50 |
| +100% (fwd) | 100 |
| −50% (rev) | 150 |
| −100% (rev) | 200 |

### Sensor response (Device → Host)

`CMD_SENSOR` triggers 7 data packets followed by ACK:

| ID | Name | Scale |
|----|------|-------|
| 0 | Voltage | raw ÷ 10 → V |
| 1 | Current | raw ÷ 100 → A |
| 2 | Board temp | raw ÷ 3 → °C |
| 3 | Left module temp | raw ÷ 3 → °C |
| 4 | Right module temp | raw ÷ 3 → °C |
| 5 | Left fault | 0 or 1 |
| 6 | Right fault | 0 or 1 |

---

## Setup Notes

### LiDAR PWM permissions

The LD06 spin motor is driven by hardware PWM on GPIO 18. Add to `/boot/firmware/config.txt`:

```
dtoverlay=pwm,pin=18,func=2
```

The kernel creates the sysfs PWM node as `root:root`. Install the following udev rule so the `gpio` group can write to it without `sudo`:

```
# /etc/udev/rules.d/99-pwm.rules
SUBSYSTEM=="pwm", KERNEL=="pwmchip[0-9]*", ACTION=="add", \
    RUN+="/bin/chgrp gpio /sys%p/export /sys%p/unexport", \
    RUN+="/bin/chmod g+w  /sys%p/export /sys%p/unexport"
SUBSYSTEM=="pwm", KERNEL=="pwm[0-9]*", ACTION=="add", \
    RUN+="/bin/sh -c 'chgrp -R gpio /sys%p && chmod -R g+rw /sys%p'"
```

Then reload: `sudo udevadm control --reload-rules`

### Camera channel order

picamera2 `RGB888` format returns BGR bytes in memory. `robot.py` and `camera_monitor.py` both apply `[:, :, ::-1]` immediately after `capture_array()` to correct this before any processing or display.
