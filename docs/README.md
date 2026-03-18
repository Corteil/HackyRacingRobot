# Yukon Robot Controller

Raspberry Pi–hosted controller for a Pimoroni Yukon robot. A Pi reads an RC transmitter via iBUS and sends motor commands to the Yukon over USB serial using a compact 5-byte protocol.

---

## Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) — Component diagram, key files, data flow, `SystemState` dataclass, threading model
- [PROTOCOL.md](PROTOCOL.md) — 5-byte serial protocol specification, command table, motor encoding, sensor response format
- [SETUP.md](SETUP.md) — Hardware wiring, device tree overlays, udev rules, dependencies, upload instructions
- [allystar.pdf](allystar.pdf) — Allystar TAU1308 GNSS module datasheet

---

## Quick start

```bash
# Run the full robot stack with pygame GUI
python3 robot_gui.py

# Run the web dashboard
python3 robot_web.py

# RC drive only (no GUI)
python3 rc_drive.py

# Standalone LiDAR visualiser
python3 lidar_gui.py

# Camera focus/colour tool
python3 camera_monitor.py
```

---

## Hardware

| Component | Detail |
|-----------|--------|
| Pimoroni Yukon | RP2040-based motor controller board |
| Left motors | `DualMotorModule` in SLOT2 |
| Right motors | `DualMotorModule` in SLOT5 |
| RC receiver | FlySky iBUS on GPIO 9 / `/dev/ttyAMA3` (`uart3-pi5` overlay) |
| Host ↔ Yukon | USB serial `/dev/ttyACM0` at 115200 baud |
| Camera | IMX296 (global shutter, fixed focus) via picamera2 |
| LiDAR | LD06 on `/dev/ttyAMA0`; PWM motor drive on GPIO 18 |

---

## Key files

| File | Purpose |
|------|---------|
| `main.py` | MicroPython firmware for the Yukon RP2040 |
| `robot.py` | Pi-side robot daemon (camera, LiDAR, GPS, RC, Yukon serial) |
| `robot_gui.py` | 4-panel pygame monitor |
| `robot_web.py` | Flask web dashboard with MJPEG stream |
| `rc_drive.py` | Minimal RC-to-motor bridge |
| `ibus.py` | FlySky iBUS receiver library |
| `ld06.py` | LD06 LiDAR driver |
| `aruco_detector.py` | OpenCV ArUco tag detector |
| `gnss/` | GNSS driver package (TAU1308, UBlox variants, NTRIP) |
| `robot.ini` | Runtime configuration |
| `tools/upload.py` | MicroPython file uploader |
| `tools/yukon_sim.py` | Yukon serial simulator (PTY) |
| `tests/` | Unit tests and live-display tools |
