# Yukon Tools

Helper scripts for calibration, hardware testing, firmware upload, and content generation.
Run from the repo root unless noted otherwise.

---

## calibrate_camera.py

Interactive lens calibration tool for the IMX296 global shutter camera.
Captures checkerboard images across a 3×3 zone grid and outputs `camera_cal.npz`.

```bash
python3 tools/calibrate_camera.py
```

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

Output: `camera_cal.npz` in the repo root. Used by `camera_monitor.py` (key `K`) and `camera_web.py`.

---

## generate_aruco_tags.py

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

## make_checkerboard_pdf.py

Generate a printable checkerboard calibration target.

```bash
python3 tools/make_checkerboard_pdf.py
```

Output: `docs/checkerboard_9x6.pdf`
- 9×6 inner corners (10×7 squares)
- 25 mm squares (matches `calibrate_camera.py`)
- Centred on A4 at 300 DPI

---

## upload.py

Upload a MicroPython file to the Yukon RP2040 over USB serial.

```bash
python3 tools/upload.py main.py
python3 tools/upload.py main.py --port /dev/ttyACM0
```

Use this instead of `mpremote` or `ampy` — those fail because `yukon.reset()` drops the USB connection on every Ctrl+C. This tool handles the double USB reconnect before entering the raw REPL.

---

## yukon_sim.py

PTY-based Yukon USB serial simulator for offline development and testing.

```bash
python3 tools/yukon_sim.py
```

Creates a virtual serial port (PTY) that emulates `/dev/ttyACM0`:
- Parses 5-byte command packets and responds with ACK/NAK.
- Returns simulated sensor data for `CMD_SENSOR`.

Point any client (`robot.py`, `rc_drive.py`, `tests/test_main.py`) at the PTY path printed on startup.

---

## i2c_scan.py

I2C bus scanner — runs directly on the Yukon RP2040 (MicroPython).

Upload with Thonny and run from the Thonny shell. Do **not** save as `main.py`.

```
# In Thonny shell after uploading:
import i2c_scan
```

Scans the Yukon's built-in Qw/ST I2C bus (I2C0, 400 kHz) and prints found device addresses. Useful for verifying wiring before writing drivers.

---

## test_bno085_yukon.py

BNO085 IMU driver tests — runs directly on the Yukon RP2040 (MicroPython).

Upload this file **and** `lib/bno085.py` to the Yukon with Thonny, then run from the Thonny shell. Do **not** save as `main.py`.

Tests:
1. I2C scan — BNO085 visible at expected address
2. Driver init — `BNO085()` constructs without error
3. Data flow — `update()` delivers fresh quaternion data within 500 ms
