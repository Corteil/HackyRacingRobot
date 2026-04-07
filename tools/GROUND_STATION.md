# Ground Station

Web-based operator dashboard for HackyRacingRobot. Runs on the operator's
laptop and communicates with the robot over a Holybro SiK V3 433 MHz radio
link. Open `http://localhost:5000` in any browser to see live telemetry, an
FPV camera feed, and GPS/LiDAR data, and to forward RTK corrections back to
the robot.

---

## Files

| File | Purpose |
|---|---|
| `tools/ground_station.py` | Flask web server — the ground station app |
| `tools/ground_station.html` | Frontend (generated — see Setup below) |
| `tools/build_gs_html.py` | Generates `ground_station.html` from `robot_dashboard.py` |
| `tools/serial_telemetry.py` | Runs on the **robot Pi** — SiK radio bridge |

The frontend is identical to `robot_dashboard.py` — same panel layout, presets,
and mobile view — with a FPV Camera panel added and 📡/RTK badges in the status
bar.

---

## How it works

```
Robot Pi                              Operator Laptop
────────────────────────────────      ──────────────────────────────────
serial_telemetry.py                   ground_station.py
  │                                     │
  ├─ robot.get_state() → JSON ──────────┤──→ /api/state SSE → browser
  │    5 Hz downlink                    │
  ├─ LiDAR (subsampled, 1 Hz) ─────────┤
  │                                     │
  │◄── RTCM corrections ────────────────┤◄── NTRIP caster (internet)
  │    uplink                           │
  │◄── Commands (estop, mode…) ─────────┤◄── browser → /api/cmd
  │                                     │
                                        ├─ /stream/fpv → browser
                                        │    FPV capture card (local)
```

The SiK radio is a transparent serial link — `serial_telemetry.py` owns the
port on the robot and `ground_station.py` owns the matching port on the laptop.
RTCM correction bytes and JSON command packets share the uplink; the robot side
distinguishes them by looking for a `{` at the start of each line.

---

## Setup

### 1. Install dependencies

**On the robot Pi** (if not already installed):
```bash
pip install pyserial
```

**On the operator laptop:**
```bash
pip install flask pyserial
pip install opencv-python   # optional — needed for FPV feed
```

### 2. Generate the frontend HTML

Run this once after cloning, and again whenever `robot_dashboard.py` changes:

```bash
python3 tools/build_gs_html.py
```

This extracts the HTML/CSS/JS from `robot_dashboard.py` and applies
ground-station patches (FPV panel, link-quality badges, etc.), writing
`tools/ground_station.html`.

### 3. Add the robot.ini section

Add to `robot.ini` on the Pi and set `[rtcm] disabled = true` (the SiK
radio replaces the RTCM serial port):

```ini
[telemetry_radio]
disabled     = false
port         = /dev/ttyUSB1   ; check with: ls /dev/ttyUSB*
baud         = 57600          ; must match SiK ATS1 param on both radios
telemetry_hz = 5              ; state packet downlink rate
lidar        = true           ; 1 Hz subsampled LiDAR downlink
lidar_step   = 5              ; degrees between samples (72 points at 5°)

[rtcm]
disabled     = true           ; serial_telemetry.py owns the SiK port
```

### 4. Add inject_rtcm() to Robot class

Paste the method from `inject_rtcm_patch.py` into `robot_daemon.py` after
`get_auto_type()` (around line 2531). This is the one change to Robot that
`serial_telemetry.py` needs for RTCM forwarding.

### 5. Configure SiK baud rate (if changing from default)

The SiK radios default to 57600 baud. To change to 115200 on both radios,
connect each with a serial terminal and run:

```
+++          (wait 1 second — enters command mode)
ATS1=115200
AT&W
ATZ
```

Repeat for the second radio. The air data rate (64 kbps) is independent and
unchanged.

---

## Running

### Robot side (Pi)

```bash
python3 tools/serial_telemetry.py

# Options:
python3 tools/serial_telemetry.py --port /dev/ttyUSB1 --baud 115200
python3 tools/serial_telemetry.py --no-lidar        # save bandwidth
python3 tools/serial_telemetry.py --no-camera       # faster startup
python3 tools/serial_telemetry.py --tcp-port 5010   # also expose over TCP
```

### Ground station (laptop)

```bash
# Real SiK radio
python3 tools/ground_station.py --serial-port /dev/ttyUSB0      # Linux
python3 tools/ground_station.py --serial-port COM5               # Windows

# LAN/WiFi (Pi running serial_telemetry.py --tcp-port 5010)
python3 tools/ground_station.py --backend network --network-host 192.168.1.10

# Fully fake — synthetic data, no hardware needed
python3 tools/ground_station.py --backend fake

# Change web port
python3 tools/ground_station.py --web-port 8080
```

Then open **http://localhost:5000** (or the IP shown in the terminal) in any
browser. The same URL works from a phone or tablet on the same network.

---

## Backends

Three backends are available, selected with `--backend`:

### `real` (default)
Physical SiK radio connected via USB. The radio appears as a standard serial
port — no special drivers needed on Linux or Windows.

```bash
python3 tools/ground_station.py --serial-port /dev/ttyUSB0 --baud 57600
```

### `network`
TCP socket instead of serial. Useful for testing over LAN/WiFi before the SiK
radios arrive, or running both ends on the same machine (loopback).

On the robot, start `serial_telemetry.py` with `--tcp-port`:
```bash
# Robot Pi
python3 tools/serial_telemetry.py --tcp-port 5010

# Laptop
python3 tools/ground_station.py --backend network \
    --network-host 192.168.1.10 --network-port 5010
```

Alternatively, expose the serial port over TCP with `socat`:
```bash
socat TCP-LISTEN:5010,fork,reuseaddr FILE:/dev/ttyUSB1,b57600,raw
```

### `fake`
Generates synthetic telemetry locally at 5 Hz — no hardware, no serial port,
no network. Great for UI development and testing the dashboard layout.
NTRIP is automatically disabled in fake mode.

```bash
python3 tools/ground_station.py --backend fake
python3 tools/ground_station.py --backend fake --fpv-device 0   # with webcam
```

The fake generator simulates oscillating motor speeds, a GPS fix cycling
through GPS → RTK Float → RTK Fixed, IMU rotation, battery sag, and a LiDAR
scan with a simulated wall.

---

## FPV Camera

The FPV feed comes from a local USB capture card or webcam on the operator
laptop — not from the robot. The FPV video radio delivers the feed
independently of the SiK data link.

`opencv-python` must be installed:
```bash
pip install opencv-python
```

```bash
python3 tools/ground_station.py --fpv-device 0    # first device (default)
python3 tools/ground_station.py --fpv-device 1    # second device
python3 tools/ground_station.py --fpv-device /dev/video2   # explicit path
python3 tools/ground_station.py --no-fpv           # disable entirely
```

The FPV feed appears as a **FPV Camera** panel in the panel chooser and is
set as the top-left panel in the default Race preset. Robot-side cameras
(front left, front right, rear) are not available at the ground station and
show a placeholder frame.

---

## NTRIP (RTK corrections)

The ground station connects to an NTRIP caster and forwards RTK correction
bytes back to the robot over the SiK uplink. The robot's `serial_telemetry.py`
injects them directly into the TAU1308 GNSS receiver.

```bash
python3 tools/ground_station.py \
    --ntrip-host www.rtk2go.com \
    --ntrip-mount CAMBRIDGE \
    --ntrip-user your@email.com \
    --ntrip-password none
```

Credentials via environment variables (keeps them out of shell history):
```bash
export NTRIP_USER=your@email.com
export NTRIP_PASSWORD=none
python3 tools/ground_station.py --ntrip-mount CAMBRIDGE
```

Disable NTRIP entirely:
```bash
python3 tools/ground_station.py --no-ntrip
```

NTRIP status appears in the `RTK` badge in the status bar and in the GPS
panel. RTK Fixed typically takes ~5 minutes from a good caster connection.

---

## Commands

The browser can send commands back to the robot. They are forwarded as JSON
over the SiK uplink and executed by `serial_telemetry.py`.

| Button | Effect |
|---|---|
| ESTOP | Kill motors immediately |
| Reset ESTOP | Clear ESTOP, return to MANUAL |
| AUTO / MANUAL | Switch robot mode |
| ⏺ REC / ⏹ STOP | Start / stop camera recording |
| ⬤ DLOG | Toggle ML data logging |
| No Motors | Toggle no-motors bench mode |
| ArUco | Toggle ArUco detection |

Commands that only make sense locally (camera rotation, stream pause, etc.)
are silently ignored by the ground station.

---

## Bandwidth budget

The SiK V3 air data rate is fixed at 64 kbps. Usable throughput is
approximately 4–5 KB/s each direction after FHSS/TDM overhead.

| Data | Rate | Size | Bandwidth |
|---|---|---|---|
| State JSON | 5 Hz | ~500 B | ~2.5 KB/s ↓ |
| LiDAR JSON (72 pts) | 1 Hz | ~600 B | ~0.6 KB/s ↓ |
| RTCM corrections | continuous | 500–2000 B/s | 0.5–2 KB/s ↑ |
| Commands | on demand | ~50 B | negligible ↑ |

To save downlink bandwidth: `--no-lidar` on `serial_telemetry.py`, or
`lidar = false` in `robot.ini`. To save uplink bandwidth for RTCM, limit
the caster to MSM4 messages only.

---

## Port reference

| Service | Default | Machine |
|---|---|---|
| Ground station web UI | `5000` | Laptop |
| TCP bridge (network backend) | `5010` | Pi |
| Robot dashboard | `5000` | Pi |
| SiK radio — robot side | `/dev/ttyUSB1` | Pi |
| SiK radio — laptop side | `/dev/ttyUSB0` | Laptop |
| TAU1308 GNSS | `/dev/ttyUSB0` | Pi |

> **Note:** On the Pi, the TAU1308 GNSS takes `/dev/ttyUSB0` so the SiK
> radio lands on `/dev/ttyUSB1`. Confirm with `ls /dev/ttyUSB*` with both
> plugged in.

---

## Troubleshooting

**"Cannot open serial port"**
Run `ls /dev/ttyUSB*` (Linux) or check Device Manager (Windows). On Linux,
if you get a permission error: `sudo usermod -aG dialout $USER` then log out
and back in.

**"HTML not found: ground_station.html"**
Run `python3 tools/build_gs_html.py` first.

**No telemetry — 📡 badge shows red**
Check that `serial_telemetry.py` is running on the Pi and that both SiK
radios have matching `NET_ID` and `AIR_SPEED` parameters. Use `ATI5` in a
serial terminal to inspect each radio's config, or use Mission Planner.

**FPV feed not showing**
Install `opencv-python`. Try `--fpv-device 1` if device 0 is the laptop's
built-in webcam rather than the capture card. On Linux, `ls /dev/video*`
lists available devices.

**NTRIP not connecting**
Browse to `http://www.rtk2go.com:2101/` to list available mountpoints and
check the mount name is correct. The RTK2go password is literally `none`.

**RTK not fixing**
Allow ~5 minutes with a clear sky view. RTK Fixed shows `< 0.05 m` in the
`H Error` field of the GPS panel. Check that RTCM bytes are flowing — the
`RTCM recv` counter in the GPS panel should be increasing.
