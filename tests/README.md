# Yukon Test Programs

All tests run on the Raspberry Pi host (not the Yukon itself).
Run from the repo root or from the `tests/` directory.

---

## test_main.py

Host-side tests for the Yukon `main.py` serial protocol.
Connects over USB serial and exercises motor control, LED, kill, and sensor commands.

```
python3 tests/test_main.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | auto-detect | Serial port (e.g. `/dev/ttyACM0`) |
| `--baud N` | `115200` | Baud rate |
| `--dry-run` | off | Run protocol encoder tests only — no hardware required |
| `--ramp` | off | Run 30-second motor ramp test instead of the standard suite |

**Examples**
```
python3 tests/test_main.py --dry-run
python3 tests/test_main.py --port /dev/ttyACM0
python3 tests/test_main.py --port /dev/ttyACM0 --ramp
```

---

## test_ibus.py

Unit tests and live channel display for the iBUS RC receiver.
Shows a live bar graph of all 14 channels, updating in-place.

```
python3 tests/test_ibus.py [PORT]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `PORT` (positional) | `/dev/ttyUSB0` | Serial port for the iBUS receiver |

Press `Ctrl+C` to exit the live display.

**Examples**
```
python3 tests/test_ibus.py
python3 tests/test_ibus.py /dev/ttyAMA3
```

---

## test_ld06.py

Unit tests and live display for the LD06 LiDAR driver.
Shows 16 compass sectors (22.5° each) as a bar graph of minimum distance per sector.

```
python3 tests/test_ld06.py [PORT]
python3 tests/test_ld06.py -u
```

| Argument | Default | Description |
|----------|---------|-------------|
| `PORT` (positional) | `/dev/ttyAMA0` | Serial port for the LD06 |
| `-u` | off | Run unit tests only — no hardware required |

Press `Ctrl+C` to exit the live display.

**Examples**
```
python3 tests/test_ld06.py -u
python3 tests/test_ld06.py /dev/ttyAMA0
```

---

## test_gps.py

TAU1308 RTK GNSS rover with a pygame GUI.
Settings are read from `tests/config.ini`; CLI flags override them.

```
python3 tests/test_gps.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--config PATH` | `tests/config.ini` | Path to config file |
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

**Examples**
```
python3 tests/test_gps.py
python3 tests/test_gps.py --port /dev/ttyUSB0 --debug
python3 tests/test_gps.py --no-config --hz 5
```

---

## test_bno085.py

Unit tests and hardware verification for the BNO085 IMU.
Tests quaternion/heading math and bearing protocol without hardware; hardware flags require the Yukon connected with the IMU fitted.

```
python3 tests/test_bno085.py [flags]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | auto-detect | Yukon serial port |
| `--hardware` | off | Run hardware protocol round-trip tests (requires Yukon + IMU) |
| `--live` | off | Live heading display (requires Yukon + IMU) |

**Examples**
```
python3 tests/test_bno085.py
python3 tests/test_bno085.py --hardware
python3 tests/test_bno085.py --port /dev/ttyACM0 --live
```
