# Hardware Setup

## Hardware overview

| Component     | Detail |
|---------------|--------|
| Pimoroni Yukon | RP2040-based motor controller board |
| Left motors   | `DualMotorModule` in SLOT2 |
| Right motors  | `DualMotorModule` in SLOT5 |
| RC receiver   | FlySky iBUS on GPIO 9 / `/dev/ttyAMA3` (`uart3-pi5` overlay) |
| Host ↔ Yukon  | USB serial `/dev/ttyACM0` at 115200 baud |
| Camera        | IMX296 (global shutter, fixed focus) via picamera2 |
| LiDAR         | LD06 on `/dev/ttyAMA0`; PWM motor drive on GPIO 18 |

---

## Device tree overlays

Add the following lines to `/boot/firmware/config.txt`:

```ini
# UART3 for iBUS RC receiver (GPIO 9 RX)
dtoverlay=uart3-pi5

# UART0 for LD06 LiDAR (GPIO 15 RX)
dtoverlay=uart0-pi5

# Hardware PWM on GPIO 18 for LD06 spin motor
dtoverlay=pwm,pin=18,func=2
```

Reboot after editing.

---

## LiDAR PWM permissions

The LD06 spin motor is driven by hardware PWM on GPIO 18. The kernel creates the sysfs PWM node as `root:root`. Install the following udev rule so the `gpio` group can write to it without `sudo`:

```
# /etc/udev/rules.d/99-pwm.rules
SUBSYSTEM=="pwm", KERNEL=="pwmchip[0-9]*", ACTION=="add", \
    RUN+="/bin/chgrp gpio /sys%p/export /sys%p/unexport", \
    RUN+="/bin/chmod g+w  /sys%p/export /sys%p/unexport"
SUBSYSTEM=="pwm", KERNEL=="pwm[0-9]*", ACTION=="add", \
    RUN+="/bin/sh -c 'chgrp -R gpio /sys%p && chmod -R g+rw /sys%p'"
```

Reload rules: `sudo udevadm control --reload-rules`

---

## Python dependencies

```bash
pip install pyserial picamera2 pygame opencv-python-headless
```

---

## Uploading MicroPython firmware to the Yukon

Use `tools/upload.py` (not `mpremote` or `ampy` — these fail because `yukon.reset()` drops USB on every Ctrl+C):

```bash
python3 tools/upload.py main.py
```

This handles the double USB reconnect caused by `yukon.reset()` on each Ctrl+C interrupt before entering the raw REPL.

---

## Camera channel order

picamera2 `RGB888` format returns BGR bytes in memory. `robot.py` and `camera_monitor.py` both apply `[:, :, ::-1]` immediately after `capture_array()` to correct this before any processing or display.

---

## Configuration

All runtime settings live in `robot.ini`. See the file for section-by-section documentation. The config overlay in `robot_gui.py` (press `C`) allows live editing without restarting.
