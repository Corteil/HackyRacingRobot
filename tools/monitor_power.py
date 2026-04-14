#!/usr/bin/env python3
"""
monitor_power.py — Raspberry Pi 5 input voltage & current monitor
Reads from the onboard PMIC via the kernel hwmon interface.
Pi 4 and earlier: only core voltage is available via vcgencmd.

Usage:
    python3 tools/monitor_power.py              # single reading
    python3 tools/monitor_power.py --watch      # live updating display
    python3 tools/monitor_power.py --watch --interval 0.5
"""

import argparse
import glob
import os
import subprocess
import sys
import time


# ── hwmon helpers ──────────────────────────────────────────────────────────────

def _read_hwmon_file(path: str) -> int | None:
    """Return integer value from a hwmon sysfs file, or None on failure."""
    try:
        with open(path) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def find_pmic_hwmon() -> str | None:
    """
    Locate the hwmon directory for a PMIC that exposes readable voltage/current.
    'rpi_volt' is excluded — on Pi 5 it only provides a low-voltage alarm bit,
    not an actual voltage reading.  Use read_rpi_volt_alarm() for that.
    Returns the hwmon path, or None if not found.
    """
    for hwmon_dir in sorted(glob.glob('/sys/class/hwmon/hwmon*/')):
        name_path = os.path.join(hwmon_dir, 'name')
        try:
            with open(name_path) as f:
                name = f.read().strip()
            if name in ('mp2760', 'rt5759', 'rpi_pmic'):
                return hwmon_dir
        except OSError:
            continue
    return None


def find_rpi_volt_hwmon() -> str | None:
    """Return the hwmon path for the Pi 5 rpi_volt alarm driver, or None."""
    for hwmon_dir in sorted(glob.glob('/sys/class/hwmon/hwmon*/')):
        name_path = os.path.join(hwmon_dir, 'name')
        try:
            with open(name_path) as f:
                if f.read().strip() == 'rpi_volt':
                    return hwmon_dir
        except OSError:
            continue
    return None


def read_rpi_volt_alarm(hwmon_dir: str) -> bool | None:
    """
    Read the Pi 5 low-voltage alarm bit from rpi_volt hwmon.
    Returns True if undervoltage is detected, False if OK, None on failure.
    """
    raw = _read_hwmon_file(os.path.join(hwmon_dir, 'in0_lcrit_alarm'))
    if raw is None:
        return None
    return raw != 0


def read_pmic(hwmon_dir: str) -> dict:
    """
    Read voltage (µV → mV), current (mA), and derive power from the PMIC hwmon.
    Returns a dict with keys: voltage_mv, current_ma, power_mw (floats).
    Missing channels are returned as None.
    """
    result = {}

    # Voltage channels: in0_input, in1_input … (µV)
    voltage_uv = None
    for channel in ('in1_input', 'in0_input'):
        raw = _read_hwmon_file(os.path.join(hwmon_dir, channel))
        if raw is not None:
            voltage_uv = raw
            break
    result['voltage_mv'] = voltage_uv / 1000 if voltage_uv is not None else None

    # Current channels: curr1_input, curr2_input … (mA)
    current_ma = None
    for channel in ('curr1_input', 'curr2_input', 'curr0_input'):
        raw = _read_hwmon_file(os.path.join(hwmon_dir, channel))
        if raw is not None:
            current_ma = raw
            break
    result['current_ma'] = float(current_ma) if current_ma is not None else None

    # Derived power
    if result['voltage_mv'] is not None and result['current_ma'] is not None:
        result['power_mw'] = (result['voltage_mv'] * result['current_ma']) / 1000
    else:
        result['power_mw'] = None

    return result


# ── vcgencmd fallback ──────────────────────────────────────────────────────────

def read_vcgencmd_voltage() -> float | None:
    """
    Read core voltage via vcgencmd (works on all Pi models).
    Returns voltage in mV, or None on failure.
    """
    try:
        out = subprocess.check_output(
            ['vcgencmd', 'measure_volts'],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        # format: "volt=0.8688V"
        value = float(out.split('=')[1].rstrip('V'))
        return value * 1000  # → mV
    except (subprocess.SubprocessError, ValueError, IndexError):
        return None


# ── display ───────────────────────────────────────────────────────────────────

def _fmt(value, unit, decimals=0, warn_low=None, warn_high=None) -> str:
    if value is None:
        return 'N/A'
    s = f"{value:.{decimals}f} {unit}"
    if warn_low is not None and value < warn_low:
        s += '  ⚠ LOW'
    if warn_high is not None and value > warn_high:
        s += '  ⚠ HIGH'
    return s


def print_reading(data: dict, source: str, alarm: bool | None = None, is_input_voltage: bool = True):
    v = data.get('voltage_mv')
    i = data.get('current_ma')
    p = data.get('power_mw')

    print(f"  Source  : {source}")
    # Warn thresholds only make sense when monitoring the 5V input rail
    if is_input_voltage:
        print(f"  Voltage : {_fmt(v, 'mV', 0, warn_low=4750, warn_high=5250)}")
    else:
        print(f"  Voltage : {_fmt(v, 'mV', 0)}")
    if i is not None:
        print(f"  Current : {_fmt(i, 'mA', 0)}")
    if p is not None:
        print(f"  Power   : {_fmt(p, 'mW', 0)}")
    if alarm is not None:
        status = '  *** UNDERVOLTAGE DETECTED ***' if alarm else 'OK'
        print(f"  5V alarm: {status}")


def watch_loop(hwmon_dir: str | None, alarm_dir: str | None, interval: float):
    print(f"Watching power — press Ctrl+C to stop  (interval: {interval}s)\n")
    try:
        while True:
            ts = time.strftime('%H:%M:%S')
            alarm = read_rpi_volt_alarm(alarm_dir) if alarm_dir else None

            if hwmon_dir:
                data = read_pmic(hwmon_dir)
                source = f"PMIC hwmon ({os.path.basename(hwmon_dir.rstrip('/'))})"
            else:
                v = read_vcgencmd_voltage()
                data = {'voltage_mv': v, 'current_ma': None, 'power_mw': None}
                source = 'vcgencmd (core voltage)'

            # Overwrite previous block
            if 'watching' in watch_loop.__dict__:
                lines = 2  # source + voltage
                if data.get('current_ma') is not None:
                    lines += 1
                if data.get('power_mw') is not None:
                    lines += 1
                if alarm is not None:
                    lines += 1
                sys.stdout.write(f'\033[{lines + 1}A')  # cursor up

            watch_loop.watching = True
            print(f"[{ts}]")
            print_reading(data, source, alarm, is_input_voltage=hwmon_dir is not None)
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nStopped.")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Monitor Raspberry Pi input power via hwmon or vcgencmd.")
    parser.add_argument('--watch', action='store_true', help='Continuously update readings')
    parser.add_argument('--interval', type=float, default=1.0, metavar='SECS',
                        help='Refresh interval for --watch (default: 1.0)')
    args = parser.parse_args()

    hwmon_dir = find_pmic_hwmon()
    alarm_dir = find_rpi_volt_hwmon()

    if args.watch:
        watch_loop(hwmon_dir, alarm_dir, args.interval)
        return

    # Single reading
    alarm = read_rpi_volt_alarm(alarm_dir) if alarm_dir else None

    if hwmon_dir:
        data = read_pmic(hwmon_dir)
        source = f"PMIC hwmon  ({hwmon_dir.rstrip('/')})"
        print_reading(data, source, alarm, is_input_voltage=True)
    else:
        v = read_vcgencmd_voltage()
        if v is None and alarm is None:
            # Fallback: list all hwmon devices so user can investigate
            all_hwmon = glob.glob('/sys/class/hwmon/hwmon*/')
            if all_hwmon:
                print("No readable PMIC hwmon found. Available hwmon devices:")
                for h in sorted(all_hwmon):
                    name_path = os.path.join(h, 'name')
                    try:
                        with open(name_path) as f:
                            name = f.read().strip()
                    except OSError:
                        name = '?'
                    print(f"  {h}  ({name})")
                print()
            print("No power data available — vcgencmd also failed.")
            sys.exit(1)
        data = {'voltage_mv': v, 'current_ma': None, 'power_mw': None}
        source = 'vcgencmd (core voltage — Pi 5 does not expose input voltage via hwmon)'
        print_reading(data, source, alarm, is_input_voltage=False)


if __name__ == '__main__':
    main()
