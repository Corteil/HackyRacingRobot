#!/usr/bin/env python3
"""
tools/ground_station.py — Ground station web dashboard for HackyRacingRobot.

Runs on the operator's laptop.  Open http://localhost:5000 in any browser.
Uses the same HTML/JS frontend as robot_dashboard.py — identical panel layout,
presets, and mobile view — backed by the SiK radio link instead of a Robot.

Data flow
---------
  Downlink  SiK radio → JSON state → SSE → browser  (telemetry, GPS, LiDAR…)
  FPV       local USB capture card → MJPEG → browser  (/stream/fpv)
  Uplink    browser → /api/cmd → JSON over SiK → serial_telemetry.py → robot

Three serial backends (select with --backend)
----------------------------------------------
  real      Physical SiK radio on a serial port (default).
              --serial-port /dev/ttyUSB0   (Linux)
              --serial-port COM5           (Windows)

  network   TCP socket instead of serial — useful for testing over LAN or
            when running robot_dashboard.py and ground_station.py on the
            same machine via a loopback TCP bridge.
              --network-host 192.168.1.10  --network-port 5010
            On the robot side, start a bridge:
              socat TCP-LISTEN:5010,fork,reuseaddr \
                    FILE:/dev/ttyUSB1,b57600,raw
            Or run serial_telemetry.py with --tcp-port 5010.

  fake      Generates synthetic telemetry locally — no hardware needed.
            Great for UI development and frontend testing.
              --backend fake

Requires : flask pyserial
Optional : opencv-python  (FPV stream)

Usage
-----
  # Real hardware
  python3 tools/ground_station.py --serial-port /dev/ttyUSB0

  # Network (same LAN as robot)
  python3 tools/ground_station.py --backend network --network-host 192.168.1.10

  # Fully local fake data (no hardware at all)
  python3 tools/ground_station.py --backend fake

  # Fake data with FPV from webcam
  python3 tools/ground_station.py --backend fake --fpv-device 0

  # Change web port
  python3 tools/ground_station.py --web-port 8080

NTRIP credentials (preferred via env vars):
  NTRIP_USER=you@example.com NTRIP_PASSWORD=none python3 tools/ground_station.py
"""

import argparse
import base64
import json
import logging
import math
import os
import queue
import signal
import socket
import sys
import threading
import time
from typing import Optional

log = logging.getLogger("ground_station")

try:
    from flask import Flask, Response, request, jsonify
except ImportError:
    sys.exit("flask not found — pip install flask")

try:
    import serial as _pyserial
    _SERIAL = True
except ImportError:
    _SERIAL = False

try:
    import cv2
    _CV2 = True
except ImportError:
    _CV2 = False


# ══════════════════════════════════════════════════════════════════════════════
# Shared state
# ══════════════════════════════════════════════════════════════════════════════

class _GsState:
    """Thread-safe store for the latest expanded state dict."""

    def __init__(self):
        self._lock        = threading.Lock()
        self._state: Optional[dict] = None
        self.packets      = 0
        self.last_rx      = 0.0
        self.ntrip_status = "disabled"
        self.ntrip_bytes  = 0

    def update(self, expanded: dict):
        with self._lock:
            self._state   = expanded
            self.packets += 1
            self.last_rx  = time.monotonic()

    def get(self) -> Optional[dict]:
        with self._lock:
            return self._state

    def link_age(self) -> float:
        return time.monotonic() - self.last_rx if self.last_rx else 999.0


_gs = _GsState()
_gs_lidar: Optional[dict] = None   # latest lidar packet, updated by RX thread


# ══════════════════════════════════════════════════════════════════════════════
# Compact SiK JSON → full robot_dashboard state dict
# ══════════════════════════════════════════════════════════════════════════════

def _expand(msg: dict) -> dict:
    """Convert compact SiK state packet to robot_dashboard.py _serialise() format."""
    tl  = msg.get("telem", {})
    g   = msg.get("gps",   {})
    s   = msg.get("sys",   {})
    nav = msg.get("nav",   {})
    fl  = msg.get("flags", {})
    drv = msg.get("drv",   [0.0, 0.0])

    lidar_angles:    list = []
    lidar_distances: list = []
    if _gs_lidar:
        lidar_angles    = [float(a) for a in _gs_lidar.get("a", [])]
        lidar_distances = [float(d) for d in _gs_lidar.get("d", [])]

    cam_ok = fl.get("cam_ok", False)

    # NTRIP: prefer ground-station status when it's actively running;
    # fall back to the robot's own NTRIP status from the packet.
    gs_ntrip_active = _gs.ntrip_status not in ("", "disabled")
    ntrip_st    = _gs.ntrip_status if gs_ntrip_active else g.get("ntrip", "")
    ntrip_bytes = _gs.ntrip_bytes

    return {
        # Top-level flags
        "mode":          msg.get("mode", "MANUAL"),
        "auto_type":     "Camera",
        "speed_scale":   fl.get("speed", 0.25),
        "rc_active":     msg.get("rc", False),
        "camera_ok":     cam_ok,
        "lidar_ok":      fl.get("lidar_ok", False),
        "gps_ok":        fl.get("gps_ok", False),
        "aruco_ok":      False,
        "aruco_enabled": {},
        "cam_rotation":  0,
        "gps_logging":   False,
        "cam_recording": fl.get("cam_rec", False),
        "data_logging":  fl.get("data_log", False),
        "no_motors":     fl.get("no_motors", False),
        "bench_enabled": True,
        # Per-camera status — only FL known over SiK; FR/RE unavailable
        "cam_fl_ok":  cam_ok,
        "cam_fr_ok":  False,
        "cam_re_ok":  False,
        "cam_fl_rec": fl.get("cam_rec", False),
        "cam_fr_rec": False,
        "cam_re_rec": False,
        "gate_confirmed": False, "current_gate_id": 0,
        "cam_cap_w": {}, "cam_cap_h": {}, "aruco": {},
        # Navigation
        "nav_state":          nav.get("st", "IDLE"),
        "nav_gate":           nav.get("gate", 0),
        "nav_bearing_err":    nav.get("berr"),
        "nav_target_bearing": nav.get("bear"),
        "nav_target_dist":    nav.get("dist"),
        "nav_tags_visible":   nav.get("tags", 0),
        "nav_wp":             nav.get("wp", 0),
        "nav_wp_dist":        nav.get("dist"),
        "nav_wp_bear":        nav.get("bear"),
        # Drive
        "drive": {"left": round(drv[0], 3), "right": round(drv[1], 3)},
        # Telemetry — abbreviated keys → full names
        "telemetry": {
            "voltage":     tl.get("v"),
            "current":     tl.get("i"),
            "board_temp":  tl.get("bt"),
            "left_temp":   tl.get("lt"),
            "right_temp":  tl.get("rt"),
            "left_fault":  tl.get("lf", False),
            "right_fault": tl.get("rf", False),
            "heading":     tl.get("hdg"),
            "pitch":       tl.get("pit"),
            "roll":        tl.get("rol"),
        },
        # GPS — abbreviated keys → full names
        "gps": {
            "latitude":         g.get("lat"),
            "longitude":        g.get("lon"),
            "altitude":         g.get("alt"),
            "speed":            g.get("spd"),
            "heading":          g.get("hdg"),
            "fix_quality":      g.get("fix", 0),
            "fix_quality_name": g.get("fqn", "Invalid"),
            "h_error_m":        g.get("herr"),
            "satellites":       g.get("sats"),
            "satellites_view":  g.get("sats_view", g.get("sats")),
            "satellites_data":  g.get("sat_data", []),
            "hdop":             g.get("hdop"),
            "ntrip_status":     ntrip_st,
            "ntrip_bytes_recv": ntrip_bytes,
        },
        # LiDAR
        "lidar": {"angles": lidar_angles, "distances": lidar_distances},
        # System
        "system": {
            "cpu_percent":  s.get("cpu",  0.0),
            "cpu_temp_c":   s.get("temp", 0.0),
            "cpu_freq_mhz": 0.0,
            "mem_used_mb":  0.0,
            "mem_total_mb": 0.0,
            "mem_percent":  s.get("mem",  0.0),
            "disk_used_gb": 0.0,
            "disk_total_gb":0.0,
            "disk_percent": s.get("disk", 0.0),
        },
        # Ground-station extras (read by patched JS badges)
        "_link_age":     round(_gs.link_age(), 1),
        "_link_packets": _gs.packets,
        "_ntrip_status": ntrip_st,
        "_ntrip_bytes":  ntrip_bytes,
    }


def _dispatch_rx(msg: dict):
    """Route a parsed inbound JSON packet."""
    global _gs_lidar
    t = msg.get("t")
    if t == "state":
        _gs.update(_expand(msg))
    elif t == "lidar":
        _gs_lidar = msg
    # Unknown types silently ignored


# ══════════════════════════════════════════════════════════════════════════════
# Serial link abstraction
# ══════════════════════════════════════════════════════════════════════════════

class _LinkBase:
    """Common RX-line parser and TX queue drainer shared by all backends."""

    def __init__(self, rtcm_q: queue.Queue, cmd_q: queue.Queue):
        self._rtcm_q  = rtcm_q
        self._cmd_q   = cmd_q
        self._stop    = threading.Event()
        self._tx_lock = threading.Lock()

    def stop(self):
        self._stop.set()

    # ── subclasses must implement ─────────────────────────────────────────────
    def _read(self, n: int) -> bytes:
        raise NotImplementedError

    def _write(self, data: bytes):
        raise NotImplementedError

    # ── shared RX loop ────────────────────────────────────────────────────────
    def _rx_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._read(512)
            except Exception as e:
                log.warning("RX error: %s", e)
                time.sleep(1.0)
                continue
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    _dispatch_rx(json.loads(line))
                except (json.JSONDecodeError, Exception) as e:
                    log.debug("RX parse: %s", e)

    # ── shared TX loop ────────────────────────────────────────────────────────
    def _tx_loop(self):
        CHUNK    = 200
        rtcm_buf = b""
        while not self._stop.is_set():
            sent = False
            try:
                rtcm_buf += self._rtcm_q.get_nowait()
            except queue.Empty:
                pass
            while len(rtcm_buf) >= CHUNK:
                self._safe_write(rtcm_buf[:CHUNK])
                rtcm_buf = rtcm_buf[CHUNK:]
                sent = True
            try:
                obj  = self._cmd_q.get_nowait()
                line = (json.dumps(obj, separators=(',', ':')) + '\n').encode()
                self._safe_write(line)
                sent = True
            except queue.Empty:
                pass
            if not sent:
                if rtcm_buf:
                    self._safe_write(rtcm_buf)
                    rtcm_buf = b""
                time.sleep(0.02)

    def _safe_write(self, data: bytes):
        try:
            with self._tx_lock:
                self._write(data)
        except Exception as e:
            log.warning("TX error: %s", e)

    def _start_threads(self):
        threading.Thread(target=self._rx_loop, daemon=True, name="gs_rx").start()
        threading.Thread(target=self._tx_loop, daemon=True, name="gs_tx").start()


# ══════════════════════════════════════════════════════════════════════════════
# Backend 1: real serial (SiK radio)
# ══════════════════════════════════════════════════════════════════════════════

class SerialLink(_LinkBase):
    def __init__(self, port: str, baud: int,
                 rtcm_q: queue.Queue, cmd_q: queue.Queue):
        super().__init__(rtcm_q, cmd_q)
        if not _SERIAL:
            raise ImportError("pyserial not installed — pip install pyserial")
        self._ser = _pyserial.Serial(port, baud, timeout=0.1, write_timeout=1.0)
        time.sleep(0.3)
        self._ser.reset_input_buffer()
        log.info("SiK serial: %s @ %d baud", port, baud)

    def _read(self, n):
        return self._ser.read(n)

    def _write(self, data):
        self._ser.write(data)

    def stop(self):
        super().stop()
        try: self._ser.close()
        except Exception: pass

    def start(self):
        self._start_threads()


# ══════════════════════════════════════════════════════════════════════════════
# Backend 2: TCP network socket
# ══════════════════════════════════════════════════════════════════════════════

class NetworkLink(_LinkBase):
    """
    Connects to a TCP server that bridges the SiK serial port.

    On the robot side, expose the serial port over TCP with:
      socat TCP-LISTEN:5010,fork,reuseaddr FILE:/dev/ttyUSB1,b57600,raw

    Or add --tcp-port 5010 to serial_telemetry.py (see its --help).

    Reconnects automatically on disconnect.
    """

    def __init__(self, host: str, port: int,
                 rtcm_q: queue.Queue, cmd_q: queue.Queue):
        super().__init__(rtcm_q, cmd_q)
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._conn_lock = threading.Lock()
        log.info("Network link: %s:%d", host, port)

    def _connect(self):
        while not self._stop.is_set():
            try:
                s = socket.create_connection((self._host, self._port), timeout=5)
                s.settimeout(0.5)
                with self._conn_lock:
                    self._sock = s
                log.info("Network link connected: %s:%d", self._host, self._port)
                return
            except Exception as e:
                log.warning("Network connect failed: %s — retrying in 3s", e)
                time.sleep(3)

    def _read(self, n):
        with self._conn_lock:
            s = self._sock
        if s is None:
            time.sleep(0.1)
            return b""
        try:
            return s.recv(n)
        except socket.timeout:
            return b""
        except Exception:
            with self._conn_lock:
                self._sock = None
            log.warning("Network link disconnected — reconnecting")
            threading.Thread(target=self._connect, daemon=True,
                             name="gs_net_conn").start()
            return b""

    def _write(self, data):
        with self._conn_lock:
            s = self._sock
        if s:
            s.sendall(data)

    def stop(self):
        super().stop()
        with self._conn_lock:
            if self._sock:
                try: self._sock.close()
                except Exception: pass
                self._sock = None

    def start(self):
        # Connect in a background thread so Flask can start immediately.
        # _read() returns b"" while _sock is None, so the RX loop waits.
        threading.Thread(target=self._connect, daemon=True,
                         name="gs_net_conn").start()
        self._start_threads()


# ══════════════════════════════════════════════════════════════════════════════
# Backend 3: fake data generator (no hardware)
# ══════════════════════════════════════════════════════════════════════════════

class FakeLink(_LinkBase):
    """
    Generates synthetic telemetry locally at 5 Hz.
    No serial port or network connection needed — useful for UI development.

    Simulates:
      - Oscillating motor speeds (figure-of-eight)
      - Slowly drifting GPS position (Cambridge area)
      - Realistic IMU heading (slow rotation)
      - Battery voltage sag over time (resets after 60 s)
      - LiDAR scan with a simulated wall on one side
      - RTK fix quality cycling through 0→1→4
    """

    def __init__(self, rtcm_q: queue.Queue, cmd_q: queue.Queue):
        super().__init__(rtcm_q, cmd_q)
        log.info("Fake data backend started — no hardware needed")

    def _read(self, n):
        # Never receives anything meaningful — just keep the RX loop alive
        time.sleep(0.1)
        return b""

    def _write(self, data):
        # Silently discard uplink data
        pass

    def start(self):
        # Only need the TX loop (discards) and the generator
        threading.Thread(target=self._tx_loop, daemon=True, name="gs_tx").start()
        threading.Thread(target=self._generate, daemon=True, name="gs_fake").start()

    def _generate(self):
        t0    = time.monotonic()
        lat0  = 52.204268    # Cambridge
        lon0  = 0.118880
        modes = ["MANUAL", "MANUAL", "AUTO"]
        fix_seq = [0, 1, 1, 1, 4, 4, 4, 4, 4, 5]

        while not self._stop.is_set():
            t   = time.monotonic() - t0
            idx = int(t / 5) % len(modes)
            fix = fix_seq[int(t / 3) % len(fix_seq)]

            # Slowly oscillating drive
            left  = round(math.sin(t * 0.5) * 0.6, 3)
            right = round(math.sin(t * 0.5 + 0.8) * 0.6, 3)

            # Battery sag
            volt = round(12.6 - (t % 60) * 0.015, 2)

            # GPS drift
            lat = lat0 + math.sin(t * 0.05) * 0.0002
            lon = lon0 + math.cos(t * 0.05) * 0.0002

            # IMU heading slow rotation
            hdg = round((t * 8) % 360, 1)

            state = {
                "t":    "state",
                "ts":   round(t, 3),
                "mode": modes[idx],
                "rc":   True,
                "drv":  [left, right],
                "telem": {
                    "v": volt,  "i": round(abs(left + right) * 8, 2),
                    "bt": 38.0, "lt": 32.0, "rt": 31.5,
                    "lf": False, "rf": False,
                    "hdg": hdg,
                    "pit": round(math.sin(t * 0.3) * 4, 1),
                    "rol": round(math.cos(t * 0.4) * 3, 1),
                },
                "gps": {
                    "lat": round(lat, 7), "lon": round(lon, 7),
                    "alt": 12.5,          "spd": round(abs(left + right) * 2, 2),
                    "hdg": hdg,
                    "fix": fix,
                    "fqn": {0:"Invalid",1:"GPS",2:"DGPS",4:"RTK Fixed",5:"RTK Float"}.get(fix,"GPS"),
                    "herr": 0.025 if fix == 4 else (0.3 if fix == 5 else None),
                    "sats": 10,  "sats_view": 14,  "hdop": 0.9,
                    "ntrip": "connected",
                    "sat_data": [
                        {"svid":  1, "elev": 72, "azim":  45, "snr": int(38+math.sin(t*0.3+ 0)*6), "system":"GPS"},
                        {"svid":  3, "elev": 35, "azim": 120, "snr": int(32+math.sin(t*0.3+ 1)*6), "system":"GPS"},
                        {"svid":  8, "elev": 18, "azim": 210, "snr": int(25+math.sin(t*0.3+ 2)*6), "system":"GPS"},
                        {"svid": 14, "elev": 55, "azim": 300, "snr": int(41+math.sin(t*0.3+ 3)*4), "system":"GPS"},
                        {"svid": 19, "elev": 42, "azim":  80, "snr": int(35+math.sin(t*0.3+ 4)*5), "system":"GPS"},
                        {"svid": 22, "elev":  8, "azim": 170, "snr": int(18+math.sin(t*0.3+ 5)*4), "system":"GPS"},
                        {"svid": 27, "elev": 63, "azim": 250, "snr": int(44+math.sin(t*0.3+ 6)*3), "system":"GPS"},
                        {"svid": 31, "elev": 29, "azim": 340, "snr": int(30+math.sin(t*0.3+ 7)*5), "system":"GPS"},
                        {"svid": 66, "elev": 48, "azim":  20, "snr": int(39+math.sin(t*0.3+ 8)*4), "system":"GLONASS"},
                        {"svid": 73, "elev": 15, "azim": 150, "snr": int(22+math.sin(t*0.3+ 9)*5), "system":"GLONASS"},
                        {"svid": 84, "elev": 61, "azim": 270, "snr": int(43+math.sin(t*0.3+10)*3), "system":"GLONASS"},
                        {"svid":201, "elev": 38, "azim": 195, "snr": int(36+math.sin(t*0.3+11)*5), "system":"GALILEO"},
                        {"svid":210, "elev": 22, "azim":  95, "snr": int(28+math.sin(t*0.3+12)*6), "system":"GALILEO"},
                        {"svid":401, "elev": 80, "azim": 130, "snr": int(47+math.sin(t*0.3+13)*2), "system":"BDS"},
                    ],
                },
                "sys": {
                    "cpu": round(25 + math.sin(t * 0.1) * 10, 1),
                    "temp": round(52 + math.sin(t * 0.07) * 5, 1),
                    "mem": 42.0, "disk": 18.5,
                },
                "nav": {
                    "st": "APPROACHING" if modes[idx] == "AUTO" else "IDLE",
                    "gate": int(t / 10) % 5,
                    "wp": 0,
                    "dist": round(abs(math.sin(t * 0.15)) * 4, 1),
                    "bear": round(hdg + math.sin(t * 0.2) * 15, 1),
                    "berr": round(math.sin(t * 0.3) * 8, 1),
                    "tags": 2 if modes[idx] == "AUTO" else 0,
                },
                "flags": {
                    "lidar_ok": True, "gps_ok": fix > 0,
                    "cam_ok": True,   "data_log": False,
                    "speed": 0.5,
                },
            }

            # LiDAR: simulated wall at ~2 m on left side, open on right
            lidar = {"t": "lidar", "ts": round(t, 3), "a": [], "d": []}
            for deg in range(0, 360, 5):
                lidar["a"].append(deg)
                # Wall on left (90-150°), obstacle ahead (355-5°)
                if 85 <= deg <= 155:
                    dist = int(2000 + math.sin(math.radians(deg)) * 200)
                elif deg >= 355 or deg <= 5:
                    dist = int(1500 + math.sin(t) * 200)
                else:
                    dist = int(4000 + math.sin(math.radians(deg + t * 20)) * 500)
                lidar["d"].append(max(200, dist))

            _dispatch_rx(state)
            _dispatch_rx(lidar)

            time.sleep(0.2)   # 5 Hz


# ══════════════════════════════════════════════════════════════════════════════
# NTRIP client
# ══════════════════════════════════════════════════════════════════════════════

class NtripClient:
    def __init__(self, host, port, mount, user, password,
                 lat, lon, alt, rtcm_q):
        self._host, self._port     = host, port
        self._mount                = mount
        self._user, self._password = user, password
        self._lat, self._lon, self._alt = lat, lon, alt
        self._q    = rtcm_q
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True, name="ntrip").start()

    def stop(self):
        self._stop.set()

    def _gga(self):
        lat, lon = abs(self._lat), abs(self._lon)
        ns = 'N' if self._lat >= 0 else 'S'
        ew = 'E' if self._lon >= 0 else 'W'
        gga = (f"GPGGA,{time.strftime('%H%M%S')}.00,"
               f"{int(lat):02d}{(lat-int(lat))*60:07.4f},{ns},"
               f"{int(lon):03d}{(lon-int(lon))*60:07.4f},{ew},"
               f"1,08,1.0,{self._alt:.1f},M,0.0,M,,")
        ck = 0
        for c in gga: ck ^= ord(c)
        return f"${gga}*{ck:02X}\r\n"

    def _run(self):
        cred = base64.b64encode(
            f"{self._user}:{self._password}".encode()).decode()
        while not self._stop.is_set():
            _gs.ntrip_status = "connecting"
            try:
                sock = socket.create_connection(
                    (self._host, self._port), timeout=10)
                req = (f"GET /{self._mount} HTTP/1.0\r\n"
                       f"Host: {self._host}\r\n"
                       f"User-Agent: HackyRacingRobot/1.0\r\n"
                       f"Authorization: Basic {cred}\r\n"
                       f"Ntrip-Version: Ntrip/2.0\r\n\r\n")
                sock.sendall(req.encode())
                resp = b""
                while b"\r\n\r\n" not in resp:
                    resp += sock.recv(256)
                if b"200" not in resp and b"ICY 200" not in resp:
                    raise ConnectionError(f"Bad response: {resp[:60]}")
                sock.sendall(self._gga().encode())
                _gs.ntrip_status = "connected"
                last_gga = time.monotonic()
                sock.settimeout(5.0)
                while not self._stop.is_set():
                    try:
                        chunk = sock.recv(1024)
                    except socket.timeout:
                        if time.monotonic() - last_gga > 30:
                            sock.sendall(self._gga().encode())
                            last_gga = time.monotonic()
                        continue
                    if not chunk:
                        raise ConnectionError("Connection closed")
                    _gs.ntrip_bytes += len(chunk)
                    self._q.put(chunk)
            except Exception as e:
                _gs.ntrip_status = "error"
                log.warning("NTRIP: %s", e)
                time.sleep(5)


# ══════════════════════════════════════════════════════════════════════════════
# FPV capture
# ══════════════════════════════════════════════════════════════════════════════

class FpvCapture:
    """Captures from a local webcam and provides JPEG frames for MJPEG stream."""

    def __init__(self, device):
        self._device = device
        self._frame: Optional[bytes] = None
        self._lock   = threading.Lock()
        self._stop   = threading.Event()
        self.ok      = False
        if _CV2:
            threading.Thread(target=self._run, daemon=True, name="fpv").start()
        else:
            log.warning("FPV disabled — pip install opencv-python")

    def stop(self):
        self._stop.set()

    def get_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._frame

    def _run(self):
        try:
            dev = int(self._device)
        except (ValueError, TypeError):
            dev = self._device

        cap = cv2.VideoCapture(dev)
        if not cap.isOpened():
            log.warning("FPV: cannot open device %s", self._device)
            return

        self.ok = True
        log.info("FPV capture: device %s", self._device)

        while not self._stop.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self._lock:
                    self._frame = buf.tobytes()

        cap.release()
        self.ok = False


# ══════════════════════════════════════════════════════════════════════════════
# Flask app
# ══════════════════════════════════════════════════════════════════════════════

app    = Flask(__name__)
_fpv:  Optional[FpvCapture] = None
_cmd_q: queue.Queue         = queue.Queue(maxsize=20)
_HTML  = ""

# Commands that are meaningful to forward over the radio uplink
_UPLINK_CMDS = frozenset({
    'estop', 'reset', 'set_mode',
    'data_log_toggle', 'gps_bookmark',
    'record_start', 'record_stop', 'record_toggle',
    'bench_toggle', 'no_motors_toggle', 'aruco_toggle',
})


@app.route('/')
def index():
    return Response(_HTML, mimetype='text/html')


@app.route('/api/state')
def api_state():
    """SSE stream — pushes latest robot state at 10 Hz."""
    def _gen():
        while True:
            state = _gs.get()
            if state is not None:
                yield f"data: {json.dumps(state)}\n\n"
            else:
                yield ":\n\n"   # SSE keepalive comment — no onmessage fired
            time.sleep(0.1)
    return Response(
        _gen(),
        mimetype='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'},
    )


@app.route('/stream/fpv')
def stream_fpv():
    """MJPEG stream from the local FPV capture card."""
    def _gen():
        while True:
            data = _fpv.get_jpeg() if _fpv else None
            if data:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
                time.sleep(0.033)   # ~30 fps cap
            else:
                time.sleep(0.5)
    return Response(
        _gen(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-store'},
    )


@app.route('/stream/<cam>')
def stream_cam(cam):
    """Stub for robot-side cameras — returns a grey placeholder frame."""
    placeholder: Optional[bytes] = None
    if _CV2:
        import numpy as np
        img = np.full((240, 320, 3), 28, dtype=np.uint8)
        cv2.putText(img, f"{cam}", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 80), 1)
        cv2.putText(img, "not available at", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1)
        cv2.putText(img, "ground station", (10, 148),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1)
        _, buf = cv2.imencode('.jpg', img)
        placeholder = buf.tobytes()

    def _gen():
        while True:
            if placeholder:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n'
                       + placeholder + b'\r\n')
            time.sleep(5.0)   # very low rate — placeholder only
    return Response(
        _gen(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-store'},
    )


@app.route('/api/cmd', methods=['POST'])
def api_cmd():
    body = request.json or {}
    cmd  = body.get('cmd', '')
    if cmd not in _UPLINK_CMDS:
        return jsonify({'ok': True, 'ignored': True})
    try:
        _cmd_q.put_nowait({"t": "cmd", **body})
        return jsonify({'ok': True})
    except queue.Full:
        return jsonify({'ok': False, 'error': 'TX queue full'}), 503


@app.route('/api/logs')
def api_logs():
    return jsonify({'lines': []})


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def _local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


def main():
    global _fpv, _HTML

    ap = argparse.ArgumentParser(
        description="HackyRacingRobot ground station web dashboard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Requires")[0].strip(),
    )
    ap.add_argument("--backend", default="real",
                    choices=["real", "network", "fake"],
                    help="Serial backend: real=SiK radio, network=TCP, fake=synthetic data")
    # real backend
    ap.add_argument("--serial-port",  default="/dev/ttyUSB0",
                    help="SiK serial port [real backend] (default: /dev/ttyUSB0)")
    ap.add_argument("--baud",         default=57600, type=int,
                    help="SiK baud rate [real backend] (default: 57600)")
    # network backend
    ap.add_argument("--network-host", default="192.168.1.1",
                    help="Robot IP [network backend] (default: 192.168.1.1)")
    ap.add_argument("--network-port", default=5010, type=int,
                    help="TCP port [network backend] (default: 5010)")
    # web server
    ap.add_argument("--web-port",     default=5000, type=int)
    ap.add_argument("--web-host",     default="0.0.0.0")
    # FPV
    ap.add_argument("--no-fpv",       action="store_true")
    ap.add_argument("--fpv-device",   default="0",
                    help="FPV capture device index or path (default: 0)")
    # NTRIP
    ap.add_argument("--no-ntrip",     action="store_true")
    ap.add_argument("--ntrip-host",   default="www.rtk2go.com")
    ap.add_argument("--ntrip-port",   default=2101, type=int)
    ap.add_argument("--ntrip-mount",  default="CAMBRIDGE")
    ap.add_argument("--ntrip-user",
                    default=os.environ.get("NTRIP_USER", "user@example.com"))
    ap.add_argument("--ntrip-password",
                    default=os.environ.get("NTRIP_PASSWORD", "none"))
    ap.add_argument("--ntrip-lat",    default=52.2,  type=float)
    ap.add_argument("--ntrip-lon",    default=0.1,   type=float)
    ap.add_argument("--ntrip-alt",    default=20.0,  type=float)
    ap.add_argument("--log-level",    default="INFO")
    args = ap.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # ── HTML ──────────────────────────────────────────────────────────────────
    html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "ground_station.html")
    if not os.path.exists(html_path):
        sys.exit(
            f"HTML not found: {html_path}\n"
            f"Generate it first:\n"
            f"  python3 tools/build_gs_html.py"
        )
    with open(html_path, encoding="utf-8") as f:
        _HTML = f.read()

    # ── Radio backend ─────────────────────────────────────────────────────────
    rtcm_q: queue.Queue = queue.Queue(maxsize=200)

    if args.backend == "real":
        if not _SERIAL:
            sys.exit("pyserial not installed — pip install pyserial")
        try:
            link = SerialLink(args.serial_port, args.baud, rtcm_q, _cmd_q)
        except Exception as e:
            sys.exit(f"Cannot open {args.serial_port}: {e}")
        link.start()
        log.info("Backend: serial  %s @ %d baud", args.serial_port, args.baud)

    elif args.backend == "network":
        link = NetworkLink(args.network_host, args.network_port, rtcm_q, _cmd_q)
        link.start()
        log.info("Backend: network  %s:%d", args.network_host, args.network_port)

    else:  # fake
        link = FakeLink(rtcm_q, _cmd_q)
        link.start()
        log.info("Backend: fake  (synthetic data — no hardware)")
        args.no_ntrip = True   # pointless to send RTCM to /dev/null

    # ── NTRIP ─────────────────────────────────────────────────────────────────
    ntrip = None
    if not args.no_ntrip:
        ntrip = NtripClient(
            host     = args.ntrip_host,
            port     = args.ntrip_port,
            mount    = args.ntrip_mount,
            user     = args.ntrip_user,
            password = args.ntrip_password,
            lat      = args.ntrip_lat,
            lon      = args.ntrip_lon,
            alt      = args.ntrip_alt,
            rtcm_q   = rtcm_q,
        )
        log.info("NTRIP: %s/%s", args.ntrip_host, args.ntrip_mount)
    else:
        _gs.ntrip_status = "disabled"

    # ── FPV ───────────────────────────────────────────────────────────────────
    if not args.no_fpv:
        _fpv = FpvCapture(args.fpv_device)

    # ── Shutdown ──────────────────────────────────────────────────────────────
    def _shutdown(sig, frame):
        log.info("Shutting down")
        link.stop()
        if ntrip: ntrip.stop()
        if _fpv:  _fpv.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── Start ─────────────────────────────────────────────────────────────────
    ip = _local_ip()
    print(f"\n  Ground station → http://{ip}:{args.web_port}/")
    print(f"  Backend        : {args.backend}")
    if args.backend == "real":
        print(f"  SiK radio      : {args.serial_port} @ {args.baud} baud")
    elif args.backend == "network":
        print(f"  Network        : {args.network_host}:{args.network_port}")
    else:
        print(f"  Fake data      : synthetic telemetry at 5 Hz")
    print(f"  FPV            : {'device ' + args.fpv_device if not args.no_fpv else 'disabled'}")
    print(f"  NTRIP          : {'disabled' if args.no_ntrip else args.ntrip_host + '/' + args.ntrip_mount}")
    print()

    app.run(host=args.web_host, port=args.web_port,
            threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
