#!/usr/bin/env python3
"""
tools/serial_telemetry.py — SiK radio telemetry bridge for HackyRacingRobot.

Owns the SiK radio serial port exclusively, replacing gnss/rtcm_serial.py
when the SiK link is active.  Set [rtcm] disabled = true in robot.ini.

Downlink (robot → ground station)
----------------------------------
Newline-delimited JSON at [telemetry_radio] telemetry_hz (default 5 Hz).
LiDAR packets injected at 1 Hz when enabled (switchable at runtime).

Packet types ("t" field):

  {"t":"state","ts":1234.5,"mode":"MANUAL","rc":true,
   "drv":[0.0,0.0],
   "telem":{"v":12.1,"i":1.2,"bt":35.1,"lt":32.0,"rt":31.5,
            "lf":false,"rf":false,"hdg":180.0,"pit":2.1,"rol":-0.5},
   "gps":{"lat":52.123,"lon":0.456,"alt":10.2,"spd":0.0,"hdg":null,
          "fix":1,"fqn":"GPS","herr":null,"sats":8,"hdop":1.2,
          "ntrip":"connected"},
   "sys":{"cpu":18.5,"temp":55.0,"mem":42.1,"disk":12.3},
   "nav":{"st":"IDLE","gate":0,"wp":0,"dist":null,"bear":null,
          "berr":null,"tags":0},
   "flags":{"lidar_ok":true,"gps_ok":true,"cam_ok":true,
            "data_log":false,"speed":0.25}}

  {"t":"lidar","ts":1234.5,
   "a":[0,5,10,...],    <- angles in degrees (integers, every lidar_step deg)
   "d":[1200,1350,...]} <- distances in mm (integers)

Uplink (ground station -> robot)
---------------------------------
The SiK radio is full-duplex.  Incoming bytes are scanned for newline-
delimited JSON lines.  Any bytes that are NOT valid JSON lines are treated
as raw RTCM3 and forwarded to the TAU1308 via robot.inject_rtcm().

This mirrors how gnss/rtcm_serial.py works — the ground station dashboard
can send raw RTCM bytes and/or interleave JSON control messages.

Supported uplink JSON messages:
  {"t":"ping"}   -> ignored (keepalive)

robot.ini additions
-------------------
  [telemetry_radio]
  disabled     = false
  port         = /dev/ttyUSB1   ; SiK radio port (check: ls /dev/ttyUSB*)
  baud         = 57600          ; must match SiK ATS1 param on both radios
  telemetry_hz = 5              ; state packet downlink rate
  lidar        = true           ; include subsampled LiDAR at 1 Hz
  lidar_step   = 5              ; degrees between LiDAR samples (1-45)

  [rtcm]
  disabled     = true           ; serial_telemetry.py owns the SiK port

Usage
-----
  python3 tools/serial_telemetry.py
  python3 tools/serial_telemetry.py --port /dev/ttyUSB1 --baud 115200
  python3 tools/serial_telemetry.py --no-lidar
  python3 tools/serial_telemetry.py --no-camera --no-gps   # bench test
"""

import argparse
import configparser
import json
import logging
import os
import signal
import sys
import threading
import time

log = logging.getLogger("serial_telemetry")


# ── config helpers ────────────────────────────────────────────────────────────

def _cfg(cfg, section, key, fallback, cast=str):
    try:
        v = cfg.get(section, key).split('#')[0].strip()
        return cast(v) if v else fallback
    except Exception:
        return fallback


def _cfg_bool(cfg, section, key, fallback: bool) -> bool:
    try:
        v = cfg.get(section, key).split('#')[0].strip().lower()
        return v not in ('false', '0', 'no', 'off')
    except Exception:
        return fallback


# ── state serialisers ─────────────────────────────────────────────────────────

def _state_to_dict(state) -> dict:
    t = state.telemetry
    g = state.gps
    s = state.system
    return {
        "t":    "state",
        "ts":   round(time.monotonic(), 3),
        "mode": state.mode.name,
        "rc":   state.rc_active,
        "drv":  [round(state.drive.left, 3), round(state.drive.right, 3)],
        "telem": {
            "v":   round(t.voltage, 2),
            "i":   round(t.current, 2),
            "bt":  round(t.board_temp, 1),
            "lt":  round(t.left_temp, 1),
            "rt":  round(t.right_temp, 1),
            "lf":  t.left_fault,
            "rf":  t.right_fault,
            "hdg": round(t.heading, 1) if t.heading is not None else None,
            "pit": round(t.pitch,   1) if t.pitch   is not None else None,
            "rol": round(t.roll,    1) if t.roll    is not None else None,
        },
        "gps": {
            "lat":   round(g.latitude,  7) if g.latitude  is not None else None,
            "lon":   round(g.longitude, 7) if g.longitude is not None else None,
            "alt":   round(g.altitude,  1) if g.altitude  is not None else None,
            "spd":   round(g.speed,     2) if g.speed     is not None else None,
            "hdg":   round(g.heading,   1) if g.heading   is not None else None,
            "fix":   g.fix_quality,
            "fqn":   g.fix_quality_name,
            "herr":  round(g.h_error_m, 3) if g.h_error_m is not None else None,
            "sats":  g.satellites,
            "sats_view": g.satellites_view,
            "hdop":  round(g.hdop, 2)      if g.hdop      is not None else None,
            "ntrip": g.ntrip_status,
            "sat_data": g.satellites_data,
        },
        "sys": {
            "cpu":  round(s.cpu_percent, 1),
            "temp": round(s.cpu_temp_c,  1),
            "mem":  round(s.mem_percent, 1),
            "disk": round(s.disk_percent, 1),
        },
        "nav": {
            "st":   state.nav_state,
            "gate": state.nav_gate,
            "wp":   state.nav_wp,
            "dist": round(state.nav_wp_dist,     1) if state.nav_wp_dist     is not None else None,
            "bear": round(state.nav_wp_bear,     1) if state.nav_wp_bear     is not None else None,
            "berr": round(state.nav_bearing_err, 1) if state.nav_bearing_err is not None else None,
            "tags": state.nav_tags_visible,
        },
        "flags": {
            "lidar_ok":  state.lidar_ok,
            "gps_ok":    state.gps_ok,
            "cam_ok":    state.camera_ok,
            "cam_rec":   state.cam_recording,
            "data_log":  state.data_logging,
            "no_motors": state.no_motors,
            "speed":     round(state.speed_scale, 2),
        },
    }


def _lidar_to_dict(scan, step: int) -> dict:
    angles_out: list[int] = []
    dists_out:  list[int] = []

    if scan.angles and scan.distances:
        lut: dict[int, float] = {}
        for a, d in zip(scan.angles, scan.distances):
            deg = int(round(a)) % 360
            if deg not in lut or d < lut[deg]:
                lut[deg] = d
        for deg in range(0, 360, step):
            angles_out.append(deg)
            dists_out.append(int(lut.get(deg, 0)))

    return {
        "t":  "lidar",
        "ts": round(time.monotonic(), 3),
        "a":  angles_out,
        "d":  dists_out,
    }


# ── bridge ────────────────────────────────────────────────────────────────────

class SerialTelemetry:
    """
    Owns the SiK radio serial port.

    TX thread  — serialises RobotState -> compact JSON -> radio at
                 telemetry_hz.  LiDAR packets at 1 Hz when lidar_enabled.

    RX thread  — reads incoming bytes from the radio.
                 Lines that parse as JSON are handled as control messages.
                 All other bytes are treated as raw RTCM3 and forwarded to
                 the TAU1308 via robot.inject_rtcm().
    """

    def __init__(self, robot, port: str, baud: int,
                 telemetry_hz: float, lidar_enabled: bool, lidar_step: int):
        self._robot        = robot
        self._port         = port
        self._baud         = baud
        self._hz           = telemetry_hz
        self.lidar_enabled = lidar_enabled   # public — toggle at runtime
        self._lidar_step   = lidar_step

        self._ser          = None
        self._stop         = threading.Event()
        self._tx_thread: threading.Thread | None = None
        self._rx_thread: threading.Thread | None = None
        self._tx_lock      = threading.Lock()

        # Stats
        self.rtcm_bytes_forwarded = 0
        self.packets_sent         = 0

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start(self, no_serial: bool = False):
        if not no_serial:
            import serial
            log.info("Opening SiK radio on %s @ %d baud", self._port, self._baud)
            self._ser = serial.Serial(self._port, self._baud,
                                      timeout=0.05, write_timeout=1.0)
            time.sleep(0.3)
            self._ser.reset_input_buffer()
            self._rx_thread = threading.Thread(target=self._rx_loop,
                                               daemon=True, name="sik_rx")
            self._rx_thread.start()
        else:
            log.info("SiK radio disabled — TCP-only mode")

        self._tx_thread = threading.Thread(target=self._tx_loop,
                                           daemon=True, name="sik_tx")
        self._tx_thread.start()
        log.info("SiK bridge started  lidar=%s step=%d°",
                 self.lidar_enabled, self._lidar_step)

    def stop(self):
        self._stop.set()
        for t in (self._tx_thread, self._rx_thread):
            if t and t.is_alive():
                t.join(timeout=2.0)
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        log.info("SiK bridge stopped  rtcm_fwd=%d B  pkts_sent=%d",
                 self.rtcm_bytes_forwarded, self.packets_sent)

    # ── TX ────────────────────────────────────────────────────────────────────

    def _send_json(self, obj: dict):
        try:
            data = (json.dumps(obj, separators=(',', ':')) + '\n').encode()
            if self._ser is not None:
                with self._tx_lock:
                    self._ser.write(data)
            self.packets_sent += 1
        except Exception as e:
            log.warning("SiK TX error: %s", e)

    def _tx_loop(self):
        period       = 1.0 / self._hz
        lidar_period = 1.0
        stats_period = 10.0
        next_tick    = time.monotonic()
        next_lidar   = time.monotonic()
        next_stats   = time.monotonic() + stats_period

        while not self._stop.is_set():
            now = time.monotonic()

            # State packet
            try:
                self._send_json(_state_to_dict(self._robot.get_state()))
            except Exception as e:
                log.warning("State serialise error: %s", e)

            # LiDAR packet (1 Hz, when enabled)
            if self.lidar_enabled and now >= next_lidar:
                try:
                    scan = self._robot.get_state().lidar
                    self._send_json(_lidar_to_dict(scan, self._lidar_step))
                except Exception as e:
                    log.warning("LiDAR serialise error: %s", e)
                next_lidar = now + lidar_period

            # Periodic TX stats (10 s)
            if now >= next_stats:
                log.info("SiK TX: %d packets sent  rtcm_fwd=%d B",
                         self.packets_sent, self.rtcm_bytes_forwarded)
                next_stats = now + stats_period

            next_tick += period
            sleep_for  = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()

    # ── RX ────────────────────────────────────────────────────────────────────

    def _rx_loop(self):
        """
        Scan the uplink byte stream for newline-terminated lines.

        Lines that start with '{' and parse as JSON -> control message.
        Everything else -> raw RTCM3 bytes, forwarded to TAU1308.

        RTCM3 is binary (0xD3 preamble) and will almost never form valid
        UTF-8 JSON.  The rare case where a RTCM payload byte is 0x0A
        (newline) is handled gracefully: the fragment fails JSON parsing
        and is forwarded as raw bytes.
        """
        buf = bytearray()

        while not self._stop.is_set():
            try:
                chunk = self._ser.read(512)
            except Exception as e:
                log.warning("SiK RX error: %s", e)
                time.sleep(1.0)
                continue

            if not chunk:
                continue

            buf.extend(chunk)

            # Process all complete lines
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line_s = line.strip()
                if not line_s:
                    continue

                if line_s.startswith(b'{'):
                    try:
                        msg = json.loads(line_s)
                        self._handle_json(msg)
                        continue
                    except json.JSONDecodeError:
                        pass

                # Not JSON — forward as RTCM (restore the newline we split on)
                self._forward_rtcm(bytes(line) + b'\n')

            # Flush large non-JSON accumulations (RTCM without 0x0A bytes)
            if len(buf) > 512 and not buf.lstrip().startswith(b'{'):
                self._forward_rtcm(bytes(buf))
                buf = bytearray()

    def _forward_rtcm(self, data: bytes):
        if not data:
            return
        ok = self._robot.inject_rtcm(data)
        if ok:
            self.rtcm_bytes_forwarded += len(data)
            log.debug("RTCM fwd %d B (total %d B)",
                      len(data), self.rtcm_bytes_forwarded)
        else:
            log.debug("RTCM dropped — GPS unavailable (%d B)", len(data))

    def _handle_json(self, msg: dict):
        t = msg.get("t")
        if t == "ping":
            pass  # keepalive

        elif t == "cmd":
            # Command forwarded from the ground station browser
            cmd = msg.get("cmd", "")
            log.info("Uplink command: %s", cmd)
            try:
                self._dispatch_cmd(cmd, msg)
            except Exception as e:
                log.warning("Command dispatch error: %s", e)

        else:
            log.debug("Unknown uplink JSON type: %r", t)

    def _dispatch_cmd(self, cmd: str, body: dict):
        """Execute a remote command on the Robot instance."""
        from robot_daemon import RobotMode
        r = self._robot
        if   cmd == "estop":            r.estop()
        elif cmd == "reset":            r.reset_estop()
        elif cmd == "data_log_toggle":
            r.stop_data_log() if r.is_data_logging() else r.start_data_log()
        elif cmd == "gps_bookmark":     r.bookmark_gps()
        elif cmd == "record_start":     r.start_cam_recording()
        elif cmd == "record_stop":      r.stop_cam_recording()
        elif cmd == "record_toggle":
            r.stop_cam_recording() if r.is_cam_recording() else r.start_cam_recording()
        elif cmd == "bench_toggle":
            r.set_bench(not r.get_state().bench_enabled)
        elif cmd == "no_motors_toggle":
            r.set_no_motors(not r.get_state().no_motors)
        elif cmd == "aruco_toggle":
            r.toggle_aruco(cam=body.get("cam", "all"))
        elif cmd == "set_mode":
            mode = body.get("mode", "")
            if   mode == "MANUAL": r.set_mode(RobotMode.MANUAL)
            elif mode == "AUTO":   r.set_mode(RobotMode.AUTO)
            else: log.warning("Unknown mode: %r", mode)
        else:
            log.debug("Unhandled remote command: %r", cmd)


# ── TCP bridge (for network backend testing) ──────────────────────────────────

def _start_tcp_bridge(bridge: SerialTelemetry, port: int):
    """
    Expose the SiK link over a TCP server so ground_station.py can connect
    with --backend network instead of needing a physical serial port on the
    ground station machine.

    Each connected client gets:
      - All downlink JSON lines (tee'd from the TX thread)
      - Its uplink bytes forwarded to the robot via bridge._forward_rtcm /
        bridge._handle_json (same path as the SiK RX loop)

    Useful for:
      - Testing on a single machine (loopback)
      - LAN testing before SiK radios arrive
      - Connecting a laptop on the same WiFi network
    """
    import socket as _socket

    clients: list = []
    clients_lock  = threading.Lock()

    def _tee_tx(original_send_json):
        """Wrap bridge._send_json to also push each JSON line to TCP clients."""
        def _wrapped(obj: dict):
            original_send_json(obj)
            line = (json.dumps(obj, separators=(',', ':')) + '\n').encode()
            with clients_lock:
                dead = []
                for sock in clients:
                    try:
                        sock.sendall(line)
                    except Exception:
                        dead.append(sock)
                for s in dead:
                    clients.remove(s)
                    try: s.close()
                    except Exception: pass
        return _wrapped

    # Monkey-patch the bridge's _send_json so TCP clients get every packet
    bridge._send_json = _tee_tx(bridge._send_json)

    def _client_rx(sock: _socket.socket, addr):
        """Read uplink bytes from a TCP client and inject into the bridge."""
        buf = b""
        try:
            while True:
                chunk = sock.recv(512)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line_s = line.strip()
                    if not line_s:
                        continue
                    if line_s.startswith(b'{'):
                        try:
                            bridge._handle_json(json.loads(line_s))
                            continue
                        except Exception:
                            pass
                    bridge._forward_rtcm(bytes(line) + b'\n')
                if len(buf) > 512 and not buf.lstrip().startswith(b'{'):
                    bridge._forward_rtcm(bytes(buf))
                    buf = b""
        except Exception:
            pass
        finally:
            with clients_lock:
                if sock in clients:
                    clients.remove(sock)
            try: sock.close()
            except Exception: pass
            log.info("TCP bridge: client disconnected %s:%d", *addr)

    def _accept_loop():
        srv = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
        srv.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", port))
        srv.listen(4)
        log.info("TCP bridge listening on port %d  "
                 "(ground_station.py --backend network --network-port %d)", port, port)
        while True:
            try:
                sock, addr = srv.accept()
                log.info("TCP bridge: client connected %s:%d", *addr)
                with clients_lock:
                    clients.append(sock)
                threading.Thread(target=_client_rx, args=(sock, addr),
                                 daemon=True, name=f"tcp_rx_{addr[1]}").start()
            except Exception as e:
                log.warning("TCP bridge accept error: %s", e)

    threading.Thread(target=_accept_loop, daemon=True, name="tcp_bridge").start()


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="SiK radio telemetry bridge for HackyRacingRobot")
    ap.add_argument("--config",     default="robot.ini")
    ap.add_argument("--port",       default=None,
                    help="SiK radio serial port (overrides robot.ini)")
    ap.add_argument("--baud",       default=None, type=int)
    ap.add_argument("--hz",         default=None, type=float,
                    help="Telemetry packet rate Hz (overrides robot.ini)")
    ap.add_argument("--no-lidar",   action="store_true")
    ap.add_argument("--lidar-step", default=None, type=int,
                    help="LiDAR angular step degrees (default 5)")
    ap.add_argument("--no-camera",  action="store_true")
    ap.add_argument("--no-gps",     action="store_true",
                    help="Disable GPS — RTCM uplink will be dropped")
    ap.add_argument("--no-motors",  action="store_true")
    ap.add_argument("--no-yukon",   action="store_true",
                    help="Skip Yukon connection — useful when robot_dashboard.py "
                         "is already running and owns /dev/ttyACM0")
    ap.add_argument("--no-serial",  action="store_true",
                    help="Skip opening the SiK serial port entirely — "
                         "useful for LAN/TCP-only testing without a radio")
    ap.add_argument("--tcp-port",   default=None, type=int,
                    help="Also expose the SiK link over TCP on this port "
                         "(for ground_station.py --backend network). "
                         "Example: --tcp-port 5010")
    ap.add_argument("--log-level",  default="INFO")
    args = ap.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    cfg = configparser.ConfigParser()
    cfg.read(args.config)

    sik_port   = args.port       or _cfg(cfg, "telemetry_radio", "port",         "/dev/ttyUSB1")
    sik_baud   = args.baud       or _cfg(cfg, "telemetry_radio", "baud",         57600,  cast=int)
    telem_hz   = args.hz         or _cfg(cfg, "telemetry_radio", "telemetry_hz", 5.0,    cast=float)
    lidar_step = args.lidar_step or _cfg(cfg, "telemetry_radio", "lidar_step",   5,      cast=int)
    lidar_on   = (not args.no_lidar and
                  _cfg_bool(cfg, "telemetry_radio", "lidar", True))

    log.info("SiK port=%s  baud=%d  hz=%.1f  lidar=%s (step=%d°)",
             sik_port, sik_baud, telem_hz, lidar_on, lidar_step)

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from robot_daemon import Robot, setup_logging
    setup_logging()

    # Robot.__init__ takes explicit keyword args, not a config dict/object.
    # Extract them here, applying --no-camera / --no-gps CLI overrides.
    camera_disabled = args.no_camera or _cfg_bool(cfg, "camera", "disabled", False)
    gps_disabled    = args.no_gps    or _cfg_bool(cfg, "gps",    "disabled", False)

    robot = Robot(
        yukon_port     = '' if args.no_yukon else
                         (lambda v: None if v.lower() in ("auto", "") else v)(
                             _cfg(cfg, "robot", "yukon_port", "auto")),
        ibus_port      = _cfg(cfg, "robot", "ibus_port",       "/dev/ttyAMA3"),
        lidar_port     = _cfg(cfg, "lidar", "port",            "/dev/ttyAMA0"),
        gps_port       = _cfg(cfg, "gps",   "port",            "/dev/ttyUSB0"),
        ntrip_host     = "" if _cfg_bool(cfg, "ntrip", "disabled", False) else _cfg(cfg, "ntrip", "host", ""),
        ntrip_port     = _cfg(cfg, "ntrip", "port",            2101,  int),
        ntrip_mount    = _cfg(cfg, "ntrip", "mount",           ""),
        ntrip_user     = _cfg(cfg, "ntrip", "user",            ""),
        ntrip_password = _cfg(cfg, "ntrip", "password",        ""),
        ntrip_lat      = _cfg(cfg, "ntrip", "lat",             0.0,   float),
        ntrip_lon      = _cfg(cfg, "ntrip", "lon",             0.0,   float),
        ntrip_height   = _cfg(cfg, "ntrip", "height",          0.0,   float),
        rtcm_port      = "",   # serial_telemetry owns the SiK port; RTCM comes via uplink
        enable_camera  = not camera_disabled,
        enable_lidar   = not _cfg_bool(cfg, "lidar", "disabled", False),
        enable_gps     = not gps_disabled,
        cam_width      = _cfg(cfg, "camera", "width",          640,   int),
        cam_height     = _cfg(cfg, "camera", "height",         480,   int),
        cam_fps        = _cfg(cfg, "camera", "fps",            30,    int),
        cam_rotation   = _cfg(cfg, "camera", "rotation",       0,     int),
        enable_aruco   = _cfg_bool(cfg, "aruco", "enabled",    False),
        aruco_dict     = _cfg(cfg, "aruco",  "dict",           "DICT_4X4_1000"),
        aruco_calib    = _cfg(cfg, "aruco",  "calib_file",     ""),
        aruco_tag_size = _cfg(cfg, "aruco",  "tag_size",       0.15,  float),
        throttle_ch    = _cfg(cfg, "rc", "throttle_ch",        3,     int),
        steer_ch       = _cfg(cfg, "rc", "steer_ch",           1,     int),
        mode_ch        = _cfg(cfg, "rc", "mode_ch",            5,     int),
        speed_ch       = _cfg(cfg, "rc", "speed_ch",           6,     int),
        auto_type_ch   = _cfg(cfg, "rc", "auto_type_ch",       7,     int),
        gps_log_ch     = _cfg(cfg, "rc", "gps_log_ch",         8,     int),
        gps_bookmark_ch= _cfg(cfg, "rc", "gps_bookmark_ch",    10,    int),
        gps_log_dir    = _cfg(cfg, "gps", "log_dir",           ""),
        gps_log_hz     = _cfg(cfg, "gps", "log_hz",            5.0,   float),
        deadzone       = _cfg(cfg, "rc", "deadzone",           30,    int),
        failsafe_s     = _cfg(cfg, "rc", "failsafe_s",         0.5,   float),
        speed_min      = _cfg(cfg, "rc", "speed_min",          0.25,  float),
        control_hz     = _cfg(cfg, "rc", "control_hz",         50,    int),
        no_motors      = args.no_motors,
        rec_dir        = _cfg(cfg, "output", "videos_dir",     ""),
        max_recording_minutes = _cfg(cfg, "output", "max_recording_minutes", 0.0, float),
        data_log_dir   = _cfg(cfg, "output", "data_log_dir",   ""),
    )

    bridge = SerialTelemetry(
        robot         = robot,
        port          = sik_port,
        baud          = sik_baud,
        telemetry_hz  = telem_hz,
        lidar_enabled = lidar_on,
        lidar_step    = lidar_step,
    )

    def _shutdown(sig, frame):
        log.info("Shutting down")
        bridge.stop()
        robot.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    robot.start()

    bridge.start(no_serial=args.no_serial)

    # Optional TCP bridge — lets ground_station.py --backend network connect
    # directly over LAN without needing socat.
    if args.tcp_port:
        _start_tcp_bridge(bridge, args.tcp_port)

    log.info("Running — Ctrl+C to stop")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
