"""Shared utility helpers used by robot_daemon, robot_web, robot_mobile, and robot_gui."""

import configparser
import socket


def _cfg(cfg, section: str, key: str, fallback, cast=str):
    """Read a config value with type casting and a safe fallback."""
    try:
        raw = cfg.get(section, key).strip()
        return cast(raw) if raw else fallback
    except (configparser.NoSectionError, configparser.NoOptionError, ValueError):
        return fallback


def _local_ip() -> str:
    """Return the local LAN IP address, or 'localhost' if unavailable."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('8.8.8.8', 80))
            return s.getsockname()[0]
    except OSError:
        return 'localhost'
