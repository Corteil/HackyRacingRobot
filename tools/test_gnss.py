#!/usr/bin/env python3
"""
test_gnss.py — Unit tests for the gnss/ package.

No hardware required.  All tests use in-process function calls only.

Sections:
  1.  nmea.py — checksum functions
  2.  nmea.py — coordinate parsing
  3.  nmea.py — safe helpers (_safe_float, _safe_int)
  4.  base.py — binary packing helpers (_pack_u8/u16/u32, _fletcher8)
  5.  GNSSBase — initial state
  6.  GNSSBase — GGA sentence parsing
  7.  GNSSBase — RMC sentence parsing
  8.  GNSSBase — GST sentence parsing
  9.  GNSSBase — RTK fix quality
  10. GNSSBase — bad / unknown sentences
  11. NTRIPClient — initial state

Usage:
    python3 tools/test_gnss.py
"""

import math
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _REPO)

from gnss.nmea import (
    FIX_INVALID,
    FIX_GPS,
    FIX_RTK_FIXED,
    FIX_RTK_FLOAT,
    _nmea_checksum,
    _parse_lat,
    _parse_lon,
    _safe_float,
    _safe_int,
    _verify_nmea_checksum,
)
from gnss.base import (
    GNSSBase,
    _fletcher8,
    _pack_u8,
    _pack_u16,
    _pack_u32,
)
from gnss.ntrip import NTRIPClient, NTRIP_DISCONNECTED


# ---------------------------------------------------------------------------
# Test harness
# ---------------------------------------------------------------------------

_passed = 0
_failed = 0


def _check(name, condition, detail=""):
    global _passed, _failed
    if condition:
        print(f"  PASS  {name}")
        _passed += 1
    else:
        info = f"  ({detail})" if detail else ""
        print(f"  FAIL  {name}{info}")
        _failed += 1


def _approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# Section 1: nmea.py — checksum functions
# ---------------------------------------------------------------------------

def test_checksums():
    print("\nSection 1: nmea.py — checksum functions")

    body = "GPGGA,123519"
    cs = _nmea_checksum(body)

    # XOR of each character in "GPGGA,123519"
    expected_xor = 0
    for ch in body:
        expected_xor ^= ord(ch)
    expected_hex = "{:02X}".format(expected_xor)

    _check("_nmea_checksum returns correct XOR hex",
           cs == expected_hex,
           f"got {cs!r}, expected {expected_hex!r}")

    good_sentence = "$GPGGA,123519*" + cs
    _check("_verify_nmea_checksum returns True for correct checksum",
           _verify_nmea_checksum(good_sentence) is True)

    _check("_verify_nmea_checksum returns False for wrong checksum",
           _verify_nmea_checksum("$GPGGA,123519*00") is False)

    _check("_verify_nmea_checksum returns False for sentence without $ or *",
           _verify_nmea_checksum("invalid") is False)


# ---------------------------------------------------------------------------
# Section 2: nmea.py — coordinate parsing
# ---------------------------------------------------------------------------

def test_coordinate_parsing():
    print("\nSection 2: nmea.py — coordinate parsing")

    # 53 deg + 21.6802/60 = 53.361337
    expected_lat = 53 + 21.6802 / 60.0

    result = _parse_lat("5321.6802", "N")
    _check("_parse_lat N positive",
           result is not None and _approx(result, expected_lat, tol=1e-5),
           f"got {result}")

    result = _parse_lat("5321.6802", "S")
    _check("_parse_lat S negative",
           result is not None and _approx(result, -expected_lat, tol=1e-5),
           f"got {result}")

    _check("_parse_lat empty string returns None",
           _parse_lat("", "N") is None)

    _check("_parse_lat non-numeric returns None",
           _parse_lat("bad", "N") is None)

    # 6 deg + 30.3372/60 = 6.50562
    expected_lon = 6 + 30.3372 / 60.0

    result = _parse_lon("00630.3372", "E")
    _check("_parse_lon E positive",
           result is not None and _approx(result, expected_lon, tol=1e-5),
           f"got {result}")

    result = _parse_lon("00630.3372", "W")
    _check("_parse_lon W negative",
           result is not None and _approx(result, -expected_lon, tol=1e-5),
           f"got {result}")


# ---------------------------------------------------------------------------
# Section 3: nmea.py — safe helpers
# ---------------------------------------------------------------------------

def test_safe_helpers():
    print("\nSection 3: nmea.py — safe helpers")

    _check("_safe_float valid string",
           _approx(_safe_float("3.14"), 3.14))
    _check("_safe_float empty string returns None",
           _safe_float("") is None)
    _check("_safe_float non-numeric returns None",
           _safe_float("bad") is None)

    _check("_safe_int valid string",
           _safe_int("42") == 42)
    _check("_safe_int empty string returns None",
           _safe_int("") is None)
    _check("_safe_int non-numeric returns None",
           _safe_int("bad") is None)


# ---------------------------------------------------------------------------
# Section 4: base.py — binary packing helpers
# ---------------------------------------------------------------------------

def test_binary_helpers():
    print("\nSection 4: base.py — binary helpers")

    _check("_pack_u8(0xFF) == b'\\xff'",
           _pack_u8(0xFF) == b'\xff')

    _check("_pack_u8(0x1AB) masked to 8 bits == b'\\xab'",
           _pack_u8(0x1AB) == b'\xab')

    _check("_pack_u16(0x1234) little-endian == b'\\x34\\x12'",
           _pack_u16(0x1234) == b'\x34\x12')

    _check("_pack_u32(0x12345678) little-endian == b'\\x78\\x56\\x34\\x12'",
           _pack_u32(0x12345678) == b'\x78\x56\x34\x12')

    # ck_a = 1+2 = 3,  ck_b = (0+1)+(1+2) = 1+3 = 4
    _check("_fletcher8(b'\\x01\\x02') == (3, 4)",
           _fletcher8(b'\x01\x02') == (3, 4))

    _check("_fletcher8(b'') == (0, 0)",
           _fletcher8(b'') == (0, 0))


# ---------------------------------------------------------------------------
# Section 5: GNSSBase — initial state
# ---------------------------------------------------------------------------

def test_gnss_initial_state():
    print("\nSection 5: GNSSBase — initial state")

    gnss = GNSSBase(None, validate_checksum=False)

    _check("has_fix is False initially",
           gnss.has_fix is False)

    _check("fix_quality == 0 initially",
           gnss.fix_quality == 0)

    _check("latitude is None initially",
           gnss.latitude is None)

    _check("fix_quality_name == 'Invalid' initially",
           gnss.fix_quality_name == "Invalid")

    _check("position == (None, None) initially",
           gnss.position == (None, None))

    _check("h_error_m is None initially",
           gnss.h_error_m is None)

    _check("stats == {'parsed': 0, 'errors': 0, 'rtcm_bytes': 0} initially",
           gnss.stats == {"parsed": 0, "errors": 0, "rtcm_bytes": 0})


# ---------------------------------------------------------------------------
# Section 6: GNSSBase — GGA parsing
# ---------------------------------------------------------------------------

def test_gnss_gga_parsing():
    print("\nSection 6: GNSSBase — GGA parsing")

    gnss = GNSSBase(None, validate_checksum=False)
    sentence = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,"

    result = gnss._parse_sentence(sentence)
    _check("_parse_sentence GGA returns True",
           result is True)

    expected_lat = 53 + 21.6802 / 60.0
    _check("latitude parsed correctly",
           gnss.latitude is not None and _approx(gnss.latitude, expected_lat, tol=0.0001),
           f"got {gnss.latitude}")

    expected_lon = -(6 + 30.3372 / 60.0)
    _check("longitude parsed correctly (West negative)",
           gnss.longitude is not None and _approx(gnss.longitude, expected_lon, tol=0.0001),
           f"got {gnss.longitude}")

    _check("fix_quality == 1",
           gnss.fix_quality == 1)

    _check("satellites == 8",
           gnss.satellites == 8)

    _check("hdop ~= 1.03",
           gnss.hdop is not None and _approx(gnss.hdop, 1.03),
           f"got {gnss.hdop}")

    _check("altitude ~= 61.7",
           gnss.altitude is not None and _approx(gnss.altitude, 61.7),
           f"got {gnss.altitude}")

    _check("has_fix is True after GGA fix_quality=1",
           gnss.has_fix is True)

    _check("fix_quality_name == 'GPS'",
           gnss.fix_quality_name == "GPS")

    _check("position == (latitude, longitude)",
           gnss.position == (gnss.latitude, gnss.longitude))

    _check("stats['parsed'] == 1 after one GGA",
           gnss.stats["parsed"] == 1)


# ---------------------------------------------------------------------------
# Section 7: GNSSBase — RMC parsing
# ---------------------------------------------------------------------------

def test_gnss_rmc_parsing():
    print("\nSection 7: GNSSBase — RMC parsing")

    gnss = GNSSBase(None, validate_checksum=False)
    sentence = "$GPRMC,092750.000,A,5321.6802,N,00630.3372,W,022.4,084.4,230394,"

    result = gnss._parse_sentence(sentence)
    _check("_parse_sentence RMC returns True",
           result is True)

    _check("rmc_valid is True",
           gnss.rmc_valid is True)

    _check("speed_knots ~= 22.4",
           gnss.speed_knots is not None and _approx(gnss.speed_knots, 22.4),
           f"got {gnss.speed_knots}")

    expected_speed_ms = 22.4 * 0.514444
    _check("speed_ms ~= 22.4 * 0.514444",
           gnss.speed_ms is not None and _approx(gnss.speed_ms, expected_speed_ms, tol=0.01),
           f"got {gnss.speed_ms}")

    _check("course ~= 84.4",
           gnss.course is not None and _approx(gnss.course, 84.4),
           f"got {gnss.course}")


# ---------------------------------------------------------------------------
# Section 8: GNSSBase — GST parsing
# ---------------------------------------------------------------------------

def test_gnss_gst_parsing():
    print("\nSection 8: GNSSBase — GST parsing")

    gnss = GNSSBase(None, validate_checksum=False)
    # Parts: p[0]=GPGST p[1]=082356.00 p[2]=1.8 p[3]='' p[4]='' p[5]=''
    #        p[6]=1.7(std_lat) p[7]=1.3(std_lon) p[8]=2.2(std_alt)
    sentence = "$GPGST,082356.00,1.8,,,,1.7,1.3,2.2"

    result = gnss._parse_sentence(sentence)
    _check("_parse_sentence GST returns True",
           result is True)

    _check("std_lat ~= 1.7",
           gnss.std_lat is not None and _approx(gnss.std_lat, 1.7),
           f"got {gnss.std_lat}")

    _check("std_lon ~= 1.3",
           gnss.std_lon is not None and _approx(gnss.std_lon, 1.3),
           f"got {gnss.std_lon}")

    expected_h_error = math.sqrt(1.7 ** 2 + 1.3 ** 2)
    _check("h_error_m ~= sqrt(1.7^2 + 1.3^2)",
           gnss.h_error_m is not None and _approx(gnss.h_error_m, expected_h_error, tol=0.01),
           f"got {gnss.h_error_m}, expected {expected_h_error:.4f}")


# ---------------------------------------------------------------------------
# Section 9: GNSSBase — RTK fix quality
# ---------------------------------------------------------------------------

def test_gnss_rtk_fix():
    print("\nSection 9: GNSSBase — RTK fix quality")

    gnss = GNSSBase(None, validate_checksum=False)
    sentence = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,4,12,0.5,61.7,M,55.2,M,,"

    gnss._parse_sentence(sentence)

    _check("fix_quality == 4 (RTK Fixed)",
           gnss.fix_quality == 4)

    _check("has_rtk_fixed is True",
           gnss.has_rtk_fixed is True)

    _check("has_rtk_float is False",
           gnss.has_rtk_float is False)

    _check("fix_quality_name == 'RTK Fixed'",
           gnss.fix_quality_name == "RTK Fixed")


# ---------------------------------------------------------------------------
# Section 10: GNSSBase — bad / unknown sentences
# ---------------------------------------------------------------------------

def test_gnss_bad_sentences():
    print("\nSection 10: GNSSBase — bad / unknown sentences")

    gnss = GNSSBase(None, validate_checksum=False)

    _check("_parse_sentence unknown type '$GPXXX,data' returns False",
           gnss._parse_sentence("$GPXXX,data") is False)

    _check("_parse_sentence non-NMEA string returns False",
           gnss._parse_sentence("not_nmea") is False)

    _check("_parse_sentence empty string returns False",
           gnss._parse_sentence("") is False)


# ---------------------------------------------------------------------------
# Section 11: NTRIPClient — initial state
# ---------------------------------------------------------------------------

def test_ntrip_initial_state():
    print("\nSection 11: NTRIPClient — initial state")

    client = NTRIPClient(
        "host", 2101, "MOUNT", "user", "pass",
        lat=51.0, lon=-0.1, height=10.0,
    )

    _check("client.status == NTRIP_DISCONNECTED on creation",
           client.status == NTRIP_DISCONNECTED,
           f"got {client.status!r}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    print("=" * 60)
    print("test_gnss.py — gnss/ package unit tests")
    print("=" * 60)

    test_checksums()
    test_coordinate_parsing()
    test_safe_helpers()
    test_binary_helpers()
    test_gnss_initial_state()
    test_gnss_gga_parsing()
    test_gnss_rmc_parsing()
    test_gnss_gst_parsing()
    test_gnss_rtk_fix()
    test_gnss_bad_sentences()
    test_ntrip_initial_state()

    print("\n" + "=" * 60)
    print(f"Results: {_passed} passed, {_failed} failed")
    print("=" * 60)
    sys.exit(0 if _failed == 0 else 1)


if __name__ == "__main__":
    main()
