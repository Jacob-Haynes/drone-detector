"""Parse DJI DroneID from 802.11 beacon vendor-specific IEs (Wi-Fi-link DJI drones).

DJI aircraft that use Wi-Fi for their video link (Spark, Mavic Air/Pro Wi-Fi mode,
Mini SE/Mini 2 with Wi-Fi, Tello is not one) put DroneID telemetry in a vendor
IE of their beacons. Aircraft on OcuSync/O2/O3/O4 transmit DroneID on their own
radio link instead, which a Wi-Fi card cannot decode (that needs an SDR). Newer
DJI aircraft also broadcast standard Remote ID, see ``remote_id``.

Layout (after the 3-byte OUI 26:37:12), per Kismet's ``dot11_ie_221_dji_droneid``
and the "Drone Security and the Mysterious Case of DJI's DroneID" (NDSS 2023) work::

    u8  vendor_type   (0x58)
    u8  unk1          (0x62)
    u8  unk2          (0x13)
    u8  subcommand    0x10 = flight registration (telemetry), 0x11 = flight purpose

    flight registration (0x10):
      u8  version     1 or 2
      u16 seq
      u16 state_info  bit flags, 0x40 = GPS valid
      16s serial
      s32 lon, s32 lat              radians * 1e7  ->  degrees = raw / 174533.0
      v1: s16 altitude (m), s16 height (m),
          s16 v_north, s16 v_east, s16 v_up (cm/s), s16 pitch, s16 roll, s16 yaw (deg*100),
          s32 home_lon, s32 home_lat, u8 product_type, u8 uuid_len, uuid
      v2: s16 height (dm), s16 altitude (m),
          s16 v_north, s16 v_east, s16 v_up (cm/s), s16 yaw (deg*100),
          u64 gps_time, s32 app_lat, s32 app_lon (pilot/app position),
          s32 home_lon, s32 home_lat, u8 product_type, u8 uuid_len, uuid

    flight purpose (0x11):
      16s serial, u8 drone_id_len, 10s drone_id, u8 purpose_len, purpose...

DJI firmware varies and these fields are reverse-engineered, so only serial and
drone position are treated as reliable. Everything else is best effort and sanity checked.
"""

from __future__ import annotations

import logging
import math
import struct

from detector.models import DroneIDData

logger = logging.getLogger(__name__)

DJI_OUI = b"\x26\x37\x12"
SUBCMD_FLIGHT_REG = 0x10
SUBCMD_FLIGHT_PURPOSE = 0x11
RAW_TO_DEG = 174533.0  # raw = radians * 1e7  =>  deg = raw * 180 / (pi * 1e7)

STATE_GPS_VALID = 0x40

_HEADER = 4  # vendor_type, unk1, unk2, subcommand
_REG_COMMON = struct.Struct("<BHH16sii")  # version, seq, state, serial, lon, lat  (29 bytes)
_V1_TAIL = struct.Struct("<hhhhhhhhii")  # alt, height, vn, ve, vu, pitch, roll, yaw, home_lon, home_lat
_V2_TAIL = struct.Struct("<hhhhhhQiiii")  # height, alt, vn, ve, vu, yaw, gps_time, app_lat, app_lon, home_lon, home_lat


def _deg(raw: int) -> float:
    return raw / RAW_TO_DEG


def _valid(lat: float, lon: float) -> bool:
    return abs(lat) <= 90 and abs(lon) <= 180 and not (lat == 0 and lon == 0)


def _serial(raw: bytes) -> str:
    return raw.split(b"\x00")[0].decode("ascii", errors="replace").strip()


def parse_vendor_ie(oui: bytes, payload: bytes) -> DroneIDData | None:
    """Parse a vendor IE (payload = bytes after the OUI). None if not DJI DroneID or unparseable."""
    if oui != DJI_OUI or len(payload) < _HEADER + 1:
        return None
    subcommand = payload[3]
    record = payload[_HEADER:]
    try:
        if subcommand == SUBCMD_FLIGHT_REG:
            return _parse_flight_reg(record)
        if subcommand == SUBCMD_FLIGHT_PURPOSE:
            return _parse_flight_purpose(record)
    except (struct.error, ValueError):
        logger.debug("Failed to parse DJI DroneID subcommand 0x%02x (%d bytes)", subcommand, len(record))
    return None


def _parse_flight_reg(rec: bytes) -> DroneIDData | None:
    if len(rec) < _REG_COMMON.size:
        return None
    version, _seq, state, serial_raw, raw_lon, raw_lat = _REG_COMMON.unpack_from(rec, 0)
    data = DroneIDData(serial=_serial(serial_raw), source="dji-droneid")

    lat, lon = _deg(raw_lat), _deg(raw_lon)
    # state_info bits are not reliable across firmware, so trust sane coordinates instead
    if _valid(lat, lon):
        data.drone_lat, data.drone_lon = lat, lon

    off = _REG_COMMON.size
    yaw = None
    vn = ve = 0
    if version == 2 and len(rec) >= off + _V2_TAIL.size:
        height_dm, alt, vn, ve, _vu, yaw, _gps_time, app_lat, app_lon, home_lon, home_lat = \
            _V2_TAIL.unpack_from(rec, off)
        data.drone_alt = float(alt)
        data.height_agl = height_dm / 10.0
        if _valid(_deg(app_lat), _deg(app_lon)):
            data.operator_lat, data.operator_lon = _deg(app_lat), _deg(app_lon)
        if _valid(_deg(home_lat), _deg(home_lon)):
            data.home_lat, data.home_lon = _deg(home_lat), _deg(home_lon)
    elif len(rec) >= off + _V1_TAIL.size:
        alt, height, vn, ve, _vu, _pitch, _roll, yaw, home_lon, home_lat = _V1_TAIL.unpack_from(rec, off)
        data.drone_alt = float(alt)
        data.height_agl = float(height)
        if _valid(_deg(home_lat), _deg(home_lon)):
            data.home_lat, data.home_lon = _deg(home_lat), _deg(home_lon)
            # v1 has no pilot position; the home point is where the pilot usually is
            data.operator_lat, data.operator_lon = data.home_lat, data.home_lon

    speed = math.hypot(vn, ve) / 100.0
    if speed < 100:  # >100 m/s means we decoded garbage
        data.speed = speed
    if yaw is not None and -18000 <= yaw <= 18000:
        data.heading = (yaw / 100.0) % 360
    elif data.speed > 0.5:
        data.heading = math.degrees(math.atan2(ve, vn)) % 360

    if not data.serial and not data.has_position:
        return None
    return data


def _parse_flight_purpose(rec: bytes) -> DroneIDData | None:
    if len(rec) < 16:
        return None
    serial = _serial(rec[:16])
    if not serial:
        return None
    return DroneIDData(serial=serial, source="dji-droneid")
