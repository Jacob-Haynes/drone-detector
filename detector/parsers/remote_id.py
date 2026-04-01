"""Parse FAA/EU Remote ID (ASTM F3411) from 802.11 beacon frames.

Drones compliant with FAA Remote ID or EU regulations broadcast
OpenDroneID message packs in 802.11 Neighbor Awareness Networking
(NAN) action frames or vendor-specific IEs.

Reference: https://github.com/opendroneid
"""

import logging
import struct

from detector.models import DroneIDData

logger = logging.getLogger(__name__)

# OpenDroneID uses the ASTM OUI in vendor-specific action frames
# or the CSD (Common Service Discovery) type in NAN frames.
# The message pack starts with a header byte indicating the message type.

# Message types per ASTM F3411-22a
MSG_BASIC_ID = 0x0
MSG_LOCATION = 0x1
MSG_AUTH = 0x2
MSG_SELF_ID = 0x3
MSG_SYSTEM = 0x4
MSG_OPERATOR_ID = 0x5
MSG_PACK = 0xF

# OpenDroneID vendor-specific IE OUI (ASTM)
OPENDRONEID_OUI = b"\xFA\x0B\xBC"


def parse_vendor_ie(oui: bytes, payload: bytes) -> DroneIDData | None:
    """Parse an OpenDroneID vendor-specific IE payload."""
    if oui != OPENDRONEID_OUI:
        return None

    if len(payload) < 25:
        return None

    try:
        return _parse_message_pack(payload)
    except Exception:
        logger.debug("Failed to parse OpenDroneID payload (%d bytes)", len(payload))
        return None


def _parse_message_pack(data: bytes) -> DroneIDData | None:
    """Parse an OpenDroneID message pack containing multiple messages."""
    result = DroneIDData()
    has_data = False

    offset = 0
    # Message pack header: 1 byte (type=0xF in upper nibble, proto version in lower)
    if len(data) > 0 and (data[0] >> 4) == MSG_PACK:
        # Pack: header byte, then count byte, then N x 25-byte messages
        if len(data) < 2:
            return None
        count = data[1]
        offset = 2
        for _ in range(count):
            if offset + 25 > len(data):
                break
            msg = data[offset : offset + 25]
            if _parse_single_message(msg, result):
                has_data = True
            offset += 25
    else:
        # Single message (25 bytes)
        if _parse_single_message(data[:25], result):
            has_data = True

    return result if has_data else None


def _parse_single_message(msg: bytes, result: DroneIDData) -> bool:
    """Parse a single 25-byte OpenDroneID message into result. Returns True if data extracted."""
    if len(msg) < 25:
        return False

    msg_type = (msg[0] >> 4) & 0xF

    if msg_type == MSG_BASIC_ID:
        # Bytes 1-20: UAS ID (serial number), null-padded ASCII
        serial = msg[1:21].split(b"\x00")[0].decode("ascii", errors="replace")
        if serial:
            result.serial = serial
            return True

    elif msg_type == MSG_LOCATION:
        # Byte 1: status/flags
        # Bytes 2-3: direction (uint16, 0.01 deg), speed (uint16, 0.25 m/s)
        # Bytes 4-7: latitude (int32, 1e-7 deg)
        # Bytes 8-11: longitude (int32, 1e-7 deg)
        # Bytes 12-13: altitude (uint16 - 1000, 0.5m increments)
        heading = struct.unpack_from("<H", msg, 2)[0] * 0.01
        speed = struct.unpack_from("<H", msg, 4)[0] * 0.25
        lat_i = struct.unpack_from("<i", msg, 6)[0]
        lon_i = struct.unpack_from("<i", msg, 10)[0]
        alt_raw = struct.unpack_from("<H", msg, 14)[0]

        lat = lat_i / 1e7
        lon = lon_i / 1e7
        alt = (alt_raw * 0.5) - 1000.0

        if abs(lat) <= 90 and abs(lon) <= 180:
            result.drone_lat = lat
            result.drone_lon = lon
            result.drone_alt = alt
            result.speed = speed
            result.heading = heading
            return True

    elif msg_type == MSG_SYSTEM:
        # Operator location
        # Bytes 1: flags
        # Bytes 2-5: operator latitude (int32, 1e-7 deg)
        # Bytes 6-9: operator longitude (int32, 1e-7 deg)
        op_lat_i = struct.unpack_from("<i", msg, 2)[0]
        op_lon_i = struct.unpack_from("<i", msg, 6)[0]

        op_lat = op_lat_i / 1e7
        op_lon = op_lon_i / 1e7

        if abs(op_lat) <= 90 and abs(op_lon) <= 180 and (op_lat != 0 or op_lon != 0):
            result.operator_lat = op_lat
            result.operator_lon = op_lon
            return True

    return False