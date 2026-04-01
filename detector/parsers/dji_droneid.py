"""Parse DJI DroneID from 802.11 beacon frame vendor-specific IEs.

DJI drones broadcast vendor-specific information elements in beacon frames
with OUI 26:D5:B0. The payload contains drone telemetry including GPS
coordinates, altitude, speed, and serial number.

Reference: https://github.com/proto17/dji_droneid
"""

import logging
import struct

from detector.models import DroneIDData

logger = logging.getLogger(__name__)

DJI_OUI = b"\x26\xd5\xb0"

# DroneID message types
MSG_TYPE_DRONE_ID = 0x10


def parse_vendor_ie(oui: bytes, payload: bytes) -> DroneIDData | None:
    """Parse a DJI vendor-specific IE payload into DroneIDData.

    Returns None if the IE is not a DJI DroneID or cannot be parsed.
    """
    if oui != DJI_OUI:
        return None

    if len(payload) < 58:
        return None

    try:
        return _parse_droneid_payload(payload)
    except Exception:
        logger.debug("Failed to parse DJI DroneID payload (%d bytes)", len(payload))
        return None


def _parse_droneid_payload(data: bytes) -> DroneIDData | None:
    """Parse the DJI DroneID binary payload.

    The format is reverse-engineered and may vary across DJI firmware versions.
    Field offsets based on proto17/dji_droneid research.
    """
    if len(data) < 58:
        return None

    msg_type = data[0]

    # Extract serial number (typically at offset 2, 16 bytes, null-terminated ASCII)
    serial_raw = data[2:18]
    serial = serial_raw.split(b"\x00")[0].decode("ascii", errors="replace")

    # GPS coordinates stored as int32 scaled by 1e-7 (like Meshtastic)
    # Offsets vary by message version; common layout:
    drone_lon_i = struct.unpack_from("<i", data, 18)[0]
    drone_lat_i = struct.unpack_from("<i", data, 22)[0]
    drone_alt = struct.unpack_from("<h", data, 26)[0]  # metres relative to takeoff
    speed_x = struct.unpack_from("<h", data, 28)[0]  # cm/s
    speed_y = struct.unpack_from("<h", data, 30)[0]  # cm/s

    drone_lat = drone_lat_i / 1e7
    drone_lon = drone_lon_i / 1e7
    speed = (speed_x**2 + speed_y**2) ** 0.5 / 100.0  # m/s

    # Heading from velocity vector
    import math

    heading = math.degrees(math.atan2(speed_x, speed_y)) % 360

    # Operator/home position (if present, typically at offset 38+)
    operator_lat = 0.0
    operator_lon = 0.0
    home_lat = 0.0
    home_lon = 0.0

    if len(data) >= 50:
        home_lon_i = struct.unpack_from("<i", data, 34)[0]
        home_lat_i = struct.unpack_from("<i", data, 38)[0]
        home_lat = home_lat_i / 1e7
        home_lon = home_lon_i / 1e7

    if len(data) >= 58:
        op_lon_i = struct.unpack_from("<i", data, 42)[0]
        op_lat_i = struct.unpack_from("<i", data, 46)[0]
        operator_lat = op_lat_i / 1e7
        operator_lon = op_lon_i / 1e7

    # Sanity check coordinates
    if abs(drone_lat) > 90 or abs(drone_lon) > 180:
        return None

    return DroneIDData(
        serial=serial,
        drone_lat=drone_lat,
        drone_lon=drone_lon,
        drone_alt=float(drone_alt),
        speed=speed,
        heading=heading,
        operator_lat=operator_lat,
        operator_lon=operator_lon,
        home_lat=home_lat,
        home_lon=home_lon,
    )