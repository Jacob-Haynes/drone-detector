"""Parse ASTM F3411 / ASD-STAN EN 4709-002 broadcast Remote ID (OpenDroneID).

Remote ID uses the same 25-byte messages over three transports. All three are handled here:

* **Wi-Fi Beacon**: vendor IE 221 with OUI ``FA:0B:BC``, OUI type ``0x0D``, then a
  1-byte message counter and a message pack.
* **Wi-Fi NAN**: public action frame (category 4, action 9, OUI ``50:6F:9A`` type 0x13)
  to ``51:6F:9A:01:00:00``. It carries a Service Descriptor Attribute (ID 0x03) whose
  service id is ``88 69 19 9D 92 09`` (the first 6 bytes of SHA-256 of
  "org.opendroneid.remoteid"). Its service info is a counter byte plus a message pack.
* **Bluetooth LE**: service data for 16-bit UUID ``0xFFFA``, app code ``0x0D``, then
  a counter byte and one message (BT4 legacy) or a message pack (BT5 extended).

Byte layouts follow ``opendroneid-core-c`` (``opendroneid.h``, ``wifi.c``).
"""

from __future__ import annotations

import logging
import struct

from detector.models import DroneIDData

logger = logging.getLogger(__name__)

MSG_SIZE = 25

MSG_BASIC_ID = 0x0
MSG_LOCATION = 0x1
MSG_AUTH = 0x2
MSG_SELF_ID = 0x3
MSG_SYSTEM = 0x4
MSG_OPERATOR_ID = 0x5
MSG_PACK = 0xF

OPENDRONEID_OUI = b"\xFA\x0B\xBC"
OPENDRONEID_OUI_TYPE = 0x0D
NAN_DEST = "51:6f:9a:01:00:00"
WFA_OUI = b"\x50\x6F\x9A"
NAN_OUI_TYPE = 0x13
NAN_SERVICE_ID = bytes([0x88, 0x69, 0x19, 0x9D, 0x92, 0x09])
BLE_SERVICE_UUID16 = 0xFFFA
BLE_SERVICE_UUID = "0000fffa-0000-1000-8000-00805f9b34fb"
BLE_APP_CODE = 0x0D

ID_TYPES = {0: "", 1: "serial", 2: "caa-registration", 3: "utm-uuid", 4: "specific-session"}
UA_TYPES = {
    0: "", 1: "aeroplane", 2: "multirotor", 3: "gyroplane", 4: "hybrid-lift", 5: "ornithopter",
    6: "glider", 7: "kite", 8: "free-balloon", 9: "captive-balloon", 10: "airship",
    11: "parachute", 12: "rocket", 13: "tethered", 14: "ground-obstacle", 15: "other",
}

INVALID_ALT = -1000.0
SPEED_INVALID = 255  # encoded value meaning "unknown"
DIRECTION_INVALID = 361


def _ascii(raw: bytes) -> str:
    return raw.split(b"\x00")[0].decode("ascii", errors="replace").strip()


def _alt(raw: int) -> float:
    return raw * 0.5 - 1000.0


# ── single messages ───────────────────────────────────────────────


def parse_message(msg: bytes, result: DroneIDData) -> bool:
    """Decode one 25-byte message into ``result``. Returns True if it carried useful data."""
    if len(msg) < MSG_SIZE:
        return False
    msg_type = msg[0] >> 4

    if msg_type == MSG_BASIC_ID:
        id_type, ua_type = msg[1] >> 4, msg[1] & 0x0F
        uas_id = _ascii(msg[2:22])
        if not uas_id:
            return False
        # A drone may send two BasicIDs (serial + registration). Prefer the serial.
        if id_type == 1 or not result.serial:
            result.serial = uas_id
        result.ua_type = UA_TYPES.get(ua_type, "")
        return True

    if msg_type == MSG_LOCATION:
        flags = msg[1]
        ew = (flags >> 1) & 1
        speed_mult = flags & 1
        direction = msg[2] + (180 if ew else 0)
        speed_enc = msg[3]
        lat_i, lon_i = struct.unpack_from("<ii", msg, 5)
        alt_baro, alt_geo, height = struct.unpack_from("<HHH", msg, 13)
        lat, lon = lat_i / 1e7, lon_i / 1e7
        if lat == 0 and lon == 0 or abs(lat) > 90 or abs(lon) > 180:
            return False
        result.drone_lat, result.drone_lon = lat, lon
        geo = _alt(alt_geo)
        baro = _alt(alt_baro)
        result.drone_alt = geo if geo > INVALID_ALT else (baro if baro > INVALID_ALT else 0.0)
        h = _alt(height)
        if h > INVALID_ALT:
            result.height_agl = h
        if speed_enc != SPEED_INVALID:
            result.speed = speed_enc * 0.75 + 255 * 0.25 if speed_mult else speed_enc * 0.25
        if direction < DIRECTION_INVALID:
            result.heading = float(direction % 360)
        return True

    if msg_type == MSG_SELF_ID:
        text = _ascii(msg[2:25])
        if text:
            result.description = text
            return True
        return False

    if msg_type == MSG_SYSTEM:
        op_lat_i, op_lon_i = struct.unpack_from("<ii", msg, 2)
        op_lat, op_lon = op_lat_i / 1e7, op_lon_i / 1e7
        if (op_lat or op_lon) and abs(op_lat) <= 90 and abs(op_lon) <= 180:
            result.operator_lat, result.operator_lon = op_lat, op_lon
            return True
        return False

    if msg_type == MSG_OPERATOR_ID:
        op_id = _ascii(msg[2:22])
        if op_id:
            result.operator_id = op_id
            return True
        return False

    return False  # MSG_AUTH and unknown types carry nothing we display


def parse_messages(data: bytes, source: str = "") -> DroneIDData | None:
    """Decode a message pack or a single message. Returns None if nothing useful was found."""
    if not data:
        return None
    result = DroneIDData(source=source)
    found = False
    if data[0] >> 4 == MSG_PACK:
        if len(data) < 3:
            return None
        size, count = data[1], data[2]
        if size != MSG_SIZE or count > 9:
            return None
        for i in range(count):
            off = 3 + i * MSG_SIZE
            if off + MSG_SIZE > len(data):
                break
            found |= parse_message(data[off: off + MSG_SIZE], result)
    else:
        found = parse_message(data[:MSG_SIZE], result)
    return result if found else None


# ── transports ────────────────────────────────────────────────────


def parse_vendor_ie(oui: bytes, payload: bytes) -> DroneIDData | None:
    """Wi-Fi Beacon transport. ``payload`` = IE bytes after the OUI (type, counter, pack)."""
    if oui != OPENDRONEID_OUI or len(payload) < 2 + MSG_SIZE:
        return None
    if payload[0] != OPENDRONEID_OUI_TYPE:
        return None
    try:
        return parse_messages(payload[2:], source="remoteid-wifi-beacon")
    except struct.error:
        logger.debug("Bad OpenDroneID beacon IE (%d bytes)", len(payload))
        return None


def parse_nan_action(body: bytes) -> DroneIDData | None:
    """Wi-Fi NAN transport. ``body`` = action frame body after the 24-byte management header."""
    # category, action, OUI(3), OUI type
    if len(body) < 6 or body[0] != 0x04 or body[1] != 0x09 or body[2:5] != WFA_OUI or body[5] != NAN_OUI_TYPE:
        return None
    off = 6
    while off + 3 <= len(body):
        attr_id = body[off]
        attr_len = struct.unpack_from("<H", body, off + 1)[0]
        attr = body[off + 3: off + 3 + attr_len]
        if attr_id == 0x03 and len(attr) >= 10 and attr[:6] == NAN_SERVICE_ID:
            # service_id(6) instance(1) requestor(1) control(1) info_len(1) info...
            info_len = attr[9]
            info = attr[10: 10 + info_len]
            if len(info) < 1 + MSG_SIZE:
                return None
            try:
                return parse_messages(info[1:], source="remoteid-wifi-nan")
            except struct.error:
                return None
        off += 3 + attr_len
    return None


def parse_ble_service_data(data: bytes) -> DroneIDData | None:
    """Bluetooth transport. ``data`` = service data for UUID 0xFFFA (app code, counter, message/pack)."""
    if len(data) < 2 + MSG_SIZE or data[0] != BLE_APP_CODE:
        return None
    try:
        return parse_messages(data[2:], source="remoteid-ble")
    except struct.error:
        return None
