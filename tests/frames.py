"""Real-format sample frames for tests.

Remote ID messages follow ASTM F3411 / opendroneid-core-c byte layouts; DJI DroneID
follows Kismet's dot11_ie_221_dji_droneid layout. Frames are full 802.11 frames
(RadioTap + management header) serialised to bytes, as a monitor interface delivers them.
"""

import struct

from scapy.all import Dot11, Dot11Beacon, Dot11Elt, Dot11ProbeReq, RadioTap, Raw

# Hand-checked Location message: lat 51.4778, lon -0.0015, dir 90, 10 m/s, +2 m/s up,
# baro 100 m, geo 120 m, height 50 m, airborne.
LOCATION_HEX = "12205a280490e3ae1e68c5ffff9808c0083408000070170000"
# BasicID: ID type 1 (serial), UA type 2 (multirotor), serial "1596F35D7A12345"
BASIC_ID_HEX = "02123135393646333544374131323334350000000000000000"
LOCATION = bytes.fromhex(LOCATION_HEX)
BASIC_ID = bytes.fromhex(BASIC_ID_HEX)


def system_msg(op_lat, op_lon):
    return bytes([0x42, 0x01]) + struct.pack("<ii", round(op_lat * 1e7), round(op_lon * 1e7)) + bytes(15)


def operator_id_msg(op_id: bytes):
    return bytes([0x52, 0x00]) + op_id.ljust(20, b"\0") + bytes(3)


def message_pack(*msgs):
    return bytes([0xF2, 25, len(msgs)]) + b"".join(msgs)


OPERATOR_LAT, OPERATOR_LON = 51.4790, -0.0030
PACK = message_pack(BASIC_ID, LOCATION, system_msg(OPERATOR_LAT, OPERATOR_LON), operator_id_msg(b"GBR-OP-12345678"))


def _radiotap(rssi):
    return RadioTap(present="dBm_AntSignal", dBm_AntSignal=rssi)


def beacon(mac, ssid="", ies=(), rssi=-60):
    pkt = (_radiotap(rssi) / Dot11(type=0, subtype=8, addr1="ff:ff:ff:ff:ff:ff", addr2=mac, addr3=mac)
           / Dot11Beacon(cap="ESS") / Dot11Elt(ID=0, info=ssid.encode()))
    for oui, payload in ies:
        pkt = pkt / Dot11Elt(ID=221, info=oui + payload)
    return RadioTap(bytes(pkt))


def probe_req(mac, ssid="", rssi=-70):
    pkt = (_radiotap(rssi) / Dot11(type=0, subtype=4, addr1="ff:ff:ff:ff:ff:ff", addr2=mac, addr3="ff:ff:ff:ff:ff:ff")
           / Dot11ProbeReq() / Dot11Elt(ID=0, info=ssid.encode()))
    return RadioTap(bytes(pkt))


def odid_beacon(mac="AC:12:34:56:78:9A", counter=7):
    """Wi-Fi Beacon Remote ID: vendor IE FA:0B:BC, type 0x0D, counter, message pack."""
    return beacon(mac, ssid="", ies=[(b"\xFA\x0B\xBC", bytes([0x0D, counter]) + PACK)])


def nan_action_body(pack=PACK, counter=3):
    """NAN service discovery body as built by opendroneid-core-c odid_wifi_build_message_pack_nan_action_frame."""
    service_info = bytes([counter]) + pack
    sda_body = bytes([0x88, 0x69, 0x19, 0x9D, 0x92, 0x09, 0x01, 0x00, 0x10, len(service_info)]) + service_info
    sda = bytes([0x03]) + struct.pack("<H", len(sda_body)) + sda_body
    sdea = bytes([0x0E]) + struct.pack("<H", 4) + bytes([0x01]) + struct.pack("<H", 0x0200) + bytes([counter])
    return bytes([0x04, 0x09, 0x50, 0x6F, 0x9A, 0x13]) + sda + sdea


def nan_frame(mac="AC:12:34:56:78:9B", rssi=-58):
    pkt = (_radiotap(rssi) / Dot11(type=0, subtype=13, addr1="51:6f:9a:01:00:00", addr2=mac,
                                   addr3="50:6f:9a:01:00:ff") / Raw(nan_action_body()))
    return RadioTap(bytes(pkt))


RAW_PER_DEG = 174533.0


def dji_flight_reg_v1(serial=b"0JXDE4A0010123", lat=51.15, lon=-1.75, alt=85, height=80,
                      vn=300, ve=400, yaw=4500, home=(51.1490, -1.7510)):
    rec = struct.pack("<BHH16sii", 1, 1234, 0x0FFF, serial.ljust(16, b"\0"),
                      int(lon * RAW_PER_DEG), int(lat * RAW_PER_DEG))
    rec += struct.pack("<hhhhhhhhii", alt, height, vn, ve, 0, 0, 0, yaw,
                       int(home[1] * RAW_PER_DEG), int(home[0] * RAW_PER_DEG))
    rec += bytes([16, 0])
    return bytes([0x58, 0x62, 0x13, 0x10]) + rec


def dji_flight_reg_v2(serial=b"1581F5FHC245ABCD", lat=51.16, lon=-1.76, height_dm=600, alt=70,
                      app=(51.1585, -1.7620), home=(51.1586, -1.7621)):
    rec = struct.pack("<BHH16sii", 2, 99, 0x0FFF, serial.ljust(16, b"\0"),
                      int(lon * RAW_PER_DEG), int(lat * RAW_PER_DEG))
    rec += struct.pack("<hhhhhhQiiii", height_dm, alt, -500, 0, 0, -9000, 1731760000000,
                       int(app[0] * RAW_PER_DEG), int(app[1] * RAW_PER_DEG),
                       int(home[1] * RAW_PER_DEG), int(home[0] * RAW_PER_DEG))
    rec += bytes([60, 0])
    return bytes([0x58, 0x62, 0x13, 0x10]) + rec


def dji_beacon(payload, mac="60:60:1F:AA:BB:CC", ssid="DJI-MAVICAIR-AABBCC"):
    return beacon(mac, ssid=ssid, ies=[(b"\x26\x37\x12", payload)])
