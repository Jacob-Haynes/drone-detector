"""Scripted demo scenario for ``--simulate``.

A 5-minute loop centred on the sensor (``SENSOR_LAT/LON``, else Salisbury Plain):

* t=20-280 s  **Remote ID quad** on Bluetooth LE: circles 250 m around a point ~600 m NE,
  80 m up, with its operator standing still. Shows serial, UA type, operator position and ID.
* t=60-300 s  **DJI Wi-Fi drone** (DJI DroneID beacons): crosses W→E ~300 m north at 12 m/s,
  with home and pilot position. A DJI controller probes nearby while it flies.
* t=120-240 s **Parrot access point** with no Remote ID: "probable" only, placed at the sensor.
* always      a DJI Osmo camera and a DJI controller ("drone-related", never alerted) plus
  background phones and routers.

Every payload is built in the real wire format, so the real parsers, classifier and CoT
code run exactly as they would on live traffic. RSSI follows distance. Simulated CoT
callsigns carry a ``SIM-`` prefix.
"""

from __future__ import annotations

import logging
import math
import random
import struct
import time

from detector.engine import DetectionEngine
from detector.parsers import remote_id

log = logging.getLogger("detector.sim")

PERIOD_S = 300
TICK_S = 0.2
DEFAULT_CENTRE = (51.15, -1.75)


def offset(lat: float, lon: float, east_m: float, north_m: float) -> tuple[float, float]:
    return (lat + north_m / 111_320.0,
            lon + east_m / (111_320.0 * math.cos(math.radians(lat))))


def rssi_at(distance_m: float) -> int:
    """Inverse of DetectedDevice.distance_estimate_m (20 dBm EIRP, 40 dB at 1 m, n=2.5), plus noise."""
    return int(round(20 - 40 - 25 * math.log10(max(distance_m, 1.0)) + random.gauss(0, 2.5)))


# ── real-format payload builders ──────────────────────────────────

def dji_flight_reg_v2(serial: bytes, lat, lon, alt, speed_n, speed_e, yaw_deg, app, home) -> bytes:
    raw = lambda deg: int(deg * 174533.0)  # noqa: E731  (radians * 1e7)
    rec = struct.pack("<BHH16sii", 2, random.randint(0, 65535), 0x0FFF, serial.ljust(16, b"\0"),
                      raw(lon), raw(lat))
    rec += struct.pack("<hhhhhhQiiii", int(alt * 10), int(alt), int(speed_n * 100), int(speed_e * 100), 0,
                       int(((yaw_deg + 180) % 360 - 180) * 100), int(time.time() * 1000),
                       raw(app[0]), raw(app[1]), raw(home[1]), raw(home[0]))
    rec += bytes([60, 0])
    return bytes([0x58, 0x62, 0x13, 0x10]) + rec


def rid_location(lat, lon, alt, heading, speed) -> bytes:
    ew = heading >= 180
    direction = int(heading - 180 if ew else heading)
    speed_enc = min(int(speed / 0.25), 254)
    enc = lambda a: int((a + 1000) / 0.5)  # noqa: E731
    msg = bytes([0x12, 0x20 | (0x02 if ew else 0), direction, speed_enc, 0])
    msg += struct.pack("<iiHHH", int(lat * 1e7), int(lon * 1e7), enc(alt - 5), enc(alt), enc(alt - 10))
    return msg + bytes(6)


def rid_basic(serial: bytes) -> bytes:
    return bytes([0x02, 0x12]) + serial.ljust(20, b"\0") + bytes(3)


def rid_system(op_lat, op_lon) -> bytes:
    return bytes([0x42, 0x01]) + struct.pack("<ii", int(op_lat * 1e7), int(op_lon * 1e7)) + bytes(15)


def rid_operator(op_id: bytes) -> bytes:
    return bytes([0x52, 0x00]) + op_id.ljust(20, b"\0") + bytes(3)


# ── scenario ──────────────────────────────────────────────────────

BACKGROUND = [
    ("E4:5F:01:3C:11:02", "BTHome-7Q2X", "beacon", 120),
    ("F8:1A:67:22:90:1D", "EE-Hub-4F2A", "beacon", 220),
    ("3A:91:C2:55:10:EE", "", "probe_req", 40),
    ("76:22:0B:9E:31:4A", "", "probe_req", 25),
    ("C8:3A:35:10:22:7B", "Tenda_22C7", "beacon", 300),
    ("5E:10:AB:03:FF:12", "", "probe_req", 60),
]


class DemoScenario:
    def __init__(self, engine: DetectionEngine, centre: tuple[float, float] | None = None):
        self.engine = engine
        self.lat0, self.lon0 = centre or DEFAULT_CENTRE
        self.t0 = time.time()

    def _emit(self, frame: dict, probability: float = 1.0):
        if random.random() < probability:
            self.engine.process_frame(frame)

    def tick(self, now: float | None = None):
        t = ((now or time.time()) - self.t0) % PERIOD_S

        # Remote ID quad over BLE: circle around a point ~600 m NE
        if 20 <= t < 280:
            cx, cy, r = 420.0, 420.0, 250.0
            ang = (t - 20) * (8.0 / r)  # 8 m/s along the circle
            east, north = cx + r * math.cos(ang), cy + r * math.sin(ang)
            heading = (math.degrees(math.atan2(-math.sin(ang), math.cos(ang))) + 360) % 360  # tangent, CCW
            lat, lon = offset(self.lat0, self.lon0, east, north)
            op_lat, op_lon = offset(self.lat0, self.lon0, 700, 650)
            msg = random.choice([rid_basic(b"1581F5BKD23C0042"), rid_location(lat, lon, 80, heading, 8.0),
                                 rid_location(lat, lon, 80, heading, 8.0), rid_system(op_lat, op_lon),
                                 rid_operator(b"GBR-OP-SIMDEMO01")])
            self._emit({"mac": "BLE:E1:7A:3C:90:22:41", "rssi": rssi_at(math.hypot(east, north)) - 10,
                        "ssid": "", "frame_type": "ble", "vendor_ies": [], "transport": "ble",
                        "drone_data": remote_id.parse_ble_service_data(bytes([0x0D, int(t) % 256]) + msg)},
                       probability=0.5)

        # DJI Wi-Fi drone crossing W -> E, 300 m north, 12 m/s
        if 60 <= t < 300:
            east, north = -1440.0 + (t - 60) * 12.0, 300.0
            lat, lon = offset(self.lat0, self.lon0, east, north)
            home = offset(self.lat0, self.lon0, -1500, 280)
            app = offset(self.lat0, self.lon0, -1490, 270)
            ie = dji_flight_reg_v2(b"08QDE5K0010042", lat, lon, 60, 0.0, 12.0, 90.0, app, home)
            self._emit({"mac": "60:60:1F:5A:21:C4", "rssi": rssi_at(math.hypot(east, north)),
                        "ssid": "DJI-MAVICAIR-5A21C4", "frame_type": "beacon",
                        "vendor_ies": [(b"\x26\x37\x12", ie)], "transport": "wifi"}, probability=0.6)
            # its controller, probing from the pilot's position
            self._emit({"mac": "60:60:1F:5A:21:C5", "rssi": rssi_at(math.hypot(-1490, 270)),
                        "ssid": "", "frame_type": "probe_req", "vendor_ies": [], "transport": "wifi"},
                       probability=0.1)

        # Parrot AP, no Remote ID: probable only
        if 120 <= t < 240:
            d = 150 + abs(t - 180) * 6
            self._emit({"mac": "90:3A:E6:41:0C:7E", "rssi": rssi_at(d), "ssid": "Anafi-410C7E",
                        "frame_type": "beacon", "vendor_ies": [], "transport": "wifi"}, probability=0.3)

        # Drone-related devices that must NOT raise an alarm
        self._emit({"mac": "60:60:1F:44:55:66", "rssi": rssi_at(35), "ssid": "OsmoAction4-5566",
                    "frame_type": "beacon", "vendor_ies": [], "transport": "wifi"}, probability=0.15)

        for mac, ssid, ftype, dist in BACKGROUND:
            self._emit({"mac": mac, "rssi": rssi_at(dist), "ssid": ssid, "frame_type": ftype,
                        "vendor_ies": [], "transport": "wifi"}, probability=0.2)

    def run(self):
        log.info("Demo scenario running around %.5f, %.5f (5-minute loop)", self.lat0, self.lon0)
        while True:
            try:
                self.tick()
            except Exception:
                log.exception("Simulator tick failed")
            time.sleep(TICK_S)
