"""Scripted demo scenario for ``--simulate``.

A 5-minute loop centred on the sensor (``SENSOR_LAT/LON``, else Salisbury Plain):

* t=20-280 s  **Remote ID quad, inbound** on Bluetooth LE: flies from ~2 km NE towards the OP
  at 8 m/s on course 225°, passing 120 m from it (closing, with a CPA), 80 m up, its operator
  standing still. Shows serial, UA type, operator position and ID.
* t=60-300 s  **DJI Wi-Fi drone, crossing** (DJI DroneID beacons): orbits the OP at 600 m,
  12 m/s anticlockwise, with home and pilot position. A DJI controller probes nearby.
* t=30-290 s  **Remote ID quad, outbound** on Bluetooth LE: leaves from 200 m S of the OP on
  course 200° at 10 m/s (opening).
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
from dataclasses import dataclass
from typing import Callable

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


def rid_location(lat, lon, alt, heading, speed, airborne: bool = True) -> bytes:
    ew = heading >= 180
    direction = int(heading - 180 if ew else heading)
    speed_enc = min(int(speed / 0.25), 254)
    enc = lambda a: int((a + 1000) / 0.5)  # noqa: E731
    msg = bytes([0x12, (0x20 if airborne else 0x10) | (0x02 if ew else 0), direction, speed_enc, 0])
    msg += struct.pack("<iiHHH", int(lat * 1e7), int(lon * 1e7), enc(alt - 5), enc(alt), enc(alt - 10))
    return msg + bytes(6)


def rid_basic(serial: bytes) -> bytes:
    return bytes([0x02, 0x12]) + serial.ljust(20, b"\0") + bytes(3)


def rid_system(op_lat, op_lon) -> bytes:
    return bytes([0x42, 0x01]) + struct.pack("<ii", int(op_lat * 1e7), int(op_lon * 1e7)) + bytes(15)


def rid_operator(op_id: bytes) -> bytes:
    return bytes([0x52, 0x00]) + op_id.ljust(20, b"\0") + bytes(3)


# ── flight models (shared by the continuous loop and the on-demand demo, detector.demo) ──

Emit = Callable[[dict, float], None]  # (frame, probability)


@dataclass
class RemoteIdFlight:
    """A Remote ID quad over Bluetooth LE on a straight line from ``start`` (east, north metres
    from the OP) at ``heading``/``speed``. With ``stop_after_s`` it stops there, comes down at
    ``descent_mps`` and keeps broadcasting from the ground (status "on ground")."""

    mac: str
    serial: bytes
    start: tuple[float, float]
    heading: float
    speed: float
    alt: float
    operator: tuple[float, float] | None = None  # east, north of the pilot (Remote ID System message)
    operator_id: bytes | None = None
    stop_after_s: float | None = None
    descent_mps: float = 2.5
    probability: float = 0.5

    def state(self, t: float) -> tuple[float, float, float, float, bool]:
        """(east, north, alt, speed, airborne) ``t`` seconds into the flight."""
        flying = t if self.stop_after_s is None else min(t, self.stop_after_s)
        rad = math.radians(self.heading)
        east = self.start[0] + self.speed * math.sin(rad) * flying
        north = self.start[1] + self.speed * math.cos(rad) * flying
        if self.stop_after_s is None or t <= self.stop_after_s:
            return east, north, self.alt, self.speed, True
        alt = max(0.0, self.alt - self.descent_mps * (t - self.stop_after_s))
        return east, north, alt, 0.0, alt > 0

    def tick(self, t: float, centre: tuple[float, float], emit: Emit, seq: int):
        east, north, alt, speed, airborne = self.state(t)
        lat, lon = offset(centre[0], centre[1], east, north)
        msgs = [rid_basic(self.serial), rid_location(lat, lon, alt, self.heading, speed, airborne),
                rid_location(lat, lon, alt, self.heading, speed, airborne)]
        if self.operator is not None:
            msgs.append(rid_system(*offset(centre[0], centre[1], *self.operator)))
        if self.operator_id:
            msgs.append(rid_operator(self.operator_id))
        msg = random.choice(msgs)
        emit({"mac": self.mac, "rssi": rssi_at(math.hypot(east, north)) - 10,
              "ssid": "", "frame_type": "ble", "vendor_ies": [], "transport": "ble", "simulated": True,
              "drone_data": remote_id.parse_ble_service_data(bytes([0x0D, seq % 256]) + msg)},
             self.probability)


@dataclass
class DjiOrbit:
    """A DJI Wi-Fi drone (DroneID beacons) orbiting the OP anticlockwise, with home and pilot
    positions, and its controller probing from the pilot's position."""

    mac: str
    serial: bytes
    ssid: str
    controller_mac: str
    radius: float = 600.0
    speed: float = 12.0
    start_deg: float = 150.0
    alt: float = 60.0
    home: tuple[float, float] = (-1500.0, 280.0)
    pilot: tuple[float, float] = (-1490.0, 270.0)

    def tick(self, t: float, centre: tuple[float, float], emit: Emit):
        ang = math.radians(self.start_deg) + t * self.speed / self.radius
        east, north = self.radius * math.cos(ang), self.radius * math.sin(ang)
        ve, vn = -self.speed * math.sin(ang), self.speed * math.cos(ang)  # tangent
        lat, lon = offset(centre[0], centre[1], east, north)
        home = offset(centre[0], centre[1], *self.home)
        app = offset(centre[0], centre[1], *self.pilot)
        ie = dji_flight_reg_v2(self.serial, lat, lon, self.alt, vn, ve,
                               math.degrees(math.atan2(ve, vn)) % 360, app, home)
        emit({"mac": self.mac, "rssi": rssi_at(math.hypot(east, north)),
              "ssid": self.ssid, "frame_type": "beacon", "simulated": True,
              "vendor_ies": [(b"\x26\x37\x12", ie)], "transport": "wifi"}, 0.6)
        emit({"mac": self.controller_mac, "rssi": rssi_at(math.hypot(*self.pilot)), "simulated": True,
              "ssid": "", "frame_type": "probe_req", "vendor_ies": [], "transport": "wifi"}, 0.1)


# ── continuous scenario (--simulate, DEMO_MODE=continuous) ────────

BACKGROUND = [
    ("E4:5F:01:3C:11:02", "BTHome-7Q2X", "beacon", 120),
    ("F8:1A:67:22:90:1D", "EE-Hub-4F2A", "beacon", 220),
    ("3A:91:C2:55:10:EE", "", "probe_req", 40),
    ("76:22:0B:9E:31:4A", "", "probe_req", 25),
    ("C8:3A:35:10:22:7B", "Tenda_22C7", "beacon", 300),
    ("5E:10:AB:03:FF:12", "", "probe_req", 60),
]

# start 2 km out along 045°, shifted 120 m sideways so it misses the OP by 120 m
INBOUND = RemoteIdFlight("BLE:E1:7A:3C:90:22:41", b"1581F5BKD23C0042", (1414.0 + 85.0, 1414.0 - 85.0),
                         225.0, 8.0, 80.0, operator=(700.0, 650.0), operator_id=b"GBR-OP-SIMDEMO01")
ORBIT = DjiOrbit("60:60:1F:5A:21:C4", b"08QDE5K0010042", "DJI-MAVICAIR-5A21C4", "60:60:1F:5A:21:C5")
OUTBOUND = RemoteIdFlight("BLE:C4:2D:91:07:5E:13", b"1581F5FKD2B10099", (0.0, -200.0), 200.0, 10.0, 60.0)
PARROT_MAC = "90:3A:E6:41:0C:7E"


def continuous_tracks() -> list[tuple[str, str, bool]]:
    """(MAC, serial, has an operator marker) for every drone the continuous loop puts on ATAK.

    Fixed MACs and serials, so a restart (or a switch to DEMO_MODE=on_demand) can withdraw the
    markers a previous run left behind. The Osmo camera, controller and background devices are
    never alerted, so they have no markers."""
    return [(INBOUND.mac, INBOUND.serial.decode(), INBOUND.operator is not None),
            (ORBIT.mac, ORBIT.serial.decode(), True),  # DroneID carries the pilot position
            (OUTBOUND.mac, OUTBOUND.serial.decode(), OUTBOUND.operator is not None),
            (PARROT_MAC, "", False)]


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
        centre = (self.lat0, self.lon0)

        if 20 <= t < 280:  # Remote ID quad over BLE, inbound: from ~2 km NE towards the OP, CPA 120 m
            INBOUND.tick(t - 20, centre, self._emit, int(t))
        if 60 <= t < 300:  # DJI Wi-Fi drone, crossing: orbits the OP at 600 m, 12 m/s anticlockwise
            ORBIT.tick(t - 60, centre, self._emit)
        if 30 <= t < 290:  # Remote ID quad over BLE, outbound: from 200 m S of the OP, course 200°, 10 m/s
            OUTBOUND.tick(t - 30, centre, self._emit, int(t))

        # Parrot AP, no Remote ID: probable only (signal rises then falls: closing~ then opening~)
        if 120 <= t < 240:
            d = 150 + abs(t - 180) * 6
            self._emit({"mac": PARROT_MAC, "rssi": rssi_at(d), "ssid": "Anafi-410C7E", "simulated": True,
                        "frame_type": "beacon", "vendor_ies": [], "transport": "wifi"}, probability=0.3)

        # Drone-related devices that must NOT raise an alarm
        self._emit({"mac": "60:60:1F:44:55:66", "rssi": rssi_at(35), "ssid": "OsmoAction4-5566", "simulated": True,
                    "frame_type": "beacon", "vendor_ies": [], "transport": "wifi"}, probability=0.15)

        for mac, ssid, ftype, dist in BACKGROUND:
            self._emit({"mac": mac, "rssi": rssi_at(dist), "ssid": ssid, "frame_type": ftype, "simulated": True,
                        "vendor_ies": [], "transport": "wifi"}, probability=0.2)

    def run(self):
        log.info("Demo scenario running around %.5f, %.5f (5-minute loop)", self.lat0, self.lon0)
        while True:
            try:
                self.tick()
            except Exception:
                log.exception("Simulator tick failed")
            time.sleep(TICK_S)
