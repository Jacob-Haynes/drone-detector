"""Course, speed, closing/opening, CPA, smoothing and RSSI trend (detector.motion)."""

import math
import random

import pytest

from detector import motion as mo
from detector.models import Classification, DetectedDevice, DroneIDData
from detector.simulator import offset

OP = (51.15, -1.75)


def at(east, north):
    return offset(*OP, east, north)


def test_radial_speed_sign():
    assert mo.radial_speed(1000, 0, -10, 0) == pytest.approx(-10)  # east of the OP flying west: closing
    assert mo.radial_speed(1000, 0, 10, 0) == pytest.approx(10)  # flying east: opening
    assert mo.radial_speed(1000, 0, 0, 10) == pytest.approx(0)  # flying north: crossing
    assert mo.radial_speed(0, 0, 5, 5) == 0.0


def test_cpa_maths():
    # 1 km east, 100 m north, flying due west at 10 m/s: CPA 100 m after 100 s
    t, d = mo.cpa(1000, 100, -10, 0)
    assert t == pytest.approx(100) and d == pytest.approx(100)
    # head-on: CPA at the OP
    t, d = mo.cpa(0, -500, 0, 5)
    assert t == pytest.approx(100) and d == pytest.approx(0, abs=1e-9)
    assert mo.cpa(1000, 0, 10, 0) is None  # moving away
    assert mo.cpa(1000, 0, 0, 0) is None  # not moving
    assert mo.fmt_mmss(100) == "01:40" and mo.fmt_mmss(5) == "00:05"


def test_bearing_wrap_at_360():
    assert mo.fmt_course(359.6) == "000°" and mo.fmt_course(0.4) == "000°"
    assert mo.fmt_course(45) == "045°" and mo.fmt_course(225.2) == "225°" and mo.fmt_course(-10) == "350°"
    assert mo.course_of(-1e-9, 1) == pytest.approx(360, abs=1e-6) or mo.course_of(-1e-9, 1) < 1e-6
    assert mo.course_of(0, 1) == 0 and mo.course_of(1, 0) == 90 and mo.course_of(-1, 0) == 270
    assert mo.compass(359) == "N" and mo.compass(44) == "NE" and mo.compass(226) == "SW"
    ve, vn = mo.velocity(350, 10)
    assert mo.course_of(ve, vn) == pytest.approx(350)


def track_of(points, broadcast=None):
    """points: [(t, east, north)] around the OP."""
    tr = mo.MotionTrack()
    for t, e, n in points:
        c, s = broadcast if broadcast else (None, None)
        tr.add_fix(t, *at(e, n), course=c, speed=s)
    return tr


def test_inbound_from_fixes_is_closing_with_cpa():
    # from 1 km east, 100 m north, due west at 10 m/s, a fix every 2 s
    tr = track_of([(t, 1000 - 10 * t, 100) for t in range(0, 21, 2)])
    m = tr.summary(OP)
    assert m.source == "fixes" and m.course == pytest.approx(270, abs=1) and m.speed == pytest.approx(10, rel=.02)
    assert m.relation == "closing" and m.radial < -1
    assert m.cpa_m == pytest.approx(100, abs=5) and m.cpa_s == pytest.approx(80, abs=3)
    assert m.label.startswith("Closing · CPA 10") and "in 01:2" in m.label
    assert m.mesh_text() == "hdg 270 closing"


def test_outbound_crossing_hovering():
    assert track_of([(t, 200 + 10 * t, 0) for t in range(0, 21, 2)]).summary(OP).relation == "opening"
    # orbiting at 600 m: radial speed ~0
    orbit = [(t, 600 * math.cos(t * 0.02), 600 * math.sin(t * 0.02)) for t in range(0, 21, 2)]
    m = track_of(orbit).summary(OP)
    assert m.relation == "crossing" and m.cpa_s is None and m.label == "Crossing"
    hover = track_of([(t, 500 + random.uniform(-0.3, 0.3), 500) for t in range(0, 21, 2)]).summary(OP)
    assert hover.relation == "hovering" and hover.mesh_text() == "hovering"


def test_fixes_closer_than_5s_and_10m_give_no_course():
    tr = track_of([(0, 500, 0), (1.5, 499, 0), (3, 498, 0)])  # 3 s, 2 m apart
    assert tr.summary(OP).course is None and tr.summary(OP).label == "–"
    tr.add_fix(4.5, *at(480, 0))  # 20 m from the first fix: now derivable
    assert tr.summary(OP).course == pytest.approx(270, abs=1)


def test_broadcast_track_wins_and_label_is_smoothed():
    # broadcast says course 225 at 8 m/s while fixes jitter: course/speed are the broadcast's
    random.seed(2)
    pts = [(t, 1400 - 5.66 * t + random.uniform(-15, 15), 1400 - 5.66 * t + random.uniform(-15, 15))
           for t in range(0, 30, 2)]
    m = track_of(pts, broadcast=(225.0, 8.0)).summary(OP)
    assert m.source == "broadcast" and m.course == pytest.approx(225) and m.speed == pytest.approx(8)
    assert m.relation == "closing"
    # one wild sample among five doesn't flip the label: median of the window
    tr = mo.MotionTrack()
    for t, c in [(0, 270), (2, 270), (4, 90), (6, 270), (8, 270)]:  # 90 = opening for a drone to the east
        tr.add_fix(t, *at(1000 - 10 * t, 0), course=c, speed=10)
    assert tr.summary(OP).relation == "closing"


def test_rssi_trend_closing_opening_and_flat():
    random.seed(3)
    rising = [(t, int(-90 + 0.8 * t + random.gauss(0, 2))) for t in range(0, 21)]
    falling = [(t, int(-60 - 0.8 * t + random.gauss(0, 2))) for t in range(0, 21)]
    flat = [(t, int(-75 + random.gauss(0, 2.5))) for t in range(0, 21)]
    assert mo.rssi_trend(rising, now=20) == "closing"
    assert mo.rssi_trend(falling, now=20) == "opening"
    assert mo.rssi_trend(flat, now=20) == ""
    assert mo.rssi_trend(rising[:4], now=3) == ""  # too few samples / too short
    m = mo.rssi_motion(rising, now=20)
    assert m.label == "Closing~" and m.course is None and m.estimate and m.mesh_text() == "closing~"
    assert mo.rssi_motion(flat, now=20).label == "–"


def test_device_motion_uses_position_else_rssi():
    dev = DetectedDevice(mac="x", classification=Classification.DRONE_PROBABLE)
    dev.rssi_history = [(1000 + t, -90 + t) for t in range(20)]
    m = dev.motion(OP, now=1019)
    assert m.relation == "closing" and m.estimate and m.course is None
    dev.drone_id = DroneIDData(drone_lat=at(1000, 0)[0], drone_lon=at(1000, 0)[1])
    for t in range(0, 12, 2):
        dev.track.add_fix(t, *at(1000 + 10 * t, 0))
    m = dev.motion(OP)
    assert m.relation == "opening" and not m.estimate and m.course == pytest.approx(90, abs=1)
    assert dev.to_dict(OP)["motion"]["label"] == "Opening"


def test_cot_track_and_mesh_text_carry_motion():
    from detector.cot_emitter import CotEmitter
    from detector.mesh_alerts import build_text
    import xml.etree.ElementTree as ET

    dev = DetectedDevice(mac="AA:BB:CC:DD:EE:FF", classification=Classification.DRONE_CONFIRMED)
    lat, lon = at(1000, 100)
    dev.drone_id = DroneIDData(serial="1581F5BKD23C0042", drone_lat=lat, drone_lon=lon, drone_alt=80,
                               speed=10, course=270, velocity_valid=True, heading=10)  # yaw 010, track 270
    dev.track.add_fix(0, lat, lon, course=270, speed=10)
    xml = CotEmitter(sensor_lat=OP[0], sensor_lon=OP[1], simulated=True).build(dev)[0]
    track = ET.fromstring(xml).find("detail/track")
    assert float(track.get("course")) == pytest.approx(270) and float(track.get("speed")) == pytest.approx(10)
    assert "Closing - CPA 100 m" in ET.fromstring(xml).find("detail/remarks").text
    text = build_text([dev], simulated=True, op=OP)
    assert text.startswith("[SIM] DRONE: UAV ") and text.endswith("hdg 270 closing") and len(text) <= 200


def test_merge_keeps_zero_course_and_speed():
    a = DroneIDData(speed=8, course=90, velocity_valid=True)
    a.merge(DroneIDData(serial="X"))  # BasicID message: no velocity, must not wipe it
    assert (a.speed, a.course) == (8, 90)
    a.merge(DroneIDData(speed=0.0, course=0.0, velocity_valid=True))  # hovering, nose north: real values
    assert (a.speed, a.course) == (0.0, 0.0)
