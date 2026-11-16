"""Rehearsal fixes: DD-1 CoT rate, DD-2 SIMULATED labels, DD-3 mesh alerts (contract C4.2-C4.4)."""

import time
import xml.etree.ElementTree as ET

from detector.cot_emitter import MIN_UPDATE_S, CotEmitter
from detector.engine import DetectionEngine
from detector.mesh_alerts import MAX_TEXT, MeshAlerter, build_text
from detector.models import Classification
from detector.scanner import packet_to_frame
from detector.simulator import DemoScenario
from tests import frames


def feed(engine, pkt, n=1):
    for _ in range(n):
        engine.process_frame(packet_to_frame(pkt))


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


# ── DD-1: at most one CoT update per 5 s per track ───────────────

def test_cot_rate_limited_per_track():
    e = DetectionEngine()
    feed(e, frames.odid_beacon())
    dev = e.devices["AC:12:34:56:78:9A"]
    em = CotEmitter()
    em._running = True  # don't start the sender thread
    t0 = 1000.0
    for i in range(50):  # 10 Hz for 5 s
        em.emit(dev, now=t0 + i * 0.1)
    assert em.sent == 1 and em.throttled == 49
    em.emit(dev, now=t0 + MIN_UPDATE_S)
    assert em.sent == 2


def test_classification_change_is_sent_immediately():
    e = DetectionEngine()
    feed(e, frames.beacon("90:3A:E6:11:22:33", ssid="Anafi-123456", rssi=-60))
    dev = e.devices["90:3A:E6:11:22:33"]
    em = CotEmitter(sensor_lat=51.2, sensor_lon=-1.8)
    em._running = True
    em.emit(dev, now=1000.0)
    dev.classification = Classification.DRONE_CONFIRMED
    em.emit(dev, now=1001.0)
    assert em.sent == 2


def test_simulated_scenario_cot_rate_is_bounded():
    """Five minutes of the demo scenario: no track updates more often than every 5 s."""
    em = CotEmitter(sensor_lat=51.15, sensor_lon=-1.75, simulated=True)
    em._running = True
    clock = Clock(0.0)
    sent_at: dict[str, list[float]] = {}
    orig_build = em.build

    def build(device, now=None):
        out = orig_build(device, now)
        if out:
            sent_at.setdefault(device.mac, []).append(now)
        return out

    em.build = build
    e = DetectionEngine(on_alert=lambda d: em.emit(d, now=clock()))
    sim = DemoScenario(e, None)
    for i in range(0, 300 * 5):
        clock.t = i * 0.2
        sim.tick(clock.t)
    total = sum(len(v) for v in sent_at.values())
    assert len(sent_at) >= 3 and total > 0
    assert total < 300 * 3 / 5 + 20  # 3 alerting tracks, <=1 per 5 s each
    for times in sent_at.values():
        gaps = [b - a for a, b in zip(times, times[1:])]
        assert all(g >= MIN_UPDATE_S - 1e-6 for g in gaps)


# ── DD-2: SIMULATED: in remarks ──────────────────────────────────

def test_simulated_remarks_prefix_on_uav_and_operator():
    e = DetectionEngine()
    feed(e, frames.odid_beacon())
    dev = e.devices["AC:12:34:56:78:9A"]
    for xml in CotEmitter(simulated=True).build(dev):
        assert ET.fromstring(xml).find("detail/remarks").text.startswith("SIMULATED: ")
    for xml in CotEmitter().build(dev):
        assert not ET.fromstring(xml).find("detail/remarks").text.startswith("SIMULATED")


def test_no_position_no_cot_never_zero_zero():
    e = DetectionEngine()
    feed(e, frames.beacon("90:3A:E6:11:22:33", ssid="Anafi-123456", rssi=-60))
    assert CotEmitter(simulated=True).build(e.devices["90:3A:E6:11:22:33"]) == []


# ── DD-3: mesh alerts for new detections, <=1 per 30 s ───────────

def test_mesh_alert_new_detection_then_aggregated_and_rate_limited():
    sent, clock = [], Clock()
    m = MeshAlerter(sent.append, simulated=True, clock=clock)
    e = DetectionEngine(on_alert=m.note)
    feed(e, frames.odid_beacon())
    assert m.flush() is None  # settling: serial/position may follow in the next frames
    clock.t += 5
    assert m.flush().startswith("[SIM] DRONE: UAV ")
    assert "51.47780" in sent[0]
    # a burst of new detections within 30 s: nothing until the interval passes, then one text
    feed(e, frames.dji_beacon(frames.dji_flight_reg_v1()))
    feed(e, frames.beacon("90:3A:E6:11:22:33", ssid="Anafi-123456", rssi=-60))
    clock.t += 10
    assert m.flush() is None
    clock.t += 21
    text = m.flush()
    assert text.startswith("[SIM] DRONE: 2 new: UAV ") and "probable UAV" in text
    assert len(sent) == 2


def test_mesh_alert_not_repeated_for_known_track_and_live_has_no_prefix():
    sent, clock = [], Clock()
    m = MeshAlerter(sent.append, simulated=False, clock=clock)
    e = DetectionEngine(on_alert=m.note)
    pkt = frames.odid_beacon()
    feed(e, pkt)
    clock.t += 5
    m.flush()
    for _ in range(5):
        clock.t += 31
        e._last_alert.clear()  # force CoT-style refresh alerts
        feed(e, pkt)
        assert m.flush() is None
    assert len(sent) == 1 and sent[0].startswith("DRONE: ")


def test_mesh_text_fits_200_chars():
    e = DetectionEngine()
    for i in range(30):
        feed(e, frames.beacon(f"90:3A:E6:11:22:{i:02X}", ssid=f"Anafi-1234{i:02d}", rssi=-60))
    devs = [d for d in e.devices.values() if d.classification == Classification.DRONE_PROBABLE]
    text = build_text(devs, simulated=True)
    assert len(text) <= MAX_TEXT and "more" in text


# ── contract v5: registered sender against a fake OTS ───────────

def test_drone_cot_reaches_atak_through_fake_ots(monkeypatch):
    """With the FK-drop, bind-delay and unique-callsign rules enforced, a listener gets our UAV."""
    from detector.tak_client import TakClient
    from tests.fake_ots import FakeOTS

    monkeypatch.setenv("TAK_REGISTER_DELAY_S", "0.5")
    ots = FakeOTS(bind_delay=0.3)
    got = []
    atak = TakClient("atak", "ATAK", port=ots.port, station="phone", on_event=got.append).start()
    try:
        assert atak.wait_registered(3)
        em = CotEmitter(tak_port=ots.port, sensor_lat=51.15, sensor_lon=-1.75, simulated=True)
        e = DetectionEngine(on_alert=em.emit)
        feed(e, frames.odid_beacon())
        end = time.time() + 5
        while time.time() < end and not any("DRONE-SIM-AC123456789A" in x for x in got):
            time.sleep(0.05)
        assert any("DRONE-SIM-AC123456789A" in x for x in got)
        assert not ots.dropped and em.connected
        assert "DRONE-SIM-AC123456789A" in ots.markers and not ots.marker_errors  # on OTS's web map
        em.stop()
    finally:
        atak.stop()
        ots.close()
