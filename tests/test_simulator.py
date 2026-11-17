import random

import pytest

from detector.cot_emitter import CotEmitter
from detector.engine import DetectionEngine
from detector.models import Classification
from detector.simulator import DemoScenario, offset, rssi_at


def test_demo_scenario_full_loop_classifies_like_live_traffic():
    random.seed(4)
    alerts = []
    engine = DetectionEngine(on_alert=alerts.append)
    sim = DemoScenario(engine, centre=(51.2, -1.8))
    for i in range(0, 300 * 5):
        sim.tick(now=sim.t0 + i * 0.2)
    devs = engine.devices
    rid = devs["BLE:E1:7A:3C:90:22:41"]
    assert rid.classification == Classification.DRONE_CONFIRMED and rid.transport == "ble"
    assert rid.drone_id.serial == "1581F5BKD23C0042" and rid.drone_id.ua_type == "multirotor"
    assert rid.drone_id.operator_id == "GBR-OP-SIMDEMO01" and rid.drone_id.has_operator
    assert abs(rid.drone_id.drone_lat - 51.2) < 0.02 and rid.drone_id.drone_alt == 80
    dji = devs["60:60:1F:5A:21:C4"]
    assert dji.classification == Classification.DRONE_CONFIRMED and dji.drone_id.serial == "08QDE5K0010042"
    assert devs["90:3A:E6:41:0C:7E"].classification == Classification.DRONE_PROBABLE
    assert devs["60:60:1F:44:55:66"].classification == Classification.DRONE_RELATED
    assert devs["60:60:1F:5A:21:C5"].classification == Classification.DRONE_RELATED
    alerted = {a.mac for a in alerts}
    assert "60:60:1F:44:55:66" not in alerted and "60:60:1F:5A:21:C5" not in alerted
    cot = CotEmitter(sensor_lat=51.2, sensor_lon=-1.8, simulated=True)
    assert all("SIM-" in x for x in cot.build(rid))


def test_rssi_model_matches_distance_estimate():
    from detector.models import DetectedDevice

    random.seed(1)
    d = DetectedDevice(mac="x")
    d.rssi = round(sum(rssi_at(400) for _ in range(200)) / 200)
    assert 250 < d.distance_estimate_m < 650
    lat, lon = offset(51.0, 0.0, 1000, 1000)
    assert abs((lat - 51.0) * 111320 - 1000) < 1


def test_simulated_drones_show_closing_crossing_and_opening(monkeypatch):
    """Inbound Remote ID quad, DJI orbit and outbound quad give all three states at t=150 s."""
    import detector.engine as engine_mod
    import detector.models as models_mod

    random.seed(7)
    clock = {"t": 0.0}
    monkeypatch.setattr(engine_mod, "time", lambda: clock["t"])
    monkeypatch.setattr(models_mod, "time", lambda: clock["t"])
    centre = (51.2, -1.8)
    engine = DetectionEngine()
    engine.op = centre
    sim = DemoScenario(engine, centre=centre)
    sim.t0 = 0.0
    for i in range(int(150 / 0.2)):
        clock["t"] = i * 0.2
        sim.tick(now=clock["t"])
    by_mac = {d["mac"]: d["motion"] for d in engine.get_active_devices()}
    inbound, orbit, outbound = (by_mac["BLE:E1:7A:3C:90:22:41"], by_mac["60:60:1F:5A:21:C4"],
                                by_mac["BLE:C4:2D:91:07:5E:13"])
    assert inbound["relation"] == "closing" and inbound["course_text"] == "225°"
    assert 80 < inbound["cpa_m"] < 160 and inbound["label"].startswith("Closing · CPA")
    assert orbit["relation"] == "crossing" and orbit["speed"] == pytest.approx(12, abs=1)
    assert outbound["relation"] == "opening" and outbound["course_text"] == "200°"
