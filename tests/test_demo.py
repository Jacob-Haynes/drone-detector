"""Step-by-step demo (DEMO_MODE=on_demand) and the demo trigger API, in both modes."""

import random
import sys
import xml.etree.ElementTree as ET

import pytest
from fastapi.testclient import TestClient

import detector.engine as engine_mod
import detector.models as models_mod
from detector.cot_emitter import CotEmitter
from detector.demo import SCENARIOS, DemoRunner, demo_mode
from detector.engine import DetectionEngine
from detector.heartbeat import build_payload
from detector.mesh_alerts import MeshAlerter
from detector.scanner import packet_to_frame
from detector.web import app as web_module
from tests import frames

CENTRE = (51.2, -1.8)


from detector.tak_client import TakClient


class FakeTak(TakClient):
    """Builds markers like the real client; records instead of connecting."""

    def __init__(self):
        super().__init__("drone-detector", "DRONE-DET", station="test")
        self.sent, self.sa_now = [], 0

    def send(self, xml):
        self.sent.append(xml)

    def send_sa_now(self):
        self.sa_now += 1

    def start(self):
        return self


@pytest.fixture
def clock(monkeypatch):
    c = {"t": 1_000_000.0}
    monkeypatch.setattr(engine_mod, "time", lambda: c["t"])
    monkeypatch.setattr(models_mod, "time", lambda: c["t"])
    return c


def make(clock, cot=None, mesh=None, mode="on_demand"):
    random.seed(3)
    engine = DetectionEngine(on_alert=cot.emit if cot else None)
    engine.op = CENTRE
    return engine, DemoRunner(engine, CENTRE, cot=cot, mesh=mesh, mode=mode, clock=lambda: clock["t"])


def run_for(runner, clock, seconds, step=0.2):
    for _ in range(int(seconds / step)):
        clock["t"] += step
        runner.tick()


def motions(engine):
    return {d["mac"]: d for d in engine.get_active_devices()}


def test_demo_mode_env(monkeypatch):
    monkeypatch.delenv("DEMO_MODE", raising=False)
    assert demo_mode() == "continuous"
    assert demo_mode("on_demand") == demo_mode("on-demand") == "on_demand"
    assert demo_mode("nonsense") == "continuous"


def test_scenarios_follow_the_contract(clock):
    _, runner = make(clock)
    s = runner.scenarios()
    assert s["mode"] == "on_demand" and s["active"] == []
    assert [x["id"] for x in s["scenarios"]] == ["inbound", "crossing", "outbound", "swarm"]
    assert all(set(x) == {"id", "label", "hint", "duration_s"} for x in s["scenarios"])
    assert all(x["label"][0].isupper() and not x["label"].isupper() for x in s["scenarios"])


def test_nothing_flies_until_triggered(clock):
    engine, runner = make(clock)
    run_for(runner, clock, 20)
    assert engine.get_active_devices() == []


def test_inbound_closes_with_cpa_then_lands(clock):
    engine, runner = make(clock)
    r = runner.trigger("inbound")
    assert r["ok"] and r["id"] == "inbound" and r["until"] == int(clock["t"] + SCENARIOS["inbound"].duration_s)
    assert runner.active() == ["inbound"]
    run_for(runner, clock, 60)
    d = motions(engine)["BLE:D0:5A:7E:10:01:01"]
    assert d["simulated"] and d["classification"] == "DRONE_CONFIRMED"
    m = d["motion"]
    assert m["relation"] == "closing" and m["course_text"] == "225°"
    assert 90 < m["cpa_m"] < 150 and m["label"].startswith("Closing · CPA")
    run_for(runner, clock, 100)  # t=160: stopped at the CPA point, on the way down or landed
    d = motions(engine)["BLE:D0:5A:7E:10:01:01"]
    assert d["motion"]["relation"] == "hovering" and d["drone_id"]["drone_alt"] < 30
    run_for(runner, clock, 20)  # past 175 s: ended on its own
    assert runner.active() == []
    frames_then = engine.devices["BLE:D0:5A:7E:10:01:01"].frame_count
    run_for(runner, clock, 5)
    assert engine.devices["BLE:D0:5A:7E:10:01:01"].frame_count == frames_then


@pytest.mark.parametrize("sid,mac,relation", [("crossing", "60:60:1F:D3:40:11", "crossing"),
                                              ("outbound", "BLE:D0:5A:7E:20:01:01", "opening")])
def test_crossing_and_outbound(clock, sid, mac, relation):
    engine, runner = make(clock)
    runner.trigger(sid)
    run_for(runner, clock, 40)
    assert motions(engine)[mac]["motion"]["relation"] == relation


def test_swarm_is_three_drones_closing(clock):
    engine, runner = make(clock)
    runner.trigger("swarm")
    run_for(runner, clock, 40)
    swarm = [d for m, d in motions(engine).items() if m.startswith("BLE:D0:5A:7E:30:")]
    assert len(swarm) == 3 and all(d["motion"]["relation"] == "closing" for d in swarm)


def test_unknown_scenario_raises(clock):
    _, runner = make(clock)
    with pytest.raises(KeyError):
        runner.trigger("nope")


def test_clear_removes_demo_cot_at_once_but_never_real_detections(clock):
    cot = CotEmitter(sensor_lat=CENTRE[0], sensor_lon=CENTRE[1], simulated=False)  # live mode emitter
    cot.client = FakeTak()
    engine, runner = make(clock, cot=cot)
    engine.process_frame(packet_to_frame(frames.odid_beacon()))  # a real Remote ID drone
    real_mac = next(iter(engine.devices))
    runner.trigger("inbound")
    runner.trigger("crossing")
    run_for(runner, clock, 30)
    demo_cot = [x for x in cot.client.sent if "DRONE-SIM-" in x]
    assert demo_cot and all("SIMULATED: " in x for x in demo_cot), "demo drones labelled in live mode"
    assert not any("SIM" in x for x in cot.client.sent if "DRONE-SIM-" not in x and "OPERATOR" not in x)
    cot.client.sent.clear()

    out = runner.clear()
    assert out == {"ok": True, "cleared": 2, "tracks": 3}  # inbound, DJI and its controller
    assert runner.active() == [] and real_mac in engine.devices
    assert not any(m.startswith(("BLE:D0:5A", "60:60:1F:D3")) for m in engine.devices)
    events = [ET.fromstring(x) for x in cot.client.sent]
    deletes = [e for e in events if e.get("type") == "t-x-d-d"]
    targets = {e.find("detail/link").get("uid") for e in deletes}
    assert targets == {"DRONE-SIM-BLED05A7E100101", "DRONE-SIM-BLED05A7E100101-OPERATOR",
                       "DRONE-SIM-60601FD34011", "DRONE-SIM-60601FD34011-OPERATOR"}
    assert all(e.find("detail/__forcedelete") is not None for e in deletes)
    assert all(e.find("point").get("lat") == "51.2000000" for e in deletes), "at the OP"
    stale_now = [e for e in events if e.get("type") != "t-x-d-d"]
    assert stale_now and all(e.get("stale") == e.get("time") for e in stale_now)
    assert cot.client.sa_now == 1, "OTS marks the sender disconnected on t-x-d-d"
    assert not any(real_mac.replace(":", "") in x for x in cot.client.sent)
    run_for(runner, clock, 5)
    assert not any(m.startswith("BLE:D0:5A") for m in engine.devices), "stopped"


def test_clear_after_a_scenario_ended_still_removes_its_markers(clock):
    cot = CotEmitter(sensor_lat=CENTRE[0], sensor_lon=CENTRE[1])
    cot.client = FakeTak()
    engine, runner = make(clock, cot=cot)
    runner.trigger("outbound")
    run_for(runner, clock, 151)
    assert runner.active() == []
    out = runner.clear()
    assert out["cleared"] == 0 and out["tracks"] == 1
    assert any('type="t-x-d-d"' in x for x in cot.client.sent)


def test_retrigger_restarts_the_scenario(clock):
    engine, runner = make(clock)
    runner.trigger("outbound")
    run_for(runner, clock, 60)
    first = runner.trigger("outbound")
    assert "BLE:D0:5A:7E:20:01:01" not in engine.devices
    assert first["until"] == int(clock["t"] + 150) and runner.active() == ["outbound"]


def test_mesh_never_mixes_real_and_simulated(clock):
    sent = []
    t = {"now": 0.0}
    mesh = MeshAlerter(sent.append, simulated=False, clock=lambda: t["now"])
    engine, runner = make(clock)
    engine._on_alert = mesh.note
    engine.process_frame(packet_to_frame(frames.odid_beacon()))
    runner.trigger("outbound")
    run_for(runner, clock, 3)
    t["now"] = 10.0
    first = mesh.flush()
    assert first.startswith("DRONE: ") and "[SIM]" not in first
    t["now"] = 45.0
    second = mesh.flush()
    assert second.startswith("[SIM] DRONE: ")


def test_heartbeat_payload_carries_demo_fields():
    p = build_payload("drone-detector", "u", "cpu", "ok", "d",
                      extra={"demo_mode": "on_demand", "demo_active": ["inbound"], "status": "bogus"})
    assert p["demo_mode"] == "on_demand" and p["demo_active"] == ["inbound"] and p["status"] == "ok"


# ── web API ───────────────────────────────────────────────────────

def test_demo_api(clock):
    engine, runner = make(clock)
    web_module.engine, web_module.demo, web_module.status = engine, runner, {"mode": "live"}
    c = TestClient(web_module.app)
    assert c.get("/api/demo/scenarios").json()["mode"] == "on_demand"
    assert c.post("/api/demo/trigger", json={"id": "nope"}).status_code == 400
    assert c.post("/api/demo/trigger", json=["x"]).status_code in (400, 422)
    r = c.post("/api/demo/trigger", json={"id": "swarm"}).json()
    assert r["ok"] and r["id"] == "swarm" and r["until"] > clock["t"]
    st = c.get("/api/status").json()
    assert st["demo_mode"] == "on_demand" and st["demo_active"] == ["swarm"]
    run_for(runner, clock, 5)
    assert all(d["simulated"] for d in c.get("/api/devices").json())
    assert c.post("/api/demo/clear").json()["ok"]
    assert c.get("/api/devices").json() == []
    web_module.demo = None
    assert c.post("/api/demo/clear").status_code == 503


def test_page_has_demo_card():
    html = (web_module.STATIC_DIR / "index.html").read_text()
    assert 'id="demo-card"' in html and "Clear demo" in html and "/api/demo/trigger" in html
    assert "d.simulated" in html  # SIM tag per demo drone in live mode


# ── modes at start-up ─────────────────────────────────────────────

def _start(monkeypatch, argv, env_mode, tak_port=None):
    """Run detector.__main__.main() far enough to see what it starts (no server, no hardware)."""
    import detector.__main__ as main_mod
    import detector.ble_scanner as ble_mod
    import detector.scanner as scanner_mod
    import detector.simulator as sim_mod

    started = {"sim": 0, "capture": 0, "ble": 0}

    class Sim:
        def __init__(self, *a, **k):
            pass

        def run(self):
            started["sim"] += 1

    class Ble:
        active = True

        def __init__(self, *a, **k):
            pass

        def start(self):
            started["ble"] += 1

    class Hopper:
        current = 6

        def __init__(self, *a, **k):
            pass

        def start(self):
            pass

    class StopServer(Exception):
        pass

    monkeypatch.setattr(sim_mod, "DemoScenario", Sim)
    monkeypatch.setattr(scanner_mod, "start_capture", lambda *a: started.__setitem__("capture", 1))
    monkeypatch.setattr(scanner_mod, "ChannelHopper", Hopper)
    monkeypatch.setattr(ble_mod, "BleRemoteIdScanner", Ble)
    monkeypatch.setattr(main_mod.asyncio, "run", lambda coro: (coro.close(), (_ for _ in ()).throw(StopServer()))[1])
    monkeypatch.setattr(main_mod.signal, "signal", lambda *a: None)
    monkeypatch.setenv("HEARTBEAT", "0")
    if env_mode:
        monkeypatch.setenv("DEMO_MODE", env_mode)
    else:
        monkeypatch.delenv("DEMO_MODE", raising=False)
    cot = ["--no-cot"] if tak_port is None else ["--tak-host", "127.0.0.1", "--tak-port", str(tak_port)]
    monkeypatch.setattr(sys, "argv", ["detector", *cot, "--no-mesh", *argv])
    with pytest.raises(StopServer):
        main_mod.main()
    import time
    time.sleep(0.2)  # threads started by main()
    web_module.demo.stop()
    return started, web_module.demo


def test_continuous_simulate_runs_the_loop_as_before(monkeypatch):
    started, demo = _start(monkeypatch, ["--simulate"], None)
    assert started["sim"] == 1 and started["capture"] == 0 and demo.mode == "continuous"
    assert demo.active() == []


def test_on_demand_simulate_runs_no_background_simulation(monkeypatch):
    started, demo = _start(monkeypatch, ["--simulate"], "on_demand")
    assert started == {"sim": 0, "capture": 0, "ble": 0} and demo.mode == "on_demand"


def test_on_demand_live_keeps_real_capture(monkeypatch):
    started, demo = _start(monkeypatch, [], "on_demand")
    assert started["sim"] == 0 and started["capture"] == 1 and started["ble"] == 1
    assert demo.mode == "on_demand"
