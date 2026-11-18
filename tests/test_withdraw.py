"""Withdrawing the continuous simulator's CoT: on shutdown and on any start in on_demand mode.

After a switch from continuous to step-by-step the old SIM- drones stayed on ATAK until stale
(5 min). Their uids are fixed, so a stop, or the next start in on_demand, takes them off at once:
each marker again with stale = now, then a t-x-d-d forcedelete, then the registration SA."""

import random
import time
import xml.etree.ElementTree as ET

import pytest

from detector import simulator
from detector.cot_emitter import CotEmitter
from detector.engine import DetectionEngine
from detector.simulator import DEFAULT_CENTRE, PERIOD_S, DemoScenario, continuous_tracks
from detector.tak_client import TakClient
from tests.fake_ots import FakeOTS
from tests.test_demo import _start

WANT = {"DRONE-SIM-BLEE17A3C902241", "DRONE-SIM-BLEE17A3C902241-OPERATOR", "DRONE-SIM-60601F5A21C4",
        "DRONE-SIM-60601F5A21C4-OPERATOR", "DRONE-SIM-BLEC42D91075E13", "DRONE-SIM-903AE6410C7E"}


class Recorder(TakClient):
    def __init__(self):
        super().__init__("drone-detector", "DRONE-DET", station="test", sensor_lat=DEFAULT_CENTRE[0],
                         sensor_lon=DEFAULT_CENTRE[1])
        self.out = []

    def send(self, xml):
        self.out.append(xml)

    def start(self):
        return self


def emitter(**kw):
    cot = CotEmitter(sensor_lat=DEFAULT_CENTRE[0], sensor_lon=DEFAULT_CENTRE[1], **kw)
    cot.client = Recorder()
    return cot


def test_withdraw_list_covers_every_marker_the_continuous_loop_sends():
    random.seed(3)
    cot = emitter(simulated=True)
    built = set()
    engine = DetectionEngine(on_alert=lambda d: built.update(ET.fromstring(x).get("uid") for x in cot.build(d)))
    engine.op = DEFAULT_CENTRE
    sim = DemoScenario(engine, DEFAULT_CENTRE)
    sim.t0 = 0.0
    for i in range(int(PERIOD_S / simulator.TICK_S)):
        sim.tick(i * simulator.TICK_S + 0.001)
    assert built - WANT == set(), "a continuous drone the withdrawal would miss"
    assert {u for u in built if not u.endswith("OPERATOR")} == {u for u in WANT if not u.endswith("OPERATOR")}


@pytest.mark.parametrize("simulated", [True, False])  # --simulate, or live capture in on_demand
def test_withdraw_sends_stale_markers_then_deletes_then_sa(simulated):
    cot = emitter(simulated=simulated)
    assert cot.withdraw(continuous_tracks(), *DEFAULT_CENTRE, now=1_000_000.0) == 6
    events = [ET.fromstring(x) for x in cot.client.out]
    markers, deletes, sa = events[:6], events[6:12], events[12:]
    assert {e.get("uid") for e in markers} == WANT
    assert all(e.get("stale") == e.get("time") and e.find("detail/remarks").text.startswith("SIMULATED: ")
               for e in markers)
    assert {e.find("detail/contact").get("callsign") for e in markers} >= {
        "SIM-1581F5BKD23C0042", "SIM-08QDE5K0010042", "SIM-1581F5FKD2B10099", "SIM-UAV 0C:7E"}
    assert all(e.get("type") == "t-x-d-d" and e.find("detail/__forcedelete") is not None for e in deletes)
    assert {e.find("detail/link").get("uid") for e in deletes} == WANT
    assert len(sa) == 1 and sa[0].get("uid") == cot.client.uid, "SA last: OTS marks us disconnected on t-x-d-d"


def test_restart_withdraws_what_the_previous_run_left_on_the_ots():
    ots = FakeOTS(bind_delay=0.1)
    try:
        # a continuous --simulate run puts its drones on the map, then dies without cleaning up
        first = CotEmitter(tak_host="127.0.0.1", tak_port=ots.port, sensor_lat=DEFAULT_CENTRE[0],
                           sensor_lon=DEFAULT_CENTRE[1], simulated=True)
        first.client.register_delay = 0.15
        random.seed(5)
        engine = DetectionEngine(on_alert=first.emit)
        engine.op = DEFAULT_CENTRE
        sim = DemoScenario(engine, DEFAULT_CENTRE)
        sim.t0 = 0.0
        for i in range(int(PERIOD_S / simulator.TICK_S)):
            sim.tick(i * simulator.TICK_S + 0.001)
        assert first.client.flush(5)
        first.stop()
        left = {u for u in ots.markers if u.startswith("DRONE-SIM-")}
        assert len(left) >= 4
        # the restarted detector (another process: a fresh emitter) takes them off
        second = CotEmitter(tak_host="127.0.0.1", tak_port=ots.port, sensor_lat=DEFAULT_CENTRE[0],
                            sensor_lon=DEFAULT_CENTRE[1])
        second.client.register_delay = 0.15
        second.withdraw(continuous_tracks(), *DEFAULT_CENTRE)
        assert second.client.flush(5)
        time.sleep(0.2)
        deleted = {u.split("-del-")[0] for u in ots.relayed if "-del-" in u}
        assert left <= deleted == WANT and not ots.nacked and not ots.marker_errors
        second.stop()
    finally:
        ots.close()


def test_flush_waits_for_the_queue_and_gives_up_without_a_server():
    ots = FakeOTS(bind_delay=0.1)
    try:
        c = TakClient("drone-detector", "DRONE-DET", host="127.0.0.1", port=ots.port, station="t",
                      register_delay=0.2, sensor_lat=51.0, sensor_lon=-1.0)
        c.send(c.marker("X-1", "a-u-G", "X", 51.0, -1.0))
        c.start()
        assert c.flush(5) and c.sent == 1
        c.stop()
    finally:
        ots.close()
    dead = TakClient("drone-detector", "DRONE-DET", host="127.0.0.1", port=ots.port, station="t")
    dead.send("<event/>")
    t = time.monotonic()
    assert dead.flush(0.3) is False and time.monotonic() - t < 1.0


# ── start-up and shutdown in __main__ ─────────────────────────────

def _deletes_through_main(monkeypatch, argv, mode):
    monkeypatch.setenv("TAK_REGISTER_DELAY_S", "0.15")
    ots = FakeOTS(bind_delay=0.1)
    try:
        _start(monkeypatch, argv, mode, tak_port=ots.port)  # runs main() through its shutdown
        from detector.web import app as web_module
        web_module.status["cot"].stop()
        time.sleep(0.2)
        return {u.split("-del-")[0] for u in ots.relayed if "-del-" in u}, ots
    finally:
        ots.close()


def test_on_demand_start_withdraws_the_continuous_markers(monkeypatch):
    deleted, ots = _deletes_through_main(monkeypatch, ["--simulate"], "on_demand")
    assert deleted == WANT and not ots.nacked


def test_on_demand_live_start_withdraws_them_too(monkeypatch):
    deleted, _ = _deletes_through_main(monkeypatch, [], "on_demand")
    assert deleted == WANT


def test_continuous_simulate_withdraws_at_shutdown(monkeypatch):
    deleted, _ = _deletes_through_main(monkeypatch, ["--simulate"], None)
    assert deleted == WANT


def test_live_continuous_sends_no_deletes(monkeypatch):
    deleted, _ = _deletes_through_main(monkeypatch, [], None)
    assert deleted == set(), "no simulator ran: nothing to withdraw"


def test_shutdown_also_clears_demo_scenario_drones():
    from types import SimpleNamespace

    from detector.__main__ import shutdown_cleanup
    from detector.demo import DemoRunner

    cot = emitter()
    engine = DetectionEngine(on_alert=cot.emit)
    engine.op = DEFAULT_CENTRE
    demo = DemoRunner(engine, centre=DEFAULT_CENTRE, cot=cot, mode="on_demand")
    demo.trigger("outbound")
    for i in range(30):
        demo.tick(demo._runs["outbound"].t0 + i * 0.5)
    called = []
    shutdown_cleanup(SimpleNamespace(simulate=False, demo_mode="on_demand"), cot, demo, called.append, flush_s=0.1)
    links = {ET.fromstring(x).find("detail/link").get("uid") for x in cot.client.out if "t-x-d-d" in x}
    assert "DRONE-SIM-BLED05A7E200101" in links and called == [], "on_demand withdrew the loop at start-up"
