"""Step-by-step demo (ui-kit DEMO-TRIGGER-CONTRACT.md): simulated drones fired on demand.

``DEMO_MODE=continuous`` (default) changes nothing: ``--simulate`` runs its 5-minute loop as
before. ``DEMO_MODE=on_demand`` starts no background simulation, and real Wi-Fi and Bluetooth
Remote ID capture keeps running (unless ``--simulate``, which never captures). In both modes the
presenter can fire a scenario from the page or the launcher:

* ``inbound``: one Remote ID quad flies at the OP from 1.2 km NE (Closing, CPA 120 m after
  about 2 min), stops there, comes down and keeps broadcasting from the ground.
* ``crossing``: a DJI Wi-Fi drone orbits the OP at 600 m (Crossing), with its pilot located.
* ``outbound``: a Remote ID quad leaves from 200 m S of the OP (Opening).
* ``swarm``: three Remote ID quads at once, in loose formation from 1.5 km NE.

The flights are the simulator's own models (``detector.simulator``), in the real wire format,
so the real parsers, classifier, motion, CoT and mesh code run on them. Demo drones have their
own MACs and serials, so they never merge with the continuous loop's. Every frame is marked
simulated: CoT callsigns ``SIM-``, remarks ``SIMULATED: ``, mesh texts ``[SIM] ``, SIM tag on the page.

``clear()`` stops every scenario and removes their drones (live or ended) from the picture and
from ATAK at once: each marker (and its operator marker) again with stale = now, then a
``t-x-d-d`` forcedelete, then the registration SA. Real detections are never touched.
"""

from __future__ import annotations

import logging
import os
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

from detector.simulator import DEFAULT_CENTRE, TICK_S, DjiOrbit, RemoteIdFlight

log = logging.getLogger("detector.demo")

MODES = ("continuous", "on_demand")


def demo_mode(value: str | None = None) -> str:
    """``DEMO_MODE`` (continuous | on_demand); anything else is continuous, with a warning."""
    raw = (os.environ.get("DEMO_MODE", "") if value is None else value).strip().lower().replace("-", "_")
    if raw in ("", "continuous"):
        return "continuous"
    if raw in ("on_demand", "ondemand", "step", "step_by_step"):
        return "on_demand"
    log.warning("DEMO_MODE=%r not understood: using continuous", raw)
    return "continuous"


@dataclass(frozen=True)
class Scenario:
    id: str
    label: str
    hint: str
    duration_s: float
    flights: Callable[[], list]  # fresh flight objects for one run

    def to_dict(self) -> dict:
        return {"id": self.id, "label": self.label, "hint": self.hint, "duration_s": self.duration_s}


def _inbound():
    # 1.2 km out along 045°, shifted 120 m sideways: CPA 120 m at t=120 s, then it lands there
    return [RemoteIdFlight("BLE:D0:5A:7E:10:01:01", b"1581F5DEMO0IN001", (848.5 + 85.0, 848.5 - 85.0),
                           225.0, 10.0, 80.0, operator=(500.0, 420.0), operator_id=b"GBR-OP-DEMOIN01",
                           stop_after_s=120.0, descent_mps=2.5)]


def _crossing():
    return [DjiOrbit("60:60:1F:D3:40:11", b"08QDE5KDEMO0X01", "DJI-MINI4-D34011", "60:60:1F:D3:40:12",
                     radius=600.0, speed=12.0, start_deg=60.0, alt=70.0,
                     home=(-1200.0, -400.0), pilot=(-1190.0, -410.0))]


def _outbound():
    return [RemoteIdFlight("BLE:D0:5A:7E:20:01:01", b"1581F5DEMO0OUT01", (0.0, -200.0), 200.0, 10.0, 60.0,
                           operator=(60.0, -150.0), operator_id=b"GBR-OP-DEMOOUT1")]


def _swarm():
    # line abreast 150 m apart, 1.5 km NE, all on 225° at 10 m/s: CPAs about 150, 40 and 150 m
    out = []
    for i, (side, alt) in enumerate(((-150.0, 60.0), (40.0, 80.0), (150.0, 100.0)), start=1):
        out.append(RemoteIdFlight(f"BLE:D0:5A:7E:30:0{i}:01", f"1581F5DEMO0SW00{i}".encode(),
                                  (1060.7 + side * 0.7071, 1060.7 - side * 0.7071), 225.0, 10.0, alt,
                                  operator=(900.0, 800.0), operator_id=b"GBR-OP-DEMOSWRM"))
    return out


SCENARIOS: dict[str, Scenario] = {s.id: s for s in (
    Scenario("inbound", "Fly inbound", "One Remote ID drone flies at the OP: closing, CPA, then lands",
             175.0, _inbound),
    Scenario("crossing", "Fly crossing", "A DJI drone circles the OP at 600 m, pilot located", 180.0, _crossing),
    Scenario("outbound", "Fly outbound", "A Remote ID drone flies away from the OP", 150.0, _outbound),
    Scenario("swarm", "Launch swarm of 3", "Three Remote ID drones inbound together from the north-east", 180.0, _swarm),
)}


def flight_macs(flight) -> list[str]:
    macs = [flight.mac]
    if getattr(flight, "controller_mac", None):
        macs.append(flight.controller_mac)
    return macs


@dataclass
class _Run:
    scenario: Scenario
    t0: float
    flights: list = field(default_factory=list)
    seq: int = 0

    @property
    def until(self) -> float:
        return self.t0 + self.scenario.duration_s


class DemoRunner:
    """Runs triggered scenarios on its own thread and clears them on request."""

    def __init__(self, engine, centre: tuple[float, float] | None = None, cot=None, mesh=None,
                 mode: str = "continuous", clock: Callable[[], float] = time.time):
        self.engine = engine
        self.centre = centre or DEFAULT_CENTRE
        self.cot = cot
        self.mesh = mesh
        self.mode = mode
        self._clock = clock
        self._lock = threading.Lock()
        self._runs: dict[str, _Run] = {}
        self._macs: set[str] = set()  # every MAC a demo has used: what clear() may remove
        self._running = False
        self.triggered = 0

    # ── API ───────────────────────────────────────────────────────

    def scenarios(self) -> dict:
        return {"mode": self.mode, "scenarios": [s.to_dict() for s in SCENARIOS.values()],
                "active": self.active()}

    def active(self) -> list[str]:
        now = self._clock()
        with self._lock:
            return [sid for sid, r in self._runs.items() if now < r.until]

    def trigger(self, scenario_id: str, **_options) -> dict:
        """Start a scenario (restarts it if it is already running). KeyError if unknown."""
        scenario = SCENARIOS[scenario_id]
        now = self._clock()
        run = _Run(scenario, now, scenario.flights())
        macs = [m for f in run.flights for m in flight_macs(f)]
        with self._lock:
            restarted = scenario_id in self._runs
            self._runs[scenario_id] = run
            self._macs.update(macs)
        if restarted:  # start again from the beginning, not from where the old track was
            self.engine.remove(macs)
        self.triggered += 1
        log.info("Demo scenario %s started (%s, %.0f s, SIMULATED)", scenario_id, scenario.label,
                 scenario.duration_s)
        return {"ok": True, "id": scenario_id, "until": int(run.until)}

    def clear(self) -> dict:
        """Stop every scenario and remove their drones from the picture and from ATAK at once."""
        with self._lock:
            stopped = len([r for r in self._runs.values() if self._clock() < r.until])
            self._runs.clear()
            macs = list(self._macs)
        removed = self.engine.remove(macs)
        if self.mesh is not None:
            self.mesh.forget(macs)
        retracted = 0
        if self.cot is not None:
            for dev in removed:
                retracted += self.cot.retract(dev)
            if retracted:
                self.cot.client.send_sa_now()  # OTS marks the sender disconnected on t-x-d-d
        log.info("Demo cleared: %d scenarios stopped, %d simulated tracks removed, %d CoT deletes",
                 stopped, len(removed), retracted)
        return {"ok": True, "cleared": stopped, "tracks": len(removed)}

    # ── running ───────────────────────────────────────────────────

    def _emit(self, frame: dict, probability: float = 1.0):
        if random.random() < probability:
            self.engine.process_frame(frame)

    def tick(self, now: float | None = None):
        now = self._clock() if now is None else now
        with self._lock:
            runs = list(self._runs.items())
        for sid, run in runs:
            if now >= run.until:
                with self._lock:
                    if self._runs.get(sid) is run:
                        del self._runs[sid]
                log.info("Demo scenario %s finished (its CoT expires as normal)", sid)
                continue
            t = now - run.t0
            run.seq += 1
            for flight in run.flights:
                if isinstance(flight, RemoteIdFlight):
                    flight.tick(t, self.centre, self._emit, run.seq)
                else:
                    flight.tick(t, self.centre, self._emit)

    def start(self) -> "DemoRunner":
        if not self._running:
            self._running = True
            threading.Thread(target=self._loop, daemon=True, name="demo").start()
        return self

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            try:
                self.tick()
            except Exception:
                log.exception("Demo tick failed")
            time.sleep(TICK_S)
