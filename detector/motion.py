"""Course, speed and motion relative to the OP for each drone.

* **Course and speed**: the broadcast track (Remote ID track direction and ground speed, DJI
  velocity N/E) when present, else derived from position fixes at least ``MIN_DT_S`` (5 s) or
  ``MIN_DIST_M`` (10 m) apart.
* **Relative motion**: radial speed is the velocity component along the line from the OP to the
  drone (negative = coming towards the OP). Below -``RADIAL_MPS`` it is *closing*, above
  +``RADIAL_MPS`` *opening*, otherwise *crossing*, or *hovering* below ``HOVER_MPS`` ground speed.
  The label uses the median over the last 3 to 5 fixes so it doesn't flicker.
* **CPA**: when closing, the time and distance of the closest point of approach to the OP on the
  current straight-line track.
* **No position** (RSSI only): the signal trend over ``RSSI_WINDOW_S`` (20 s). A clear rise is
  "closing~", a clear fall "opening~" (the "~" marks an estimate); never a course.

Flat-earth (local east/north metres) maths: fine for the few kilometres a sensor hears.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from statistics import median

M_PER_DEG = 111_320.0
MIN_DT_S = 5.0
MIN_DIST_M = 10.0
RADIAL_MPS = 1.0
HOVER_MPS = 1.0
SMOOTH_N = 5  # fixes in the smoothing window (at least 3 used for a label)
MIN_LABEL_FIXES = 2
MAX_FIXES = 12
MAX_AGE_S = 60.0
RSSI_WINDOW_S = 20.0
RSSI_MIN_SAMPLES = 6
RSSI_MIN_SPAN_S = 10.0
RSSI_MIN_CHANGE_DB = 4.0  # over the window
RSSI_MIN_T = 3.0  # slope / standard error

POINTS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")


def enu(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """(east, north) metres of lat/lon from the reference point."""
    return ((lon - ref_lon) * M_PER_DEG * math.cos(math.radians(ref_lat)), (lat - ref_lat) * M_PER_DEG)


def course_of(ve: float, vn: float) -> float:
    """True bearing in [0, 360) of a velocity (east, north)."""
    return math.degrees(math.atan2(ve, vn)) % 360


def velocity(course: float, speed: float) -> tuple[float, float]:
    rad = math.radians(course)
    return speed * math.sin(rad), speed * math.cos(rad)


def fmt_course(course: float) -> str:
    """3-digit true bearing, wrapping 359.6 to 000."""
    return f"{int(round(course)) % 360:03d}°"


def compass(course: float) -> str:
    return POINTS[int((course % 360 + 22.5) // 45) % 8]


def radial_speed(east: float, north: float, ve: float, vn: float) -> float:
    """Velocity component along the OP→drone line: negative when approaching the OP."""
    r = math.hypot(east, north)
    if r < 1e-6:
        return 0.0
    return (east * ve + north * vn) / r


def cpa(east: float, north: float, ve: float, vn: float) -> tuple[float, float] | None:
    """(seconds until, metres at) the closest point of approach to the OP, or None if moving away
    or not moving."""
    v2 = ve * ve + vn * vn
    if v2 < 1e-9:
        return None
    t = -(east * ve + north * vn) / v2
    if t <= 0:
        return None
    return t, math.hypot(east + ve * t, north + vn * t)


def fmt_mmss(seconds: float) -> str:
    s = max(0, int(round(seconds)))
    return f"{s // 60:02d}:{s % 60:02d}"


def rssi_trend(history: list[tuple[float, int]], now: float, window: float = RSSI_WINDOW_S) -> str:
    """'closing' / 'opening' from a clear RSSI rise/fall over the window, else ''."""
    pts = [(t, r) for t, r in history if now - t <= window]
    if len(pts) < RSSI_MIN_SAMPLES:
        return ""
    ts = [t for t, _ in pts]
    span = max(ts) - min(ts)
    if span < RSSI_MIN_SPAN_S:
        return ""
    n = len(pts)
    tm = sum(ts) / n
    rm = sum(r for _, r in pts) / n
    sxx = sum((t - tm) ** 2 for t in ts)
    slope = sum((t - tm) * (r - rm) for t, r in pts) / sxx
    resid = [r - (rm + slope * (t - tm)) for t, r in pts]
    sigma = math.sqrt(sum(e * e for e in resid) / max(1, n - 2)) or 0.5
    t_stat = abs(slope) / (sigma / math.sqrt(sxx))
    if abs(slope) * span < RSSI_MIN_CHANGE_DB or t_stat < RSSI_MIN_T:
        return ""
    return "closing" if slope > 0 else "opening"


@dataclass
class Motion:
    course: float | None = None  # true bearing of travel
    speed: float | None = None  # m/s over the ground
    source: str = ""  # broadcast | fixes | rssi | ""
    relation: str = ""  # closing | opening | crossing | hovering | ""
    estimate: bool = False  # RSSI-only
    radial: float | None = None  # m/s, negative = towards the OP
    cpa_s: float | None = None
    cpa_m: float | None = None

    @property
    def label(self) -> str:
        """'Closing · CPA 120 m in 01:40', 'Opening', 'Crossing', 'Hovering', 'Closing~' or '–'."""
        if not self.relation:
            return "–"
        text = self.relation.capitalize() + ("~" if self.estimate else "")
        if self.relation == "closing" and self.cpa_s is not None and self.cpa_m is not None:
            text += f" · CPA {self.cpa_m:.0f} m in {fmt_mmss(self.cpa_s)}"
        return text

    @property
    def course_text(self) -> str:
        return fmt_course(self.course) if self.course is not None else ""

    def mesh_text(self) -> str:
        """'hdg 225 closing' (mesh alerts), 'closing~' for an RSSI estimate, '' if unknown."""
        parts = []
        if self.course is not None and self.speed is not None and self.speed >= HOVER_MPS:
            parts.append(f"hdg {int(round(self.course)) % 360:03d}")
        if self.relation:
            parts.append(self.relation + ("~" if self.estimate else ""))
        return " ".join(parts)

    def to_dict(self) -> dict:
        return {"course": None if self.course is None else round(self.course, 1),
                "course_text": self.course_text,
                "speed": None if self.speed is None else round(self.speed, 1),
                "source": self.source, "relation": self.relation, "estimate": self.estimate,
                "radial": None if self.radial is None else round(self.radial, 2),
                "cpa_s": None if self.cpa_s is None else round(self.cpa_s),
                "cpa_m": None if self.cpa_m is None else round(self.cpa_m),
                "label": self.label}


@dataclass
class _Fix:
    t: float
    lat: float
    lon: float
    vel: tuple[float, float] | None = None  # (east, north) m/s from the broadcast
    broadcast: bool = False


class MotionTrack:
    """Recent position fixes of one drone."""

    def __init__(self, maxlen: int = MAX_FIXES):
        self.fixes: deque[_Fix] = deque(maxlen=maxlen)

    def add_fix(self, t: float, lat: float, lon: float, course: float | None = None,
                speed: float | None = None):
        """One position report; ``course``/``speed`` only if broadcast with it."""
        if not (lat or lon):
            return
        vel = velocity(course, speed) if course is not None and speed is not None else None
        fix = _Fix(t, lat, lon, vel, vel is not None)
        if self.fixes and t - self.fixes[-1].t < 1.0:
            self.fixes[-1] = fix  # several frames a second: keep the latest
        else:
            self.fixes.append(fix)
        while self.fixes and t - self.fixes[0].t > MAX_AGE_S:
            self.fixes.popleft()

    def _velocities(self, ref: tuple[float, float]) -> list[tuple[_Fix, tuple[float, float]]]:
        """(fix, velocity) for each fix with a broadcast or derivable velocity, oldest first."""
        out = []
        fixes = list(self.fixes)
        for i, f in enumerate(fixes):
            if f.vel is not None:
                out.append((f, f.vel))
                continue
            e1, n1 = enu(f.lat, f.lon, *ref)
            for g in reversed(fixes[:i]):  # most recent earlier fix that is far enough back
                e0, n0 = enu(g.lat, g.lon, *ref)
                dt = f.t - g.t
                if dt > 0 and (dt >= MIN_DT_S or math.hypot(e1 - e0, n1 - n0) >= MIN_DIST_M):
                    out.append((f, ((e1 - e0) / dt, (n1 - n0) / dt)))
                    break
        return out

    def summary(self, op: tuple[float, float] | None) -> Motion:
        if not self.fixes:
            return Motion()
        last = self.fixes[-1]
        ref = op if op else (last.lat, last.lon)
        vels = self._velocities(ref)[-SMOOTH_N:]
        if not vels:
            return Motion()
        ve = sum(v[0] for _, v in vels) / len(vels)
        vn = sum(v[1] for _, v in vels) / len(vels)
        latest_fix, latest_v = vels[-1]
        if latest_fix.broadcast:  # report what the drone says, smooth only the label
            course, speed = course_of(*latest_v), math.hypot(*latest_v)
            source = "broadcast"
        else:
            course, speed = course_of(ve, vn), math.hypot(ve, vn)
            source = "fixes"
        m = Motion(course=course, speed=speed, source=source)
        if op is None or (len(vels) < MIN_LABEL_FIXES and not latest_fix.broadcast):
            return m
        radials = [radial_speed(*enu(f.lat, f.lon, *op), *v) for f, v in vels]
        speeds = [math.hypot(*v) for _, v in vels]
        m.radial = median(radials)
        if median(speeds) < HOVER_MPS:
            m.relation = "hovering"
        elif m.radial < -RADIAL_MPS:
            m.relation = "closing"
            hit = cpa(*enu(last.lat, last.lon, *op), ve, vn)
            if hit:
                m.cpa_s, m.cpa_m = hit
        elif m.radial > RADIAL_MPS:
            m.relation = "opening"
        else:
            m.relation = "crossing"
        return m


def rssi_motion(history: list[tuple[float, int]], now: float) -> Motion:
    trend = rssi_trend(history, now)
    return Motion(source="rssi" if trend else "", relation=trend, estimate=bool(trend))
