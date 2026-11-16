"""Emit drone detections as CoT XML over TCP to a TAK server (OpenTAKServer on the Pi).

CoT goes over TCP to ``TAK_HOST:TAK_PORT`` (default 127.0.0.1:8087, OpenTAKServer's TCP
streaming port as configured by blufor-tracker's install) through ``detector.tak_client``
(kit contract v5). It is one long-lived connection that registers this sensor with OTS
(uid ``isrkit-drone-detector-<ISR_HOST>``, callsign ``DRONE-DET <ISR_HOST>``) 3 s after each
connect, before any marker is sent. Real OTS on PostgreSQL drops CoT from unregistered senders.

Sending runs on that client's background thread with a bounded queue and reconnect backoff,
so a missing or restarting TAK server never stalls packet capture.

Markers:
* DRONE_CONFIRMED with a position: ``a-h-A-M-F-Q`` (hostile air UAV) at the broadcast position.
  ``COT_AFFILIATION=u`` makes it unknown instead of hostile.
* DRONE_PROBABLE or no position: ``a-u-A-M-F-Q`` (unknown air UAV) at the sensor's own position
  (``SENSOR_LAT``/``SENSOR_LON``), with the RSSI distance as the error circle. Without a sensor
  position nothing is placed on the map, rather than putting a marker at 0,0.
* Operator/pilot position from Remote ID / DroneID: ``a-u-G`` ground marker (its remarks name the UAV).
  Every marker's ``<link>`` points at this tool's registered uid: OTS only stores markers whose
  parent is an EUD.

Rate (contract C4.4): at most one update per ``MIN_UPDATE_S`` (5 s) per track, with a long stale
time so markers don't flicker between updates. A change of classification is sent at once.
Labelling (C4.3): in ``--simulate`` the remarks start ``SIMULATED: `` and callsigns ``SIM-``.
"""

from __future__ import annotations

import logging
import os
import time

from detector.models import Classification, DetectedDevice
from detector.tak_client import TakClient

logger = logging.getLogger(__name__)

UID_PREFIX = "DRONE-"
STALE_SECONDS = 300
MIN_UPDATE_S = 5.0
SIM_REMARKS = "SIMULATED: "
TAK_REPO, TAK_SHORT = "drone-detector", "DRONE-DET"


class CotEmitter:
    """Builds drone CoT and sends it to the TAK server in the background."""

    def __init__(self, tak_host: str | None = None, tak_port: int | None = None,
                 sensor_lat: float | None = None, sensor_lon: float | None = None,
                 affiliation: str | None = None, simulated: bool = False):
        self.tak_host = tak_host or os.environ.get("TAK_HOST", "127.0.0.1")
        self.tak_port = int(tak_port or os.environ.get("TAK_PORT", "8087"))
        self.sensor_lat = sensor_lat
        self.sensor_lon = sensor_lon
        self.affiliation = (affiliation or os.environ.get("COT_AFFILIATION", "h")).strip()[:1] or "h"
        self.simulated = simulated
        self.client = TakClient(TAK_REPO, TAK_SHORT, host=self.tak_host, port=self.tak_port,
                                sensor_lat=sensor_lat, sensor_lon=sensor_lon)
        self._last_sent: dict[str, tuple[float, Classification]] = {}
        self.sent = 0
        self.throttled = 0

    # ── building ──────────────────────────────────────────────────

    def build(self, device: DetectedDevice, now: float | None = None) -> list[str]:
        """CoT events for a device (UAV marker, plus operator marker if known)."""
        now = time.time() if now is None else now
        if device.classification not in (Classification.DRONE_CONFIRMED, Classification.DRONE_PROBABLE):
            return []

        d = device.drone_id
        tag = "SIM-" if self.simulated else ""
        uid = f"{UID_PREFIX}{tag}{device.mac.replace(':', '')}"
        callsign = f"{tag}{d.serial}" if d and d.serial else f"{tag}UAV {device.mac[-5:]}"

        parts = [f"drone-detector: {device.classification.value}", device.reason or None,
                 f"RSSI {device.rssi} dBm"]
        if device.vendor:
            parts.append(f"vendor {device.vendor}")
        if device.ssid:
            parts.append(f"SSID {device.ssid}")
        if d:
            if d.ua_type:
                parts.append(d.ua_type)
            if d.speed:
                parts.append(f"{d.speed:.1f} m/s")
            if d.height_agl:
                parts.append(f"{d.height_agl:.0f} m AGL")
            if d.operator_id:
                parts.append(f"operator ID {d.operator_id}")
            if d.description:
                parts.append(d.description)
        prefix = SIM_REMARKS if self.simulated else ""
        dist = device.distance_estimate_m

        if d and d.has_position:
            lat, lon, hae, ce = d.drone_lat, d.drone_lon, d.drone_alt, 15.0
            aff = self.affiliation if device.classification == Classification.DRONE_CONFIRMED else "u"
        elif self.sensor_lat is not None and self.sensor_lon is not None:
            lat, lon, hae = self.sensor_lat, self.sensor_lon, 0.0
            ce = max(dist or 300.0, 50.0)
            aff = "u"
            parts.append(f"no GPS in broadcast — within ~{ce:.0f} m of the sensor")
        else:
            logger.debug("No position for %s and no SENSOR_LAT/LON set — not sending CoT", device.mac)
            return []

        track = {}
        if d and (d.speed or d.heading):
            track = {"speed": d.speed, "course": d.heading}
        # tak_client.marker: parent link to this tool's registered uid, so OTS also stores it
        # (OTS web map); see tak_client's module docstring for the fields OTS requires
        uav = self.client.marker(uid, f"a-{aff}-A-M-F-Q", callsign, lat, lon, now=now, stale_s=STALE_SECONDS,
                                 hae=hae, ce=ce, le=50.0, remarks=prefix + ", ".join(p for p in parts if p),
                                 location_source="GPS" if d and d.has_position else None, **track)
        events = [uav] if uav else []

        if d and d.has_operator:
            op = self.client.marker(f"{uid}-OPERATOR", "a-u-G", f"{callsign} operator", d.operator_lat,
                                    d.operator_lon, now=now, stale_s=STALE_SECONDS, ce=15.0, le=50.0,
                                    remarks=f"{prefix}drone-detector: pilot/operator position for {callsign} "
                                            f"(UAV uid {uid})")
            if op:
                events.append(op)
        return events

    # ── sending ───────────────────────────────────────────────────

    def should_send(self, device: DetectedDevice, now: float) -> bool:
        """Per-track rate limit: one update per MIN_UPDATE_S, except on a classification change."""
        last = self._last_sent.get(device.mac)
        if last is not None and last[1] == device.classification and now - last[0] < MIN_UPDATE_S:
            return False
        return True

    def emit(self, device: DetectedDevice, now: float | None = None):
        now = time.time() if now is None else now
        if not self.should_send(device, now):
            self.throttled += 1
            return
        events = self.build(device, now)
        if not events:
            return
        self._last_sent[device.mac] = (now, device.classification)
        if len(self._last_sent) > 2000:  # forget long-gone tracks
            for mac in [m for m, (t, _) in self._last_sent.items() if now - t > STALE_SECONDS]:
                del self._last_sent[mac]
        self.sent += 1
        for xml in events:
            self.client.send(xml)
        self.client.start()

    @property
    def connected(self) -> bool:
        """True once registered with the TAK server (not just TCP-connected)."""
        return self.client.registered.is_set()

    def start(self):
        self.client.start()

    def stop(self):
        self.client.stop()
