"""Emit drone detections as CoT XML over TCP to a TAK server (OpenTAKServer on the Pi).

Plain CoT over TCP to ``TAK_HOST:TAK_PORT`` (default 127.0.0.1:8087, OpenTAKServer's TCP
streaming port as configured by blufor-tracker's install). OTS needs no handshake
from a sender. Events with no ``<dest>`` go to every connected client.

Sending runs on a background thread with a bounded queue and reconnect backoff,
so a missing or restarting TAK server never stalls packet capture.

Markers:
* DRONE_CONFIRMED with a position: ``a-h-A-M-F-Q`` (hostile air UAV) at the broadcast position.
  ``COT_AFFILIATION=u`` makes it unknown instead of hostile.
* DRONE_PROBABLE or no position: ``a-u-A-M-F-Q`` (unknown air UAV) at the sensor's own position
  (``SENSOR_LAT``/``SENSOR_LON``), with the RSSI distance as the error circle. Without a sensor
  position nothing is placed on the map, rather than putting a marker at 0,0.
* Operator/pilot position from Remote ID / DroneID: ``a-u-G`` ground marker linked to the UAV.
"""

from __future__ import annotations

import logging
import os
import queue
import socket
import threading
import time
from datetime import datetime, timezone
from xml.sax.saxutils import escape, quoteattr

from detector.models import Classification, DetectedDevice

logger = logging.getLogger(__name__)

UID_PREFIX = "DRONE-"
STALE_SECONDS = 120
QUEUE_MAX = 200
BACKOFF_MIN, BACKOFF_MAX = 1.0, 30.0


def _isotime(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _event(uid, cot_type, lat, lon, hae, ce, callsign, remarks, now, extra=""):
    return (
        f'<event version="2.0" uid={quoteattr(uid)} type={quoteattr(cot_type)} how="m-g"'
        f' time="{_isotime(now)}" start="{_isotime(now)}" stale="{_isotime(now + STALE_SECONDS)}">'
        f'<point lat="{lat:.7f}" lon="{lon:.7f}" hae="{hae:.1f}" ce="{ce:.0f}" le="50"/>'
        f"<detail><contact callsign={quoteattr(callsign)}/><remarks>{escape(remarks)}</remarks>{extra}"
        f"</detail></event>"
    )


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
        self.connected = False
        self._queue: queue.Queue[str] = queue.Queue(maxsize=QUEUE_MAX)
        self._running = False

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

        extra = ""
        if d and (d.speed or d.heading):
            extra = f'<track speed="{d.speed:.1f}" course="{d.heading:.1f}"/>'
        events = [_event(uid, f"a-{aff}-A-M-F-Q", lat, lon, hae, ce, callsign,
                         ", ".join(p for p in parts if p), now, extra)]

        if d and d.has_operator:
            events.append(_event(
                f"{uid}-OPERATOR", "a-u-G", d.operator_lat, d.operator_lon, 0.0, 15.0,
                f"{callsign} operator", f"drone-detector: pilot/operator position for {callsign}", now,
                f'<link uid={quoteattr(uid)} relation="p-p" type="a-{aff}-A-M-F-Q"/>',
            ))
        return events

    # ── sending ───────────────────────────────────────────────────

    def emit(self, device: DetectedDevice):
        for xml in self.build(device):
            try:
                self._queue.put_nowait(xml)
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                self._queue.put_nowait(xml)
        if not self._running:
            self.start()

    def start(self):
        if self._running:
            return
        self._running = True
        threading.Thread(target=self._sender, daemon=True, name="cot-sender").start()

    def stop(self):
        self._running = False

    def _sender(self):
        backoff = BACKOFF_MIN
        sock = None
        pending: str | None = None
        while self._running:
            if sock is None:
                try:
                    sock = socket.create_connection((self.tak_host, self.tak_port), timeout=5)
                    sock.settimeout(10)
                    self.connected = True
                    backoff = BACKOFF_MIN
                    logger.info("Connected to TAK server at %s:%d", self.tak_host, self.tak_port)
                except OSError as e:
                    self.connected = False
                    logger.warning("TAK server %s:%d unavailable (%s) — retry in %.0fs",
                                   self.tak_host, self.tak_port, e, backoff)
                    time.sleep(backoff)
                    backoff = min(backoff * 2, BACKOFF_MAX)
                    continue
            if pending is None:
                try:
                    pending = self._queue.get(timeout=1.0)
                except queue.Empty:
                    continue
            try:
                sock.sendall(pending.encode("utf-8"))
                pending = None
            except OSError as e:
                logger.warning("TAK connection lost (%s), reconnecting", e)
                self.connected = False
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None
