"""Mesh (LoRa) alerts for NEW drone detections, via meshbus/tx (the blufor-tracker bridge owns the radio).

Contract C4.4: at most one mesh text per ``MIN_INTERVAL_S`` (30 s) from this tool. Detections
that arrive in between are aggregated into the next text. C4.3: simulated texts start ``[SIM] ``.
Simulated (demo) and real detections never share a text: real ones go first, the simulated ones
wait for the next interval.

A detection is "new" the first time a device is seen as DRONE_CONFIRMED or DRONE_PROBABLE, when it
upgrades from probable to confirmed, or when it comes back after ``FORGET_S`` of silence. Periodic
CoT refreshes never trigger a mesh text. A text goes out ``SETTLE_S`` after the first frame,
so the serial and position decoded from the following frames are in it.

Env: ``MESH_ALERTS=1`` (default on; 0 disables) plus the usual ``MQTT_HOST/PORT/USER/PASS``.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Callable

from detector.models import Classification, DetectedDevice

logger = logging.getLogger(__name__)

TX_TOPIC = "meshbus/tx"
SOURCE = "drone-detector"
MAX_TEXT = 200
MIN_INTERVAL_S = 30.0
FORGET_S = 600.0
SETTLE_S = 5.0  # wait this long after the first frame so serial and position have arrived
SIM_PREFIX = "[SIM] "


def describe(device: DetectedDevice, op: tuple[float, float] | None = None) -> str:
    """Short one-detection summary for a mesh text, ending "hdg 225 closing" when known."""
    motion = device.motion(op).mesh_text()
    tail = f" {motion}" if motion else ""
    return _describe(device) + tail


def _describe(device: DetectedDevice) -> str:
    d = device.drone_id
    if device.classification == Classification.DRONE_CONFIRMED:
        name = (d.serial if d and d.serial else device.mac[-5:])[-8:]
        source = getattr(d, "source", "") or ""
        src = "DJI" if source.startswith("dji") else ("RID" if source.startswith("remoteid") else "ID")
        text = f"UAV {name} {src}"
        if d and d.has_position:
            text += f" {d.drone_lat:.5f},{d.drone_lon:.5f}"
            if d.drone_alt:
                text += f" {d.drone_alt:.0f}m"
        return text
    what = device.vendor or device.ssid or device.mac[-5:]
    dist = device.distance_estimate_m
    return f"probable UAV {what}" + (f" ~{dist:.0f}m" if dist else "")


def build_text(items: list[DetectedDevice], simulated: bool, op: tuple[float, float] | None = None) -> str:
    prefix = (SIM_PREFIX if simulated else "") + "DRONE: "
    confirmed = sum(1 for i in items if i.classification == Classification.DRONE_CONFIRMED)
    head = f"{len(items)} new: " if len(items) > 1 else ""
    text = prefix + head
    shown = 0
    for dev in sorted(items, key=lambda i: i.classification != Classification.DRONE_CONFIRMED):
        part = describe(dev, op)
        rest = len(items) - shown - 1
        tail = f"; +{rest} more" if rest else ""
        candidate = text + ("; " if shown else "") + part
        if len(candidate) + len(tail) > MAX_TEXT and shown:
            break
        text = candidate
        shown += 1
    if shown < len(items):
        text += f"; +{len(items) - shown} more ({confirmed} confirmed)"
    return text[:MAX_TEXT]


class MeshAlerter:
    """Collects new detections and sends at most one aggregated text per interval."""

    def __init__(self, publish: Callable[[str], None], simulated: bool = False,
                 interval: float = MIN_INTERVAL_S, clock: Callable[[], float] = time.time):
        self._publish = publish
        self.simulated = simulated
        self.interval = interval
        self._clock = clock
        self._lock = threading.Lock()
        self._seen: dict[str, tuple[Classification, float]] = {}
        self._pending: dict[str, tuple[DetectedDevice, float]] = {}
        self._last_sent: float | None = None
        self.sent = 0
        self._running = False
        self.op: tuple[float, float] | None = None  # OP position, for "hdg 225 closing"

    def note(self, device: DetectedDevice):
        if device.classification not in (Classification.DRONE_CONFIRMED, Classification.DRONE_PROBABLE):
            return
        now = self._clock()
        with self._lock:
            prev = self._seen.get(device.mac)
            is_new = (prev is None or now - prev[1] > FORGET_S
                      or (prev[0] == Classification.DRONE_PROBABLE
                          and device.classification == Classification.DRONE_CONFIRMED))
            self._seen[device.mac] = (device.classification, now)
            if is_new:
                self._pending[device.mac] = (device, now)

    def flush(self) -> str | None:
        """Send the pending detections if the rate limit allows. Returns the text sent."""
        now = self._clock()
        with self._lock:
            if not self._pending:
                return None
            if self._last_sent is not None and now - self._last_sent < self.interval:
                return None
            if now - min(t for _, t in self._pending.values()) < SETTLE_S:
                return None
            real = [m for m, (d, _) in self._pending.items() if not d.simulated]
            macs = real if (real and not self.simulated) else list(self._pending)
            items = [self._pending.pop(m)[0] for m in macs]
            self._last_sent = now
            if len(self._seen) > 2000:
                for mac in [m for m, (_, t) in self._seen.items() if now - t > FORGET_S]:
                    del self._seen[mac]
        text = build_text(items, self.simulated or all(d.simulated for d in items), self.op)
        try:
            self._publish(text)
            self.sent += 1
            logger.info("Mesh alert: %s", text)
        except Exception:
            logger.exception("Mesh alert publish failed")
        return text

    def forget(self, macs):
        """Drop pending texts for these devices (demo clear)."""
        with self._lock:
            for mac in macs:
                self._pending.pop(mac, None)

    def start(self) -> "MeshAlerter":
        self._running = True
        threading.Thread(target=self._loop, daemon=True, name="mesh-alerts").start()
        return self

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            self.flush()
            time.sleep(1.0)


def mqtt_publisher() -> Callable[[str], None] | None:
    """A paho client publishing to meshbus/tx; None if paho is missing."""
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        logger.error("paho-mqtt not installed; mesh alerts disabled")
        return None
    import socket

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                    client_id=f"drone-detector-mesh-{socket.gethostname().split('.')[0]}-{os.getpid()}",
                    protocol=mqtt.MQTTv311)
    c.username_pw_set(os.environ.get("MQTT_USER", "guest"), os.environ.get("MQTT_PASS", "guest"))
    c.reconnect_delay_set(min_delay=2, max_delay=30)
    c.connect_async(os.environ.get("MQTT_HOST", "127.0.0.1"), int(os.environ.get("MQTT_PORT", "1883")),
                    keepalive=30)
    c.loop_start()

    def publish(text: str):
        if not c.is_connected():
            raise ConnectionError("MQTT broker not connected; mesh alert dropped")
        c.publish(TX_TOPIC, json.dumps({"text": text, "to": None, "channel": 0, "source": SOURCE}), qos=1)

    return publish
