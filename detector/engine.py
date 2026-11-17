"""Detection engine: classify, track, and alert on Wi-Fi and Bluetooth devices."""

from __future__ import annotations

import logging
import threading
from time import time
from typing import Callable

from detector.models import Classification, DetectedDevice, DroneIDData
from detector.parsers import dji_droneid, oui, remote_id

logger = logging.getLogger(__name__)

STALE_TIMEOUT = 60  # seconds a device stays "active" after its last frame
CLEANUP_INTERVAL = 30
UNKNOWN_PERIOD_FRAMES = 10

# How often an alerting device is re-sent to TAK (CoT stale is 5 min). Contract C4.4: at most one
# CoT update per 5 s per track; the CoT emitter enforces that too.
ALERT_INTERVAL = {
    Classification.DRONE_CONFIRMED: 5.0,
    Classification.DRONE_PROBABLE: 60.0,
}
NEW_DATA_MIN_S = 5.0

_RANK = {
    Classification.DRONE_CONFIRMED: 0,
    Classification.DRONE_PROBABLE: 1,
    Classification.DRONE_RELATED: 2,
    Classification.UNKNOWN_NEW: 3,
    Classification.DEVICE: 4,
}


def _better(a: Classification, b: Classification) -> bool:
    return _RANK[a] < _RANK[b]


class DetectionEngine:
    """Processes raw frames, maintains device tracking, and emits alerts.

    ``on_alert(device)`` is called (outside the lock) when a device becomes
    DRONE_CONFIRMED or DRONE_PROBABLE, when its DroneID/Remote ID data updates,
    and then periodically while it is still being heard (see ``ALERT_INTERVAL``).
    DRONE_RELATED devices (controllers, goggles, cameras, operator phones) are shown
    in the UI but never alerted as UAVs.
    """

    def __init__(self, on_alert: Callable[[DetectedDevice], None] | None = None):
        self._devices: dict[str, DetectedDevice] = {}
        self._lock = threading.Lock()
        self._on_alert = on_alert
        self._last_alert: dict[str, float] = {}
        self._last_cleanup = time()
        self.op: tuple[float, float] | None = None  # OP position (SENSOR_LAT/LON) for motion

    @property
    def devices(self) -> dict[str, DetectedDevice]:
        with self._lock:
            return dict(self._devices)

    @staticmethod
    def _decode_ies(vendor_ies) -> DroneIDData | None:
        for vie_oui, payload in vendor_ies:
            data = dji_droneid.parse_vendor_ie(vie_oui, payload) or remote_id.parse_vendor_ie(vie_oui, payload)
            if data:
                return data
        return None

    def process_frame(self, frame: dict):
        """Process one observation.

        ``frame`` keys: mac, rssi, ssid, frame_type (beacon|probe_req|probe_resp|nan|ble),
        vendor_ies [(oui, payload)], and optionally drone_data (pre-decoded, e.g. BLE/NAN)
        and transport (wifi|ble).
        """
        mac = frame["mac"]
        rssi = frame["rssi"]
        ssid = frame.get("ssid", "")
        frame_type = frame.get("frame_type", "")
        drone_data = frame.get("drone_data") or self._decode_ies(frame.get("vendor_ies", []))
        now = time()

        with self._lock:
            device = self._devices.get(mac)
            if device is None:
                device = DetectedDevice(mac=mac, vendor=oui.lookup(mac), ssid=ssid,
                                        classification=Classification.UNKNOWN_NEW,
                                        transport=frame.get("transport", "wifi"))
                self._devices[mac] = device
            previous = device.classification

            device.update_rssi(rssi)
            if ssid and not device.ssid:
                device.ssid = ssid

            if drone_data:
                if device.drone_id is None:
                    device.drone_id = drone_data
                else:
                    device.drone_id.merge(drone_data)
                device.classification = Classification.DRONE_CONFIRMED
                device.reason = f"decoded {drone_data.source or 'drone ID'} broadcast"
                if drone_data.has_position:  # a fix for course/speed/CPA (detector.motion)
                    bc = drone_data.velocity_valid
                    device.track.add_fix(now, drone_data.drone_lat, drone_data.drone_lon,
                                         course=drone_data.course if bc else None,
                                         speed=drone_data.speed if bc else None)
            elif device.classification != Classification.DRONE_CONFIRMED:
                verdict = oui.assess(mac, ssid or device.ssid, frame_type)
                if verdict:
                    cls = Classification[verdict[0]]
                    if _better(cls, device.classification) or device.classification == Classification.DEVICE:
                        device.classification, device.reason = cls, verdict[1]
                elif (device.classification == Classification.UNKNOWN_NEW
                      and device.frame_count > UNKNOWN_PERIOD_FRAMES):
                    device.classification = Classification.DEVICE

            alert = self._should_alert(device, previous, bool(drone_data), now)
            if device.classification != previous and device.classification in (
                    Classification.DRONE_CONFIRMED, Classification.DRONE_PROBABLE, Classification.DRONE_RELATED):
                log = logger.warning if device.classification != Classification.DRONE_RELATED else logger.info
                d = device.drone_id
                log("%s: %s RSSI=%d %s%s", device.classification.value, mac, rssi, device.reason,
                    f" serial={d.serial} lat={d.drone_lat:.6f} lon={d.drone_lon:.6f}" if d else "")

            if now - self._last_cleanup > CLEANUP_INTERVAL:
                self._cleanup_locked(now)

        if alert and self._on_alert:
            try:
                self._on_alert(device)
            except Exception:
                logger.exception("Alert callback failed")

    def _should_alert(self, device: DetectedDevice, previous: Classification, new_data: bool, now: float) -> bool:
        interval = ALERT_INTERVAL.get(device.classification)
        if interval is None:
            return False
        last = self._last_alert.get(device.mac)
        upgraded = device.classification != previous
        if upgraded or last is None or now - last >= interval or (new_data and now - last >= NEW_DATA_MIN_S):
            self._last_alert[device.mac] = now
            return True
        return False

    def get_active_devices(self) -> list[dict]:
        """Return active devices as dicts, sorted by classification priority."""
        with self._lock:
            active = [d.to_dict(self.op) for d in self._devices.values() if d.is_active]
        active.sort(key=lambda d: (_RANK.get(Classification(d["classification"]), 99), -d["last_seen"]))
        return active

    def get_alerts(self) -> list[dict]:
        """Return confirmed and probable drone detections."""
        with self._lock:
            return [
                d.to_dict(self.op)
                for d in self._devices.values()
                if d.is_active
                and d.classification in (Classification.DRONE_CONFIRMED, Classification.DRONE_PROBABLE)
            ]

    def cleanup(self):
        """Remove devices not seen for a long time."""
        with self._lock:
            self._cleanup_locked(time())

    def _cleanup_locked(self, now: float):
        self._last_cleanup = now
        cutoff = now - STALE_TIMEOUT * 5
        for mac in [m for m, d in self._devices.items() if d.last_seen < cutoff]:
            del self._devices[mac]
            self._last_alert.pop(mac, None)
