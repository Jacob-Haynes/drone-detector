"""Detection engine: classify, track, and alert on WiFi devices."""

import logging
import threading
from time import time
from typing import Callable

from detector.models import Classification, DetectedDevice
from detector.parsers import dji_droneid, oui, remote_id

logger = logging.getLogger(__name__)

STALE_TIMEOUT = 60  # seconds


class DetectionEngine:
    """Processes raw WiFi frames, maintains device tracking, and emits alerts."""

    def __init__(self, on_alert: Callable[[DetectedDevice], None] | None = None):
        self._devices: dict[str, DetectedDevice] = {}
        self._lock = threading.Lock()
        self._on_alert = on_alert

    @property
    def devices(self) -> dict[str, DetectedDevice]:
        with self._lock:
            return dict(self._devices)

    def process_frame(self, frame: dict):
        """Process a parsed WiFi frame from the scanner."""
        mac = frame["mac"]
        rssi = frame["rssi"]
        ssid = frame.get("ssid", "")
        vendor_ies = frame.get("vendor_ies", [])

        with self._lock:
            device = self._devices.get(mac)
            if device is None:
                vendor = oui.lookup(mac)
                device = DetectedDevice(mac=mac, vendor=vendor, ssid=ssid)

                if oui.is_drone_oui(mac):
                    device.classification = Classification.DRONE_PROBABLE
                    logger.warning("DRONE_PROBABLE: %s (%s) RSSI=%d", mac, vendor, rssi)
                else:
                    device.classification = Classification.UNKNOWN_NEW

                self._devices[mac] = device

            device.update_rssi(rssi)
            if ssid and not device.ssid:
                device.ssid = ssid

            # Parse vendor-specific IEs for DroneID / Remote ID
            drone_data = None
            for vie_oui, payload in vendor_ies:
                drone_data = dji_droneid.parse_vendor_ie(vie_oui, payload)
                if drone_data:
                    break
                drone_data = remote_id.parse_vendor_ie(vie_oui, payload)
                if drone_data:
                    break

            if drone_data:
                device.drone_id = drone_data
                if device.classification != Classification.DRONE_CONFIRMED:
                    device.classification = Classification.DRONE_CONFIRMED
                    logger.warning(
                        "DRONE_CONFIRMED: %s serial=%s lat=%.6f lon=%.6f alt=%.0f",
                        mac,
                        drone_data.serial,
                        drone_data.drone_lat,
                        drone_data.drone_lon,
                        drone_data.drone_alt,
                    )

            # After initial UNKNOWN_NEW period, downgrade to DEVICE if no drone indicators
            if (
                device.classification == Classification.UNKNOWN_NEW
                and device.frame_count > 10
                and not oui.is_drone_oui(mac)
            ):
                device.classification = Classification.DEVICE

        # Fire alert callback outside lock
        if drone_data or (
            device.classification in (Classification.DRONE_CONFIRMED, Classification.DRONE_PROBABLE)
            and device.frame_count == 1
        ):
            if self._on_alert:
                self._on_alert(device)

    def get_active_devices(self) -> list[dict]:
        """Return active devices as dicts, sorted by classification priority."""
        priority = {
            Classification.DRONE_CONFIRMED: 0,
            Classification.DRONE_PROBABLE: 1,
            Classification.UNKNOWN_NEW: 2,
            Classification.DEVICE: 3,
        }
        with self._lock:
            active = [d.to_dict() for d in self._devices.values() if d.is_active]
        active.sort(key=lambda d: (priority.get(Classification(d["classification"]), 99), d["last_seen"] * -1))
        return active

    def get_alerts(self) -> list[dict]:
        """Return confirmed and probable drone detections."""
        with self._lock:
            return [
                d.to_dict()
                for d in self._devices.values()
                if d.is_active
                and d.classification
                in (Classification.DRONE_CONFIRMED, Classification.DRONE_PROBABLE)
            ]

    def cleanup(self):
        """Remove devices not seen for a long time."""
        cutoff = time() - STALE_TIMEOUT * 5
        with self._lock:
            stale = [mac for mac, d in self._devices.items() if d.last_seen < cutoff]
            for mac in stale:
                del self._devices[mac]