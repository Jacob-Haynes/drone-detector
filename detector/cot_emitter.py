"""Emit CoT XML to FreeTAKServer for ATAK integration."""

import logging
import socket
import threading
import time
import uuid

from detector.models import Classification, DetectedDevice

logger = logging.getLogger(__name__)

UID_PREFIX = "DRONE-"
STALE_SECONDS = 120


def _isotime(ts: float | None = None) -> str:
    from datetime import datetime, timezone

    dt = datetime.fromtimestamp(ts or time.time(), tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%SZ")


def _xml_escape(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")


class CotEmitter:
    """Sends drone detections to FreeTAKServer as hostile CoT markers."""

    def __init__(self, fts_host: str = "127.0.0.1", fts_port: int = 8087):
        self.fts_host = fts_host
        self.fts_port = fts_port
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()

    def emit(self, device: DetectedDevice):
        """Send a drone detection as a CoT event to FTS."""
        if device.classification not in (
            Classification.DRONE_CONFIRMED,
            Classification.DRONE_PROBABLE,
        ):
            return

        cot_xml = self._build_cot(device)
        if cot_xml:
            self._send(cot_xml)

    def _build_cot(self, device: DetectedDevice) -> str:
        now = time.time()
        drone_id = device.drone_id

        # Use drone GPS if available, otherwise 0,0 with remarks
        if drone_id and (drone_id.drone_lat != 0 or drone_id.drone_lon != 0):
            lat = drone_id.drone_lat
            lon = drone_id.drone_lon
            hae = drone_id.drone_alt
            cot_type = "a-h-A-M-F-Q"  # hostile, airborne, military, fixed-wing, UAV
        else:
            lat = 0.0
            lon = 0.0
            hae = 0.0
            cot_type = "a-h-A-M-F-Q"

        uid = f"{UID_PREFIX}{device.mac.replace(':', '')}"
        callsign = drone_id.serial if drone_id and drone_id.serial else f"Drone-{device.mac[-5:]}"

        remarks_parts = [
            f"RSSI: {device.rssi} dBm",
            f"Vendor: {device.vendor}" if device.vendor else None,
            f"Classification: {device.classification.value}",
        ]
        if device.distance_estimate_m:
            remarks_parts.append(f"Est. distance: {device.distance_estimate_m:.0f}m")
        if drone_id and drone_id.speed:
            remarks_parts.append(f"Speed: {drone_id.speed:.1f} m/s")
        remarks = ", ".join(r for r in remarks_parts if r)

        detail_xml = (
            f'<contact callsign="{_xml_escape(callsign)}"/>'
            f"<remarks>{_xml_escape(remarks)}</remarks>"
        )

        # Add track info if we have speed/heading
        if drone_id and (drone_id.speed or drone_id.heading):
            detail_xml += f'<track speed="{drone_id.speed:.1f}" course="{drone_id.heading:.1f}"/>'

        return (
            '<?xml version="1.0" encoding="UTF-8"?>'
            f'<event version="2.0" type="{cot_type}" uid="{uid}"'
            f' time="{_isotime(now)}" start="{_isotime(now)}"'
            f' stale="{_isotime(now + STALE_SECONDS)}" how="m-g">'
            f'<point lat="{lat}" lon="{lon}" hae="{hae}" ce="50" le="50"/>'
            f"<detail>{detail_xml}</detail>"
            f"</event>"
        )

    def _send(self, cot_xml: str):
        with self._lock:
            try:
                if self._sock is None:
                    self._connect()
                if self._sock is None:
                    return
                self._sock.sendall(cot_xml.encode("utf-8"))
            except (OSError, BrokenPipeError):
                logger.warning("FTS connection lost")
                self._close()

    def _connect(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((self.fts_host, self.fts_port))
            sock.settimeout(None)
            self._sock = sock
            logger.info("Connected to FTS at %s:%d", self.fts_host, self.fts_port)
        except OSError as e:
            logger.debug("FTS not available at %s:%d: %s", self.fts_host, self.fts_port, e)
            self._sock = None

    def _close(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None