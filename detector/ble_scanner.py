"""Bluetooth LE Remote ID scanning with bleak (BlueZ on the Pi's onboard Bluetooth).

Most Remote ID modules and many drones (DJI Mini 3/4, Avata, Air 3 in FAA/EU
mode) broadcast ASTM F3411 messages as BLE advertisements: service data for UUID
0xFFFA. The hotspot only uses the Pi's Wi-Fi, so the onboard Bluetooth radio is
free for this. Scanning is passive-friendly and needs no pairing.

Range is typically 100-300 m with the Pi's chip antenna, more with BT5 Coded PHY.
That depends on the drone and the adapter.
"""

from __future__ import annotations

import asyncio
import logging
import threading
from typing import Callable

from detector.parsers import remote_id

logger = logging.getLogger(__name__)


def advertisement_to_frame(address: str, rssi: int | None, service_data: dict) -> dict | None:
    """Turn one BLE advertisement into an engine frame, or None if it is not Remote ID."""
    payload = None
    for uuid, data in (service_data or {}).items():
        if str(uuid).lower() == remote_id.BLE_SERVICE_UUID:
            payload = bytes(data)
            break
    if payload is None:
        return None
    drone_data = remote_id.parse_ble_service_data(payload)
    if drone_data is None:
        return None
    return {
        "mac": f"BLE:{address.upper()}",
        "rssi": int(rssi if rssi is not None else -100),
        "ssid": "",
        "frame_type": "ble",
        "vendor_ies": [],
        "drone_data": drone_data,
        "transport": "ble",
    }


class BleRemoteIdScanner:
    """Runs a bleak scanner on its own asyncio loop/thread and feeds the engine."""

    RETRY_S = 15

    def __init__(self, callback: Callable[[dict], None], adapter: str | None = None):
        self._callback = callback
        self._adapter = adapter
        self._running = False
        self.active = False

    def start(self):
        self._running = True
        threading.Thread(target=lambda: asyncio.run(self._run()), daemon=True, name="ble-rid").start()

    def stop(self):
        self._running = False

    def _on_adv(self, device, adv):
        frame = advertisement_to_frame(device.address, adv.rssi, adv.service_data)
        if frame:
            self._callback(frame)

    async def _run(self):
        try:
            from bleak import BleakScanner
        except ImportError:
            logger.warning("bleak not installed — BLE Remote ID scanning disabled")
            return

        while self._running:
            try:
                kwargs = {"detection_callback": self._on_adv}
                if self._adapter:
                    kwargs["adapter"] = self._adapter
                async with BleakScanner(**kwargs):
                    self.active = True
                    logger.info("BLE Remote ID scanning on %s", self._adapter or "default adapter")
                    while self._running:
                        await asyncio.sleep(1)
            except Exception as e:
                self.active = False
                logger.warning("BLE scanner unavailable (%s: %s) — retrying in %ds",
                               e.__class__.__name__, e, self.RETRY_S)
                await asyncio.sleep(self.RETRY_S)
        self.active = False
