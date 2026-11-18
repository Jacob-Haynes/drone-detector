"""Wi-Fi monitor-mode capture (scapy) and channel hopping.

A monitor interface only hears the channel it is tuned to, so ``ChannelHopper``
walks a channel list with ``iw``. The default list covers 2.4 GHz (1-13) and 5 GHz,
including the 5.8 GHz band (149-165) that consumer drones' Wi-Fi links use most. It
visits 1/6/11 and 149/153/157/161 (the Remote ID and common drone channels) more often.
"""

from __future__ import annotations

import logging
import subprocess
import threading
import time
from typing import Callable

logger = logging.getLogger(__name__)

# Live capture state, for status/heartbeat
CAPTURE_STATE = {"running": False, "error": ""}

CHANNELS_24 = list(range(1, 14))
CHANNELS_5 = [36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144]
CHANNELS_58 = [149, 153, 157, 161, 165]
PRIORITY = [6, 1, 149, 11, 153, 157, 161]


def default_channel_plan(bands: str = "2.4,5") -> list[int]:
    """Hop order: priority channels interleaved with a sweep of the rest."""
    wanted: list[int] = []
    if "2.4" in bands:
        wanted += CHANNELS_24
    if "5" in bands.replace("2.4", ""):
        wanted += CHANNELS_5 + CHANNELS_58
    prio = [c for c in PRIORITY if c in wanted]
    rest = [c for c in wanted if c not in prio]
    plan: list[int] = []
    for i, ch in enumerate(rest):
        plan.append(ch)
        if i % 2 == 1 and prio:  # a priority channel after every second sweep channel
            plan.append(prio[(i // 2) % len(prio)])
    plan += [c for c in prio if c not in plan]  # make sure every priority channel appears
    return plan


def parse_channels(spec: str) -> list[int]:
    """'1,6,11,149' -> [1, 6, 11, 149]"""
    return [int(c) for c in spec.replace(" ", "").split(",") if c]


class ChannelHopper:
    """Retunes ``interface`` through ``channels`` every ``dwell`` seconds using ``iw``."""

    def __init__(self, interface: str, channels: list[int], dwell: float = 0.3, runner=subprocess.run):
        self.interface = interface
        self.channels = channels
        self.dwell = dwell
        self.current: int | None = None
        self._runner = runner
        self._running = False
        self._unsupported: set[int] = set()

    def start(self):
        if len(self.channels) < 2:
            if self.channels:
                self._set(self.channels[0])
            return
        self._running = True
        threading.Thread(target=self._loop, daemon=True, name="chan-hop").start()
        logger.info("Channel hopping on %s: %d channels, %.0f ms dwell", self.interface, len(self.channels),
                    self.dwell * 1000)

    def stop(self):
        self._running = False

    def _set(self, ch: int) -> bool:
        res = self._runner(["iw", "dev", self.interface, "set", "channel", str(ch)],
                           capture_output=True, text=True)
        if res.returncode != 0:
            if ch not in self._unsupported:
                self._unsupported.add(ch)
                logger.info("Channel %d not supported by %s (%s) — skipping it", ch, self.interface,
                            (res.stderr or "").strip()[:80])
            return False
        self.current = ch
        return True

    def _loop(self):
        while self._running:
            usable = [c for c in self.channels if c not in self._unsupported]
            if not usable:
                logger.error("No usable channels on %s; staying put", self.interface)
                return
            for ch in usable:
                if not self._running:
                    return
                if self._set(ch):
                    time.sleep(self.dwell)


def get_rssi(packet) -> int:
    """Extract RSSI from RadioTap header."""
    from scapy.all import RadioTap

    if packet.haslayer(RadioTap):
        value = getattr(packet[RadioTap], "dBm_AntSignal", None)
        if value is not None:
            return int(value)
    return -100


def get_ssid(packet) -> str:
    """Extract SSID from Dot11Elt layers."""
    from scapy.all import Dot11Elt

    elt = packet.getlayer(Dot11Elt)
    while elt:
        if elt.ID == 0 and elt.info:  # SSID element
            return elt.info.decode("utf-8", errors="replace")
        elt = elt.payload.getlayer(Dot11Elt)
    return ""


def get_vendor_ies(packet) -> list[tuple[bytes, bytes]]:
    """Extract vendor-specific information elements (ID=221) as (oui, payload) tuples."""
    from scapy.all import Dot11Elt

    vendor_ies = []
    elt = packet.getlayer(Dot11Elt)
    while elt:
        if elt.ID == 221 and len(elt.info) >= 3:
            vendor_ies.append((bytes(elt.info[:3]), bytes(elt.info[3:])))
        elt = elt.payload.getlayer(Dot11Elt)
    return vendor_ies


def packet_to_frame(pkt) -> dict | None:
    """Turn a captured 802.11 packet into the engine's frame dict, or None if irrelevant."""
    from scapy.all import Dot11, Dot11Beacon, Dot11ProbeReq

    from detector.parsers import remote_id

    if not pkt.haslayer(Dot11):
        return None
    dot11 = pkt[Dot11]
    mac = dot11.addr2
    if not mac or dot11.type != 0:  # management frames only
        return None

    drone_data = None
    if pkt.haslayer(Dot11Beacon):
        frame_type = "beacon"
    elif pkt.haslayer(Dot11ProbeReq):
        frame_type = "probe_req"
    elif dot11.subtype == 5:
        frame_type = "probe_resp"
    elif dot11.subtype == 13 and (dot11.addr1 or "").lower() == remote_id.NAN_DEST:
        # Wi-Fi NAN action frame: Remote ID service discovery
        body = bytes(dot11.payload)
        drone_data = remote_id.parse_nan_action(body)
        if drone_data is None:
            return None
        frame_type = "nan"
    else:
        return None

    return {
        "mac": mac.upper(),
        "rssi": get_rssi(pkt),
        "ssid": get_ssid(pkt) if frame_type != "nan" else "",
        "frame_type": frame_type,
        "vendor_ies": get_vendor_ies(pkt) if frame_type != "nan" else [],
        "drone_data": drone_data,
        "transport": "wifi",
    }


def start_capture(interface: str, callback: Callable[[dict], None]):
    """Capture Wi-Fi frames in monitor mode (blocking). Retries if the interface goes away."""
    from scapy.all import sniff

    def _handle_packet(pkt):
        try:
            frame = packet_to_frame(pkt)
        except Exception:
            logger.debug("Failed to decode a frame", exc_info=True)
            return
        if frame:
            callback(frame)

    while True:
        logger.info("Starting capture on %s", interface)
        CAPTURE_STATE.update(running=True, error="")
        try:
            sniff(iface=interface, prn=_handle_packet, store=False)
            CAPTURE_STATE.update(running=False, error="capture stopped")
        except Exception as e:
            CAPTURE_STATE.update(running=False, error=str(e)[:120])
            logger.error("Capture on %s failed (%s) — retrying in 10s", interface, e)
        time.sleep(10)
