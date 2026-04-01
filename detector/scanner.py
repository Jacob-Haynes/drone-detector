"""WiFi monitor mode frame capture using scapy."""

import logging
from typing import Callable

from scapy.all import Dot11, Dot11Beacon, Dot11Elt, Dot11ProbeReq, RadioTap, sniff

logger = logging.getLogger(__name__)


def get_rssi(packet) -> int:
    """Extract RSSI from RadioTap header."""
    if packet.haslayer(RadioTap):
        return getattr(packet[RadioTap], "dBm_AntSignal", -100)
    return -100


def get_ssid(packet) -> str:
    """Extract SSID from Dot11Elt layers."""
    elt = packet.getlayer(Dot11Elt)
    while elt:
        if elt.ID == 0 and elt.info:  # SSID element
            try:
                return elt.info.decode("utf-8", errors="replace")
            except Exception:
                pass
        elt = elt.payload.getlayer(Dot11Elt)
    return ""


def get_vendor_ies(packet) -> list[tuple[bytes, bytes]]:
    """Extract vendor-specific information elements (ID=221).

    Returns list of (oui, payload) tuples.
    """
    vendor_ies = []
    elt = packet.getlayer(Dot11Elt)
    while elt:
        if elt.ID == 221 and len(elt.info) >= 3:  # Vendor-specific
            oui = elt.info[:3]
            payload = elt.info[3:]
            vendor_ies.append((oui, payload))
        elt = elt.payload.getlayer(Dot11Elt)
    return vendor_ies


def start_capture(interface: str, callback: Callable[[dict], None]):
    """Start capturing WiFi frames in monitor mode.

    Calls callback with a dict for each relevant frame:
    {
        "mac": str,
        "rssi": int,
        "ssid": str,
        "frame_type": "beacon" | "probe_req" | "probe_resp",
        "vendor_ies": [(oui, payload), ...],
    }
    """
    logger.info("Starting capture on %s", interface)

    def _handle_packet(pkt):
        if not pkt.haslayer(Dot11):
            return

        dot11 = pkt[Dot11]
        mac = dot11.addr2
        if not mac:
            return

        frame_type = None
        if pkt.haslayer(Dot11Beacon):
            frame_type = "beacon"
        elif pkt.haslayer(Dot11ProbeReq):
            frame_type = "probe_req"
        elif dot11.type == 0 and dot11.subtype == 5:  # Probe response
            frame_type = "probe_resp"
        else:
            return

        callback(
            {
                "mac": mac.upper(),
                "rssi": get_rssi(pkt),
                "ssid": get_ssid(pkt),
                "frame_type": frame_type,
                "vendor_ies": get_vendor_ies(pkt),
            }
        )

    sniff(iface=interface, prn=_handle_packet, store=False)