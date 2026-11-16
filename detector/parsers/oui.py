"""OUI (Organizationally Unique Identifier) lookup and drone-likeness rules.

A drone maker's OUI alone does not mean "drone": DJI also makes remote
controllers, goggles, Osmo/Pocket/Action cameras and mics, and phones running a
drone app probe for the aircraft's network. ``assess()`` combines OUI, SSID and
frame type into a conservative classification. Only decoded DroneID / Remote ID
(see the engine) makes a device DRONE_CONFIRMED.
"""

import logging
import re
from pathlib import Path

logger = logging.getLogger(__name__)

# Known drone manufacturer OUI prefixes (first 3 bytes of MAC)
DRONE_OUIS: dict[str, str] = {
    "60:60:1F": "DJI",
    "34:D2:62": "DJI",
    "48:1C:B9": "DJI",
    "D8:2D:CB": "DJI",
    "C8:6C:1E": "DJI",
    "A0:14:1A": "DJI",
    "90:3A:E6": "Parrot",
    "A0:14:3D": "Parrot",
    "00:12:1C": "Parrot",
    "00:26:7E": "Parrot",
    "90:03:B7": "Parrot",
    "A4:CF:12": "Autel Robotics",
    "74:4D:28": "Autel Robotics",
    "20:A6:0C": "Skydio",
    "84:D8:1B": "Skydio",
}

# Broader OUI database loaded from file
_oui_db: dict[str, str] = {}
_oui_loaded = False


def _load_oui_db():
    """Load IEEE OUI database from oui.txt if available."""
    global _oui_db, _oui_loaded
    if _oui_loaded:
        return

    oui_path = Path(__file__).parent.parent.parent / "oui.txt"
    if oui_path.exists():
        try:
            with open(oui_path) as f:
                for line in f:
                    line = line.strip()
                    if "(hex)" in line:
                        parts = line.split("(hex)")
                        if len(parts) == 2:
                            prefix = parts[0].strip().replace("-", ":")
                            vendor = parts[1].strip()
                            _oui_db[prefix.upper()] = vendor
            logger.info("Loaded %d OUI entries", len(_oui_db))
        except Exception:
            logger.warning("Failed to load OUI database from %s", oui_path)

    _oui_loaded = True


def lookup(mac: str) -> str:
    """Return vendor name for a MAC address, or empty string if unknown."""
    prefix = mac.upper()[:8]  # "AA:BB:CC"

    # Check drone OUIs first
    if prefix in DRONE_OUIS:
        return DRONE_OUIS[prefix]

    # Check full OUI database
    _load_oui_db()
    if prefix in _oui_db:
        return _oui_db[prefix]

    return ""


def is_locally_administered(mac: str) -> bool:
    """Randomised/private MACs have bit 1 of the first octet set; their OUI means nothing."""
    try:
        return bool(int(mac[:2], 16) & 0x02)
    except ValueError:
        return False


def is_drone_oui(mac: str) -> bool:
    """Check if MAC belongs to a known drone manufacturer."""
    return not is_locally_administered(mac) and mac.upper()[:8] in DRONE_OUIS


# SSIDs broadcast by Wi-Fi-link aircraft (the aircraft is the access point)
DRONE_SSID_RE = re.compile(
    r"^(dji[-_ ]?(mavic|spark|mini|air|phantom|inspire|tello|fpv|avata|neo|flip)"
    r"|mavic|spark[-_]|phantom|tello[-_]|rmtt[-_]|anafi|bebop|disco[-_]|parrot|skycontroller"
    r"|autel|evo[-_ ]?(ii|2|lite|nano)|skydio|holy ?stone|hs\d{3}|potensic|hubsan|sjrc|eachine|fimi"
    r"|snaptain|ruko|bwine|syma|wifi[-_]?uav|wifi[-_]?fpv|drone[-_ ]|.*[-_ ]drone\b|uav[-_ ])",
    re.IGNORECASE,
)

# Drone-maker devices that are NOT aircraft
ACCESSORY_SSID_RE = re.compile(
    r"(osmo|pocket|ronin|goggles|dji[-_ ]?(rc|mic|action|om|power|cellular)|^rc[-_ ]?(n\d|pro|plus)?[-_]"
    r"|^om[-_ ]?\d|action ?\d|^mic[-_ ]?\d|smart ?controller)",
    re.IGNORECASE,
)

AP_FRAMES = ("beacon", "probe_resp")


def assess(mac: str, ssid: str, frame_type: str):
    """Return ``(classification_name, reason)`` for drone indicators, or None if there are none.

    classification_name is "DRONE_PROBABLE" or "DRONE_RELATED".
    """
    vendor = DRONE_OUIS.get(mac.upper()[:8]) if is_drone_oui(mac) else None
    is_ap = frame_type in AP_FRAMES
    ssid = ssid or ""

    if ssid and ACCESSORY_SSID_RE.search(ssid) and (vendor or ssid.lower().startswith("dji")):
        return "DRONE_RELATED", f"{vendor or 'DJI'} accessory (SSID {ssid!r})"
    if ssid and DRONE_SSID_RE.match(ssid):
        if is_ap:
            return "DRONE_PROBABLE", f"access point with drone SSID {ssid!r}"
        # A phone/tablet probing for a drone's network: likely the operator
        return "DRONE_RELATED", f"device probing for drone network {ssid!r} (possible operator)"
    if vendor and is_ap:
        why = f"{vendor} access point" + (f" (SSID {ssid!r})" if ssid else " (hidden SSID)")
        return "DRONE_PROBABLE", why
    if vendor:
        return "DRONE_RELATED", f"{vendor} client device (controller, goggles or app)"
    return None