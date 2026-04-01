"""OUI (Organizationally Unique Identifier) lookup for MAC address classification."""

import logging
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


def is_drone_oui(mac: str) -> bool:
    """Check if MAC belongs to a known drone manufacturer."""
    return mac.upper()[:8] in DRONE_OUIS