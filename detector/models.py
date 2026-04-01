"""Data models for drone detection."""

from dataclasses import dataclass, field
from enum import Enum
from time import time


class Classification(Enum):
    DRONE_CONFIRMED = "DRONE_CONFIRMED"
    DRONE_PROBABLE = "DRONE_PROBABLE"
    UNKNOWN_NEW = "UNKNOWN_NEW"
    DEVICE = "DEVICE"


@dataclass
class DroneIDData:
    """Parsed DJI DroneID or Remote ID data."""

    serial: str = ""
    drone_lat: float = 0.0
    drone_lon: float = 0.0
    drone_alt: float = 0.0
    speed: float = 0.0
    heading: float = 0.0
    operator_lat: float = 0.0
    operator_lon: float = 0.0
    home_lat: float = 0.0
    home_lon: float = 0.0


@dataclass
class DetectedDevice:
    """A device seen on WiFi."""

    mac: str
    vendor: str = ""
    ssid: str = ""
    classification: Classification = Classification.DEVICE
    rssi: int = -100
    rssi_history: list[tuple[float, int]] = field(default_factory=list)
    first_seen: float = field(default_factory=time)
    last_seen: float = field(default_factory=time)
    drone_id: DroneIDData | None = None
    frame_count: int = 0

    def update_rssi(self, rssi: int):
        now = time()
        self.rssi = rssi
        self.last_seen = now
        self.frame_count += 1
        self.rssi_history.append((now, rssi))
        # Keep last 60 seconds of history
        cutoff = now - 60
        self.rssi_history = [(t, r) for t, r in self.rssi_history if t > cutoff]

    @property
    def distance_estimate_m(self) -> float | None:
        """Rough distance estimate from RSSI using log-distance path loss model."""
        if self.rssi >= 0:
            return None
        # Assume tx_power=20dBm, path_loss_exponent=2.5 (open air)
        tx_power = 20
        n = 2.5
        return 10 ** ((tx_power - self.rssi) / (10 * n))

    @property
    def is_active(self) -> bool:
        return (time() - self.last_seen) < 60

    def to_dict(self) -> dict:
        return {
            "mac": self.mac,
            "vendor": self.vendor,
            "ssid": self.ssid,
            "classification": self.classification.value,
            "rssi": self.rssi,
            "distance_m": round(self.distance_estimate_m, 1) if self.distance_estimate_m else None,
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
            "frame_count": self.frame_count,
            "active": self.is_active,
            "drone_id": {
                "serial": self.drone_id.serial,
                "drone_lat": self.drone_id.drone_lat,
                "drone_lon": self.drone_id.drone_lon,
                "drone_alt": self.drone_id.drone_alt,
                "speed": self.drone_id.speed,
                "heading": self.drone_id.heading,
                "operator_lat": self.drone_id.operator_lat,
                "operator_lon": self.drone_id.operator_lon,
            }
            if self.drone_id
            else None,
        }