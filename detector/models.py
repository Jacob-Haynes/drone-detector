"""Data models for drone detection."""

from dataclasses import dataclass, field
from enum import Enum
from time import time

from detector.motion import Motion, MotionTrack, rssi_motion


class Classification(Enum):
    DRONE_CONFIRMED = "DRONE_CONFIRMED"  # decoded DroneID / Remote ID broadcast
    DRONE_PROBABLE = "DRONE_PROBABLE"  # drone-maker access point with a drone-like SSID or no SSID
    DRONE_RELATED = "DRONE_RELATED"  # drone-maker device that is not the aircraft: controller, goggles, camera, phone app
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
    height_agl: float = 0.0
    operator_id: str = ""
    ua_type: str = ""
    description: str = ""
    source: str = ""  # dji-droneid | remoteid-wifi-beacon | remoteid-wifi-nan | remoteid-ble
    # Track over the ground as broadcast (Remote ID track direction; DJI velocity N/E). Unlike
    # ``heading`` (DJI yaw: where the nose points), this is the direction of travel.
    course: float = 0.0
    velocity_valid: bool = False  # speed (and course, if speed > 0.5 m/s) came in the broadcast

    @property
    def has_position(self) -> bool:
        return self.drone_lat != 0.0 or self.drone_lon != 0.0

    @property
    def has_operator(self) -> bool:
        return self.operator_lat != 0.0 or self.operator_lon != 0.0

    def merge(self, other: "DroneIDData") -> None:
        """Fill fields from a newer partial message (Remote ID sends BasicID and Location separately)."""
        for name, value in vars(other).items():
            if name in ("speed", "course", "velocity_valid"):
                continue
            if value not in ("", 0.0, 0):
                setattr(self, name, value)
        if other.velocity_valid:  # 0 m/s and 000° are real values here
            self.speed, self.course, self.velocity_valid = other.speed, other.course, True
        elif other.speed:
            self.speed = other.speed


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
    reason: str = ""  # why it got its classification (shown in UI and CoT remarks)
    transport: str = "wifi"  # wifi | ble
    track: MotionTrack = field(default_factory=MotionTrack, repr=False)  # position fixes for course/CPA

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
        # Log-distance model: ~20 dBm EIRP, ~40 dB free-space loss at 1 m (2.4 GHz),
        # path-loss exponent 2.5 (open ground). Very rough: +-2x is normal.
        tx_power = 20
        ref_loss_1m = 40
        n = 2.5
        return 10 ** ((tx_power - ref_loss_1m - self.rssi) / (10 * n))

    def motion(self, op: tuple[float, float] | None = None, now: float | None = None) -> Motion:
        """Course, speed and closing/opening relative to the OP (see detector.motion). From the
        position fixes when the drone broadcasts a position, else the RSSI trend (no course)."""
        if self.drone_id and self.drone_id.has_position and self.track.fixes:
            return self.track.summary(op)
        return rssi_motion(self.rssi_history, time() if now is None else now)

    @property
    def is_active(self) -> bool:
        return (time() - self.last_seen) < 60

    def to_dict(self, op: tuple[float, float] | None = None) -> dict:
        return {
            "motion": self.motion(op).to_dict(),
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
            "reason": self.reason,
            "transport": self.transport,
            "drone_id": {
                "serial": self.drone_id.serial,
                "drone_lat": self.drone_id.drone_lat,
                "drone_lon": self.drone_id.drone_lon,
                "drone_alt": self.drone_id.drone_alt,
                "speed": self.drone_id.speed,
                "heading": self.drone_id.heading,
                "operator_lat": self.drone_id.operator_lat,
                "operator_lon": self.drone_id.operator_lon,
                "height_agl": self.drone_id.height_agl,
                "operator_id": self.drone_id.operator_id,
                "ua_type": self.drone_id.ua_type,
                "source": self.drone_id.source,
                "course": self.drone_id.course if self.drone_id.velocity_valid else None,
            }
            if self.drone_id
            else None,
        }