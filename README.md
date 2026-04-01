# Drone Detector

WiFi-based drone detection for Raspberry Pi 5. Detects DJI drones (DroneID beacons), FAA/EU Remote ID broadcasts, and classifies devices by manufacturer OUI.

## Features

- DJI DroneID parsing (serial, GPS, operator location, speed, altitude)
- OpenDroneID / Remote ID (ASTM F3411) parsing
- OUI-based drone manufacturer detection (DJI, Parrot, Autel, Skydio)
- Real-time web dashboard on port 8080
- Optional CoT output to FreeTAKServer for ATAK integration
- Simulation mode for testing without hardware

## Hardware

- Raspberry Pi 5
- USB WiFi adapter with monitor mode support (e.g. Alfa AWUS036ACHM)

The Pi's built-in WiFi stays available for the BFT hotspot when running in combined mode.

## Install

```bash
sudo bash deploy/install.sh
```

## Usage

```bash
# Standalone
sudo pi-profiles drone-detector

# Combined with BFT
sudo pi-profiles bft-drone

# Manual (simulation mode)
source .venv/bin/activate
python -m detector --simulate
```

## Web UI

Browse to `http://<pi-ip>:8080` for the detection dashboard.

## ATAK Integration

When FreeTAKServer is running, drone detections appear as hostile UAV markers on the ATAK map. Disable with `--no-cot`.