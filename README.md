# Drone Detector

Passive drone detection for Raspberry Pi 5 using Wi-Fi (a USB adapter in monitor mode) and Bluetooth LE (the Pi's onboard Bluetooth). It decodes DJI DroneID and ASTM F3411 / EU Remote ID broadcasts. It flags drone-maker devices by OUI and SSID without calling every DJI gadget a drone. Detections go to ATAK as CoT through OpenTAKServer.

## What It Can and Can't See

| Source | How | Gives |
|---|---|---|
| **Remote ID, Wi-Fi Beacon** | vendor IE `FA:0B:BC` type `0x0D` | serial, UA type, position, altitude, speed, heading, operator position and ID |
| **Remote ID, Wi-Fi NAN** | NAN action frames to `51:6F:9A:01:00:00` | same |
| **Remote ID, Bluetooth LE** | service data UUID `0xFFFA` (BT4 legacy single messages, BT5 message packs) | same |
| **DJI DroneID (Wi-Fi)** | beacon vendor IE `26:37:12` | serial, position, altitude, speed, heading, home/pilot position |
| **OUI + SSID** | beacons and probe requests | "probable drone" or "drone-related device", no position |

The limits matter when you brief what the kit can do:
- **DJI OcuSync/O2/O3/O4 aircraft** (most current DJI drones) send DroneID on their own radio link. A Wi-Fi card cannot decode that; it needs an SDR. These aircraft *are* seen when they broadcast standard Remote ID (Wi-Fi or Bluetooth), which is mandatory in the US and EU.
- **Analogue or digital FPV quads** without Remote ID and with no Wi-Fi are invisible to this tool. rf-spectrum is the tool for those.
- Distances from RSSI are rough (±2× is normal). Remote ID/DroneID positions are what the drone reports.

## Classification

| Class | Meaning | Sent to ATAK |
|---|---|---|
| `DRONE_CONFIRMED` | A DroneID / Remote ID broadcast was decoded | Yes: hostile UAV (`a-h-A-M-F-Q`) at its reported position, plus an operator marker when known. `COT_AFFILIATION=u` makes it unknown instead |
| `DRONE_PROBABLE` | A drone maker's access point (DJI/Parrot/Autel/Skydio OUI), or an AP with a drone-style SSID (`TELLO-`, `Anafi-`, `DJI-Mavic…`) | Yes: unknown UAV (`a-u-A-M-F-Q`) at the sensor position (`SENSOR_LAT/LON`) with RSSI distance as the error circle; not sent if no sensor position is set |
| `DRONE_RELATED` | A drone maker's client device (controllers, goggles), DJI cameras/mics (Osmo, Pocket, Action, RC), or a phone probing for a drone's network (likely the operator) | No, UI only |
| `UNKNOWN_NEW` → `DEVICE` | Everything else | No |

A drone OUI alone never confirms a drone. Randomised (locally administered) MACs are never matched against OUIs.

## Hardware

- Raspberry Pi 5
- USB Wi-Fi adapter with monitor mode on 2.4 and 5 GHz (e.g. Alfa AWUS036ACHM / ACM). The Pi's onboard Wi-Fi (`wlan0`) stays on the ATAK hotspot and is never touched.
- The Pi's onboard Bluetooth for BLE Remote ID. The hotspot does not use it.

## Install

```bash
sudo bash deploy/install.sh
```

The install is idempotent and does not enable the service at boot; pi-profiles starts it. Code goes to `/opt/drone-detector` and config to `/etc/drone-detector.env` (see `deploy/drone-detector.env.example`).

**Port 8080:** OpenTAKServer's nginx also listens on 8080 (Marti HTTP API). When OTS is installed on the same Pi, set `WEB_PORT` in `/etc/drone-detector.env` to a free port, for example 8150, and the install warns about this. The kit's port plan is owned by pi-profiles.

## How Capture Works

`deploy/start.sh` (run by the service as root):
1. Picks the capture adapter: `WIFI_IFACE`, or the first wireless interface that is not `HOTSPOT_IFACE` (default `wlan0`). This works for `wlan1` and `wlx…` names.
2. Tells NetworkManager to leave it alone (`nmcli device set … managed no`), switches it to monitor mode and checks that it took.
3. Starts the detector. It hops channels: all of 2.4 GHz plus 5 GHz including the 5.8 GHz band (149–165), 300 ms per channel. Channels 1/6/11 and 149–161 are visited more often. Channels the adapter refuses (e.g. DFS) are skipped. Override with `WIFI_CHANNELS=1,6,11,149,153,157,161` or `WIFI_BANDS=2.4`.

With no adapter, or one without monitor mode, it runs **Bluetooth Remote ID only** and says so in the log. It never falls back to fake data. `SIMULATE=1` in the env file does that explicitly, and every simulated CoT callsign is prefixed `SIM-`.

## Usage

```bash
sudo pi-profiles drone-detector

# Manual
source .venv/bin/activate
python -m detector --simulate                            # no hardware
sudo .venv/bin/python -m detector -i wlan1 --channels 6,149
python -m detector --no-wifi                              # BLE Remote ID only
```

## Web UI

Browse to `http://<pi-ip>:<WEB_PORT>` (default 8080) for the dashboard. `GET /api/status` shows the current channel, BLE state and TAK connection.

## ATAK Integration

CoT goes over plain TCP to `TAK_HOST:TAK_PORT` (default `127.0.0.1:8087`, OpenTAKServer as set up by blufor-tracker). The sender runs in the background and reconnects with backoff. The detector starts and keeps detecting when no TAK server is up. Set `SENSOR_LAT`/`SENSOR_LON` so OUI/SSID-only detections can be placed near the sensor. Disable CoT with `--no-cot`.

## Tests

```bash
pip install -r requirements.txt pytest httpx
python -m pytest
```

The tests build real-format 802.11 frames with scapy. They cover DJI DroneID v1/v2, Remote ID over beacon, NAN and BLE with hand-checked byte vectors, the classification rules, CoT output and the channel hopper.
