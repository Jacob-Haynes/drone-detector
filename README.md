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

### DJI O3 / O4 FPV builds (like the demo quad)

Be clear about this in a brief: **a DJI O3 or O4 Air Unit on its own broadcasts no Remote ID**, so this detector cannot see it.
- The air units have no GPS, no Bluetooth and no Wi-Fi radio of their own, so there is nothing to carry broadcast Remote ID. This is from FPV community sources ([IntoFPV](https://intofpv.com/printthread.php?tid=28648)). DJI's O4 FAQ does not cover Remote ID, and DJI publishes no Remote ID statement for the air units.
- Their video link is DJI's proprietary OcuSync/O3/O4 radio on 2.4/5.8 GHz, not Wi-Fi. A monitor-mode Wi-Fi card cannot decode it. Seeing that link at all needs an SDR that covers 2.4/5.8 GHz (an RTL-SDR does not).
- To make the FPV quad show up live, fit a **standalone broadcast Remote ID module with its own GPS**. It is then detected over Bluetooth LE with position, serial and operator data. [ArduRemoteID](https://github.com/ArduPilot/ArduRemoteID) (ESP32-S3/C3, Wi-Fi beacon/NAN and BT4/BT5) needs an ArduPilot/PX4 flight controller to feed it position over MAVLink/DroneCAN, so it does not suit a Betaflight build on its own.

### Demo options, in order of hardware needed

1. **No extra hardware: simulation.** `SIMULATE=1` with `SENSOR_LAT`/`SENSOR_LON` set to the demo site (in `/etc/drone-detector.env`). A scripted 5-minute scenario plays around the site:
   - a Remote ID quad circling on Bluetooth, with its operator;
   - a DJI Wi-Fi drone crossing;
   - a Parrot access point ("probable");
   - a DJI controller and an Osmo camera that are correctly *not* alarmed.

   It runs through the real parsers, classifier and CoT output. Every ATAK callsign starts `SIM-`, every CoT remark starts `SIMULATED: ` (so tac-advisor's SITREP says so too), mesh texts start `[SIM] ` and the dashboard shows a SIMULATED badge. Say on screen that it is a simulation.
2. **No extra hardware: live Bluetooth Remote ID.** The Pi's onboard Bluetooth picks up any drone nearby that broadcasts Remote ID over Bluetooth. Many current consumer drones do when Remote ID is on (check the model), as do add-on modules. Without a Wi-Fi adapter the detector runs Bluetooth-only automatically.
3. **USB monitor-mode Wi-Fi adapter** (e.g. Alfa AWUS036ACHM). Adds Wi-Fi beacon/NAN Remote ID, DJI Wi-Fi DroneID and the OUI/SSID classification.

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

**Port 8060:** the dashboard is on 8060 (the kit port plan). 8080 belongs to OpenTAKServer's nginx (Marti HTTP API). Re-running the install moves an older `/etc/drone-detector.env` from 8080 to 8060.

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

Browse to `http://isr-pi.local:8060` (`WEB_PORT`, default 8060) for the dashboard. `GET /api/status` shows the current channel, BLE state and TAK connection.

## ATAK Integration

CoT goes over plain TCP to `TAK_HOST:TAK_PORT` (default `127.0.0.1:8087`, OpenTAKServer as set up by blufor-tracker). The sender runs in the background and reconnects with backoff. The detector starts and keeps detecting when no TAK server is up. Set `SENSOR_LAT`/`SENSOR_LON` (the shared OP position) so OUI/SSID-only detections can be placed near the sensor. Without it they are not put on the map (never at 0,0), and the dashboard and heartbeat say "position not set". Disable CoT with `--no-cot`.

## Mesh Alerts

New detections (`DRONE_CONFIRMED` or `DRONE_PROBABLE`) go out as a mesh text on `meshbus/tx`, which blufor-tracker's bridge transmits on the LoRa mesh. For example:

```
DRONE: UAV K0010042 DJI 51.15269,-1.77060 60m
[SIM] DRONE: 2 new: UAV 5BKD23C0042 RID 51.15500,-1.74500 80m; probable UAV Parrot ~180m
```

- At most one text every 30 s from this tool. Detections that arrive in between are combined into the next text (up to 200 characters, then "+N more").
- A text waits 5 s after the first frame, so the serial and position are in it.
- Only new tracks alert: a first sighting, an upgrade from probable to confirmed, or a track that comes back after 10 minutes. Ongoing tracks only update the map.
- `MESH_ALERTS=0` (or `--no-mesh`) turns them off.

CoT updates are limited to one per 5 s per track, with a 5-minute stale time so markers stay up between updates.

## Heartbeat

Every 10 s the detector publishes a retained JSON message to `isrkit/heartbeat/drone-detector` for the launcher. It uses `MQTT_HOST` etc., default the Pi's broker as `guest`. `host` is `ISR_HOST` if set, else the lowercased short hostname.
- `ok`: Wi-Fi capture plus Bluetooth are running.
- `degraded`: only one of them is running, e.g. no Wi-Fi adapter.
- `error`: neither is running.

The `detail` field says which, plus the channel and the TAK state. `mode` is `cpu`, or `simulate` in simulation. It publishes `stopped` on a clean exit; `HEARTBEAT=0` turns it off.

## Tests

```bash
pip install -r requirements.txt pytest httpx
python -m pytest
```

The tests build real-format 802.11 frames with scapy. They cover DJI DroneID v1/v2, Remote ID over beacon, NAN and BLE with hand-checked byte vectors, the classification rules, CoT output and the channel hopper.
