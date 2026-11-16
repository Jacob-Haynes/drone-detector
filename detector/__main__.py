"""Entry point: python -m detector"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import random
import struct
import threading
import time

import uvicorn

from detector.cot_emitter import CotEmitter
from detector.engine import DetectionEngine
from detector.web import app as web_module

log = logging.getLogger("detector")


def _env_float(name: str) -> float | None:
    value = os.environ.get(name, "").strip()
    return float(value) if value else None


def parse_args():
    env = os.environ.get
    parser = argparse.ArgumentParser(description="Wi-Fi + Bluetooth drone detector (DJI DroneID, Remote ID)")
    parser.add_argument("--interface", "-i", default=env("WIFI_IFACE", "wlan1mon"),
                        help="Wi-Fi interface already in monitor mode (env WIFI_IFACE)")
    parser.add_argument("--no-wifi", action="store_true", help="No Wi-Fi capture (BLE only)")
    parser.add_argument("--channels", default=env("WIFI_CHANNELS", ""),
                        help="Comma-separated channels to hop, e.g. 1,6,11,149. Default: 2.4+5 GHz plan")
    parser.add_argument("--bands", default=env("WIFI_BANDS", "2.4,5"), help="Bands for the default plan")
    parser.add_argument("--dwell", type=float, default=float(env("HOP_DWELL", "0.3")),
                        help="Seconds per channel")
    parser.add_argument("--no-hop", action="store_true", help="Stay on the interface's current channel")
    ble = parser.add_mutually_exclusive_group()
    ble.add_argument("--ble", dest="ble", action="store_true", default=env("BLE_SCAN", "1") != "0",
                     help="Scan Bluetooth LE Remote ID (default on; env BLE_SCAN=0 disables)")
    ble.add_argument("--no-ble", dest="ble", action="store_false")
    parser.add_argument("--ble-adapter", default=env("BLE_ADAPTER") or None, help="e.g. hci0")
    parser.add_argument("--web-port", type=int, default=int(env("WEB_PORT", "8080")), help="Web UI port")
    parser.add_argument("--tak-host", "--fts-host", default=env("TAK_HOST", "127.0.0.1"),
                        help="TAK server host for CoT output (env TAK_HOST)")
    parser.add_argument("--tak-port", "--fts-port", type=int, default=int(env("TAK_PORT", "8087")),
                        help="TAK server CoT TCP port (env TAK_PORT)")
    parser.add_argument("--sensor-lat", type=float, default=_env_float("SENSOR_LAT"),
                        help="This sensor's latitude, used to place detections that carry no GPS")
    parser.add_argument("--sensor-lon", type=float, default=_env_float("SENSOR_LON"))
    parser.add_argument("--no-cot", action="store_true", help="Disable CoT output")
    parser.add_argument("--simulate", action="store_true",
                        help="Simulated detections, no hardware (CoT callsigns get a SIM- prefix)")
    return parser.parse_args()


# ── simulator (real-format payloads, so the parsers run for real) ──

def _dji_flight_reg(serial: bytes, lat: float, lon: float, alt: int, home_lat: float, home_lon: float) -> bytes:
    raw = lambda deg: int(deg * 174533.0)  # noqa: E731
    rec = struct.pack("<BHH16sii", 1, random.randint(0, 65535), 0x0FFF, serial.ljust(16, b"\0"),
                      raw(lon), raw(lat))
    rec += struct.pack("<hhhhhhhhii", alt, alt, random.randint(-800, 800), random.randint(-800, 800), 0,
                       0, 0, random.randint(-18000, 18000), raw(home_lon), raw(home_lat))
    rec += bytes([16, 0])
    return bytes([0x58, 0x62, 0x13, 0x10]) + rec


def _rid_location(lat: float, lon: float, alt: float) -> bytes:
    enc_alt = int((alt + 1000) / 0.5)
    msg = bytes([0x12, 0x20, random.randint(0, 179), random.randint(10, 60), 0])
    msg += struct.pack("<iiHHH", int(lat * 1e7), int(lon * 1e7), enc_alt, enc_alt, int((40 + 1000) / 0.5))
    return msg + bytes(6)


def _rid_basic(serial: bytes) -> bytes:
    return bytes([0x02, 0x12]) + serial.ljust(20, b"\0") + bytes(3)


def run_simulator(engine: DetectionEngine):
    """Generate fake detections for testing without hardware."""
    base_lat, base_lon = 51.15, -1.75
    devices = [
        {"mac": "60:60:1F:AA:BB:CC", "ssid": "DJI-Mavic3-AABBCC", "kind": "dji"},
        {"mac": "60:60:1F:11:22:33", "ssid": "", "kind": "dji-rc"},  # DJI controller probing
        {"mac": "60:60:1F:44:55:66", "ssid": "OsmoAction4-5566", "kind": "ap"},  # camera, not a drone
        {"mac": "90:3A:E6:11:22:33", "ssid": "Anafi-123456", "kind": "ap"},
        {"mac": "E4:5F:01:AA:BB:01", "ssid": "BTHome-Hub", "kind": "ap"},
        {"mac": "AA:BB:CC:DD:EE:02", "ssid": "Galaxy-S24", "kind": "probe"},
        {"mac": "BLE:C0:FF:EE:00:00:01", "ssid": "", "kind": "ble-rid"},
    ]
    log.info("Simulator started with %d fake devices", len(devices))
    while True:
        dev = random.choice(devices)
        rssi = random.randint(-85, -45)
        frame = {"mac": dev["mac"], "rssi": rssi, "ssid": dev["ssid"], "vendor_ies": [],
                 "frame_type": "beacon", "transport": "wifi"}
        lat = base_lat + random.uniform(-0.01, 0.01)
        lon = base_lon + random.uniform(-0.01, 0.01)
        if dev["kind"] == "dji" and random.random() < 0.3:
            frame["vendor_ies"] = [(b"\x26\x37\x12", _dji_flight_reg(b"1581F4XSIM00001", lat, lon,
                                                                     random.randint(30, 120),
                                                                     base_lat + 0.002, base_lon - 0.003))]
        elif dev["kind"] in ("dji-rc", "probe"):
            frame["frame_type"] = "probe_req"
        elif dev["kind"] == "ble-rid":
            from detector.parsers import remote_id
            msg = _rid_basic(b"1596ASIMRID0001") if random.random() < 0.4 else _rid_location(lat, lon, 80.0)
            frame.update(frame_type="ble", transport="ble",
                         drone_data=remote_id.parse_ble_service_data(bytes([0x0D, random.randint(0, 255)]) + msg))
        engine.process_frame(frame)
        time.sleep(random.uniform(0.1, 0.5))


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    args = parse_args()

    cot = None
    if not args.no_cot:
        cot = CotEmitter(tak_host=args.tak_host, tak_port=args.tak_port, sensor_lat=args.sensor_lat,
                         sensor_lon=args.sensor_lon, simulated=args.simulate)
        cot.start()

    engine = DetectionEngine(on_alert=cot.emit if cot else None)
    web_module.engine = engine
    web_module.status = {"mode": "simulate" if args.simulate else "live", "interface": None,
                         "hopper": None, "ble": None, "cot": cot}

    if args.simulate:
        threading.Thread(target=run_simulator, args=(engine,), daemon=True).start()
    else:
        if not args.no_wifi:
            from detector.scanner import ChannelHopper, default_channel_plan, parse_channels, start_capture

            web_module.status["interface"] = args.interface
            if not args.no_hop:
                channels = parse_channels(args.channels) if args.channels else default_channel_plan(args.bands)
                hopper = ChannelHopper(args.interface, channels, dwell=args.dwell)
                hopper.start()
                web_module.status["hopper"] = hopper
            threading.Thread(target=start_capture, args=(args.interface, engine.process_frame),
                             daemon=True).start()
        else:
            log.warning("Wi-Fi capture disabled (--no-wifi)")
        if args.ble:
            from detector.ble_scanner import BleRemoteIdScanner

            ble = BleRemoteIdScanner(engine.process_frame, adapter=args.ble_adapter)
            ble.start()
            web_module.status["ble"] = ble

    async def run_web():
        config = uvicorn.Config(web_module.app, host="0.0.0.0", port=args.web_port,
                                log_level="warning", loop="asyncio")
        server = uvicorn.Server(config)
        broadcast_task = asyncio.create_task(web_module.periodic_broadcast())
        try:
            await server.serve()
        finally:
            broadcast_task.cancel()

    log.info("Drone Detector started — Web UI at http://0.0.0.0:%d", args.web_port)
    try:
        asyncio.run(run_web())
    except KeyboardInterrupt:
        log.info("Shutting down")


if __name__ == "__main__":
    main()
