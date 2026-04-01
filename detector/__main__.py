"""Entry point: python -m detector"""

import argparse
import asyncio
import logging
import threading

import uvicorn

from detector.cot_emitter import CotEmitter
from detector.engine import DetectionEngine
from detector.scanner import start_capture
from detector.web import app as web_module


def parse_args():
    parser = argparse.ArgumentParser(description="WiFi Drone Detector")
    parser.add_argument(
        "--interface", "-i", default="wlan1mon", help="WiFi monitor mode interface"
    )
    parser.add_argument(
        "--web-port", type=int, default=8080, help="Web UI port"
    )
    parser.add_argument(
        "--fts-host", default="127.0.0.1", help="FreeTAKServer host for CoT output"
    )
    parser.add_argument(
        "--fts-port", type=int, default=8087, help="FreeTAKServer port"
    )
    parser.add_argument(
        "--no-cot", action="store_true", help="Disable CoT output to FTS"
    )
    parser.add_argument(
        "--simulate", action="store_true", help="Run with simulated detections (no hardware)"
    )
    return parser.parse_args()


def run_simulator(engine: DetectionEngine):
    """Generate fake detections for testing without hardware."""
    import random
    import time

    fake_devices = [
        {"mac": "60:60:1F:AA:BB:CC", "vendor": "DJI", "ssid": "DJI-Mavic3-AABBCC"},
        {"mac": "90:3A:E6:11:22:33", "vendor": "Parrot", "ssid": ""},
        {"mac": "AA:BB:CC:DD:EE:01", "vendor": "", "ssid": "iPhone-Jake"},
        {"mac": "AA:BB:CC:DD:EE:02", "vendor": "", "ssid": "Galaxy-S24"},
        {"mac": "AA:BB:CC:DD:EE:03", "vendor": "", "ssid": ""},
    ]

    logging.getLogger("detector").info("Simulator started with %d fake devices", len(fake_devices))

    while True:
        dev = random.choice(fake_devices)
        base_rssi = -45 if "DJI" in dev.get("vendor", "") else -70
        rssi = base_rssi + random.randint(-15, 15)

        frame = {
            "mac": dev["mac"],
            "rssi": rssi,
            "ssid": dev.get("ssid", ""),
            "frame_type": "beacon",
            "vendor_ies": [],
        }

        # Simulate DJI DroneID vendor IE for DJI devices
        if dev["vendor"] == "DJI" and random.random() < 0.3:
            import struct

            payload = bytearray(58)
            payload[0] = 0x10  # msg type
            serial = b"DJI1234567890\x00\x00\x00"
            payload[2:18] = serial
            # Salisbury Plain area coords
            lat = 51.15 + random.uniform(-0.01, 0.01)
            lon = -1.75 + random.uniform(-0.01, 0.01)
            struct.pack_into("<i", payload, 18, int(lon * 1e7))
            struct.pack_into("<i", payload, 22, int(lat * 1e7))
            struct.pack_into("<h", payload, 26, random.randint(30, 120))
            struct.pack_into("<h", payload, 28, random.randint(-500, 500))
            struct.pack_into("<h", payload, 30, random.randint(-500, 500))

            frame["vendor_ies"] = [(b"\x26\xd5\xb0", bytes(payload))]

        engine.process_frame(frame)
        time.sleep(random.uniform(0.1, 0.5))


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    args = parse_args()

    # CoT emitter (optional)
    cot = None
    if not args.no_cot:
        cot = CotEmitter(fts_host=args.fts_host, fts_port=args.fts_port)

    def on_alert(device):
        if cot:
            cot.emit(device)

    engine = DetectionEngine(on_alert=on_alert)
    web_module.engine = engine

    # Start scanner or simulator in background thread
    if args.simulate:
        threading.Thread(target=run_simulator, args=(engine,), daemon=True).start()
    else:
        threading.Thread(
            target=start_capture,
            args=(args.interface, engine.process_frame),
            daemon=True,
        ).start()

    # Start periodic WebSocket broadcast
    loop = asyncio.new_event_loop()

    async def run_web():
        config = uvicorn.Config(
            web_module.app,
            host="0.0.0.0",
            port=args.web_port,
            log_level="warning",
            loop="asyncio",
        )
        server = uvicorn.Server(config)

        broadcast_task = asyncio.create_task(web_module.periodic_broadcast())
        try:
            await server.serve()
        finally:
            broadcast_task.cancel()

    logging.getLogger("detector").info(
        "Drone Detector started — Web UI at http://0.0.0.0:%d", args.web_port
    )

    try:
        loop.run_until_complete(run_web())
    except KeyboardInterrupt:
        logging.getLogger("detector").info("Shutting down")


if __name__ == "__main__":
    main()