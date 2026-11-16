"""Entry point: python -m detector"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import threading

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
        from detector.simulator import DemoScenario

        centre = (args.sensor_lat, args.sensor_lon) if args.sensor_lat is not None and args.sensor_lon is not None \
            else None
        if cot is not None and centre is None:
            # put "probable" detections near the scenario too
            from detector.simulator import DEFAULT_CENTRE
            cot.sensor_lat, cot.sensor_lon = DEFAULT_CENTRE
        threading.Thread(target=DemoScenario(engine, centre).run, daemon=True, name="sim").start()
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

    from detector.heartbeat import Heartbeat
    from detector.scanner import CAPTURE_STATE

    def heartbeat_state():
        if args.simulate:
            return "simulate", "ok", "scripted demo scenario (SIM- callsigns)"
        wifi = (not args.no_wifi) and CAPTURE_STATE["running"]
        ble = web_module.status.get("ble")
        ble_on = bool(getattr(ble, "active", False))
        hopper = web_module.status.get("hopper")
        parts = [f"wifi {args.interface} ch {getattr(hopper, 'current', '-')}" if wifi else
                 f"no wifi capture{': ' + CAPTURE_STATE['error'] if CAPTURE_STATE['error'] else ''}",
                 "ble on" if ble_on else "ble off"]
        if cot is not None:
            parts.append("tak connected" if cot.connected else "tak down")
        status = "ok" if wifi and (ble_on or not args.ble) else ("degraded" if wifi or ble_on else "error")
        return "cpu", status, "; ".join(parts)

    heartbeat = Heartbeat("drone-detector", heartbeat_state, port=args.web_port).start()

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
    finally:
        heartbeat.stop()


if __name__ == "__main__":
    main()
