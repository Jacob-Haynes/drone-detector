"""Entry point: python -m detector"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal
import sys
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
    parser.add_argument("--web-port", type=int, default=int(env("WEB_PORT", "8060")), help="Web UI port")
    parser.add_argument("--tak-host", "--fts-host", default=env("TAK_HOST", "127.0.0.1"),
                        help="TAK server host for CoT output (env TAK_HOST)")
    parser.add_argument("--tak-port", "--fts-port", type=int, default=int(env("TAK_PORT", "8087")),
                        help="TAK server CoT TCP port (env TAK_PORT)")
    parser.add_argument("--sensor-lat", type=float, default=_env_float("SENSOR_LAT"),
                        help="This sensor's latitude, used to place detections that carry no GPS")
    parser.add_argument("--sensor-lon", type=float, default=_env_float("SENSOR_LON"))
    parser.add_argument("--no-cot", action="store_true", help="Disable CoT output")
    parser.add_argument("--no-mesh", dest="mesh", action="store_false", default=env("MESH_ALERTS", "1") != "0",
                        help="No mesh alerts on meshbus/tx (env MESH_ALERTS=0)")
    parser.add_argument("--simulate", action="store_true",
                        help="Simulated detections, no hardware (CoT callsigns get a SIM- prefix)")
    parser.add_argument("--demo-mode", default=env("DEMO_MODE", "continuous"),
                        help="continuous (default: --simulate runs its loop) or on_demand (no background "
                             "simulation; scenarios fired from the page). Env DEMO_MODE")
    return parser.parse_args()


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    args = parse_args()
    from detector.demo import DemoRunner, demo_mode

    args.demo_mode = demo_mode(args.demo_mode)
    # uvicorn re-raises SIGTERM after its own shutdown; turn it into a normal exit so the
    # finally blocks run (heartbeat publishes "stopped" instead of the broker's "error" will)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    cot = None
    if not args.no_cot:
        cot = CotEmitter(tak_host=args.tak_host, tak_port=args.tak_port, sensor_lat=args.sensor_lat,
                         sensor_lon=args.sensor_lon, simulated=args.simulate)
        cot.start()

    mesh = None
    if args.mesh:
        from detector.mesh_alerts import MeshAlerter, mqtt_publisher

        publish = mqtt_publisher()
        if publish is not None:
            mesh = MeshAlerter(publish, simulated=args.simulate).start()

    def on_alert(device):
        if cot is not None:
            cot.emit(device)
        if mesh is not None:
            mesh.note(device)

    engine = DetectionEngine(on_alert=on_alert if (cot or mesh) else None)
    web_module.engine = engine
    web_module.status = {"mode": "simulate" if args.simulate else "live", "interface": None,
                         "hopper": None, "ble": None, "cot": cot, "mesh": mesh,
                         "sensor_position": None}

    if args.simulate:  # no hardware capture
        from detector.simulator import DemoScenario

        centre = (args.sensor_lat, args.sensor_lon) if args.sensor_lat is not None and args.sensor_lon is not None \
            else None
        if cot is not None and centre is None:
            # put "probable" detections near the scenario too
            from detector.simulator import DEFAULT_CENTRE
            cot.sensor_lat, cot.sensor_lon = DEFAULT_CENTRE
            cot.client.sensor_lat, cot.client.sensor_lon = DEFAULT_CENTRE  # registration SA too
        if args.demo_mode == "continuous":
            threading.Thread(target=DemoScenario(engine, centre).run, daemon=True, name="sim").start()
        else:
            log.info("DEMO_MODE=on_demand: no background simulation; fire scenarios from the Demo card")
        if centre is None:
            log.warning("SENSOR_LAT/SENSOR_LON not set: simulating around the default demo site")
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

    if cot is not None and cot.sensor_lat is not None and cot.sensor_lon is not None:
        web_module.status["sensor_position"] = (cot.sensor_lat, cot.sensor_lon)
    elif args.sensor_lat is not None and args.sensor_lon is not None:
        web_module.status["sensor_position"] = (args.sensor_lat, args.sensor_lon)
    else:
        log.warning("SENSOR_LAT/SENSOR_LON not set: detections without GPS are not put on the map")
    op = web_module.status["sensor_position"]
    if op is None and args.simulate:
        from detector.simulator import DEFAULT_CENTRE
        op = DEFAULT_CENTRE
    engine.op = op  # closing/opening and CPA are relative to the OP
    if mesh is not None:
        mesh.op = op

    from detector.simulator import DEFAULT_CENTRE

    demo = DemoRunner(engine, centre=op or DEFAULT_CENTRE, cot=cot, mesh=mesh, mode=args.demo_mode).start()
    web_module.demo = demo
    if args.demo_mode == "on_demand" and not args.simulate:
        log.info("DEMO_MODE=on_demand: real capture on, simulated drones only when a scenario is fired")

    from detector.heartbeat import Heartbeat
    from detector.scanner import CAPTURE_STATE

    def demo_text():
        active = demo.active()
        if args.demo_mode == "continuous" and not active:
            return ""
        return f"; demo {args.demo_mode}" + (f": {', '.join(active)} (SIMULATED)" if active else "")

    def heartbeat_state():
        if args.simulate:
            what = ("scripted demo scenario (SIMULATED)" if args.demo_mode == "continuous"
                    else "step-by-step demo, no hardware")
            return "simulate", "ok", what + demo_text()
        wifi = (not args.no_wifi) and CAPTURE_STATE["running"]
        ble = web_module.status.get("ble")
        ble_on = bool(getattr(ble, "active", False))
        hopper = web_module.status.get("hopper")
        parts = [f"wifi {args.interface} ch {getattr(hopper, 'current', '-')}" if wifi else
                 f"no wifi capture{': ' + CAPTURE_STATE['error'] if CAPTURE_STATE['error'] else ''}",
                 "ble on" if ble_on else "ble off"]
        if cot is not None:
            parts.append("tak connected" if cot.connected else "tak down")
        if web_module.status.get("sensor_position") is None:
            parts.append("position not set")
        status = "ok" if wifi and (ble_on or not args.ble) else ("degraded" if wifi or ble_on else "error")
        return "cpu", status, "; ".join(parts) + demo_text()

    heartbeat = Heartbeat("drone-detector", heartbeat_state, port=args.web_port,
                          extra_fn=lambda: {"demo_mode": args.demo_mode, "demo_active": demo.active()}).start()

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
