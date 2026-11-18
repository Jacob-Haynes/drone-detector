"""FastAPI web UI for drone detector dashboard."""

import asyncio
import json
import logging
from pathlib import Path

from fastapi import Body, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"

app = FastAPI(title="Drone Detector")
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

# Set by __main__.py before starting the server
engine = None
status: dict = {}
demo = None  # detector.demo.DemoRunner

# Connected WebSocket clients
_clients: set[WebSocket] = set()


@app.get("/", response_class=HTMLResponse)
async def index():
    return (STATIC_DIR / "index.html").read_text()


@app.get("/api/devices")
async def get_devices():
    if engine is None:
        return []
    return engine.get_active_devices()


@app.get("/api/alerts")
async def get_alerts():
    if engine is None:
        return []
    return engine.get_alerts()


@app.get("/api/status")
async def get_status():
    hopper = status.get("hopper")
    ble = status.get("ble")
    cot = status.get("cot")
    return {
        "mode": status.get("mode"),
        "interface": status.get("interface"),
        "channel": getattr(hopper, "current", None),
        "hopping": hopper is not None,
        "ble_scanning": bool(getattr(ble, "active", False)),
        "cot_enabled": cot is not None,
        "tak_connected": bool(getattr(cot, "connected", False)),
        "cot_sent": getattr(cot, "sent", 0),
        "mesh_alerts": status.get("mesh") is not None,
        "mesh_alerts_sent": getattr(status.get("mesh"), "sent", 0),
        "sensor_position": status.get("sensor_position"),
        "sensor_position_text": ("%.5f, %.5f" % tuple(status["sensor_position"]))
        if status.get("sensor_position") else "position not set",
        "demo_mode": getattr(demo, "mode", "continuous"),
        "demo_active": demo.active() if demo is not None else [],
    }


# ── step-by-step demo (ui-kit DEMO-TRIGGER-CONTRACT.md) ───────────

def _demo():
    if demo is None:
        raise HTTPException(503, "Demo scenarios are not available")
    return demo


@app.get("/api/demo/scenarios")
async def demo_scenarios():
    return _demo().scenarios()


@app.post("/api/demo/trigger")
async def demo_trigger(body: dict = Body(...)):
    runner = _demo()
    sid = body.get("id") if isinstance(body, dict) else None
    if not isinstance(sid, str) or sid not in {s["id"] for s in runner.scenarios()["scenarios"]}:
        raise HTTPException(400, "Unknown scenario")
    options = {k: v for k, v in body.items() if k != "id"}
    return runner.trigger(sid, **options)


@app.post("/api/demo/clear")
async def demo_clear():
    return _demo().clear()


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    _clients.add(ws)
    try:
        while True:
            await ws.receive_text()  # Keep connection alive
    except WebSocketDisconnect:
        _clients.discard(ws)


async def broadcast_update():
    """Send current state to all connected WebSocket clients."""
    if engine is None:
        return
    data = json.dumps(
        {
            "devices": engine.get_active_devices(),
            "alerts": engine.get_alerts(),
        }
    )
    dead = set()
    for ws in _clients:
        try:
            await ws.send_text(data)
        except Exception:
            dead.add(ws)
    _clients.difference_update(dead)


async def periodic_broadcast():
    """Broadcast updates to WebSocket clients every second."""
    while True:
        await broadcast_update()
        await asyncio.sleep(1)