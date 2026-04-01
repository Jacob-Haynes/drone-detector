"""FastAPI web UI for drone detector dashboard."""

import asyncio
import json
import logging
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"

app = FastAPI(title="Drone Detector")
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

# Set by __main__.py before starting the server
engine = None

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
    _clients -= dead


async def periodic_broadcast():
    """Broadcast updates to WebSocket clients every second."""
    while True:
        await broadcast_update()
        await asyncio.sleep(1)