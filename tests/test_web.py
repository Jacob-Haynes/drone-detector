import asyncio

from fastapi.testclient import TestClient

from detector.engine import DetectionEngine
from detector.scanner import packet_to_frame
from detector.web import app as web_module
from tests import frames


def test_api_and_broadcast_survive_dead_client():
    web_module.engine = DetectionEngine()
    web_module.engine.process_frame(packet_to_frame(frames.odid_beacon()))
    client = TestClient(web_module.app)
    devices = client.get("/api/devices").json()
    assert devices[0]["classification"] == "DRONE_CONFIRMED"
    assert devices[0]["drone_id"]["source"] == "remoteid-wifi-beacon"
    assert client.get("/api/status").status_code == 200

    class DeadWS:
        async def send_text(self, data):
            raise RuntimeError("closed")

    web_module._clients.add(DeadWS())
    asyncio.run(web_module.broadcast_update())  # used to raise UnboundLocalError
    assert not web_module._clients
