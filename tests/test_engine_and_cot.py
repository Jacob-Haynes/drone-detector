import socket
import threading
import time
import xml.etree.ElementTree as ET

from detector.ble_scanner import advertisement_to_frame
from detector.cot_emitter import CotEmitter
from detector.engine import DetectionEngine
from detector.models import Classification
from detector.parsers import remote_id
from detector.scanner import ChannelHopper, default_channel_plan, packet_to_frame
from tests import frames


def feed(engine, pkt, n=1):
    for _ in range(n):
        engine.process_frame(packet_to_frame(pkt))


def cls(engine, mac):
    return engine.devices[mac].classification


def test_dji_droneid_beacon_confirms_and_alerts():
    alerts = []
    e = DetectionEngine(on_alert=alerts.append)
    feed(e, frames.dji_beacon(frames.dji_flight_reg_v1()))
    assert cls(e, "60:60:1F:AA:BB:CC") == Classification.DRONE_CONFIRMED
    assert alerts and alerts[0].drone_id.serial == "0JXDE4A0010123"


def test_dji_oui_alone_does_not_false_alarm_on_controllers_and_cameras():
    alerts = []
    e = DetectionEngine(on_alert=alerts.append)
    feed(e, frames.probe_req("60:60:1F:11:22:33"), n=20)  # DJI RC / goggles probing
    feed(e, frames.beacon("60:60:1F:44:55:66", ssid="OsmoAction4-5566"), n=20)  # action cam AP
    feed(e, frames.probe_req("3A:11:22:33:44:55", ssid="TELLO-5A3F2B"), n=3)  # phone looking for a Tello
    assert cls(e, "60:60:1F:11:22:33") == Classification.DRONE_RELATED
    assert cls(e, "60:60:1F:44:55:66") == Classification.DRONE_RELATED
    assert "operator" in e.devices["3A:11:22:33:44:55"].reason
    assert alerts == []


def test_drone_maker_access_point_is_probable_and_ordinary_wifi_is_device():
    alerts = []
    e = DetectionEngine(on_alert=alerts.append)
    feed(e, frames.beacon("90:3A:E6:11:22:33", ssid="Anafi-123456"))
    feed(e, frames.beacon("E4:5F:01:AA:BB:01", ssid="BTHome-Hub"), n=15)
    assert cls(e, "90:3A:E6:11:22:33") == Classification.DRONE_PROBABLE
    assert cls(e, "E4:5F:01:AA:BB:01") == Classification.DEVICE
    assert [a.mac for a in alerts] == ["90:3A:E6:11:22:33"]


def test_randomised_mac_with_drone_oui_bits_is_ignored():
    e = DetectionEngine()
    feed(e, frames.beacon("62:60:1F:AA:BB:CC", ssid=""), n=15)  # locally administered
    assert cls(e, "62:60:1F:AA:BB:CC") == Classification.DEVICE


def test_ble_remote_id_messages_merge_per_device():
    e = DetectionEngine()
    sd = {remote_id.BLE_SERVICE_UUID: bytes([0x0D, 1]) + frames.BASIC_ID}
    e.process_frame(advertisement_to_frame("c0:ff:ee:00:00:01", -70, sd))
    sd = {remote_id.BLE_SERVICE_UUID: bytes([0x0D, 2]) + frames.LOCATION}
    e.process_frame(advertisement_to_frame("c0:ff:ee:00:00:01", -68, sd))
    dev = e.devices["BLE:C0:FF:EE:00:00:01"]
    assert dev.classification == Classification.DRONE_CONFIRMED and dev.transport == "ble"
    assert dev.drone_id.serial == "1596F35D7A12345" and dev.drone_id.has_position
    assert advertisement_to_frame("aa:bb", -50, {"0000180f-0000-1000-8000-00805f9b34fb": b"\x01"}) is None


def test_alerts_are_throttled_but_refreshed():
    alerts = []
    e = DetectionEngine(on_alert=alerts.append)
    pkt = frames.dji_beacon(frames.dji_flight_reg_v1())
    feed(e, pkt, n=10)  # 10 beacons in a burst -> 1 alert
    assert len(alerts) == 1
    e._last_alert["60:60:1F:AA:BB:CC"] -= 5  # pretend 5 s passed
    feed(e, pkt)
    assert len(alerts) == 2


def test_cot_confirmed_drone_and_operator():
    e = DetectionEngine()
    feed(e, frames.odid_beacon())
    dev = e.devices["AC:12:34:56:78:9A"]
    events = [ET.fromstring(x) for x in CotEmitter().build(dev)]
    uav, op = events
    assert uav.get("type") == "a-h-A-M-F-Q" and uav.get("uid") == "DRONE-AC123456789A"
    assert float(uav.find("point").get("lat")) == 51.4778
    assert uav.find("detail/contact").get("callsign") == "1596F35D7A12345"
    assert "drone-detector" in uav.find("detail/remarks").text
    assert op.get("type") == "a-u-G" and op.find("detail/link").get("uid") == uav.get("uid")


def test_cot_probable_uses_sensor_position_or_nothing():
    e = DetectionEngine()
    feed(e, frames.beacon("90:3A:E6:11:22:33", ssid="Anafi-123456", rssi=-60))
    dev = e.devices["90:3A:E6:11:22:33"]
    assert CotEmitter().build(dev) == []  # never at 0,0
    (xml,) = CotEmitter(sensor_lat=51.2, sensor_lon=-1.8, simulated=True).build(dev)
    ev = ET.fromstring(xml)
    assert ev.get("type") == "a-u-A-M-F-Q" and float(ev.find("point").get("ce")) >= 50
    assert ev.find("detail/contact").get("callsign").startswith("SIM-")


def test_related_devices_never_produce_cot():
    e = DetectionEngine()
    feed(e, frames.probe_req("60:60:1F:11:22:33"))
    assert CotEmitter(sensor_lat=1, sensor_lon=1).build(e.devices["60:60:1F:11:22:33"]) == []


def test_cot_sender_reconnects_and_delivers():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    port = srv.getsockname()[1]
    srv.close()  # nothing listening yet: emitter must not block or crash
    em = CotEmitter(tak_port=port)
    e = DetectionEngine(on_alert=em.emit)
    feed(e, frames.odid_beacon())
    time.sleep(0.3)
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    srv.settimeout(10)
    got = b""
    conn, _ = srv.accept()
    conn.settimeout(5)
    while b"</event>" not in got:
        got += conn.recv(65536)
    assert b"DRONE-AC123456789A" in got
    em.stop()
    conn.close()
    srv.close()


def test_channel_plan_covers_both_bands_and_hopper_skips_unsupported():
    plan = default_channel_plan()
    assert {1, 6, 11, 36, 149, 165} <= set(plan)
    assert default_channel_plan("2.4") and max(default_channel_plan("2.4")) <= 13

    calls = []

    class Res:
        def __init__(self, rc):
            self.returncode, self.stderr = rc, "Invalid argument"

    def runner(cmd, **kw):
        calls.append(int(cmd[-1]))
        return Res(1 if int(cmd[-1]) == 52 else 0)

    hop = ChannelHopper("wlan1", [1, 52, 149], dwell=0.01, runner=runner)
    hop.start()
    time.sleep(0.2)
    hop.stop()
    assert 52 in hop._unsupported and calls.count(52) == 1 and hop.current in (1, 149)
