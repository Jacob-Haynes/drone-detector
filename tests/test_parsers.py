import pytest

from detector.parsers import dji_droneid, remote_id
from detector.scanner import packet_to_frame
from tests import frames


def test_location_vector_decodes_to_spec_values():
    d = remote_id.parse_messages(frames.LOCATION)
    assert d.drone_lat == pytest.approx(51.4778)
    assert d.drone_lon == pytest.approx(-0.0015)
    assert d.heading == 90 and d.speed == pytest.approx(10.0)
    assert d.drone_alt == pytest.approx(120.0)  # geodetic preferred over baro
    assert d.height_agl == pytest.approx(50.0)


def test_location_east_west_flag_and_speed_multiplier():
    msg = bytearray(frames.LOCATION)
    msg[1] |= 0b11  # EW direction (+180) and speed multiplier
    msg[2], msg[3] = 10, 20
    d = remote_id.parse_messages(bytes(msg))
    assert d.heading == 190
    assert d.speed == pytest.approx(20 * 0.75 + 255 * 0.25)


def test_basic_id_vector():
    d = remote_id.parse_messages(frames.BASIC_ID)
    assert d.serial == "1596F35D7A12345" and d.ua_type == "multirotor"


def test_message_pack_with_operator():
    d = remote_id.parse_messages(frames.PACK)
    assert d.serial == "1596F35D7A12345"
    assert (d.operator_lat, d.operator_lon) == (pytest.approx(frames.OPERATOR_LAT), pytest.approx(frames.OPERATOR_LON))
    assert d.operator_id == "GBR-OP-12345678"
    assert d.has_position


def test_wifi_beacon_transport():
    frame = packet_to_frame(frames.odid_beacon())
    oui, payload = frame["vendor_ies"][0]
    d = remote_id.parse_vendor_ie(oui, payload)
    assert d.source == "remoteid-wifi-beacon" and d.serial == "1596F35D7A12345"


def test_wifi_nan_transport():
    frame = packet_to_frame(frames.nan_frame())
    assert frame["frame_type"] == "nan"
    d = frame["drone_data"]
    assert d.source == "remoteid-wifi-nan" and d.drone_lat == pytest.approx(51.4778)


def test_nan_frame_to_other_destination_is_ignored():
    pkt = frames.nan_frame()
    pkt.addr1 = "ff:ff:ff:ff:ff:ff"
    assert packet_to_frame(frames.RadioTap(bytes(pkt))) is None


def test_ble_legacy_single_message_and_bt5_pack():
    d = remote_id.parse_ble_service_data(bytes([0x0D, 0x05]) + frames.LOCATION)
    assert d.source == "remoteid-ble" and d.drone_lon == pytest.approx(-0.0015)
    d = remote_id.parse_ble_service_data(bytes([0x0D, 0x06]) + frames.PACK)
    assert d.serial == "1596F35D7A12345"
    assert remote_id.parse_ble_service_data(bytes([0x0E, 0]) + frames.LOCATION) is None


def test_rejects_garbage():
    assert remote_id.parse_vendor_ie(b"\xFA\x0B\xBC", b"\x0D\x00" + bytes(25)) is None  # basic id with empty serial
    assert remote_id.parse_messages(bytes([0xF2, 25, 9]) + bytes(10)) is None
    assert remote_id.parse_nan_action(b"\x04\x09\x50\x6F\x9A\x13\x03\xff\xff") is None


def test_dji_v1_flight_registration():
    frame = packet_to_frame(frames.dji_beacon(frames.dji_flight_reg_v1()))
    oui, payload = frame["vendor_ies"][0]
    d = dji_droneid.parse_vendor_ie(oui, payload)
    assert d.serial == "0JXDE4A0010123" and d.source == "dji-droneid"
    assert d.drone_lat == pytest.approx(51.15, abs=1e-5)
    assert d.drone_lon == pytest.approx(-1.75, abs=1e-5)
    assert d.drone_alt == 85 and d.height_agl == 80
    assert d.speed == pytest.approx(5.0)  # hypot(3, 4) m/s
    assert d.heading == pytest.approx(45.0)
    assert d.home_lat == pytest.approx(51.149, abs=1e-5)
    assert d.operator_lat == d.home_lat  # v1 has no app position; home is the best guess


def test_dji_v2_has_app_position():
    d = dji_droneid.parse_vendor_ie(dji_droneid.DJI_OUI, frames.dji_flight_reg_v2())
    assert d.serial == "1581F5FHC245ABCD"
    assert d.height_agl == pytest.approx(60.0) and d.drone_alt == 70
    assert d.operator_lat == pytest.approx(51.1585, abs=1e-5)
    assert d.operator_lon == pytest.approx(-1.7620, abs=1e-5)
    assert d.heading == pytest.approx(270.0)


def test_dji_flight_purpose_and_wrong_oui():
    rec = b"1581F5FHC245ABCD" + bytes([4]) + b"ABCD".ljust(10, b"\0") + bytes([5]) + b"Recon"
    d = dji_droneid.parse_vendor_ie(dji_droneid.DJI_OUI, bytes([0x58, 0x62, 0x13, 0x11]) + rec)
    assert d.serial == "1581F5FHC245ABCD" and not d.has_position
    assert dji_droneid.parse_vendor_ie(b"\x26\xd5\xb0", frames.dji_flight_reg_v1()) is None


def test_dji_zero_position_is_not_a_fix():
    d = dji_droneid.parse_vendor_ie(dji_droneid.DJI_OUI, frames.dji_flight_reg_v1(lat=0.0, lon=0.0))
    assert d.serial and not d.has_position
