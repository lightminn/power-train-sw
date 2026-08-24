"""Pure-Python protocol tests for the read-only PDIST80B codec."""

from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from powertrain_ros.pdist80b import (
    BATTERY_FAULT_MASK,
    BATTERY_FLAG_LABELS,
    OPERATOR_MID,
    PDIST_MID,
    PID_BMS_MONITOR,
    PID_REQUEST_DATA,
    PROTECTION_FAULT_MASK,
    PROTECTION_FLAG_LABELS,
    battery_flag_labels,
    bms_monitor_request,
    checksum,
    fault_reasons,
    parse_bms_monitor_response,
    protection_flag_labels,
)


def _response_packet(*, device_id=1):
    data = b"".join(
        (
            (487).to_bytes(2, "little"),
            (-123).to_bytes(2, "little", signed=True),
            bytes((81, 0xA5, 0x5A, 0x00)),
            (45).to_bytes(2, "little", signed=True),
            b"\x00\x00",
        )
    )
    body = bytes(
        (
            OPERATOR_MID,
            PDIST_MID,
            device_id,
            PID_BMS_MONITOR,
            len(data),
        )
    ) + data
    return body + bytes((checksum(body),))


def test_bms_monitor_request_bytes_and_checksum():
    packet = bms_monitor_request(1)

    assert packet == bytes(
        (
            PDIST_MID,
            OPERATOR_MID,
            1,
            PID_REQUEST_DATA,
            1,
            PID_BMS_MONITOR,
            0xA6,
        )
    )
    assert sum(packet) & 0xFF == 0


def test_parse_valid_bms_monitor_response():
    status = parse_bms_monitor_response(_response_packet(), device_id=1)

    assert status.voltage_v == pytest.approx(48.7)
    assert status.discharge_current_a == pytest.approx(-12.3)
    assert status.soc_percent == 81
    assert status.battery_flags == 0xA5
    assert status.protection_flags == 0x5A
    assert status.charge_current_a == pytest.approx(4.5)


def test_manual_v17_flag_tables_and_fault_masks():
    assert BATTERY_FLAG_LABELS == (
        "수동 충전기 결합",
        "자동 충전기 결합",
        "과전압 보호",
        "저전압 보호",
        "충전 과온 보호",
        "충전 저온 보호",
        "방전 과온 보호",
        "방전 저온 보호",
    )
    assert PROTECTION_FLAG_LABELS == (
        "충전 과전류 보호",
        "방전 과전류 보호",
        "단락 보호",
        "단락 보호",
        "예약",
        "외부 제어",
        "충전 플래그",
    )
    assert BATTERY_FAULT_MASK == 0xFC
    assert PROTECTION_FAULT_MASK == 0x0F


def test_fault_reasons_ignore_status_bits_and_keep_stable_bit_order():
    assert fault_reasons(0x03, 0x60) == ()
    assert fault_reasons(0x8C, 0x0B) == (
        "과전압 보호",
        "저전압 보호",
        "방전 저온 보호",
        "충전 과전류 보호",
        "방전 과전류 보호",
        "단락 보호",
    )
    assert fault_reasons(0, 0x0C) == ("단락 보호",)


def test_all_set_flag_labels_include_status_bits_for_developer_display():
    assert battery_flag_labels(0x03) == (
        "수동 충전기 결합",
        "자동 충전기 결합",
    )
    assert protection_flag_labels(0x60) == (
        "외부 제어",
        "충전 플래그",
    )
    assert protection_flag_labels(0x80) == ()


def test_parse_rejects_wrong_response_length():
    with pytest.raises(ValueError, match="18 bytes"):
        parse_bms_monitor_response(_response_packet()[:-1])


def test_parse_rejects_bad_checksum():
    packet = bytearray(_response_packet())
    packet[-1] ^= 0x01

    with pytest.raises(ValueError, match="checksum"):
        parse_bms_monitor_response(bytes(packet))


def test_parse_rejects_unexpected_header():
    packet = bytearray(_response_packet())
    packet[0] = PDIST_MID
    packet[-1] = checksum(bytes(packet[:-1]))

    with pytest.raises(ValueError, match="header"):
        parse_bms_monitor_response(bytes(packet))


@pytest.mark.parametrize("device_id", (0, 253))
def test_request_accepts_device_id_boundaries(device_id):
    assert bms_monitor_request(device_id)[2] == device_id


@pytest.mark.parametrize("device_id", (-1, 254))
def test_request_rejects_device_id_outside_protocol_range(device_id):
    with pytest.raises(ValueError, match="0..253"):
        bms_monitor_request(device_id)
