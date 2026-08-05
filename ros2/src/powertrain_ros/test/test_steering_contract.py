"""조향/트랜스포트 순수 계약 — ROS 없는 호스트에서 검증한다."""
import json

import pytest

from powertrain_ros import steering_contract as contract


class _FakeManager:
    def __init__(self):
        self.steering_mode = "ackermann"
        self.steering_available = True


def test_usb_transport_rejects_ackermann_at_startup():
    with pytest.raises(ValueError, match="ackermann"):
        contract.validate_transport_mode("usb", "ackermann")


def test_usb_transport_accepts_skid():
    contract.validate_transport_mode("usb", "skid")     # 예외가 나면 실패


def test_can_transport_accepts_both():
    contract.validate_transport_mode("can", "ackermann")
    contract.validate_transport_mode("can", "skid")


def test_unknown_transport_is_refused():
    with pytest.raises(ValueError, match="drive_transport"):
        contract.validate_transport_mode("spi", "skid")


def test_safety_state_payload_carries_the_mode_fields():
    payload = contract.steering_state_fields(
        _FakeManager(), drive_transport="usb")

    assert payload == {
        "steering_mode": "ackermann",
        "steering_available": True,
        "drive_transport": "usb",
    }
    json.dumps(payload)          # 직렬화 가능해야 한다
