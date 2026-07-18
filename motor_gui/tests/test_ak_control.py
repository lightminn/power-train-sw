import struct
import sys
from pathlib import Path

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "motor_control" / "steering"))
from ak_control import AK40  # noqa: E402


class StubBus:
    def __init__(self):
        self.sent = []
    def send(self, msg, timeout=None):
        self.sent.append(msg)


def test_send_brake_prohibited_on_active_profile():
    """실전 프로파일(AK45-36)에서 브레이크 패킷은 제동이 아니라 폭주 → 호출 자체가 금지."""
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    with pytest.raises(RuntimeError, match="금지"):
        m.send_brake(2.0)
    assert bus.sent == []                             # 프레임이 버스로 나가면 안 됨


def test_send_duty_frame():
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    assert m.send_duty(0.5) is True
    msg = bus.sent[-1]
    assert msg.arbitration_id == (0 << 8) | 10        # PKT_SET_DUTY=0
    assert msg.data == struct.pack(">i", 50000)       # 0.5 × 100000


def test_send_duty_clamps():
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    m.send_duty(5.0)                                  # 과도 → 0.95 클램프
    assert bus.sent[-1].data == struct.pack(">i", 95000)


def test_send_brake_prohibited_regardless_of_value():
    """값과 무관하게 금지 — 0A/음수도 프레임이 나가면 안 됨."""
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    for value in (-1.0, 0.0, 2.0):
        with pytest.raises(RuntimeError):
            m.send_brake(value)
    assert bus.sent == []


def test_send_duty_clamps_negative():
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    m.send_duty(-5.0)                                 # 과도 음수 → -0.95 클램프
    assert bus.sent[-1].data == struct.pack(">i", -95000)
