import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "motor_control" / "steering"))
from ak_control import AK40  # noqa: E402


class StubBus:
    def __init__(self):
        self.sent = []
    def send(self, msg, timeout=None):
        self.sent.append(msg)


def test_send_brake_frame():
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    assert m.send_brake(2.0) is True
    msg = bus.sent[-1]
    assert msg.is_extended_id is True
    assert msg.arbitration_id == (2 << 8) | 10        # PKT_SET_BRAKE=2, id=10
    assert msg.data == struct.pack(">i", 2000)        # 2.0A → 2000 mA


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


def test_send_brake_clamps_negative():
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    m.send_brake(-1.0)                                # 음수 → 0 클램프
    assert bus.sent[-1].data == struct.pack(">i", 0)


def test_send_duty_clamps_negative():
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    m.send_duty(-5.0)                                 # 과도 음수 → -0.95 클램프
    assert bus.sent[-1].data == struct.pack(">i", -95000)
