"""Raw AK clockwise-positive angles must not leak into the wheel frame."""
import struct

import can
import pytest

from chassis.chassis_manager import build_real_corners, WheelMap
from corner_module.steer_ak40 import SteerAk40


class Bus:
    def __init__(self):
        self.sent = []
        self.rx = []

    def send(self, message, timeout=None):
        self.sent.append(message)

    def recv(self, timeout=0.0):
        return self.rx.pop(0) if self.rx else None


def _status(motor_id, raw_deg):
    return can.Message(
        arbitration_id=(41 << 8) | motor_id, is_extended_id=True,
        data=struct.pack(">hhhbb", int(raw_deg * 10), 0, 0, 25, 0))


def _raw_target(bus):
    message = bus.sent[-1]
    assert message.is_extended_id
    assert message.arbitration_id >> 8 == 6
    return struct.unpack(">ihh", message.data)[0] / 10000.0


@pytest.mark.parametrize("invert,raw_target,wheel_feedback", [
    (True, -30.0, 12.5), (False, 30.0, -12.5),
])
def test_command_and_feedback_cross_the_same_sign_boundary(
        monkeypatch, invert, raw_target, wheel_feedback):
    bus = Bus()
    monkeypatch.setattr(can.interface, "Bus", lambda **kwargs: bus)
    steer = SteerAk40(motor_id=3, invert=invert)
    steer.connect()
    steer.set_angle(30.0)
    bus.rx.append(_status(3, -12.5))

    steer.tick()

    assert _raw_target(bus) == raw_target
    assert steer.health_state()["target_deg"] == 30.0
    assert steer.health_state()["actual_deg"] == wheel_feedback


def test_inverted_arm_preserves_raw_position_and_estop_still_sends_zero(monkeypatch):
    bus = Bus()
    monkeypatch.setattr(can.interface, "Bus", lambda **kwargs: bus)
    steer = SteerAk40(motor_id=4, invert=True)
    steer.connect()
    bus.rx.append(_status(4, 17.0))

    steer.arm()
    assert bus.sent == []
    steer.tick()

    assert _raw_target(bus) == 17.0  # No jump to the negated physical pose.
    assert steer.health_state()["target_deg"] == -17.0
    assert steer.health_state()["actual_deg"] == -17.0
    steer.estop()
    assert bus.sent[-1].arbitration_id == (3 << 8) | 4
    assert struct.unpack(">i", bus.sent[-1].data)[0] == 0


@pytest.mark.parametrize("wheel,motor_id", [
    ("front_left", 1), ("front_right", 2),
    ("rear_left", 3), ("rear_right", 4), ("front_left", 21),
])
def test_real_builder_applies_measured_clockwise_mounting_to_all_steering(
        monkeypatch, wheel, motor_id):
    bus = Bus()
    monkeypatch.setattr(can.interface, "Bus", lambda **kwargs: bus)
    corners = build_real_corners(wheel_map=[WheelMap(wheel, motor_id, 101)])
    steer = corners[wheel].steer
    steer.connect()
    steer.set_angle(30.0)  # Wheel front edge must turn to body left.
    steer.tick()

    assert _raw_target(bus) == -30.0
    assert bus.sent[-1].arbitration_id == (6 << 8) | motor_id
