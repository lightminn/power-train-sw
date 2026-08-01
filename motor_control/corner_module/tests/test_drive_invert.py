"""ODrive 미러 장착 축의 모터↔바퀴 프레임 부호 변환 검증."""
import struct

import can
import pytest

from corner_module.drive_odrive_can import DriveOdriveCan

_GET_ENCODER_ESTIMATES = 0x09
_SET_INPUT_VEL = 0x0D


class FakeCanBus:
    def __init__(self, rx=()):
        self.sent = []
        self._rx = list(rx)

    def recv(self, timeout=0.0):
        return self._rx.pop(0) if self._rx else None

    def send(self, message):
        self.sent.append(message)


def _encoder(node_id, velocity):
    return can.Message(
        arbitration_id=(node_id << 5) | _GET_ENCODER_ESTIMATES,
        data=struct.pack("<ff", 0.0, velocity),
        is_extended_id=False,
    )


def _input_vel(bus, node_id):
    arbitration_id = (node_id << 5) | _SET_INPUT_VEL
    frames = [
        message
        for message in bus.sent
        if message.arbitration_id == arbitration_id
        and not message.is_remote_frame
    ]
    assert len(frames) == 1
    return struct.unpack("<ff", bytes(frames[0].data))


def test_inverted_drive_sends_negative_motor_velocity_for_forward_wheel_command():
    bus = FakeCanBus()
    drive = DriveOdriveCan(node_id=12, bus=bus, gear_ratio=5.0, invert=True)

    drive.set_velocity(2.0)
    drive.tick()

    motor_tps, _torque_ff = _input_vel(bus, 12)
    assert motor_tps == pytest.approx(-10.0)


def test_inverted_drive_keeps_target_velocity_in_wheel_frame():
    drive = DriveOdriveCan(bus=FakeCanBus(), gear_ratio=5.0, invert=True)

    drive.set_velocity(2.0)
    drive.tick()

    assert drive.state()["target_vel"] == pytest.approx(2.0)


def test_inverted_drive_restores_motor_feedback_to_wheel_frame():
    node_id = 12
    bus = FakeCanBus(rx=[_encoder(node_id, velocity=5.0)])
    drive = DriveOdriveCan(
        node_id=node_id,
        bus=bus,
        gear_ratio=5.0,
        invert=True,
    )

    drive.tick()

    assert drive.state()["actual_vel"] == pytest.approx(-1.0)


def test_friction_feedforward_follows_inverted_motor_direction():
    inverted_bus = FakeCanBus()
    normal_bus = FakeCanBus()
    inverted = DriveOdriveCan(
        bus=inverted_bus,
        gear_ratio=5.0,
        friction_ff=0.25,
        v_knee=0.5,
        invert=True,
    )
    normal = DriveOdriveCan(
        bus=normal_bus,
        gear_ratio=5.0,
        friction_ff=0.25,
        v_knee=0.5,
        invert=False,
    )

    inverted.set_velocity(0.05)
    normal.set_velocity(0.05)
    inverted.tick()
    normal.tick()

    assert _input_vel(inverted_bus, 11)[1] == pytest.approx(-0.25)
    assert _input_vel(normal_bus, 11)[1] == pytest.approx(0.25)


def test_default_noninverted_drive_preserves_existing_command_sign():
    bus = FakeCanBus()
    drive = DriveOdriveCan(node_id=11, bus=bus, gear_ratio=5.0)

    drive.set_velocity(2.0)
    drive.tick()

    assert _input_vel(bus, 11)[0] == pytest.approx(10.0)
    assert drive.invert is False


def test_invert_property_is_read_only_and_not_added_to_state_schema():
    drive = DriveOdriveCan(bus=FakeCanBus(), invert=True)

    assert drive.invert is True
    assert "invert" not in drive.state()
    with pytest.raises(AttributeError):
        drive.invert = False
