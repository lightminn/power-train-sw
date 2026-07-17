"""§2.2b 저속 마찰/코깅 보상 torque_ff — Set_Input_Vel 2번째 필드 검증."""
import struct

import pytest

from corner_module.drive_odrive_can import DriveOdriveCan

_SET_INPUT_VEL = 0x0D


class FakeCanBus:
    def __init__(self):
        self.sent = []

    def recv(self, timeout=0.0):
        return None

    def send(self, message):
        self.sent.append(message)


def _input_vel_frames(bus, node_id):
    out = []
    for m in bus.sent:
        if m.arbitration_id == ((node_id << 5) | _SET_INPUT_VEL) and not m.is_remote_frame:
            out.append(struct.unpack("<ff", bytes(m.data)[0:8]))
    return out


def _drive(**kwargs):
    bus = FakeCanBus()
    drive = DriveOdriveCan(node_id=11, bus=bus, **kwargs)
    return drive, bus


@pytest.mark.parametrize(
    ("target", "expected_ff"),
    [
        (0.3, 0.25),     # knee 아래 전진 → +ff
        (-0.3, -0.25),   # knee 아래 후진 → -ff (부호 추종)
        (0.5, 0.0),      # knee 경계(포함 안 함) → 0
        (0.8, 0.0),      # knee 위 → 0
        (0.0, 0.0),      # 정지 지령 → 정확히 0 (크리프 방지)
    ],
)
def test_tick_applies_friction_ff_only_inside_knee(target, expected_ff):
    drive, bus = _drive(friction_ff=0.25, v_knee=0.5)
    drive.set_velocity(target)
    drive.tick()
    frames = _input_vel_frames(bus, 11)
    assert len(frames) == 1
    vel, ff = frames[0]
    assert vel == pytest.approx(target)
    assert ff == pytest.approx(expected_ff)


def test_default_is_off_and_sends_zero_ff():
    drive, bus = _drive()
    drive.set_velocity(0.3)
    drive.tick()
    assert _input_vel_frames(bus, 11)[0][1] == pytest.approx(0.0)


def test_arm_disarm_estop_always_send_zero_ff():
    drive, bus = _drive(friction_ff=0.25, v_knee=0.5)
    drive.set_velocity(0.3)
    drive.arm()
    drive.disarm()
    drive.estop()
    for _vel, ff in _input_vel_frames(bus, 11):
        assert ff == pytest.approx(0.0)
