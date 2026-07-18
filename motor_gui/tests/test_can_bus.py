import struct

import can
import pytest

from motor_gui.backend.transport.can_bus import (
    CanBackend,
    C_GET_ENC_EST,
    C_SET_INPUT_VEL,
    C_SET_LIMITS,
    C_SET_TRAJ_VEL_LIMIT,
    C_SET_TRAJ_ACCEL_LIMITS,
)


class StubBus:
    def __init__(self):
        self.sent = []

    def send(self, msg, timeout=None):
        self.sent.append(msg)

    def recv(self, timeout=None):
        return None


class StubAk:
    pos_out_deg = 12.0
    spd_erpm = 34.0
    cur_a = 1.5
    temp_c = 42
    fault = 0

    def poll(self, timeout=None):
        return None


def test_legacy_can_backend_converts_odrive_velocity_but_not_ak():
    backend = CanBackend(gear_ratio=5.0)
    backend._bus = StubBus()
    backend._ak = StubAk()

    backend._apply_odrive("set_input", {"vel": 1.0})
    sent = backend._bus.sent[-1]
    motor_vel, _tff = struct.unpack("<ff", sent.data)
    msg = can.Message(
        arbitration_id=(backend._node << 5) | C_GET_ENC_EST,
        data=struct.pack("<ff", 2.0, 10.0),
        is_extended_id=False,
    )
    backend._decode_odrive(msg)
    sample = backend.sample()

    assert (sent.arbitration_id & 0x1F) == C_SET_INPUT_VEL
    assert motor_vel == 5.0
    assert sample["odrive.vel"] == 2.0
    assert sample["ak.speed"] == 34.0
    assert backend.capabilities()["drive_gear_ratio"] == 5.0


def test_legacy_can_backend_rejects_nonpositive_ratio():
    with pytest.raises(ValueError, match="gear_ratio must be finite and positive"):
        CanBackend(gear_ratio=0.0)


def test_legacy_can_backend_converts_wheel_velocity_tunables_to_motor_units():
    backend = CanBackend(gear_ratio=5.0)
    backend._bus = StubBus()

    backend._apply_odrive("set_limit", {"vel_limit": 10.0, "current_lim": 9.0})
    limit = backend._bus.sent[-1]
    assert (limit.arbitration_id & 0x1F) == C_SET_LIMITS
    assert struct.unpack("<ff", limit.data) == (50.0, 9.0)

    backend._apply_odrive("set_gain", {"trap_vel_limit": 4.0})
    trap_vel = backend._bus.sent[-1]
    assert (trap_vel.arbitration_id & 0x1F) == C_SET_TRAJ_VEL_LIMIT
    assert struct.unpack("<f", trap_vel.data)[0] == 20.0

    backend._apply_odrive("set_gain", {
        "trap_accel_limit": 2.0,
        "trap_decel_limit": 3.0,
    })
    trap_accel = backend._bus.sent[-1]
    assert (trap_accel.arbitration_id & 0x1F) == C_SET_TRAJ_ACCEL_LIMITS
    assert struct.unpack("<ff", trap_accel.data) == (10.0, 15.0)
    assert backend.read_tunables()["vel_limit"] == 10.0
    assert backend.capabilities()["limits"]["odrive"]["vel"] == 40.0
