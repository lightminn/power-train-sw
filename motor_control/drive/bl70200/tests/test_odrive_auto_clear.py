"""Field auto-clear only touches idle MOTOR_FAILED axes."""
import struct

import can

from drive.bl70200.odrive_auto_clear import AutoClear


def heartbeat(node, axis_error, state, tail=b"\x00\x00\x00"):
    return can.Message(arbitration_id=(node << 5) | 0x01, is_extended_id=False,
                       data=struct.pack("<IB", axis_error, state) + tail)


def test_idle_motor_failed_is_cleared_once_per_cooldown():
    policy = AutoClear(cooldown_s=1.0)
    out = policy.on_frame(heartbeat(15, 0x40, 1), 10.0)
    assert [(m.arbitration_id, m.is_remote_frame) for m in out] == [
        ((15 << 5) | 0x03, True), ((15 << 5) | 0x18, False)]
    assert bytes(out[1].data) == bytes(8)
    assert policy.on_frame(heartbeat(15, 0x40, 1), 10.5) == []
    assert len(policy.on_frame(heartbeat(15, 0x40, 1), 11.1)) == 2


def test_other_errors_states_and_nodes_are_left_alone():
    policy = AutoClear()
    assert policy.on_frame(heartbeat(15, 0, 1), 0.0) == []
    assert policy.on_frame(heartbeat(15, 0x40, 8), 0.0) == []          # closed loop
    assert policy.on_frame(heartbeat(15, 0x4000, 1), 0.0) == []        # ESTOP_REQUESTED
    assert policy.on_frame(heartbeat(15, 0x4040, 1), 0.0) == []
    assert policy.on_frame(heartbeat(15, 0x100, 1), 0.0) == []         # encoder
    assert policy.on_frame(heartbeat(3, 0x40, 1), 0.0) == []           # steering id range
    assert policy.on_frame(heartbeat(15, 0x40, 1, b"\xA3\x01\x02"), 0.0) == []  # recovery latch
    assert len(policy.on_frame(heartbeat(15, 0x40, 1, b"\xA3\x00\x02"), 0.0)) == 2
