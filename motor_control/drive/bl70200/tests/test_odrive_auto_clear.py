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


def motor_error(node, value):
    return can.Message(arbitration_id=(node << 5) | 0x03, is_extended_id=False,
                       data=struct.pack("<Q", value))


def demo_policy():
    return AutoClear(mask=0x40 | 0x800, motor_allow=0x1000, cooldown_s=0.0)


def kinds(frames):
    return [(m.arbitration_id & 0x1F, m.is_remote_frame) for m in frames]


def test_motor_allow_queries_first_and_clears_only_after_allowed_reply_and_next_heartbeat():
    policy = demo_policy()
    assert kinds(policy.on_frame(heartbeat(15, 0x40, 1), 10.0)) == [(0x03, True)]
    assert policy.on_frame(motor_error(15, 0x1000), 10.05) == []
    out = policy.on_frame(heartbeat(15, 0x40, 1), 10.1)
    assert kinds(out) == [(0x18, False)]
    assert bytes(out[0].data) == bytes(8)


def test_motor_allow_refuses_motor_errors_outside_the_allowlist():
    policy = demo_policy()
    policy.on_frame(heartbeat(15, 0x40, 1), 10.0)
    policy.on_frame(motor_error(15, 0x1000 | 0x2), 10.05)   # phase resistance etc.
    assert 0x18 not in [k for k, _ in kinds(policy.on_frame(heartbeat(15, 0x40, 1), 10.1))]


def test_motor_allow_drops_the_query_when_the_axis_changes_or_reply_is_late():
    policy = demo_policy()
    policy.on_frame(heartbeat(15, 0x40, 1), 10.0)
    policy.on_frame(motor_error(15, 0x1000), 10.05)
    assert kinds(policy.on_frame(heartbeat(15, 0x40 | 0x100, 1), 10.1)) == []  # new bit
    policy.on_frame(heartbeat(15, 0x40, 1), 20.0)
    policy.on_frame(motor_error(15, 0x1000), 20.5)                               # too late
    assert 0x18 not in [k for k, _ in kinds(policy.on_frame(heartbeat(15, 0x40, 1), 20.6))]


def test_motor_allow_accepts_watchdog_expiry_with_no_motor_error():
    policy = demo_policy()
    policy.on_frame(heartbeat(13, 0x800, 1), 1.0)
    policy.on_frame(motor_error(13, 0), 1.05)
    assert kinds(policy.on_frame(heartbeat(13, 0x800, 1), 1.1)) == [(0x18, False)]


def test_motor_allow_limits_clears_per_axis_window():
    policy = demo_policy()
    now = 0.0
    cleared = 0
    for _ in range(5):
        policy.on_frame(heartbeat(15, 0x40, 1), now)
        policy.on_frame(motor_error(15, 0x1000), now + 0.05)
        cleared += kinds(policy.on_frame(heartbeat(15, 0x40, 1), now + 0.1)).count((0x18, False))
        now += 1.0
    assert cleared == 3
