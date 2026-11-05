"""분리/교체 전에 기존 도구의 정지 실패를 확인하는 ROS 어댑터 검증."""

import json
import threading
from types import SimpleNamespace

from std_msgs.msg import String

from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge
from dynamixel_control.toolchanger_lock import ToolchangerLock
from dynamixel_control.toolchanger_sequence import ToolchangerSequence
from test_toolchanger_lock import IO


def _bridge(torque_state, zero_state='LOCKED', scope='END_EFFECTOR_ONLY'):
    """노드 생성 없이 잠금 명령 경로만 호출하는 최소 대역."""
    io = IO()
    lock = ToolchangerLock(io)
    lock.capture_zero(zero_state)
    lock.enable()
    replies = []
    bridge = SimpleNamespace(
        _bus_lock=threading.RLock(), manual_coupling_lock=False,
        toolchanger=lock, read_only=False, emergency_stop_active=False,
        control_scope=scope, control_mode='MANUAL', mock_mode=True,
        tool_discovered=True,
        tool_ids=[5], _tool_samples={5: {'torque_state': torque_state}},
        _stop_tool=lambda reason: None,
        get_logger=lambda: SimpleNamespace(warn=lambda message: None),
        toolchanger_status_pub=SimpleNamespace(publish=lambda message:
                                               replies.append(json.loads(message.data))))
    bridge._toolchanger_command_locked = lambda msg: MoveItDynamixelBridge._toolchanger_command_locked(bridge, msg)
    bridge._prepare_tool_for_lock = lambda: MoveItDynamixelBridge._prepare_tool_for_lock(bridge)
    bridge._toolchanger_snapshot = lambda: MoveItDynamixelBridge._toolchanger_snapshot(bridge)
    bridge.lock_sequence = ToolchangerSequence(lock, bridge._prepare_tool_for_lock)
    return bridge, io, replies


def _send(bridge, payload):
    MoveItDynamixelBridge._toolchanger_command(bridge, String(data=json.dumps(payload)))


def test_failed_tool_stop_prevents_lock_motor_goal():
    bridge, io, replies = _bridge('ON')
    before = list(io.writes)
    _send(bridge, {'command': 'UNLOCK', 'direction': 1})
    assert io.writes == before
    assert replies[-1]['state'] == 'LOCKED'
    assert 'OFF' in replies[-1]['command_error']


def test_change_sequence_requires_confirmed_tool_stop():
    bridge, io, replies = _bridge('ON', zero_state='UNLOCKED')
    before = list(io.writes)
    _send(bridge, {'command': 'CHANGE', 'to': 'LOCKED', 'direction': 1})
    assert io.writes == before
    assert 'OFF' in replies[-1]['command_error']
    assert replies[-1]['sequence_state'] == 'IDLE'


def test_change_sequence_is_rejected_outside_end_effector_bench():
    bridge, io, replies = _bridge('OFF', zero_state='UNLOCKED', scope='FULL_ROBOT')
    before = list(io.writes)
    _send(bridge, {'command': 'CHANGE', 'to': 'LOCKED', 'direction': 1})
    assert io.writes == before
    assert 'MANUAL END_EFFECTOR_ONLY' in replies[-1]['command_error']
    assert replies[-1]['sequence_state'] == 'IDLE'


def test_unknown_change_target_is_rejected_without_lock_write():
    bridge, io, replies = _bridge('OFF', zero_state='UNLOCKED')
    before = list(io.writes)
    _send(bridge, {'command': 'CHANGE', 'to': 'HALF', 'direction': 1})
    assert io.writes == before
    assert 'LOCKED or UNLOCKED' in replies[-1]['command_error']


def test_opt_in_full_robot_manual_lock_requires_stationary_arm():
    bridge, io, replies = _bridge('OFF', scope='FULL_ROBOT')
    bridge.manual_coupling_lock = True
    checked = []
    bridge._require_manual_lock_arm_stopped = lambda: checked.append(True)
    bridge._poll_toolchanger = lambda: None
    _send(bridge, {'command': 'CHANGE', 'to': 'UNLOCKED', 'direction': -1})
    assert checked == [True]
    assert bridge.lock_sequence.state == 'MOVING'


def test_moving_arm_rejects_lock_without_motor_write():
    bridge, io, replies = _bridge('OFF', scope='FULL_ROBOT')
    bridge.manual_coupling_lock = True
    def reject():
        raise RuntimeError('arm must be stationary')
    bridge._require_manual_lock_arm_stopped = reject
    before = list(io.writes)
    _send(bridge, {'command': 'CHANGE', 'to': 'UNLOCKED', 'direction': -1})
    assert io.writes == before
    assert 'stationary' in replies[-1]['command_error']


def test_stationary_gate_reads_live_arm_velocity_error_and_pending_goal():
    from dynamixel_control.toolchanger_lock import LockError
    import pytest
    registers = {70: 0, 128: 0, 132: 2525, 116: 2525}
    bridge = SimpleNamespace(mock_mode=False, active_ids={16},
                             _bus_lock=threading.RLock(),
                             _read_register=lambda motor, address, *args, **kwargs: registers[address])
    check = lambda: MoveItDynamixelBridge._require_manual_lock_arm_stopped(bridge)
    check()
    for address, value in ((70, 32), (128, 1), (116, 2538)):
        original = registers[address]
        registers[address] = value
        with pytest.raises(LockError, match='stationary'):
            check()
        registers[address] = original
    bridge.active_ids = set()
    with pytest.raises(LockError, match='no active arm'):
        check()
