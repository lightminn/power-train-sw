"""Actual FSMs plus a fake bus: targets, receiver leases and hot-swap fencing."""
import pytest

from dynamixel_control.tool_manual_control import ToolManualControl
from test_dual_motor_gripper_fsm import ready_fsm
from test_single_motor_gripper_fsm import MockBridge as SingleBridge, profile as single_profile
from dynamixel_control.tool_fsm.single_motor_gripper_fsm import SingleMotorGripperFSM


def fixture():
    fsm, b = ready_fsm()
    b.positions.update({3: 550, 4: 1550})
    b.goal_position = lambda i, value: b.commands.append(('goal', i, value))
    b.tool_fsm = fsm
    b.tool_type = 'dual_motor_gripper'
    b._tool_context_generation = 7
    b.control_mode = 'MANUAL'
    b.control_scope = 'FULL_ROBOT'
    b.read_only = b.emergency_stop_active = b.tool_detached = False
    b.tool_discovered = True
    clock = [100.0]
    control = ToolManualControl(b, clock=lambda: clock[0])
    return control, b, clock


def request(**changes):
    result = dict(command='JOG', tool_type='dual_motor_gripper', tool_context_generation=7,
                  target='both', direction=1, ttl_s=.6, sequence=1, sent_at_s=100.0)
    result.update(changes)
    result.setdefault('valid_until_s', result['sent_at_s'] + result['ttl_s'])
    return result


@pytest.mark.parametrize('target,motor', [('left', 3), ('right', 4)])
def test_side_endpoint_and_jog_touch_only_requested_motor(target, motor):
    control, b, _ = fixture()
    control.handle(request(command='OPEN', target=target))
    assert b.commands[-1][0:2] == ('goal', motor)
    control.handle(request(target=target))
    assert all(row[0:2] == ('goal', motor) for row in b.commands)


def test_pair_jog_soft_release_keeps_torque_and_stops_refresh():
    control, b, _ = fixture()
    control.handle(request())
    assert b.commands[-1] == ('jog', {3: 650, 4: 1650})
    control.handle(request(command='HOLD', sequence=2))
    assert b.commands[-1][0] == 'hold'
    assert b.torque == {3: 1, 4: 1} and not control.leases


def test_receiver_expires_without_gui_stop_and_refresh_extends_lease():
    control, b, clock = fixture()
    control.handle(request())
    clock[0] = 100.4
    control.handle(request(sent_at_s=100.4))
    clock[0] = 100.9
    control.poll()
    assert control.leases
    clock[0] = 101.01
    control.poll()
    assert b.commands[-1][0] == 'hold' and not control.leases


@pytest.mark.parametrize('changes', [dict(tool_context_generation=6), dict(tool_type='cleaner'),
    dict(target='single'), dict(sent_at_s=98.0), dict(ttl_s=99), dict(sequence=True),
    dict(direction=True), dict(ttl_s=float('nan'))])
def test_wrong_or_stale_command_makes_no_write(changes):
    control, b, _ = fixture()
    with pytest.raises((ValueError, RuntimeError)):
        control.handle(request(**changes))
    assert not b.commands


def test_old_stop_cannot_cancel_a_newer_hold_and_duplicate_refresh_is_rejected():
    control, b, _ = fixture()
    control.handle(request(sequence=10))
    before = list(b.commands)
    control.handle(request(command='HOLD', sequence=9))
    assert b.commands == before and control.leases
    with pytest.raises(ValueError, match='out-of-order'):
        control.handle(request(sequence=10))


def test_detach_ends_lease_and_generation_change_never_holds_replacement():
    control, b, _ = fixture()
    control.handle(request())
    b.tool_detached = True
    control.poll()
    assert not control.leases and b.commands[-1][0] == 'hold'
    b._tool_context_generation += 1
    before = list(b.commands)
    control.poll()
    assert b.commands == before


def test_single_gripper_uses_bounded_steps_and_soft_hold():
    control, b, clock = fixture()
    single = SingleBridge(position=2000, torque=1)
    b.tool_fsm = SingleMotorGripperFSM(single_profile(), single)
    b.tool_fsm.startup()
    b.tool_type = 'spur_1motor_gripper'
    control.handle(request(tool_type=b.tool_type, target='single'))
    assert single.writes[-1] == ('goal', 5, 2100)
    clock[0] = 100.7
    control.poll()
    assert single.writes[-1] == ('goal', 5, 2000)
    assert single.torque == 1


def test_new_authenticated_connection_can_restart_sequence_without_old_stop_winning():
    control, b, clock = fixture()
    control.handle(request(sequence=999, source_id='old'))
    clock[0] = 100.2
    control.handle(request(sequence=1, sent_at_s=100.2, source_id='new'))
    before = list(b.commands)
    control.handle(request(command='HOLD', sequence=1000, sent_at_s=100.1, source_id='old'))
    assert b.commands == before and control.leases


def test_explicit_soft_stop_works_without_a_lease_or_manual_grant():
    control, b, _ = fixture()
    b.control_mode = 'FSM'
    b.hold_tool_position = lambda ids: (b.commands.append(('hold', ids)) or True)
    control.handle(request(command='HOLD'))
    assert b.commands[-1] == ('hold', (3, 4))
    assert b.torque == {3: 1, 4: 1}


def test_dds_queue_delay_cannot_renew_an_expired_jog():
    control, b, clock = fixture()
    clock[0] = 100.7
    with pytest.raises(ValueError, match='expired in ROS queue'):
        control.handle(request(valid_until_s=100.6))
    assert not b.commands and not control.leases
