"""The coupling locks itself after a tool attach, and only when that is safe."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace

SOURCE = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'


class LockError(RuntimeError):
    pass


def method(name='_run_auto_lock'):
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'MoveItDynamixelBridge')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
              and n.name == name)
    namespace = dict(json=json, LockError=LockError,
                     String=lambda data: SimpleNamespace(data=data))
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace[name]


def bridge(**overrides):
    sent = []

    def clear():
        return None

    fields = dict(
        _auto_lock_pending=True, read_only=False, control_mode='MANUAL',
        emergency_stop_active=False, toolchanger=SimpleNamespace(state='UNLOCKED', sample={}),
        _tc_running=lambda: False, _require_no_manual_arm_command=clear,
        get_logger=lambda: SimpleNamespace(info=lambda _m: None),
        _toolchanger_command=lambda msg: sent.append(json.loads(msg.data)))
    fields.update(overrides)
    return SimpleNamespace(**fields), sent


def test_attach_locks_the_coupling_once():
    node, sent = bridge()
    method()(node)
    method()(node)
    assert sent == [{'command': 'CHANGE', 'to': 'LOCKED', 'direction': 1}]
    assert node._auto_lock_pending is False


def test_nothing_happens_without_an_attach_edge():
    node, sent = bridge(_auto_lock_pending=False)
    method()(node)
    assert sent == []


def test_it_waits_for_manual_and_e_stop_then_locks():
    node, sent = bridge(control_mode='FSM')
    method()(node)
    assert sent == [] and node._auto_lock_pending is True
    node.control_mode, node.emergency_stop_active = 'MANUAL', True
    method()(node)
    assert sent == [] and node._auto_lock_pending is True
    node.emergency_stop_active = False
    method()(node)
    assert len(sent) == 1


def test_it_waits_while_the_arm_is_moving():
    def busy():
        raise LockError('wait')

    node, sent = bridge(_require_no_manual_arm_command=busy)
    method()(node)
    assert sent == [] and node._auto_lock_pending is True


def test_it_waits_for_a_silent_lock_servo():
    node, sent = bridge(toolchanger=SimpleNamespace(state='FAULT', sample=None))
    method()(node)
    assert sent == [] and node._auto_lock_pending is True
    node.toolchanger.state, node.toolchanger.sample = 'BETWEEN', {}
    method()(node)
    assert len(sent) == 1


def test_a_slot_exchange_owns_the_lock():
    node, sent = bridge(_tc_running=lambda: True)
    method()(node)
    assert sent == [] and node._auto_lock_pending is False


def test_an_already_locked_coupling_is_left_alone():
    node, sent = bridge(toolchanger=SimpleNamespace(state='LOCKED', sample={}))
    method()(node)
    assert sent == [] and node._auto_lock_pending is False


def poll_node(detected, **overrides):
    fields = dict(
        _bus_tool_identity=SimpleNamespace(
            supported_tool_types=('spur_1motor_gripper',), last_reason='', last_present_ids=()),
        port_connected=True, tool_type='spur_1motor_gripper',
        _observe_active_tool_signature=lambda: detected,
        _rescan_physical_tool=lambda: detected,
        _confirmed_tool_observation=lambda _d: True,
        _physical_tool_detached=True, tool_auto_lock_on_attach=True,
        _auto_lock_armed=True, _auto_lock_pending=False,
        emergency_stop_active=True)  # returns just past the attach decision
    fields.update(overrides)
    return SimpleNamespace(**fields)


def test_a_tool_id_arms_the_lock_before_the_profile_check():
    node = poll_node('dual_motor_gripper')
    method('_poll_physical_tool_once')(node)
    assert node._auto_lock_pending is True and node._auto_lock_armed is False


def test_a_rejected_tool_does_not_relock_after_a_manual_unlock():
    node = poll_node('dual_motor_gripper', _auto_lock_armed=False)
    method('_poll_physical_tool_once')(node)
    assert node._auto_lock_pending is False


def test_pulling_the_tool_rearms_and_cancels():
    node = poll_node(None, _auto_lock_armed=False, _auto_lock_pending=True)
    method('_poll_physical_tool_once')(node)
    assert node._auto_lock_pending is False and node._auto_lock_armed is True


def test_a_bridge_restart_with_another_tool_fitted_does_not_arm():
    node = poll_node(
        'dual_motor_gripper', _physical_tool_detached=False, _auto_lock_armed=False,
        _tool_change_lock=SimpleNamespace(acquire=lambda blocking: True, release=lambda: None),
        tool_discovered=True, _stop_tool=lambda _r: None,
        _mark_tool_feedback_offline=lambda: None, _clear_runtime_sessions=lambda: None,
        _publish_tool_status_safely=lambda: None)
    method('_poll_physical_tool_once')(node)
    assert node._physical_tool_detached is True and node._auto_lock_armed is False


def test_an_empty_coupling_arms():
    node = poll_node(
        None, _physical_tool_detached=False, _auto_lock_armed=False,
        _tool_change_lock=SimpleNamespace(acquire=lambda blocking: True, release=lambda: None),
        tool_discovered=True, _stop_tool=lambda _r: None,
        _mark_tool_feedback_offline=lambda: None, _clear_runtime_sessions=lambda: None,
        _publish_tool_status_safely=lambda: None)
    method('_poll_physical_tool_once')(node)
    assert node._auto_lock_armed is True
