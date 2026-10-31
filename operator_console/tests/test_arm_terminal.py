"""Headless commands, key-repeat rate, loss of authority and shutdown."""
from dataclasses import replace
import subprocess
import sys
from types import SimpleNamespace

import pytest

from operator_console import arm_terminal as T
from operator_console.arm_ops import contract as K
from operator_console.arm_ui import contracts as C


@pytest.fixture
def client(monkeypatch, tmp_path):
    clock = [100.]
    monkeypatch.setattr(T.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(T.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    flags = dict(K.BENCH_JOG_FLAGS)
    K.BENCH_JOG_FLAGS.clear()

    class Ops:
        connected = True
        closed = False

        def __init__(self, *args, submit_sink, schedule, **kwargs):
            self.calls = []
            self.sink, self.schedule = submit_sink, schedule

        def submit(self, action, params, revision):
            request = str(len(self.calls))
            self.calls.append((action, params, revision))
            self.schedule(lambda: self.sink(dict(request_id=request,
                status='FINAL_SUCCESS', detail='published')))
            return request

        def latest_state(self): return {'revision': 7}
        def close(self): self.closed = True

    class Receiver:
        closed = False
        def __init__(self, _port, *, exclusive): assert exclusive
        def latest(self): return None
        def close(self): self.closed = True

    monkeypatch.setattr(T, 'ConsoleOpsClient', Ops)
    monkeypatch.setattr(T, 'LatestArmTelemetryReceiver', Receiver)
    token = tmp_path / 'token'
    token.write_text('test-token')
    instance = T.ArmTerminalClient('localhost', 1234, token)
    state = [replace(C.default_state(),
        link=C.SourceLink(state=C.LINK_LIVE),
        detected_tool=C.ToolIdentity(kind='dual_gripper',
            tool_id='dual_motor_gripper:3,4', generation=3, actuator_ids=(3, 4)),
        authority=C.ControlAuthority(granted_mode='MANUAL', status=C.AUTHORITY_GRANTED),
        capabilities=frozenset({'control_mode', 'coupling_lock', 'teleop_jog',
            'teleop_pose', 'tool_enable', 'tool_change', 'gripper_command',
            'tool_jog', 'dual_sync', 'individual_targets'}))]
    instance._binding = SimpleNamespace(state=lambda *args, **kwargs: state[0])
    instance.tick()
    yield instance, clock, state
    instance.close()
    K.BENCH_JOG_FLAGS.clear()
    K.BENCH_JOG_FLAGS.update(flags)


def test_import_and_help_need_no_gui_ros_or_driver():
    result = subprocess.run([sys.executable, '-c', '''
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] in ('gi', 'rclpy', 'dynamixel_sdk'):
        raise AssertionError(name)
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
from operator_console.arm_terminal import main
main(['--help'])
'''], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert '--token-file' in result.stdout


@pytest.mark.parametrize('command,action,params', [
    ('lock', K.ACTION_LOCK_COMMAND, dict(command='CHANGE', to='LOCKED', direction=1)),
    ('unlock', K.ACTION_LOCK_COMMAND, dict(command='CHANGE', to='UNLOCKED', direction=-1)),
    ('lock -1', K.ACTION_LOCK_COMMAND, dict(command='CHANGE', to='LOCKED', direction=-1)),
    ('zero locked', K.ACTION_LOCK_COMMAND, dict(command='SET_ZERO', state='LOCKED')),
    ('lock-disable', K.ACTION_LOCK_STOP, {}),
    ('save home', K.ACTION_POSE_SAVE, dict(name='home')),
    ('move home', K.ACTION_POSE_MOVE, dict(name='home')),
    ('delete home', K.ACTION_POSE_DELETE, dict(name='home')),
    ('freedrive', K.ACTION_FREEDRIVE, {}),
    ('release', K.ACTION_FREEDRIVE_RELEASE, {}),
    ('capture 1 dual above', K.ACTION_SLOT_CAPTURE,
        dict(slot='1', tool_type='dual_motor_gripper', pose='above')),
    ('exchange dual dry-run', K.ACTION_EXCHANGE_START,
        dict(tool_type='dual_motor_gripper', dry_run=True)),
])
def test_commands_use_existing_contract(client, command, action, params):
    c, _, _ = client
    assert not c._ops.calls  # Merely connecting never enables torque or requests MANUAL.
    assert c.command(command)
    assert c._ops.calls[-1] == (action, params, 7)


def test_release_timeout_ends_jog_and_never_sends_another_refresh(client):
    c, clock, _ = client
    c.jog(2, 1)
    c.tick()
    clock[0] += .151
    c.tick()
    assert not c.session.registry.active_channels
    assert c._ops.calls[-1][0] == K.ACTION_JOG_STOP
    count = len(c._ops.calls)
    clock[0] += 2
    c.tick()
    assert len(c._ops.calls) == count


def test_auto_repeat_does_not_increase_motor_refresh_rate(client):
    c, clock, _ = client
    for _ in range(50):
        c.jog(1, 1)
        c.tick()
        clock[0] += .02
    jogs = [p for action, p, _ in c._ops.calls if action == K.ACTION_JOG]
    assert 4 <= len(jogs) <= 6
    assert all(p['expires_at_s'] - 100 <= 1.6 for p in jogs)


def test_lost_authority_requires_a_new_press_after_a_release(client):
    c, clock, state = client
    c.jog(3, 1)
    c.tick()
    state[0] = replace(state[0], authority=C.ControlAuthority())
    c.tick()
    state[0] = replace(state[0], authority=C.ControlAuthority(
        granted_mode='MANUAL', status=C.AUTHORITY_GRANTED))
    c.tick()
    count = len([1 for a, *_ in c._ops.calls if a == K.ACTION_JOG])
    for _ in range(10):
        clock[0] += .02
        c.jog(3, 1)
        c.tick()
    assert len([1 for a, *_ in c._ops.calls if a == K.ACTION_JOG]) == count
    clock[0] += .16
    c.tick()
    c.jog(3, 1)
    c.tick()
    assert len([1 for a, *_ in c._ops.calls if a == K.ACTION_JOG]) == count + 1


def test_stale_state_refuses_motion_but_still_sends_stop(client):
    c, _, state = client
    state[0] = replace(state[0], link=C.SourceLink(state=C.LINK_STALE))
    c.tick()
    assert not c.command('lock')
    assert not c._ops.calls
    assert c.command('lock-stop')


def test_shutdown_sends_stops_and_closes_resources(client):
    c, _, _ = client
    c.jog(1, 1)
    c.tick()
    c.close()
    actions = {a for a, *_ in c._ops.calls}
    assert {K.ACTION_JOG_STOP, K.ACTION_LOCK_STOP, K.ACTION_EXCHANGE_STOP, K.ACTION_STOP} <= actions
    assert c._ops.closed and c._receiver.closed


def test_tool_selection_and_torque_never_happen_on_implicit_input(client):
    c, _, _ = client
    c.command('target left')
    c.command('limits off')
    c.command('trip off')
    assert c.target == 'left' and not c._ops.calls
    assert c.command('tool open left')
    action, params, _ = c._ops.calls[-1]
    assert action == K.ACTION_TOOL_COMMAND
    assert params == dict(target='left', command='open',
        tool_id='dual_motor_gripper:3,4', tool_generation=3)


def test_unknown_command_does_not_send_anything(client):
    c, _, _ = client
    with pytest.raises(ValueError):
        c.command('exchange dual move-anyway')
    assert not c._ops.calls


def test_shutdown_reports_rejected_stop_instead_of_claiming_success(client, capsys):
    c, _, _ = client
    c.command('lock-stop')
    request_id = next(iter(c._pending))
    c._ack(dict(request_id=request_id, status='FINAL_REJECTED', detail='fixture rejection'))
    c.close()
    assert '정지 ACK 미확인' in capsys.readouterr().err


def test_exclusive_telemetry_refuses_to_share_a_gui_port():
    from operator_console.arm_telemetry import LatestArmTelemetryReceiver
    existing = LatestArmTelemetryReceiver(0)
    try:
        with pytest.raises(OSError):
            LatestArmTelemetryReceiver(existing._socket.getsockname()[1], exclusive=True)
    finally:
        existing.close()


def test_tool_only_shutdown_does_not_send_an_unsupported_slot_action(client, capsys):
    c, _, state = client
    state[0] = replace(state[0], capabilities=state[0].capabilities - {'teleop_pose'})
    c.tick()
    c.close()
    assert K.ACTION_EXCHANGE_STOP not in {a for a, *_ in c._ops.calls}
    assert not capsys.readouterr().err


@pytest.mark.parametrize('axis', range(1, 6))
def test_all_five_axes_keep_the_existing_bench_switches_and_speed(client, axis):
    c, _, _ = client
    c.command('limits off')
    c.command('trip on')
    c.session.change_speed(2)
    c.jog(axis, -1)
    c.tick()
    action, params, _ = c._ops.calls[-1]
    assert action == K.ACTION_JOG
    assert (params['axis'], params['direction'], params['speed_level']) == (axis, -1, 7)
    assert params['limits'] is False and params['trip'] is True


def test_axis_switch_stops_the_previous_axis_before_the_new_jog(client):
    c, _, _ = client
    c.jog(1, 1)
    c.tick()
    c.jog(5, -1)
    c.tick()
    assert c._ops.calls[-1][0] == K.ACTION_JOG_STOP
    assert c._ops.calls[-1][1]['axis'] == 1
    c.tick()
    assert c._ops.calls[-1][0] == K.ACTION_JOG
    assert c._ops.calls[-1][1]['axis'] == 5


def test_ops_disconnect_ends_a_hold_even_with_live_udp(client):
    c, clock, _ = client
    c.jog(1, 1)
    c.tick()
    c._ops.connected = False
    c.tick()
    assert not c.session.context.link_live
    assert c._needs_release
    c._ops.connected = True
    c.tick()
    c.jog(1, 1)
    assert c._jog_axis is None
    clock[0] += .16
    c.tick()
    c.jog(1, 1)
    assert c._jog_axis == 1
