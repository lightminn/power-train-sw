"""Lock panel, telemetry freshness and the authenticated command boundary."""
from dataclasses import replace
import time

import pytest

from operator_console.arm_ui import contracts as C, fixtures
from operator_console.arm_ui.manual_tab import ArmManualTab
from operator_console.arm_ops import contract as K
from operator_console.arm_ops.adapter import ArmOpsCallbackAdapter
from operator_console.arm_ops.session import ArmCommandSession
from operator_console.arm_ops.transport import RecordingArmTransport
from operator_console.arm_ui_binding import ArmUiTelemetryBinding
from operator_console.arm_telemetry import parse_arm_telemetry
from powertrain_ros.arm_console_mirror import build_arm_telemetry_payload
from powertrain_observability.tool_snapshot import validate_lock_command


def lock_state(**updates):
    lock = dict(enabled=True, state='LOCKED', zero_tick=2048, zero_state='LOCKED',
                position=2048, torque=1, sequence_state='IDLE', coupling_proxy='DISABLED')
    lock.update(updates)
    state = fixtures.single_gripper()
    return replace(state, coupling_lock=lock,
                   capabilities=state.capabilities | {C.CAP_COUPLING_LOCK})


def test_real_widget_click_builds_only_the_registered_lock_command():
    transport = RecordingArmTransport()
    adapter = ArmOpsCallbackAdapter(ArmCommandSession(clock=time.monotonic, transport=transport))
    tab = ArmManualTab(adapter.callbacks())
    try:
        state = lock_state()
        adapter.update_state(state, state_revision=7)
        tab.update_state(state)
        panel = tab.coupling_lock
        panel.direction.set_active_id('-1')
        panel.buttons['detach'].clicked()
        request, = transport.accepted
        assert request.action == K.ACTION_LOCK_COMMAND
        assert request.params == dict(command='CHANGE', to='UNLOCKED', direction=-1)
        assert request.expected_state_revision == 7
        # Command submission must never turn into an optimistic completion.
        assert panel.metrics['state'].value.get_text() == '결합 위치'
    finally:
        tab.dispose()


@pytest.mark.parametrize('change', [
    dict(link=C.SourceLink(state=C.LINK_STALE)),
    dict(capabilities=frozenset(), developer_mode=True),
    dict(authority=C.ControlAuthority(), developer_mode=True),
    dict(coupling_lock={}),
    dict(coupling_lock=dict(enabled=True, state='MOVING', torque=1)),
])
def test_motion_blocked_and_stop_remains_available(change):
    calls = []
    tab = ArmManualTab(C.ArmUiCallbacks(coupling_lock_command=lambda *args: calls.append(args)))
    try:
        tab.update_state(replace(lock_state(), **change))
        panel = tab.coupling_lock
        assert not panel.buttons['detach'].get_sensitive()
        panel.buttons['detach'].clicked()
        assert not calls
        assert panel.buttons['stop'].get_sensitive()
        panel.buttons['stop'].clicked()
        assert calls == [('STOP', {})]
    finally:
        tab.dispose()


def test_missing_source_and_callback_never_enable_a_lock_control():
    tab = ArmManualTab()
    try:
        assert not any(w.get_sensitive() for w in tab.coupling_lock.buttons.values())
    finally:
        tab.dispose()


def test_zero_capture_requires_torque_off_and_enable_requires_a_zero():
    tab = ArmManualTab(C.ArmUiCallbacks(coupling_lock_command=lambda *_: None))
    try:
        tab.update_state(lock_state())
        panel = tab.coupling_lock
        assert not panel.buttons['zero_locked'].get_sensitive()
        tab.update_state(lock_state(torque=0, zero_tick=None, state='UNCALIBRATED'))
        assert panel.buttons['zero_locked'].get_sensitive()
        assert not panel.buttons['enable'].get_sensitive()
        tab.update_state(lock_state(torque=0))
        assert panel.buttons['enable'].get_sensitive()
    finally:
        tab.dispose()


@pytest.mark.parametrize('age,scope,read_only,ops,expected', [
    (0., 'END_EFFECTOR_ONLY', False, True, True),
    (2., 'END_EFFECTOR_ONLY', False, True, False),
    (0., 'FULL_ROBOT', False, True, False),
    (0., 'TOOL_CHANGE', False, True, False),
    (0., 'END_EFFECTOR_ONLY', True, True, False),
    (0., 'END_EFFECTOR_ONLY', False, False, False),
])
def test_udp_projection_does_not_invent_lock_permission(age, scope, read_only, ops, expected):
    payload = build_arm_telemetry_payload(sequence=1, stamp_s=0, motors=None,
        joints=None, source_age_s={}, tool_runtime={
            'source_age_s': 0., 'tool': dict(tool_type='spur_1motor_gripper',
                actuator_ids=[5], actuators=[], read_only=read_only, emergency_stop=False,
                control_scope=scope, control_mode='MANUAL'),
            'coupling_lock': lock_state().coupling_lock, 'coupling_lock_age_s': age},
        arm_runtime=dict(control_mode='MANUAL', source_age_s={'control_mode': 0.}))
    snapshot = parse_arm_telemetry(payload)
    state = ArmUiTelemetryBinding().state(snapshot, ops_link_ready=ops)
    assert (C.CAP_COUPLING_LOCK in state.capabilities) is expected
    if age > 1:
        assert state.coupling_lock == {}


@pytest.mark.parametrize('bad', [
    dict(command='CHANGE', to='LOCKED', direction=True),
    dict(command='CHANGE', to='LOCKED', direction=0),
    dict(command='SET_ZERO', state='UNKNOWN'),
    dict(command='ENABLE', actuator_id=1), dict(command='LOCK'), dict(command=[]),
])
def test_ops_lock_vocabulary_rejects_raw_or_ambiguous_commands(bad):
    with pytest.raises(ValueError):
        validate_lock_command(bad)


def test_command_policy_keeps_lock_capability_and_manual_grant_in_developer_mode():
    from operator_console.arm_ops import policy as P
    for context in (
        P.ArmCommandContext(link_live=True, state_revision=1, developer_mode=True,
                            granted_mode='MANUAL'),
        P.ArmCommandContext(link_live=True, state_revision=1, developer_mode=True,
                            capabilities=frozenset({C.CAP_COUPLING_LOCK})),
    ):
        result = P.authorize(context, K.ACTION_LOCK_COMMAND, {'command': 'ENABLE'})
        assert not result


def test_session_rejects_incomplete_lock_payload_without_transporting_it():
    from operator_console.arm_ops import policy as P
    transport = RecordingArmTransport()
    session = ArmCommandSession(clock=time.monotonic, transport=transport,
        context=P.ArmCommandContext(link_live=True, state_revision=1,
            granted_mode='MANUAL', capabilities=frozenset({C.CAP_COUPLING_LOCK})))
    outcome = session.coupling_lock_command('CHANGE', {'to': 'LOCKED'})
    assert not outcome and outcome.reason == 'invalid_params'
    assert not transport.accepted


def test_pressed_wrist_jog_is_refreshed_only_while_active_and_permitted():
    calls = []
    from operator_console.arm_ui.keyboard_panel import KeyboardTeleopPanel
    callbacks = C.ArmUiCallbacks(jog_start=lambda *args: calls.append(args),
                                jog_stop=lambda *_args: None)
    panel = KeyboardTeleopPanel(callbacks)
    state = fixtures.single_gripper()
    state = replace(state, teleop=replace(state.teleop, selected_axis=5),
                    capabilities=state.capabilities | {C.CAP_TELEOP_JOG})
    try:
        panel.set_active(True)
        panel.update(state)
        panel._jog_plus.emit('pressed')
        panel.update(state)
        assert calls == [(5, 1), (5, 1)]
        panel._jog_plus.emit('released')
        panel.update(state)
        assert len(calls) == 2
    finally:
        panel.dispose()
