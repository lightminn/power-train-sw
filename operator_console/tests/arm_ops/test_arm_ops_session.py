"""The session: gestures in, authorized and transport-gated requests out."""
from __future__ import annotations

import pytest

from operator_console.arm_ops import contract as K
from operator_console.arm_ops import jog as J
from operator_console.arm_ops import policy as P
from operator_console.arm_ops.session import ArmCommandSession
from operator_console.arm_ops.transport import (
    ArmActionNotRegistered,
    ContractGatedTransport,
    NullArmTransport,
    RecordingArmTransport,
    registered_actions_from,
)

from test_arm_ops_policy import ALL_CAPS, granted_context


class FakeClock:
    def __init__(self, start: float = 100.0) -> None:
        self.now = float(start)

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> float:
        self.now += float(seconds)
        return self.now


def build(**overrides):
    """A session whose transport accepts everything, for behaviour tests."""
    clock = FakeClock()
    transport = RecordingArmTransport()
    session = ArmCommandSession(
        clock=clock,
        transport=transport,
        context=overrides.pop("context", granted_context()),
        registry=J.JogRegistry(ttl_s=1.0, refresh_interval_s=0.2),
        **overrides,
    )
    return clock, transport, session


# --- the default session sends nothing --------------------------------------
def test_a_session_with_no_transport_refuses_every_command():
    session = ArmCommandSession(clock=FakeClock(), context=granted_context())

    outcome = session.stop_motion()

    assert not outcome
    assert outcome.reason == "transport_refused"
    assert "연결되지 않았습니다" in outcome.korean


def test_the_default_transport_is_the_refusing_one():
    session = ArmCommandSession(clock=FakeClock())

    assert isinstance(session._transport, NullArmTransport)


def test_a_default_context_session_refuses_everything_but_reports_why():
    _clock, _transport, session = build(context=P.ArmCommandContext())

    outcome = session.go_home()

    assert not outcome
    assert outcome.reason == "capability_missing"
    assert session.audits[-1].action == K.ACTION_HOME


# --- transport gate ---------------------------------------------------------
def test_the_contract_gate_refuses_actions_the_ops_contract_does_not_know():
    transport = ContractGatedTransport(
        submit_fn=lambda *args: "should-not-happen",
        registered_actions=frozenset({"estop", "arm"}),
    )

    with pytest.raises(ArmActionNotRegistered, match="등록되지 않았습니다"):
        transport.submit(K.ArmOpsRequest(action=K.ACTION_STOP))

    assert transport.refusals[0][0] == K.ACTION_STOP
    assert set(transport.missing_actions()) == K.ARM_ACTIONS


def test_the_contract_gate_passes_a_registered_action_through():
    sent = []
    transport = ContractGatedTransport(
        submit_fn=lambda action, params, revision: sent.append(
            (action, params, revision)
        ) or "req-1",
        registered_actions=frozenset({K.ACTION_STOP}),
    )

    request_id = transport.submit(
        K.ArmOpsRequest(action=K.ACTION_STOP, expected_state_revision=5)
    )

    assert request_id == "req-1"
    assert sent == [(K.ACTION_STOP, {}, 5)]


def test_registered_actions_can_be_read_off_the_real_ops_contract_module():
    """Reading the real module proves the arm actions really are absent."""
    import importlib.util
    from pathlib import Path

    path = (
        Path(__file__).resolve().parents[3] / "ros2" / "src" / "powertrain_ros"
        / "powertrain_ros" / "ops_contract.py"
    )
    spec = importlib.util.spec_from_file_location("_ops_contract_probe", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    registered = registered_actions_from(module)

    assert "estop" in registered
    assert {K.ACTION_MODE_REQUEST, K.ACTION_TOOL_COMMAND} <= registered
    transport = ContractGatedTransport(
        submit_fn=lambda *args: "no", registered_actions=registered,
    )
    for action in sorted(K.ARM_ACTIONS):
        assert transport.known(action) == (action in {
            K.ACTION_MODE_REQUEST, K.ACTION_TOOL_COMMAND,
        })


def test_a_transport_error_is_reported_not_swallowed():
    clock = FakeClock()
    transport = RecordingArmTransport(fail_with=RuntimeError("socket gone"))
    session = ArmCommandSession(
        clock=clock, transport=transport, context=granted_context(),
    )

    outcome = session.stop_motion()

    assert not outcome
    assert outcome.reason == "transport_error"
    assert "socket gone" in outcome.korean


# --- one-shot commands ------------------------------------------------------
def test_an_allowed_command_reaches_the_transport_with_the_state_revision():
    _clock, transport, session = build()

    outcome = session.request_mode(K.MODE_MANUAL)

    assert outcome
    request, = transport.accepted
    assert request.action == K.ACTION_MODE_REQUEST
    assert request.params == {"mode": K.MODE_MANUAL}
    assert request.expected_state_revision == 42


def test_a_refused_command_never_reaches_the_transport():
    _clock, transport, session = build(
        context=granted_context(capabilities=frozenset()),
    )

    assert not session.request_mode(K.MODE_MANUAL)
    assert transport.accepted == []


def test_a_tool_command_carries_the_identity_it_was_issued_against():
    _clock, transport, session = build()

    session.tool_command("single", "open")

    request, = transport.accepted
    assert request.params["tool_id"] == "TOOL-S1"
    assert request.params["tool_generation"] == 7


def test_invalid_params_are_refused_as_a_policy_audit_not_an_exception():
    _clock, transport, session = build()

    outcome = session.request_mode("AUTO")

    assert not outcome
    assert outcome.reason == "invalid_params"
    assert transport.accepted == []


# --- holds ------------------------------------------------------------------
def test_a_press_sends_nothing_until_the_next_pump():
    clock, transport, session = build()

    assert session.press_axis_jog(2, 1)
    assert transport.accepted == []

    session.pump()
    assert transport.actions == (K.ACTION_JOG,)


def test_a_press_without_a_grant_never_starts_a_hold():
    clock, transport, session = build(
        context=granted_context(granted_mode=""),
    )

    outcome = session.press_axis_jog(2, 1)

    assert not outcome
    assert outcome.reason == "manual_not_granted"
    assert session.registry.active_channels == ()
    session.pump()
    assert transport.accepted == []


def test_a_grant_withdrawn_between_press_and_pump_stops_instead_of_sending():
    clock, transport, session = build()
    session.press_axis_jog(2, 1)

    session.update_context(granted_context(granted_mode=""))
    clock.advance(0.1)
    session.pump()

    # The hold ended and a stop was produced; no jog refresh went out.
    assert K.ACTION_JOG not in transport.actions
    assert K.ACTION_JOG_STOP in transport.actions


def test_release_sends_the_stop_on_the_next_pump():
    clock, transport, session = build()
    session.press_axis_jog(2, 1)
    session.pump()
    transport.accepted.clear()

    session.release_axis_jog(2)
    session.pump()

    request, = transport.accepted
    assert request.action == K.ACTION_JOG_STOP
    assert request.params["reason"] == K.STOP_REASON_EXPLICIT


def test_stops_are_sent_before_any_refresh():
    clock, transport, session = build()
    session.press_axis_jog(2, 1)
    session.press_tool_jog("left", 1)
    session.release_axis_jog(2)

    session.pump()

    # With a serialising broker the stop must not queue behind a refresh.
    assert all(action in K.STOP_ACTIONS for action in transport.actions)


def test_an_expired_hold_stops_itself():
    clock, transport, session = build()
    session.press_axis_jog(2, 1)
    session.pump()
    transport.accepted.clear()

    clock.advance(2.0)                 # past the 1.0 s TTL
    session.pump()

    assert transport.actions == (K.ACTION_JOG_STOP,)
    assert transport.accepted[0].params["reason"] == K.STOP_REASON_EXPIRED


def test_explicit_stop_motion_also_ends_every_hold():
    clock, transport, session = build()
    session.press_axis_jog(2, 1)
    session.press_tool_jog("left", 1)

    session.stop_motion()

    assert session.registry.active_channels == ()
    assert session.pending_stop_count == 2


# --- external events leave a stop intent ------------------------------------
@pytest.mark.parametrize("mutate,expected", [
    (lambda ctx: granted_context(link_live=False), K.STOP_REASON_LINK_LOST),
    (lambda ctx: granted_context(tool_generation=99), K.STOP_REASON_TOOL_CHANGED),
    (lambda ctx: granted_context(tool_id="TOOL-OTHER"), K.STOP_REASON_TOOL_CHANGED),
    (lambda ctx: granted_context(granted_mode=""), K.STOP_REASON_AUTHORITY_LOST),
    (lambda ctx: granted_context(calibration_active=True),
     K.STOP_REASON_CALIBRATION),
    (lambda ctx: granted_context(tool_change_active=True),
     K.STOP_REASON_TOOL_CHANGED),
])
def test_a_context_change_ends_holds_with_the_matching_reason(mutate, expected):
    clock, _transport, session = build()
    session.press_axis_jog(2, 1)

    stops = session.update_context(mutate(session.context))

    assert [stop.reason for stop in stops] == [expected]
    assert session.registry.active_channels == ()


def test_leaving_the_tab_ends_holds():
    clock, _transport, session = build()
    session.press_axis_jog(2, 1)

    stops = session.blur()

    assert [stop.reason for stop in stops] == [K.STOP_REASON_FOCUS_LOST]


def test_an_unchanged_context_does_not_manufacture_a_stop():
    clock, _transport, session = build()
    session.press_axis_jog(2, 1)

    assert session.update_context(granted_context()) == ()
    assert session.registry.active_channels != ()


def test_a_stop_that_cannot_be_delivered_stays_pending_and_is_retried():
    clock = FakeClock()
    transport = RecordingArmTransport()
    session = ArmCommandSession(
        clock=clock, transport=transport, context=granted_context(),
        registry=J.JogRegistry(ttl_s=1.0, refresh_interval_s=0.2),
    )
    session.press_axis_jog(2, 1)

    # The link dies: the hold ends, and the stop cannot go out.
    transport.fail_with = RuntimeError("connection lost")
    session.on_disconnect()
    session.pump()

    assert session.pending_stop_count == 1
    assert transport.accepted == []

    # Reconnected: the same stop intent is delivered, not forgotten.
    transport.fail_with = None
    session.pump()

    assert transport.actions == (K.ACTION_JOG_STOP,)
    assert transport.accepted[0].params["reason"] == K.STOP_REASON_LINK_LOST
    assert session.pending_stop_count == 0


def test_disconnect_frees_outstanding_slots_so_the_retry_is_not_blocked():
    clock, transport, session = build()
    session.press_axis_jog(2, 1)
    session.pump()                       # one jog outstanding, never answered
    transport.accepted.clear()

    session.on_disconnect()
    session.pump()

    assert transport.actions == (K.ACTION_JOG_STOP,)


# --- acks -------------------------------------------------------------------
def test_an_unknown_outcome_frees_the_slot_rather_than_wedging_the_hold():
    clock, transport, session = build()
    session.press_axis_jog(2, 1)
    session.pump()
    transport.accepted.clear()

    session.on_ack(K.ArmOpsAck(
        request_id="r1", status="OUTCOME_UNKNOWN", action=K.ACTION_JOG,
    ))
    clock.advance(0.3)
    session.pump()

    assert transport.actions == (K.ACTION_JOG,)


def test_a_pending_ack_does_not_free_the_slot():
    clock, transport, session = build()
    session.press_axis_jog(2, 1)
    session.pump()
    transport.accepted.clear()

    session.on_ack(K.ArmOpsAck(
        request_id="r1", status="PENDING", action=K.ACTION_JOG,
    ))
    clock.advance(0.3)
    session.pump()

    assert transport.accepted == []


# --- calibration ------------------------------------------------------------
def test_a_calibration_session_blocks_a_new_jog_press():
    clock, _transport, session = build(
        context=granted_context(calibration_active=True),
    )

    outcome = session.press_axis_jog(2, 1)

    assert not outcome
    assert outcome.reason == "calibration_active"


def test_calibration_commands_name_their_scope():
    _clock, transport, session = build()

    session.calibration_command(K.SCOPE_ARM, "gear_ratio", "measure")

    request, = transport.accepted
    assert request.action == K.ACTION_CALIBRATION
    assert request.params["scope"] == K.SCOPE_ARM
    assert "tool_id" not in request.params


def test_tool_scope_calibration_carries_the_tool_identity():
    _clock, transport, session = build()

    session.calibration_command(K.SCOPE_TOOL, "single", "start")

    request, = transport.accepted
    assert request.params["tool_id"] == "TOOL-S1"
    assert request.params["tool_generation"] == 7


# --- audit ------------------------------------------------------------------
def test_every_refusal_is_recorded_with_its_context():
    audits = []
    clock = FakeClock()
    session = ArmCommandSession(
        clock=clock, transport=RecordingArmTransport(),
        context=granted_context(granted_mode=""),
        audit_sink=audits.append,
    )

    session.press_axis_jog(2, 1)

    assert len(audits) == 1
    assert audits[0].reason == "manual_not_granted"
    assert audits[0].context.granted_mode == ""
