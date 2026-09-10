"""Bind an :class:`ArmCommandSession` to the arm UI's callback surface.

The arm tabs report intent through ``ArmUiCallbacks`` and nothing else; this
module fills that structure with bound session methods, so the UI keeps knowing
nothing about ops, actions or transports.

It imports ``arm_ui.contracts`` directly rather than the ``arm_ui`` package
root, because the root pulls in the GTK tab modules.  Keeping the import narrow
is what lets this whole package — and its tests — run with no display at all,
which a test asserts.

Capability translation is deliberately identity: the tokens the UI greys out on
and the tokens the policy refuses to build on are the *same strings*, checked by
a test.  Two vocabularies for one fact is how a control ends up enabled for a
command that can never be built.
"""
from __future__ import annotations

from typing import Any, Callable

from ..arm_ui.contracts import (
    ArmUiCallbacks,
    ArmUiState,
    AUTHORITY_GRANTED,
    LINK_LIVE,
    REQUEST_ACTIVE,
)
from . import contract as K
from . import policy as P
from .session import ArmCommandSession, CommandOutcome


def context_from_state(
    state: ArmUiState, *, state_revision: int | None = None,
) -> P.ArmCommandContext:
    """Project the UI state onto the flat snapshot the policy may look at.

    ``state_revision`` comes from the ops channel, not from the arm: it is the
    optimistic-concurrency guard the broker already uses for chassis commands,
    and ``None`` (no ops state) is itself a refusal reason.

    Note what is *not* copied: nothing about widgets, and no last-known values.
    A grant is read from ``granted_mode``, never from ``requested_mode``.
    """
    granted = (
        state.authority.granted_mode
        if state.authority.status == AUTHORITY_GRANTED
        else ""
    )
    return P.ArmCommandContext(
        link_live=state.link.state == LINK_LIVE,
        granted_mode=granted,
        tool_id=state.detected_tool.tool_id,
        tool_kind=state.detected_tool.kind,
        tool_generation=state.detected_tool.generation,
        actuator_ids=tuple(state.detected_tool.actuator_ids),
        capabilities=frozenset(state.capabilities),
        calibration_active=state.calibration.session_active,
        tool_change_active=state.tool_change.status in REQUEST_ACTIVE,
        blocked_reasons=tuple(
            reason.korean for reason in state.block_reasons
        ),
        state_revision=state_revision,
    )


class ArmOpsCallbackAdapter:
    """Produces the ``ArmUiCallbacks`` the arm tabs consume.

    Every callback returns the session's :class:`CommandOutcome` rather than
    swallowing it, so a refusal can be surfaced to the operator instead of
    looking like a button that did nothing.  ``outcome_sink`` receives every
    outcome — accepted or not — and is the intended place to drive the console's
    existing ack/operation-log presentation.
    """

    def __init__(
        self,
        session: ArmCommandSession,
        *,
        outcome_sink: Callable[[CommandOutcome], None] | None = None,
    ) -> None:
        self._session = session
        self._outcome_sink = outcome_sink

    @property
    def session(self) -> ArmCommandSession:
        return self._session

    def _report(self, outcome: CommandOutcome) -> CommandOutcome:
        if self._outcome_sink is not None:
            self._outcome_sink(outcome)
        return outcome

    def _wrap(self, method: Callable[..., CommandOutcome]) -> Callable[..., CommandOutcome]:
        def call(*args: Any) -> CommandOutcome:
            return self._report(method(*args))
        return call

    # --- state plumbing ---------------------------------------------------
    def update_state(
        self, state: ArmUiState, *, state_revision: int | None = None,
    ) -> None:
        """Feed the UI state through to the session.

        Any hold the change invalidates is ended here, which is what makes a
        tool swap or a withdrawn grant produce a stop without the UI having to
        remember to ask for one.
        """
        self._session.update_context(
            context_from_state(state, state_revision=state_revision)
        )

    def blur(self) -> None:
        self._session.blur()

    def on_disconnect(self) -> None:
        self._session.on_disconnect()

    def pump(self) -> tuple[CommandOutcome, ...]:
        outcomes = self._session.pump()
        for outcome in outcomes:
            self._report(outcome)
        return outcomes

    # --- the callback surface --------------------------------------------
    def callbacks(self) -> ArmUiCallbacks:
        """Every arm-UI intent bound to this session.

        The UI still decides what to *offer* — a callback present here does not
        mean the control is enabled, because ``arm_ui.contracts.gate`` also
        requires the capability and the grant.  This structure only decides
        where a pressed control's intent goes.
        """
        session = self._session
        return ArmUiCallbacks(
            request_tool_change=self._wrap(session.request_tool_change),
            request_control_mode=self._wrap(session.request_mode),
            release_control_mode=self._wrap(session.release_mode),
            # Axis selection is a console-side view change: it moves no joint
            # and therefore has no ops action.  Left unbound on purpose.
            select_axis=None,
            jog_start=self._wrap(
                lambda axis, direction: session.press_axis_jog(axis, direction)
            ),
            jog_stop=self._wrap(session.release_axis_jog),
            # Speed is carried on the next jog refresh rather than sent as its
            # own command; until the arm side confirms how speed is expressed
            # there is nothing honest to send.  See BACKEND_REQUIREMENTS.
            change_speed=None,
            stop_motion=self._wrap(session.stop_motion),
            resume_hold=self._wrap(session.resume_hold),
            go_home=self._wrap(session.go_home),
            save_pose=self._wrap(session.save_pose),
            move_to_pose=self._wrap(session.move_to_pose),
            delete_pose=self._wrap(session.delete_pose),
            tool_command=self._wrap(session.tool_command),
            tool_jog_start=self._wrap(session.press_tool_jog),
            tool_jog_stop=self._wrap(session.release_tool_jog),
            arm_calibration_command=self._wrap(
                lambda step, action: session.calibration_command(
                    K.SCOPE_ARM, step, action,
                )
            ),
            tool_calibration_command=self._wrap(
                lambda target, action: session.calibration_command(
                    K.SCOPE_TOOL, target, action,
                )
            ),
            tool_calibration_jog_start=self._wrap(session.press_calibration_jog),
            tool_calibration_jog_stop=self._wrap(session.release_calibration_jog),
        )


UNBOUND_CALLBACKS = ("select_axis", "change_speed")
"""Callbacks this adapter deliberately leaves ``None``.

``select_axis`` is a view change with no ops meaning.  ``change_speed`` has no
confirmed arm-side representation yet, and binding it to a guess would put a
command on the wire whose meaning nobody agreed to.  Both stay disabled in the
UI as a result, which is the honest rendering of "not connected yet".
"""
