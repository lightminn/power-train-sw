"""The command session: operator intent in, authorized ops requests out.

This is the only place an :class:`~contract.ArmOpsRequest` is constructed, and
it constructs none without :mod:`policy` allowing it first.  It owns no socket,
no widget and no clock — the clock is injected so expiry and retry are
deterministic in tests.

Two loops meet here.

*Gestures* arrive from the UI: press, release, click.  A press does not send
anything; it registers a hold with :class:`~jog.JogRegistry`.

*Pump* is called on the caller's timer.  It expires stale holds, asks the
registry what should go out now, re-authorizes each request against the
*current* context (a grant can be withdrawn between the press and the send),
and hands the survivors to the transport.

Context changes are the third input, and they are what turn an external event
into a stop: a new tool generation, a withdrawn grant, a dead link and a
started calibration each end every live hold with the matching reason.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Callable, Mapping

from . import contract as K
from . import jog as J
from . import policy as P
from .transport import ArmTransportError, NullArmTransport


@dataclass(frozen=True)
class CommandOutcome:
    """What happened to one gesture.  Never silently nothing."""

    accepted: bool
    action: str = ""
    request_id: str = ""
    reason: str = ""
    korean: str = ""

    def __bool__(self) -> bool:
        return self.accepted


def _refused(action: str, decision: P.Decision) -> CommandOutcome:
    return CommandOutcome(
        accepted=False, action=action,
        reason=decision.reason, korean=decision.korean,
    )


class ArmCommandSession:
    """Turns operator intent into authorized, transport-gated arm commands."""

    def __init__(
        self,
        *,
        clock: Callable[[], float],
        transport: Any = None,
        registry: J.JogRegistry | None = None,
        context: P.ArmCommandContext | None = None,
        audit_sink: Callable[[P.PolicyAudit], None] | None = None,
    ) -> None:
        self._clock = clock
        # Refusing is the default.  A session built without a transport can
        # still be exercised end to end and will emit nothing.
        self._transport = transport if transport is not None else NullArmTransport()
        self._registry = registry if registry is not None else J.JogRegistry()
        self._context = context or P.ArmCommandContext()
        self._audit_sink = audit_sink
        self._audits: list[P.PolicyAudit] = []
        self._submitted: list[K.ArmOpsRequest] = []

    # --- state ------------------------------------------------------------
    @property
    def context(self) -> P.ArmCommandContext:
        return self._context

    @property
    def registry(self) -> J.JogRegistry:
        return self._registry

    @property
    def audits(self) -> tuple[P.PolicyAudit, ...]:
        return tuple(self._audits)

    @property
    def submitted(self) -> tuple[K.ArmOpsRequest, ...]:
        return tuple(self._submitted)

    def update_context(self, context: P.ArmCommandContext) -> tuple[J.PendingStop, ...]:
        """Adopt a new context and end any hold the change invalidates.

        The checks are ordered by how fundamental the loss is, and only the
        first matching reason is used: a link that died *because* the tool was
        swapped should still read as a tool change.
        """
        previous, self._context = self._context, context
        now = float(self._clock())
        reason = ""
        if not context.link_live and previous.link_live:
            reason = K.STOP_REASON_LINK_LOST
        elif context.tool_generation != previous.tool_generation:
            reason = K.STOP_REASON_TOOL_CHANGED
        elif context.tool_id != previous.tool_id:
            reason = K.STOP_REASON_TOOL_CHANGED
        elif previous.manual_granted and not context.manual_granted:
            reason = K.STOP_REASON_AUTHORITY_LOST
        elif context.calibration_active and not previous.calibration_active:
            reason = K.STOP_REASON_CALIBRATION
        elif context.tool_change_active and not previous.tool_change_active:
            reason = K.STOP_REASON_TOOL_CHANGED
        if not reason:
            return ()
        return self._registry.release_all(now=now, reason=reason)

    def blur(self) -> tuple[J.PendingStop, ...]:
        """The tab went away, focus was lost, or the window closed."""
        return self._registry.release_all(
            now=float(self._clock()), reason=K.STOP_REASON_FOCUS_LOST,
        )

    def on_disconnect(self) -> tuple[J.PendingStop, ...]:
        """The ops connection dropped.

        Both halves matter: the holds end (so a stop intent exists), and the
        outstanding-request slots are freed (so the retry after reconnect is
        not blocked by a request that will never be answered).
        """
        stops = self._registry.release_all(
            now=float(self._clock()), reason=K.STOP_REASON_LINK_LOST,
        )
        self._registry.release_outstanding()
        return stops

    def on_ack(self, ack: K.ArmOpsAck) -> None:
        """Free the outstanding slot for a settled request.

        OUTCOME_UNKNOWN settles too.  Treating an unknown outcome as still
        in flight would wedge the hold on a single lost response, and the
        deadman — not this bookkeeping — is what keeps that safe.
        """
        if not str(ack.status).startswith("FINAL_") and \
                ack.status != "OUTCOME_UNKNOWN":
            return
        for request in reversed(self._submitted):
            if request.action == ack.action:
                self._registry.settle(request)
                return

    # --- one-shot gestures ------------------------------------------------
    def _issue(
        self, action: str, params: Mapping[str, Any] | None = None,
        *, intent: str = "",
    ) -> CommandOutcome:
        params = dict(params or {})
        decision = P.authorize(self._context, action, params)
        if not decision:
            return self._audit(action, decision)
        try:
            request = K.ArmOpsRequest(
                action=action,
                params=params,
                expected_state_revision=self._context.state_revision,
                issued_s=float(self._clock()),
                intent=intent,
            )
        except K.ArmParamsError as exc:
            return self._audit(
                action, P.Decision(False, "invalid_params", str(exc)),
            )
        return self._submit(request)

    def _audit(self, action: str, decision: P.Decision) -> CommandOutcome:
        audit = P.PolicyAudit(
            action=action, reason=decision.reason, korean=decision.korean,
            context=self._context,
        )
        self._audits.append(audit)
        if self._audit_sink is not None:
            self._audit_sink(audit)
        return _refused(action, decision)

    def _submit(self, request: K.ArmOpsRequest) -> CommandOutcome:
        try:
            request_id = self._transport.submit(request)
        except ArmTransportError as exc:
            return self._audit(
                request.action, P.Decision(False, "transport_refused", str(exc)),
            )
        except Exception as exc:                       # transport bug or socket error
            return self._audit(
                request.action, P.Decision(False, "transport_error", str(exc)),
            )
        self._submitted.append(request)
        return CommandOutcome(
            accepted=True, action=request.action, request_id=str(request_id),
        )

    def request_tool_change(self, kind: str) -> CommandOutcome:
        return self._issue(
            K.ACTION_TOOL_CHANGE,
            {
                "requested_kind": str(kind),
                "observed_tool_generation": self._context.tool_generation,
            },
            intent="도구 변경 요청",
        )

    def request_mode(self, mode: str) -> CommandOutcome:
        return self._issue(
            K.ACTION_MODE_REQUEST, {"mode": str(mode)}, intent="제어권 요청",
        )

    def release_mode(self) -> CommandOutcome:
        return self._issue(K.ACTION_MODE_RELEASE, intent="제어권 반납")

    def stop_motion(self) -> CommandOutcome:
        """The operator's explicit stop.

        It also ends every live hold, so a stop cannot leave a jog refreshing
        behind it.  The stop action itself is never blocked by policy.
        """
        self._registry.release_all(
            now=float(self._clock()), reason=K.STOP_REASON_EXPLICIT,
        )
        return self._issue(K.ACTION_STOP, intent="정지")

    def resume_hold(self) -> CommandOutcome:
        return self._issue(K.ACTION_RESUME_HOLD, intent="복귀")

    def go_home(self) -> CommandOutcome:
        return self._issue(K.ACTION_HOME, intent="home")

    def save_pose(self, name: str) -> CommandOutcome:
        return self._issue(
            K.ACTION_POSE_SAVE, {"name": str(name)}, intent="자세 저장")

    def move_to_pose(self, name: str) -> CommandOutcome:
        return self._issue(
            K.ACTION_POSE_MOVE, {"name": str(name)}, intent="자세 이동")

    def delete_pose(self, name: str) -> CommandOutcome:
        return self._issue(
            K.ACTION_POSE_DELETE, {"name": str(name)}, intent="자세 삭제")

    def tool_command(self, target: str, command: str) -> CommandOutcome:
        # In explicit bench mode, STOP is allowed to release torque and the
        # next OPEN/CLOSE should be usable without a second operator ritual.
        # Queue the profile-bound enable first; the broker serializes both
        # mutations, while the arm FSM remains the final motion gate.
        if self._context.developer_mode \
                and str(command).lower() in (K.COMMAND_OPEN, K.COMMAND_CLOSE):
            enabled = self.set_tool_enabled(True)
            if not enabled:
                return enabled
        return self._issue(
            K.ACTION_TOOL_COMMAND,
            {
                "target": str(target),
                "command": str(command),
                **self._tool_identity(),
            },
            intent=f"도구 {command}",
        )

    def set_tool_enabled(self, enabled: bool) -> CommandOutcome:
        return self._issue(
            K.ACTION_TOOL_ENABLE,
            {"enabled": bool(enabled), **self._tool_identity()},
            intent="도구 토크 활성화" if enabled else "도구 토크 해제",
        )

    def calibration_command(
        self, scope: str, step: str, action: str,
    ) -> CommandOutcome:
        params: dict[str, Any] = {
            "scope": str(scope), "step": str(step), "action": str(action),
        }
        if scope == K.SCOPE_TOOL:
            params.update(self._tool_identity())
        return self._issue(
            K.ACTION_CALIBRATION, params, intent=f"보정 {scope}/{step}/{action}",
        )

    def _tool_identity(self) -> dict[str, Any]:
        return {
            "tool_id": self._context.tool_id,
            "tool_generation": self._context.tool_generation,
        }

    # --- holds -------------------------------------------------------------
    def _press(
        self, channel: J.JogChannel, direction: int, *, speed_level: int | None = None,
    ) -> CommandOutcome:
        decision = P.authorize(
            self._context, channel.action, channel.subject_params,
        )
        if not decision:
            return self._audit(channel.action, decision)
        self._registry.press(
            channel, direction=direction, now=float(self._clock()),
            speed_level=speed_level,
        )
        # A press is not a send: the request goes out on the next pump, after
        # being re-authorized against the context as it is *then*.
        return CommandOutcome(accepted=True, action=channel.action)

    def _release(self, channel: J.JogChannel) -> CommandOutcome:
        stop = self._registry.release(
            channel, now=float(self._clock()), reason=K.STOP_REASON_EXPLICIT,
        )
        if stop is None:
            return CommandOutcome(
                accepted=False, action=channel.stop_action,
                reason="not_held", korean="진행 중인 조그가 없습니다",
            )
        return CommandOutcome(accepted=True, action=channel.stop_action)

    def press_axis_jog(
        self, axis: int, direction: int, *, speed_level: int | None = None,
    ) -> CommandOutcome:
        return self._press(
            J.arm_axis_channel(axis), direction, speed_level=speed_level,
        )

    def release_axis_jog(self, axis: int) -> CommandOutcome:
        return self._release(J.arm_axis_channel(axis))

    def press_tool_jog(self, target: str, direction: int) -> CommandOutcome:
        return self._press(self._tool_channel(target), direction)

    def release_tool_jog(self, target: str) -> CommandOutcome:
        return self._release(self._tool_channel(target))

    def press_calibration_jog(self, target: str, direction: int) -> CommandOutcome:
        return self._press(self._calibration_channel(target), direction)

    def release_calibration_jog(self, target: str) -> CommandOutcome:
        return self._release(self._calibration_channel(target))

    def _tool_channel(self, target: str) -> J.JogChannel:
        return J.tool_channel(
            target, self._context.tool_id, self._context.tool_generation,
        )

    def _calibration_channel(self, target: str) -> J.JogChannel:
        return J.calibration_channel(
            target, self._context.tool_id, self._context.tool_generation,
        )

    # --- the timer half ----------------------------------------------------
    def pump(self) -> tuple[CommandOutcome, ...]:
        """Expire stale holds and send whatever is due.  Call on a timer.

        Every request is re-authorized here rather than trusting the check made
        at press time, because the context can change in between — and a stop
        is exempt from that re-check, so a withdrawn grant can never strand a
        moving joint.
        """
        now = float(self._clock())
        self._registry.expire(now)
        outcomes: list[CommandOutcome] = []

        # Stops first, and unconditionally: they are exempt from policy and
        # they hold the single serialised slot ahead of any refresh.
        for stop in self._registry.pending_stops:
            self._registry.record_stop_attempt(stop.channel)
            request = stop.request()
            outcome = self._submit(self._with_revision(request))
            outcomes.append(outcome)
            if outcome.accepted:
                self._registry.settle(request)
        if outcomes:
            # One mutation is in flight; refreshes wait for the next pump.
            return tuple(outcomes)

        for request in self._registry.due(now):
            decision = P.authorize(self._context, request.action, request.params)
            if not decision:
                outcomes.append(self._audit(request.action, decision))
                # An unauthorized refresh must not leave the hold alive: end
                # every hold so a stop intent exists for it.
                self._registry.release_all(
                    now=now, reason=K.STOP_REASON_AUTHORITY_LOST,
                )
                continue
            outcomes.append(self._submit(self._with_revision(request)))
        return tuple(outcomes)

    def _with_revision(self, request: K.ArmOpsRequest) -> K.ArmOpsRequest:
        """Stamp the current ops state revision onto a registry-built request."""
        if request.expected_state_revision is not None:
            return request
        return replace(
            request, expected_state_revision=self._context.state_revision,
        )

    @property
    def pending_stop_count(self) -> int:
        """Stops decided but not yet confirmed delivered."""
        return len(self._registry.pending_stops)
