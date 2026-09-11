"""The safety decision: may this arm command exist at all?

This module is pure — no GTK, no sockets, no clock of its own — so every rule
below is directly testable and none of them can be bypassed by a UI change.
It is the *second* gate, not the only one: ``arm_ui.contracts.gate`` already
greys out controls the operator must not press.  That one is presentation and
can be defeated by a code path that never consults it; this one decides whether
an :class:`~operator_console.arm_ops.contract.ArmOpsRequest` may be constructed,
so a command that violates a rule here has no object to travel in.

The rules, in the order they are applied:

1. **A capability the backend never reported means the command cannot be
   built.**  Not disabled, not queued — refused at construction.
2. **A stop is always allowed to be built.**  Every other rule can block a
   command; none of them may block ending motion, because a stop that needs
   permission is not a stop.
3. **The feed must be LIVE.**  A stale snapshot is not evidence of the present.
4. **Requesting MANUAL is not holding it.**  Only a reported grant unlocks
   motion; the request that asks for the grant stays available so the operator
   can ever obtain one.
5. **A calibration session owns the arm.**  General jog and tool change are
   blocked for its duration.
6. **A tool change in flight blocks tool-scoped commands.**
7. **A tool-scoped command must name the tool it was issued against**, and that
   tool must still be the active one.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from . import contract as K


# Capability tokens.  These are the same strings ``arm_ui.contracts`` uses, and
# a test pins them together so the UI and the command path can never disagree
# about what the backend said it supports.
CAP_TOOL_CHANGE = "tool_change"
CAP_CONTROL_MODE = "control_mode"
CAP_TELEOP_JOG = "teleop_jog"
CAP_TELEOP_POSE = "teleop_pose"
CAP_GRIPPER_COMMAND = "gripper_command"
CAP_DUAL_SYNC = "dual_sync"
CAP_ARM_CALIBRATION = "arm_calibration"
CAP_TOOL_CALIBRATION = "tool_calibration"
CAP_DUAL_TOOL_CALIBRATION = "dual_tool_calibration"

# Which capability each action needs.  A stop action needs none: see rule 2.
ACTION_CAPABILITY: dict[str, str] = {
    K.ACTION_TOOL_CHANGE: CAP_TOOL_CHANGE,
    K.ACTION_MODE_REQUEST: CAP_CONTROL_MODE,
    K.ACTION_MODE_RELEASE: CAP_CONTROL_MODE,
    K.ACTION_JOG: CAP_TELEOP_JOG,
    K.ACTION_RESUME_HOLD: CAP_TELEOP_JOG,
    K.ACTION_HOME: CAP_TELEOP_POSE,
    K.ACTION_POSE_SAVE: CAP_TELEOP_POSE,
    K.ACTION_POSE_MOVE: CAP_TELEOP_POSE,
    K.ACTION_POSE_DELETE: CAP_TELEOP_POSE,
    K.ACTION_TOOL_COMMAND: CAP_GRIPPER_COMMAND,
    K.ACTION_TOOL_ENABLE: "tool_enable",
    K.ACTION_TOOL_JOG: CAP_GRIPPER_COMMAND,
    K.ACTION_CALIBRATION: CAP_ARM_CALIBRATION,
    K.ACTION_CALIBRATION_JOG: CAP_TOOL_CALIBRATION,
}


@dataclass(frozen=True)
class ArmCommandContext:
    """Everything the policy is allowed to look at, and nothing else.

    Deliberately a flat, pure snapshot rather than the UI state object: the
    policy must be testable without GTK, and it must be impossible to make a
    decision from a widget's current appearance.

    **Every default is the unsafe-to-act value.**  A default-constructed
    context permits nothing but stops, which is the correct behaviour for
    "the console has not heard from the arm".
    """

    link_live: bool = False
    granted_mode: str = ""
    tool_id: str = ""
    tool_kind: str = ""
    tool_generation: int = 0
    actuator_ids: tuple[int, ...] = ()
    capabilities: frozenset[str] = frozenset()
    calibration_active: bool = False
    tool_change_active: bool = False
    blocked_reasons: tuple[str, ...] = ()
    # The ops state revision this context was taken at, used as the optimistic
    # concurrency guard on the wire.  ``None`` means the ops channel had no
    # state, which is itself a reason to refuse.
    state_revision: int | None = None
    developer_mode: bool = False

    @property
    def manual_granted(self) -> bool:
        return self.granted_mode == K.MODE_MANUAL


@dataclass(frozen=True)
class Decision:
    """Allowed, or refused with a reason an operator can read."""

    allowed: bool
    reason: str = ""
    # Korean text suitable for a tooltip or the operation log.
    korean: str = ""

    def __bool__(self) -> bool:
        return self.allowed


ALLOWED = Decision(True)


def _refuse(reason: str, korean: str) -> Decision:
    return Decision(False, reason, korean)


def authorize(
    context: ArmCommandContext,
    action: str,
    params: Mapping[str, Any] | None = None,
) -> Decision:
    """Decide whether ``action`` may be built in ``context``.

    Order matters: the most fundamental obstacle is reported, so an operator
    with no reported capability is told that, not "no control authority".
    """
    if action not in K.ARM_ACTIONS:
        return _refuse("unknown_action", f"알 수 없는 팔 명령: {action}")

    params = dict(params or {})

    # Rule 2 first: a stop must not be gated on anything below.
    if action in K.STOP_ACTIONS:
        return _stop_decision(context, action, params)

    # Rule 1.
    capability = ACTION_CAPABILITY.get(action, "")
    if capability and capability not in context.capabilities \
            and not context.developer_mode:
        return _refuse(
            "capability_missing",
            "지원하지 않음 — 백엔드가 이 기능을 보고하지 않음",
        )
    if action == K.ACTION_CALIBRATION:
        extra = _calibration_capability(context, params)
        if extra is not None:
            return extra

    # Rule 3.
    if not context.link_live:
        return _refuse("link_not_live", "상태 미수신 — 명령을 만들 수 없음")
    if context.state_revision is None:
        return _refuse("no_ops_state", "ops 상태 미수신 — 명령을 만들 수 없음")

    # Rule 4.  The mode request itself is exempt, or the grant is unreachable.
    if action in K.MOTION_ACTIONS and not context.manual_granted \
            and not context.developer_mode:
        return _refuse("manual_not_granted", "수동 제어권 미승인")

    # Rule 5.
    if context.calibration_active and action not in K.CALIBRATION_ACTIONS:
        return _refuse(
            "calibration_active", "캘리브레이션 진행 중 — 일반 조작 차단",
        )

    # Rule 6.
    if context.tool_change_active and action != K.ACTION_TOOL_CHANGE:
        return _refuse("tool_change_active", "도구 교체 중 — 조작 차단")

    # Rule 7.
    if action in K.TOOL_SCOPED_ACTIONS:
        mismatch = _tool_identity_decision(context, params)
        if mismatch is not None:
            return mismatch

    # Tool change is also the bridge's existing re-scan/re-initialization
    # operation.  Let it observe a disconnected/faulted tool so it can report
    # fresh hardware state; it never enables torque or commands motion.
    if context.blocked_reasons and action not in {
            K.ACTION_TOOL_CHANGE, K.ACTION_MODE_REQUEST, K.ACTION_MODE_RELEASE,
    }:
        return _refuse("backend_blocked", context.blocked_reasons[0])

    return ALLOWED


def _stop_decision(
    context: ArmCommandContext,
    action: str,
    params: Mapping[str, Any],
) -> Decision:
    """Stops are permitted even with no capability, no grant and no link.

    The one thing still checked is that a *tool-scoped* stop names a tool: a
    stop addressed at nothing is not safer than no stop, it just hides that
    nothing was addressed.  It is intentionally NOT checked against the current
    generation — stopping the tool that was just swapped away is exactly what
    has to happen.
    """
    if action in K.TOOL_SCOPED_ACTIONS and not params.get("tool_id"):
        return _refuse("stop_without_tool", "정지 대상 도구가 지정되지 않음")
    return ALLOWED


def _calibration_capability(
    context: ArmCommandContext,
    params: Mapping[str, Any],
) -> Decision | None:
    """Arm and tool calibration are separate capabilities; dual is a third."""
    scope = params.get("scope")
    if scope == K.SCOPE_ARM:
        needed = CAP_ARM_CALIBRATION
    elif scope == K.SCOPE_TOOL:
        needed = (
            CAP_DUAL_TOOL_CALIBRATION
            if context.tool_kind == "dual_gripper"
            else CAP_TOOL_CALIBRATION
        )
    else:
        return _refuse("unknown_scope", "알 수 없는 보정 범위")
    if needed not in context.capabilities:
        return _refuse(
            "capability_missing",
            "지원하지 않음 — 이 대상의 보정 구현이 없음",
        )
    return None


def _tool_identity_decision(
    context: ArmCommandContext,
    params: Mapping[str, Any],
) -> Decision | None:
    if not context.tool_id:
        return _refuse("no_active_tool", "활성 도구가 관측되지 않음")
    if params.get("tool_id") != context.tool_id:
        return _refuse("tool_id_mismatch", "명령 대상 도구가 현재 도구와 다름")
    if params.get("tool_generation") != context.tool_generation:
        return _refuse(
            "tool_generation_mismatch",
            "도구 세대 불일치 — 교체 이후의 명령이 아님",
        )
    return None


@dataclass(frozen=True)
class PolicyAudit:
    """A record of one refusal, for the operation log and for tests."""

    action: str
    reason: str
    korean: str
    context: ArmCommandContext = field(default_factory=ArmCommandContext)
