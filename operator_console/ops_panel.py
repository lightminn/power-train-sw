"""Pure confirmation policy for the token-gated operator command panel."""
from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from typing import Any


GESTURE_STRIP = "confirm_strip"
GESTURE_HOLD = "hold_to_confirm"
GESTURE_SPACER = "spacer"
HOLD_CONFIRM_S = 1.5


@dataclass(frozen=True)
class PanelAction:
    """One panel row; ``action=None`` is a non-command spacer marker."""

    action: str | None
    label: str
    gesture: str
    needs_bool: bool = False
    confirm_text: str = ""


@dataclass(frozen=True)
class ConfirmState:
    """State and revision captured when the operator begins confirmation."""

    action: str
    gesture: str
    started_s: float
    revision: int
    state_snapshot: dict[str, Any]
    params: dict[str, Any]


PANEL_ACTIONS: tuple[PanelAction, ...] = (
    PanelAction(
        "clear_transient_hold",
        "Clear transient hold",
        GESTURE_STRIP,
        confirm_text="Clear recoverable teleop and authority holds?",
    ),
    PanelAction(
        "authority_manual",
        "Authority: manual",
        GESTURE_STRIP,
        confirm_text="Transfer command authority to the manual operator?",
    ),
    PanelAction(
        "authority_auto",
        "Authority: auto",
        GESTURE_STRIP,
        confirm_text="Transfer command authority to autonomy?",
    ),
    PanelAction(
        "authority_idle",
        "Authority: idle",
        GESTURE_STRIP,
        confirm_text="Return command authority to IDLE?",
    ),
    PanelAction(
        "estop_reset",
        "Reset E-stop latch",
        GESTURE_STRIP,
        confirm_text="Reset the E-stop latch to IDLE? This does not arm the chassis.",
    ),
    PanelAction(
        None,
        "Reset and arm are independent actions",
        GESTURE_SPACER,
    ),
    PanelAction(
        "arm",
        "Arm — hold 1.5 s",
        GESTURE_HOLD,
        confirm_text="Hold for 1.5 seconds to arm after reviewing the live state.",
    ),
    PanelAction(
        "disarm",
        "Disarm",
        GESTURE_STRIP,
        confirm_text="Disarm the chassis and return to IDLE?",
    ),
    PanelAction(
        "arm_lock_override",
        "Enable arm lock override",
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text=(
            "DANGER: ENABLE ARM SAFETY LOCK OVERRIDE. "
            "Use only for supervised recovery."
        ),
    ),
    PanelAction(
        "mission_arrive_pickup",
        "Mission: arrive pickup",
        GESTURE_STRIP,
        confirm_text="Report arrival at the pickup mission section?",
    ),
    PanelAction(
        "mission_arrive_drop",
        "Mission: arrive drop",
        GESTURE_STRIP,
        confirm_text="Report arrival at the drop mission section?",
    ),
    PanelAction(
        "mission_skip",
        "Mission: skip",
        GESTURE_STRIP,
        confirm_text="Skip the current failed mission step?",
    ),
    PanelAction(
        "mission_retry",
        "Mission: retry",
        GESTURE_STRIP,
        confirm_text="Retry the current failed mission step?",
    ),
    PanelAction(
        "mission_regrasp_confirmed",
        "Mission: regrasp confirmed",
        GESTURE_STRIP,
        confirm_text="Confirm that the arm has completed the regrasp?",
    ),
    PanelAction(
        "mission_clear_grip_lost",
        "Mission: clear grip lost",
        GESTURE_STRIP,
        confirm_text="Clear the grip-lost mission latch?",
    ),
    PanelAction(
        "operator_hold",
        "Operator hold",
        GESTURE_STRIP,
        confirm_text="Request an operator mission hold?",
    ),
    PanelAction(
        "operator_resume",
        "Operator resume",
        GESTURE_STRIP,
        confirm_text="Resume mission supervision after the operator hold?",
    ),
)


_ACTION_BY_NAME = {
    action.action: action
    for action in PANEL_ACTIONS
    if action.action is not None
}


class ConfirmFlow:
    """Two-step confirmation with state-revision revalidation."""

    def __init__(
        self,
        *,
        clock: Callable[[], float],
        state_provider: Callable[[], Mapping[str, Any] | None],
    ) -> None:
        self._clock = clock
        self._state_provider = state_provider
        self._pending: ConfirmState | None = None

    @property
    def pending(self) -> ConfirmState | None:
        return self._pending

    @staticmethod
    def _panel_action(action: str | PanelAction) -> PanelAction:
        if isinstance(action, PanelAction):
            panel_action = action
        else:
            panel_action = _ACTION_BY_NAME.get(str(action))
        if panel_action is None or panel_action.action is None:
            raise ValueError("unknown panel action: %r" % (action,))
        return panel_action

    @staticmethod
    def _revision(state: Mapping[str, Any]) -> int:
        revision = state.get("revision")
        if (
            not isinstance(revision, int)
            or isinstance(revision, bool)
            or revision < 0
        ):
            raise RuntimeError("ops state revision unavailable")
        return revision

    def begin(self, action: str | PanelAction) -> ConfirmState:
        """Capture one immutable state snapshot for the first confirmation step."""
        panel_action = self._panel_action(action)
        state = self._state_provider()
        if state is None:
            raise RuntimeError("ops state unavailable")
        snapshot = deepcopy(dict(state))
        params = {"data": True} if panel_action.needs_bool else {}
        self._pending = ConfirmState(
            action=panel_action.action,
            gesture=panel_action.gesture,
            started_s=float(self._clock()),
            revision=self._revision(snapshot),
            state_snapshot=snapshot,
            params=params,
        )
        return self._pending

    def confirm(
        self,
        action: str | PanelAction,
        *,
        held_s: float | None = None,
    ) -> dict[str, Any] | None:
        """Revalidate the revision and return keyword arguments for ``submit``."""
        panel_action = self._panel_action(action)
        pending = self._pending
        if pending is None or pending.action != panel_action.action:
            return None

        current = self._state_provider()
        try:
            current_revision = None if current is None else self._revision(current)
        except RuntimeError:
            current_revision = None
        if current_revision != pending.revision:
            self.reset()
            return None

        if panel_action.gesture == GESTURE_HOLD:
            try:
                duration_s = float(held_s)
            except (TypeError, ValueError):
                return None
            if duration_s < HOLD_CONFIRM_S:
                return None

        result = {
            "action": pending.action,
            "params": dict(pending.params),
            "expected_state_revision": pending.revision,
        }
        self.reset()
        return result

    def reset(self) -> None:
        self._pending = None
