"""Pure confirmation policy for the token-gated operator command panel."""
from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from html import escape
from typing import Any


GESTURE_STRIP = "confirm_strip"
GESTURE_HOLD = "hold_to_confirm"
GESTURE_SPACER = "spacer"
HOLD_CONFIRM_S = 1.5
_ACK_COLORS = {
    "FINAL_REJECTED": "#d32f2f",
    "OUTCOME_UNKNOWN": "#e67e22",
}


def format_ack_line(action: str, status: str, detail: str = "") -> str:
    """Build the latest command acknowledgement shown in the ops panel."""
    line = "last: %s %s" % (str(action), str(status))
    detail_text = str(detail).strip()
    if detail_text:
        line += " · %s" % detail_text
    return line


def format_ack_markup(action: str, status: str, detail: str = "") -> str:
    """Return escaped Pango markup with explicit rejected/unknown colors."""
    status_text = str(status)
    line = escape(format_ack_line(action, status_text, detail), quote=False)
    color = _ACK_COLORS.get(status_text)
    if color is None:
        return line
    return '<span foreground="%s">%s</span>' % (color, line)


@dataclass(frozen=True)
class PanelAction:
    """One panel row; ``action=None`` is a non-command spacer marker."""

    action: str | None
    label: str
    gesture: str
    needs_bool: bool = False
    confirm_text: str = ""
    bool_value_from_state: Callable[[dict[str, Any]], bool] | None = None


@dataclass(frozen=True)
class ConfirmState:
    """State and revision captured when the operator begins confirmation."""

    action: str
    gesture: str
    started_s: float
    revision: int
    state_snapshot: dict[str, Any]
    params: dict[str, Any]


def component_mask_from_state(
    state: Mapping[str, Any] | None,
) -> dict[str, bool] | None:
    """Return a validated component mask from one ops-state snapshot."""
    if state is None:
        return None
    raw_mask = state.get("component_mask")
    if not isinstance(raw_mask, Mapping):
        return None
    component_mask: dict[str, bool] = {}
    for component, enabled in raw_mask.items():
        if not isinstance(component, str) or not isinstance(enabled, bool):
            return None
        component_mask[component] = enabled
    return component_mask


def _component_toggle_value(component: str) -> Callable[[dict[str, Any]], bool]:
    def value_from_state(state: dict[str, Any]) -> bool:
        component_mask = component_mask_from_state(state)
        if component_mask is None or component not in component_mask:
            raise RuntimeError("component mask unavailable")
        return not component_mask[component]

    return value_from_state


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
        "extraction_grant",
        "Extraction grant — US-100 단독 latch에서만 · 후진 −0.2 m/s · TTL 3 s",
        GESTURE_STRIP,
        confirm_text=(
            "US-100 단독 latch에서만 · 후진 −0.2 m/s · TTL 3 s — "
            "grant extraction recovery?"
        ),
    ),
    PanelAction(
        "disarm",
        "Disarm",
        GESTURE_STRIP,
        confirm_text="Disarm the chassis and return to IDLE?",
    ),
    PanelAction(
        "drive_enable",
        "Drive motors",
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="Toggle drive motor participation for this session?",
        bool_value_from_state=_component_toggle_value("drive"),
    ),
    PanelAction(
        "steer_enable",
        "Steer motors",
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="Toggle steer motor participation for this session?",
        bool_value_from_state=_component_toggle_value("steer"),
    ),
    PanelAction(
        "us100_enable",
        "US-100 safety",
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="충돌 안전 센서를 끕니다 — 접근 시 자동 정지 없음",
        bool_value_from_state=_component_toggle_value("us100"),
    ),
    PanelAction(
        "robot_arm_enable",
        "Robot arm",
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="Toggle robot-arm participation for this session?",
        bool_value_from_state=_component_toggle_value("robot_arm"),
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
        if panel_action.bool_value_from_state is not None:
            params = {"data": panel_action.bool_value_from_state(snapshot)}
        else:
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
