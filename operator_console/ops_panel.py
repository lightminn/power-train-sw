"""Pure confirmation policy for the token-gated operator command panel."""
from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass
from html import escape
from typing import Any

from .labels import COMPONENT_KOREAN, estop_source_korean, mode_korean


GESTURE_STRIP = "confirm_strip"
GESTURE_HOLD = "hold_to_confirm"
GESTURE_IMMEDIATE = "immediate"
GESTURE_SPACER = "spacer"
HOLD_CONFIRM_S = 1.5
_ACK_COLORS = {
    "FINAL_REJECTED": "#d32f2f",
    "OUTCOME_UNKNOWN": "#e67e22",
}


def _estop_cause_text(estop_source: str, estop_detail: str) -> str:
    source_text = estop_source_korean(estop_source)
    detail_text = str(estop_detail)
    if detail_text:
        return f"{source_text} ({detail_text})"
    return source_text


def format_ops_status_line(
    chassis_mode: str,
    latest_ack: str,
    estop_source: str = "",
    estop_detail: str = "",
) -> str:
    """Build the Gtk-free operator status line for the latest ops state."""
    mode_text = str(chassis_mode)
    if mode_text == "ESTOP":
        return (
            f"모드: {mode_korean(mode_text)} — 원인: "
            f"{_estop_cause_text(estop_source, estop_detail)} · "
            f"최근: {latest_ack}"
        )
    return f"모드: {mode_korean(mode_text)} · 최근: {latest_ack}"


def next_estop_cause_event(
    previous_key: tuple[str, str] | None,
    *,
    chassis_mode: str,
    estop_source: str,
    estop_detail: str,
) -> tuple[tuple[str, str] | None, str | None]:
    """Return one event only when the visible latched E-stop cause changes."""
    source_text = str(estop_source)
    detail_text = str(estop_detail)
    if str(chassis_mode) != "ESTOP" or not source_text:
        return None, None
    current_key = (source_text, detail_text)
    if current_key == previous_key:
        return current_key, None
    return (
        current_key,
        "비상정지 원인: " + _estop_cause_text(source_text, detail_text),
    )


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
    advanced: bool = False


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


def mode_allows_action(action: str, chassis_mode: str) -> bool:
    """Gate only drive/steer mutation to IDLE; other modules stay operable."""
    if str(action) not in {"drive_enable", "steer_enable"}:
        return True
    return str(chassis_mode) == "IDLE"


def _component_toggle_value(component: str) -> Callable[[dict[str, Any]], bool]:
    def value_from_state(state: dict[str, Any]) -> bool:
        component_mask = component_mask_from_state(state)
        if component_mask is None or component not in component_mask:
            raise RuntimeError("component mask unavailable")
        return not component_mask[component]

    return value_from_state


PANEL_ACTIONS: tuple[PanelAction, ...] = (
    PanelAction(
        "estop",
        "비상정지 (ESTOP)",
        GESTURE_IMMEDIATE,
    ),
    PanelAction(
        "estop_reset",
        "경고 초기화",
        GESTURE_STRIP,
        confirm_text="비상정지 경고를 초기화하고 대기(IDLE) 상태로 돌아갑니까? 시동은 별도입니다.",
    ),
    PanelAction(
        None,
        "경고 초기화와 시동은 별도 조작입니다",
        GESTURE_SPACER,
    ),
    PanelAction(
        "arm",
        "시동 — 1.5초 홀드",
        GESTURE_HOLD,
        confirm_text="현재 상태를 확인한 뒤 1.5초 동안 눌러 시동합니다.",
    ),
    PanelAction(
        "disarm",
        "시동 해제",
        GESTURE_STRIP,
        confirm_text="차대를 시동 해제하고 대기(IDLE) 상태로 돌아갑니까?",
    ),
    PanelAction(
        "drive_enable",
        COMPONENT_KOREAN["drive"],
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="이번 세션의 구동 모터 사용 상태를 전환합니까?",
        bool_value_from_state=_component_toggle_value("drive"),
    ),
    PanelAction(
        "steer_enable",
        COMPONENT_KOREAN["steer"],
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="이번 세션의 조향 모터 사용 상태를 전환합니까?",
        bool_value_from_state=_component_toggle_value("steer"),
    ),
    PanelAction(
        "us100_enable",
        COMPONENT_KOREAN["us100"],
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="충돌 안전 센서를 끕니다 — 접근 시 자동 정지 없음",
        bool_value_from_state=_component_toggle_value("us100"),
    ),
    PanelAction(
        "robot_arm_enable",
        COMPONENT_KOREAN["robot_arm"],
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="이번 세션의 로봇팔 사용 상태를 전환합니까?",
        bool_value_from_state=_component_toggle_value("robot_arm"),
    ),
    PanelAction(
        "authority_manual",
        "권한: 수동",
        GESTURE_STRIP,
        confirm_text="명령 권한을 수동 조작자로 전환합니까?",
        advanced=True,
    ),
    PanelAction(
        "authority_auto",
        "권한: 자동",
        GESTURE_STRIP,
        confirm_text="명령 권한을 자율주행으로 전환합니까?",
        advanced=True,
    ),
    PanelAction(
        "authority_idle",
        "권한: 대기",
        GESTURE_STRIP,
        confirm_text="명령 권한을 대기(IDLE) 상태로 전환합니까?",
        advanced=True,
    ),
    PanelAction(
        "extraction_grant",
        "구조 탈출 허가 — 후진 0.2 m/s·3초",
        GESTURE_STRIP,
        confirm_text="US-100 단독 비상정지에서만 구조 탈출을 허가합니다 — 후진 0.2 m/s·3초.",
        advanced=True,
    ),
    PanelAction(
        "arm_lock_override",
        "로봇팔 잠금 해제",
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="위험: 로봇팔 안전 잠금을 해제합니다. 감독하의 복구 작업에만 사용하십시오.",
        advanced=True,
    ),
    PanelAction(
        "clear_transient_hold",
        "일시 정지 해제",
        GESTURE_STRIP,
        confirm_text="복구 가능한 텔레옵·권한 일시 정지를 해제합니까?",
        advanced=True,
    ),
    PanelAction(
        "mission_arrive_pickup",
        "임무: 수거 지점 도착",
        GESTURE_STRIP,
        confirm_text="수거 임무 구간 도착을 보고합니까?",
        advanced=True,
    ),
    PanelAction(
        "mission_arrive_drop",
        "임무: 하역 지점 도착",
        GESTURE_STRIP,
        confirm_text="하역 임무 구간 도착을 보고합니까?",
        advanced=True,
    ),
    PanelAction(
        "mission_skip",
        "임무: 건너뛰기",
        GESTURE_STRIP,
        confirm_text="현재 실패한 임무 단계를 건너뜁니까?",
        advanced=True,
    ),
    PanelAction(
        "mission_retry",
        "임무: 재시도",
        GESTURE_STRIP,
        confirm_text="현재 실패한 임무 단계를 다시 시도합니까?",
        advanced=True,
    ),
    PanelAction(
        "mission_regrasp_confirmed",
        "임무: 재파지 확인",
        GESTURE_STRIP,
        confirm_text="로봇팔 재파지가 완료되었음을 확인합니까?",
        advanced=True,
    ),
    PanelAction(
        "mission_clear_grip_lost",
        "임무: 파지 상실 해제",
        GESTURE_STRIP,
        confirm_text="파지 상실 임무 래치를 해제합니까?",
        advanced=True,
    ),
    PanelAction(
        "operator_hold",
        "운영자 일시 정지",
        GESTURE_STRIP,
        confirm_text="운영자 임무 일시 정지를 요청합니까?",
        advanced=True,
    ),
    PanelAction(
        "operator_resume",
        "운영자 임무 재개",
        GESTURE_STRIP,
        confirm_text="운영자 일시 정지 후 임무 감독을 재개합니까?",
        advanced=True,
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
