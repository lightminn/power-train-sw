"""Pure presentation mapping for the judge-facing Mission Control view.

Transport parsers intentionally keep canonical technical values.  This module
is the only place that turns those values into concise, non-technical copy.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MissionPresentation:
    title: str
    description: str
    step: int
    tone: str


_MISSION_STATES = {
    "IDLE": MissionPresentation(
        "운용 준비", "임무 시작을 기다리고 있습니다", 0, "idle",
    ),
    "WAITING": MissionPresentation("운용 준비", "임무 시작을 기다리고 있습니다", 0, "idle"),
    "READY": MissionPresentation("운용 준비", "임무 시작을 기다리고 있습니다", 0, "idle"),
    "ARMED": MissionPresentation(
        "탐색", "주변에서 작업 대상을 찾고 있습니다", 1, "active",
    ),
    "PERCEIVE": MissionPresentation(
        "탐색", "주변에서 작업 대상을 찾고 있습니다", 1, "active",
    ),
    "PERCEIVING": MissionPresentation(
        "탐색", "주변에서 작업 대상을 찾고 있습니다", 1, "active",
    ),
    "PLAN": MissionPresentation(
        "접근", "확인된 대상까지의 이동 경로를 계산하고 있습니다", 2, "active",
    ),
    "PLANNING": MissionPresentation(
        "접근", "확인된 대상까지의 이동 경로를 계산하고 있습니다", 2, "active",
    ),
    "APPROACH": MissionPresentation(
        "접근", "확인된 대상으로 안전하게 이동하고 있습니다", 2, "active",
    ),
    "RUN": MissionPresentation(
        "접근", "확인된 대상으로 안전하게 이동하고 있습니다", 2, "active",
    ),
    "EXECUTING": MissionPresentation(
        "도구 작업", "장착된 도구로 작업을 수행하고 있습니다", 3, "active",
    ),
    "DESCEND": MissionPresentation("도구 작업", "장착된 도구로 작업을 수행하고 있습니다", 3, "active"),
    "GRASP": MissionPresentation("도구 작업", "장착된 도구로 작업을 수행하고 있습니다", 3, "active"),
    "LIFT": MissionPresentation("도구 작업", "장착된 도구로 작업을 수행하고 있습니다", 3, "active"),
    "CARRY": MissionPresentation("도구 작업", "장착된 도구로 작업을 수행하고 있습니다", 3, "active"),
    "RELEASE": MissionPresentation("도구 작업", "장착된 도구로 작업을 수행하고 있습니다", 3, "active"),
    "DONE": MissionPresentation(
        "완료", "계획된 작업을 완료했습니다", 4, "success",
    ),
    "COMPLETE": MissionPresentation("완료", "계획된 작업을 완료했습니다", 4, "success"),
    "MOTION_HOLD": MissionPresentation(
        "일시 정지", "안전 조건 회복을 기다리고 있습니다", 2, "warning",
    ),
    "HOLD": MissionPresentation(
        "일시 정지", "안전 조건 회복을 기다리고 있습니다", 2, "warning",
    ),
    "LOCKED": MissionPresentation("일시 정지", "안전 조건 회복을 기다리고 있습니다", 2, "warning"),
    "ESTOP": MissionPresentation(
        "비상 정지", "안전을 위해 모든 움직임을 즉시 정지했습니다", 2, "danger",
    ),
}


def canonical_mission_state(chassis_mode: str, drive_state: str = "") -> str:
    """Choose an observed state without inventing an unavailable FSM value."""
    candidates = (
        str(chassis_mode).strip().upper(),
        str(drive_state).split("/", 1)[0].strip().upper(),
    )
    for candidate in candidates:
        if candidate in _MISSION_STATES:
            return candidate
    return "UNKNOWN"


def mission_presentation(
    chassis_mode: str,
    drive_state: str = "",
) -> MissionPresentation:
    state = canonical_mission_state(chassis_mode, drive_state)
    if state == "UNKNOWN":
        return MissionPresentation(
            "운용 준비", "로봇의 현재 작업 단계를 확인하고 있습니다", 0, "offline",
        )
    return _MISSION_STATES[state]


def step_states(active_step: int) -> tuple[str, str, str, str, str]:
    """Return completed/current/upcoming state for the five public stages."""
    bounded = min(4, max(0, active_step))
    return tuple(
        "completed" if index < bounded
        else "active" if index == bounded
        else "upcoming"
        for index in range(5)
    )


def public_freshness(
    state: str,
    *,
    waiting: str = "연결 대기",
    unavailable: str = "정보 없음",
) -> str:
    normalized = str(state).strip().upper()
    return {
        "LIVE": "정상",
        "CLEAR": "정상",
        "OK": "정상",
        "NORMAL": "정상",
        "WAITING": waiting,
        "CONNECTING": "연결 중",
        "STALE": "업데이트 지연",
        "DISABLED": "사용 안 함",
        "MOTION_HOLD": "일시 정지",
        "LOCKED": "일시 정지",
        "ESTOP": "비상 정지",
        "FAULT": "확인 필요",
        "ERROR": "확인 필요",
        "CRITICAL": "확인 필요",
        "UNKNOWN": "연결 확인 중",
        "UNAVAILABLE": unavailable,
    }.get(normalized, "확인 필요" if normalized else "연결 확인 중")


def public_safety(
    *,
    telemetry_live: bool,
    estop_required: bool | None,
    us100_enabled: bool | None,
) -> tuple[str, str]:
    if us100_enabled is False:
        return "사용 안 함", "warning"
    if not telemetry_live or estop_required is None:
        return "연결 확인 중", "offline"
    if estop_required:
        return "비상 정지", "danger"
    return "정상", "success"
