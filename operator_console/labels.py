"""Pure Korean operator-facing labels shared by the console panels."""
from __future__ import annotations


COMPONENT_KOREAN = {
    "drive": "구동 모터",
    "steer": "조향 모터",
    "us100": "US-100 안전",
    "robot_arm": "로봇팔",
}
ON_LABEL, OFF_LABEL = "켜짐", "꺼짐"


def mode_korean(mode: str) -> str:
    """Return the Korean chassis-mode label while retaining its state code."""
    text = str(mode)
    return {
        "IDLE": "대기(IDLE)",
        "ESTOP": "비상정지(ESTOP)",
        "ARMED": "주행(ARMED)",
    }.get(text, text)


def estop_source_korean(source: str) -> str:
    """Return the operator-facing Korean label for an E-stop trip source."""
    text = str(source)
    return {
        # us100 소스는 근접 감지와 센서 무응답 latch를 모두 담는다 — 구체
        # 사유는 괄호의 detail(near/reader_not_started 등)이 말해준다.
        "us100": "US-100 안전 센서",
        "safety_topic_stale": "안전 센서 링크 두절",
        "console": "콘솔 비상정지 버튼",
        "manual_service": "수동 비상정지",
        "corner_fault": "모터(코너) 결함",
        "arm_failure": "시동 실패",
        "reset_failure": "초기화 실패",
        "extraction_budget_exhausted": "구조 탈출 한도 소진",
        "extraction_arm_failure": "구조 탈출 시동 실패",
        "active_estop_sources_not_us100_only": "구조 탈출 불가 조건",
        "extraction_complete": "구조 탈출 종료",
        "": "",
    }.get(text, text)


def freshness_korean(state: str) -> str:
    """Return the Korean receive-state label while retaining its state code."""
    text = str(state)
    return {
        "LIVE": "정상수신(LIVE)",
        "STALE": "지연(STALE)",
        "UNAVAILABLE": "미수신(UNAVAILABLE)",
        "WAITING": "대기중(WAITING)",
    }.get(text, text)


def ack_korean(status: str, detail: str) -> str:
    """Translate final command outcomes and the known operator rejection reasons."""
    status_text = str(status)
    detail_text = str(detail).strip()
    if status_text == "FINAL_SUCCESS":
        return "성공"
    if status_text == "OUTCOME_UNKNOWN":
        return "결과 미확정 — 재시도 가능"
    if status_text == "FINAL_REJECTED":
        if detail_text == "not_idle":
            reason = "대기(IDLE) 상태에서만 가능"
        elif detail_text == "busy: mutation in flight":
            reason = "다른 명령 처리 중"
        elif detail_text.startswith("service unavailable"):
            reason = "대상 노드 없음"
        else:
            reason = detail_text or "사유 없음"
        return f"거부 — {reason}"
    return status_text if not detail_text else f"{status_text} — {detail_text}"
