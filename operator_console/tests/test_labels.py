import pytest

from operator_console.labels import (
    COMPONENT_KOREAN,
    OFF_LABEL,
    ON_LABEL,
    ack_korean,
    estop_source_korean,
    freshness_korean,
    mode_korean,
)


def test_component_and_toggle_labels_match_operator_copy():
    assert COMPONENT_KOREAN == {
        "drive": "구동 모터",
        "steer": "조향 모터",
        "us100": "US-100 안전",
        "robot_arm": "로봇팔",
    }
    assert (ON_LABEL, OFF_LABEL) == ("켜짐", "꺼짐")


@pytest.mark.parametrize(
    ("mode", "expected"),
    (
        ("IDLE", "대기(IDLE)"),
        ("ESTOP", "비상정지(ESTOP)"),
        ("ARMED", "주행(ARMED)"),
        ("PAUSED", "PAUSED"),
    ),
)
def test_mode_korean_maps_known_modes_and_preserves_unknown(mode, expected):
    assert mode_korean(mode) == expected


@pytest.mark.parametrize(
    ("source", "expected"),
    (
        ("robot_arm", "로봇팔 연동 정지"),
        ("cmd_watchdog", "명령 두절"),
        ("us100_checking", "US-100 점검 중"),
        ("boot_qualification", "부팅 자격화 미충족"),
        ("remote_operator", "원격 조작자 비상정지"),
        ("control_exception", "차대 제어 예외"),
        ("node_shutdown", "차대 노드 종료"),
        ("extraction_estop_source", "구조 탈출 중 추가 비상정지"),
        ("extraction_ttl_expired", "구조 탈출 허가 시간 만료"),
        ("unmapped_source", "unmapped_source"),
    ),
)
def test_estop_source_korean_maps_real_interlock_sources_and_preserves_unknown(
    source,
    expected,
):
    assert estop_source_korean(source) == expected


@pytest.mark.parametrize(
    ("state", "expected"),
    (
        ("LIVE", "정상수신(LIVE)"),
        ("STALE", "지연(STALE)"),
        ("UNAVAILABLE", "미수신(UNAVAILABLE)"),
        ("WAITING", "대기중(WAITING)"),
        ("RECOVERING", "RECOVERING"),
    ),
)
def test_freshness_korean_maps_known_states_and_preserves_unknown(
    state,
    expected,
):
    assert freshness_korean(state) == expected


@pytest.mark.parametrize(
    ("status", "detail", "expected"),
    (
        ("FINAL_SUCCESS", "", "성공"),
        (
            "FINAL_REJECTED",
            "not_idle",
            "거부 — 대기(IDLE) 상태에서만 가능",
        ),
        (
            "FINAL_REJECTED",
            "busy: mutation in flight",
            "거부 — 다른 명령 처리 중",
        ),
        (
            "FINAL_REJECTED",
            "service unavailable: /chassis_node/arm",
            "거부 — 대상 노드 없음",
        ),
        ("FINAL_REJECTED", "interlock_open", "거부 — interlock_open"),
        (
            "OUTCOME_UNKNOWN",
            "no response",
            "결과 미확정 — 재시도 가능",
        ),
        ("PENDING", "request 7", "PENDING — request 7"),
    ),
)
def test_ack_korean_maps_final_status_and_rejection_reasons(
    status,
    detail,
    expected,
):
    assert ack_korean(status, detail) == expected
