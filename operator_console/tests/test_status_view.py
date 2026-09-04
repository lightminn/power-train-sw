import inspect
import json
from pathlib import Path
from types import SimpleNamespace

import operator_console.status_view as status_view
from operator_console.status_view import (
    END_EFFECTOR_PURPOSES,
    GRAPH_WINDOW_S,
    MAX_GRAPH_SAMPLES,
    CompetitionStatusDashboard,
    RobotStatusDashboard,
    TimedSeries,
    communication_badge_state,
    pdist_alarm_names,
    power_card_state,
    public_link_state,
    safety_badge_state,
)
from operator_console.telemetry import parse_telemetry


def test_timed_series_is_bounded_by_count_and_time_window():
    series = TimedSeries()
    for index in range(MAX_GRAPH_SAMPLES + 50):
        series.append(float(index) / 5.0, float(index))

    samples = series.samples()
    assert len(samples) <= MAX_GRAPH_SAMPLES
    assert samples[-1].timestamp_s - samples[0].timestamp_s <= GRAPH_WINDOW_S


def test_timed_series_keeps_explicit_stale_gap():
    series = TimedSeries()
    series.append(1.0, 2.0)
    series.mark_gap(2.0)
    series.mark_gap(3.0)

    assert [sample.value for sample in series.samples()] == [2.0, None]


def test_pdist_operating_bits_are_not_misclassified_as_alarms():
    assert pdist_alarm_names(0b00000011, 0b11100000) == ()
    assert pdist_alarm_names(1 << 3, 1 << 1) == (
        "저전압 보호", "방전 과전류",
    )


def test_status_view_defaults_match_operator_view_options():
    assert RobotStatusDashboard.PANEL_ORDER == (
        "drive", "power", "safety", "network", "ai", "arm",
    )


def test_operator_end_effector_inventory_is_limited_to_four_confirmed_categories():
    assert tuple(END_EFFECTOR_PURPOSES) == (
        "그리퍼 1", "그리퍼 2", "청소 모듈", "환경 센서 모듈",
    )


def test_competition_status_is_unified_around_power_communication_and_safety():
    source = inspect.getsource(CompetitionStatusDashboard)
    assert '"power", "전원 · PDIST80B"' in source
    assert '"communication", "통신"' in source
    assert '"safety", "안전"' in source
    assert '"drive", "주행"' not in source
    assert '"arm", "로봇팔"' not in source
    assert "값은 추정하지 않습니다" in source


def test_status_dashboard_has_no_control_or_transport_send_surface():
    source = (
        Path(__file__).resolve().parents[1] / "status_view.py"
    ).read_text(encoding="utf-8")

    assert "OpsClient" not in source
    assert "submit(" not in source
    assert "sendto(" not in source
    assert "component_mask[" not in source


def test_mission_display_options_only_change_overlay_visibility():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert "def set_display_options(" in source
    assert "self._show_objects" in source
    assert "self._show_distance" in source
    assert "set_metadata_display_options(" in source
    assert "YOLO Node" not in source
    assert "metadata_receiver.close()" not in source[
        source.index("def _on_display_option_toggled"):
        source.index("def _on_video_area_allocated")
    ]


def _power_snapshot(**overrides):
    payload = {
        "schema_version": 1,
        "sequence": 1,
        "voltage_v": 47.8,
        "current_a": None,
        "power_w": None,
        "drive_state": "PDIST unavailable",
        "can_state": "unavailable",
        "pdist_soc_percent": 80,
        "pdist_battery_flags": 0,
        "pdist_protection_flags": 0,
        "rs485_state": "LIVE",
        "rs485_consecutive_failures": 0,
    }
    payload.update(overrides)
    return parse_telemetry(json.dumps(payload).encode("utf-8"))


def test_power_card_is_not_normal_when_rs485_link_is_dead():
    power_card_state = getattr(status_view, "power_card_state", None)
    assert power_card_state is not None
    snapshot = _power_snapshot(
        voltage_v=None,
        pdist_soc_percent=None,
        pdist_battery_flags=None,
        pdist_protection_flags=None,
        rs485_state="ERROR",
        rs485_consecutive_failures=3691,
    )

    state, reason = power_card_state(snapshot, fresh=True)

    assert state == "확인 필요"
    assert "ERROR" in reason or "전원" in reason


def test_power_card_rs485_error_alone_is_enough_to_demote():
    """rs485 분기를 **독립적으로** 검증한다.

    바로 위 테스트는 전압·SOC 가 둘 다 결측이라 다른 분기가 대신 잡아 준다 —
    그래서 rs485 검사를 통째로 빼도 통과했다(2026-07-29 음성 대조에서 확인).
    계측값이 멀쩡한데 링크만 ERROR 인 경우를 따로 봐야 게이트가 성립한다.
    """
    power_card_state = getattr(status_view, "power_card_state", None)
    assert power_card_state is not None
    snapshot = _power_snapshot(
        voltage_v=48.2,
        pdist_soc_percent=91,
        rs485_state="ERROR",
        rs485_consecutive_failures=12,
    )

    state, reason = power_card_state(snapshot, fresh=True)

    assert state == "확인 필요"
    assert "ERROR" in reason


def test_power_card_is_not_normal_when_voltage_and_soc_are_both_missing():
    power_card_state = getattr(status_view, "power_card_state", None)
    assert power_card_state is not None
    snapshot = _power_snapshot(voltage_v=None, pdist_soc_percent=None)

    assert power_card_state(snapshot, fresh=True) == (
        "확인 필요",
        "전원 계측값 없음",
    )


def test_power_card_preserves_protection_flag_warning():
    power_card_state = getattr(status_view, "power_card_state", None)
    assert power_card_state is not None
    snapshot = _power_snapshot(pdist_protection_flags=2)

    assert power_card_state(snapshot, fresh=True) == (
        "확인 필요",
        "보호 상태 확인 필요",
    )


def test_power_card_reports_normal_only_for_fresh_healthy_measurement():
    power_card_state = getattr(status_view, "power_card_state", None)
    assert power_card_state is not None
    snapshot = _power_snapshot()

    assert power_card_state(snapshot, fresh=True) == ("정상", "LIVE")
    assert power_card_state(snapshot, fresh=False) == (
        "확인 필요",
        "전원 정보 수신 지연",
    )
    assert power_card_state(None, fresh=True) == (
        "정보 없음",
        "전원 장치 정보 없음",
    )


def test_power_card_does_not_infer_normal_without_protection_flags():
    snapshot = _power_snapshot(
        pdist_battery_flags=None,
        pdist_protection_flags=None,
    )

    assert power_card_state(snapshot, fresh=True) == (
        "확인 필요",
        "보호 상태 정보 없음",
    )


def test_system_link_codes_are_translated_for_operator_view():
    assert public_link_state("LIVE") == "실시간 수신"
    assert public_link_state("VALID") == "정상"
    assert public_link_state("NO_RESPONSE") == "응답 없음"
    assert public_link_state("UNAVAILABLE") == "정보 없음"
    assert public_link_state("LIVE", control=True) == "연결됨"
    assert public_link_state("UNAVAILABLE", control=True) == "사용 불가"


def test_communication_badge_requires_every_displayed_link():
    complete = dict(
        chassis_fresh=True,
        power_fresh=True,
        metadata_fresh=True,
        front_video_state="LIVE",
        work_video_state="LIVE",
        control_link_ready=True,
        any_telemetry_seen=True,
    )
    assert communication_badge_state(**complete) == ("정상", "status-live")

    one_camera_missing = {**complete, "work_video_state": "CONNECTING"}
    assert communication_badge_state(**one_camera_missing) == (
        "확인 필요",
        "status-warn",
    )

    no_ai = {**complete, "metadata_fresh": False}
    assert communication_badge_state(**no_ai) == (
        "확인 필요",
        "status-warn",
    )


def test_safety_badge_requires_safety_enable_and_required_arm_data():
    chassis = SimpleNamespace(
        safety_estop_required=False,
        component_mask={"us100": True, "robot_arm": True},
    )
    arm = SimpleNamespace(dynamixel=())
    assert safety_badge_state(
        chassis=chassis, chassis_fresh=True, arm=arm, arm_fresh=True,
    ) == ("정상", "status-live")
    assert safety_badge_state(
        chassis=chassis, chassis_fresh=True, arm=None, arm_fresh=False,
    ) == ("확인 필요", "status-warn")

    unknown_enable = SimpleNamespace(
        safety_estop_required=False,
        component_mask=None,
    )
    assert safety_badge_state(
        chassis=unknown_enable, chassis_fresh=True, arm=arm, arm_fresh=True,
    ) == ("확인 필요", "status-warn")


def test_safety_badge_preserves_estop_priority():
    chassis = SimpleNamespace(
        safety_estop_required=True,
        component_mask={"us100": True, "robot_arm": True},
    )
    assert safety_badge_state(
        chassis=chassis, chassis_fresh=True, arm=None, arm_fresh=False,
    ) == ("비상정지", "status-bad")


def test_status_dashboard_uses_power_health_not_freshness_for_ready_count():
    source = inspect.getsource(RobotStatusDashboard.update)

    assert "power_card_state(power, fresh=power_fresh)" in source
