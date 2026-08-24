import ast
import inspect
import json
from pathlib import Path
import textwrap
from types import SimpleNamespace

import operator_console.status_view as status_view
from operator_console.status_view import (
    GRAPH_WINDOW_S,
    MAX_GRAPH_SAMPLES,
    RobotStatusDashboard,
    TimedSeries,
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


def test_status_view_defaults_to_drive_detail_only():
    constructor = ast.parse(textwrap.dedent(
        inspect.getsource(RobotStatusDashboard.__init__)
    ))
    selected_panel_values = []
    for node in ast.walk(constructor):
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = (node.target,)
        else:
            continue
        if any(
            isinstance(target, ast.Attribute)
            and isinstance(target.value, ast.Name)
            and target.value.id == "self"
            and target.attr == "_selected_panel"
            for target in targets
        ):
            selected_panel_values.append(node.value)

    assert len(selected_panel_values) == 1
    default_panel = ast.literal_eval(selected_panel_values[0])
    assert default_panel == "drive"

    dashboard = SimpleNamespace(_selected_panel=default_panel)
    assert {
        key: RobotStatusDashboard.view_enabled(dashboard, key)
        for key in RobotStatusDashboard.PANEL_ORDER
    } == {
        "drive": True,
        "power": False,
        "arm": False,
        "safety": False,
        "ai": False,
        "network": False,
    }


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


def test_power_card_treats_charger_status_bits_as_normal():
    power_card_state = getattr(status_view, "power_card_state", None)
    assert power_card_state is not None

    assert power_card_state(
        _power_snapshot(
            pdist_battery_flags=0x02,
            pdist_protection_flags=0x20,
        ),
        fresh=True,
    ) == ("정상", "LIVE")
    assert power_card_state(
        _power_snapshot(
            pdist_battery_flags=0,
            pdist_protection_flags=0x40,
        ),
        fresh=True,
    ) == ("정상", "LIVE")


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
        "전원 장치 정보 수신 대기",
    )


def test_status_dashboard_uses_power_health_not_freshness_for_ready_count():
    source = inspect.getsource(RobotStatusDashboard.update)

    assert "power_card_state(power, fresh=power_fresh)" in source


from operator_console.status_view import drive_mode_badges


def test_drive_mode_badges_render_both_axes():
    badges = drive_mode_badges({"drive_transport": "usb", "steering_mode": "skid"})

    assert badges == ("구동 USB", "조향 스키드")


def test_drive_mode_badges_render_can_ackermann():
    badges = drive_mode_badges({"drive_transport": "can",
                                "steering_mode": "ackermann"})

    assert badges == ("구동 CAN", "조향 애커만")


def test_drive_mode_badges_mark_unknown_state():
    assert drive_mode_badges(None) == ("구동 —", "조향 —")
