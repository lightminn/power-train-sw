import json

import gi
import pytest

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

from operator_console.app import EventLog
from operator_console.arm_telemetry import parse_arm_telemetry
from operator_console.environment_telemetry import parse_environment_telemetry
from operator_console.metadata import (
    Detection,
    DisplayTargetTracker,
    MetadataFrame,
    parse_metadata,
)
from operator_console.runtime_smoke import (
    _arm_payload,
    _chassis_payload,
    _metadata_payload,
    _telemetry_payload,
)
from operator_console.status_view import (
    CompetitionStatusDashboard,
    EnvironmentSensorDashboard,
    RobotStatusDashboard,
    drive_card_state,
)
from operator_console.telemetry import parse_telemetry


requires_gtk = pytest.mark.skipif(
    not Gtk.init_check(None)[0],
    reason="GTK display is unavailable",
)


def _visible_rows(log: EventLog) -> list:
    return [
        row for row in log._operation_list.get_children()
        if row.get_visible()
    ]


def _environment_snapshot(**overrides):
    payload = {
        "schema_version": 1,
        "sequence": 1,
        "source": "test-environment",
        "sensor_ok": True,
        "errors": [],
        "temperature_c": 24.0,
        "humidity_pct": 50.0,
        "pressure_hpa": 1005.0,
        "eco2_ppm": 420.0,
        "tvoc_ppb": 2.0,
        "sgp30_warming_up": False,
        "co_estimated_ppm": 1.0,
        "lpg_estimated_ppm": 20.0,
        "flame_detected": False,
        "flame_voltage_v": 2.7,
        "flame_threshold_v": 1.5,
    }
    payload.update(overrides)
    return parse_environment_telemetry(
        json.dumps(payload).encode("utf-8"),
        received_monotonic_s=10.0,
    )


def _parsed(parser, payload, *, received_monotonic_s=10.0):
    return parser(
        json.dumps(payload).encode('utf-8'),
        received_monotonic_s=received_monotonic_s,
    )


def _healthy_competition_fixtures():
    power_payload = _telemetry_payload(1)
    power_payload.update(
        pdist_protection_flags=0,
        pdist_battery_flags=0,
        rs485_state='LIVE',
    )
    chassis_payload = _chassis_payload(1)
    chassis_payload.update(
        drive_state='IDLE/OK',
        can_state='OK',
        wheel_fault_count=0,
        wheel_stale_count=0,
        wheel_axis_error_count=0,
        wheel_steer_fault_count=0,
        component_mask={
            'drive': True, 'steer': True, 'us100': True, 'robot_arm': True,
        },
    )
    for wheel in chassis_payload['wheel_statuses']:
        wheel.update(stale=False, drive_axis_error=0, steer_fault=0)
    arm_payload = _arm_payload(1)
    for motor in arm_payload['dynamixel']:
        motor['temperature_c'] = 34
    return power_payload, chassis_payload, arm_payload, _metadata_payload(1)


def _update_competition_dashboard(
    chassis_overrides=None, wheel_overrides=None, metadata_age_s=0.0,
):
    power_payload, chassis_payload, arm_payload, metadata_payload = (
        _healthy_competition_fixtures()
    )
    if chassis_overrides:
        chassis_payload.update(chassis_overrides)
    if wheel_overrides:
        chassis_payload['wheel_statuses'][0].update(wheel_overrides)
    dashboard = CompetitionStatusDashboard()
    dashboard.update(
        power=_parsed(parse_telemetry, power_payload),
        chassis=_parsed(parse_telemetry, chassis_payload),
        arm=_parsed(parse_arm_telemetry, arm_payload),
        metadata=_parsed(
            parse_metadata, metadata_payload,
            received_monotonic_s=10.0 - metadata_age_s,
        ),
        front_video_state='LIVE', work_video_state='LIVE',
        front_fps=30.0, work_fps=30.0,
        control_link_ready=True, now_s=10.0,
    )
    return dashboard


@requires_gtk
def test_environment_dashboard_distinguishes_hazard_and_missing_data():
    dashboard = EnvironmentSensorDashboard(port=15008)

    dashboard.update(None, now_s=10.0)
    assert all(
        label.get_text() == "● 미연결"
        for label in dashboard._statuses.values()
    )
    assert dashboard._overall_status["bad"].get_text() == "0"
    assert dashboard._overall_status["muted"].get_text() == "8"

    dashboard.update(
        _environment_snapshot(flame_detected=True, temperature_c=None),
        now_s=10.0,
    )
    assert dashboard._statuses["flame"].get_text() == "● 위험"
    assert dashboard._statuses["flame"].get_style_context().has_class(
        "status-bad"
    )
    assert dashboard._statuses["temperature"].get_text() == "● 데이터 없음"
    assert dashboard._overall_status["bad"].get_text() == "1"
    assert dashboard._overall_status["muted"].get_text() == "1"
    assert "위험 신호 감지" in dashboard._summary.get_text()
    assert dashboard._values["flame"].get_text() == "O"


@requires_gtk
def test_environment_dashboard_never_turns_unknown_flame_into_normal():
    dashboard = EnvironmentSensorDashboard(port=15008)
    dashboard.update(
        _environment_snapshot(flame_detected=None), now_s=10.0,
    )

    assert dashboard._statuses["flame"].get_text() == "● 데이터 없음"
    assert "불꽃 —" in dashboard.probe_values()[2]


@requires_gtk
def test_environment_summary_never_leaks_none_for_missing_values():
    dashboard = EnvironmentSensorDashboard(port=15008)
    dashboard.update(
        _environment_snapshot(
            temperature_c=None,
            eco2_ppm=None,
            co_estimated_ppm=None,
            flame_threshold_v=None,
        ),
        now_s=10.0,
    )

    assert all("None" not in value for value in dashboard.probe_values())
    assert "온도 —" in dashboard.probe_values()[0]
    assert "eCO₂ —" in dashboard.probe_values()[1]
    assert "CO 정보 없음" in dashboard.probe_values()[2]


@requires_gtk
def test_environment_dashboard_clears_last_values_when_connection_is_lost():
    dashboard = EnvironmentSensorDashboard(port=15008)
    snapshot = _environment_snapshot()
    dashboard.update(snapshot, now_s=10.0)

    assert dashboard._values["temperature"].get_text() == "24.00 °C"
    assert dashboard._values["co"].get_text() == "1.0 ppm"
    assert dashboard._values["lpg"].get_text() == "20.0 ppm"
    assert dashboard._values["flame"].get_text() == "X"
    assert dashboard._statuses["co"].get_text() == "● 정상"
    assert dashboard._statuses["lpg"].get_text() == "● 정상"
    assert "교정 전" not in dashboard._summary.get_text()

    dashboard.update(snapshot, now_s=20.0)

    assert all(
        label.get_text() == "● 미연결"
        for label in dashboard._statuses.values()
    )
    assert all(
        label.get_text() == "미연결"
        for label in dashboard._values.values()
    )
    assert dashboard._connection.get_text() == "●  미연결"
    assert dashboard.probe_values() == ("미연결", "미연결", "미연결")
    assert dashboard._overall_status["muted"].get_text() == "8"
    assert all(len(trend._series) == 0 for trend in dashboard._trends.values())


@requires_gtk
def test_environment_dashboard_treats_sensor_power_loss_as_disconnected():
    dashboard = EnvironmentSensorDashboard(port=15008)
    dashboard.update(
        _environment_snapshot(sensor_ok=False, errors=["sensor power lost"]),
        now_s=10.0,
    )

    assert dashboard._summary.get_text() == "환경 센서 미연결"
    assert all(
        value.get_text() == "미연결"
        for value in dashboard._values.values()
    )


@requires_gtk
def test_environment_dashboard_marks_sgp30_warmup_without_counting_it_normal():
    dashboard = EnvironmentSensorDashboard(port=15008)

    dashboard.update(
        _environment_snapshot(sgp30_warming_up=True), now_s=10.0,
    )

    assert dashboard._statuses['eco2'].get_text() == '● 예열 중'
    assert dashboard._statuses['tvoc'].get_text() == '● 예열 중'
    assert dashboard._values['eco2'].get_text() == '예열 중'
    assert dashboard._values['tvoc'].get_text() == '예열 중'
    assert dashboard._overall_status['live'].get_text() == '6'
    assert dashboard._overall_status['warn'].get_text() == '2'

    dashboard.update(
        _environment_snapshot(sequence=2, sgp30_warming_up=False), now_s=10.0,
    )
    assert dashboard._statuses['eco2'].get_text() == '● 정상'
    assert dashboard._statuses['tvoc'].get_text() == '● 정상'
    assert dashboard._overall_status['live'].get_text() == '8'
    assert dashboard._overall_status['warn'].get_text() == '0'


@requires_gtk
def test_competition_summary_can_be_ready_with_fully_healthy_fixtures():
    dashboard = _update_competition_dashboard()

    assert dashboard._system_overall.get_text() == '●  운용 준비 완료'


@requires_gtk
@pytest.mark.parametrize(
    ('metadata_age_s', 'fresh'),
    [(0.75, False), (0.30, False), (0.10, True), (0.25, True)],
    ids=('age-075', 'age-030', 'age-010', 'status-threshold'),
)
def test_competition_metadata_uses_status_freshness_contract(
    metadata_age_s, fresh,
):
    dashboard = _update_competition_dashboard(metadata_age_s=metadata_age_s)

    expected_detail = '정상' if fresh else '갱신 지연'
    expected_public = '실시간 수신' if fresh else '갱신 지연'
    expected_badge = '정상' if fresh else '확인 필요'
    expected_overall = (
        '●  운용 준비 완료' if fresh else '●  확인 필요한 항목 있음'
    )
    assert dashboard._detail_metrics['network']['metadata'].get_text() == expected_detail
    assert dashboard._system_values['comm_ai'].get_text() == expected_public
    assert dashboard._system_badges['communication'].get_text() == expected_badge
    assert dashboard._system_overall.get_text() == expected_overall


@requires_gtk
@pytest.mark.parametrize(
    ('can_state', 'expected_state', 'expected_reason', 'expected_overall'),
    [
        (
            'UNHEALTHY · AK 4/4 · ODrive 6/6',
            '확인 필요', 'CAN 상태 확인 필요', '●  확인 필요한 항목 있음',
        ),
        (
            'HEALTHY · AK 4/4 · ODrive 6/6',
            '정상', 'IDLE/OK', '●  운용 준비 완료',
        ),
    ],
    ids=('producer-unhealthy', 'producer-healthy'),
)
def test_producer_can_health_controls_drive_readiness(
    can_state, expected_state, expected_reason, expected_overall,
):
    _power, chassis_payload, _arm, _metadata = _healthy_competition_fixtures()
    chassis_payload['can_state'] = can_state
    snapshot = _parsed(parse_telemetry, chassis_payload)

    assert drive_card_state(snapshot, fresh=True) == (
        expected_state, expected_reason,
    )
    dashboard = _update_competition_dashboard(
        chassis_overrides={'can_state': can_state},
    )
    assert dashboard._system_overall.get_text() == expected_overall


@requires_gtk
@pytest.mark.parametrize(
    ('chassis_overrides', 'wheel_overrides'),
    [
        ({'wheel_stale_count': 1}, {'stale': True}),
        ({'wheel_axis_error_count': 1}, {'drive_axis_error': 16}),
        ({'wheel_steer_fault_count': 1}, {'steer_fault': 1}),
        ({'drive_state': 'unavailable'}, None),
        ({'can_state': 'unavailable'}, None),
    ],
    ids=('wheel-stale', 'axis-error', 'steer-fault', 'drive-unavailable', 'can-unavailable'),
)
def test_competition_summary_is_not_ready_when_drive_health_is_bad(
    chassis_overrides, wheel_overrides,
):
    dashboard = _update_competition_dashboard(
        chassis_overrides=chassis_overrides,
        wheel_overrides=wheel_overrides,
    )

    assert dashboard._system_overall.get_text() == '●  확인 필요한 항목 있음'


def test_draw_does_not_advance_the_distance_filter():
    """렌더 횟수가 필터 결과를 바꾸면 화면 가림 여부에 따라 거리가 달라진다."""
    tracker = DisplayTargetTracker()
    target = Detection(
        "box-segmentation", 0.96, (500, 290, 220, 190),
        (0.0, 0.0, 0.30), is_pick_target=True,
    )
    frame = MetadataFrame(
        sequence=1, width=848, height=480, detections=(target,),
        received_monotonic_s=100.0, frame_id="camera_color_optical_frame",
    )
    tracker.update(frame, now_s=100.0)
    before = tracker.view()
    for _ in range(5):
        assert tracker.view() == before


def test_metadata_canvas_draw_path_is_read_only():
    """실제 회귀 가드: draw 핸들러가 상태를 가진 update() 를 부르면 안 된다.

    위 테스트는 view() 의 멱등성만 보므로 draw 가 update() 로 되돌아가는 회귀를
    잡지 못한다.
    """
    import inspect

    from operator_console.app import MetadataCanvas

    source = inspect.getsource(MetadataCanvas._on_draw)
    assert "_target_tracker.view(" in source
    assert "_target_tracker.update(" not in source


@requires_gtk
def test_pip_slot_ignores_the_child_natural_size():
    """PiP 는 스트림 해상도(848x480/1280x720)와 무관하게 지정 크기를 요구한다."""
    from operator_console.app import FixedSizeSlot

    big = Gtk.DrawingArea()
    big.set_size_request(1280, 720)  # gtksink 가 영상 해상도를 요구하는 상황
    slot = FixedSizeSlot()
    slot.add(big)
    slot.set_slot_size(360, 204)
    # GTK3 는 보이지 않는 위젯의 preferred size 를 0 으로 돌려주므로 반드시
    # show_all() 뒤에 물어야 한다.  이걸 빠뜨리면 구현이 옳아도 (0,0) 이 나온다.
    slot.show_all()

    minimum_w, natural_w = slot.get_preferred_width()
    minimum_h, natural_h = slot.get_preferred_height()
    assert (minimum_w, natural_w) == (360, 360)
    assert (minimum_h, natural_h) == (204, 204)


@requires_gtk
def test_pip_slot_allocation_stays_fixed_when_large_child_is_swapped():
    """848x480 자식을 1280x720 자식으로 바꿔도 실제 슬롯 할당은 그대로다."""
    from operator_console.app import FixedSizeSlot

    window = Gtk.OffscreenWindow()
    frame = Gtk.Frame()
    slot = FixedSizeSlot()
    first = Gtk.DrawingArea()
    first.set_size_request(848, 480)
    slot.add(first)
    slot.set_slot_size(360, 204)
    frame.add(slot)
    window.add(frame)
    window.show_all()
    while Gtk.events_pending():
        Gtk.main_iteration()
    assert (slot.get_allocated_width(), slot.get_allocated_height()) == (360, 204)

    slot.remove(first)
    second = Gtk.DrawingArea()
    second.set_size_request(1280, 720)
    slot.add(second)
    window.show_all()
    while Gtk.events_pending():
        Gtk.main_iteration()
    assert (slot.get_allocated_width(), slot.get_allocated_height()) == (360, 204)
    window.destroy()


def test_operator_console_pip_size_and_swap_use_fixed_slot():
    """크기 계산과 스왑 양쪽이 프레임이 아니라 고정 슬롯을 소유해야 한다."""
    import inspect

    from operator_console.app import OperatorConsole

    constructor = inspect.getsource(OperatorConsole.__init__)
    allocated = inspect.getsource(OperatorConsole._on_video_area_allocated)
    swap = inspect.getsource(OperatorConsole.swap_camera_views)
    assert "self._pip_slot = FixedSizeSlot()" in constructor
    assert "self._pip_slot.add(self._d435)" in constructor
    assert "self._pip_slot.set_slot_size(pip_width, pip_height)" in allocated
    assert "self._pip_slot.remove(selected)" in swap
    assert "self._pip_slot.add(secondary)" in swap


# NOTE: 여기서 실 VideoPanel 을 만들면 안 된다.  Gst.init(None) 은 app.main()
# 에서만 호출되므로, 테스트가 VideoPanel 을 직접 생성하면 초기화되지 않은
# GStreamer 위에서 Gst.parse_launch 가 돌아 **세그폴트로 스위트 전체가 죽는다**
# (2026-07-29 실측: app.py:1046 에서 코어 덤프).  클릭 스왑 경로 제거는 아래
# 소스 수준 가드로 확인하고, 실제 클릭 동작은 실기 라이브 검증에서 본다.
def test_swap_is_reachable_from_the_pip_control():
    """VideoPanel 전체 클릭은 없고 작은 영상 버튼과 V 키만 스왑한다."""
    import inspect

    from operator_console import app as console_app

    panel_source = inspect.getsource(console_app.VideoPanel)
    assert "_on_swap_click" not in panel_source, (
        "영상 패널에서 클릭 스왑 경로가 남아 있다"
    )
    assert "set_swap_handler" not in panel_source
    assert "_swap_handler" not in panel_source
    assert "클릭하여 크게 보기" not in panel_source

    constructor = inspect.getsource(console_app.OperatorConsole.__init__)
    key_handler = inspect.getsource(console_app.OperatorConsole._on_key_press)
    assert 'Gtk.Button(label="화면 전환")' in constructor
    assert "self._d435.set_header_action(self._swap_button)" in constructor
    assert "pip_overlay.add_overlay(self._swap_button)" not in constructor
    assert "display_options.pack_start(self._swap_button" not in constructor
    assert "Gdk.KEY_v" in key_handler and "Gdk.KEY_V" in key_handler
    assert "user_initiated=True" in constructor
    assert "user_initiated=True" in key_handler
    assert "secondary.set_header_action(self._swap_button)" in inspect.getsource(
        console_app.OperatorConsole.swap_camera_views
    )


@requires_gtk
def test_event_filters_hide_and_restore_existing_rows_independently():
    log = EventLog()
    log.add_event("SYSTEM", "CRITICAL fault")
    log.add_event("SYSTEM", "WARN waiting")
    log.add_event("SYSTEM", "NOTICE ready")
    assert len(log._entries) == 3
    assert len(_visible_rows(log)) == 3

    for severity in ("ERROR", "WARNING", "INFO"):
        toggle = log._filters[severity]
        toggle.set_active(False)
        assert log.event_filter_state[severity] is False
        assert len(_visible_rows(log)) == 2
        toggle.set_active(True)
        assert log.event_filter_state[severity] is True
        assert len(_visible_rows(log)) == 3


@requires_gtk
def test_event_filters_all_off_show_empty_without_clearing_buffer():
    log = EventLog()
    log.add_event("SYSTEM", "ERROR fault")
    original = tuple(log._entries)
    for toggle in log._filters.values():
        toggle.set_active(False)

    assert _visible_rows(log) == []
    assert log._empty.get_visible()
    assert tuple(log._entries) == original


@requires_gtk
def test_event_latest_summary_respects_selected_levels():
    log = EventLog()
    log.add_event("SYSTEM", "ERROR fault")
    log.add_event("SYSTEM", "NOTICE ready")

    log._filters["INFO"].set_active(False)
    severity, _message = log.latest_public()
    assert severity == "ERROR"

    log._filters["ERROR"].set_active(False)
    assert log.latest_public() == (
        "INFO", "선택한 분류의 기록이 없습니다",
    )

    log._filters["WARNING"].set_active(False)
    assert log.latest_public() == (
        "INFO", "표시할 이벤트 분류를 선택해 주세요",
    )


def test_public_event_copy_hides_camera_model_names():
    assert EventLog._public_message(
        "L515", "L515 SRT transport normal",
    ) == "전방 화면 상태를 확인하고 있습니다"
    assert EventLog._public_message(
        "D435i", "frame flow live",
    ) == "작업 화면 영상이 연결되었습니다"


@requires_gtk
def test_status_summary_cards_select_one_detail_panel():
    dashboard = RobotStatusDashboard()
    assert all(
        isinstance(button, Gtk.Button)
        for button in dashboard._card_buttons.values()
    )
    assert dashboard.view_enabled("drive")
    assert not dashboard.view_enabled("power")

    dashboard._card_buttons["safety"].clicked()
    assert dashboard.view_enabled("safety")
    assert not dashboard.view_enabled("drive")
    assert dashboard._detail_title.get_text() == "안전 상세 정보"
    assert dashboard._card_buttons["safety"].get_style_context().has_class(
        "selected"
    )

    dashboard._card_buttons["network"].clicked()
    assert dashboard.view_enabled("network")
    assert dashboard._detail_title.get_text() == "카메라·통신·AI 상태 상세 정보"


@requires_gtk
def test_real_power_packet_updates_voltage_ring_buffer():
    dashboard = RobotStatusDashboard()
    packet = json.dumps({
        "schema_version": 1,
        "sequence": 4,
        "voltage_v": 47.6,
        "pdist_soc_percent": 80,
        "pdist_protection_flags": 0,
        "pdist_battery_flags": 0,
        "rs485_state": "OK",
    }).encode()
    snapshot = parse_telemetry(packet, received_monotonic_s=10.0)
    dashboard.update(
        power=snapshot, chassis=None, arm=None, metadata=None,
        front_video_state="WAITING", work_video_state="WAITING",
        front_fps=None, work_fps=None, now_s=10.0,
    )

    samples = dashboard.power_voltage.samples()
    assert samples[-1].value == 47.6
    assert dashboard._soc_bar.get_fraction() == pytest.approx(0.8)


@requires_gtk
def test_power_panel_does_not_derive_unreported_power_or_operating_state():
    dashboard = RobotStatusDashboard()
    packet = json.dumps({
        "schema_version": 1,
        "sequence": 5,
        "voltage_v": 48.0,
        "current_a": 2.0,
        "power_w": None,
        "pdist_soc_percent": 80,
        "pdist_protection_flags": None,
        "pdist_battery_flags": None,
        "rs485_state": "OK",
    }).encode()
    snapshot = parse_telemetry(packet, received_monotonic_s=10.0)
    dashboard.update(
        power=snapshot, chassis=None, arm=None, metadata=None,
        front_video_state="WAITING", work_video_state="WAITING",
        front_fps=None, work_fps=None, now_s=10.0,
    )

    assert dashboard._power_metrics["power"].get_text() == "정보 없음"
    assert dashboard._power_metrics["operating"].get_text() == "정보 없음"


@requires_gtk
@pytest.mark.parametrize('drive_state', ['IDLE/RUN', 'ARMED/RUN'])
def test_producer_non_estop_modes_remain_ready(drive_state):
    dashboard = _update_competition_dashboard(chassis_overrides={'drive_state': drive_state})
    assert dashboard._system_overall.get_text() == '●  운용 준비 완료'


@requires_gtk
def test_producer_manual_estop_is_visible_even_with_clear_us100_verdict():
    dashboard = _update_competition_dashboard(
        chassis_overrides={'drive_state': 'ESTOP/ESTOP', 'safety_estop_required': False},
    )
    assert dashboard._system_overall.get_text() == '●  확인 필요한 항목 있음'
    assert dashboard._system_badges['safety'].get_text() != '정상'
    assert dashboard._system_values['safety_estop'].get_text() == '비상정지'


@requires_gtk
def test_producer_wheel_fault_without_axis_error_is_not_ready():
    dashboard = _update_competition_dashboard(
        wheel_overrides={'mode': 'FAULT', 'drive_axis_error': 0, 'steer_fault': 0},
    )
    assert dashboard._system_overall.get_text() == '●  확인 필요한 항목 있음'
