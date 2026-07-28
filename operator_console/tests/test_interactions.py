import json

import gi
import pytest

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

from operator_console.app import EventLog
from operator_console.metadata import (
    Detection,
    DisplayTargetTracker,
    MetadataFrame,
)
from operator_console.status_view import RobotStatusDashboard
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
def test_swap_is_reachable_only_from_the_explicit_control():
    """VideoPanel 클릭 경로는 없고 버튼과 V 키 경로만 명시 스왑을 요청한다."""
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
    assert "주/보조 화면 교체" in constructor, "명시 스왑 버튼이 없다"
    assert "Gdk.KEY_v" in key_handler and "Gdk.KEY_V" in key_handler
    assert "user_initiated=True" in constructor
    assert "user_initiated=True" in key_handler


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
def test_status_toggle_buttons_are_multi_select_and_compact():
    dashboard = RobotStatusDashboard()
    assert all(
        isinstance(button, Gtk.ToggleButton)
        and not isinstance(button, Gtk.CheckButton)
        for button in dashboard._toggles.values()
    )
    dashboard._toggles["safety"].set_active(True)
    assert dashboard.view_enabled("drive")
    assert dashboard.view_enabled("power")
    assert dashboard.view_enabled("safety")

    for button in dashboard._toggles.values():
        button.set_active(False)
    assert dashboard._no_panels.get_visible()
    assert not dashboard._panel_flow.get_visible()


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
