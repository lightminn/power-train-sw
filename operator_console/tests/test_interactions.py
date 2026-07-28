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
