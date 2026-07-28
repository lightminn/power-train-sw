import json

import gi
import pytest

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

from operator_console.app import EventLog
from operator_console.status_view import RobotStatusDashboard
from operator_console.telemetry import parse_telemetry


if not Gtk.init_check(None)[0]:
    pytest.skip("GTK display is unavailable", allow_module_level=True)


def _visible_rows(log: EventLog) -> list:
    return [
        row for row in log._operation_list.get_children()
        if row.get_visible()
    ]


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


def test_event_filters_all_off_show_empty_without_clearing_buffer():
    log = EventLog()
    log.add_event("SYSTEM", "ERROR fault")
    original = tuple(log._entries)
    for toggle in log._filters.values():
        toggle.set_active(False)

    assert _visible_rows(log) == []
    assert log._empty.get_visible()
    assert tuple(log._entries) == original


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
