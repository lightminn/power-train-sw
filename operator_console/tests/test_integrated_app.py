"""Real GTK wiring; run with xvfb-run and system Python (no hardware)."""
import os
import time

# xvfb-run must not inherit the host Wayland display.
os.environ["GDK_BACKEND"] = "x11"
os.environ.pop("WAYLAND_DISPLAY", None)

import gi
import pytest

gi.require_version('Gtk', '3.0')
gi.require_version('Gst', '1.0')
from gi.repository import Gtk, Gst, GLib, Gdk
from operator_console import app
from operator_console.operation_runtime import OperationRuntime
from operator_console.tests.test_operation_runtime import Session, Child


def pump(seconds=.15):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        while GLib.MainContext.default().pending():
            GLib.MainContext.default().iteration(False)
        time.sleep(.005)


@pytest.fixture
def console(tmp_path):
    Gst.init(None)
    session, child = Session(), Child()
    token = tmp_path / 'token'
    token.write_text('fixture-token')
    runner = OperationRuntime({}, session_factory=lambda _: session, supervisor=child)
    window = None
    try:
        window = app.OperatorConsole('127.0.0.1', 51002, 51000, 0, 60, 0, 0,
                                     arm_telemetry_port=0, ops_token_file=str(token),
                                     operation_runtime=runner)
        window.show_all()
        pump(.3)
        yield window, runner, session, child
    finally:
        if window is not None:
            window.destroy()
        runner.close()


def front_frame(window):
    # This fixture controls freshness at the video health boundary; it does
    # not establish decoded real SRT or physical camera acceptance.
    window._l515._pipeline_live = True
    window._l515._last_frame_monotonic = time.monotonic()
    window._refresh_operation()


def test_integrated_boot_keeps_start_disabled_and_preserves_estop(console):
    window, runner, session, _ = console
    assert not window._operation_panel.start_button.get_sensitive()
    assert not session.operations
    assert 'estop' in window._ops_panel._action_buttons
    assert 'estop_reset' in window._ops_panel._action_buttons
    for action in ('arm', 'disarm', 'authority_manual', 'clear_transient_hold'):
        assert action not in window._ops_panel._action_buttons
    assert '전방' in window._operation_panel.video_label.get_text()


@pytest.mark.parametrize('cancel', ['release', 'window_focus', 'button_focus'])
def test_real_gtk_hold_cancel_never_starts(console, cancel):
    window, runner, session, _ = console
    front_frame(window)
    button = window._operation_panel.start_button
    assert button.get_sensitive()
    button.emit('pressed')
    pump(.2)
    assert 'start_begin' in session.operations
    # Readiness refresh must not disable a depressed button and lose release.
    assert button.get_sensitive()
    if cancel == 'release':
        button.emit('released')
    else:
        event = Gdk.Event.new(Gdk.EventType.FOCUS_CHANGE)
        event.in_ = False
        (window if cancel == 'window_focus' else button).emit('focus-out-event', event)
    pump(.25)
    assert not runner.snapshot()['holding']
    assert 'start_cancel' in session.operations
    assert 'start' not in session.operations


def test_stop_button_uses_integrated_stop_and_disconnect_clears_video(console):
    window, runner, session, _ = console
    front_frame(window)
    window._operation_panel.stop_button.emit('clicked')
    pump(.2)
    assert 'stop' in session.operations
    session.available = False
    pump(.3)
    assert not runner.snapshot()['ready']
    assert window._l515._last_frame_monotonic is None
    assert not window._operation_panel.start_button.get_sensitive()


def test_authenticated_host_retargets_both_real_srt_sources(console):
    window, runner, session, _ = console
    front_frame(window)
    session.host = '127.0.0.2'
    pump(.3)
    for panel, port in ((window._l515, 51000), (window._d435, 51002)):
        source = panel._pipeline.get_by_name('operator_source')
        from urllib.parse import urlparse, parse_qs
        uri = urlparse(source.get_property('uri'))
        assert uri.hostname == '127.0.0.2' and uri.port == port
        assert parse_qs(uri.query) == {'mode': ['caller'], 'latency': ['60']}
        assert panel._last_frame_monotonic is None
    assert not runner.snapshot()['ready']


def test_keyboard_hold_starts_once_and_release_after_threshold_keeps_transaction(console):
    window, runner, session, _ = console
    front_frame(window)
    button = window._operation_panel.start_button
    event = Gdk.Event.new(Gdk.EventType.KEY_PRESS)
    event.keyval = Gdk.KEY_space
    button.emit('key-press-event', event)
    deadline = time.monotonic() + 1.9
    while time.monotonic() < deadline:
        front_frame(window)
        button.emit('key-press-event', event)  # keyboard auto-repeat
        pump(.05)
    assert session.operations.count('start') == 1
    event = Gdk.Event.new(Gdk.EventType.KEY_RELEASE)
    event.keyval = Gdk.KEY_space
    button.emit('key-release-event', event)
    pump(.2)
    assert 'start_cancel' not in session.operations
    assert not runner.snapshot()['holding']


def test_latest_start_outcome_remains_visible_after_connection_loss(console):
    window, _, session, _ = console
    session.snapshot.update(action='start', status='REJECTED', detail='비상정지 확인 필요')
    pump(.25)
    assert '비상정지 확인 필요' in window._operation_panel.outcome_label.get_text()
    session.available = False
    pump(.3)
    assert '비상정지 확인 필요' in window._operation_panel.outcome_label.get_text()
    assert '연결 대기' in window._operation_panel.connection_label.get_text()
