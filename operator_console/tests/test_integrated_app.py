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
from operator_console import __main__ as integrated_entrypoint
from operator_console.ops_panel import ConfirmFlow, PANEL_ACTIONS
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


def test_recovery_window_exposes_existing_ops_without_hiding_drive_controls(
    console, monkeypatch,
):
    window, _, session, _ = console
    operations_before = list(session.operations)
    ops_submissions = []
    monkeypatch.setattr(
        window._ops_panel._client, 'submit',
        lambda **kwargs: ops_submissions.append(kwargs),
    )

    window._ops_settings_button.clicked()
    pump()

    assert session.operations == operations_before
    assert ops_submissions == []
    assert [
        window._stack.child_get_property(page, 'name')
        for page in window._stack.get_children()
    ] == ['mission', 'systems']
    assert window._ops_panel._action_buttons['estop_reset'].get_mapped()
    assert window._ops_panel._action_buttons['us100_enable'].get_mapped()
    assert window._ops_panel._action_buttons['steer_mode_skid'].get_mapped()
    assert window._operation_panel.start_button.get_mapped()
    assert window._global_estop.get_mapped()


def test_recovery_window_hide_cancels_confirmation_and_reuses_panel(console):
    window, _, _, _ = console
    panel = window._ops_panel
    client = panel._client
    state = {
        'revision': 7,
        'authority_mode': 'IDLE',
        'chassis_mode': 'ESTOP',
        'estop_latched': True,
        'active_estop_sources': [],
        'component_mask': {
            'drive': True, 'steer': True, 'us100': True, 'robot_arm': True,
        },
        'wheels_stopped': True,
        'steering_mode': 'ackermann',
        'steering_available': True,
    }
    panel._flow = ConfirmFlow(clock=time.monotonic, state_provider=lambda: state)

    window._ops_settings_button.clicked()
    action = next(item for item in PANEL_ACTIONS if item.action == 'estop_reset')
    assert panel._begin(action)
    assert panel._active_action is action

    window._ops_settings_window.close()
    pump()
    assert not window._ops_settings_window.get_visible()
    assert panel._active_action is None
    assert not panel._confirm_strip.get_visible()

    window._ops_settings_button.clicked()
    pump()
    assert window._ops_settings_window.get_visible()
    assert window._ops_panel is panel
    assert window._ops_panel._client is client


def test_parent_shutdown_destroys_recovery_window(console):
    window, _, _, _ = console
    settings = window._ops_settings_window
    destroyed = []
    settings.connect('destroy', lambda _widget: destroyed.append(True))
    window._ops_settings_button.clicked()
    pump()

    window.destroy()
    pump()

    assert destroyed == [True]


def test_integrated_controls_use_scoped_dark_theme(console):
    window, _, _, _ = console
    window._ops_settings_button.clicked()
    pump()

    assert window._operation_panel.get_style_context().has_class(
        'integrated-operation'
    )
    assert window._ops_settings_window.get_style_context().has_class(
        'ops-settings-window'
    )
    assert window._ops_panel.get_style_context().has_class(
        'ops-settings-panel'
    )
    label_color = window._operation_panel.connection_label.get_style_context().get_color(
        Gtk.StateFlags.NORMAL
    )
    assert min(label_color.red, label_color.green, label_color.blue) > 0.65
    with pytest.warns(DeprecationWarning, match='get_background_color'):
        panel_background = (
            window._ops_panel.get_style_context().get_background_color(
                Gtk.StateFlags.NORMAL
            )
        )
    assert max(
        panel_background.red, panel_background.green, panel_background.blue,
    ) < 0.20


def test_integrated_console_fits_1366_by_768_laptop_view(console):
    window, _, _, _ = console
    window.resize(1366, 768)
    pump(.3)

    minimum, _natural = window.get_preferred_height()
    assert minimum <= 768
    assert window.get_size().height <= 768
    assert window._mission_event_expander.get_mapped()
    assert window._pip_frame.get_mapped()


def test_mission_latest_event_uses_its_own_active_filters(console):
    window, _, _, _ = console
    window._add_event('SYSTEM', 'ERROR 구동 고장')
    window._add_event('SYSTEM', 'NOTICE 상태 수신')
    window._mission_events._filters['INFO'].set_active(False)

    window._refresh_health()

    assert '구동 고장' in window._mission_event_latest.get_text()


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


def test_integrated_entrypoint_forwards_environment_port(monkeypatch, tmp_path):
    token = tmp_path / 'token'
    token.write_text('fixture-token')
    config = {
        'token_file': str(token),
        'environment_telemetry_port': 15008,
    }
    captured = {}

    class FakeRuntime:
        def __init__(self, received):
            captured['runtime_config'] = received

        def close(self):
            captured['runtime_closed'] = captured.get('runtime_closed', 0) + 1

    class FakeLock:
        def __init__(self, _path):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

    class FakeConsole:
        def __init__(self, *args, **kwargs):
            captured['console_args'] = args
            captured['console_kwargs'] = kwargs

        def show_all(self):
            captured['shown'] = True

        def destroy(self):
            captured['destroyed'] = True

    monkeypatch.setattr(integrated_entrypoint, 'load_config', lambda *_args, **_kwargs: config)
    monkeypatch.setattr(integrated_entrypoint, 'OperationRuntime', FakeRuntime)
    monkeypatch.setattr(integrated_entrypoint, 'ConsoleInstanceLock', FakeLock)
    monkeypatch.setattr(app, 'OperatorConsole', FakeConsole)
    monkeypatch.setattr(app.Gtk, 'main', lambda: None)
    monkeypatch.setattr(app, '_add_unix_signal_watch', lambda *_args: None)

    assert integrated_entrypoint.main(['--config', str(tmp_path / 'operator.json')]) == 0
    assert captured['console_kwargs']['environment_telemetry_port'] == 15008
    assert captured['shown'] is True
    assert captured['runtime_closed'] == 1


@pytest.mark.parametrize('invalid_port', [0, 65536, True, '5008'])
def test_integrated_entrypoint_rejects_invalid_environment_port_before_runtime(
    monkeypatch, tmp_path, invalid_port, capsys,
):
    token = tmp_path / 'token'
    token.write_text('fixture-token')
    config = {
        'token_file': str(token),
        'environment_telemetry_port': invalid_port,
    }
    runtime_started = []

    monkeypatch.setattr(
        integrated_entrypoint, 'load_config', lambda *_args, **_kwargs: config,
    )
    monkeypatch.setattr(
        integrated_entrypoint, 'OperationRuntime',
        lambda _config: (
            runtime_started.append(True),
            pytest.fail('runtime must not start for an invalid port'),
        )[1],
    )

    assert integrated_entrypoint.main(['--config', str(tmp_path / 'operator.json')]) == 2
    assert runtime_started == []
    assert 'environment_telemetry_port' in capsys.readouterr().err
