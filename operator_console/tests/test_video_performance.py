"""Exercise overload on the actual decoded-frame queue, without camera hardware."""
import os
from pathlib import Path
import subprocess
import time

import gi
import pytest

gi.require_version('Gst', '1.0')
from gi.repository import Gst

from operator_console.pipelines import pipeline_description


def test_heartbeat_gate_detects_missing_callbacks_at_both_boundaries():
    from operator_console.tests.vision_benchmark import heartbeat_gap
    assert heartbeat_gap(10., 12., []) == 2.
    assert heartbeat_gap(10., 12., [10.01, 10.02]) > 1.9
    assert heartbeat_gap(10., 12., [11.98, 11.99]) > 1.9
    assert heartbeat_gap(10., 12., [10.01, 11.99]) > 1.9


def test_benchmark_can_force_x11_before_gtk_import(monkeypatch):
    from operator_console.tests.vision_benchmark import configure_backend

    monkeypatch.setenv('GDK_BACKEND', 'wayland')
    monkeypatch.setenv('WAYLAND_DISPLAY', 'wayland-0')
    configure_backend(True)
    assert os.environ['GDK_BACKEND'] == 'x11'
    assert 'WAYLAND_DISPLAY' not in os.environ


def test_console_caps_inherited_blas_before_numpy_import():
    pytest.importorskip('threadpoolctl', reason='optional BLAS runtime inspection dependency')
    result = subprocess.run(['/usr/bin/python3', '-c',
        'import operator_console; import numpy as np; '
        'import os; assert os.environ["OPENBLAS_NUM_THREADS"] == "1"; '
        'np.show_runtime()'], env=dict(os.environ, OPENBLAS_NUM_THREADS='8'),
        capture_output=True, text=True, check=True)
    assert "'num_threads': 1" in result.stdout, result.stdout
    service = Path('scripts/systemd/powertrain-operator-console.service').read_text()
    assert 'Environment=OPENBLAS_NUM_THREADS=1' in service


def test_decoded_frame_overload_discards_old_frames_and_bounds_memory():
    Gst.init(None)
    # Reuse the real production tail, replacing only GTK output with a slow sink.
    tail = pipeline_description('localhost', 5000, 60).split('avdec_h264 max-threads=1 ! ', 1)[1]
    tail = tail.replace('gtksink name=video_sink sync=false force-aspect-ratio=true',
                        'identity sleep-time=50000 ! fakesink name=video_sink sync=false')
    pipeline = Gst.parse_launch(
        'videotestsrc is-live=true num-buffers=120 ! '
        'video/x-raw,width=160,height=90,framerate=60/1 ! '+tail)
    queue = pipeline.get_by_name('video_latest')
    timestamps = []
    def received(pad, info):
        timestamps.append(info.get_buffer().pts/Gst.SECOND)
        return Gst.PadProbeReturn.OK
    pipeline.get_by_name('video_sink').get_static_pad('sink').add_probe(
        Gst.PadProbeType.BUFFER, received)
    started = time.monotonic()
    try:
        pipeline.set_state(Gst.State.PLAYING)
        message = pipeline.get_bus().timed_pop_filtered(9*Gst.SECOND,
                                                       Gst.MessageType.EOS | Gst.MessageType.ERROR)
        assert message and message.type == Gst.MessageType.EOS
        assert time.monotonic()-started < 4
        assert 20 <= len(timestamps) < 80, len(timestamps)
        assert timestamps[-1] > 1.9  # retain the newest frame, not the oldest backlog
        assert queue.get_property('current-level-buffers') <= 1
    finally:
        pipeline.set_state(Gst.State.NULL)


def test_streaming_probe_keeps_only_latest_values_without_gtk_idle_backlog(monkeypatch):
    from types import SimpleNamespace
    from operator_console.app import VideoPanel, GLib
    def unexpected(*args, **kwargs):
        raise AssertionError('streaming thread must not queue GTK notifications')
    monkeypatch.setattr(GLib, 'idle_add', unexpected)
    caps = Gst.Caps.from_string('video/x-raw,width=1280,height=720')
    pad = SimpleNamespace(get_current_caps=lambda: caps)
    model = SimpleNamespace(_frames=0, _sample_start=0., _video_size=None,
                            _last_frame_monotonic=None, _last_fps=None,
                            _fresh_frame_run=0)
    for _ in range(2000):
        assert VideoPanel._on_video_buffer(model, pad, None) == Gst.PadProbeReturn.OK
    assert model._video_size == (1280, 720)
    assert model._last_frame_monotonic > 0
    assert model._last_fps is not None
    assert model._fresh_frame_run == 2000


def test_gui_demo_chassis_values_are_explicitly_synthetic_and_parseable():
    import json
    from operator_console.telemetry import parse_telemetry
    from operator_console.tests.vision_benchmark import demo_chassis_payload

    payload = demo_chassis_payload(7)
    snapshot = parse_telemetry(json.dumps(payload).encode(), 1.0)
    assert snapshot.suspension_source == 'TEST 더미 AS5048B'
    assert len(snapshot.suspension_joints_rad) == 4
    assert len(snapshot.wheel_statuses) == 6


class _StalledVideo:
    """VideoPanel health fields after three fresh frames, then a silent stall."""

    def __init__(self):
        from types import SimpleNamespace
        label = lambda: SimpleNamespace(set_text=lambda text: None, get_text=lambda: '')
        self._stopped = False
        self._video_size = None
        self._published_video_size = None
        self._last_fps = None
        self._fps = label()
        self._status = label()
        self._header_state = label()
        self._placeholder = SimpleNamespace(show_all=lambda: None, hide=lambda: None)
        self._name = '전방 카메라'
        self._pipeline_live = True
        self._last_frame_monotonic = 100.0
        self._fresh_frame_run = 3
        self._freshness_state = 'live'
        self._retry_source_id = None
        self._stale_retry_pending = False
        self._sample_start = 90.0
        self.reconnects = 0
        self.events = []

    def _publish_video_live(self, live):
        pass

    def _emit_event(self, text):
        self.events.append(text)

    def _schedule_reconnect(self):
        self.reconnects += 1
        self._retry_source_id = 77


def _health_tick(monkeypatch, model, now_s):
    from operator_console import app
    from operator_console.app import VideoPanel
    monkeypatch.setattr(app.time, 'monotonic', lambda: now_s)
    model.health_state = lambda: VideoPanel.health_state(model)
    model._watchdog_reconnect = lambda reason: VideoPanel._watchdog_reconnect(model, reason)
    return VideoPanel._refresh_video_health(model)


def test_silent_video_stall_still_reaches_the_reconnect_watchdog(monkeypatch):
    # srtsrc can swallow a dead listener without ERROR/EOS; only the stale
    # watchdog recovers it, so the 3-frame warm-up must not hide the stall.
    model = _StalledVideo()
    for now_s in (100.2, 101.5, 102.5, 103.5, 104.5):
        _health_tick(monkeypatch, model, now_s)
    assert model.reconnects == 0
    _health_tick(monkeypatch, model, 105.2)
    assert model.reconnects == 1


def test_stalled_video_reports_stale_not_waiting(monkeypatch):
    from operator_console import app
    from operator_console.app import VideoPanel
    model = _StalledVideo()
    _health_tick(monkeypatch, model, 101.5)
    monkeypatch.setattr(app.time, 'monotonic', lambda: 102.0)
    assert VideoPanel.health_state(model) == 'STALE'


def test_restart_without_any_first_frame_still_retries(monkeypatch):
    # PLAYING but no frame, ERROR or EOS after a restart: retry once the
    # stale limit passes instead of waiting forever.
    model = _StalledVideo()
    model._last_frame_monotonic = None
    model._fresh_frame_run = 0
    model._sample_start = 100.0
    _health_tick(monkeypatch, model, 104.9)
    assert model.reconnects == 0
    _health_tick(monkeypatch, model, 105.2)
    assert model.reconnects == 1


def test_watchdog_retry_is_cancelled_when_frames_recover(monkeypatch):
    from operator_console import app
    removed = []
    monkeypatch.setattr(app.GLib, 'source_remove', removed.append)
    model = _StalledVideo()
    _health_tick(monkeypatch, model, 105.2)
    assert model.reconnects == 1 and model._retry_source_id == 77
    model._last_frame_monotonic = 105.3
    model._fresh_frame_run = 3
    _health_tick(monkeypatch, model, 105.4)
    assert removed == [77]
    assert model._retry_source_id is None


def test_error_path_retry_is_not_cancelled_by_frames(monkeypatch):
    from operator_console import app
    removed = []
    monkeypatch.setattr(app.GLib, 'source_remove', removed.append)
    model = _StalledVideo()
    model._retry_source_id = 55  # scheduled by a bus ERROR/EOS, not the watchdog
    _health_tick(monkeypatch, model, 100.2)
    assert removed == [] and model._retry_source_id == 55


def test_watchdog_restart_callback_skips_when_frames_already_recovered(monkeypatch):
    from operator_console import app
    from operator_console.app import VideoPanel
    states = []
    model = _StalledVideo()
    model._pipeline = type('P', (), {'set_state': lambda self, s: states.append(s)})()
    model._stale_retry_pending = True
    model._retry_source_id = 77
    model._last_frame_monotonic = 106.0
    model._fresh_frame_run = 3
    monkeypatch.setattr(app.time, 'monotonic', lambda: 106.1)
    assert VideoPanel._restart_pipeline(model) is False
    assert states == []
    assert model._retry_source_id is None and model._stale_retry_pending is False


def test_preroll_without_any_frame_still_retries(monkeypatch):
    # srtsrc stuck in PAUSED (pending PLAYING) never sets _pipeline_live.
    model = _StalledVideo()
    model._pipeline_live = False
    model._last_frame_monotonic = None
    model._fresh_frame_run = 0
    model._sample_start = 100.0
    _health_tick(monkeypatch, model, 104.9)
    assert model.reconnects == 0
    _health_tick(monkeypatch, model, 105.2)
    assert model.reconnects == 1


def test_endpoint_change_always_restarts_even_after_a_recovered_watchdog(monkeypatch):
    from operator_console import app
    from operator_console.app import VideoPanel
    from types import SimpleNamespace
    states = []
    source = SimpleNamespace(set_property=lambda *a: None)
    model = _StalledVideo()
    model._pipeline = SimpleNamespace(set_state=states.append,
                                      get_by_name=lambda name: source)
    model._endpoint = ('10.0.0.1', 5000, 120)
    model._stale_retry_pending = True
    model._retry_source_id = None
    model._last_frame_monotonic = 106.0
    model._fresh_frame_run = 3
    model._restart_pipeline = lambda: VideoPanel._restart_pipeline(model)
    monkeypatch.setattr(app.time, 'monotonic', lambda: 106.1)
    VideoPanel.set_endpoint(model, '10.0.0.2')
    assert states[-1] == app.Gst.State.PLAYING
