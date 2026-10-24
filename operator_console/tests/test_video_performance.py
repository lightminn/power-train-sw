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
