from types import SimpleNamespace
import sys
import threading
import time

from l515_dashboard.config import DashboardConfig
from l515_dashboard.gateway_source import (
    EXPECTED_L515_SERIAL,
    GatewayFrames,
    GatewaySourceState,
    L515GatewaySource,
)


class Profile:
    def __init__(self, stream): self._stream = stream
    def stream_type(self): return self._stream


class Frame:
    def __init__(self, stream, number):
        self.stream = stream; self.number = number; self.kept = 0
    def get_profile(self): return Profile(self.stream)
    def get_frame_number(self): return self.number
    def get_timestamp(self): return float(self.number)
    def keep(self): self.kept += 1


class Frameset:
    def __init__(self, *children): self.children = children
    def is_frameset(self): return True
    def as_frameset(self): return self
    def __iter__(self): return iter(self.children)


class Config:
    def __init__(self): self.serial = None; self.streams = []
    def enable_device(self, serial): self.serial = serial
    def enable_stream(self, *args): self.streams.append(args)


class Pipeline:
    def __init__(self, rs):
        self.rs = rs; self.callback = None; self.stop_calls = 0
    def start(self, config, callback):
        self.rs.started.append(config); self.callback = callback
    def stop(self): self.stop_calls += 1


class RS:
    stream = SimpleNamespace(color="color", depth="depth", accel="accel", gyro="gyro")
    format = SimpleNamespace(bgr8="bgr8", z16="z16")
    camera_info = SimpleNamespace(serial_number="serial")
    config = Config
    def __init__(self):
        self.started = []; self.pipelines = []; self.present = True
    def context(self):
        devices = [] if not self.present else [
            SimpleNamespace(get_info=lambda _: EXPECTED_L515_SERIAL)
        ]
        return SimpleNamespace(query_devices=lambda: devices)
    def pipeline(self):
        pipeline = Pipeline(self); self.pipelines.append(pipeline); return pipeline


def wait_until(predicate, timeout=.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate(): return True
        time.sleep(.002)
    return predicate()


def test_gateway_frames_latest_slot_and_empty():
    slot = GatewayFrames(); slot.put(raw_color="old"); slot.put(raw_color="new")
    assert slot.drain().raw_color == "new" and slot.drain().empty


def test_async_start_exact_profiles_and_independent_video_readers():
    rs = RS(); source = L515GatewaySource(rs, mapper_factory=object)
    source.start(); assert wait_until(lambda: source.state is GatewaySourceState.STREAMING)
    pipeline = rs.pipelines[0]
    color, depth = Frame("color", 1), Frame("depth", 2)
    pipeline.callback(Frameset(color, depth))
    color_sequence, color_sample = source.read_color_after(0)
    depth_sequence, depth_sample = source.read_depth_after(0)
    assert (color_sequence, color_sample.frame) == (1, color)
    assert (depth_sequence, depth_sample.frame) == (1, depth)
    assert rs.started[0].streams == [
        ("color", 1280, 720, "bgr8", 30),
        ("depth", 640, 480, "z16", 30), ("accel",), ("gyro",),
    ]
    assert source.connected_profile == {"color":[1280,720,30], "depth":[640,480,30]}
    source.stop()


def test_callback_splits_composite_deduplicates_by_stream_and_frame_number():
    source = L515GatewaySource(RS(), mapper_factory=object)
    source._generation = 3; source._stop_event.clear(); token = source._reset_capture(3)
    color, depth = Frame("color", 7), Frame("depth", 7)
    source._on_frame(Frameset(color, depth, color), 3, token)
    assert source.read_color_after(0)[1].frame_number == 7
    assert source.read_depth_after(0)[1].frame_number == 7
    assert color.kept == depth.kept == 1


def test_native_callback_rates_count_unique_frames_and_reset_on_capture():
    source = L515GatewaySource(RS(), mapper_factory=object)
    source._generation = 3; source._stop_event.clear(); token = source._reset_capture(3)
    source._on_frame(Frame("color", 1), 3, token)
    time.sleep(.002)
    source._on_frame(Frame("color", 2), 3, token)
    source._on_frame(Frame("color", 2), 3, token)

    rates = source.native_callback_rates()
    assert rates["color"] > 0
    assert rates["depth"] == rates["accel"] == rates["gyro"] == 0.0
    assert source.native_frame_stats()["color"] == {
        "count":2, "first":1, "last":2, "gap_count":0,
        "discontinuity_count":1, "duplicate_count":1}

    source._clear_capture(token)
    assert source.native_callback_rates() == {
        "color":0.0, "depth":0.0, "accel":0.0, "gyro":0.0}
    assert source.native_frame_stats()["color"] == {
        "count":0, "first":None, "last":None, "gap_count":0,
        "discontinuity_count":0, "duplicate_count":0}


def test_native_frame_stats_count_forward_device_number_gaps_only():
    source = L515GatewaySource(RS(), mapper_factory=object)
    source._generation = 1; source._stop_event.clear(); token = source._reset_capture(1)
    for number in (10, 11, 14, 9):
        source._on_frame(Frame("depth", number), 1, token)
    assert source.native_frame_stats()["depth"] == {
        "count":4, "first":10, "last":9, "gap_count":2,
        "discontinuity_count":1, "duplicate_count":0}


def test_native_frame_stats_separate_duplicate_backward_and_capture_reset():
    source = L515GatewaySource(RS(), mapper_factory=object)
    source._generation = 1; source._stop_event.clear(); token = source._reset_capture(1)
    for number in (20, 20, 19, 23):
        source._on_frame(Frame("color", number), 1, token)
    assert source.native_frame_stats()["color"] == {
        "count":3, "first":20, "last":23, "gap_count":3,
        "discontinuity_count":2, "duplicate_count":1}
    source._reset_capture(1)
    assert source.native_frame_stats()["color"] == {
        "count":0, "first":None, "last":None, "gap_count":0,
        "discontinuity_count":0, "duplicate_count":0}


def test_composite_callback_retains_real_latest_bundle_and_counts_overwrites():
    source = L515GatewaySource(RS(), mapper_factory=object)
    source._generation = 3; source._stop_event.clear(); token = source._reset_capture(3)
    first = Frameset(Frame("color", 1), Frame("depth", 1))
    second = Frameset(Frame("color", 2), Frame("depth", 2))
    first.kept = second.kept = 0
    first.keep = lambda: setattr(first, "kept", first.kept + 1)
    second.keep = lambda: setattr(second, "kept", second.kept + 1)
    source._on_frame(first, 3, token); source._on_frame(second, 3, token)
    sequence, bundle = source.read_video_bundle_after(0)
    assert sequence == 2 and bundle.frameset is second
    assert (bundle.generation, bundle.capture_token) == (3, token)
    assert first.kept == second.kept == 1
    assert source.video_bundle_overwrites == 1


def test_alignment_bundle_uses_native_composite_conversion_not_base_frame():
    source = L515GatewaySource(RS(), mapper_factory=object)
    source._generation = 3; source._stop_event.clear(); token = source._reset_capture(3)
    converted = Frameset(Frame("color", 1), Frame("depth", 1))
    class NativeBaseFrame:
        kept = 0
        def is_frameset(self): return True
        def keep(self): self.kept += 1
        def as_frameset(self): return converted
    base = NativeBaseFrame()

    source._on_frame(base, 3, token)

    assert source.read_video_bundle_after(0)[1].frameset is converted
    assert base.kept == 1


def test_source_overwrite_counters_do_not_count_consumed_samples():
    source = L515GatewaySource(RS(), mapper_factory=object)
    source._generation = 1; source._stop_event.clear(); token = source._reset_capture(1)
    def frameset(number):
        value = Frameset(Frame("color", number), Frame("depth", number))
        value.keep = lambda: None
        return value
    source._on_frame(frameset(1), 1, token)
    source.read_color_after(0); source.read_video_bundle_after(0)
    source._on_frame(frameset(2), 1, token)
    assert source.color_overwrites == source.video_bundle_overwrites == 0
    source._on_frame(frameset(3), 1, token)
    assert source.color_overwrites == source.video_bundle_overwrites == 1


def test_single_motion_frames_use_capacity_32_ring():
    source = L515GatewaySource(RS(), mapper_factory=object)
    source._generation = 2; source._stop_event.clear(); token = source._reset_capture(2)
    for number in range(40): source._on_frame(Frame("gyro", number), 2, token)
    result = source.read_gyro_after(0, 100)
    assert [sample.frame_number for sample in result.samples] == list(range(8, 40))
    assert source._buffers["gyro"].dropped == 8


def test_poll_latest_is_nonblocking_compatibility_view_over_stream_buffers():
    source = L515GatewaySource(RS(), mapper_factory=object)
    source._generation = 1; source._stop_event.clear(); token = source._reset_capture(1)
    color, accel = Frame("color", 1), Frame("accel", 2)
    source._on_frame(color, 1, token); source._on_frame(accel, 1, token)
    result = source.poll_latest()
    assert result.raw_color is color and result.accel is accel and result.mapper is not None
    assert source.poll_latest().empty


def test_default_mapper_is_real_timestamp_mapper(monkeypatch):
    sentinel = object()
    monkeypatch.setitem(sys.modules, "powertrain_ros.l515_adapter",
                        SimpleNamespace(TimestampMapper=lambda: sentinel))
    from l515_dashboard.gateway_source import _new_timestamp_mapper
    assert _new_timestamp_mapper() is sentinel


def test_disconnect_invalidates_buffers_mapper_and_reconnects():
    rs = RS(); config = DashboardConfig(reconnect_interval_s=.01)
    source = L515GatewaySource(rs, config=config, mapper_factory=object,
                               video_startup_grace_s=.02,
                               video_stale_timeout_s=.02)
    source.start(); assert wait_until(lambda: source.state is GatewaySourceState.STREAMING)
    old_pipeline = rs.pipelines[0]
    old_pipeline.callback(Frame("color", 1)); assert source.read_color_after(0)[1]
    assert wait_until(lambda: source.state is GatewaySourceState.DISCONNECTED)
    assert source.read_color_after(0)[1] is None and source.poll_latest().mapper is None
    assert wait_until(lambda: len(rs.pipelines) == 2 and source.state is GatewaySourceState.STREAMING)
    source.stop()


def test_active_stream_never_reenumerates_and_fresh_video_keeps_it_streaming():
    rs = RS(); config = DashboardConfig(reconnect_interval_s=.01)
    source = L515GatewaySource(rs, config=config, mapper_factory=object,
                               video_startup_grace_s=.02,
                               video_stale_timeout_s=.02)
    calls = 0
    original = source._matching_serial
    def prestart_only():
        nonlocal calls
        calls += 1
        if calls > 1:
            raise AssertionError("active stream re-enumerated RSUSB")
        return original()
    source._matching_serial = prestart_only
    source.start(); assert wait_until(lambda: source.state is GatewaySourceState.STREAMING)
    callback = rs.pipelines[0].callback
    for number in range(1, 6):
        callback(Frameset(Frame("color", number), Frame("depth", number)))
        time.sleep(.01)
    assert source.state is GatewaySourceState.STREAMING
    assert calls == 1
    source.stop()


def test_old_pipeline_callback_cannot_contaminate_reconnected_capture():
    rs = RS(); config = DashboardConfig(reconnect_interval_s=.01)
    source = L515GatewaySource(rs, config=config, mapper_factory=object,
                               video_startup_grace_s=.02,
                               video_stale_timeout_s=.02)
    source.start(); assert wait_until(lambda: source.state is GatewaySourceState.STREAMING)
    callback1 = rs.pipelines[0].callback
    assert wait_until(lambda: source.state is GatewaySourceState.DISCONNECTED)
    assert wait_until(lambda: len(rs.pipelines) == 2
                      and source.state is GatewaySourceState.STREAMING)
    callback2 = rs.pipelines[1].callback

    stale_color, stale_gyro = Frame("color", 8), Frame("gyro", 8)
    callback1(Frameset(stale_color, stale_gyro))
    assert source.read_color_after(0)[1] is None
    assert source.read_gyro_after(0, 10).samples == ()

    current_color, current_gyro = Frame("color", 8), Frame("gyro", 8)
    callback2(Frameset(current_color, current_gyro))
    assert source.read_color_after(0)[1].frame is current_color
    assert source.read_gyro_after(0, 10).samples[0].frame is current_gyro
    assert stale_color.kept == stale_gyro.kept == 0
    source.stop()


def test_late_callback_after_stop_is_rejected_and_every_buffer_is_empty():
    rs = RS(); source = L515GatewaySource(rs, stop_timeout=.02, mapper_factory=object)
    source.start(); assert wait_until(lambda: source.state is GatewaySourceState.STREAMING)
    callback = rs.pipelines[0].callback
    source.stop(); callback(Frame("color", 9))
    assert source.state is GatewaySourceState.STOPPED
    assert source.read_color_after(0)[1] is None
    assert source.read_gyro_after(0, 10).samples == ()


def test_stop_is_bounded_idempotent_and_stops_pipeline_once():
    class BlockingPipeline(Pipeline):
        def stop(self): self.stop_calls += 1; time.sleep(1)
    rs = RS(); pipeline = BlockingPipeline(rs); rs.pipeline = lambda: pipeline
    source = L515GatewaySource(rs, stop_timeout=.01, mapper_factory=object)
    source.start(); assert wait_until(lambda: source.state is GatewaySourceState.STREAMING)
    started = time.monotonic(); source.stop(); source.stop()
    assert time.monotonic() - started < .1 and pipeline.stop_calls == 1


def test_stop_immediately_before_native_start_prevents_sdk_start():
    ready, release = threading.Event(), threading.Event(); rs = RS()
    source = L515GatewaySource(rs, stop_timeout=.01, mapper_factory=object)
    source._before_pipeline_start = lambda: (ready.set(), release.wait())
    source.start(); assert ready.wait(.2); source.stop(); worker = source._thread
    release.set(); worker.join(.2)
    assert rs.started == [] and source.state is GatewaySourceState.STOPPED


def test_cancelled_native_start_is_cleaned_by_one_stop_attempt():
    entered, release = threading.Event(), threading.Event()
    class ActivePipeline(Pipeline):
        def start(self, config, callback):
            super().start(config, callback); entered.set(); release.wait()
    rs = RS(); pipeline = ActivePipeline(rs); rs.pipeline = lambda: pipeline
    source = L515GatewaySource(rs, stop_timeout=.01, mapper_factory=object)
    source.start(); assert entered.wait(.2); source.stop(); release.set()
    assert wait_until(lambda: pipeline.stop_calls == 1)
    source.stop(); assert pipeline.stop_calls == 1


def test_stop_during_device_query_prevents_pipeline_creation():
    entered, release = threading.Event(), threading.Event(); rs = RS()
    def context():
        def query(): entered.set(); release.wait(); return [SimpleNamespace(get_info=lambda _:EXPECTED_L515_SERIAL)]
        return SimpleNamespace(query_devices=query)
    rs.context = context
    source = L515GatewaySource(rs, stop_timeout=.01, mapper_factory=object)
    source.start(); assert entered.wait(.2); source.stop(); worker = source._thread
    release.set(); worker.join(.2)
    assert rs.pipelines == [] and rs.started == [] and source.state is GatewaySourceState.STOPPED
