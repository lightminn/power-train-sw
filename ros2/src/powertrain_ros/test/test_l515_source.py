from types import SimpleNamespace
import threading
import time

import pytest

from powertrain_ros.l515_source import (
    EXPECTED_L515_SERIAL,
    L515Config,
    L515Source,
    L515State,
    LatestFrames,
)


class FakeDevice:
    def __init__(self, serial):
        self.serial = serial

    def get_info(self, _):
        return self.serial


class FakePipeline:
    def __init__(self, rs):
        self.rs = rs
        self.stopped = False

    def start(self, config):
        self.rs.started.append(config)
        return object()

    def wait_for_frames(self):
        item = self.rs.results.pop(0)
        if isinstance(item, BaseException):
            raise item
        return item

    def stop(self):
        self.stopped = True


class FakeConfig:
    def __init__(self):
        self.serial = None
        self.streams = []

    def enable_device(self, serial):
        self.serial = serial

    def enable_stream(self, *args):
        self.streams.append(args)


class FakeRs:
    stream = SimpleNamespace(
        color="color", depth="depth", accel="accel", gyro="gyro"
    )
    format = SimpleNamespace(bgr8="bgr8", z16="z16")
    camera_info = SimpleNamespace(serial_number="serial_number")

    def __init__(self, serials, results=()):
        self.serials = list(serials)
        self.results = list(results)
        self.started = []
        self.pipelines = []

    def context(self):
        devices = [FakeDevice(serial) for serial in self.serials]
        return SimpleNamespace(query_devices=lambda: devices)

    def pipeline(self):
        pipeline = FakePipeline(self)
        self.pipelines.append(pipeline)
        return pipeline

    config = FakeConfig


class FakeFrames:
    def __init__(self, tag, on_read=None):
        self.color = f"color-{tag}"
        self.depth = f"depth-{tag}"
        self.accel = f"accel-{tag}"
        self.gyro = f"gyro-{tag}"
        self.on_read = on_read

    def get_color_frame(self):
        return self.color

    def get_depth_frame(self):
        return self.depth

    def first_or_default(self, stream):
        if stream == "gyro" and self.on_read is not None:
            self.on_read()
        return getattr(self, stream)


def test_config_is_immutable_and_rejects_empty_or_non_l515_serial():
    assert L515Config().serial == EXPECTED_L515_SERIAL
    with pytest.raises(ValueError, match="serial"):
        L515Config(serial="")
    with pytest.raises(ValueError, match="serial"):
        L515Config(serial="250222071245")
    with pytest.raises(Exception):
        L515Config().serial = "changed"


def test_latest_frames_overwrites_each_stream_and_drain_clears_slots():
    latest = LatestFrames()
    latest.put(color="old-color", depth="old-depth")
    latest.put(color="new-color", gyro="new-gyro")

    drained = latest.drain()

    assert drained.color == "new-color"
    assert drained.depth == "old-depth"
    assert drained.accel is None
    assert drained.gyro == "new-gyro"
    assert latest.drain().empty


def test_source_selects_only_expected_serial_and_enables_exact_streams():
    rs = FakeRs(
        ["250222071245", EXPECTED_L515_SERIAL],
        [RuntimeError("disconnect")],
    )
    waits = []
    source = L515Source(
        rs,
        wait_fn=lambda seconds: waits.append(seconds) or False,
        mapper_factory=object,
    )

    source._run()

    config = rs.started[0]
    assert config.serial == EXPECTED_L515_SERIAL
    assert config.streams == [
        ("color", 640, 480, "bgr8", 30),
        ("depth", 640, 480, "z16", 30),
        ("accel",),
        ("gyro",),
    ]
    assert waits == [2.0]


def test_source_canonicalizes_sdk_serial_and_opens_that_exact_device():
    rs = FakeRs(
        ["250222071245", "f0271544"],
        [RuntimeError("disconnect")],
    )
    source = L515Source(
        rs, wait_fn=lambda _: False, mapper_factory=object
    )

    source._run()

    assert rs.started[0].serial == "f0271544"


def test_source_never_starts_d435_when_expected_serial_is_absent():
    rs = FakeRs(["250222071245"])
    waits = []
    source = L515Source(
        rs, wait_fn=lambda seconds: waits.append(seconds) or False
    )

    source._run()

    assert rs.started == []
    assert waits == [2.0]
    assert source.state is L515State.DISCONNECTED


def test_disconnect_clears_stale_frames_and_reconnect_uses_new_mapper():
    waits = []
    mapper_values = [object(), object()]
    mappers = iter(mapper_values)

    def wait_fn(seconds):
        waits.append(seconds)
        return True

    rs = FakeRs([EXPECTED_L515_SERIAL])
    source = L515Source(
        rs, wait_fn=wait_fn, mapper_factory=lambda: next(mappers)
    )
    original_put = source._latest.put
    put_count = 0

    def put_then_stop(**payload):
        nonlocal put_count
        original_put(**payload)
        put_count += 1
        if put_count == 2:
            source._stop_event.set()

    source._latest.put = put_then_stop
    rs.results = [
        FakeFrames("old"),
        RuntimeError("disconnect"),
        FakeFrames("new"),
    ]
    source._run()

    payload = source.poll_latest()
    assert waits == [2.0]
    assert payload.color == "color-new"
    assert payload.depth == "depth-new"
    assert payload.timestamp_mapper is mapper_values[1]
    assert len(rs.started) == 2
    assert all(pipeline.stopped for pipeline in rs.pipelines)
    assert source.poll_latest().empty


def test_poll_latest_is_nonblocking_when_worker_has_no_data():
    source = L515Source(FakeRs([]), wait_fn=lambda _: False)

    assert source.poll_latest().empty


def test_state_transitions_use_injected_clock():
    ticks = iter([10.0, 11.0])
    source = L515Source(FakeRs([]), clock=lambda: next(ticks))

    source._set_state(L515State.CONNECTING)

    assert source.state_changed_at == 11.0


def test_stop_is_bounded_and_pipeline_stop_is_best_effort():
    class Worker:
        def __init__(self):
            self.timeout = None

        def is_alive(self):
            return True

        def join(self, timeout):
            self.timeout = timeout

    class BrokenPipeline:
        def stop(self):
            raise RuntimeError("already disconnected")

    source = L515Source(FakeRs([]), stop_timeout=0.25)
    source._thread = Worker()
    source._pipeline = BrokenPipeline()
    source._set_state(L515State.STREAMING)

    source.stop()

    assert source._thread.timeout == 0.25
    assert source.state is not L515State.STOPPED


def test_stop_does_not_block_on_blocking_sdk_stop():
    stop_entered = threading.Event()
    release_stop = threading.Event()

    class BlockingPipeline:
        def stop(self):
            stop_entered.set()
            release_stop.wait()

    source = L515Source(FakeRs([]), stop_timeout=0.02)
    source._pipeline = BlockingPipeline()

    started = time.monotonic()
    source.stop()
    elapsed = time.monotonic() - started

    assert stop_entered.wait(0.2)
    assert elapsed < 0.15
    release_stop.set()


def test_late_worker_cannot_enqueue_or_regress_state_after_stop():
    frames_ready = threading.Event()
    release_frames = threading.Event()

    class LatePipeline(FakePipeline):
        def wait_for_frames(self):
            frames_ready.set()
            release_frames.wait()
            return FakeFrames("late")

    rs = FakeRs([EXPECTED_L515_SERIAL])
    rs.pipeline = lambda: LatePipeline(rs)
    source = L515Source(rs, stop_timeout=0.01, mapper_factory=object)
    source.start()
    assert frames_ready.wait(0.2)

    source.stop()
    assert source.state is not L515State.STOPPED
    worker = source._thread
    release_frames.set()
    worker.join(0.2)

    assert source.state is L515State.STOPPED
    assert source.poll_latest().empty


def test_stop_during_device_query_prevents_pipeline_creation_and_start():
    query_entered = threading.Event()
    release_query = threading.Event()

    class RacingRs(FakeRs):
        def context(self):
            def query_devices():
                query_entered.set()
                release_query.wait()
                return [FakeDevice(EXPECTED_L515_SERIAL)]

            return SimpleNamespace(query_devices=query_devices)

    rs = RacingRs([EXPECTED_L515_SERIAL])
    source = L515Source(rs, stop_timeout=0.01)
    source.start()
    assert query_entered.wait(0.2)

    source.stop()
    worker = source._thread
    release_query.set()
    worker.join(0.2)

    assert rs.pipelines == []
    assert rs.started == []
    assert source.state is L515State.STOPPED


def test_stop_after_generation_check_prevents_frame_commit():
    before_commit = threading.Event()
    release_commit = threading.Event()

    class OneFramePipeline(FakePipeline):
        def wait_for_frames(self):
            return FakeFrames("racing")

    rs = FakeRs([EXPECTED_L515_SERIAL])
    rs.pipeline = lambda: OneFramePipeline(rs)
    source = L515Source(rs, stop_timeout=0.01, mapper_factory=object)

    def block_before_commit():
        before_commit.set()
        release_commit.wait()

    source._before_frame_commit = block_before_commit
    source.start()
    assert before_commit.wait(0.2)

    source.stop()
    worker = source._thread
    release_commit.set()
    worker.join(0.2)

    assert source.poll_latest().empty
    assert source.state is L515State.STOPPED


def test_stop_immediately_before_native_start_prevents_sdk_start():
    before_start = threading.Event()
    release_start = threading.Event()
    rs = FakeRs([EXPECTED_L515_SERIAL])
    source = L515Source(rs, stop_timeout=0.01, mapper_factory=object)

    def block_before_start():
        before_start.set()
        release_start.wait()

    source._before_pipeline_start = block_before_start
    source.start()
    assert before_start.wait(0.2)

    source.stop()
    worker = source._thread
    release_start.set()
    worker.join(0.2)

    assert rs.started == []
    assert source.state is L515State.STOPPED


def test_concurrent_start_cannot_replace_in_progress_stop():
    join_entered = threading.Event()
    release_join = threading.Event()
    restart_entered = threading.Event()

    class JoiningWorker:
        def __init__(self):
            self.alive = True

        def is_alive(self):
            return self.alive

        def join(self, _timeout):
            join_entered.set()
            release_join.wait()
            self.alive = False

    source = L515Source(FakeRs([]), stop_timeout=0.2)
    source._thread = JoiningWorker()
    source._stop_event.clear()
    stopping = threading.Thread(target=source.stop)

    def restart():
        restart_entered.set()
        source.start()

    restarting = threading.Thread(target=restart)

    stopping.start()
    assert join_entered.wait(0.2)
    restarting.start()
    assert restart_entered.wait(0.2)

    assert source._stop_event.is_set()
    assert restarting.is_alive()

    release_join.set()
    stopping.join(0.2)
    restarting.join(0.2)
    source.stop()

    assert not stopping.is_alive()
    assert not restarting.is_alive()


def test_stop_after_prestart_validation_cleans_late_native_start():
    validated = threading.Event()
    release_start = threading.Event()
    started = threading.Event()
    cleaned_after_start = threading.Event()

    class LateStartPipeline(FakePipeline):
        def __init__(self, rs):
            super().__init__(rs)
            self.active = False

        def start(self, config):
            self.rs.started.append(config)
            self.active = True
            started.set()

        def stop(self):
            if self.active:
                self.active = False
                cleaned_after_start.set()

    rs = FakeRs([EXPECTED_L515_SERIAL])
    pipeline = LateStartPipeline(rs)
    rs.pipeline = lambda: pipeline
    source = L515Source(rs, stop_timeout=0.01, mapper_factory=object)

    def block_after_validation():
        validated.set()
        release_start.wait()

    source._after_pipeline_start_validation = block_after_validation
    source.start()
    assert validated.wait(0.2)

    before = time.monotonic()
    source.stop()
    elapsed = time.monotonic() - before
    worker = source._thread
    release_start.set()
    worker.join(0.2)

    assert elapsed < 0.15
    assert started.is_set()
    assert cleaned_after_start.is_set()
    assert not pipeline.active
    assert source._starting is None
    assert source.state is L515State.STOPPED
    assert source.poll_latest().empty


def test_cancelled_late_start_uses_one_bounded_worker_stop_attempt():
    validated = threading.Event()
    release_start = threading.Event()
    blocking_stop_entered = threading.Event()
    release_blocking_stop = threading.Event()

    class BlockingCleanupPipeline(FakePipeline):
        def __init__(self, rs):
            super().__init__(rs)
            self.active = False
            self.prestart_stop_calls = 0
            self.active_stop_calls = 0

        def start(self, config):
            self.rs.started.append(config)
            self.active = True

        def stop(self):
            if not self.active:
                self.prestart_stop_calls += 1
                return
            self.active_stop_calls += 1
            blocking_stop_entered.set()
            release_blocking_stop.wait()

    rs = FakeRs([EXPECTED_L515_SERIAL])
    pipeline = BlockingCleanupPipeline(rs)
    rs.pipeline = lambda: pipeline
    source = L515Source(rs, stop_timeout=0.02, mapper_factory=object)

    def block_after_validation():
        validated.set()
        release_start.wait()

    source._after_pipeline_start_validation = block_after_validation
    source.start()
    assert validated.wait(0.2)
    source.stop()
    worker = source._thread
    assert pipeline.prestart_stop_calls == 1

    started = time.monotonic()
    release_start.set()
    worker.join(0.2)
    elapsed = time.monotonic() - started

    assert blocking_stop_entered.wait(0.2)
    assert elapsed < 0.15
    assert not worker.is_alive()
    assert pipeline.prestart_stop_calls == 1
    assert pipeline.active_stop_calls == 1
    assert source._starting is None
    assert source.state is L515State.STOPPED
    assert source.poll_latest().empty
    release_blocking_stop.set()
