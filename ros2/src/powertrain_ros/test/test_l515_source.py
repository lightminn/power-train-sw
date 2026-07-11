from types import SimpleNamespace

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
    rs.results = [
        FakeFrames("old"),
        RuntimeError("disconnect"),
        FakeFrames("new", on_read=source._stop_event.set),
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

    source.stop()

    assert source._thread.timeout == 0.25
    assert source.state is L515State.STOPPED
