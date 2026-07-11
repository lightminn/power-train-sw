import threading
import time
from types import SimpleNamespace

import numpy as np

from l515_dashboard.gateway_workers import (
    ColorWorker, DepthWorker, ImuWorker, WorkerGroup, WorkerStopTimeout, _Worker,
)
from l515_dashboard.gateway_source import VideoBundle
from l515_dashboard.stream_buffer import BoundedRing, LatestSlot, StreamSample


class Source:
    def __init__(self):
        self.color = LatestSlot()
        self.depth = LatestSlot()
        self.accel = BoundedRing(32)
        self.gyro = BoundedRing(32)
        self.mapper = object()
        self.bundle = LatestSlot()
        self.identity = (1, 1)

    def read_color_after(self, sequence): return self.color.read_after(sequence)
    def read_depth_after(self, sequence): return self.depth.read_after(sequence)
    def read_accel_after(self, sequence, limit): return self.accel.read_after(sequence, limit)
    def read_gyro_after(self, sequence, limit): return self.gyro.read_after(sequence, limit)
    def read_video_bundle_after(self, sequence): return self.bundle.read_after(sequence)
    def capture_identity(self): return self.identity


def sample(number, frame=None):
    return StreamSample("stream", number, float(number), time.monotonic_ns(), frame or object())


def test_slow_depth_never_reduces_color_publish_count():
    source = Source()
    counts = {"color": 0, "depth": 0}

    class Ros:
        def publish_color(self, value, mapper): counts["color"] += 1; return ()
        def publish_imu(self, *args): return ()
        def publish_depth(self, value, mapper):
            counts["depth"] += 1
            time.sleep(.2)
            return ()

    workers = WorkerGroup(source=source, ros=Ros(), fatal=lambda exc: None,
                          depth_period_s=.1, aligner=lambda frameset: None)
    workers.start()
    deadline = time.monotonic() + 1.0
    number = 0
    while time.monotonic() < deadline:
        number += 1
        source.color.publish(sample(number))
        source.depth.publish(sample(number))
        time.sleep(1 / 30)
    workers.stop()
    assert counts["color"] >= 29
    assert counts["depth"] <= 10


def test_imu_publish_rate_is_bounded_without_unbounded_backlog():
    source = Source()
    published = []

    class Ros:
        def publish_imu(self, stream, value, mapper): published.append((stream, value)); return ()

    worker = ImuWorker(source, Ros(), "gyro", max_rate_hz=100,
                       fatal=lambda exc: None)
    worker.start()
    deadline = time.monotonic() + 1.0
    number = 0
    while time.monotonic() < deadline:
        number += 1
        source.gyro.publish(sample(number))
        time.sleep(1 / 200)
    worker.stop()
    assert 95 <= len(published) <= 100
    assert len(source.gyro._items) <= 32


def test_depth_discards_alignment_when_input_changes_and_stores_immutable_array():
    source = Source()
    entered, release = threading.Event(), threading.Event()
    source.depth.publish(sample(1))
    source.bundle.publish(VideoBundle(1, 1, object(), time.monotonic_ns()))

    calls = 0
    def align(frameset):
        nonlocal calls
        calls += 1
        if calls == 1:
            entered.set(); release.wait(1)
        return np.full((2, 3), calls, dtype=np.uint16)

    worker = DepthWorker(source, SimpleNamespace(publish_depth=lambda *args: ()),
                         period_s=.01, aligner=align, fatal=lambda exc: None)
    worker.start(); assert entered.wait(1)
    source.identity = (2, 2)
    source.bundle.publish(VideoBundle(2, 2, object(), time.monotonic_ns()))
    release.set(); time.sleep(.04); worker.stop()
    aligned = worker.aligned_depth
    assert aligned is not None and aligned.created_ns > 0
    assert np.all(aligned.array == 2)
    assert not aligned.array.flags.writeable


def test_depth_alignment_uses_real_composite_frameset_only():
    source = Source(); real = object(); source.depth.publish(sample(1))
    source.bundle.publish(VideoBundle(1, 1, real, 1)); seen = []
    worker = DepthWorker(source, SimpleNamespace(publish_depth=lambda *a: ()),
                         period_s=.01, aligner=lambda frameset: seen.append(frameset) or np.ones((1,1)),
                         fatal=lambda exc: None)
    worker.start(); time.sleep(.03); worker.stop()
    assert seen == [real]


def test_worker_group_stop_fails_if_dependency_user_is_still_alive_then_retries():
    source = Source(); entered, release = threading.Event(), threading.Event()
    source.color.publish(sample(1))
    class Ros:
        def publish_color(self,*a): entered.set(); release.wait(); return ()
        publish_depth=lambda *a: ()
        publish_imu=lambda *a: ()
    workers = WorkerGroup(source=source, ros=Ros(), fatal=lambda exc: None,
                          stop_timeout=.01, aligner=lambda _: None)
    workers.start(); assert entered.wait(1)
    with __import__('pytest').raises(WorkerStopTimeout): workers.stop()
    release.set(); workers.stop()
    assert all(not worker.is_alive for worker in workers.workers)


def test_immediate_worker_thread_failure_is_reported_by_start_barrier():
    source = Source()
    source.read_color_after = lambda sequence: (_ for _ in ()).throw(RuntimeError("reader died"))
    worker = ColorWorker(source, SimpleNamespace(publish_color=lambda *a: ()),
                         fatal=lambda exc: None, stop_timeout=.05)
    with __import__('pytest').raises(RuntimeError, match="reader died"):
        worker.start()


def test_nonready_worker_timeout_stops_and_joins_its_thread():
    class NeverReady(_Worker):
        def __init__(self): super().__init__(name="never-ready", fatal=lambda exc: None,
                                             stop_timeout=.03)
        def _on_start(self):
            while not self._stop_event.wait(.001): pass
        def _run(self): return None
    worker = NeverReady()
    with __import__('pytest').raises(RuntimeError, match="did not become ready"):
        worker.start()
    assert not worker.is_alive


def test_overrun_waits_a_positive_period_instead_of_catching_up():
    source = Source(); source.gyro.publish(sample(1)); calls=[]
    class Ros:
        def publish_imu(self,*a): calls.append(time.monotonic()); time.sleep(.02); source.gyro.publish(sample(len(calls)+1)); return ()
    worker=ImuWorker(source,Ros(),"gyro",max_rate_hz=100,fatal=lambda exc:None)
    worker.start(); time.sleep(.12); worker.stop()
    assert len(calls) <= 5
    assert all(later - earlier >= .025 for earlier, later in zip(calls, calls[1:]))


def test_stop_is_repeatable_and_leaves_no_worker_threads():
    source = Source()
    ros = SimpleNamespace(publish_color=lambda *a: (), publish_depth=lambda *a: (),
                          publish_imu=lambda *a: ())
    workers = WorkerGroup(source=source, ros=ros, fatal=lambda exc: None,
                          aligner=lambda frameset: None)
    workers.start(); workers.stop(); workers.stop()
    assert all(not worker.is_alive for worker in workers.workers)
