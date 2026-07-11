import threading
import time
from types import SimpleNamespace

import numpy as np

from l515_dashboard.gateway_workers import ColorWorker, DepthWorker, ImuWorker, WorkerGroup
from l515_dashboard.stream_buffer import BoundedRing, LatestSlot, StreamSample


class Source:
    def __init__(self):
        self.color = LatestSlot()
        self.depth = LatestSlot()
        self.accel = BoundedRing(32)
        self.gyro = BoundedRing(32)
        self.mapper = object()

    def read_color_after(self, sequence): return self.color.read_after(sequence)
    def read_depth_after(self, sequence): return self.depth.read_after(sequence)
    def read_accel_after(self, sequence, limit): return self.accel.read_after(sequence, limit)
    def read_gyro_after(self, sequence, limit): return self.gyro.read_after(sequence, limit)


def sample(number, frame=None):
    return StreamSample("stream", number, float(number), time.monotonic_ns(), frame or object())


def test_slow_depth_never_reduces_color_publish_count():
    source = Source()
    counts = {"color": 0, "depth": 0}

    class Ros:
        def publish_color(self, value, mapper): counts["color"] += 1; return ()
        def publish_depth(self, value, mapper):
            counts["depth"] += 1
            time.sleep(.2)
            return ()

    workers = WorkerGroup(source=source, ros=Ros(), fatal=lambda exc: None,
                          depth_period_s=.1, aligner=lambda depth, color: None)
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
    source.color.publish(sample(1))
    source.depth.publish(sample(1))

    calls = 0
    def align(depth, color):
        nonlocal calls
        calls += 1
        if calls == 1:
            entered.set(); release.wait(1)
        return np.full((2, 3), calls, dtype=np.uint16)

    worker = DepthWorker(source, SimpleNamespace(publish_depth=lambda *args: ()),
                         period_s=.01, aligner=align, fatal=lambda exc: None)
    worker.start(); assert entered.wait(1)
    source.color.publish(sample(2))
    release.set(); time.sleep(.04); worker.stop()
    aligned = worker.aligned_depth
    assert aligned is not None and aligned.created_ns > 0
    assert np.all(aligned.array == 2)
    assert not aligned.array.flags.writeable


def test_stop_is_repeatable_and_leaves_no_worker_threads():
    source = Source()
    ros = SimpleNamespace(publish_color=lambda *a: (), publish_depth=lambda *a: (),
                          publish_imu=lambda *a: ())
    workers = WorkerGroup(source=source, ros=ros, fatal=lambda exc: None,
                          aligner=lambda depth, color: None)
    workers.start(); workers.stop(); workers.stop()
    assert all(not worker.is_alive for worker in workers.workers)
