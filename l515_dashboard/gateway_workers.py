"""Independent, interruptible consumers for captured L515 streams."""

from dataclasses import dataclass
import threading
import time

import numpy as np


@dataclass(frozen=True)
class AlignedDepth:
    array: np.ndarray
    created_ns: int


class _Worker:
    def __init__(self, *, name, fatal):
        self._name = name
        self._fatal = fatal
        self._stop_event = threading.Event()
        self._thread = None

    @property
    def is_alive(self):
        return bool(self._thread and self._thread.is_alive())

    def start(self):
        if self.is_alive:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_guarded, name=self._name,
                                        daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(1.0)
        if thread is None or not thread.is_alive():
            self._thread = None

    def _run_guarded(self):
        try:
            self._run()
        except Exception as exc:
            if not self._stop_event.is_set():
                self._fatal(exc)

    def _wait(self, timeout):
        return self._stop_event.wait(timeout)


def _mapper(source):
    return getattr(source, "mapper", getattr(source, "_mapper", None))


class ColorWorker(_Worker):
    def __init__(self, source, ros, *, fatal, published=None, streamer=None):
        super().__init__(name="l515-color-worker", fatal=fatal)
        self.source, self.ros = source, ros
        self._published = published or (lambda sample, topics: None)
        self._streamer = streamer

    def _run(self):
        sequence = 0
        while not self._stop_event.is_set():
            sequence, sample = self.source.read_color_after(sequence)
            mapper = _mapper(self.source)
            if sample is None or mapper is None:
                self._wait(.001)
                continue
            topics = self.ros.publish_color(sample, mapper) or ()
            self._published(sample, topics)
            if self._streamer is not None:
                self._streamer(sample)


class DepthWorker(_Worker):
    def __init__(self, source, ros, *, period_s=.1, aligner=None, fatal,
                 published=None, streamer=None, now_ns=time.monotonic_ns):
        super().__init__(name="l515-depth-worker", fatal=fatal)
        self.source, self.ros = source, ros
        self.period_s = period_s
        self._aligner = aligner or self._default_aligner(source)
        self._published = published or (lambda sample, topics: None)
        self._streamer = streamer
        self._now_ns = now_ns
        self._aligned_lock = threading.Lock()
        self._aligned_depth = None

    @staticmethod
    def _default_aligner(source):
        align = source._rs.align(source._rs.stream.color)
        return lambda depth, color: align.process(_frameset(source._rs, depth, color))

    @property
    def aligned_depth(self):
        with self._aligned_lock:
            return self._aligned_depth

    def _run(self):
        depth_sequence = color_sequence = 0
        latest_depth = latest_color = None
        while not self._stop_event.is_set():
            started = time.monotonic()
            new_depth_sequence, depth = self.source.read_depth_after(depth_sequence)
            new_color_sequence, color = self.source.read_color_after(color_sequence)
            mapper = _mapper(self.source)
            if depth is not None:
                depth_sequence = new_depth_sequence
                latest_depth = depth
                if mapper is not None:
                    topics = self.ros.publish_depth(depth, mapper) or ()
                    self._published(depth, topics)
            if color is not None:
                color_sequence = new_color_sequence
                latest_color = color
            if ((depth is not None or color is not None)
                    and latest_depth is not None and latest_color is not None
                    and self._aligner is not None):
                result = self._aligner(latest_depth.frame, latest_color.frame)
                check_depth, _ = self.source.read_depth_after(depth_sequence)
                check_color, _ = self.source.read_color_after(color_sequence)
                if check_depth == depth_sequence and check_color == color_sequence:
                    if hasattr(result, "get_depth_frame"):
                        result = result.get_depth_frame()
                    if hasattr(result, "get_data"):
                        result = result.get_data()
                    array = np.asanyarray(result).copy()
                    array.setflags(write=False)
                    aligned = AlignedDepth(array=array, created_ns=self._now_ns())
                    with self._aligned_lock:
                        self._aligned_depth = aligned
                    if self._streamer is not None:
                        self._streamer(aligned)
            self._wait(max(0.0, self.period_s - (time.monotonic() - started)))


class ImuWorker(_Worker):
    def __init__(self, source, ros, stream, *, max_rate_hz=100, fatal,
                 published=None):
        super().__init__(name=f"l515-{stream}-worker", fatal=fatal)
        self.source, self.ros, self.stream = source, ros, stream
        self.period_s = 1.0 / max_rate_hz
        self._published = published or (lambda sample, topics: None)

    def _run(self):
        sequence = 0
        reader = getattr(self.source, f"read_{self.stream}_after")
        while not self._stop_event.is_set():
            started = time.monotonic()
            result = reader(sequence, 32)
            sequence = result.sequence
            mapper = _mapper(self.source)
            if result.samples and mapper is not None:
                sample = result.samples[-1]
                topics = self.ros.publish_imu(self.stream, sample, mapper) or ()
                self._published(sample, topics)
            self._wait(max(0.0, self.period_s - (time.monotonic() - started)))


def _frameset(rs, depth, color):
    """Construct an SDK composite when the binding exposes a frameset ctor."""
    try:
        return rs.composite_frame((depth, color))
    except (AttributeError, TypeError):
        return (depth, color)


class WorkerGroup:
    def __init__(self, *, source, ros, fatal, depth_period_s=.1,
                 imu_max_rate_hz=100, aligner=None, published=None,
                 color_streamer=None, depth_streamer=None):
        common = {"fatal": fatal, "published": published}
        self.workers = (
            ColorWorker(source, ros, streamer=color_streamer, **common),
            DepthWorker(source, ros, period_s=depth_period_s, aligner=aligner,
                        streamer=depth_streamer, **common),
            ImuWorker(source, ros, "gyro", max_rate_hz=imu_max_rate_hz, **common),
            ImuWorker(source, ros, "accel", max_rate_hz=imu_max_rate_hz, **common),
        )

    def start(self):
        started = []
        try:
            for worker in self.workers:
                worker.start(); started.append(worker)
        except Exception:
            for worker in reversed(started): worker.stop()
            raise

    def stop(self):
        for worker in reversed(self.workers):
            worker.stop()
