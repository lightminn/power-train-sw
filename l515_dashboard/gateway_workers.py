"""Independent, interruptible consumers for captured L515 streams."""

from dataclasses import dataclass
import threading
import time

import numpy as np


@dataclass(frozen=True)
class AlignedDepth:
    array: np.ndarray
    created_ns: int


class WorkerStopTimeout(RuntimeError):
    pass


class _Worker:
    def __init__(self, *, name, fatal, stop_timeout=1.0):
        self._name = name
        self._fatal = fatal
        self._stop_event = threading.Event()
        self._thread = None
        self._ready = threading.Event()
        self._error = None
        self._failed = threading.Event()
        self._stop_timeout = stop_timeout
        self._startup_lock = threading.Lock()
        self._startup_committed = False

    @property
    def is_alive(self):
        return bool(self._thread and self._thread.is_alive())

    def start(self):
        if self.is_alive:
            return
        self._stop_event.clear()
        self._ready.clear(); self._failed.clear(); self._error = None
        with self._startup_lock:
            self._startup_committed = False
        self._thread = threading.Thread(target=self._run_guarded, name=self._name,
                                        daemon=True)
        self._thread.start()
        if not self._ready.wait(self._stop_timeout):
            error = RuntimeError(f"{self._name} did not become ready")
            self.stop()
            raise error
        self._failed.wait(min(.01, self._stop_timeout))
        with self._startup_lock:
            if self._error is None:
                self._startup_committed = True
                return
            error = self._error
        self.stop()
        raise error

    def stop(self):
        self._stop_event.set()
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(self._stop_timeout)
        if thread is None or not thread.is_alive():
            self._thread = None
            return
        raise WorkerStopTimeout(f"{self._name} is still alive")

    def _run_guarded(self):
        try:
            self._on_start()
            self._ready.set()
            self._run()
        except Exception as exc:
            with self._startup_lock:
                self._error = exc
                runtime_error = self._startup_committed
                self._failed.set()
                self._ready.set()
            if runtime_error and not self._stop_event.is_set():
                self._fatal(exc)

    def _on_start(self):
        return None

    def _wait(self, timeout):
        return self._stop_event.wait(timeout)


def _mapper(source):
    return getattr(source, "mapper", getattr(source, "_mapper", None))


class ColorWorker(_Worker):
    def __init__(self, source, ros, *, fatal, published=None, streamer=None,
                 stop_timeout=1.0):
        super().__init__(name="l515-color-worker", fatal=fatal,
                         stop_timeout=stop_timeout)
        self.source, self.ros = source, ros
        self._published = published or (lambda sample, topics: None)
        self._streamer = streamer

    def _on_start(self):
        if not callable(getattr(self.ros, "publish_color", None)):
            raise TypeError("ROS color publisher is unavailable")

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
                 published=None, streamer=None, now_ns=time.monotonic_ns,
                 alignment_required=None,
                 stop_timeout=1.0):
        super().__init__(name="l515-depth-worker", fatal=fatal,
                         stop_timeout=stop_timeout)
        self.source, self.ros = source, ros
        self.period_s = period_s
        self._aligner = aligner or self._default_aligner(source)
        self._published = published or (lambda sample, topics: None)
        self._streamer = streamer
        self._alignment_required = alignment_required or (lambda: True)
        self._now_ns = now_ns
        self._aligned_lock = threading.Lock()
        self._aligned_depth = None

    @staticmethod
    def _default_aligner(source):
        align = source._rs.align(source._rs.stream.color)
        return align.process

    def _on_start(self):
        if not callable(getattr(self.ros, "publish_depth", None)):
            raise TypeError("ROS depth publisher is unavailable")

    @property
    def aligned_depth(self):
        with self._aligned_lock:
            return self._aligned_depth

    def _run(self):
        depth_sequence = bundle_sequence = 0
        deadline = time.monotonic()
        while not self._stop_event.is_set():
            new_depth_sequence, depth = self.source.read_depth_after(depth_sequence)
            new_bundle_sequence, bundle = self.source.read_video_bundle_after(bundle_sequence)
            mapper = _mapper(self.source)
            if depth is not None:
                depth_sequence = new_depth_sequence
                if mapper is not None:
                    topics = self.ros.publish_depth(depth, mapper) or ()
                    self._published(depth, topics)
            if (bundle is not None and self._aligner is not None
                    and self._alignment_required()):
                bundle_sequence = new_bundle_sequence
                identity = (bundle.generation, bundle.capture_token)
                if self.source.capture_identity() == identity:
                    result = self._aligner(bundle.frameset)
                else:
                    result = None
                if result is not None and self.source.capture_identity() == identity:
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
            deadline += self.period_s
            delay = deadline - time.monotonic()
            if delay < 0:
                deadline = time.monotonic() + self.period_s
                delay = self.period_s
            self._wait(delay)


class ImuWorker(_Worker):
    def __init__(self, source, ros, stream, *, max_rate_hz=100, fatal,
                 published=None, stop_timeout=1.0):
        super().__init__(name=f"l515-{stream}-worker", fatal=fatal,
                         stop_timeout=stop_timeout)
        self.source, self.ros, self.stream = source, ros, stream
        self.period_s = 1.0 / max_rate_hz
        self._published = published or (lambda sample, topics: None)

    def _on_start(self):
        if not callable(getattr(self.ros, "publish_imu", None)):
            raise TypeError("ROS IMU publisher is unavailable")

    def _run(self):
        sequence = 0
        reader = getattr(self.source, f"read_{self.stream}_after")
        while not self._stop_event.is_set():
            result = reader(sequence, 32)
            sequence = result.sequence
            mapper = _mapper(self.source)
            if result.samples and mapper is not None:
                sample = result.samples[-1]
                topics = self.ros.publish_imu(self.stream, sample, mapper) or ()
                self._published(sample, topics)
            self._wait(self.period_s)


class WorkerGroup:
    def __init__(self, *, source, ros, fatal, depth_period_s=.1,
                 imu_max_rate_hz=100, aligner=None, published=None,
                 color_streamer=None, depth_streamer=None,
                 alignment_required=None, stop_timeout=1.0):
        common = {"fatal": fatal, "published": published,
                  "stop_timeout": stop_timeout}
        self.workers = (
            ColorWorker(source, ros, streamer=color_streamer, **common),
            DepthWorker(source, ros, period_s=depth_period_s, aligner=aligner,
                        streamer=depth_streamer,
                        alignment_required=alignment_required, **common),
            ImuWorker(source, ros, "gyro", max_rate_hz=imu_max_rate_hz, **common),
            ImuWorker(source, ros, "accel", max_rate_hz=imu_max_rate_hz, **common),
        )

    def start(self):
        started = []
        try:
            for worker in self.workers:
                started.append(worker); worker.start()
        except Exception:
            for worker in reversed(started): worker.stop()
            raise

    def stop(self):
        failures = []
        for worker in reversed(self.workers):
            try:
                worker.stop()
            except WorkerStopTimeout as exc:
                failures.append(exc)
        if failures:
            raise WorkerStopTimeout(str(failures[0]))
