"""Single-owner, reconnecting RealSense source for the L515 Gateway."""

from dataclasses import dataclass, field
from enum import Enum
import threading
import time

from .config import DashboardConfig
from .stream_buffer import BoundedRing, LatestSlot, StreamSample

EXPECTED_L515_SERIAL = "00000000F0271544"


def _canonical_serial(value):
    normalized = str(value).casefold().lstrip("0")
    return normalized or "0"


def _new_timestamp_mapper():
    from powertrain_ros.l515_adapter import TimestampMapper

    return TimestampMapper()


class GatewaySourceState(Enum):
    STOPPED = "stopped"
    CONNECTING = "connecting"
    STREAMING = "streaming"
    DISCONNECTED = "disconnected"


@dataclass
class GatewayFrames:
    raw_color: object = None
    raw_depth: object = None
    aligned_depth: object = None
    accel: object = None
    gyro: object = None
    mapper: object = None
    _lock: threading.Lock = field(
        default_factory=threading.Lock, init=False, repr=False, compare=False
    )

    @property
    def empty(self):
        with self._lock:
            return all(
                getattr(self, name) is None
                for name in (
                    "raw_color", "raw_depth", "aligned_depth", "accel", "gyro"
                )
            )

    def put(self, **payload):
        with self._lock:
            for name, value in payload.items():
                if value is not None:
                    setattr(self, name, value)

    def clear(self):
        with self._lock:
            for name in (
                "raw_color", "raw_depth", "aligned_depth", "accel", "gyro", "mapper"
            ):
                setattr(self, name, None)

    def drain(self):
        names = (
            "raw_color", "raw_depth", "aligned_depth", "accel", "gyro", "mapper"
        )
        with self._lock:
            result = GatewayFrames(**{name: getattr(self, name) for name in names})
            for name in names:
                setattr(self, name, None)
            return result


class L515GatewaySource:
    """Own one SDK pipeline using the proven generation-safe lifecycle."""

    def __init__(
        self,
        rs_module,
        config=None,
        *,
        clock=time.monotonic,
        wait_fn=None,
        mapper_factory=_new_timestamp_mapper,
        stop_timeout=1.0,
    ):
        self._rs = rs_module
        self.config = config or DashboardConfig()
        if not isinstance(self.config, DashboardConfig):
            raise TypeError("config must be DashboardConfig")
        self._clock = clock
        self._stop_event = threading.Event()
        self._wait_fn = wait_fn or self._interruptible_wait
        self._mapper_factory = mapper_factory
        self._stop_timeout = float(stop_timeout)
        self._buffers = {
            self._rs.stream.color: LatestSlot(),
            self._rs.stream.depth: LatestSlot(),
            self._rs.stream.accel: BoundedRing(32),
            self._rs.stream.gyro: BoundedRing(32),
        }
        self._capture_lock = threading.Lock()
        self._capture_generation = None
        self._last_frame_numbers = {}
        self._mapper = None
        self._poll_sequences = {stream: 0 for stream in self._buffers}
        self._thread = None
        self._pipeline = None
        self._pipeline_cleanup = None
        self._starting = None
        self._public_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._generation = 0
        self._before_pipeline_start = lambda: None
        self._after_pipeline_start_validation = lambda: None
        self.state = GatewaySourceState.STOPPED
        self.state_changed_at = clock()
        self.connected_serial = None
        self.connected_profile = None

    def _set_state(self, state):
        self.state = state
        self.state_changed_at = self._clock()

    def _interruptible_wait(self, seconds):
        return not self._stop_event.wait(seconds)

    def _is_current(self, generation):
        return not self._stop_event.is_set() and generation == self._generation

    def start(self):
        with self._public_lock:
            with self._lifecycle_lock:
                if self._thread is not None and self._thread.is_alive():
                    return
                self._generation += 1
                generation = self._generation
                self._stop_event.clear()
                self._thread = threading.Thread(
                    target=self._run,
                    args=(generation,),
                    name="l515-gateway-source",
                    daemon=True,
                )
                thread = self._thread
            thread.start()

    @staticmethod
    def _stop_pipeline(pipeline):
        try:
            pipeline.stop()
        except Exception:
            pass

    def _stop_pipeline_bounded(self, pipeline):
        stopper = threading.Thread(
            target=self._stop_pipeline, args=(pipeline,), daemon=True
        )
        stopper.start()
        stopper.join(self._stop_timeout)

    def _stop_pipeline_once(self, pipeline, cleanup):
        with self._lifecycle_lock:
            if cleanup is None:
                cleanup = {"claimed": False}
                if self._pipeline is pipeline:
                    self._pipeline_cleanup = cleanup
            if cleanup["claimed"]:
                return False
            cleanup["claimed"] = True
        self._stop_pipeline_bounded(pipeline)
        return True

    def _finish_stopped(self, thread):
        with self._lifecycle_lock:
            if thread is not None and self._thread is not thread:
                return
            if thread is not None:
                self._thread = None
            self._pipeline = None
            self._pipeline_cleanup = None
            self._starting = None
            self._clear_capture()
            self.connected_serial = None
            self.connected_profile = None
            self._set_state(GatewaySourceState.STOPPED)

    def stop(self):
        with self._public_lock:
            with self._lifecycle_lock:
                self._stop_event.set()
                self._generation += 1
                if self._starting is not None:
                    self._starting["cancel_requested"] = True
                pipeline = self._pipeline
                cleanup = self._pipeline_cleanup
                starting = self._starting
                thread = self._thread
            # Never call stop while native start may still be in progress.
            if pipeline is not None and starting is None:
                self._stop_pipeline_once(pipeline, cleanup)
            if thread is not None and thread.is_alive():
                thread.join(self._stop_timeout)
            if thread is None or not thread.is_alive():
                self._finish_stopped(thread)

    def poll_latest(self):
        color_sequence, color = self.read_color_after(
            self._poll_sequences[self._rs.stream.color]
        )
        depth_sequence, depth = self.read_depth_after(
            self._poll_sequences[self._rs.stream.depth]
        )
        accel = self.read_accel_after(
            self._poll_sequences[self._rs.stream.accel], 32
        )
        gyro = self.read_gyro_after(
            self._poll_sequences[self._rs.stream.gyro], 32
        )
        self._poll_sequences.update({
            self._rs.stream.color: color_sequence,
            self._rs.stream.depth: depth_sequence,
            self._rs.stream.accel: accel.sequence,
            self._rs.stream.gyro: gyro.sequence,
        })
        return GatewayFrames(
            raw_color=None if color is None else color.frame,
            raw_depth=None if depth is None else depth.frame,
            accel=None if not accel.samples else accel.samples[-1].frame,
            gyro=None if not gyro.samples else gyro.samples[-1].frame,
            mapper=self._mapper,
        )

    def read_color_after(self, sequence):
        return self._buffers[self._rs.stream.color].read_after(sequence)

    def read_depth_after(self, sequence):
        return self._buffers[self._rs.stream.depth].read_after(sequence)

    def read_accel_after(self, sequence, limit):
        return self._buffers[self._rs.stream.accel].read_after(sequence, limit)

    def read_gyro_after(self, sequence, limit):
        return self._buffers[self._rs.stream.gyro].read_after(sequence, limit)

    def _reset_capture(self, generation):
        with self._capture_lock:
            self._capture_generation = generation
            self._last_frame_numbers.clear()
            self._mapper = self._mapper_factory()
            for stream, buffer in self._buffers.items():
                buffer.clear()
                self._poll_sequences[stream] = 0

    def _clear_capture(self, generation=None):
        with self._capture_lock:
            if generation is not None and generation != self._capture_generation:
                return
            self._capture_generation = None
            self._last_frame_numbers.clear()
            self._mapper = None
            for stream, buffer in self._buffers.items():
                buffer.clear()
                self._poll_sequences[stream] = 0

    def _sample_from_frame(self, stream, frame):
        keeper = getattr(frame, "keep", None)
        if callable(keeper):
            keeper()
        return StreamSample(
            stream=stream,
            frame_number=int(frame.get_frame_number()),
            timestamp_ms=float(frame.get_timestamp()),
            received_ns=time.monotonic_ns(),
            frame=frame,
        )

    @staticmethod
    def _children(frame):
        is_frameset = getattr(frame, "is_frameset", None)
        if callable(is_frameset) and is_frameset():
            frameset = frame.as_frameset()
            try:
                return tuple(frameset)
            except TypeError:
                return tuple(frameset[index] for index in range(frameset.size()))
        return (frame,)

    def _on_frame(self, frame, generation):
        if not self._is_current(generation):
            return
        for child in self._children(frame):
            profile = child.get_profile()
            stream = profile.stream_type()
            if stream not in self._buffers:
                continue
            number = int(child.get_frame_number())
            with self._capture_lock:
                if (not self._is_current(generation)
                        or self._capture_generation != generation):
                    return
                if self._last_frame_numbers.get(stream) == number:
                    continue
                self._last_frame_numbers[stream] = number
                sample = self._sample_from_frame(stream, child)
                self._buffers[stream].publish(sample)

    def _matching_serial(self):
        expected = _canonical_serial(EXPECTED_L515_SERIAL)
        matches = [
            device.get_info(self._rs.camera_info.serial_number)
            for device in self._rs.context().query_devices()
            if _canonical_serial(
                device.get_info(self._rs.camera_info.serial_number)
            ) == expected
        ]
        return matches[0] if len(matches) == 1 else None

    def _sdk_config(self, serial):
        config = self._rs.config()
        config.enable_device(serial)
        config.enable_stream(
            self._rs.stream.color,
            self.config.color_width,
            self.config.color_height,
            self._rs.format.bgr8,
            self.config.fps,
        )
        config.enable_stream(
            self._rs.stream.depth,
            self.config.depth_width,
            self.config.depth_height,
            self._rs.format.z16,
            self.config.fps,
        )
        config.enable_stream(self._rs.stream.accel)
        config.enable_stream(self._rs.stream.gyro)
        return config

    def _run(self, generation=None):
        generation = self._generation if generation is None else generation
        worker = threading.current_thread()
        try:
            self._run_generation(generation)
        finally:
            if self._stop_event.is_set() and self._thread is worker:
                self._finish_stopped(worker)

    def _run_generation(self, generation):
        while self._is_current(generation):
            pipeline = None
            cleanup = None
            cleanup_done = False
            with self._lifecycle_lock:
                if not self._is_current(generation):
                    break
                self._set_state(GatewaySourceState.CONNECTING)
            try:
                serial = self._matching_serial()
                if serial is None:
                    raise RuntimeError("expected L515 serial is not present")
                if not self._is_current(generation):
                    break
                pipeline = self._rs.pipeline()
                cleanup = {"claimed": False}
                with self._lifecycle_lock:
                    if not self._is_current(generation):
                        break
                    self._pipeline = pipeline
                    self._pipeline_cleanup = cleanup
                    self._starting = {
                        "generation": generation,
                        "pipeline": pipeline,
                        "cancel_requested": False,
                    }
                sdk_config = self._sdk_config(serial)
                self._before_pipeline_start()
                with self._lifecycle_lock:
                    if not self._is_current(generation):
                        break
                self._after_pipeline_start_validation()
                self._reset_capture(generation)
                pipeline.start(
                    sdk_config,
                    lambda frame, current=generation: self._on_frame(frame, current),
                )
                with self._lifecycle_lock:
                    starting = self._starting
                    same_start = (
                        starting is not None
                        and starting["generation"] == generation
                        and starting["pipeline"] is pipeline
                    )
                    cancelled = (
                        not self._is_current(generation)
                        or (same_start and starting["cancel_requested"])
                    )
                    if same_start and not cancelled:
                        self._starting = None
                if cancelled:
                    self._stop_pipeline_once(pipeline, cleanup)
                    cleanup_done = True
                    break
                self.connected_serial = serial
                self.connected_profile = {
                    "color": [self.config.color_width, self.config.color_height,
                              self.config.fps],
                    "depth": [self.config.depth_width, self.config.depth_height,
                              self.config.fps],
                }
                with self._lifecycle_lock:
                    if not self._is_current(generation):
                        break
                    self._set_state(GatewaySourceState.STREAMING)
                while self._is_current(generation):
                    if self._stop_event.wait(
                            min(self.config.reconnect_interval_s, 0.1)):
                        break
                    if self._matching_serial() != serial:
                        raise RuntimeError("expected L515 disconnected")
            except Exception:
                if self._is_current(generation):
                    self._clear_capture(generation)
                    self.connected_serial = None
                    self.connected_profile = None
                    self._set_state(GatewaySourceState.DISCONNECTED)
            finally:
                self._clear_capture(generation)
                if pipeline is not None and not cleanup_done:
                    self._stop_pipeline_once(pipeline, cleanup)
                with self._lifecycle_lock:
                    if self._pipeline is pipeline:
                        self._pipeline = None
                        self._pipeline_cleanup = None
                    starting = self._starting
                    if (
                        starting is not None
                        and starting["generation"] == generation
                        and starting["pipeline"] is pipeline
                    ):
                        self._starting = None
            if not self._is_current(generation):
                break
            if not self._wait_fn(self.config.reconnect_interval_s):
                break
