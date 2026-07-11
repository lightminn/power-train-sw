"""Single-owner, reconnecting RealSense source for the L515 Gateway."""

from dataclasses import dataclass, field
from enum import Enum
import threading
import time

from .config import DashboardConfig

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
        self._latest = GatewayFrames()
        self._thread = None
        self._pipeline = None
        self._pipeline_cleanup = None
        self._starting = None
        self._public_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._generation = 0
        self._before_pipeline_start = lambda: None
        self._after_pipeline_start_validation = lambda: None
        self._before_frame_commit = lambda: None
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
            self._latest.clear()
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
        return self._latest.drain()

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

    @staticmethod
    def _dedup(payload, last):
        for name in ("raw_color", "raw_depth", "aligned_depth", "accel", "gyro"):
            sample = payload[name]
            getter = getattr(sample, "get_timestamp", None)
            if sample is None or not callable(getter):
                continue
            stamp = float(getter())
            if last.get(name) == stamp:
                payload[name] = None
            else:
                last[name] = stamp

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
                pipeline.start(sdk_config)
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
                align = self._rs.align(self._rs.stream.color)
                self.connected_serial = serial
                self.connected_profile = {
                    "color": [self.config.color_width, self.config.color_height,
                              self.config.fps],
                    "depth": [self.config.depth_width, self.config.depth_height,
                              self.config.fps],
                }
                mapper = self._mapper_factory()
                last = {}
                self._latest.clear()
                with self._lifecycle_lock:
                    if not self._is_current(generation):
                        break
                    self._set_state(GatewaySourceState.STREAMING)
                while self._is_current(generation):
                    frames = pipeline.wait_for_frames()
                    aligned = align.process(frames)
                    payload = {
                        "raw_color": frames.get_color_frame(),
                        "raw_depth": frames.get_depth_frame(),
                        "aligned_depth": aligned.get_depth_frame(),
                        "accel": frames.first_or_default(self._rs.stream.accel),
                        "gyro": frames.first_or_default(self._rs.stream.gyro),
                        "mapper": mapper,
                    }
                    self._dedup(payload, last)
                    self._before_frame_commit()
                    with self._lifecycle_lock:
                        if not self._is_current(generation):
                            break
                        self._latest.put(**payload)
            except Exception:
                if self._is_current(generation):
                    self._latest.clear()
                    self.connected_serial = None
                    self.connected_profile = None
                    self._set_state(GatewaySourceState.DISCONNECTED)
            finally:
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
