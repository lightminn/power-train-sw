"""Serial-locked RealSense SDK worker for the powertrain-owned L515."""

from dataclasses import dataclass, field
from enum import Enum
import threading
import time
from typing import Any, Callable, Optional

EXPECTED_L515_SERIAL = "00000000F0271544"


def _new_timestamp_mapper():
    # Keep SDK source importable without ROS message packages.  The adapter is
    # loaded only when a real streaming session starts.
    from .l515_adapter import TimestampMapper

    return TimestampMapper()


@dataclass(frozen=True)
class L515Config:
    """Fixed L515 stream and reconnect configuration."""

    serial: str = EXPECTED_L515_SERIAL
    width: int = 640
    height: int = 480
    fps: int = 30
    reconnect_interval: float = 2.0

    def __post_init__(self):
        if self.serial != EXPECTED_L515_SERIAL:
            raise ValueError("serial must identify the powertrain L515")


class L515State(Enum):
    STOPPED = "stopped"
    CONNECTING = "connecting"
    STREAMING = "streaming"
    DISCONNECTED = "disconnected"


@dataclass
class LatestFrames:
    """Thread-safe, one-slot-per-stream frame handoff."""

    color: Any = None
    depth: Any = None
    accel: Any = None
    gyro: Any = None
    timestamp_mapper: Any = None
    _lock: threading.Lock = field(
        default_factory=threading.Lock, init=False, repr=False, compare=False
    )

    @property
    def empty(self) -> bool:
        with self._lock:
            return all(
                value is None
                for value in (self.color, self.depth, self.accel, self.gyro)
            )

    def put(self, *, color=None, depth=None, accel=None, gyro=None,
            timestamp_mapper=None) -> None:
        with self._lock:
            for name, value in (
                ("color", color),
                ("depth", depth),
                ("accel", accel),
                ("gyro", gyro),
            ):
                if value is not None:
                    setattr(self, name, value)
            if timestamp_mapper is not None:
                self.timestamp_mapper = timestamp_mapper

    def clear(self) -> None:
        with self._lock:
            self.color = None
            self.depth = None
            self.accel = None
            self.gyro = None
            self.timestamp_mapper = None

    def drain(self) -> "LatestFrames":
        with self._lock:
            result = LatestFrames(
                color=self.color,
                depth=self.depth,
                accel=self.accel,
                gyro=self.gyro,
                timestamp_mapper=self.timestamp_mapper,
            )
            self.color = None
            self.depth = None
            self.accel = None
            self.gyro = None
            self.timestamp_mapper = None
            return result


class L515Source:
    """Read only the configured L515 on a dedicated reconnecting worker."""

    def __init__(
        self,
        rs_module,
        config: Optional[L515Config] = None,
        *,
        clock: Callable[[], float] = time.monotonic,
        wait_fn: Optional[Callable[[float], bool]] = None,
        mapper_factory: Callable[[], Any] = _new_timestamp_mapper,
        stop_timeout: float = 1.0,
    ):
        self._rs = rs_module
        self.config = config or L515Config()
        self._clock = clock
        self._stop_event = threading.Event()
        self._wait_fn = wait_fn or self._interruptible_wait
        self._mapper_factory = mapper_factory
        self._stop_timeout = float(stop_timeout)
        self._latest = LatestFrames()
        self._thread = None
        self._pipeline = None
        self._starting = None
        self._public_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._generation = 0
        self._before_pipeline_start = lambda: None
        self._after_pipeline_start_validation = lambda: None
        self._before_frame_commit = lambda: None
        self.state = L515State.STOPPED
        self.state_changed_at = self._clock()

    def _set_state(self, state: L515State) -> None:
        self.state = state
        self.state_changed_at = self._clock()

    def _interruptible_wait(self, seconds: float) -> bool:
        return not self._stop_event.wait(seconds)

    def start(self) -> None:
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
                    name="l515-source",
                    daemon=True,
                )
                thread = self._thread
            thread.start()

    def stop(self) -> None:
        with self._public_lock:
            with self._lifecycle_lock:
                self._stop_event.set()
                self._generation += 1
                if self._starting is not None:
                    self._starting["cancel_requested"] = True
                pipeline = self._pipeline
                thread = self._thread
            if pipeline is not None:
                threading.Thread(
                    target=self._stop_pipeline,
                    args=(pipeline,),
                    name="l515-sdk-stop",
                    daemon=True,
                ).start()
            if thread is not None and thread.is_alive():
                thread.join(self._stop_timeout)
            if thread is None or not thread.is_alive():
                self._finish_stopped(thread)

    def poll_latest(self) -> LatestFrames:
        return self._latest.drain()

    def _expected_device_present(self) -> bool:
        devices = self._rs.context().query_devices()
        return any(
            device.get_info(self._rs.camera_info.serial_number)
            == self.config.serial
            for device in devices
        )

    def _sdk_config(self):
        sdk_config = self._rs.config()
        sdk_config.enable_device(self.config.serial)
        sdk_config.enable_stream(
            self._rs.stream.color,
            self.config.width,
            self.config.height,
            self._rs.format.bgr8,
            self.config.fps,
        )
        sdk_config.enable_stream(
            self._rs.stream.depth,
            self.config.width,
            self.config.height,
            self._rs.format.z16,
            self.config.fps,
        )
        sdk_config.enable_stream(self._rs.stream.accel)
        sdk_config.enable_stream(self._rs.stream.gyro)
        return sdk_config

    def _is_current(self, generation: int) -> bool:
        return (
            not self._stop_event.is_set()
            and generation == self._generation
        )

    def _is_current_locked(self, generation: int) -> bool:
        return self._is_current(generation)

    @staticmethod
    def _stop_pipeline(pipeline) -> None:
        try:
            pipeline.stop()
        except Exception:
            pass

    def _finish_stopped(self, thread) -> None:
        with self._lifecycle_lock:
            if thread is not None and self._thread is not thread:
                return
            if thread is not None:
                self._thread = None
            self._pipeline = None
            self._starting = None
            self._latest.clear()
            self._set_state(L515State.STOPPED)

    def _set_state_if_current(
        self, generation: int, state: L515State
    ) -> bool:
        with self._lifecycle_lock:
            if not self._is_current(generation):
                return False
            self._set_state(state)
            return True

    def _clear_if_current(self, generation: int) -> bool:
        with self._lifecycle_lock:
            if not self._is_current_locked(generation):
                return False
            self._latest.clear()
            return True

    def _run(self, generation: Optional[int] = None) -> None:
        if generation is None:
            generation = self._generation
        worker = threading.current_thread()
        try:
            self._run_generation(generation)
        finally:
            if self._stop_event.is_set() and self._thread is worker:
                self._finish_stopped(worker)

    def _run_generation(self, generation: int) -> None:
        while self._is_current(generation):
            pipeline = None
            if not self._set_state_if_current(
                generation, L515State.CONNECTING
            ):
                break
            try:
                if not self._expected_device_present():
                    raise RuntimeError("expected L515 serial is not present")
                if not self._is_current(generation):
                    break
                pipeline = self._rs.pipeline()
                with self._lifecycle_lock:
                    if not self._is_current_locked(generation):
                        break
                    self._pipeline = pipeline
                    self._starting = {
                        "generation": generation,
                        "pipeline": pipeline,
                        "cancel_requested": False,
                    }
                sdk_config = self._sdk_config()
                self._before_pipeline_start()
                with self._lifecycle_lock:
                    if not self._is_current_locked(generation):
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
                        not self._is_current_locked(generation)
                        or (same_start and starting["cancel_requested"])
                    )
                    if same_start and not cancelled:
                        self._starting = None
                if cancelled:
                    self._stop_pipeline(pipeline)
                    with self._lifecycle_lock:
                        if self._starting is starting:
                            self._starting = None
                    break
                mapper = self._mapper_factory()
                if not self._clear_if_current(generation):
                    break
                if not self._set_state_if_current(
                    generation, L515State.STREAMING
                ):
                    break
                while self._is_current(generation):
                    frames = pipeline.wait_for_frames()
                    payload = dict(
                        color=frames.get_color_frame(),
                        depth=frames.get_depth_frame(),
                        accel=frames.first_or_default(self._rs.stream.accel),
                        gyro=frames.first_or_default(self._rs.stream.gyro),
                        timestamp_mapper=mapper,
                    )
                    self._before_frame_commit()
                    with self._lifecycle_lock:
                        if not self._is_current_locked(generation):
                            break
                        self._latest.put(**payload)
            except Exception:
                if self._clear_if_current(generation):
                    self._set_state_if_current(
                        generation, L515State.DISCONNECTED
                    )
            finally:
                if pipeline is not None:
                    self._stop_pipeline(pipeline)
                with self._lifecycle_lock:
                    if self._pipeline is pipeline:
                        self._pipeline = None
                    starting = self._starting
                    if (
                        starting is not None
                        and starting["generation"] == generation
                        and starting["pipeline"] is pipeline
                    ):
                        self._starting = None

            if not self._is_current(generation):
                break
            if not self._wait_fn(self.config.reconnect_interval):
                break
