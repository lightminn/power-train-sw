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
        self._pipeline_lock = threading.Lock()
        self.state = L515State.STOPPED
        self.state_changed_at = self._clock()

    def _set_state(self, state: L515State) -> None:
        self.state = state
        self.state_changed_at = self._clock()

    def _interruptible_wait(self, seconds: float) -> bool:
        return not self._stop_event.wait(seconds)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, name="l515-source", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        with self._pipeline_lock:
            pipeline = self._pipeline
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(self._stop_timeout)
        self._latest.clear()
        self._set_state(L515State.STOPPED)

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

    def _run(self) -> None:
        while not self._stop_event.is_set():
            pipeline = None
            self._set_state(L515State.CONNECTING)
            try:
                if not self._expected_device_present():
                    raise RuntimeError("expected L515 serial is not present")
                pipeline = self._rs.pipeline()
                with self._pipeline_lock:
                    self._pipeline = pipeline
                pipeline.start(self._sdk_config())
                mapper = self._mapper_factory()
                self._latest.clear()
                self._set_state(L515State.STREAMING)
                while not self._stop_event.is_set():
                    frames = pipeline.wait_for_frames()
                    self._latest.put(
                        color=frames.get_color_frame(),
                        depth=frames.get_depth_frame(),
                        accel=frames.first_or_default(self._rs.stream.accel),
                        gyro=frames.first_or_default(self._rs.stream.gyro),
                        timestamp_mapper=mapper,
                    )
            except Exception:
                self._latest.clear()
                self._set_state(L515State.DISCONNECTED)
            finally:
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
                with self._pipeline_lock:
                    if self._pipeline is pipeline:
                        self._pipeline = None

            if self._stop_event.is_set():
                break
            if not self._wait_fn(self.config.reconnect_interval):
                break
