"""ROS-free tick logic of the terrain overlay sender (spec §4.1, §4.4).

The ROS node only converts messages and sends bytes; everything that decides
what a packet says lives here so it can be tested on any host with numpy/cv2.
All times are monotonic seconds; milliseconds appear only in the packet.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import secrets
import threading
import time
from typing import Callable

import numpy as np

from .camera import CameraModel
from .contours import build_obstacles, fit_packet_size
from .contract import (
    CAMERA_HEIGHT_RANGE_M, FRAME_SIZE, PACKET_TYPE, SCHEMA_VERSION,
    encode_packet, parse_packet,
)
from .detector import DetectorConfig, TerrainDetector, TerrainResult
from .projection import DEFAULT_COLOR_OFFSET_M, DepthToColorProjector
from .floor import project_floor

BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
PUBLISH_HZ_RANGE = (5.0, 10.0)
EMPTY_ZONES = {"left": None, "center": None, "right": None}


@dataclass(frozen=True)
class RuntimeConfig:
    publish_hz: float = 10.0
    depth_scale_m: float = 0.00025
    depth_stale_s: float = 0.5
    ground_hold_s: float = 0.5
    color_offset_m: tuple[float, float, float] = DEFAULT_COLOR_OFFSET_M
    path_half_width_m: float = 0.4445
    front_offset_m: float = 0.0
    alignment_verified: bool = False
    publish_floor_mask: bool = True

    def __post_init__(self) -> None:
        numbers = (self.publish_hz, self.depth_scale_m, self.depth_stale_s,
                   self.ground_hold_s, self.path_half_width_m, self.front_offset_m)
        if not all(math.isfinite(float(n)) for n in numbers):
            raise ValueError("terrain overlay parameters must be finite")
        low, high = PUBLISH_HZ_RANGE
        if not low <= self.publish_hz <= high:
            raise ValueError("publish_hz must be within 5-10 Hz (stale contract)")
        if min(self.depth_scale_m, self.depth_stale_s, self.ground_hold_s,
               self.path_half_width_m) <= 0.0:
            raise ValueError("scale, stale, hold and half width must be > 0")
        if self.front_offset_m < 0.0:
            raise ValueError("front_offset_m must be >= 0")
        if len(self.color_offset_m) != 3:
            raise ValueError("color_offset_m needs three values")
        if type(self.publish_floor_mask) is not bool:
            raise ValueError('publish_floor_mask must be bool')
        if not isinstance(self.alignment_verified, bool):
            raise ValueError("alignment_verified must be bool")

    @property
    def detector_config(self) -> DetectorConfig:
        return DetectorConfig(
            depth_scale_m=self.depth_scale_m,
            ground_hold_s=self.ground_hold_s,
            path_half_width_m=self.path_half_width_m,
            front_offset_m=self.front_offset_m,
        )


@dataclass(frozen=True)
class SenderSession:
    boot_id: str
    start_mono_ns: int
    instance: str

    @classmethod
    def create(
        cls, *, boot_id_path: Path = BOOT_ID_PATH,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        token_hex: Callable[[int], str] = secrets.token_hex,
    ) -> "SenderSession":
        boot_id = boot_id_path.read_text(encoding="ascii").strip().replace("-", "").lower()
        if len(boot_id) != 32 or any(c not in "0123456789abcdef" for c in boot_id):
            raise ValueError(f"unexpected kernel boot_id {boot_id!r}")
        return cls(boot_id=boot_id, start_mono_ns=int(monotonic_ns()), instance=token_hex(8))


@dataclass(frozen=True)
class _Depth:
    array: np.ndarray
    stamp_ns: int
    rx_s: float


@dataclass(frozen=True)
class _Processed:
    stamp_ns: int
    rx_s: float
    status: str
    reason: str | None
    ground_state: str | None
    fresh_rx_s: float | None
    camera_height_m: float | None
    camera_tilt_deg: float | None
    zones: dict
    obstacles: list
    proc_ms: float | None
    floor_mask: dict | None = None


def _round_height(height_m: float) -> float:
    rounded = round(height_m, 3)
    low, high = CAMERA_HEIGHT_RANGE_M
    return rounded if low < rounded < high else height_m


def _round_zones(zones: dict) -> dict:
    return {name: None if value is None else round(value, 2) for name, value in zones.items()}


class TerrainOverlayRuntime:
    def __init__(
        self, config: RuntimeConfig, session: SenderSession, *,
        log_error: Callable[[str], None] = lambda _message: None,
        log_overrun: Callable[[str], None] | None = None,
        unix_ns: Callable[[], int] = time.time_ns,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.session = session
        self._log_error = log_error
        # A separate sink so log throttling of general errors can never hide
        # the processing-time evidence the load check (§7.3-2) relies on.
        self._log_overrun = log_error if log_overrun is None else log_overrun
        self._unix_ns = unix_ns
        self._clock = clock
        # on_depth/on_bad_depth may run on another executor thread than tick.
        self._input_lock = threading.Lock()
        self._depth_camera: CameraModel | None = None
        self._color_camera: CameraModel | None = None
        self._detector: TerrainDetector | None = None
        self._projector: DepthToColorProjector | None = None
        self._latest: _Depth | None = None
        self._bad_depth_rx_s: float | None = None
        self._cache: _Processed | None = None
        self.sequence = 0
        self.max_processing_s = 0.0
        self.processing_overruns = 0

    # -- inputs ---------------------------------------------------------
    def set_depth_info(self, camera: CameraModel) -> None:
        if camera != self._depth_camera:
            self._depth_camera = camera
            self._detector = TerrainDetector(camera, self.config.detector_config)
            self._cache = None

    def set_color_info(self, camera: CameraModel) -> None:
        if camera != self._color_camera:
            self._color_camera = camera
            self._projector = DepthToColorProjector(camera, self.config.color_offset_m)
            self._cache = None

    def on_depth(self, array: np.ndarray, stamp_ns: int, rx_s: float) -> None:
        with self._input_lock:
            self._latest = _Depth(array, int(stamp_ns), float(rx_s))

    def on_bad_depth(self, rx_s: float) -> None:
        """A depth message failed the 16UC1/endian/step check and was dropped."""
        with self._input_lock:
            self._bad_depth_rx_s = float(rx_s)

    # -- tick -----------------------------------------------------------
    def tick(self) -> bytes | None:
        """Build this tick's packet; None only if even the error packet is invalid.

        The first evaluation may process a new depth frame (tens of ms); the
        clock is read again afterwards so stale/expiry/ages are judged at send
        time (spec §4.1-4).  The second evaluation reuses the cache.
        """
        with self._input_lock:
            depth, bad_rx_s = self._latest, self._bad_depth_rx_s
        self._evaluate(self._clock(), depth, bad_rx_s)
        now_s = self._clock()
        status, reason, processed = self._evaluate(now_s, depth, bad_rx_s)
        stamp = None if depth is None else depth.stamp_ns
        age_ms = None if depth is None else round(max(0.0, now_s - depth.rx_s) * 1000.0, 1)
        try:
            payload = self._payload(now_s, status, reason, processed, stamp, age_ms)
            raw = encode_packet(fit_packet_size(payload))
            parse_packet(raw)
        except (ValueError, TypeError) as exc:
            self._log_error(f"terrain packet failed self-check: {exc}")
            raw = self._error_packet(now_s, stamp, age_ms)
            if raw is None:
                return None
        self.sequence += 1
        return raw

    def _evaluate(self, now_s: float, latest: _Depth | None, bad: float | None):
        cfg = self.config
        if self._depth_camera is None or self._color_camera is None:
            return "SENSOR_ERROR", "CAMERA_INFO_MISSING", None
        if (self._color_camera.width, self._color_camera.height) != FRAME_SIZE:
            return "SENSOR_ERROR", "COLOR_SHAPE_MISMATCH", None
        if (
            bad is not None and now_s - bad <= cfg.depth_stale_s
            and (latest is None or bad > latest.rx_s)
        ):
            return "SENSOR_ERROR", "DEPTH_ENCODING", None
        if latest is None or now_s - latest.rx_s > cfg.depth_stale_s:
            return "SENSOR_ERROR", "DEPTH_STALE", None
        if latest.array.shape != (self._depth_camera.height, self._depth_camera.width):
            return "SENSOR_ERROR", "DEPTH_SHAPE_MISMATCH", None
        if self._cache is None or self._cache.stamp_ns != latest.stamp_ns:
            self._cache = self._process(latest)
        processed = self._cache
        if (
            processed.status == "OK"
            and now_s - processed.fresh_rx_s > cfg.ground_hold_s
        ):
            return "NO_GROUND", "GROUND_EXPIRED", processed
        return processed.status, processed.reason, processed

    def _process(self, depth: _Depth) -> _Processed:
        started = self._clock()
        try:
            result: TerrainResult = self._detector.process(depth.array, depth.rx_s)
            obstacles = []
            floor_mask = None
            if result.status == "OK":
                obstacles = build_obstacles(
                    result.obstacle_points, result.obstacle_fwd, result.obstacle_lat,
                    self._projector,
                    half_width_m=self.config.path_half_width_m,
                    front_offset_m=self.config.front_offset_m,
                )
            if result.status == 'OK' and self.config.publish_floor_mask:
                floor_mask = project_floor(result,self._projector)
        except Exception as exc:  # noqa: BLE001 — any core failure becomes a status
            self._record_processing_time(self._clock() - started)
            self._log_error(f"terrain processing failed: {type(exc).__name__}: {exc}")
            return _Processed(depth.stamp_ns, depth.rx_s, "SENSOR_ERROR", "PROCESSING_ERROR",
                              None, None, None, None, dict(EMPTY_ZONES), [], None)
        elapsed_s = self._clock() - started
        self._record_processing_time(elapsed_s)
        return _Processed(
            depth.stamp_ns, depth.rx_s, result.status, result.reason,
            result.ground_state, result.fresh_rx_s,
            result.camera_height_m, result.camera_tilt_deg,
            dict(result.zones), obstacles, round(elapsed_s * 1000.0, 1), floor_mask,
        )

    def _record_processing_time(self, elapsed_s: float) -> None:
        """Track every processing attempt — successful or not — against one tick.

        The DEPTH_STALE send bound is stale + 1/hz + processing + 0.05 s, so the
        load check (§7.3-2) needs the true maximum: compared unrounded, and
        including attempts that ended in an exception.
        """
        self.max_processing_s = max(self.max_processing_s, elapsed_s)
        tick_s = 1.0 / self.config.publish_hz
        if elapsed_s > tick_s:
            self.processing_overruns += 1
            self._log_overrun(
                f"terrain processing {elapsed_s * 1000.0:.2f} ms exceeds the "
                f"{tick_s * 1000.0:.0f} ms tick ({self.processing_overruns} overruns, "
                f"max {self.max_processing_s * 1000.0:.2f} ms)",
            )

    # -- packet ---------------------------------------------------------
    def _common(self, stamp, age_ms) -> dict:
        cfg = self.config
        return {
            "schema_version": SCHEMA_VERSION,
            "type": PACKET_TYPE,
            "sender_boot_id": self.session.boot_id,
            "sender_start_mono_ns": self.session.start_mono_ns,
            "sender_instance": self.session.instance,
            "sequence": self.sequence,
            "sent_unix_ns": int(self._unix_ns()),
            "depth_stamp_ns": stamp,
            "depth_age_ms": age_ms,
            "frame_size": list(FRAME_SIZE),
            "distance_origin": "rover_front" if cfg.front_offset_m > 0.0 else "l515_depth_optical",
            "alignment_verified": cfg.alignment_verified,
            "path_half_width_m": cfg.path_half_width_m,
        }

    def _payload(self, now_s, status, reason, processed, stamp, age_ms) -> dict:
        payload = self._common(stamp, age_ms)
        payload.update(status=status, reason=reason)
        if status == "OK":
            payload.update(
                ground={
                    "state": processed.ground_state,
                    "age_ms": round(max(0.0, now_s - processed.fresh_rx_s) * 1000.0, 1),
                    "camera_height_m": _round_height(processed.camera_height_m),
                    "camera_tilt_deg": round(processed.camera_tilt_deg, 1),
                },
                zones=_round_zones(processed.zones),
                obstacles=processed.obstacles,
                proc_ms=processed.proc_ms,
            )
        else:
            payload.update(
                ground=None, zones=dict(EMPTY_ZONES), obstacles=[],
                proc_ms=None if processed is None else processed.proc_ms,
            )
        if status == 'OK' and processed.floor_mask is not None:
            payload['floor_mask'] = processed.floor_mask
        return payload

    def _error_packet(self, now_s, stamp, age_ms) -> bytes | None:
        payload = self._common(stamp, age_ms)
        payload.update(status="SENSOR_ERROR", reason="PROCESSING_ERROR", ground=None,
                       zones=dict(EMPTY_ZONES), obstacles=[], proc_ms=None)
        try:
            raw = encode_packet(payload)
            parse_packet(raw)
        except (ValueError, TypeError) as exc:
            self._log_error(f"terrain error packet failed self-check, not sent: {exc}")
            return None
        return raw
