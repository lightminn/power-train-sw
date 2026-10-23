"""L515 terrain overlay UDP packet v1 — encode and strict validation.

Standard library only: the operator console runs on the system python, which
has no numpy or cv2.  Every rule here mirrors the design spec §5.2
(docs/specs/2026-10-17-l515-terrain-overlay-design.md); a packet that breaks
any rule is rejected whole, never partially displayed.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from typing import Any

UDP_PORT = 5009
SCHEMA_VERSION = 1
PACKET_TYPE = "l515_terrain"
MAX_PACKET_BYTES = 1200
FRAME_SIZE = (1280, 720)
MAX_OBSTACLES = 4
MIN_POLY_POINTS = 3
MAX_POLY_POINTS = 16
CAMERA_HEIGHT_RANGE_M = (0.25, 0.80)  # open interval
MAX_CAMERA_TILT_DEG = 40.0

STATUS_REASONS: dict[str, tuple[str | None, ...]] = {
    "OK": (None,),
    "NO_GROUND": ("GROUND_NOT_FOUND", "GROUND_EXPIRED"),
    "SENSOR_ERROR": (
        "DEPTH_STALE",
        "DEPTH_EMPTY",
        "DEPTH_ENCODING",
        "DEPTH_SHAPE_MISMATCH",
        "COLOR_SHAPE_MISMATCH",
        "CAMERA_INFO_MISSING",
        "PROCESSING_ERROR",
    ),
}
DISTANCE_ORIGINS = ("l515_depth_optical", "rover_front")
GROUND_STATES = ("FRESH", "HELD")
ZONE_NAMES = ("left", "center", "right")

_BOOT_ID = re.compile(r"[0-9a-f]{32}")
_INSTANCE = re.compile(r"[0-9a-f]{16}")


@dataclass(frozen=True)
class GroundInfo:
    state: str
    age_ms: float
    camera_height_m: float
    camera_tilt_deg: float


@dataclass(frozen=True)
class TerrainObstacle:
    dist_m: float
    in_path: bool
    poly: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class TerrainFrame:
    sender_boot_id: str
    sender_start_mono_ns: int
    sender_instance: str
    sequence: int
    sent_unix_ns: int
    depth_stamp_ns: int | None
    depth_age_ms: float | None
    status: str
    reason: str | None
    frame_size: tuple[int, int]
    distance_origin: str
    alignment_verified: bool
    ground: GroundInfo | None
    path_half_width_m: float
    zones: dict[str, float | None]
    obstacles: tuple[TerrainObstacle, ...]
    proc_ms: float | None
    received_monotonic_s: float | None = None

    @property
    def session_key(self) -> tuple[str, int, str]:
        return (self.sender_boot_id, self.sender_start_mono_ns, self.sender_instance)


def _dumps(payload: dict[str, Any]) -> bytes:
    return json.dumps(
        payload, separators=(",", ":"), ensure_ascii=False, allow_nan=False,
    ).encode("utf-8")


def encoded_size(payload: dict[str, Any]) -> int:
    """Byte length the payload would have on the wire (no size check)."""
    return len(_dumps(payload))


def encode_packet(payload: dict[str, Any]) -> bytes:
    """Compact UTF-8 JSON; NaN/Infinity and oversize packets are refused."""
    raw = _dumps(payload)
    if len(raw) > MAX_PACKET_BYTES:
        raise ValueError(f"terrain packet {len(raw)} B exceeds {MAX_PACKET_BYTES} B")
    return raw


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _nonneg_int(value: Any, name: str) -> int:
    if not _is_int(value) or value < 0:
        raise ValueError(f"{name}: non-negative integer required")
    return value


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: number required")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name}: finite number required") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name}: finite number required")
    return number


def _nonneg_finite(value: Any, name: str) -> float:
    number = _finite(value, name)
    if number < 0.0:
        raise ValueError(f"{name}: must be >= 0")
    return number


def _optional(value: Any, parse, name: str):
    return None if value is None else parse(value, name)


def _parse_ground(value: Any) -> GroundInfo:
    if not isinstance(value, dict):
        raise ValueError("ground: object required")
    state = value.get("state")
    if not isinstance(state, str) or state not in GROUND_STATES:
        raise ValueError("ground.state invalid")
    height = _finite(value.get("camera_height_m"), "ground.camera_height_m")
    low, high = CAMERA_HEIGHT_RANGE_M
    if not low < height < high:
        raise ValueError("ground.camera_height_m out of range")
    tilt = _finite(value.get("camera_tilt_deg"), "ground.camera_tilt_deg")
    if not 0.0 <= tilt <= MAX_CAMERA_TILT_DEG:
        raise ValueError("ground.camera_tilt_deg out of range")
    return GroundInfo(
        state=state,
        age_ms=_nonneg_finite(value.get("age_ms"), "ground.age_ms"),
        camera_height_m=height,
        camera_tilt_deg=tilt,
    )


def _parse_zones(value: Any) -> dict[str, float | None]:
    if not isinstance(value, dict) or any(name not in value for name in ZONE_NAMES):
        raise ValueError("zones: left/center/right required")
    return {
        name: _optional(value[name], _nonneg_finite, f"zones.{name}")
        for name in ZONE_NAMES
    }


def _parse_obstacle(value: Any) -> TerrainObstacle:
    if not isinstance(value, dict):
        raise ValueError("obstacle: object required")
    in_path = value.get("in_path")
    if not isinstance(in_path, bool):
        raise ValueError("obstacle.in_path: bool required")
    poly = value.get("poly")
    if (
        not isinstance(poly, list)
        or not MIN_POLY_POINTS <= len(poly) <= MAX_POLY_POINTS
    ):
        raise ValueError("obstacle.poly: 3-16 points required")
    width, height = FRAME_SIZE
    points = []
    for point in poly:
        if (
            not isinstance(point, list)
            or len(point) != 2
            or not all(_is_int(c) for c in point)
            or not 0 <= point[0] < width
            or not 0 <= point[1] < height
        ):
            raise ValueError("obstacle.poly: integer [x, y] inside the frame required")
        points.append((point[0], point[1]))
    return TerrainObstacle(
        dist_m=_nonneg_finite(value.get("dist_m"), "obstacle.dist_m"),
        in_path=in_path,
        poly=tuple(points),
    )


def parse_packet(raw: bytes, received_monotonic_s: float | None = None) -> TerrainFrame:
    """Validate one datagram against packet v1; raise ValueError on any breach."""
    if len(raw) > MAX_PACKET_BYTES:
        raise ValueError("oversize terrain packet")
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError, RecursionError) as exc:
        raise ValueError("terrain packet is not UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("terrain packet must be an object")
    if not _is_int(payload.get("schema_version")) or payload["schema_version"] != SCHEMA_VERSION:
        raise ValueError("unsupported schema_version")
    if payload.get("type") != PACKET_TYPE:
        raise ValueError("not an l515_terrain packet")
    boot_id = payload.get("sender_boot_id")
    if not isinstance(boot_id, str) or not _BOOT_ID.fullmatch(boot_id):
        raise ValueError("sender_boot_id: 32 lowercase hex required")
    instance = payload.get("sender_instance")
    if not isinstance(instance, str) or not _INSTANCE.fullmatch(instance):
        raise ValueError("sender_instance: 16 lowercase hex required")
    status = payload.get("status")
    if not isinstance(status, str) or status not in STATUS_REASONS:
        raise ValueError("status invalid")
    reason = payload.get("reason")
    if (reason is not None and not isinstance(reason, str)) or reason not in STATUS_REASONS[status]:
        raise ValueError(f"reason {reason!r} invalid for {status}")
    frame_size = payload.get("frame_size")
    if (
        not isinstance(frame_size, list)
        or not all(_is_int(value) for value in frame_size)
        or frame_size != list(FRAME_SIZE)
    ):
        raise ValueError("frame_size must be [1280, 720]")
    origin = payload.get("distance_origin")
    if not isinstance(origin, str) or origin not in DISTANCE_ORIGINS:
        raise ValueError("distance_origin invalid")
    alignment_verified = payload.get("alignment_verified")
    if not isinstance(alignment_verified, bool):
        raise ValueError("alignment_verified: bool required")
    half_width = _finite(payload.get("path_half_width_m"), "path_half_width_m")
    if half_width <= 0.0:
        raise ValueError("path_half_width_m must be > 0")
    zones = _parse_zones(payload.get("zones"))
    raw_obstacles = payload.get("obstacles")
    if not isinstance(raw_obstacles, list) or len(raw_obstacles) > MAX_OBSTACLES:
        raise ValueError("obstacles: list of at most 4 required")
    obstacles = tuple(_parse_obstacle(item) for item in raw_obstacles)
    raw_ground = payload.get("ground")
    if status == "OK":
        ground = _parse_ground(raw_ground)
    else:
        if raw_ground is not None:
            raise ValueError("ground must be null unless status is OK")
        if any(value is not None for value in zones.values()):
            raise ValueError("zones must be null unless status is OK")
        if obstacles:
            raise ValueError("obstacles must be empty unless status is OK")
        ground = None
    return TerrainFrame(
        sender_boot_id=boot_id,
        sender_start_mono_ns=_nonneg_int(
            payload.get("sender_start_mono_ns"), "sender_start_mono_ns",
        ),
        sender_instance=instance,
        sequence=_nonneg_int(payload.get("sequence"), "sequence"),
        sent_unix_ns=_nonneg_int(payload.get("sent_unix_ns"), "sent_unix_ns"),
        depth_stamp_ns=_optional(
            payload.get("depth_stamp_ns"), _nonneg_int, "depth_stamp_ns",
        ),
        depth_age_ms=_optional(
            payload.get("depth_age_ms"), _nonneg_finite, "depth_age_ms",
        ),
        status=status,
        reason=reason,
        frame_size=FRAME_SIZE,
        distance_origin=origin,
        alignment_verified=alignment_verified,
        ground=ground,
        path_half_width_m=half_width,
        zones=zones,
        obstacles=obstacles,
        proc_ms=_optional(payload.get("proc_ms"), _nonneg_finite, "proc_ms"),
        received_monotonic_s=received_monotonic_s,
    )
