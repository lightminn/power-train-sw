"""L515 terrain map UDP packet v1 (:5010) — encode and strict validation.

Standard library only: the operator console runs on the system python.  Every
rule mirrors spec §5 (docs/specs/2026-10-17-l515-terrain-map-design.md).  A
packet that breaks any rule is rejected whole.  All counters are bounded so the
worst-case encoding provably fits in ``MAX_PACKET_BYTES``.
"""
from __future__ import annotations

import base64
import binascii
from dataclasses import dataclass, replace
import json
import math
import re
from typing import Any, Sequence

UDP_PORT = 5010
SCHEMA_VERSION = 1
STATUS_TYPE = "terrain_map_status"
TILE_TYPE = "terrain_map_tile"
MAX_PACKET_BYTES = 1200

FREE, OBSTACLE, DITCH, INSUFFICIENT = 0, 1, 2, 4
VALID_CODES = frozenset((FREE, OBSTACLE, DITCH, INSUFFICIENT))
UNOBSERVED_H = -128
MAX_AGE_STEPS = 31

MAX_COUNTER = 2**31 - 1
MAX_UINT63 = 2**63 - 1
MAX_TILE_INDEX = 2**20

STATUS_REASONS: dict[str, tuple[str | None, ...]] = {
    "OK": (None,),
    "NO_POSE": ("ODOM_MISSING", "POSE_GAP", "WHEELS_STALE", "BIAS_PENDING"),
    "NO_GROUND": ("GROUND_NOT_FOUND", "NOT_SUPPORT"),
    "SENSOR_ERROR": (
        "DEPTH_STALE", "DEPTH_ENCODING", "DEPTH_SHAPE_MISMATCH",
        "CAMERA_INFO_MISSING", "PROCESSING_ERROR",
    ),
}
CORRECTOR_NAMES = ("identity", "height_match")
CORRECTOR_REASONS = ("identity", "flat", "no_overlap", "rejected", "applied")

_BOOT_ID = re.compile(r"[0-9a-f]{32}")
_INSTANCE = re.compile(r"[0-9a-f]{16}")
_FLOAT_DIGITS = 4

__all__ = [
    "Pose", "Thresholds", "CorrectorInfo", "PlaneTilt", "GravityTilt", "StatusPacket", "TilePacket", "replace",
    "encode", "parse_packet", "pack_cells", "unpack_cells",
]


@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Thresholds:
    obstacle_height_m: float
    ditch_depth_m: float
    min_observations: int


@dataclass(frozen=True)
class CorrectorInfo:
    name: str
    applied: bool
    score: float | None
    reason: str


@dataclass(frozen=True)
class PlaneTilt:
    """The detector's chosen plane against the body axes (r11): + = rising ahead / left side higher."""
    pitch_deg: float
    roll_deg: float
    age_ms: float          # since that depth frame was received


@dataclass(frozen=True)
class GravityTilt:
    """Measured in a still window ending at the frame (r11): the plane's slope and the body's tilt
    against gravity.  Absent while moving."""
    slope_pitch_deg: float
    slope_roll_deg: float
    slope_total_deg: float
    body_pitch_deg: float
    body_roll_deg: float


@dataclass(frozen=True)
class _Common:
    sender_boot_id: str
    sender_start_mono_ns: int
    sender_instance: str
    map_epoch: int
    sent_unix_ns: int

    @property
    def session_key(self) -> tuple[str, int, str]:
        return (self.sender_boot_id, self.sender_start_mono_ns, self.sender_instance)


@dataclass(frozen=True)
class StatusPacket(_Common):
    sequence: int
    status: str
    reason: str | None
    cell_m: float
    tile_cells: int
    age_step_s: float
    pose: Pose | None
    distance_m: float
    pose_gaps: int
    not_support: int
    late_dropped: int
    tile_count: int
    refresh_cycle_s: float
    thresholds: Thresholds
    corrector: CorrectorInfo
    proc_ms: float | None
    received_s: float | None = None
    plane_tilt: PlaneTilt | None = None
    gravity: GravityTilt | None = None


@dataclass(frozen=True)
class TilePacket(_Common):
    tx: int
    ty: int
    version: int
    tile_seq: int
    cells: str
    received_s: float | None = None


# --- cells ------------------------------------------------------------------
def pack_cells(heights_cm: Sequence[int | None], codes: Sequence[int], ages: Sequence[int]) -> str:
    """Two bytes per cell: int8 height cm (-128 = unobserved), then code<<5 | age."""
    if not len(heights_cm) == len(codes) == len(ages):
        raise ValueError("cell arrays must have the same length")
    raw = bytearray()
    for height, code, age in zip(heights_cm, codes, ages):
        if code not in VALID_CODES:
            raise ValueError(f"invalid cell code {code}")
        h = UNOBSERVED_H if height is None else max(-127, min(127, int(height)))
        a = max(0, min(MAX_AGE_STEPS, int(age)))
        raw.append(h & 0xFF)
        raw.append((code << 5) | a)
    return base64.b64encode(bytes(raw)).decode("ascii")


def unpack_cells(cells: str, tile_cells: int) -> tuple[list[int | None], list[int], list[int]]:
    """Decode and validate a tile's cell payload."""
    if not isinstance(cells, str):
        raise ValueError("cells: base64 string required")
    try:
        raw = base64.b64decode(cells.encode("ascii"), validate=True)
    except (binascii.Error, UnicodeEncodeError, ValueError) as exc:
        raise ValueError(f"cells: bad base64 ({exc})") from None
    count = tile_cells * tile_cells
    if len(raw) != 2 * count:
        raise ValueError(f"cells: {len(raw)} B, expected {2 * count}")
    heights: list[int | None] = []
    codes: list[int] = []
    ages: list[int] = []
    for index in range(count):
        h_byte, meta = raw[2 * index], raw[2 * index + 1]
        h = h_byte - 256 if h_byte >= 128 else h_byte
        code, age = meta >> 5, meta & 31
        if code not in VALID_CODES:
            raise ValueError(f"cells: reserved code {code}")
        if h == UNOBSERVED_H:
            if code != INSUFFICIENT:
                raise ValueError("cells: unobserved cell must carry the insufficient code")
            heights.append(None)
        else:
            heights.append(h)
        codes.append(code)
        ages.append(age)
    return heights, codes, ages


# --- encode -----------------------------------------------------------------
def _round(value: float | None) -> float | None:
    return None if value is None else round(float(value), _FLOAT_DIGITS)


def _common_dict(packet: _Common, kind: str) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION, "type": kind,
        "sender_boot_id": packet.sender_boot_id,
        "sender_start_mono_ns": packet.sender_start_mono_ns,
        "sender_instance": packet.sender_instance,
        "map_epoch": packet.map_epoch, "sent_unix_ns": packet.sent_unix_ns,
    }


def _to_dict(packet: StatusPacket | TilePacket) -> dict[str, Any]:
    if isinstance(packet, TilePacket):
        payload = _common_dict(packet, TILE_TYPE)
        payload.update(tx=packet.tx, ty=packet.ty, version=packet.version,
                       tile_seq=packet.tile_seq, cells=packet.cells)
        return payload
    payload = _common_dict(packet, STATUS_TYPE)
    pose = None if packet.pose is None else {
        "x_m": _round(packet.pose.x_m), "y_m": _round(packet.pose.y_m),
        "yaw_rad": _round(packet.pose.yaw_rad),
    }
    payload.update(
        sequence=packet.sequence, status=packet.status, reason=packet.reason,
        cell_m=_round(packet.cell_m), tile_cells=packet.tile_cells, age_step_s=_round(packet.age_step_s),
        pose=pose, distance_m=_round(packet.distance_m),
        pose_gaps=packet.pose_gaps, not_support=packet.not_support, late_dropped=packet.late_dropped,
        tile_count=packet.tile_count, refresh_cycle_s=_round(packet.refresh_cycle_s),
        thresholds={
            "obstacle_height_m": _round(packet.thresholds.obstacle_height_m),
            "ditch_depth_m": _round(packet.thresholds.ditch_depth_m),
            "min_observations": packet.thresholds.min_observations,
        },
        corrector={
            "name": packet.corrector.name, "applied": packet.corrector.applied,
            "score": _round(packet.corrector.score), "reason": packet.corrector.reason,
        },
        proc_ms=_round(packet.proc_ms),
        plane_tilt=None if packet.plane_tilt is None else {
            "pitch_deg": _round(packet.plane_tilt.pitch_deg), "roll_deg": _round(packet.plane_tilt.roll_deg),
            "age_ms": _round(packet.plane_tilt.age_ms),
        },
        gravity=None if packet.gravity is None else {
            "slope": {"pitch_deg": _round(packet.gravity.slope_pitch_deg),
                      "roll_deg": _round(packet.gravity.slope_roll_deg),
                      "total_deg": _round(packet.gravity.slope_total_deg)},
            "body_tilt": {"pitch_deg": _round(packet.gravity.body_pitch_deg),
                          "roll_deg": _round(packet.gravity.body_roll_deg)},
        },
    )
    return payload


def encode(packet: StatusPacket | TilePacket) -> bytes:
    """Compact UTF-8 JSON, validated by parsing it back (no invalid packet leaves)."""
    raw = json.dumps(_to_dict(packet), separators=(",", ":"), ensure_ascii=False,
                     allow_nan=False).encode("utf-8")
    parse_packet(raw)
    return raw


# --- parse ------------------------------------------------------------------
def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _int(value: Any, name: str, low: int, high: int) -> int:
    if not _is_int(value) or not low <= value <= high:
        raise ValueError(f"{name}: integer in [{low}, {high}] required")
    return value


def _num(value: Any, name: str, low: float, high: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: number required")
    try:
        number = float(value)
    except OverflowError:
        raise ValueError(f"{name}: number out of range") from None
    if not math.isfinite(number) or not low <= number <= high:
        raise ValueError(f"{name}: finite number in [{low}, {high}] required")
    return number


def _opt_num(value: Any, name: str, low: float, high: float) -> float | None:
    return None if value is None else _num(value, name, low, high)


def _str(value: Any, name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{name}: string required")
    return value


def _obj(value: Any, name: str, keys: tuple[str, ...]) -> dict[str, Any]:
    if not isinstance(value, dict) or not all(key in value for key in keys):
        raise ValueError(f"{name}: object with {keys} required")
    return value


def _common(payload: dict[str, Any]) -> dict[str, Any]:
    if _int(payload.get("schema_version"), "schema_version", 0, 1000) != SCHEMA_VERSION:
        raise ValueError("unsupported schema_version")
    boot = _str(payload.get("sender_boot_id"), "sender_boot_id")
    instance = _str(payload.get("sender_instance"), "sender_instance")
    if not _BOOT_ID.fullmatch(boot) or not _INSTANCE.fullmatch(instance):
        raise ValueError("bad session identity")
    return {
        "sender_boot_id": boot,
        "sender_start_mono_ns": _int(payload.get("sender_start_mono_ns"), "sender_start_mono_ns", 0, MAX_UINT63),
        "sender_instance": instance,
        "map_epoch": _int(payload.get("map_epoch"), "map_epoch", 0, MAX_COUNTER),
        "sent_unix_ns": _int(payload.get("sent_unix_ns"), "sent_unix_ns", 0, MAX_UINT63),
    }


def _parse_status(payload: dict[str, Any], received_s: float | None) -> StatusPacket:
    common = _common(payload)
    status = _str(payload.get("status"), "status")
    reason = payload.get("reason")
    if status not in STATUS_REASONS or reason not in STATUS_REASONS[status]:
        raise ValueError(f"bad status/reason {status}/{reason}")
    pose_raw = payload.get("pose")
    pose = None
    if pose_raw is not None:
        pose_raw = _obj(pose_raw, "pose", ("x_m", "y_m", "yaw_rad"))
        pose = Pose(
            _num(pose_raw["x_m"], "pose.x_m", -1e6, 1e6),
            _num(pose_raw["y_m"], "pose.y_m", -1e6, 1e6),
            _num(pose_raw["yaw_rad"], "pose.yaw_rad", -math.pi - 1e-3, math.pi + 1e-3),
        )
    th = _obj(payload.get("thresholds"), "thresholds", ("obstacle_height_m", "ditch_depth_m", "min_observations"))
    corr = _obj(payload.get("corrector"), "corrector", ("name", "applied", "score", "reason"))
    if corr["name"] not in CORRECTOR_NAMES or corr["reason"] not in CORRECTOR_REASONS:
        raise ValueError("bad corrector name/reason")
    if not isinstance(corr["applied"], bool):
        raise ValueError("corrector.applied: bool required")
    return StatusPacket(
        **common,
        sequence=_int(payload.get("sequence"), "sequence", 0, MAX_COUNTER),
        status=status, reason=reason,
        cell_m=_num(payload.get("cell_m"), "cell_m", 0.02, 0.2),
        tile_cells=_int(payload.get("tile_cells"), "tile_cells", 8, 16),
        age_step_s=_num(payload.get("age_step_s"), "age_step_s", 2.0, 10.0),
        pose=pose,
        distance_m=_num(payload.get("distance_m"), "distance_m", 0.0, 1e7),
        pose_gaps=_int(payload.get("pose_gaps"), "pose_gaps", 0, MAX_COUNTER),
        not_support=_int(payload.get("not_support"), "not_support", 0, MAX_COUNTER),
        late_dropped=_int(payload.get("late_dropped"), "late_dropped", 0, MAX_COUNTER),
        tile_count=_int(payload.get("tile_count"), "tile_count", 0, MAX_COUNTER),
        refresh_cycle_s=_num(payload.get("refresh_cycle_s"), "refresh_cycle_s", 0.0, 1e5),
        thresholds=Thresholds(
            _num(th["obstacle_height_m"], "thresholds.obstacle_height_m", 0.02, 0.5),
            _num(th["ditch_depth_m"], "thresholds.ditch_depth_m", 0.02, 0.5),
            _int(th["min_observations"], "thresholds.min_observations", 1, 10),
        ),
        corrector=CorrectorInfo(corr["name"], corr["applied"],
                                _opt_num(corr["score"], "corrector.score", 0.0, 1e3), corr["reason"]),
        proc_ms=_opt_num(payload.get("proc_ms"), "proc_ms", 0.0, 1e6),
        received_s=received_s,
        plane_tilt=_parse_plane_tilt(payload.get("plane_tilt")),
        gravity=_parse_gravity(payload.get("gravity")),
    )


def _angle(obj: dict, key: str, name: str, low: float = -90.0, high: float = 90.0) -> float:
    return _num(obj[key], f"{name}.{key}", low, high)


def _parse_plane_tilt(raw: Any) -> PlaneTilt | None:
    """Optional (r11): absent or null means no recent plane."""
    if raw is None:
        return None
    raw = _obj(raw, "plane_tilt", ("pitch_deg", "roll_deg", "age_ms"))
    return PlaneTilt(_angle(raw, "pitch_deg", "plane_tilt"), _angle(raw, "roll_deg", "plane_tilt"),
                     _num(raw["age_ms"], "plane_tilt.age_ms", 0.0, 1e6))


def _parse_gravity(raw: Any) -> GravityTilt | None:
    if raw is None:
        return None
    raw = _obj(raw, "gravity", ("slope", "body_tilt"))
    slope = _obj(raw["slope"], "gravity.slope", ("pitch_deg", "roll_deg", "total_deg"))
    body = _obj(raw["body_tilt"], "gravity.body_tilt", ("pitch_deg", "roll_deg"))
    return GravityTilt(
        _angle(slope, "pitch_deg", "gravity.slope"), _angle(slope, "roll_deg", "gravity.slope"),
        _angle(slope, "total_deg", "gravity.slope", 0.0, 90.0),
        _angle(body, "pitch_deg", "gravity.body_tilt"), _angle(body, "roll_deg", "gravity.body_tilt"),
    )


def _parse_tile(payload: dict[str, Any], received_s: float | None) -> TilePacket:
    common = _common(payload)
    cells = payload.get("cells")
    if not isinstance(cells, str):
        raise ValueError("cells: base64 string required")
    try:
        cell_bytes = len(base64.b64decode(cells.encode("ascii"), validate=True))
    except (binascii.Error, UnicodeEncodeError, ValueError):
        raise ValueError("cells: bad base64") from None
    side = math.isqrt(cell_bytes // 2)
    if side * side * 2 != cell_bytes or not 8 <= side <= 16:
        raise ValueError(f"cells: {cell_bytes} B is not a valid tile")
    unpack_cells(cells, side)
    return TilePacket(
        **common,
        tx=_int(payload.get("tx"), "tx", -MAX_TILE_INDEX, MAX_TILE_INDEX),
        ty=_int(payload.get("ty"), "ty", -MAX_TILE_INDEX, MAX_TILE_INDEX),
        version=_int(payload.get("version"), "version", 1, MAX_COUNTER),
        tile_seq=_int(payload.get("tile_seq"), "tile_seq", 1, MAX_COUNTER),
        cells=cells, received_s=received_s,
    )


def parse_packet(raw: bytes, received_s: float | None = None) -> StatusPacket | TilePacket:
    """Validate one datagram; any violation raises ``ValueError``."""
    if not isinstance(raw, (bytes, bytearray)) or len(raw) > MAX_PACKET_BYTES:
        raise ValueError("terrain map packet missing or oversize")
    try:
        payload = json.loads(bytes(raw).decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError, RecursionError, OverflowError) as exc:
        raise ValueError(f"bad JSON: {exc}") from None
    if not isinstance(payload, dict):
        raise ValueError("packet must be a JSON object")
    kind = payload.get("type")
    if kind == STATUS_TYPE:
        return _parse_status(payload, received_s)
    if kind == TILE_TYPE:
        return _parse_tile(payload, received_s)
    raise ValueError(f"unknown packet type {kind!r}")
