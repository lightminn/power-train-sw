"""ROS-free loader for the fail-closed L515 terrain qualification."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping

import yaml


_COMMAND_GUIDANCE = frozenset(("lane", "wall", "follow", "terrain"))


@dataclass(frozen=True)
class TerrainQualification:
    source_path: Path
    roi: tuple[int, int, int, int]
    min_depth_m: float
    max_depth_m: float
    min_valid_ratio: float
    translation_m: tuple[float, float, float]
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


def _mapping(value, label: str) -> Mapping:
    if not isinstance(value, Mapping):
        raise ValueError("%s must be a mapping" % label)
    return value


def _finite(value, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError("%s must be finite" % label)
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("%s must be finite" % label) from exc
    if not math.isfinite(result):
        raise ValueError("%s must be finite" % label)
    return result


def _integer(value, label: str, *, minimum: int) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < minimum
    ):
        raise ValueError("%s must be an integer >= %d" % (label, minimum))
    return value


def _vector(value, label: str, size: int) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError("%s must contain %d finite values" % (label, size))
    return tuple(
        _finite(item, "%s[%d]" % (label, index))
        for index, item in enumerate(value)
    )


def _quaternion_rpy(rotation_xyzw) -> tuple[float, float, float]:
    x, y, z, w = _vector(
        rotation_xyzw,
        "tf.base_link_to_l515_link.rotation_xyzw",
        4,
    )
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-3):
        raise ValueError(
            "tf.base_link_to_l515_link.rotation_xyzw must be normalized"
        )
    x, y, z, w = (item / norm for item in (x, y, z, w))
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def load_approved_terrain_qualification(path) -> TerrainQualification:
    """Load one explicitly approved, complete production qualification."""
    source_path = Path(path)
    try:
        document = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(
            "terrain qualification could not be loaded: %s" % exc
        ) from exc
    root = _mapping(document, "terrain qualification")
    if root.get("schema_version") != 1:
        raise ValueError("schema_version must be 1")

    qualification = _mapping(root.get("qualification"), "qualification")
    if qualification.get("production_enabled") is not True:
        raise ValueError("qualification.production_enabled must be true")
    if qualification.get("status") != "approved":
        raise ValueError("qualification.status must be approved")

    mount = _mapping(root.get("mount"), "mount")
    approved_pitch_deg = _finite(
        mount.get("pitch_deg"),
        "mount.pitch_deg",
    )

    terrain = _mapping(root.get("terrain"), "terrain")
    if terrain.get("backend") != "numpy":
        raise ValueError("terrain.backend must be numpy")
    roi = _mapping(terrain.get("roi"), "terrain.roi")
    roi_values = (
        _integer(roi.get("x"), "terrain.roi.x", minimum=0),
        _integer(roi.get("y"), "terrain.roi.y", minimum=0),
        _integer(roi.get("width"), "terrain.roi.width", minimum=1),
        _integer(roi.get("height"), "terrain.roi.height", minimum=1),
    )
    if roi_values[2] % 80 or roi_values[3] % 60:
        raise ValueError(
            "terrain.roi width/height must be divisible by 80/60"
        )

    thresholds = _mapping(
        terrain.get("depth_thresholds"),
        "terrain.depth_thresholds",
    )
    min_depth_m = _finite(
        thresholds.get("min_depth_m"),
        "terrain.depth_thresholds.min_depth_m",
    )
    max_depth_m = _finite(
        thresholds.get("max_depth_m"),
        "terrain.depth_thresholds.max_depth_m",
    )
    min_valid_ratio = _finite(
        thresholds.get("min_valid_ratio"),
        "terrain.depth_thresholds.min_valid_ratio",
    )
    if not 0.0 < min_depth_m < max_depth_m:
        raise ValueError(
            "terrain.depth_thresholds depth range must be positive and ordered"
        )
    if not 0.0 <= min_valid_ratio <= 1.0:
        raise ValueError(
            "terrain.depth_thresholds.min_valid_ratio must be within 0..1"
        )

    transform = _mapping(root.get("tf"), "tf")
    transform = _mapping(
        transform.get("base_link_to_l515_link"),
        "tf.base_link_to_l515_link",
    )
    translation_m = _vector(
        transform.get("translation_m"),
        "tf.base_link_to_l515_link.translation_m",
        3,
    )
    roll_rad, pitch_rad, yaw_rad = _quaternion_rpy(
        transform.get("rotation_xyzw")
    )
    if not math.isclose(
        math.degrees(pitch_rad),
        approved_pitch_deg,
        rel_tol=0.0,
        abs_tol=0.1,
    ):
        raise ValueError(
            "mount.pitch_deg must match tf.base_link_to_l515_link rotation"
        )
    return TerrainQualification(
        source_path=source_path,
        roi=roi_values,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
        min_valid_ratio=min_valid_ratio,
        translation_m=translation_m,
        roll_rad=roll_rad,
        pitch_rad=pitch_rad,
        yaw_rad=yaw_rad,
    )


def require_command_guidance_qualified(
    *,
    guidance: str,
    propose: bool,
    qualification_path,
) -> TerrainQualification | None:
    """Reject every command-producing guidance mode unless qualified."""
    if not propose or guidance not in _COMMAND_GUIDANCE:
        return None
    return load_approved_terrain_qualification(qualification_path)


def enforce_node_command_guidance_qualification(
    node,
    *,
    guidance: str,
    default_path,
) -> TerrainQualification | None:
    """Apply the same fail-closed gate when a node is run directly."""
    node.declare_parameter("terrain_qualification_file", str(default_path))
    return require_command_guidance_qualified(
        guidance=guidance,
        propose=bool(node.get_parameter("enabled").value),
        qualification_path=node.get_parameter(
            "terrain_qualification_file"
        ).value,
    )
