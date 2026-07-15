"""Value-injected sensor time, transform, and mounting qualification.

This module intentionally has no ROS imports. Callers convert ROS messages and
TF results to scalar values before invoking these functions.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np


def _finite(value: float, name: str) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _finite_tuple(values: Sequence[float], name: str, length: int) -> tuple[float, ...]:
    converted = tuple(_finite(value, name) for value in values)
    if len(converted) != length:
        raise ValueError(f"{name} must contain {length} values")
    return converted


@dataclass(frozen=True)
class TimingQualification:
    offsets_s: tuple[float, ...]
    max_abs_clock_delta_s: float
    equal_stamp_count: int
    regressing_stamp_count: int
    reject_reasons: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.reject_reasons


def qualify_stream_timing(
    *,
    header_stamps_s: Sequence[float],
    receive_times_s: Sequence[float],
    max_abs_clock_delta_s: float,
) -> TimingQualification:
    """Compare injected header stamps to the same local receive clock."""
    stamps = tuple(_finite(value, "header stamp") for value in header_stamps_s)
    receives = tuple(_finite(value, "receive time") for value in receive_times_s)
    limit = _finite(max_abs_clock_delta_s, "max_abs_clock_delta_s")
    if limit < 0.0:
        raise ValueError("max_abs_clock_delta_s must be nonnegative")
    if not stamps or len(stamps) != len(receives):
        raise ValueError("header stamps and receive times must have equal nonzero length")

    offsets = tuple(receive - stamp for stamp, receive in zip(stamps, receives))
    maximum = max(abs(value) for value in offsets)
    equal_count = sum(current == previous for previous, current in zip(stamps, stamps[1:]))
    regressing_count = sum(current < previous for previous, current in zip(stamps, stamps[1:]))
    reasons: list[str] = []
    if maximum > limit:
        reasons.append("clock_delta_exceeded")
    if equal_count:
        reasons.append("equal_stamp")
    if regressing_count:
        reasons.append("regressing_stamp")
    return TimingQualification(
        offsets_s=offsets,
        max_abs_clock_delta_s=maximum,
        equal_stamp_count=equal_count,
        regressing_stamp_count=regressing_count,
        reject_reasons=tuple(reasons),
    )


@dataclass(frozen=True)
class StreamSkewQualification:
    stamps_s: Mapping[str, float]
    max_skew_s: float
    oldest_stream: str
    newest_stream: str
    reject_reasons: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.reject_reasons


def qualify_stream_skew(
    stamps_s: Mapping[str, float],
    *,
    max_skew_s: float,
) -> StreamSkewQualification:
    """Measure RGB/depth/IMU/wheel skew without coupling their freshness."""
    required = {"rgb", "depth", "imu", "wheel"}
    if set(stamps_s) != required:
        missing = sorted(required - set(stamps_s))
        extra = sorted(set(stamps_s) - required)
        raise ValueError(f"stream stamps must contain rgb/depth/imu/wheel; missing={missing}, extra={extra}")
    stamps = {name: _finite(value, f"{name} stamp") for name, value in stamps_s.items()}
    limit = _finite(max_skew_s, "max_skew_s")
    if limit < 0.0:
        raise ValueError("max_skew_s must be nonnegative")
    oldest = min(stamps, key=stamps.get)
    newest = max(stamps, key=stamps.get)
    maximum = stamps[newest] - stamps[oldest]
    reasons = ("stream_skew_exceeded",) if maximum > limit else ()
    return StreamSkewQualification(
        stamps_s=stamps,
        max_skew_s=maximum,
        oldest_stream=oldest,
        newest_stream=newest,
        reject_reasons=reasons,
    )


@dataclass(frozen=True)
class RigidTransform:
    """REP-103 rigid transform in metres, mapping source into target frame."""

    target_frame: str
    source_frame: str
    translation_m: tuple[float, float, float]
    rotation: np.ndarray

    def __post_init__(self) -> None:
        if not self.target_frame or not self.source_frame:
            raise ValueError("transform frame names must be non-empty")
        translation = _finite_tuple(self.translation_m, "translation_m", 3)
        rotation = np.array(self.rotation, dtype=float, copy=True)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("rotation must be a finite 3x3 matrix")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
            raise ValueError("rotation must be orthonormal")
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
            raise ValueError("rotation determinant must be +1")
        rotation.setflags(write=False)
        object.__setattr__(self, "translation_m", translation)
        object.__setattr__(self, "rotation", rotation)

    def transform_point(self, point_source_m: Sequence[float]) -> tuple[float, float, float]:
        point = np.asarray(_finite_tuple(point_source_m, "point_source_m", 3))
        transformed = self.rotation @ point + np.asarray(self.translation_m)
        return tuple(float(value) for value in transformed)


@dataclass(frozen=True)
class TransformQualification:
    tf_age_s: float
    transformed_target_base_m: tuple[float, float, float]
    xyz_error_m: tuple[float, float, float]
    reject_reasons: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.reject_reasons


def qualify_transform(
    transform: RigidTransform,
    *,
    measurement_stamp_s: float,
    transform_stamp_s: float,
    max_tf_age_s: float,
    known_target_sensor_m: Sequence[float],
    expected_target_base_m: Sequence[float],
    max_xyz_error_m: Sequence[float],
) -> TransformQualification:
    """Qualify base_link→l515_link direction, stamp freshness, and target error."""
    measurement_stamp = _finite(measurement_stamp_s, "measurement_stamp_s")
    transform_stamp = _finite(transform_stamp_s, "transform_stamp_s")
    age_limit = _finite(max_tf_age_s, "max_tf_age_s")
    if age_limit < 0.0:
        raise ValueError("max_tf_age_s must be nonnegative")
    expected = _finite_tuple(expected_target_base_m, "expected_target_base_m", 3)
    error_limits = _finite_tuple(max_xyz_error_m, "max_xyz_error_m", 3)
    if any(value < 0.0 for value in error_limits):
        raise ValueError("max_xyz_error_m values must be nonnegative")

    transformed = transform.transform_point(known_target_sensor_m)
    error = tuple(actual - target for actual, target in zip(transformed, expected))
    age = abs(measurement_stamp - transform_stamp)
    reasons: list[str] = []
    if not (
        transform.target_frame == "base_link"
        and transform.source_frame == "l515_link"
    ):
        reasons.append("invalid_transform_frames")
    if age > age_limit:
        reasons.append("tf_stale")
    if any(abs(value) > limit for value, limit in zip(error, error_limits)):
        reasons.append("known_target_xyz_error")
    return TransformQualification(
        tf_age_s=age,
        transformed_target_base_m=transformed,
        xyz_error_m=error,
        reject_reasons=tuple(reasons),
    )


@dataclass(frozen=True)
class OpticalAxisQualification:
    axis_alignment: Mapping[str, float]
    reject_reasons: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.reject_reasons


def qualify_optical_axes(
    *,
    rotation_sensor_to_base: np.ndarray,
    expected_base_directions: Mapping[str, Sequence[float]],
    sensor_name: str,
    min_alignment: float,
) -> OpticalAxisQualification:
    """Verify the injected optical x/y/z axes and their signs in base_link."""
    if not sensor_name:
        raise ValueError("sensor_name must be non-empty")
    rotation = np.asarray(rotation_sensor_to_base, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation_sensor_to_base must be a finite 3x3 matrix")
    if set(expected_base_directions) != {"x", "y", "z"}:
        raise ValueError("expected base directions must define x, y, and z")
    alignment_limit = _finite(min_alignment, "min_alignment")
    if not -1.0 <= alignment_limit <= 1.0:
        raise ValueError("min_alignment must be within -1..1")

    axis_vectors = {
        "x": np.array((1.0, 0.0, 0.0)),
        "y": np.array((0.0, 1.0, 0.0)),
        "z": np.array((0.0, 0.0, 1.0)),
    }
    alignments: dict[str, float] = {}
    reasons: list[str] = []
    for axis in ("x", "y", "z"):
        expected = np.asarray(
            _finite_tuple(expected_base_directions[axis], f"expected {axis} axis", 3)
        )
        expected_length = float(np.linalg.norm(expected))
        if expected_length <= 1e-12:
            raise ValueError("expected axis directions must be nonzero")
        actual = rotation @ axis_vectors[axis]
        alignment = float(np.dot(actual, expected / expected_length))
        alignments[axis] = alignment
        if alignment < alignment_limit:
            reasons.append(f"{sensor_name}_optical_axis_{axis}_sign")
    return OpticalAxisQualification(
        axis_alignment=alignments,
        reject_reasons=tuple(reasons),
    )


@dataclass(frozen=True)
class PoseOcclusionQualification:
    occlusion_by_pose: Mapping[str, float]
    reject_reasons: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.reject_reasons


def qualify_pose_occlusion(
    occlusion_by_pose: Mapping[str, float],
    *,
    required_poses: Sequence[str],
    max_occlusion_ratio: float,
) -> PoseOcclusionQualification:
    limit = _finite(max_occlusion_ratio, "max_occlusion_ratio")
    if not 0.0 <= limit <= 1.0:
        raise ValueError("max_occlusion_ratio must be within 0..1")
    values = {
        pose: _finite(value, f"{pose} occlusion")
        for pose, value in occlusion_by_pose.items()
    }
    if any(not 0.0 <= value <= 1.0 for value in values.values()):
        raise ValueError("occlusion ratios must be within 0..1")
    reasons = [f"missing_pose:{pose}" for pose in required_poses if pose not in values]
    reasons.extend(
        f"roi_occlusion:{pose}"
        for pose in required_poses
        if pose in values and values[pose] > limit
    )
    return PoseOcclusionQualification(values, tuple(reasons))


@dataclass(frozen=True)
class ExtrinsicRepeatabilityQualification:
    max_spread_by_pose_m: Mapping[str, float]
    reject_reasons: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.reject_reasons


def qualify_extrinsic_repeatability(
    target_errors_by_pose_m: Mapping[str, Sequence[Sequence[float]]],
    *,
    required_poses: Sequence[str],
    max_spread_m: float,
) -> ExtrinsicRepeatabilityQualification:
    limit = _finite(max_spread_m, "max_spread_m")
    if limit < 0.0:
        raise ValueError("max_spread_m must be nonnegative")
    spreads: dict[str, float] = {}
    reasons: list[str] = []
    for pose in required_poses:
        samples = target_errors_by_pose_m.get(pose)
        if samples is None or len(samples) < 2:
            reasons.append(f"missing_repeatability_samples:{pose}")
            continue
        points = np.asarray(
            [_finite_tuple(sample, f"{pose} target error", 3) for sample in samples]
        )
        differences = points[:, None, :] - points[None, :, :]
        spread = float(np.max(np.linalg.norm(differences, axis=2)))
        spreads[pose] = spread
        if spread > limit:
            reasons.append(f"extrinsic_repeatability:{pose}")
    return ExtrinsicRepeatabilityQualification(spreads, tuple(reasons))


@dataclass(frozen=True)
class PitchMetrics:
    pitch_deg: float
    near_blind_spot_m: float
    coverage_min_m: float
    coverage_max_m: float
    footprint_clearance_m: float
    below_floor_separation_m: float

    def __post_init__(self) -> None:
        for name in (
            "pitch_deg",
            "near_blind_spot_m",
            "coverage_min_m",
            "coverage_max_m",
            "footprint_clearance_m",
            "below_floor_separation_m",
        ):
            value = _finite(getattr(self, name), name)
            object.__setattr__(self, name, value)
        if min(
            self.near_blind_spot_m,
            self.coverage_min_m,
            self.coverage_max_m,
            self.footprint_clearance_m,
            self.below_floor_separation_m,
        ) < 0.0:
            raise ValueError("pitch distance metrics must be nonnegative metres")
        if self.coverage_min_m > self.coverage_max_m:
            raise ValueError("coverage range must be ordered")


@dataclass(frozen=True)
class PitchRequirements:
    max_near_blind_spot_m: float
    required_coverage_min_m: float
    required_coverage_max_m: float
    min_footprint_clearance_m: float
    min_below_floor_separation_m: float

    def __post_init__(self) -> None:
        for name in (
            "max_near_blind_spot_m",
            "required_coverage_min_m",
            "required_coverage_max_m",
            "min_footprint_clearance_m",
            "min_below_floor_separation_m",
        ):
            value = _finite(getattr(self, name), name)
            if value < 0.0:
                raise ValueError("pitch requirements must be nonnegative metres")
            object.__setattr__(self, name, value)
        if self.required_coverage_min_m > self.required_coverage_max_m:
            raise ValueError("required coverage range must be ordered")


@dataclass(frozen=True)
class PitchCandidateQualification:
    metrics: PitchMetrics
    reject_reasons: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.reject_reasons


@dataclass(frozen=True)
class PitchBracketQualification:
    candidates: tuple[PitchCandidateQualification, ...]
    reject_reasons: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.reject_reasons and all(candidate.passed for candidate in self.candidates)


def qualify_pitch_candidate(
    metrics: PitchMetrics,
    *,
    requirements: PitchRequirements,
) -> PitchCandidateQualification:
    reasons: list[str] = []
    if metrics.near_blind_spot_m > requirements.max_near_blind_spot_m:
        reasons.append("near_blind_spot")
    if metrics.coverage_min_m > requirements.required_coverage_min_m:
        reasons.append("coverage_near_limit")
    if metrics.coverage_max_m < requirements.required_coverage_max_m:
        reasons.append("coverage_far_limit")
    if metrics.footprint_clearance_m < requirements.min_footprint_clearance_m:
        reasons.append("footprint_clearance")
    if metrics.below_floor_separation_m < requirements.min_below_floor_separation_m:
        reasons.append("below_floor_separation")
    return PitchCandidateQualification(metrics=metrics, reject_reasons=tuple(reasons))


def qualify_pitch_bracket(
    candidates: Sequence[PitchMetrics],
    *,
    requirements: PitchRequirements,
    required_pitches_deg: Sequence[float] = (20.0, 25.0, 30.0),
) -> PitchBracketQualification:
    required = tuple(float(value) for value in required_pitches_deg)
    counts = {pitch: 0 for pitch in required}
    unexpected: list[float] = []
    for candidate in candidates:
        if candidate.pitch_deg in counts:
            counts[candidate.pitch_deg] += 1
        else:
            unexpected.append(candidate.pitch_deg)
    reasons: list[str] = []
    for pitch in required:
        if counts[pitch] == 0:
            reasons.append(f"missing_pitch_candidate:{pitch:g}")
        elif counts[pitch] > 1:
            reasons.append(f"duplicate_pitch_candidate:{pitch:g}")
    reasons.extend(f"unexpected_pitch_candidate:{pitch:g}" for pitch in sorted(unexpected))
    qualified = tuple(
        qualify_pitch_candidate(candidate, requirements=requirements)
        for candidate in sorted(candidates, key=lambda item: item.pitch_deg)
    )
    return PitchBracketQualification(
        candidates=qualified,
        reject_reasons=tuple(reasons),
    )
