"""Load and validate the simulator-neutral ``scenario.yaml`` contract."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from types import MappingProxyType
from typing import Any

import yaml


SCHEMA_VERSION = 1
PRNG_ALGORITHM = "PCG64"
SEED_CLASSES = frozenset(
    {"dev", "regression", "hidden_evaluation", "stress"}
)
REQUIRED_UNITS = {
    "distance": "m",
    "angle": "rad",
    "time": "s",
    "linear_velocity": "m/s",
    "angular_velocity": "rad/s",
    "acceleration": "m/s^2",
    "curvature": "1/m",
    "friction": "1",
    "depth": "mm",
}


class ScenarioValidationError(ValueError):
    """Raised when a scenario does not satisfy schema version 1."""


@dataclass(frozen=True)
class ClockConfig:
    start_s: float
    dt_s: float
    duration_s: float
    sample_count: int


@dataclass(frozen=True)
class PrngConfig:
    algorithm: str
    seed: int
    seed_class: str


@dataclass(frozen=True)
class DropBoundary:
    left: bool
    right: bool


@dataclass(frozen=True)
class TrackConfig:
    centerline_m: tuple[tuple[float, float, float], ...]
    width_m: tuple[float, ...]
    height_m: tuple[float, ...]
    bank_rad: tuple[float, ...]
    curvature_per_m: tuple[float, ...]
    friction_coefficient: tuple[float, ...]
    drop_boundaries: tuple[DropBoundary, ...]


@dataclass(frozen=True)
class Scenario:
    schema_version: int
    scenario_id: str
    description: str
    units: Mapping[str, str]
    frames: Mapping[str, str]
    clock: ClockConfig
    prng: PrngConfig
    track: TrackConfig
    motion: Mapping[str, Any]
    sensors: Mapping[str, Any]
    faults: Mapping[str, Any]
    expected_metrics: Mapping[str, Any]


def _error(path: str, message: str) -> ScenarioValidationError:
    return ScenarioValidationError(f"{path}: {message}")


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise _error(path, "must be a mapping")
    return value


def _sequence(value: Any, path: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise _error(path, "must be a sequence")
    return value


def _required(mapping: Mapping[str, Any], keys: Sequence[str], path: str) -> None:
    for key in keys:
        if key not in mapping:
            label = f"{path}.{key}" if path else key
            raise _error(label, "missing required key")


def _no_unknown(
    mapping: Mapping[str, Any], allowed: Sequence[str], path: str
) -> None:
    unknown = sorted(set(mapping) - set(allowed))
    if unknown:
        raise _error(path or "scenario", f"unknown keys: {', '.join(unknown)}")


def _finite_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _error(path, "must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise _error(path, "must be finite")
    return value


def _integer(value: Any, path: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise _error(path, f"must be an integer >= {minimum}")
    return value


def _nonempty_text(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise _error(path, "must be a non-empty string")
    return value


def _reject_nonfinite(value: Any, path: str = "scenario") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise _error(path, "values must be finite")
    if isinstance(value, Mapping):
        for key, item in value.items():
            _reject_nonfinite(item, f"{path}.{key}")
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for index, item in enumerate(value):
            _reject_nonfinite(item, f"{path}[{index}]")


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType(
            {str(key): _freeze(value[key]) for key in sorted(value)}
        )
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return tuple(_freeze(item) for item in value)
    return value


def _parse_units(document: Mapping[str, Any]) -> Mapping[str, str]:
    units = _mapping(document["units"], "units")
    _required(units, tuple(REQUIRED_UNITS), "units")
    unknown = sorted(set(units) - set(REQUIRED_UNITS))
    if unknown:
        raise _error("units", f"unknown unit dimensions: {', '.join(unknown)}")
    for name, expected in REQUIRED_UNITS.items():
        if units[name] != expected:
            raise _error(f"units.{name}", f"must be exactly {expected!r}")
    return _freeze(units)


def _parse_frames(document: Mapping[str, Any]) -> Mapping[str, str]:
    frames = _mapping(document["frames"], "frames")
    required = ("world", "body", "depth", "imu")
    _required(frames, required, "frames")
    _no_unknown(frames, required, "frames")
    normalized = {key: _nonempty_text(frames[key], f"frames.{key}") for key in required}
    if len(set(normalized.values())) != len(normalized):
        raise _error("frames", "frame names must be unique")
    return _freeze(normalized)


def _parse_clock(document: Mapping[str, Any]) -> ClockConfig:
    clock = _mapping(document["clock"], "clock")
    _required(clock, ("start_s", "dt_s", "duration_s"), "clock")
    _no_unknown(clock, ("start_s", "dt_s", "duration_s"), "clock")
    start_s = _finite_number(clock["start_s"], "clock.start_s")
    dt_s = _finite_number(clock["dt_s"], "clock.dt_s")
    duration_s = _finite_number(clock["duration_s"], "clock.duration_s")
    if start_s <= 0.0:
        raise _error("clock.start_s", "must be positive")
    if dt_s <= 0.0 or duration_s <= 0.0:
        raise _error("clock", "dt_s and duration_s must be positive")
    intervals = duration_s / dt_s
    rounded = round(intervals)
    if not math.isclose(intervals, rounded, rel_tol=0.0, abs_tol=1e-9):
        raise _error("clock.duration_s", "must be an integer multiple of dt_s")
    return ClockConfig(start_s, dt_s, duration_s, int(rounded) + 1)


def _parse_prng(document: Mapping[str, Any]) -> PrngConfig:
    prng = _mapping(document["prng"], "prng")
    _required(prng, ("algorithm", "seed", "seed_class"), "prng")
    _no_unknown(prng, ("algorithm", "seed", "seed_class"), "prng")
    algorithm = _nonempty_text(prng["algorithm"], "prng.algorithm")
    if algorithm != PRNG_ALGORITHM:
        raise _error("prng.algorithm", f"only {PRNG_ALGORITHM} is supported")
    seed = _integer(prng["seed"], "prng.seed")
    if seed >= 2**128:
        raise _error("prng.seed", "must be less than 2^128")
    seed_class = _nonempty_text(prng["seed_class"], "prng.seed_class")
    if seed_class not in SEED_CLASSES:
        raise _error(
            "prng.seed_class",
            "must be dev, regression, hidden_evaluation, or stress",
        )
    return PrngConfig(algorithm, seed, seed_class)


def _float_profile(
    track: Mapping[str, Any], name: str, count: int, *, positive: bool = False
) -> tuple[float, ...]:
    values = _sequence(track[name], f"track.{name}")
    if len(values) != count:
        raise _error(f"track.{name}", f"must contain {count} values")
    normalized = tuple(
        _finite_number(value, f"track.{name}[{index}]")
        for index, value in enumerate(values)
    )
    if positive and any(value <= 0.0 for value in normalized):
        raise _error(f"track.{name}", "values must be positive")
    return normalized


def _parse_track(document: Mapping[str, Any]) -> TrackConfig:
    track = _mapping(document["track"], "track")
    fields = (
        "centerline_m",
        "width_m",
        "height_m",
        "bank_rad",
        "curvature_per_m",
        "friction_coefficient",
        "drop_boundaries",
    )
    _required(track, fields, "track")
    _no_unknown(track, fields, "track")
    raw_centerline = _sequence(track["centerline_m"], "track.centerline_m")
    if len(raw_centerline) < 2:
        raise _error("track.centerline_m", "must contain at least two 3D points")
    centerline = []
    for index, raw_point in enumerate(raw_centerline):
        point = _sequence(raw_point, f"track.centerline_m[{index}]")
        if len(point) != 3:
            raise _error(f"track.centerline_m[{index}]", "must be a 3D point")
        centerline.append(
            tuple(
                _finite_number(value, f"track.centerline_m[{index}][{axis}]")
                for axis, value in enumerate(point)
            )
        )
    for left, right in zip(centerline, centerline[1:]):
        if left == right:
            raise _error("track.centerline_m", "adjacent points must be distinct")

    count = len(centerline)
    widths = _float_profile(track, "width_m", count, positive=True)
    heights = _float_profile(track, "height_m", count)
    if any(height < 0.0 for height in heights):
        raise _error("track.height_m", "values must be nonnegative")
    if any(
        not math.isclose(point[2], height, rel_tol=0.0, abs_tol=1e-9)
        for point, height in zip(centerline, heights)
    ):
        raise _error(
            "track.height_m",
            "must match the z coordinate of centerline_m at every station",
        )
    bank = _float_profile(track, "bank_rad", count)
    if any(abs(value) >= math.pi / 2.0 for value in bank):
        raise _error("track.bank_rad", "absolute values must be less than pi/2")
    curvature = _float_profile(track, "curvature_per_m", count)
    friction = _float_profile(
        track, "friction_coefficient", count, positive=True
    )

    raw_boundaries = _sequence(track["drop_boundaries"], "track.drop_boundaries")
    if len(raw_boundaries) != count:
        raise _error("track.drop_boundaries", f"must contain {count} values")
    boundaries = []
    for index, raw_boundary in enumerate(raw_boundaries):
        boundary = _mapping(raw_boundary, f"track.drop_boundaries[{index}]")
        _required(boundary, ("left", "right"), f"track.drop_boundaries[{index}]")
        if not isinstance(boundary["left"], bool) or not isinstance(
            boundary["right"], bool
        ):
            raise _error(
                f"track.drop_boundaries[{index}]", "left and right must be boolean"
            )
        boundaries.append(DropBoundary(boundary["left"], boundary["right"]))

    return TrackConfig(
        tuple(centerline),
        widths,
        heights,
        bank,
        curvature,
        friction,
        tuple(boundaries),
    )


def _parse_motion(
    document: Mapping[str, Any], duration_s: float
) -> Mapping[str, Any]:
    motion = _mapping(document["motion"], "motion")
    _required(motion, ("profile",), "motion")
    profile = _nonempty_text(motion["profile"], "motion.profile")
    required_by_profile = {
        "constant_speed": ("linear_speed_m_s", "yaw_rate_rad_s"),
        "pivot": ("target_yaw_rad", "yaw_rate_rad_s"),
        "trapezoidal_speed": (
            "max_linear_speed_m_s",
            "linear_acceleration_m_s2",
            "yaw_rate_rad_s",
        ),
    }
    if profile not in required_by_profile:
        raise _error("motion.profile", "unknown motion profile")
    _required(motion, required_by_profile[profile], "motion")
    _no_unknown(motion, ("profile", *required_by_profile[profile]), "motion")
    for key in required_by_profile[profile]:
        value = _finite_number(motion[key], f"motion.{key}")
        if key in {
            "max_linear_speed_m_s",
            "linear_acceleration_m_s2",
        } and value <= 0.0:
            raise _error(f"motion.{key}", "must be positive")
    if profile == "pivot":
        target = float(motion["target_yaw_rad"])
        rate = float(motion["yaw_rate_rad_s"])
        if target == 0.0:
            raise _error("motion.target_yaw_rad", "must be nonzero")
        if rate == 0.0:
            raise _error("motion.yaw_rate_rad_s", "must be nonzero")
        if math.copysign(1.0, target) != math.copysign(1.0, rate):
            raise _error("motion", "target_yaw_rad and yaw_rate_rad_s sign must match")
        if abs(rate) * duration_s + 1e-12 < abs(target):
            raise _error("motion", "target yaw cannot be reached within clock duration")
    return _freeze(motion)


def _sample_stride(config: Mapping[str, Any], path: str) -> int:
    _required(config, ("sample_every_n_steps",), path)
    return _integer(config["sample_every_n_steps"], f"{path}.sample_every_n_steps", minimum=1)


def _vector3(config: Mapping[str, Any], key: str, path: str) -> tuple[float, float, float]:
    values = _sequence(config[key], f"{path}.{key}")
    if len(values) != 3:
        raise _error(f"{path}.{key}", "must contain three values")
    return tuple(
        _finite_number(value, f"{path}.{key}[{index}]")
        for index, value in enumerate(values)
    )


def _parse_sensors(document: Mapping[str, Any]) -> Mapping[str, Any]:
    sensors = _mapping(document["sensors"], "sensors")
    _required(sensors, ("wheel_states", "imu", "depth"), "sensors")
    _no_unknown(sensors, ("wheel_states", "imu", "depth"), "sensors")

    wheel = _mapping(sensors["wheel_states"], "sensors.wheel_states")
    _sample_stride(wheel, "sensors.wheel_states")
    wheel_fields = (
        "sample_every_n_steps",
        "noise_std_turns_per_s",
        "wheel_names",
    )
    _required(wheel, wheel_fields, "sensors.wheel_states")
    _no_unknown(wheel, wheel_fields, "sensors.wheel_states")
    if _finite_number(
        wheel["noise_std_turns_per_s"],
        "sensors.wheel_states.noise_std_turns_per_s",
    ) < 0.0:
        raise _error("sensors.wheel_states.noise_std_turns_per_s", "must be nonnegative")
    raw_wheel_names = _sequence(
        wheel["wheel_names"], "sensors.wheel_states.wheel_names"
    )
    wheel_names = tuple(
        _nonempty_text(name, f"sensors.wheel_states.wheel_names[{index}]")
        for index, name in enumerate(raw_wheel_names)
    )
    if not wheel_names or len(set(wheel_names)) != len(wheel_names):
        raise _error(
            "sensors.wheel_states.wheel_names",
            "must contain unique wheel identifiers",
        )

    imu = _mapping(sensors["imu"], "sensors.imu")
    _sample_stride(imu, "sensors.imu")
    imu_fields = (
        "gyro_bias_rad_s",
        "gyro_noise_std_rad_s",
        "accel_bias_m_s2",
        "accel_noise_std_m_s2",
        "gravity_m_s2",
    )
    _required(imu, imu_fields, "sensors.imu")
    _no_unknown(imu, ("sample_every_n_steps", *imu_fields), "sensors.imu")
    _vector3(imu, "gyro_bias_rad_s", "sensors.imu")
    _vector3(imu, "accel_bias_m_s2", "sensors.imu")
    for key in ("gyro_noise_std_rad_s", "accel_noise_std_m_s2"):
        if _finite_number(imu[key], f"sensors.imu.{key}") < 0.0:
            raise _error(f"sensors.imu.{key}", "must be nonnegative")
    if _finite_number(imu["gravity_m_s2"], "sensors.imu.gravity_m_s2") <= 0.0:
        raise _error("sensors.imu.gravity_m_s2", "must be positive")

    depth = _mapping(sensors["depth"], "sensors.depth")
    _sample_stride(depth, "sensors.depth")
    depth_fields = (
        "shape_px",
        "depth_scale_m",
        "base_depth_m",
        "noise_std_m",
        "bank_depth_span_m",
        "intrinsics_px",
    )
    _required(depth, depth_fields, "sensors.depth")
    _no_unknown(depth, ("sample_every_n_steps", *depth_fields), "sensors.depth")
    shape = _sequence(depth["shape_px"], "sensors.depth.shape_px")
    if len(shape) != 2 or any(
        _integer(value, f"sensors.depth.shape_px[{index}]", minimum=3) < 3
        for index, value in enumerate(shape)
    ):
        raise _error("sensors.depth.shape_px", "must be [height, width] >= 3")
    for key in ("depth_scale_m", "base_depth_m"):
        if _finite_number(depth[key], f"sensors.depth.{key}") <= 0.0:
            raise _error(f"sensors.depth.{key}", "must be positive")
    for key in ("noise_std_m", "bank_depth_span_m"):
        if _finite_number(depth[key], f"sensors.depth.{key}") < 0.0:
            raise _error(f"sensors.depth.{key}", "must be nonnegative")
    intrinsics = _mapping(depth["intrinsics_px"], "sensors.depth.intrinsics_px")
    _required(intrinsics, ("fx", "fy", "cx", "cy"), "sensors.depth.intrinsics_px")
    _no_unknown(
        intrinsics, ("fx", "fy", "cx", "cy"), "sensors.depth.intrinsics_px"
    )
    for key in ("fx", "fy", "cx", "cy"):
        value = _finite_number(
            intrinsics[key], f"sensors.depth.intrinsics_px.{key}"
        )
        if key in {"fx", "fy"} and value <= 0.0:
            raise _error(f"sensors.depth.intrinsics_px.{key}", "must be positive")
    return _freeze(sensors)


def _fault_interval(fault: Mapping[str, Any], path: str, duration_s: float) -> None:
    _required(fault, ("start_s", "end_s"), path)
    start_s = _finite_number(fault["start_s"], f"{path}.start_s")
    end_s = _finite_number(fault["end_s"], f"{path}.end_s")
    if start_s < 0.0 or end_s <= start_s or end_s > duration_s:
        raise _error(path, "interval must satisfy 0 <= start_s < end_s <= duration_s")


def _parse_faults(
    document: Mapping[str, Any],
    duration_s: float,
    *,
    wheel_names: Sequence[str],
    depth_shape: Sequence[int],
) -> Mapping[str, Any]:
    faults = _mapping(document["faults"], "faults")
    groups = ("wheel_slip", "sensor_dropouts", "depth_holes", "depth_spikes")
    _required(faults, groups, "faults")
    _no_unknown(faults, groups, "faults")
    for group in groups:
        entries = _sequence(faults[group], f"faults.{group}")
        for index, raw_entry in enumerate(entries):
            path = f"faults.{group}[{index}]"
            entry = _mapping(raw_entry, path)
            _fault_interval(entry, path, duration_s)
            if group == "wheel_slip":
                fields = ("wheel", "start_s", "end_s", "measurement_scale")
                _required(entry, fields, path)
                _no_unknown(entry, fields, path)
                wheel = _nonempty_text(entry["wheel"], f"{path}.wheel")
                if wheel not in wheel_names:
                    raise _error(f"{path}.wheel", "must name a configured wheel")
                if _finite_number(entry["measurement_scale"], f"{path}.measurement_scale") < 0.0:
                    raise _error(f"{path}.measurement_scale", "must be nonnegative")
            elif group == "sensor_dropouts":
                fields = ("stream", "start_s", "end_s")
                _required(entry, fields, path)
                _no_unknown(entry, fields, path)
                stream = _nonempty_text(entry["stream"], f"{path}.stream")
                if stream not in {"wheel", "imu", "depth"}:
                    raise _error(f"{path}.stream", "must be wheel, imu, or depth")
            elif group == "depth_holes":
                fields = ("rows", "cols", "start_s", "end_s")
                _required(entry, fields, path)
                _no_unknown(entry, fields, path)
                for key in ("rows", "cols"):
                    bounds = _sequence(entry[key], f"{path}.{key}")
                    if len(bounds) != 2:
                        raise _error(f"{path}.{key}", "must contain [start, stop]")
                    start = _integer(bounds[0], f"{path}.{key}[0]")
                    stop = _integer(bounds[1], f"{path}.{key}[1]")
                    if stop <= start:
                        raise _error(f"{path}.{key}", "stop must be greater than start")
                    limit = int(depth_shape[0 if key == "rows" else 1])
                    if stop > limit:
                        raise _error(
                            f"{path}.{key}",
                            "bounds must fit sensors.depth.shape_px",
                        )
            else:
                fields = ("row", "col", "offset_m", "start_s", "end_s")
                _required(entry, fields, path)
                _no_unknown(entry, fields, path)
                row = _integer(entry["row"], f"{path}.row")
                col = _integer(entry["col"], f"{path}.col")
                if row >= int(depth_shape[0]) or col >= int(depth_shape[1]):
                    raise _error(
                        path, "row and col must fit sensors.depth.shape_px"
                    )
                _finite_number(entry["offset_m"], f"{path}.offset_m")
    return _freeze(faults)


def _parse_expected_metrics(document: Mapping[str, Any]) -> Mapping[str, Any]:
    metrics = _mapping(document["expected_metrics"], "expected_metrics")
    fields = (
        "completion",
        "min_clearance_m",
        "edge_overrun_count",
        "false_hold_count",
        "fail_open_count",
        "max_recovery_time_s",
        "max_estimator_runtime_ms",
    )
    _required(metrics, fields, "expected_metrics")
    _no_unknown(metrics, fields, "expected_metrics")
    if not isinstance(metrics["completion"], bool):
        raise _error("expected_metrics.completion", "must be boolean")
    for key in (
        "min_clearance_m",
        "max_recovery_time_s",
        "max_estimator_runtime_ms",
    ):
        if _finite_number(metrics[key], f"expected_metrics.{key}") < 0.0:
            raise _error(f"expected_metrics.{key}", "must be nonnegative")
    for key in ("edge_overrun_count", "false_hold_count", "fail_open_count"):
        _integer(metrics[key], f"expected_metrics.{key}")
    return _freeze(metrics)


def parse_scenario(document: Mapping[str, Any]) -> Scenario:
    """Validate one decoded YAML mapping and return its normalized contract."""
    document = _mapping(document, "scenario")
    required = (
        "schema_version",
        "scenario_id",
        "description",
        "units",
        "frames",
        "clock",
        "prng",
        "track",
        "motion",
        "sensors",
        "faults",
        "expected_metrics",
    )
    _required(document, required, "")
    _no_unknown(document, required, "")
    _reject_nonfinite(document)
    version = document["schema_version"]
    if isinstance(version, bool) or version != SCHEMA_VERSION:
        raise _error("schema_version", f"must be {SCHEMA_VERSION}")
    scenario_id = _nonempty_text(document["scenario_id"], "scenario_id")
    description = _nonempty_text(document["description"], "description")
    clock = _parse_clock(document)
    sensors = _parse_sensors(document)
    return Scenario(
        schema_version=version,
        scenario_id=scenario_id,
        description=description,
        units=_parse_units(document),
        frames=_parse_frames(document),
        clock=clock,
        prng=_parse_prng(document),
        track=_parse_track(document),
        motion=_parse_motion(document, clock.duration_s),
        sensors=sensors,
        faults=_parse_faults(
            document,
            clock.duration_s,
            wheel_names=sensors["wheel_states"]["wheel_names"],
            depth_shape=sensors["depth"]["shape_px"],
        ),
        expected_metrics=_parse_expected_metrics(document),
    )


def load_scenario(path: str | Path) -> Scenario:
    """Load one YAML file; the file is the sole scenario parameter owner."""
    path = Path(path)
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ScenarioValidationError(f"{path}: invalid YAML: {exc}") from exc
    return parse_scenario(document)
