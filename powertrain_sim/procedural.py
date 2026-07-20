"""Deterministic procedural elevated-track scenario generation.

Only ``dev`` and ``regression`` seeds may be inspected while tuning.  A
``hidden_evaluation`` seed is generated and validated only for completion
evidence; its generated document must not be inspected or used to tune
algorithms or parameters.  ``stress`` seeds deliberately intensify declared
fault schedules.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from chassis.kinematics import default_geometry

from .scenario import PRNG_ALGORITHM, REQUIRED_UNITS, SEED_CLASSES, parse_scenario


DEFAULT_TERRAIN_FAMILIES = ("flat", "bank", "bank_transition")
TERRAIN_FAMILIES = DEFAULT_TERRAIN_FAMILIES + ("undulating",)
MOTION_PROFILES = ("constant_speed", "trapezoidal_speed")
CURVATURE_MODES = ("constant", "clothoid")
MAX_CLOTHOID_RATE_PER_M2 = 0.08
L515_DEPTH_HORIZONTAL_FOV_RAD = math.radians(70.0)
L515_DEPTH_VERTICAL_FOV_RAD = math.radians(55.0)


def _range(
    values: tuple[float, float],
    name: str,
    *,
    minimum: float | None = None,
) -> tuple[float, float]:
    if (
        not isinstance(values, tuple)
        or len(values) != 2
        or any(isinstance(value, bool) for value in values)
        or not all(isinstance(value, (int, float)) for value in values)
    ):
        raise ValueError(f"{name} must be a two-number tuple")
    low, high = (float(value) for value in values)
    if not math.isfinite(low) or not math.isfinite(high) or low > high:
        raise ValueError(f"{name} must be finite and ordered")
    if minimum is not None and low < minimum:
        raise ValueError(f"{name} must be >= {minimum}")
    return low, high


@dataclass(frozen=True)
class PinchSpec:
    """One deterministic narrowed interval along the generated centreline."""

    center_ratio: float
    length_m: float
    width_m: float

    def __post_init__(self) -> None:
        values = {
            "center_ratio": self.center_ratio,
            "length_m": self.length_m,
            "width_m": self.width_m,
        }
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in values.values()
        ):
            raise ValueError("pinch values must be finite numbers")
        if not 0.0 <= float(self.center_ratio) <= 1.0:
            raise ValueError("pinch center_ratio must be within [0, 1]")
        if float(self.length_m) <= 0.0:
            raise ValueError("pinch length_m must be positive")
        if float(self.width_m) <= 0.0:
            raise ValueError("pinch width_m must be positive")


@dataclass(frozen=True)
class FrictionPatchSpec:
    """One deterministic friction interval along the generated centreline."""

    center_ratio: float
    length_m: float
    mu: float

    def __post_init__(self) -> None:
        values = {
            "center_ratio": self.center_ratio,
            "length_m": self.length_m,
            "mu": self.mu,
        }
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in values.values()
        ):
            raise ValueError("friction patch values must be finite numbers")
        if not 0.0 <= float(self.center_ratio) <= 1.0:
            raise ValueError("friction patch center_ratio must be within [0, 1]")
        if float(self.length_m) <= 0.0:
            raise ValueError("friction patch length_m must be positive")
        if float(self.mu) <= 0.0:
            raise ValueError("friction patch mu must be positive")


@dataclass(frozen=True)
class GenerationParameters:
    """All tunable ranges used by the PCG64 procedural generator."""

    track_length_range_m: tuple[float, float] = (5.0, 8.0)
    track_width_range_m: tuple[float, float] = (1.2, 1.6)
    track_height_range_m: tuple[float, float] = (0.4, 0.7)
    bank_magnitude_range_rad: tuple[float, float] = (0.08, 0.22)
    curvature_range_per_m: tuple[float, float] = (-0.12, 0.12)
    friction_range: tuple[float, float] = (0.65, 1.0)
    station_spacing_range_m: tuple[float, float] = (0.25, 0.45)
    linear_speed_range_m_s: tuple[float, float] = (0.35, 0.60)
    linear_acceleration_range_m_s2: tuple[float, float] = (0.20, 0.40)
    terrain_families: tuple[str, ...] = DEFAULT_TERRAIN_FAMILIES
    motion_profiles: tuple[str, ...] = MOTION_PROFILES
    clock_dt_s: float = 0.02
    # TerrainEstimatorConfig.depth_shape_px 와 같은 값이어야 한다 — 시뮬이
    # production 보다 좋은 센서를 갖게 되면 검증 의미가 사라진다.
    depth_shape_px: tuple[int, int] = (120, 160)
    depth_sample_every_n_steps: int = 5
    # 개루프(스크립트 모션) 검증은 트랙 끝까지 가므로 True가 기본이다.
    # 폐루프(P1)는 고가 트랙 종단 낙하 앞 ~0.55 m(≈전방 코너 반경)에서
    # fail-closed 정지하는 것이 옳으므로 95% 완주가 물리적으로 불가 — False로
    # 생성한다(정지 여유가 5%를 넘는 모든 기본 트랙 길이에서 성립).
    expected_completion: bool = True
    pinch: PinchSpec | None = None
    friction_patch: FrictionPatchSpec | None = None
    curvature_mode: str = "constant"
    undulation_amplitude_m: float = 0.05
    undulation_wavelength_m: float = 2.0

    def __post_init__(self) -> None:
        _range(self.track_length_range_m, "track_length_range_m", minimum=1.0)
        _range(self.track_width_range_m, "track_width_range_m", minimum=0.2)
        _range(self.track_height_range_m, "track_height_range_m", minimum=0.2)
        _range(
            self.bank_magnitude_range_rad,
            "bank_magnitude_range_rad",
            minimum=0.0,
        )
        curvature = _range(self.curvature_range_per_m, "curvature_range_per_m")
        if curvature[0] > 0.0 or curvature[1] < 0.0:
            raise ValueError("curvature_range_per_m must include zero")
        _range(self.friction_range, "friction_range", minimum=0.05)
        _range(
            self.station_spacing_range_m,
            "station_spacing_range_m",
            minimum=0.05,
        )
        _range(self.linear_speed_range_m_s, "linear_speed_range_m_s", minimum=0.01)
        _range(
            self.linear_acceleration_range_m_s2,
            "linear_acceleration_range_m_s2",
            minimum=0.01,
        )
        if not self.terrain_families or not set(self.terrain_families) <= set(
            TERRAIN_FAMILIES
        ):
            raise ValueError(f"terrain_families must come from {TERRAIN_FAMILIES}")
        if not self.motion_profiles or not set(self.motion_profiles) <= set(
            MOTION_PROFILES
        ):
            raise ValueError(f"motion_profiles must come from {MOTION_PROFILES}")
        if not math.isfinite(self.clock_dt_s) or self.clock_dt_s <= 0.0:
            raise ValueError("clock_dt_s must be finite and positive")
        if (
            len(self.depth_shape_px) != 2
            or any(
                isinstance(value, bool) or not isinstance(value, int) or value < 3
                for value in self.depth_shape_px
            )
        ):
            raise ValueError("depth_shape_px must contain two integers >= 3")
        if (
            isinstance(self.depth_sample_every_n_steps, bool)
            or not isinstance(self.depth_sample_every_n_steps, int)
            or self.depth_sample_every_n_steps < 1
        ):
            raise ValueError("depth_sample_every_n_steps must be a positive integer")
        if self.pinch is not None and not isinstance(self.pinch, PinchSpec):
            raise ValueError("pinch must be PinchSpec or None")
        if self.friction_patch is not None and not isinstance(
            self.friction_patch,
            FrictionPatchSpec,
        ):
            raise ValueError("friction_patch must be FrictionPatchSpec or None")
        if self.curvature_mode not in CURVATURE_MODES:
            raise ValueError(f"curvature_mode must come from {CURVATURE_MODES}")
        for name, value, allow_zero in (
            ("undulation_amplitude_m", self.undulation_amplitude_m, True),
            ("undulation_wavelength_m", self.undulation_wavelength_m, False),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or (float(value) < 0.0 if allow_zero else float(value) <= 0.0)
            ):
                qualifier = "nonnegative" if allow_zero else "positive"
                raise ValueError(f"{name} must be a finite {qualifier} number")
        if float(self.undulation_amplitude_m) > float(
            self.track_height_range_m[0]
        ):
            raise ValueError(
                "undulation_amplitude_m must not exceed the minimum track height"
            )


def _draw(rng: np.random.Generator, values: tuple[float, float]) -> float:
    low, high = (float(value) for value in values)
    if low == high:
        return low
    return float(rng.uniform(low, high))


def _rounded(value: float) -> float:
    return float(round(float(value), 12))


def _smooth_profile(
    rng: np.random.Generator,
    count: int,
    value_range: tuple[float, float],
) -> list[float]:
    knot_count = min(5, count)
    knots = rng.uniform(value_range[0], value_range[1], size=knot_count)
    values = np.interp(
        np.linspace(0.0, 1.0, count),
        np.linspace(0.0, 1.0, knot_count),
        knots,
    )
    return [_rounded(value) for value in values]


def _duration_for_motion(
    profile: str,
    length_m: float,
    speed_m_s: float,
    acceleration_m_s2: float,
    dt_s: float,
) -> float:
    if profile == "constant_speed":
        ideal_s = length_m / speed_m_s
    elif length_m <= speed_m_s * speed_m_s / acceleration_m_s2:
        ideal_s = 2.0 * math.sqrt(length_m / acceleration_m_s2)
    else:
        ideal_s = length_m / speed_m_s + speed_m_s / acceleration_m_s2
    intervals = max(1, int(math.ceil(ideal_s / dt_s - 1e-12)))
    return _rounded(intervals * dt_s)


def _interval(
    rng: np.random.Generator,
    duration_s: float,
    *,
    width_fraction: float,
) -> tuple[float, float]:
    width_s = max(0.02, duration_s * width_fraction)
    latest_start = max(0.0, duration_s - width_s)
    start_s = float(rng.uniform(0.1 * latest_start, 0.9 * latest_start))
    return _rounded(start_s), _rounded(min(duration_s, start_s + width_s))


def _faults(
    rng: np.random.Generator,
    *,
    seed_class: str,
    duration_s: float,
    depth_shape: tuple[int, int],
    wheel_names: tuple[str, ...],
) -> dict[str, list[dict[str, Any]]]:
    stress = seed_class == "stress"
    dropout_count = 3 if stress else 1
    dropouts = []
    streams = ("wheel", "imu", "depth")
    for index in range(dropout_count):
        start_s, end_s = _interval(
            rng,
            duration_s,
            width_fraction=0.045 if stress else 0.02,
        )
        dropouts.append(
            {
                "stream": streams[index] if stress else str(rng.choice(streams)),
                "start_s": start_s,
                "end_s": end_s,
            }
        )

    rows, cols = depth_shape
    hole_height = max(2, rows // (4 if stress else 6))
    hole_width = max(2, cols // (4 if stress else 6))
    row_start = int(rng.integers(1, rows - hole_height))
    col_start = int(rng.integers(1, cols - hole_width))
    hole_start, hole_end = _interval(
        rng,
        duration_s,
        width_fraction=0.06 if stress else 0.025,
    )
    spike_start, spike_end = _interval(
        rng,
        duration_s,
        width_fraction=0.04 if stress else 0.02,
    )

    wheel_slip = []
    if stress:
        slip_start, slip_end = _interval(rng, duration_s, width_fraction=0.10)
        wheel_slip.append(
            {
                "wheel": str(rng.choice(wheel_names)),
                "start_s": slip_start,
                "end_s": slip_end,
                "measurement_scale": _rounded(rng.uniform(0.2, 0.55)),
            }
        )
    faults = {
        "wheel_slip": wheel_slip,
        "sensor_dropouts": dropouts,
        "depth_holes": [
            {
                "rows": [row_start, row_start + hole_height],
                "cols": [col_start, col_start + hole_width],
                "start_s": hole_start,
                "end_s": hole_end,
            }
        ],
        "depth_spikes": [
            {
                "row": int(rng.integers(0, rows)),
                "col": int(rng.integers(0, cols)),
                "offset_m": _rounded(rng.uniform(2.0, 3.5) if stress else 2.0),
                "start_s": spike_start,
                "end_s": spike_end,
            }
        ],
    }
    if stress:
        degradation_start, degradation_end = _interval(
            rng,
            duration_s,
            width_fraction=0.15,
        )
        faults["depth_degradation"] = [
            {
                "start_s": degradation_start,
                "end_s": degradation_end,
                "dropout_ratio_start": 0.0,
                "dropout_ratio_end": 0.6,
                "noise_std_m": 0.02,
            }
        ]
    return faults


def generate_scenario(
    parameters: GenerationParameters,
    *,
    seed: int,
    seed_class: str,
) -> dict[str, Any]:
    """Generate and validate one complete scenario document.

    ``hidden_evaluation`` output is for generation and validation only.  Do
    not inspect its contents or tune against it.
    """
    if not isinstance(parameters, GenerationParameters):
        raise TypeError("parameters must be GenerationParameters")
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**128:
        raise ValueError("seed must be an integer within [0, 2^128)")
    if seed_class not in SEED_CLASSES:
        raise ValueError("seed_class must be dev, regression, hidden_evaluation, or stress")

    rng = np.random.Generator(np.random.PCG64(seed))
    family = str(rng.choice(parameters.terrain_families))
    motion_profile = str(rng.choice(parameters.motion_profiles))
    length_m = _draw(rng, parameters.track_length_range_m)
    spacing_m = _draw(rng, parameters.station_spacing_range_m)
    station_count = max(3, int(math.ceil(length_m / spacing_m)) + 1)
    stations = np.linspace(0.0, length_m, station_count)
    speed_m_s = _draw(rng, parameters.linear_speed_range_m_s)
    acceleration_m_s2 = _draw(rng, parameters.linear_acceleration_range_m_s2)

    if parameters.curvature_mode == "clothoid":
        start_curvature, requested_end_curvature = (
            float(value) for value in parameters.curvature_range_per_m
        )
        maximum_change = MAX_CLOTHOID_RATE_PER_M2 * length_m
        curvature_change = float(
            np.clip(
                requested_end_curvature - start_curvature,
                -maximum_change,
                maximum_change,
            )
        )
        end_curvature = start_curvature + curvature_change
        raw_curvature_profile = np.linspace(
            start_curvature,
            end_curvature,
            station_count,
        )
        curvature_profile = [
            _rounded(value) for value in raw_curvature_profile
        ]
        motion_curvature = 0.5 * (start_curvature + end_curvature)
    else:
        if motion_profile == "trapezoidal_speed":
            curvature = 0.0
        else:
            curvature = _draw(rng, parameters.curvature_range_per_m)
        raw_curvature_profile = np.full(station_count, curvature, dtype=float)
        curvature_profile = [_rounded(curvature)] * station_count
        motion_curvature = curvature
    heading = 0.0
    x_m = 0.0
    y_m = 0.0
    elevation_m = _draw(rng, parameters.track_height_range_m)
    if family == "undulating":
        elevation_profile = [
            _rounded(
                elevation_m
                + float(parameters.undulation_amplitude_m)
                * math.sin(
                    2.0
                    * math.pi
                    * float(station)
                    / float(parameters.undulation_wavelength_m)
                )
            )
            for station in stations
        ]
    else:
        elevation_profile = [_rounded(elevation_m)] * station_count
    centerline = [[0.0, 0.0, elevation_profile[0]]]
    for index, (left_station, right_station) in enumerate(
        zip(stations, stations[1:])
    ):
        ds = float(right_station - left_station)
        if parameters.curvature_mode == "constant":
            segment_curvature = curvature
        else:
            segment_curvature = 0.5 * (
                float(raw_curvature_profile[index])
                + float(raw_curvature_profile[index + 1])
            )
        midpoint_heading = heading + 0.5 * segment_curvature * ds
        x_m += ds * math.cos(midpoint_heading)
        y_m += ds * math.sin(midpoint_heading)
        heading += segment_curvature * ds
        centerline.append(
            [_rounded(x_m), _rounded(y_m), elevation_profile[index + 1]]
        )

    bank_magnitude = _draw(rng, parameters.bank_magnitude_range_rad)
    bank_sign = -1.0 if int(rng.integers(0, 2)) == 0 else 1.0
    if family in {"flat", "undulating"}:
        bank = [0.0] * station_count
    elif family == "bank":
        bank = [_rounded(bank_sign * bank_magnitude)] * station_count
    else:
        bank = [
            _rounded(bank_sign * bank_magnitude * math.sin(math.pi * fraction))
            for fraction in np.linspace(0.0, 1.0, station_count)
        ]
        bank[0] = 0.0
        bank[-1] = 0.0

    width = _smooth_profile(rng, station_count, parameters.track_width_range_m)
    if parameters.pinch is not None:
        centre_m = float(parameters.pinch.center_ratio) * length_m
        half_length_m = float(parameters.pinch.length_m) / 2.0
        width = [
            _rounded(parameters.pinch.width_m)
            if abs(float(station) - centre_m) <= half_length_m + 1e-12
            else value
            for station, value in zip(stations, width)
        ]
    friction = _smooth_profile(rng, station_count, parameters.friction_range)
    if parameters.friction_patch is not None:
        centre_m = float(parameters.friction_patch.center_ratio) * length_m
        half_length_m = float(parameters.friction_patch.length_m) / 2.0
        friction = [
            _rounded(parameters.friction_patch.mu)
            if abs(float(station) - centre_m) <= half_length_m + 1e-12
            else value
            for station, value in zip(stations, friction)
        ]
    duration_s = _duration_for_motion(
        motion_profile,
        length_m,
        speed_m_s,
        acceleration_m_s2,
        parameters.clock_dt_s,
    )
    wheel_names = tuple(wheel.name for wheel in default_geometry().wheels)
    faults = _faults(
        rng,
        seed_class=seed_class,
        duration_s=duration_s,
        depth_shape=parameters.depth_shape_px,
        wheel_names=wheel_names,
    )

    if motion_profile == "constant_speed":
        motion: dict[str, Any] = {
            "profile": motion_profile,
            "linear_speed_m_s": _rounded(speed_m_s),
            "yaw_rate_rad_s": _rounded(motion_curvature * speed_m_s),
        }
    else:
        motion = {
            "profile": motion_profile,
            "max_linear_speed_m_s": _rounded(speed_m_s),
            "linear_acceleration_m_s2": _rounded(acceleration_m_s2),
            "yaw_rate_rad_s": 0.0,
        }

    geometry = default_geometry()
    geometric_margin = max(abs(wheel.y) for wheel in geometry.wheels) + 0.05
    expected_clearance = max(0.0, min(width) / 2.0 - geometric_margin)
    height_values = [point[2] for point in centerline]
    height_px, width_px = parameters.depth_shape_px
    depth_fx_px = width_px / (2.0 * math.tan(L515_DEPTH_HORIZONTAL_FOV_RAD / 2.0))
    depth_fy_px = height_px / (2.0 * math.tan(L515_DEPTH_VERTICAL_FOV_RAD / 2.0))
    document: dict[str, Any] = {
        "schema_version": 1,
        "scenario_id": f"procedural_{seed_class}_{seed}_{family}",
        "description": (
            f"Deterministic {family} elevated track generated with PCG64 seed {seed}."
        ),
        "units": dict(REQUIRED_UNITS),
        "frames": {
            "world": "map",
            "body": "base_link",
            "depth": "l515_depth_optical_frame",
            "imu": "l515_imu_link",
        },
        "clock": {
            "start_s": 1.0,
            "dt_s": _rounded(parameters.clock_dt_s),
            "duration_s": duration_s,
        },
        "prng": {
            "algorithm": PRNG_ALGORITHM,
            "seed": seed,
            "seed_class": seed_class,
        },
        "track": {
            "centerline_m": centerline,
            "width_m": width,
            "height_m": height_values,
            "bank_rad": bank,
            "curvature_per_m": curvature_profile,
            "friction_coefficient": friction,
            "drop_boundaries": [
                {"left": True, "right": True} for _ in range(station_count)
            ],
        },
        "motion": motion,
        "sensors": {
            "wheel_states": {
                "sample_every_n_steps": 1,
                "noise_std_turns_per_s": 0.001,
                "wheel_names": list(wheel_names),
            },
            "imu": {
                "sample_every_n_steps": 1,
                "gyro_bias_rad_s": [0.0, 0.0, 0.002],
                "gyro_noise_std_rad_s": 0.001,
                "accel_bias_m_s2": [0.0, 0.0, 0.01],
                "accel_noise_std_m_s2": 0.005,
                "gravity_m_s2": 9.81,
            },
            "depth": {
                "sample_every_n_steps": parameters.depth_sample_every_n_steps,
                "shape_px": [height_px, width_px],
                "depth_scale_m": 0.001,
                "base_depth_m": 1.5,
                "noise_std_m": 0.002,
                "bank_depth_span_m": 0.08,
                "intrinsics_px": {
                    "fx": _rounded(depth_fx_px),
                    "fy": _rounded(depth_fy_px),
                    "cx": _rounded((width_px - 1) / 2.0),
                    "cy": _rounded((height_px - 1) / 2.0),
                },
            },
        },
        "faults": faults,
        "expected_metrics": {
            "completion": bool(parameters.expected_completion),
            "min_clearance_m": _rounded(expected_clearance),
            "edge_overrun_count": 0,
            "false_hold_count": 0,
            "fail_open_count": 0,
            "max_recovery_time_s": 0.5 if seed_class == "stress" else 0.25,
            "max_estimator_runtime_ms": 5.0,
        },
    }
    parse_scenario(document)
    return document


def canonical_json_bytes(document: dict[str, Any]) -> bytes:
    """Return the dict-order-independent canonical bytes used for drift pins."""
    parse_scenario(document)
    return json.dumps(
        document,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def canonical_json_sha256(document: dict[str, Any]) -> str:
    """Hash one complete generated document in canonical JSON form."""
    return hashlib.sha256(canonical_json_bytes(document)).hexdigest()


def scenario_yaml(document: dict[str, Any]) -> str:
    """Serialize one validated scenario as deterministic UTF-8 YAML text."""
    parse_scenario(document)
    return yaml.safe_dump(
        document,
        allow_unicode=True,
        sort_keys=True,
    )


def dump_scenario_yaml(document: dict[str, Any], path: str | Path) -> None:
    """Write deterministic YAML after applying the part-one validator."""
    Path(path).write_text(scenario_yaml(document), encoding="utf-8")


__all__ = (
    "FrictionPatchSpec",
    "GenerationParameters",
    "PinchSpec",
    "canonical_json_bytes",
    "canonical_json_sha256",
    "dump_scenario_yaml",
    "generate_scenario",
    "scenario_yaml",
)
