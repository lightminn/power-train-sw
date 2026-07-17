"""Shared deterministic scenario documents for closed-loop family checks."""
from __future__ import annotations

import math

from .procedural import (
    FrictionPatchSpec,
    GenerationParameters,
    PinchSpec,
    generate_scenario,
)


DEV_SEED = 0

# CAD URDF wheel centres in chassis.kinematics.default_geometry() have their
# widest |y| at 0.4395 m; model_builder gives each wheel 0.035 m half-width.
# The simulated physical footprint is therefore 2 * (0.4395 + 0.035) = 0.949 m.
ROBOT_FOOTPRINT_WIDTH_M = 0.949


def _terrain_document(
    family: str,
    *,
    seed: int,
    seed_class: str,
) -> dict:
    return generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=(family,),
            motion_profiles=("constant_speed",),
            # Closed loop should stop fail-closed before the terminal drop.
            expected_completion=False,
        ),
        seed=seed,
        seed_class=seed_class,
    )


def flat_document(*, seed: int = DEV_SEED, seed_class: str = "dev") -> dict:
    return _terrain_document("flat", seed=seed, seed_class=seed_class)


def bank_document(*, seed: int = DEV_SEED, seed_class: str = "dev") -> dict:
    return _terrain_document("bank", seed=seed, seed_class=seed_class)


def pinch_document(
    *,
    width_m: float,
    seed: int = DEV_SEED,
    seed_class: str = "dev",
) -> dict:
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.3, 1.3),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            station_spacing_range_m=(0.20, 0.20),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            pinch=PinchSpec(center_ratio=0.45, length_m=0.5, width_m=width_m),
            expected_completion=False,
        ),
        seed=seed,
        seed_class=seed_class,
    )
    document["clock"]["duration_s"] = 12.0
    document["faults"] = {name: [] for name in document["faults"]}
    return document


def friction_document(
    *,
    seed: int = DEV_SEED,
    seed_class: str = "dev",
) -> dict:
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            friction_range=(0.8, 0.8),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            friction_patch=FrictionPatchSpec(
                center_ratio=0.5,
                length_m=0.8,
                mu=0.3,
            ),
            expected_completion=False,
        ),
        seed=seed,
        seed_class=seed_class,
    )
    document["faults"] = {name: [] for name in document["faults"]}
    return document


def depth_degradation_document(
    *,
    seed: int = 2,
    seed_class: str = "dev",
) -> dict:
    document = flat_document(seed=seed, seed_class=seed_class)
    document["faults"] = {name: [] for name in document["faults"]}
    document["faults"]["depth_degradation"] = [
        {
            "start_s": 0.8,
            "end_s": 2.4,
            "dropout_ratio_start": 0.0,
            "dropout_ratio_end": 0.6,
            "noise_std_m": 0.02,
        }
    ]
    return document


def clothoid_document(
    *,
    seed: int = DEV_SEED,
    seed_class: str = "dev",
) -> dict:
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(-0.08, 0.08),
            station_spacing_range_m=(0.35, 0.35),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            curvature_mode="clothoid",
            expected_completion=False,
        ),
        seed=seed,
        seed_class=seed_class,
    )
    document["clock"]["duration_s"] = 12.0
    document["faults"] = {name: [] for name in document["faults"]}
    return document


def undulating_document(
    *,
    seed: int = DEV_SEED,
    seed_class: str = "dev",
) -> dict:
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            station_spacing_range_m=(0.40, 0.40),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("undulating",),
            motion_profiles=("constant_speed",),
            expected_completion=False,
        ),
        seed=seed,
        seed_class=seed_class,
    )
    document["clock"]["duration_s"] = 12.0
    document["faults"] = {name: [] for name in document["faults"]}
    return document


def follow_document(
    *,
    curve: bool,
    duration_s: float,
    seed: int,
    seed_class: str = "dev",
) -> dict:
    track_length_m = 40.0 if not curve else 16.0
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(track_length_m, track_length_m),
            track_width_range_m=(1.8, 1.8),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            station_spacing_range_m=(0.5, 0.5),
            linear_speed_range_m_s=(0.5, 0.5),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            expected_completion=False,
        ),
        seed=seed,
        seed_class=seed_class,
    )
    document["clock"]["duration_s"] = duration_s
    document["faults"] = {name: [] for name in document["faults"]}
    if curve:
        curvature_per_m = 0.025
        points = document["track"]["centerline_m"]
        for index, point in enumerate(points):
            station_m = min(index * 0.5, track_length_m)
            point[0] = math.sin(curvature_per_m * station_m) / curvature_per_m
            point[1] = (
                1.0 - math.cos(curvature_per_m * station_m)
            ) / curvature_per_m
        document["track"]["curvature_per_m"] = [
            curvature_per_m for _ in points
        ]
    return document


__all__ = (
    "DEV_SEED",
    "ROBOT_FOOTPRINT_WIDTH_M",
    "bank_document",
    "clothoid_document",
    "depth_degradation_document",
    "flat_document",
    "follow_document",
    "friction_document",
    "pinch_document",
    "undulating_document",
)
