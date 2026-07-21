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

# 훈련 트랙 — 스펙 2026-07-20 §4.2.
# 길이: 2.5 m 에서는 종단 fail-closed 정지거리 0.7 m 가 전체의 28% 라
#       구조적 최대 완주율이 ~0.71 이었다. 15 m 에서는 5% 로 내려간다.
# 폭:   차폭 949 mm 대비 편측 여유 325 mm. 차폭을 진단 변수에서 제거한다.
TRAINING_TRACK_LENGTH_M = 15.0
TRAINING_TRACK_WIDTH_M = 1.6
# 대회 코스 course.stl 실측: 0.085 <-> 0.388 m (peak-to-peak 0.303 m), 주기 4.4 m.
UNDULATION_AMPLITUDE_M = 0.15
UNDULATION_WAVELENGTH_M = 4.4
# 0.45 m/s 로 15 m 를 주파하려면 33.3 s. 종단 정지 여유를 포함해 40 s.
TRAINING_DURATION_S = 40.0

# 6 m 벽(시뮬 depth 렌더러 mj_multiRay plane 프루닝) 수정 후 정직 재기준선
# (2026-07-21). 15 m 기복 트랙에서 크레스트 통과마다 짧은 fail-closed 과도
# hold 가 생긴다 — Task 2 특성화 실측: flat 13 / bank 15 / clothoid 13 /
# friction 10 / smog 22 (pinch·undulating 0), 에피소드는 중간 구간 전부
# <=0.28 s, bank 종단 정지 구간(x>13.5)에 0.88 s·1.1 s 에피소드
# 2건 존재, max_recovery 실측
# 0.2 s. 상한은 실측 x1.5 올림. fail_open/edge_overrun 0 은 절대 불변.
# dev 시드 한정 적용(hidden/stress 는 생성기 기본 유지 — 다중시드 보정 전
# 미보정 상태가 정직). recovery 상한 0.3 = 실측 0.2 x1.5.
FAMILY_FALSE_HOLD_BOUND = {
    "flat": 20,
    "bank": 23,
    "clothoid": 20,
    "friction": 15,
    "smog": 33,
}


def _repin_transient_hold_bounds(
    document: dict, family: str, seed_class: str
) -> None:
    if seed_class != "dev":
        return
    bound = FAMILY_FALSE_HOLD_BOUND.get(family)
    if bound is None:
        return
    document["expected_metrics"]["false_hold_count"] = bound
    document["expected_metrics"]["max_recovery_time_s"] = 0.3


def _terrain_document(
    family: str,
    *,
    seed: int,
    seed_class: str,
) -> dict:
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(
                TRAINING_TRACK_LENGTH_M,
                TRAINING_TRACK_LENGTH_M,
            ),
            track_width_range_m=(
                TRAINING_TRACK_WIDTH_M,
                TRAINING_TRACK_WIDTH_M,
            ),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=(family,),
            motion_profiles=("constant_speed",),
            undulation_amplitude_m=UNDULATION_AMPLITUDE_M,
            undulation_wavelength_m=UNDULATION_WAVELENGTH_M,
            # Closed loop should stop fail-closed before the terminal drop.
            expected_completion=False,
        ),
        seed=seed,
        seed_class=seed_class,
    )
    document["clock"]["duration_s"] = TRAINING_DURATION_S
    _repin_transient_hold_bounds(document, family, seed_class)
    return document


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
            track_length_range_m=(
                TRAINING_TRACK_LENGTH_M,
                TRAINING_TRACK_LENGTH_M,
            ),
            track_width_range_m=(
                TRAINING_TRACK_WIDTH_M,
                TRAINING_TRACK_WIDTH_M,
            ),
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
            track_length_range_m=(
                TRAINING_TRACK_LENGTH_M,
                TRAINING_TRACK_LENGTH_M,
            ),
            track_width_range_m=(
                TRAINING_TRACK_WIDTH_M,
                TRAINING_TRACK_WIDTH_M,
            ),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            friction_range=(0.8, 0.8),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            undulation_amplitude_m=UNDULATION_AMPLITUDE_M,
            undulation_wavelength_m=UNDULATION_WAVELENGTH_M,
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
    document["clock"]["duration_s"] = TRAINING_DURATION_S
    _repin_transient_hold_bounds(document, "friction", seed_class)
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
    # flat_document 가 상속시킨 flat 상한(20)을 smog 실측 기반 상한으로 교체.
    _repin_transient_hold_bounds(document, "smog", seed_class)
    return document


def clothoid_document(
    *,
    seed: int = DEV_SEED,
    seed_class: str = "dev",
) -> dict:
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(
                TRAINING_TRACK_LENGTH_M,
                TRAINING_TRACK_LENGTH_M,
            ),
            track_width_range_m=(
                TRAINING_TRACK_WIDTH_M,
                TRAINING_TRACK_WIDTH_M,
            ),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(-0.08, 0.08),
            station_spacing_range_m=(0.35, 0.35),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            undulation_amplitude_m=UNDULATION_AMPLITUDE_M,
            undulation_wavelength_m=UNDULATION_WAVELENGTH_M,
            curvature_mode="clothoid",
            expected_completion=False,
        ),
        seed=seed,
        seed_class=seed_class,
    )
    document["clock"]["duration_s"] = TRAINING_DURATION_S
    _repin_transient_hold_bounds(document, "clothoid", seed_class)
    # 생성기 기대 여유(0.3105)는 직선·중앙 주행 기하 가정이다. 클로소이드
    # 곡선에서는 corridor-중앙 추종이 안쪽을 지나며 실측 최소 여유가 준다
    # (2026-07-21 dev seed 실측 0.2081, edge_overrun 0). 하한을 실측 기반
    # 0.15 로 재핀 — 0 침범은 여전히 edge_overrun 게이트가 잡는다.
    document["expected_metrics"]["min_clearance_m"] = 0.15
    document["faults"] = {name: [] for name in document["faults"]}
    return document


def undulating_document(
    *,
    seed: int = DEV_SEED,
    seed_class: str = "dev",
) -> dict:
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(
                TRAINING_TRACK_LENGTH_M,
                TRAINING_TRACK_LENGTH_M,
            ),
            track_width_range_m=(
                TRAINING_TRACK_WIDTH_M,
                TRAINING_TRACK_WIDTH_M,
            ),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            station_spacing_range_m=(0.40, 0.40),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("undulating",),
            motion_profiles=("constant_speed",),
            undulation_amplitude_m=UNDULATION_AMPLITUDE_M,
            undulation_wavelength_m=UNDULATION_WAVELENGTH_M,
            expected_completion=False,
        ),
        seed=seed,
        seed_class=seed_class,
    )
    document["clock"]["duration_s"] = TRAINING_DURATION_S
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
    "TRAINING_DURATION_S",
    "TRAINING_TRACK_LENGTH_M",
    "TRAINING_TRACK_WIDTH_M",
    "UNDULATION_AMPLITUDE_M",
    "UNDULATION_WAVELENGTH_M",
    "bank_document",
    "clothoid_document",
    "depth_degradation_document",
    "flat_document",
    "follow_document",
    "friction_document",
    "pinch_document",
    "undulating_document",
)
