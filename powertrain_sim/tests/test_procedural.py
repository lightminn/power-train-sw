from __future__ import annotations

import json
import math

import pytest
import yaml

from powertrain_sim.procedural import (
    FrictionPatchSpec,
    GenerationParameters,
    PinchSpec,
    canonical_json_sha256,
    dump_scenario_yaml,
    generate_scenario,
)
from powertrain_sim.scenario import parse_scenario


def test_same_parameters_and_seed_generate_the_same_complete_contract():
    parameters = GenerationParameters()

    first = generate_scenario(parameters, seed=20260716, seed_class="dev")
    second = generate_scenario(parameters, seed=20260716, seed_class="dev")

    assert first == second
    scenario = parse_scenario(first)
    count = len(scenario.track.centerline_m)
    assert count >= 3
    assert all(
        point[2] == height
        for point, height in zip(
            scenario.track.centerline_m,
            scenario.track.height_m,
        )
    )
    assert all(
        len(values) == count
        for values in (
            scenario.track.width_m,
            scenario.track.height_m,
            scenario.track.bank_rad,
            scenario.track.curvature_per_m,
            scenario.track.friction_coefficient,
            scenario.track.drop_boundaries,
        )
    )
    assert all(boundary.left and boundary.right for boundary in scenario.track.drop_boundaries)
    assert scenario.prng.algorithm == "PCG64"
    assert scenario.prng.seed_class == "dev"


def test_yaml_helper_round_trips_through_the_part_one_validator(tmp_path):
    document = generate_scenario(
        GenerationParameters(terrain_families=("bank_transition",)),
        seed=73,
        seed_class="regression",
    )

    output = tmp_path / "generated.yaml"
    dump_scenario_yaml(document, output)

    decoded = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert decoded == document
    assert parse_scenario(decoded).scenario_id == document["scenario_id"]


def test_stress_seed_class_increases_fault_schedule_intensity():
    parameters = GenerationParameters(terrain_families=("bank",))

    dev = generate_scenario(parameters, seed=91, seed_class="dev")
    stress = generate_scenario(parameters, seed=91, seed_class="stress")

    dev_faults = sum(len(entries) for entries in dev["faults"].values())
    stress_faults = sum(len(entries) for entries in stress["faults"].values())
    assert stress_faults > dev_faults
    assert len(stress["faults"]["sensor_dropouts"]) >= 2
    assert "depth_degradation" not in dev["faults"]
    assert len(stress["faults"]["depth_degradation"]) == 1
    degradation = stress["faults"]["depth_degradation"][0]
    assert degradation["dropout_ratio_start"] == 0.0
    assert degradation["dropout_ratio_end"] == 0.6
    assert degradation["noise_std_m"] > 0.0
    assert stress["prng"]["seed_class"] == "stress"


def test_generated_families_cover_flat_bank_and_bank_transition():
    for family in ("flat", "bank", "bank_transition"):
        document = generate_scenario(
            GenerationParameters(terrain_families=(family,)),
            seed=11,
            seed_class="dev",
        )
        bank = document["track"]["bank_rad"]
        if family == "flat":
            assert set(bank) == {0.0}
        elif family == "bank":
            assert max(abs(value) for value in bank) > 0.0
            assert bank[0] == bank[-1]
        else:
            assert bank[0] == 0.0
            assert bank[-1] == 0.0
            assert max(abs(value) for value in bank) > 0.0


def test_representative_dev_seed_canonical_json_hash_is_pinned():
    document = generate_scenario(
        GenerationParameters(),
        seed=20260716,
        seed_class="dev",
    )

    # Reviewed golden document: this catches silent PCG64 draw-order or schema drift.
    assert json.loads(json.dumps(document, sort_keys=True)) == document
    assert canonical_json_sha256(document) == (
        "9f97356d6b5cee3b2ffac8b772740b3c155d1d7a23d4bda221a8e53278fc00b3"
    )


def test_generated_depth_roi_preserves_l515_wide_field_of_view():
    document = generate_scenario(
        GenerationParameters(),
        seed=5,
        seed_class="dev",
    )

    depth = document["sensors"]["depth"]
    height, width = depth["shape_px"]
    intrinsics = depth["intrinsics_px"]
    horizontal_fov_deg = math.degrees(
        2.0 * math.atan(width / (2.0 * intrinsics["fx"]))
    )
    vertical_fov_deg = math.degrees(
        2.0 * math.atan(height / (2.0 * intrinsics["fy"]))
    )

    assert depth["shape_px"] == [60, 80]
    assert horizontal_fov_deg == pytest.approx(70.0, abs=1.0)
    assert vertical_fov_deg == pytest.approx(55.0, abs=1.0)


def test_pinch_replaces_only_widths_inside_its_station_interval():
    common = dict(
        track_length_range_m=(4.0, 4.0),
        track_width_range_m=(1.3, 1.3),
        curvature_range_per_m=(0.0, 0.0),
        station_spacing_range_m=(0.25, 0.25),
        terrain_families=("flat",),
        motion_profiles=("constant_speed",),
    )
    baseline = generate_scenario(
        GenerationParameters(**common),
        seed=101,
        seed_class="dev",
    )
    pinched = generate_scenario(
        GenerationParameters(
            **common,
            pinch=PinchSpec(center_ratio=0.5, length_m=1.0, width_m=1.05),
        ),
        seed=101,
        seed_class="dev",
    )

    stations = [
        math.dist(pinched["track"]["centerline_m"][0], point)
        for point in pinched["track"]["centerline_m"]
    ]
    for station, original, actual in zip(
        stations,
        baseline["track"]["width_m"],
        pinched["track"]["width_m"],
    ):
        if abs(station - 2.0) <= 0.5 + 1e-12:
            assert actual == 1.05
        else:
            assert actual == original


def test_friction_patch_replaces_only_mu_inside_its_station_interval():
    common = dict(
        track_length_range_m=(4.0, 4.0),
        friction_range=(0.8, 0.8),
        curvature_range_per_m=(0.0, 0.0),
        station_spacing_range_m=(0.25, 0.25),
        terrain_families=("flat",),
        motion_profiles=("constant_speed",),
    )
    baseline = generate_scenario(
        GenerationParameters(**common),
        seed=105,
        seed_class="dev",
    )
    patched = generate_scenario(
        GenerationParameters(
            **common,
            friction_patch=FrictionPatchSpec(
                center_ratio=0.5,
                length_m=1.0,
                mu=0.3,
            ),
        ),
        seed=105,
        seed_class="dev",
    )

    stations = [
        math.dist(patched["track"]["centerline_m"][0], point)
        for point in patched["track"]["centerline_m"]
    ]
    expected_friction = []
    for station, original in zip(
        stations,
        baseline["track"]["friction_coefficient"],
    ):
        expected_friction.append(
            0.3 if abs(station - 2.0) <= 0.5 + 1e-12 else original
        )
    assert patched["track"]["friction_coefficient"] == expected_friction

    patched_without_fixture = json.loads(json.dumps(patched))
    patched_without_fixture["track"]["friction_coefficient"] = baseline["track"][
        "friction_coefficient"
    ]
    assert patched_without_fixture == baseline


def test_clothoid_curvature_is_linear_across_stations():
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(4.0, 4.0),
            curvature_range_per_m=(-0.08, 0.08),
            station_spacing_range_m=(0.25, 0.25),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            curvature_mode="clothoid",
        ),
        seed=102,
        seed_class="dev",
    )

    curvature = document["track"]["curvature_per_m"]
    assert curvature[0] == -0.08
    assert curvature[-1] == 0.08
    assert curvature == pytest.approx(
        [
            -0.08 + 0.16 * index / (len(curvature) - 1)
            for index in range(len(curvature))
        ]
    )
    assert len(set(curvature)) == len(curvature)


def test_clothoid_curvature_rate_is_clamped_to_point_zero_eight_per_square_metre():
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.0, 2.0),
            curvature_range_per_m=(-0.5, 0.5),
            station_spacing_range_m=(0.2, 0.2),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            curvature_mode="clothoid",
        ),
        seed=103,
        seed_class="dev",
    )

    points = document["track"]["centerline_m"]
    curvature = document["track"]["curvature_per_m"]
    rates = [
        abs(right_curvature - left_curvature) / math.dist(left_point, right_point)
        for left_curvature, right_curvature, left_point, right_point in zip(
            curvature,
            curvature[1:],
            points,
            points[1:],
        )
    ]
    assert max(rates) <= 0.08 + 1e-9
    assert curvature[-1] < 0.5


def test_undulating_family_varies_only_elevation_with_declared_waveform():
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(4.0, 4.0),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            station_spacing_range_m=(0.25, 0.25),
            terrain_families=("undulating",),
            motion_profiles=("constant_speed",),
        ),
        seed=104,
        seed_class="dev",
    )

    heights = document["track"]["height_m"]
    assert min(heights) == pytest.approx(0.45)
    assert max(heights) == pytest.approx(0.55)
    assert heights[0] == pytest.approx(0.5)
    assert heights[2] == pytest.approx(0.55)
    assert heights[6] == pytest.approx(0.45)
    assert document["track"]["bank_rad"] == [0.0] * len(heights)
    assert [
        point[2] for point in document["track"]["centerline_m"]
    ] == heights
