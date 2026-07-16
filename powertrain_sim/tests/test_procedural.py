from __future__ import annotations

import json
import math

import pytest
import yaml

from powertrain_sim.procedural import (
    GenerationParameters,
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
