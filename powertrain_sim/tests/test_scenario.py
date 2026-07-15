from __future__ import annotations

from copy import deepcopy
from importlib import import_module
import math
from pathlib import Path

import pytest
import yaml


SCENARIO_DIR = Path(__file__).resolve().parents[1] / "scenarios"


def _scenario_module():
    return import_module("powertrain_sim.scenario")


def _valid_document() -> dict:
    return {
        "schema_version": 1,
        "scenario_id": "unit_test",
        "description": "Minimal complete simulator-neutral scenario",
        "units": {
            "distance": "m",
            "angle": "rad",
            "time": "s",
            "linear_velocity": "m/s",
            "angular_velocity": "rad/s",
            "acceleration": "m/s^2",
            "curvature": "1/m",
            "friction": "1",
            "depth": "mm",
        },
        "frames": {
            "world": "map",
            "body": "base_link",
            "depth": "l515_depth_optical_frame",
            "imu": "l515_imu_link",
        },
        "clock": {"start_s": 1.0, "dt_s": 0.02, "duration_s": 2.0},
        "prng": {
            "algorithm": "PCG64",
            "seed": 20260716,
            "seed_class": "regression",
        },
        "track": {
            "centerline_m": [[0.0, 0.0, 0.4], [1.0, 0.0, 0.4]],
            "width_m": [1.2, 1.2],
            "height_m": [0.4, 0.4],
            "bank_rad": [0.0, 0.0],
            "curvature_per_m": [0.0, 0.0],
            "friction_coefficient": [0.8, 0.8],
            "drop_boundaries": [
                {"left": True, "right": True},
                {"left": True, "right": True},
            ],
        },
        "motion": {
            "profile": "constant_speed",
            "linear_speed_m_s": 0.5,
            "yaw_rate_rad_s": 0.0,
        },
        "sensors": {
            "wheel_states": {
                "sample_every_n_steps": 1,
                "noise_std_turns_per_s": 0.0,
                "wheel_names": [
                    "front_left",
                    "front_right",
                    "mid_left",
                    "mid_right",
                    "rear_left",
                    "rear_right",
                ],
            },
            "imu": {
                "sample_every_n_steps": 1,
                "gyro_bias_rad_s": [0.0, 0.0, 0.0],
                "gyro_noise_std_rad_s": 0.0,
                "accel_bias_m_s2": [0.0, 0.0, 0.0],
                "accel_noise_std_m_s2": 0.0,
                "gravity_m_s2": 9.81,
            },
            "depth": {
                "sample_every_n_steps": 5,
                "shape_px": [40, 60],
                "depth_scale_m": 0.001,
                "base_depth_m": 1.5,
                "noise_std_m": 0.0,
                "bank_depth_span_m": 0.04,
                "intrinsics_px": {
                    "fx": 420.0,
                    "fy": 420.0,
                    "cx": 29.5,
                    "cy": 19.5,
                },
            },
        },
        "faults": {
            "wheel_slip": [],
            "sensor_dropouts": [],
            "depth_holes": [],
            "depth_spikes": [],
        },
        "expected_metrics": {
            "completion": True,
            "min_clearance_m": 0.1,
            "edge_overrun_count": 0,
            "false_hold_count": 0,
            "fail_open_count": 0,
            "max_recovery_time_s": 0.5,
            "max_estimator_runtime_ms": 5.0,
        },
    }


def test_load_normalizes_the_complete_contract_without_dict_order_dependence(tmp_path):
    module = _scenario_module()
    document = _valid_document()
    first_path = tmp_path / "first.yaml"
    second_path = tmp_path / "second.yaml"
    first_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    second_path.write_text(yaml.safe_dump(document, sort_keys=True), encoding="utf-8")

    first = module.load_scenario(first_path)
    second = module.load_scenario(second_path)

    assert first == second
    assert first.schema_version == 1
    assert first.scenario_id == "unit_test"
    assert first.prng.algorithm == "PCG64"
    assert first.prng.seed == 20260716
    assert first.prng.seed_class == "regression"
    assert first.clock.sample_count == 101
    assert first.track.centerline_m[1] == (1.0, 0.0, 0.4)
    assert first.track.drop_boundaries[0].left is True
    assert first.units["depth"] == "mm"


@pytest.mark.parametrize(
    "missing_key",
    (
        "schema_version",
        "units",
        "frames",
        "clock",
        "prng",
        "track",
        "motion",
        "sensors",
        "faults",
        "expected_metrics",
    ),
)
def test_missing_required_top_level_key_is_rejected(missing_key):
    module = _scenario_module()
    document = _valid_document()
    del document[missing_key]

    with pytest.raises(module.ScenarioValidationError, match=missing_key):
        module.parse_scenario(document)


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (lambda doc: doc["units"].__setitem__("distance", "meters"), "units.distance"),
        (lambda doc: doc["prng"].__setitem__("algorithm", "MT19937"), "PCG64"),
        (lambda doc: doc["prng"].__setitem__("seed_class", "tuning"), "seed_class"),
        (lambda doc: doc.__setitem__("schema_version", 2), "schema_version"),
        (lambda doc: doc["clock"].__setitem__("dt_s", math.nan), "finite"),
        (lambda doc: doc["track"].__setitem__("width_m", [1.2]), "width_m"),
        (
            lambda doc: doc["track"]["centerline_m"][1].__setitem__(2, 0.5),
            "height_m",
        ),
    ),
)
def test_ambiguous_units_unknown_prng_nonfinite_and_bad_track_are_rejected(
    mutation, message
):
    module = _scenario_module()
    document = deepcopy(_valid_document())
    mutation(document)

    with pytest.raises(module.ScenarioValidationError, match=message):
        module.parse_scenario(document)


@pytest.mark.parametrize(
    "mutation",
    (
        lambda doc: doc.__setitem__("unknown_top_level", 1),
        lambda doc: doc["sensors"]["depth"].__setitem__("depth_sacle_m", 0.001),
    ),
)
def test_unknown_schema_fields_are_rejected_instead_of_silently_discarded(mutation):
    module = _scenario_module()
    document = deepcopy(_valid_document())
    mutation(document)

    with pytest.raises(module.ScenarioValidationError, match="unknown"):
        module.parse_scenario(document)


@pytest.mark.parametrize(
    ("motion", "message"),
    (
        (
            {"profile": "pivot", "target_yaw_rad": math.pi / 2, "yaw_rate_rad_s": 0.0},
            "yaw_rate_rad_s",
        ),
        (
            {"profile": "pivot", "target_yaw_rad": -math.pi / 2, "yaw_rate_rad_s": 1.0},
            "sign",
        ),
        (
            {"profile": "pivot", "target_yaw_rad": math.pi, "yaw_rate_rad_s": 1.0},
            "duration",
        ),
    ),
)
def test_pivot_target_rate_and_duration_must_be_consistent(motion, message):
    module = _scenario_module()
    document = deepcopy(_valid_document())
    document["motion"] = motion

    with pytest.raises(module.ScenarioValidationError, match=message):
        module.parse_scenario(document)


@pytest.mark.parametrize("fault_kind", ("wheel_name", "hole_bounds", "spike_bounds"))
def test_fault_identifiers_and_depth_coordinates_fail_during_load(fault_kind):
    module = _scenario_module()
    document = deepcopy(_valid_document())
    if fault_kind == "wheel_name":
        document["faults"]["wheel_slip"] = [
            {
                "wheel": "rear_rigth",
                "start_s": 0.5,
                "end_s": 1.0,
                "measurement_scale": 0.5,
            }
        ]
    elif fault_kind == "hole_bounds":
        document["faults"]["depth_holes"] = [
            {
                "start_s": 0.5,
                "end_s": 1.0,
                "rows": [39, 41],
                "cols": [20, 30],
            }
        ]
    else:
        document["faults"]["depth_spikes"] = [
            {
                "start_s": 0.5,
                "end_s": 1.0,
                "row": 40,
                "col": 30,
                "offset_m": 2.0,
            }
        ]

    with pytest.raises(module.ScenarioValidationError, match="wheel|shape_px"):
        module.parse_scenario(document)


@pytest.mark.parametrize(
    "filename",
    ("flat_straight_5m.yaml", "pivot_90deg.yaml", "bank_transition.yaml"),
)
def test_representative_scenarios_are_complete_and_loadable(filename):
    module = _scenario_module()

    scenario = module.load_scenario(SCENARIO_DIR / filename)

    assert scenario.scenario_id
    assert scenario.track.centerline_m
    assert scenario.sensors["depth"]["shape_px"] == (40, 60)
    assert set(scenario.expected_metrics) >= {
        "completion",
        "min_clearance_m",
        "edge_overrun_count",
        "false_hold_count",
        "fail_open_count",
        "max_recovery_time_s",
        "max_estimator_runtime_ms",
    }
