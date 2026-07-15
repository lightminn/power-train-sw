import csv
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from powertrain_autonomy.sensor_qualification import (
    PitchMetrics,
    PitchRequirements,
    RigidTransform,
    qualify_extrinsic_repeatability,
    qualify_optical_axes,
    qualify_pitch_bracket,
    qualify_pose_occlusion,
    qualify_stream_skew,
    qualify_stream_timing,
    qualify_transform,
)


ROOT = Path(__file__).resolve().parents[2]
COMMISSIONING_SCRIPT = ROOT / "scripts" / "l515_commissioning.py"
REQUIRED_POSES = ("folded", "work", "carry", "abnormal_stop")


def test_stream_timing_counts_equal_and_regressing_stamps_and_clock_delta():
    result = qualify_stream_timing(
        header_stamps_s=(10.0, 10.0, 9.9, 10.2),
        receive_times_s=(10.01, 10.02, 10.0, 10.21),
        max_abs_clock_delta_s=0.05,
    )

    assert result.equal_stamp_count == 1
    assert result.regressing_stamp_count == 1
    assert result.max_abs_clock_delta_s == pytest.approx(0.1)
    assert set(result.reject_reasons) == {
        "clock_delta_exceeded",
        "equal_stamp",
        "regressing_stamp",
    }
    assert not result.passed


def test_stream_skew_preserves_each_sensor_value_and_identifies_extremes():
    stamps = {"rgb": 20.000, "depth": 20.012, "imu": 19.995, "wheel": 20.025}

    result = qualify_stream_skew(stamps, max_skew_s=0.02)

    assert result.stamps_s == stamps
    assert result.max_skew_s == pytest.approx(0.030)
    assert result.oldest_stream == "imu"
    assert result.newest_stream == "wheel"
    assert result.reject_reasons == ("stream_skew_exceeded",)


def test_base_link_to_l515_transform_uses_measurement_stamp_and_rep103_metres():
    transform = RigidTransform(
        target_frame="base_link",
        source_frame="l515_link",
        translation_m=(1.0, 2.0, 3.0),
        rotation=np.eye(3),
    )

    result = qualify_transform(
        transform,
        measurement_stamp_s=100.0,
        transform_stamp_s=99.98,
        max_tf_age_s=0.05,
        known_target_sensor_m=(0.2, -0.1, 1.0),
        expected_target_base_m=(1.2, 1.9, 4.0),
        max_xyz_error_m=(0.01, 0.01, 0.01),
    )

    assert result.passed
    assert result.tf_age_s == pytest.approx(0.02)
    assert result.transformed_target_base_m == pytest.approx((1.2, 1.9, 4.0))
    assert result.xyz_error_m == pytest.approx((0.0, 0.0, 0.0))


def test_transform_rejects_wrong_frame_direction_stale_tf_and_known_target_error():
    wrong_direction = RigidTransform(
        target_frame="l515_link",
        source_frame="base_link",
        translation_m=(0.0, 0.0, 0.0),
        rotation=np.eye(3),
    )

    result = qualify_transform(
        wrong_direction,
        measurement_stamp_s=10.0,
        transform_stamp_s=9.5,
        max_tf_age_s=0.1,
        known_target_sensor_m=(1.0, 0.0, 0.0),
        expected_target_base_m=(0.0, 0.0, 0.0),
        max_xyz_error_m=(0.05, 0.05, 0.05),
    )

    assert set(result.reject_reasons) == {
        "invalid_transform_frames",
        "tf_stale",
        "known_target_xyz_error",
    }


def test_d435i_optical_axis_signs_are_qualified_from_injected_rotation():
    result = qualify_optical_axes(
        rotation_sensor_to_base=np.diag((1.0, 1.0, -1.0)),
        expected_base_directions={
            "x": (1.0, 0.0, 0.0),
            "y": (0.0, 1.0, 0.0),
            "z": (0.0, 0.0, 1.0),
        },
        sensor_name="d435i",
        min_alignment=0.99,
    )

    assert result.axis_alignment["x"] == pytest.approx(1.0)
    assert result.axis_alignment["z"] == pytest.approx(-1.0)
    assert result.reject_reasons == ("d435i_optical_axis_z_sign",)


def test_arm_pose_occlusion_requires_all_named_poses_and_enforces_ratio():
    result = qualify_pose_occlusion(
        {"folded": 0.1, "work": 0.35, "carry": 0.2},
        required_poses=REQUIRED_POSES,
        max_occlusion_ratio=0.25,
    )

    assert set(result.reject_reasons) == {
        "missing_pose:abnormal_stop",
        "roi_occlusion:work",
    }


def test_extrinsic_repeatability_is_evaluated_independently_for_each_arm_pose():
    result = qualify_extrinsic_repeatability(
        {
            "folded": ((0.0, 0.0, 0.0), (0.002, 0.0, 0.0)),
            "work": ((0.0, 0.0, 0.0), (0.03, 0.0, 0.0)),
            "carry": ((0.0, 0.0, 0.0), (0.001, 0.001, 0.0)),
            "abnormal_stop": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.003)),
        },
        required_poses=REQUIRED_POSES,
        max_spread_m=0.01,
    )

    assert result.max_spread_by_pose_m["work"] == pytest.approx(0.03)
    assert result.reject_reasons == ("extrinsic_repeatability:work",)


def pitch_metrics(pitch_deg, **overrides):
    values = {
        "pitch_deg": pitch_deg,
        "near_blind_spot_m": 0.35,
        "coverage_min_m": 0.45,
        "coverage_max_m": 4.2,
        "footprint_clearance_m": 0.12,
        "below_floor_separation_m": 0.18,
    }
    values.update(overrides)
    return PitchMetrics(**values)


def pitch_requirements():
    return PitchRequirements(
        max_near_blind_spot_m=0.5,
        required_coverage_min_m=0.5,
        required_coverage_max_m=4.0,
        min_footprint_clearance_m=0.1,
        min_below_floor_separation_m=0.15,
    )


def test_pitch_bracket_requires_20_25_30_and_records_raw_metrics_per_candidate():
    result = qualify_pitch_bracket(
        (
            pitch_metrics(20.0, near_blind_spot_m=0.7),
            pitch_metrics(25.0),
            pitch_metrics(30.0, footprint_clearance_m=0.05),
        ),
        requirements=pitch_requirements(),
    )

    by_pitch = {candidate.metrics.pitch_deg: candidate for candidate in result.candidates}
    assert result.reject_reasons == ()
    assert by_pitch[20.0].reject_reasons == ("near_blind_spot",)
    assert by_pitch[25.0].passed
    assert by_pitch[30.0].reject_reasons == ("footprint_clearance",)
    assert by_pitch[25.0].metrics.coverage_max_m == 4.2


def test_pitch_bracket_rejects_missing_candidate_without_guessing_an_angle():
    result = qualify_pitch_bracket(
        (pitch_metrics(20.0), pitch_metrics(25.0)),
        requirements=pitch_requirements(),
    )

    assert result.reject_reasons == ("missing_pitch_candidate:30",)


def commissioning_input():
    common = {
        "roi": {"x": 80, "y": 160, "width": 480, "height": 240},
        "depth_thresholds": {
            "min_depth_m": 0.2,
            "max_depth_m": 4.5,
            "min_valid_ratio": 0.8,
        },
        "base_link_to_l515_link": {
            "translation_m": [0.42, 0.0, 0.61],
            "rotation_xyzw": [0.0, 0.21644, 0.0, 0.97630],
        },
    }

    def candidate(pitch, **metrics):
        raw = {
            "near_blind_spot_m": 0.35,
            "coverage_min_m": 0.45,
            "coverage_max_m": 4.2,
            "footprint_clearance_m": 0.12,
            "below_floor_separation_m": 0.18,
            "depth_valid_ratio": 0.96,
            "depth_mad_m": 0.012,
            "tf_age_s": 0.01,
            "known_target_xyz_error_m": [0.006, -0.004, 0.008],
            "roi_occlusion_by_pose": {
                "folded": 0.08,
                "work": 0.12,
                "carry": 0.05,
                "abnormal_stop": 0.15,
            },
        }
        raw.update(metrics)
        return {"pitch_deg": pitch, **common, "raw_metrics": raw}

    return {
        "fixture": {
            "bracket_owner": "mechanical_team",
            "reference_plane_owner": "mechanical_team",
            "reproducible_pitch_deg": [20, 25, 30],
            "reference_plane_available": True,
        },
        "requirements": {
            "max_near_blind_spot_m": 0.5,
            "required_coverage_min_m": 0.5,
            "required_coverage_max_m": 4.0,
            "min_footprint_clearance_m": 0.1,
            "min_below_floor_separation_m": 0.15,
        },
        "candidates": [
            candidate(20, near_blind_spot_m=0.7),
            candidate(25),
            candidate(30, footprint_clearance_m=0.05),
        ],
    }


def run_cli(tmp_path, mode, *extra):
    input_path = tmp_path / "input.json"
    jsonl_path = tmp_path / "metrics.jsonl"
    csv_path = tmp_path / "metrics.csv"
    yaml_path = tmp_path / "l515_terrain.yaml"
    input_path.write_text(json.dumps(commissioning_input()), encoding="utf-8")
    completed = subprocess.run(
        [
            sys.executable,
            str(COMMISSIONING_SCRIPT),
            "--mode",
            mode,
            "--input",
            str(input_path),
            "--jsonl",
            str(jsonl_path),
            "--csv",
            str(csv_path),
            "--output-yaml",
            str(yaml_path),
            *extra,
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    return completed, jsonl_path, csv_path, yaml_path


def test_commissioning_cli_dry_run_writes_raw_logs_and_hash_but_not_yaml(tmp_path):
    completed, jsonl_path, csv_path, yaml_path = run_cli(
        tmp_path, "dry-run", "--approve-pitch", "25"
    )

    assert completed.returncode == 0, completed.stderr
    summary = json.loads(completed.stdout)
    assert summary["mode"] == "dry-run"
    assert summary["candidate_count"] == 3
    assert summary["approved_pitch_deg"] == 25.0
    assert summary["yaml_written"] is False
    assert len(summary["yaml_sha256"]) == 64
    assert not yaml_path.exists()

    records = [json.loads(line) for line in jsonl_path.read_text().splitlines()]
    assert [record["pitch_deg"] for record in records] == [20.0, 25.0, 30.0]
    assert records[1]["raw_metrics"]["coverage_max_m"] == 4.2
    assert records[1]["raw_metrics"]["depth_valid_ratio"] == 0.96
    assert records[1]["raw_metrics"]["known_target_xyz_error_m"] == [
        0.006,
        -0.004,
        0.008,
    ]
    assert records[1]["passed"] is True

    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 3
    assert rows[1]["pitch_deg"] == "25.0"
    assert rows[1]["passed"] == "True"
    assert json.loads(rows[1]["raw_metrics_json"])["roi_occlusion_by_pose"]["work"] == 0.12


def test_commissioning_mode_freezes_only_an_explicit_passing_candidate(tmp_path):
    completed, _, _, yaml_path = run_cli(
        tmp_path, "commissioning", "--approve-pitch", "25"
    )

    assert completed.returncode == 0, completed.stderr
    summary = json.loads(completed.stdout)
    assert summary["yaml_written"] is True
    assert len(summary["yaml_sha256"]) == 64

    frozen = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    assert frozen["qualification"]["status"] == "approved"
    assert frozen["qualification"]["fixture_owner"] == "mechanical_team"
    assert frozen["mount"]["pitch_deg"] == 25.0
    assert frozen["terrain"]["backend"] == "numpy"
    assert frozen["terrain"]["roi"]["width"] == 480
    assert frozen["tf"]["base_link_to_l515_link"]["translation_m"] == [0.42, 0.0, 0.61]


def test_production_mode_is_measurement_only_and_cannot_modify_yaml(tmp_path):
    yaml_path = tmp_path / "l515_terrain.yaml"
    yaml_path.write_text("sentinel: unchanged\n", encoding="utf-8")
    input_path = tmp_path / "input.json"
    input_path.write_text(json.dumps(commissioning_input()), encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            str(COMMISSIONING_SCRIPT),
            "--mode",
            "production",
            "--input",
            str(input_path),
            "--jsonl",
            str(tmp_path / "metrics.jsonl"),
            "--csv",
            str(tmp_path / "metrics.csv"),
            "--output-yaml",
            str(yaml_path),
            "--approve-pitch",
            "25",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )

    assert completed.returncode != 0
    assert "production mode is measurement-only" in completed.stderr
    assert yaml_path.read_text(encoding="utf-8") == "sentinel: unchanged\n"


def test_cli_help_names_mechanical_fixture_handoff_and_no_angle_inference():
    completed = subprocess.run(
        [sys.executable, str(COMMISSIONING_SCRIPT), "--help"],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )

    assert completed.returncode == 0
    assert "20/25/30" in completed.stdout
    assert "mechanical-team handoff" in completed.stdout
    assert "never inferred" in completed.stdout


def test_repository_terrain_yaml_is_fail_closed_until_real_qualification():
    config_path = ROOT / "ros2/src/powertrain_ros/config/l515_terrain.yaml"
    text = config_path.read_text(encoding="utf-8")
    config = yaml.safe_load(text)

    assert config["qualification"]["status"] == "unapproved"
    assert config["qualification"]["production_enabled"] is False
    assert config["qualification"]["required_pitch_candidates_deg"] == [20, 25, 30]
    assert config["mount"]["pitch_deg"] is None
    assert config["terrain"]["backend"] == "numpy"
    assert config["terrain"]["roi"] is None
    assert config["tf"]["base_link_to_l515_link"] is None
    assert "mechanical-team handoff" in text
    assert "must not infer" in text


def test_autonomy_image_and_compose_service_are_profile_gated_and_idle():
    dockerfile = (ROOT / "docker/Dockerfile.autonomy").read_text(encoding="utf-8")
    compose = yaml.safe_load(
        (ROOT / "docker/docker-compose.jetson.yml").read_text(encoding="utf-8")
    )
    service = compose["services"]["powertrain_autonomy"]

    assert dockerfile.startswith("FROM dustynv/l4t-pytorch:r36.4.0\n")
    assert "pip install" not in dockerfile
    assert service["profiles"] == ["autonomy"]
    assert "AUTONOMY_ENABLED=false" in service["environment"]
    assert service["build"] == {
        "context": "..",
        "dockerfile": "docker/Dockerfile.autonomy",
    }
    assert service["entrypoint"] == ["/bin/bash", "-lc"]
    assert "sleep infinity" in str(service["command"])
    assert "/autonomy/cmd_vel" in str(service["command"])
