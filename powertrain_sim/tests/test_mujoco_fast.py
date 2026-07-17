from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from chassis.kinematics import default_geometry
from powertrain_autonomy.terrain.depth_quality import CameraIntrinsics
from powertrain_ros.state_estimation import StateEstimator, StateEstimatorConfig
from powertrain_sim.fixtures import DepthFrame
from powertrain_sim.mujoco_fast.runner import (
    HoldMetricsTracker,
    _TrackProjector,
    _apply_depth_degradation,
    run_scenario,
)
from powertrain_sim.procedural import GenerationParameters, generate_scenario
from powertrain_sim.recording import RecordedRun, Replayer
from powertrain_sim.scenario import load_scenario, parse_scenario


SCENARIO_DIR = Path(__file__).resolve().parents[1] / "scenarios"


def _load(name: str):
    return load_scenario(SCENARIO_DIR / name)


def _replay_estimator(run_directory: Path):
    estimator = StateEstimator(default_geometry(), StateEstimatorConfig(bias_samples=0))

    def wheel(sample):
        assert estimator.update_wheels(sample, now_s=sample.stamp_s).accepted

    def imu(sample):
        assert estimator.update_imu(sample, now_s=sample.stamp_s).accepted

    Replayer(run_directory).replay(wheel=wheel, imu=imu)
    records = list(RecordedRun(run_directory).iter_records())
    return estimator.snapshot(now_s=records[-1].stamp_s)


def _truth_distance(run_directory: Path) -> float:
    frames = list(RecordedRun(run_directory).iter_ground_truth())
    return sum(
        math.hypot(right.x_m - left.x_m, right.y_m - left.y_m)
        for left, right in zip(frames, frames[1:])
    )


def _wrapped(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def test_flat_recording_replays_into_production_estimator_within_three_percent(tmp_path):
    scenario = _load("flat_straight_5m.yaml")
    run_directory = tmp_path / "flat"

    report = run_scenario(scenario, run_directory)
    snapshot = _replay_estimator(run_directory)
    truth_distance = _truth_distance(run_directory)

    error_ratio = abs(snapshot.distance_m - truth_distance) / truth_distance
    assert error_ratio <= 0.03
    assert report.distance_error_ratio == pytest.approx(error_ratio, abs=1e-12)
    assert report.completion_ratio > 0.95
    assert report.edge_overrun_count == 0
    # 2026-07-16 물리 평가로 캘리브레이션된 YAML expected_metrics 를 회귀로 고정.
    assert report.passed, report.reasons


def test_pivot_recording_replays_into_production_estimator_within_three_percent_yaw(
    tmp_path,
):
    scenario = _load("pivot_90deg.yaml")
    run_directory = tmp_path / "pivot"

    report = run_scenario(scenario, run_directory)
    snapshot = _replay_estimator(run_directory)
    truth = list(RecordedRun(run_directory).iter_ground_truth())
    truth_yaw = _wrapped(truth[-1].yaw_rad - truth[0].yaw_rad)
    yaw_error_ratio = abs(_wrapped(snapshot.pose.yaw_rad - truth_yaw)) / abs(truth_yaw)

    assert yaw_error_ratio <= 0.03
    assert report.yaw_error_ratio == pytest.approx(yaw_error_ratio, abs=1e-12)
    # 제자리 피벗은 completion=false 가 기대값 — 캘리브레이션된 YAML 로 PASS 여야 한다.
    assert report.passed, report.reasons


def test_two_runs_have_byte_identical_jsonl_and_npz_recordings(tmp_path):
    scenario = _load("pivot_90deg.yaml")
    first = tmp_path / "first"
    second = tmp_path / "second"

    run_scenario(scenario, first)
    run_scenario(scenario, second)

    first_files = sorted(
        path.relative_to(first)
        for path in first.rglob("*")
        if path.is_file() and (path.suffix in {".jsonl", ".npz"})
    )
    second_files = sorted(
        path.relative_to(second)
        for path in second.rglob("*")
        if path.is_file() and (path.suffix in {".jsonl", ".npz"})
    )
    assert first_files == second_files
    assert first_files
    for relative_path in first_files:
        assert (first / relative_path).read_bytes() == (second / relative_path).read_bytes()


def test_depth_degradation_ramp_mask_and_noise_are_seed_deterministic():
    raw = np.full((20, 20), 1000, dtype=np.uint16)
    raw.setflags(write=False)
    frame = DepthFrame(
        stamp_s=1.5,
        depth_roi=raw,
        depth_scale_m=0.001,
        intrinsics=CameraIntrinsics(20.0, 20.0, 9.5, 9.5),
        frame_id="l515_depth_optical_frame",
    )
    faults = (
        {
            "start_s": 0.0,
            "end_s": 1.0,
            "dropout_ratio_start": 0.2,
            "dropout_ratio_end": 0.6,
            "noise_std_m": 0.01,
        },
    )

    first = _apply_depth_degradation(
        frame,
        elapsed_s=0.5,
        faults=faults,
        rng=np.random.Generator(np.random.PCG64(77).jumped()),
    )
    second = _apply_depth_degradation(
        frame,
        elapsed_s=0.5,
        faults=faults,
        rng=np.random.Generator(np.random.PCG64(77).jumped()),
    )

    np.testing.assert_array_equal(first.depth_roi, second.depth_roi)
    assert np.count_nonzero(first.depth_roi == 0) == 160
    assert np.any(first.depth_roi[first.depth_roi > 0] != 1000)
    assert not first.depth_roi.flags.writeable
    assert np.all(frame.depth_roi == 1000)


def test_depth_degradation_runner_repeats_identical_depth_bytes_from_scenario_seed(
    tmp_path,
):
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(1.5, 1.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            linear_speed_range_m_s=(0.5, 0.5),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
        ),
        seed=45,
        seed_class="dev",
    )
    document["faults"] = {name: [] for name in document["faults"]}
    document["faults"]["depth_degradation"] = [
        {
            "start_s": 0.0,
            "end_s": document["clock"]["duration_s"],
            "dropout_ratio_start": 0.2,
            "dropout_ratio_end": 0.6,
            "noise_std_m": 0.01,
        }
    ]
    scenario = parse_scenario(document)
    first = tmp_path / "degraded-first"
    second = tmp_path / "degraded-second"

    run_scenario(scenario, first)
    run_scenario(scenario, second)

    first_depth = sorted((first / "depth").glob("*.npz"))
    second_depth = sorted((second / "depth").glob("*.npz"))
    assert len(first_depth) == len(second_depth) > 0
    assert [path.read_bytes() for path in first_depth] == [
        path.read_bytes() for path in second_depth
    ]


def test_procedural_dev_scenario_runs_end_to_end_and_command_hook_gets_estimate(
    tmp_path,
):
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(1.5, 1.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            linear_speed_range_m_s=(0.5, 0.5),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
        ),
        seed=44,
        seed_class="dev",
    )
    scenario = parse_scenario(document)
    estimates = []

    def command_source(_t_s, latest_estimate):
        estimates.append(latest_estimate)
        return 0.5, 0.0

    report = run_scenario(
        scenario,
        tmp_path / "procedural",
        command_source=command_source,
    )

    assert report.completion_ratio > 0.9
    assert report.wall_clock_runtime_s > 0.0
    assert report.max_estimator_runtime_ms >= 0.0
    assert estimates[0] is None
    assert any(estimate is not None for estimate in estimates[1:])


def test_edge_overrun_counts_a_real_boundary_entry(tmp_path):
    document = yaml.safe_load(
        (SCENARIO_DIR / "flat_straight_5m.yaml").read_text(encoding="utf-8")
    )
    document["scenario_id"] = "edge_overrun"
    document["clock"]["duration_s"] = 4.0
    document["motion"]["yaw_rate_rad_s"] = 0.65
    document["expected_metrics"]["completion"] = False
    document["expected_metrics"]["edge_overrun_count"] = 20
    scenario = parse_scenario(document)

    report = run_scenario(scenario, tmp_path / "edge")

    assert report.edge_overrun_count > 0


def test_hold_metrics_count_episodes_and_measure_recovery_after_release():
    tracker = HoldMetricsTracker()

    tracker.observe(0.0, actual_hold=False, should_hold=False)
    tracker.observe(1.0, actual_hold=True, should_hold=False)
    tracker.observe(1.1, actual_hold=True, should_hold=False)
    tracker.observe(2.0, actual_hold=False, should_hold=False)
    tracker.observe(3.0, actual_hold=False, should_hold=True)
    tracker.observe(3.1, actual_hold=False, should_hold=True)
    tracker.observe(3.2, actual_hold=True, should_hold=True)
    tracker.observe(4.0, actual_hold=True, should_hold=False)
    tracker.observe(4.3, actual_hold=False, should_hold=False)

    assert tracker.false_hold_count == 1
    assert tracker.fail_open_count == 1
    assert tracker.max_recovery_time_s == pytest.approx(0.3)


def test_completion_station_uses_three_dimensional_centerline_arc_length():
    document = yaml.safe_load(
        (SCENARIO_DIR / "flat_straight_5m.yaml").read_text(encoding="utf-8")
    )
    document["track"] = {
        "centerline_m": [
            [0.0, 0.0, 0.4],
            [0.6, 0.0, 1.2],
            [1.2, 0.0, 1.2],
        ],
        "width_m": [1.2, 1.2, 1.2],
        "height_m": [0.4, 1.2, 1.2],
        "bank_rad": [0.0, 0.0, 0.0],
        "curvature_per_m": [0.0, 0.0, 0.0],
        "friction_coefficient": [0.8, 0.8, 0.8],
        "drop_boundaries": [
            {"left": True, "right": True},
            {"left": True, "right": True},
            {"left": True, "right": True},
        ],
    }
    projector = _TrackProjector(parse_scenario(document))

    projection = projector.project((0.6, 0.0, 1.2))

    assert projection.station_m == pytest.approx(1.0)
    assert projector.length_m == pytest.approx(1.6)
