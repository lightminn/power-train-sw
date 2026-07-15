from __future__ import annotations

from importlib import import_module
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from chassis.kinematics import default_geometry
from powertrain_autonomy.terrain.depth_quality import analyze_depth_quality
from powertrain_ros.state_estimation import StateEstimator, StateEstimatorConfig
from powertrain_sim.scenario import load_scenario, parse_scenario


SCENARIO_DIR = Path(__file__).resolve().parents[1] / "scenarios"


def _fixture_module():
    return import_module("powertrain_sim.fixtures")


def _load(name: str):
    return load_scenario(SCENARIO_DIR / name)


def _stream_bytes(fixture) -> bytes:
    parts: list[bytes] = []
    for sample in fixture.wheel_states:
        parts.append(np.asarray([sample.stamp_s], dtype="<f8").tobytes())
        for wheel in sample.wheels:
            parts.append(wheel.name.encode("utf-8") + b"\0")
            parts.append(
                np.asarray(
                    [
                        wheel.command_turns_per_s,
                        wheel.measured_turns_per_s,
                        wheel.steer_deg,
                        float(wheel.stale),
                    ],
                    dtype="<f8",
                ).tobytes()
            )
    for sample in fixture.imu:
        parts.append(
            np.asarray(
                [
                    sample.stamp_s,
                    sample.gyro_x_rad_s,
                    sample.gyro_y_rad_s,
                    sample.gyro_z_rad_s,
                    sample.accel_x_m_s2,
                    sample.accel_y_m_s2,
                    sample.accel_z_m_s2,
                ],
                dtype="<f8",
            ).tobytes()
        )
    for frame in fixture.depth:
        parts.append(np.asarray([frame.stamp_s], dtype="<f8").tobytes())
        parts.append(frame.depth_roi.tobytes(order="C"))
    return b"".join(parts)


def _run_estimator(fixture):
    estimator = StateEstimator(
        default_geometry(),
        StateEstimatorConfig(bias_samples=0),
    )
    timeline = sorted(
        [(sample.stamp_s, 0, sample) for sample in fixture.wheel_states]
        + [(sample.stamp_s, 1, sample) for sample in fixture.imu],
        key=lambda item: (item[0], item[1]),
    )
    for stamp_s, stream_order, sample in timeline:
        if stream_order == 0:
            decision = estimator.update_wheels(sample, now_s=stamp_s)
        else:
            decision = estimator.update_imu(sample, now_s=stamp_s)
        assert decision.accepted
    return estimator.snapshot(now_s=timeline[-1][0])


def test_same_pcg64_seed_produces_byte_identical_streams_and_scenario_clock():
    module = _fixture_module()
    scenario = _load("bank_transition.yaml")

    first = module.generate_fixture(scenario)
    second = module.generate_fixture(scenario)

    assert _stream_bytes(first) == _stream_bytes(second)
    assert first.wheel_states[0].stamp_s == scenario.clock.start_s
    assert first.wheel_states[-1].stamp_s == pytest.approx(
        scenario.clock.start_s + scenario.clock.duration_s
    )
    assert len(first.wheel_states) == scenario.clock.sample_count
    assert not hasattr(module, "time")


def test_flat_straight_fixture_drives_production_estimator_within_one_percent():
    module = _fixture_module()
    fixture = module.generate_fixture(_load("flat_straight_5m.yaml"))

    snapshot = _run_estimator(fixture)

    error_ratio = abs(snapshot.distance_m - 5.0) / 5.0
    assert error_ratio <= 0.01
    assert snapshot.pose.x_m == pytest.approx(5.0, rel=0.01)
    assert snapshot.pose.y_m == pytest.approx(0.0, abs=1e-9)


def test_pivot_fixture_drives_production_estimator_within_two_percent_yaw():
    module = _fixture_module()
    fixture = module.generate_fixture(_load("pivot_90deg.yaml"))

    snapshot = _run_estimator(fixture)

    target = math.pi / 2.0
    yaw_error = abs(math.atan2(
        math.sin(snapshot.pose.yaw_rad - target),
        math.cos(snapshot.pose.yaw_rad - target),
    ))
    assert yaw_error / target <= 0.02
    assert snapshot.yaw_source == "imu"


def test_depth_frame_is_directly_consumed_by_task4_depth_quality_contract():
    module = _fixture_module()
    frame = module.generate_fixture(_load("flat_straight_5m.yaml")).depth[0]

    result = analyze_depth_quality(
        frame.depth_roi,
        depth_scale_m=frame.depth_scale_m,
        intrinsics=frame.intrinsics,
        frame_stamp_s=frame.stamp_s,
    )

    assert frame.depth_roi.shape == (40, 60)
    assert frame.depth_roi.dtype == np.uint16
    assert result.accepted
    assert result.robust_depth_m == pytest.approx(1.5, abs=0.005)


def test_bank_fixture_contains_acceleration_slip_dropout_hole_spike_and_bank():
    module = _fixture_module()
    scenario = _load("bank_transition.yaml")
    fixture = module.generate_fixture(scenario)
    start_s = scenario.clock.start_s

    front_commands = [
        abs(sample.wheels[0].command_turns_per_s)
        for sample in fixture.wheel_states
    ]
    assert front_commands[0] == pytest.approx(0.0)
    assert max(front_commands) > front_commands[50] > 0.0
    assert front_commands[-1] == pytest.approx(0.0)

    slip_samples = [
        sample
        for sample in fixture.wheel_states
        if 3.0 <= sample.stamp_s - start_s < 4.0
    ]
    rear_right = [
        next(wheel for wheel in sample.wheels if wheel.name == "rear_right")
        for sample in slip_samples
    ]
    assert rear_right
    assert all(
        wheel.measured_turns_per_s
        == pytest.approx(0.5 * wheel.command_turns_per_s, abs=0.01)
        for wheel in rear_right
    )

    expected_imu_without_dropout = scenario.clock.sample_count
    assert len(fixture.imu) < expected_imu_without_dropout

    hole_frames = [
        frame
        for frame in fixture.depth
        if 2.0 <= frame.stamp_s - start_s < 2.4
    ]
    spike_frames = [
        frame
        for frame in fixture.depth
        if 5.0 <= frame.stamp_s - start_s < 5.2
    ]
    assert hole_frames and np.all(hole_frames[0].depth_roi[15:25, 25:35] == 0)
    assert spike_frames and spike_frames[0].depth_roi[20, 30] > 3000

    middle = min(fixture.depth, key=lambda frame: abs(frame.stamp_s - (start_s + 4.0)))
    column_medians = np.median(middle.depth_roi, axis=0)
    assert np.ptp(column_medians) > 20
    assert fixture.ground_truth[-1].x_m == pytest.approx(
        scenario.track.centerline_m[-1][0], abs=1e-6
    )
    assert fixture.ground_truth[-1].z_m == pytest.approx(
        scenario.track.centerline_m[-1][2], abs=1e-6
    )


def test_ground_truth_advances_along_3d_centerline_arc_length_not_elapsed_fraction():
    module = _fixture_module()
    document = yaml.safe_load(
        (SCENARIO_DIR / "flat_straight_5m.yaml").read_text(encoding="utf-8")
    )
    document["scenario_id"] = "right_angle_track"
    document["clock"]["duration_s"] = 4.0
    document["track"] = {
        "centerline_m": [[0.0, 0.0, 0.4], [1.0, 0.0, 0.4], [1.0, 1.0, 0.4]],
        "width_m": [1.2, 1.2, 1.2],
        "height_m": [0.4, 0.4, 0.4],
        "bank_rad": [0.0, 0.1, 0.2],
        "curvature_per_m": [0.0, 1.0, 0.0],
        "friction_coefficient": [0.8, 0.8, 0.8],
        "drop_boundaries": [
            {"left": True, "right": True},
            {"left": True, "right": True},
            {"left": True, "right": True},
        ],
    }
    scenario = parse_scenario(document)

    fixture = module.generate_fixture(scenario)

    assert fixture.ground_truth[-1].x_m == pytest.approx(1.0, abs=1e-6)
    assert fixture.ground_truth[-1].y_m == pytest.approx(1.0, abs=1e-6)
    assert fixture.ground_truth[-1].z_m == pytest.approx(0.4, abs=1e-6)
    halfway = fixture.ground_truth[len(fixture.ground_truth) // 2]
    assert halfway.x_m == pytest.approx(1.0, abs=0.02)
    assert halfway.y_m == pytest.approx(0.0, abs=0.02)


def test_pivot_stops_at_declared_target_when_duration_has_extra_time():
    module = _fixture_module()
    document = yaml.safe_load(
        (SCENARIO_DIR / "pivot_90deg.yaml").read_text(encoding="utf-8")
    )
    document["clock"]["duration_s"] = 2.0
    scenario = parse_scenario(document)

    fixture = module.generate_fixture(scenario)
    snapshot = _run_estimator(fixture)

    assert fixture.ground_truth[-1].yaw_rad == pytest.approx(math.pi / 2, abs=1e-9)
    assert fixture.imu[-1].gyro_z_rad_s == pytest.approx(0.0, abs=0.01)
    assert snapshot.pose.yaw_rad == pytest.approx(math.pi / 2, rel=0.02)
