"""Deterministic synthetic tests for the WP6-A ROS-free estimator core."""

import ast
import math
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from chassis.kinematics import default_geometry
from chassis.wheel_consistency import WheelConsistencyConfig
from powertrain_ros.state_estimation import (
    ImuSample,
    StateEstimator,
    StateEstimatorConfig,
    WheelSample,
    WheelValue,
)


MODULE = (
    Path(__file__).resolve().parents[1]
    / "powertrain_ros"
    / "state_estimation.py"
)
ODOMETRY_NODE = MODULE.with_name("odometry_node.py")
IMU_TILT_NODE = MODULE.with_name("imu_tilt_node.py")
WHEEL_NAMES = (
    "front_left",
    "front_right",
    "mid_left",
    "mid_right",
    "rear_left",
    "rear_right",
)


def _config(**overrides):
    values = {
        "sample_timeout_s": 0.25,
        "bias_samples": 10,
        "accel_lpf_alpha": 0.8,
        "complementary_alpha": 0.98,
        "stationary_command_turns_per_s": 0.05,
        "stationary_measured_turns_per_s": 0.05,
        "wheel_consistency": WheelConsistencyConfig(
            same_side_delta_turns_per_s=0.25,
            yaw_mismatch_rad_s=0.25,
            spin_turns_per_s=1.0,
            stopped_turns_per_s=0.1,
            active_command_turns_per_s=0.5,
            min_response_ratio=0.5,
            max_response_ratio=1.5,
            warn_speed_cap=0.4,
        ),
    }
    values.update(overrides)
    return StateEstimatorConfig(**values)


def _estimator(**overrides):
    geometry = default_geometry()
    return StateEstimator(geometry, _config(**overrides)), geometry


def _imu(stamp_s, *, gyro_z=0.0, accel=(0.0, 0.0, 9.81)):
    return ImuSample(
        stamp_s=stamp_s,
        gyro_x_rad_s=0.0,
        gyro_y_rad_s=0.0,
        gyro_z_rad_s=gyro_z,
        accel_x_m_s2=accel[0],
        accel_y_m_s2=accel[1],
        accel_z_m_s2=accel[2],
    )


def _wheels(
    geometry,
    stamp_s,
    *,
    speed_m_s=0.0,
    command_turns_per_s=None,
    measurements=None,
    commands=None,
    steer=None,
    stale=(),
):
    circumference = 2.0 * math.pi * geometry.wheel_radius_m
    measured_default = speed_m_s / circumference
    command_default = (
        measured_default
        if command_turns_per_s is None
        else command_turns_per_s
    )
    measurements = measurements or {}
    commands = commands or {}
    steer = steer or {}
    return WheelSample(
        stamp_s=stamp_s,
        wheels=tuple(
            WheelValue(
                name=name,
                command_turns_per_s=commands.get(name, command_default),
                measured_turns_per_s=measurements.get(name, measured_default),
                steer_deg=steer.get(name, 0.0),
                stale=name in stale,
            )
            for name in WHEEL_NAMES
        ),
    )


def _feed_straight(estimator, geometry, *, start_s, duration_s, speed_fn, dt=0.02):
    steps = int(round(duration_s / dt))
    for index in range(steps + 1):
        stamp_s = start_s + index * dt
        estimator.update_imu(_imu(stamp_s), now_s=stamp_s)
        estimator.update_wheels(
            _wheels(
                geometry,
                stamp_s,
                speed_m_s=speed_fn(index * dt),
            ),
            now_s=stamp_s,
        )
    return estimator.snapshot(now_s=start_s + duration_s)


def _pivot_wheels(geometry, stamp_s, *, turns_per_s=1.0):
    measurements = {
        name: (-turns_per_s if name.endswith("_left") else turns_per_s)
        for name in WHEEL_NAMES
    }
    return _wheels(
        geometry,
        stamp_s,
        measurements=measurements,
        commands=measurements,
    )


def test_constant_speed_straight_distance_is_integrated_exactly():
    estimator, geometry = _estimator(bias_samples=1)

    snapshot = _feed_straight(
        estimator,
        geometry,
        start_s=1.0,
        duration_s=2.0,
        speed_fn=lambda _elapsed: 0.5,
    )

    assert snapshot.pose.x_m == pytest.approx(1.0, abs=1e-6)
    assert snapshot.pose.y_m == pytest.approx(0.0, abs=1e-9)
    assert snapshot.distance_m == pytest.approx(1.0, abs=1e-6)
    assert snapshot.velocity.forward_m_s == pytest.approx(0.5, rel=1e-6)


def test_acceleration_uses_trapezoidal_distance_integration():
    estimator, geometry = _estimator(bias_samples=1)

    snapshot = _feed_straight(
        estimator,
        geometry,
        start_s=1.0,
        duration_s=2.0,
        speed_fn=lambda elapsed: 0.5 * elapsed,
    )

    assert snapshot.pose.x_m == pytest.approx(1.0, abs=2e-3)
    assert snapshot.distance_m == pytest.approx(1.0, abs=2e-3)
    assert snapshot.velocity.forward_m_s == pytest.approx(1.0, rel=1e-6)


def test_stationary_bias_converges_then_pivot_gyro_integrates_ninety_degrees():
    estimator, geometry = _estimator(bias_samples=5)
    bias = 0.02
    dt = 0.02
    t = 1.0

    estimator.update_wheels(_wheels(geometry, t), now_s=t)
    estimator.update_imu(_imu(t, gyro_z=bias), now_s=t)
    for _ in range(20):
        t += dt
        estimator.update_wheels(_wheels(geometry, t), now_s=t)
        estimator.update_imu(_imu(t, gyro_z=bias), now_s=t)

    stationary = estimator.snapshot(now_s=t)
    assert stationary.gyro_bias_rad_s[2] == pytest.approx(bias, abs=1e-9)
    assert stationary.pose.yaw_rad == pytest.approx(0.0, abs=1e-12)

    yaw_rate = math.pi / 2.0
    for _ in range(50):
        t += dt
        estimator.update_wheels(_pivot_wheels(geometry, t), now_s=t)
        estimator.update_imu(
            _imu(t, gyro_z=bias + yaw_rate),
            now_s=t,
        )

    rotated = estimator.snapshot(now_s=t)
    assert rotated.pose.yaw_rad == pytest.approx(math.pi / 2.0, abs=2e-3)
    assert rotated.yaw_source == "imu"


def test_stationary_pose_has_zero_drift_after_bias_correction():
    estimator, geometry = _estimator(bias_samples=5)
    bias = -0.015
    dt = 0.02
    t = 1.0

    for _ in range(30):
        estimator.update_wheels(_wheels(geometry, t), now_s=t)
        estimator.update_imu(_imu(t, gyro_z=bias), now_s=t)
        t += dt
    before = estimator.snapshot(now_s=t - dt)

    for _ in range(100):
        estimator.update_wheels(_wheels(geometry, t), now_s=t)
        estimator.update_imu(_imu(t, gyro_z=bias), now_s=t)
        t += dt
    after = estimator.snapshot(now_s=t - dt)

    assert after.pose == before.pose
    assert after.distance_m == before.distance_m
    assert after.velocity.forward_m_s == pytest.approx(0.0, abs=1e-12)


def test_startup_bias_learning_waits_when_wheels_report_motion():
    estimator, geometry = _estimator(bias_samples=5)
    t = 1.0

    for _ in range(5):
        estimator.update_wheels(_pivot_wheels(geometry, t), now_s=t)
        estimator.update_imu(_imu(t, gyro_z=0.5), now_s=t)
        t += 0.02
    moving = estimator.snapshot(now_s=t - 0.02)
    assert moving.gyro_bias_rad_s[2] == pytest.approx(0.0, abs=1e-12)

    for _ in range(5):
        estimator.update_wheels(_wheels(geometry, t), now_s=t)
        estimator.update_imu(_imu(t, gyro_z=0.02), now_s=t)
        t += 0.02
    stationary = estimator.snapshot(now_s=t - 0.02)
    assert stationary.gyro_bias_rad_s[2] == pytest.approx(0.02, abs=1e-12)


@pytest.mark.parametrize(
    "bad_stamp, now_s, expected_reason",
    (
        (0.0, 1.1, "stamp_zero"),
        (1.0, 1.1, "stamp_not_monotonic"),
        (0.9, 1.1, "stamp_not_monotonic"),
        (1.2, 1.1, "stamp_future"),
    ),
)
@pytest.mark.parametrize("source", ("wheel", "imu"))
def test_zero_same_regressing_and_future_stamps_are_invalid(
    source,
    bad_stamp,
    now_s,
    expected_reason,
):
    estimator, geometry = _estimator(bias_samples=1)
    if source == "wheel":
        accepted = estimator.update_wheels(
            _wheels(geometry, 1.0),
            now_s=1.0,
        )
        decision = estimator.update_wheels(
            _wheels(geometry, bad_stamp),
            now_s=now_s,
        )
    else:
        accepted = estimator.update_imu(_imu(1.0), now_s=1.0)
        decision = estimator.update_imu(_imu(bad_stamp), now_s=now_s)

    assert accepted.accepted
    assert not decision.accepted
    assert decision.reason == expected_reason


def test_disconnect_freezes_pose_and_reconnect_reinitializes_without_gap_pollution():
    estimator, geometry = _estimator(bias_samples=1)
    before_gap = _feed_straight(
        estimator,
        geometry,
        start_s=1.0,
        duration_s=0.5,
        speed_fn=lambda _elapsed: 1.0,
        dt=0.1,
    )

    stale = estimator.snapshot(now_s=2.0)
    assert stale.stale
    assert stale.pose == before_gap.pose
    assert stale.velocity.forward_m_s == 0.0

    decision = estimator.update_wheels(
        _wheels(geometry, 2.0, speed_m_s=1.0),
        now_s=2.0,
    )
    reconnected = estimator.snapshot(now_s=2.0)
    assert decision.accepted and decision.reinitialized
    assert reconnected.reinitialized
    assert not reconnected.stale
    assert reconnected.pose == before_gap.pose
    assert reconnected.reconnect_count == 1

    estimator.update_wheels(
        _wheels(geometry, 2.1, speed_m_s=1.0),
        now_s=2.1,
    )
    resumed = estimator.snapshot(now_s=2.1)
    assert resumed.pose.x_m == pytest.approx(before_gap.pose.x_m + 0.1)
    assert not resumed.reinitialized


def test_imu_reconnect_reseeds_without_integrating_the_missing_interval():
    estimator, _geometry = _estimator(bias_samples=0)
    estimator.update_imu(_imu(1.0, gyro_z=0.5), now_s=1.0)
    estimator.update_imu(_imu(1.1, gyro_z=0.5), now_s=1.1)
    before_gap = estimator.snapshot(now_s=1.1)

    decision = estimator.update_imu(_imu(1.5, gyro_z=0.5), now_s=1.5)
    reconnected = estimator.snapshot(now_s=1.5)
    assert decision.accepted and decision.reinitialized
    assert reconnected.reinitialized
    assert reconnected.pose.yaw_rad == before_gap.pose.yaw_rad

    estimator.update_imu(_imu(1.6, gyro_z=0.5), now_s=1.6)
    resumed = estimator.snapshot(now_s=1.6)
    assert not resumed.reinitialized
    assert resumed.pose.yaw_rad == pytest.approx(
        before_gap.pose.yaw_rad + 0.05,
        abs=1e-12,
    )


def test_command_measurement_mismatch_sets_slip_candidate_via_existing_monitor():
    estimator, geometry = _estimator(bias_samples=1)
    commands = {name: 1.0 for name in WHEEL_NAMES}
    measurements = {name: 0.2 for name in WHEEL_NAMES}

    decision = estimator.update_wheels(
        _wheels(
            geometry,
            1.0,
            commands=commands,
            measurements=measurements,
        ),
        now_s=1.0,
    )
    diagnostics = estimator.snapshot(now_s=1.0).diagnostics

    assert decision.accepted
    assert diagnostics.slip_candidate
    assert "response_ratio" in diagnostics.warning_codes
    assert diagnostics.terrain_speed_cap == pytest.approx(0.4)


def test_single_wheel_stop_sets_one_wheel_mismatch_via_existing_monitor():
    estimator, geometry = _estimator(bias_samples=1)
    commands = {name: 1.0 for name in WHEEL_NAMES}
    measurements = {name: 1.0 for name in WHEEL_NAMES}
    measurements["front_left"] = 0.0

    estimator.update_wheels(
        _wheels(
            geometry,
            1.0,
            commands=commands,
            measurements=measurements,
        ),
        now_s=1.0,
    )
    diagnostics = estimator.snapshot(now_s=1.0).diagnostics

    assert diagnostics.one_wheel_mismatch
    assert diagnostics.stuck_candidate
    assert "single_wheel_stop" in diagnostics.warning_codes
    assert diagnostics.affected_wheels == ("front_left",)


@pytest.mark.parametrize("failure", ("nan", "stale"))
def test_invalid_or_stale_imu_falls_back_to_auxiliary_wheel_yaw(failure):
    estimator, geometry = _estimator(bias_samples=1)
    t = 1.0
    estimator.update_wheels(_pivot_wheels(geometry, t), now_s=t)
    estimator.update_imu(_imu(t), now_s=t)

    if failure == "nan":
        t = 1.1
        decision = estimator.update_imu(
            _imu(t, gyro_z=math.nan, accel=(math.nan, math.nan, math.nan)),
            now_s=t,
        )
        assert not decision.accepted
    else:
        estimator.update_wheels(_pivot_wheels(geometry, 1.2), now_s=1.2)
        t = 1.4

    estimator.update_wheels(_pivot_wheels(geometry, t), now_s=t)
    snapshot = estimator.snapshot(now_s=t)

    assert snapshot.imu_stale
    assert snapshot.yaw_source == "wheel"
    assert abs(snapshot.pose.yaw_rad) > 0.01
    assert snapshot.diagnostics.wheel_yaw_rate_rad_s is not None


def test_accel_low_pass_produces_finite_roll_and_pitch():
    estimator, geometry = _estimator(
        bias_samples=1,
        accel_lpf_alpha=0.5,
        complementary_alpha=0.0,
    )
    estimator.update_wheels(_wheels(geometry, 1.0), now_s=1.0)
    estimator.update_imu(_imu(1.0), now_s=1.0)

    roll = math.radians(20.0)
    estimator.update_imu(
        _imu(
            1.02,
            accel=(0.0, 9.81 * math.sin(roll), 9.81 * math.cos(roll)),
        ),
        now_s=1.02,
    )
    snapshot = estimator.snapshot(now_s=1.02)

    assert 0.0 < snapshot.tilt.roll_rad < roll
    assert snapshot.tilt.pitch_rad == pytest.approx(0.0, abs=1e-9)


def test_snapshot_is_deeply_immutable_and_core_imports_no_ros_packages():
    estimator, geometry = _estimator(bias_samples=1)
    estimator.update_wheels(_wheels(geometry, 1.0), now_s=1.0)
    snapshot = estimator.snapshot(now_s=1.0)

    with pytest.raises(FrozenInstanceError):
        snapshot.stale = False
    with pytest.raises(FrozenInstanceError):
        snapshot.pose.x_m = 10.0

    tree = ast.parse(MODULE.read_text(encoding="utf-8"))
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".")[0])
    assert imported_roots.isdisjoint(
        {
            "rclpy",
            "builtin_interfaces",
            "geometry_msgs",
            "nav_msgs",
            "powertrain_msgs",
            "sensor_msgs",
            "std_msgs",
        }
    )
    assert "sole mission-arrival condition" in MODULE.read_text(encoding="utf-8")


def test_ros_nodes_are_thin_core_adapters_and_keep_existing_topic_contracts():
    odometry_source = ODOMETRY_NODE.read_text(encoding="utf-8")
    imu_source = IMU_TILT_NODE.read_text(encoding="utf-8")

    assert "StateEstimator(" in odometry_source
    assert "WheelSample(" in odometry_source
    assert "ImuSample(" in odometry_source
    assert "self.estimator.reset()" in odometry_source
    assert '"/odom"' in odometry_source
    assert '"/wheel_states"' in odometry_source
    assert '"/imu/filtered"' in odometry_source
    assert '"~/reset"' in odometry_source
    assert "solve_twist(" not in odometry_source
    assert "OdometryIntegrator(" not in odometry_source

    assert "StateEstimator(" in imu_source
    assert "ImuSample(" in imu_source
    assert '"/l515/accel/sample"' in imu_source
    assert '"/l515/gyro/sample"' in imu_source
    assert '"/imu/filtered"' in imu_source
    assert "self.roll = a *" not in imu_source


def test_large_gyro_during_wheel_stationary_integrates_as_rotation():
    """★ 원칙 '바퀴=거리, IMU=회전' — 정지 바퀴 + 큰 각속도는 bias 가 아니라 실회전이다
    (빙판 슬립·외력). max_bias_rad_s 게이트가 bias 학습을 막고 yaw 로 적분해야 한다."""
    estimator, geometry = _estimator(bias_samples=5)
    dt = 0.02
    t = 0.0
    for _ in range(50):                      # 1.0 s
        t += dt
        estimator.update_wheels(_wheels(geometry, t), now_s=t)
        estimator.update_imu(_imu(t, gyro_z=0.5), now_s=t)
    snapshot = estimator.snapshot(now_s=t)
    assert snapshot.gyro_bias_rad_s[2] == pytest.approx(0.0, abs=1e-9)
    assert snapshot.pose.yaw_rad == pytest.approx(0.5 * (1.0 - dt), rel=0.05)
