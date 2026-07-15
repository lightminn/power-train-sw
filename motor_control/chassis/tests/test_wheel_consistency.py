from dataclasses import FrozenInstanceError

import pytest

from chassis.kinematics import default_geometry
from chassis.wheel_consistency import (
    WheelConsistencyConfig,
    WheelConsistencyMonitor,
    WheelConsistencySample,
)


WHEELS = (
    "front_left",
    "front_right",
    "mid_left",
    "mid_right",
    "rear_left",
    "rear_right",
)


def samples(*, commands=None, measurements=None):
    commands = commands or {}
    measurements = measurements or {}
    return tuple(
        WheelConsistencySample(
            name=name,
            command_turns_per_s=commands.get(name, 1.0),
            measured_turns_per_s=measurements.get(name, 1.0),
        )
        for name in WHEELS
    )


def config(**overrides):
    values = {
        "same_side_delta_turns_per_s": 0.25,
        "yaw_mismatch_rad_s": 0.25,
        "spin_turns_per_s": 1.0,
        "stopped_turns_per_s": 0.1,
        "active_command_turns_per_s": 0.5,
        "min_response_ratio": 0.5,
        "max_response_ratio": 1.5,
        "warn_speed_cap": 0.4,
    }
    values.update(overrides)
    return WheelConsistencyConfig(**values)


def codes(result):
    return {warning.code for warning in result.warnings}


def test_matching_commands_measurements_and_imu_have_no_warn():
    monitor = WheelConsistencyMonitor(default_geometry(), config())

    result = monitor.evaluate(samples(), imu_yaw_rate_rad_s=0.0)

    assert result.warnings == ()
    assert result.terrain_speed_cap == 1.0
    assert result.wheel_yaw_rate_rad_s == pytest.approx(0.0)
    assert result.imu_yaw_rate_rad_s == 0.0


def test_same_side_command_measurement_delta_warns_and_caps_speed():
    monitor = WheelConsistencyMonitor(default_geometry(), config())
    measurements = {name: 1.0 for name in WHEELS}
    measurements["mid_left"] = 0.4

    result = monitor.evaluate(
        samples(measurements=measurements),
        imu_yaw_rate_rad_s=None,
    )

    assert "same_side_delta" in codes(result)
    warning = next(w for w in result.warnings if w.code == "same_side_delta")
    assert set(warning.wheels) == {"front_left", "mid_left", "rear_left"}
    assert warning.value == pytest.approx(0.6)
    assert result.terrain_speed_cap == 0.4


def test_left_right_wheel_yaw_mismatch_with_injected_imu_warns():
    monitor = WheelConsistencyMonitor(
        default_geometry(),
        config(max_response_ratio=3.0, same_side_delta_turns_per_s=1.0),
    )
    measurements = {
        name: (0.5 if name.endswith("_left") else 1.0)
        for name in WHEELS
    }

    result = monitor.evaluate(
        samples(measurements=measurements),
        imu_yaw_rate_rad_s=0.0,
    )

    assert result.wheel_yaw_rate_rad_s > 0.25
    assert "yaw_mismatch" in codes(result)


def test_single_wheel_spin_is_detected_at_zero_command():
    monitor = WheelConsistencyMonitor(default_geometry(), config())
    commands = {name: 0.0 for name in WHEELS}
    measurements = {name: 0.0 for name in WHEELS}
    measurements["rear_right"] = 1.2

    result = monitor.evaluate(
        samples(commands=commands, measurements=measurements),
        imu_yaw_rate_rad_s=None,
    )

    warning = next(w for w in result.warnings if w.code == "single_wheel_spin")
    assert warning.wheels == ("rear_right",)


def test_single_commanded_wheel_stop_is_detected():
    monitor = WheelConsistencyMonitor(default_geometry(), config())
    measurements = {name: 1.0 for name in WHEELS}
    measurements["front_left"] = 0.0

    result = monitor.evaluate(
        samples(measurements=measurements),
        imu_yaw_rate_rad_s=None,
    )

    warning = next(w for w in result.warnings if w.code == "single_wheel_stop")
    assert warning.wheels == ("front_left",)


def test_command_encoder_response_ratio_uses_injected_bounds():
    monitor = WheelConsistencyMonitor(
        default_geometry(),
        config(
            stopped_turns_per_s=0.05,
            same_side_delta_turns_per_s=1.0,
        ),
    )
    measurements = {name: 0.4 for name in WHEELS}

    result = monitor.evaluate(
        samples(measurements=measurements),
        imu_yaw_rate_rad_s=None,
    )

    ratio_warnings = [w for w in result.warnings if w.code == "response_ratio"]
    assert {w.wheels[0] for w in ratio_warnings} == set(WHEELS)
    assert all(w.value == pytest.approx(0.4) for w in ratio_warnings)


def test_stale_samples_are_excluded_and_result_is_immutable():
    monitor = WheelConsistencyMonitor(default_geometry(), config())
    stale = tuple(
        WheelConsistencySample(
            name=name,
            command_turns_per_s=1.0,
            measured_turns_per_s=100.0 if name == "front_left" else 1.0,
            stale=name == "front_left",
        )
        for name in WHEELS
    )

    result = monitor.evaluate(stale, imu_yaw_rate_rad_s=0.0)

    assert "single_wheel_spin" not in codes(result)
    with pytest.raises(FrozenInstanceError):
        result.terrain_speed_cap = 0.0
