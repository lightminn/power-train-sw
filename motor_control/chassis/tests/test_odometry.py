import math

import pytest

from chassis.kinematics import default_geometry, solve
from chassis.odometry import (
    OdometryConfig,
    OdometryIntegrator,
    TwistEstimate,
    WheelObservation,
    solve_twist,
)


def _observations(geom, v_mps, omega_rad_s):
    command = solve(geom, v_mps, omega_rad_s)
    observations = [
        WheelObservation(
            name,
            wheel.drive_mps,
            wheel.steer_deg,
        )
        for name, wheel in command.wheels.items()
    ]
    return command, observations


@pytest.mark.parametrize(
    ("v_mps", "omega_rad_s"),
    [(0.5, 0.0), (0.4, 0.25), (0.4, -0.25), (-0.3, 0.0)],
)
def test_inverse_and_forward_kinematics_round_trip(v_mps, omega_rad_s):
    geom = default_geometry()
    command, observations = _observations(geom, v_mps, omega_rad_s)

    estimate = solve_twist(geom, observations)

    assert estimate.vx == pytest.approx(v_mps, abs=1e-8)
    assert estimate.vy == pytest.approx(0.0, abs=1e-8)
    assert estimate.omega == pytest.approx(command.omega_applied, abs=1e-8)
    assert estimate.used == 6
    assert estimate.rejected == ()


def test_turns_per_second_conversion_uses_wheel_circumference():
    observation = WheelObservation.from_turns_per_s(
        "front_left",
        2.0,
        wheel_radius_m=0.1,
    )

    assert observation.drive_mps == pytest.approx(0.4 * math.pi)


def test_invalid_wheels_are_excluded_from_solution():
    geom = default_geometry()
    _, observations = _observations(geom, 0.35, 0.0)
    observations = [
        WheelObservation(o.name, o.drive_mps, o.steer_deg, o.name != "mid_left")
        for o in observations
    ]

    estimate = solve_twist(geom, observations)

    assert estimate.vx == pytest.approx(0.35, abs=1e-8)
    assert estimate.used == 5


def test_fewer_than_three_valid_wheels_fail_safe_to_zero():
    geom = default_geometry()
    _, observations = _observations(geom, 0.35, 0.0)
    observations = [
        WheelObservation(o.name, o.drive_mps, o.steer_deg, index < 2)
        for index, o in enumerate(observations)
    ]

    estimate = solve_twist(geom, observations)

    assert estimate == TwistEstimate(0.0, 0.0, 0.0, (), 0.0, 0)


def test_single_slipping_wheel_is_rejected_without_biasing_twist():
    geom = default_geometry()
    command, observations = _observations(geom, 0.4, 0.2)
    observations = [
        WheelObservation(
            o.name,
            o.drive_mps * (2.0 if o.name == "front_left" else 1.0),
            o.steer_deg,
        )
        for o in observations
    ]

    estimate = solve_twist(
        geom,
        observations,
        OdometryConfig(slip_tol_mps=0.02),
    )

    assert estimate.rejected == ("front_left",)
    assert estimate.vx == pytest.approx(0.4, abs=1e-8)
    assert estimate.omega == pytest.approx(command.omega_applied, abs=1e-8)


def test_integrator_uses_midpoint_heading_and_wraps_yaw():
    integrator = OdometryIntegrator(theta=math.pi - 0.1)
    twist = TwistEstimate(1.0, 0.0, 1.0)

    x, y, yaw = integrator.update(twist, 0.2)

    assert x == pytest.approx(math.cos(math.pi) * 0.2)
    assert y == pytest.approx(math.sin(math.pi) * 0.2, abs=1e-12)
    assert yaw == pytest.approx(-math.pi + 0.1)


def test_integrator_can_use_imu_yaw_rate_instead_of_wheel_rate():
    integrator = OdometryIntegrator()
    twist = TwistEstimate(1.0, 0.0, 9.0)

    integrator.update(twist, 0.5, yaw_rate=0.2)

    assert integrator.pose()[2] == pytest.approx(0.1)


def test_nonpositive_dt_does_not_change_pose():
    integrator = OdometryIntegrator(1.0, 2.0, 0.3)

    assert integrator.update(TwistEstimate(9.0, 9.0, 9.0), 0.0) == (1.0, 2.0, 0.3)
    assert integrator.update(TwistEstimate(9.0, 9.0, 9.0), -1.0) == (1.0, 2.0, 0.3)
