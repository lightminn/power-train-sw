"""Manual steering must not turn stationary drive wheels or flip on reverse."""
import math

import pytest

from chassis import kinematics
from chassis.authority import CommandAuthority, MANUAL, MANUAL_SOURCE, AUTO, MOTION_HOLD
from chassis.tests.test_chassis_manager import _armed_manager, FakeClock


@pytest.mark.parametrize("steering", [-1, -.5, .5, 1])
def test_stationary_steering_is_ackermann_not_pivot(steering):
    result = kinematics.solve_steering(kinematics.default_geometry(), 0, steering)
    assert result.omega_applied == 0
    for name, wheel in result.wheels.items():
        assert wheel.drive_mps == wheel.drive_turns_per_s == 0
        if name.startswith("front"):
            assert wheel.steer_deg * steering > 0
        elif name.startswith("rear"):
            assert wheel.steer_deg * steering < 0
        else:
            assert wheel.steer_deg == 0
    if abs(steering) == 1:
        assert max(abs(w.steer_deg) for w in result.wheels.values()) == pytest.approx(45)


@pytest.mark.parametrize("steering", [-1, -.3, 0, .3, 1])
def test_same_steering_angles_at_stop_forward_and_reverse(steering):
    geom = kinematics.default_geometry()
    stop = kinematics.solve_steering(geom, 0, steering)
    forward = kinematics.solve_steering(geom, .4, steering)
    reverse = kinematics.solve_steering(geom, -.4, steering)
    for name in stop.wheels:
        assert forward.wheels[name].steer_deg == pytest.approx(stop.wheels[name].steer_deg)
        assert reverse.wheels[name].steer_deg == pytest.approx(stop.wheels[name].steer_deg)
        assert reverse.wheels[name].drive_mps == pytest.approx(-forward.wheels[name].drive_mps)
        assert forward.wheels[name].drive_mps > 0


def test_all_wheels_share_icr_on_fixed_middle_axle():
    geom = kinematics.default_geometry()
    result = kinematics.solve_steering(geom, .4, .7)
    mid_x = next(w.x for w in geom.wheels if w.name == "mid_left")
    radii = []
    for w in geom.wheels:
        cmd = result.wheels[w.name]
        delta = math.radians(cmd.steer_deg)
        assert cmd.drive_mps * math.sin(delta) == pytest.approx(result.omega_applied * (w.x - mid_x))
        assert cmd.drive_mps * math.cos(delta) == pytest.approx(.4 - result.omega_applied * w.y)
        if w.steerable:
            radii.append(w.y + (w.x - mid_x) / math.tan(delta))
    assert radii == pytest.approx([radii[0]] * 4)


def test_yaw_and_wheel_speed_caps_slow_without_unsteering():
    geom = kinematics.default_geometry()
    stop = kinematics.solve_steering(geom, 0, 1)
    fast = kinematics.solve_steering(geom, 10, 1, max_omega_rad_s=.2)
    assert abs(fast.omega_applied) <= .2
    assert fast.speed_clamped
    for name, wheel in fast.wheels.items():
        assert abs(wheel.drive_mps) <= geom.drive_limit_mps
        assert wheel.steer_deg == pytest.approx(stop.wheels[name].steer_deg)


@pytest.mark.parametrize("v,steering", [(float("nan"), 0), (0, float("inf")), (0, 1.01)])
def test_invalid_manual_command_rejected(v, steering):
    with pytest.raises(ValueError):
        kinematics.solve_steering(kinematics.default_geometry(), v, steering)


def test_authority_steering_is_atomic_and_requires_neutral():
    authority = CommandAuthority()
    authority.set_mode(MANUAL)
    authority.submit(MANUAL_SOURCE, 0, 0, 0, steering=.8)
    assert not authority.select(0).ok
    authority.submit(MANUAL_SOURCE, 0, 0, .1, steering=0)
    assert authority.select(.1).ok
    authority.submit(MANUAL_SOURCE, 0, 0, .2, steering=-.8)
    command = authority.select(.2)
    assert command.ok and command.steering == -.8
    assert not authority.request_mode(AUTO, .21).accepted  # needs qualified handover
    assert not authority.select(.6).ok
    assert authority.mode == MOTION_HOLD
    assert authority.select(.61).steering is None


@pytest.mark.parametrize("steering", [float("nan"), 1.1])
def test_authority_bad_steering_enters_hold(steering):
    authority = CommandAuthority()
    authority.set_mode(MANUAL)
    authority.submit(MANUAL_SOURCE, 0, 0, 0, steering=steering)
    assert not authority.select(0).ok
    assert authority.mode == MOTION_HOLD


def test_manager_stationary_steer_and_legacy_switch_clear_intent():
    manager = _armed_manager()
    manager.set(0, 0, steering=.7)
    manager.tick()
    for name, corner in manager.corners.items():
        assert corner.drive.state()["target_vel"] == 0
        if name.startswith("front"):
            assert corner.steer.state()["target_deg"] > 0
    manager.set(0, 0)  # explicit legacy command must not reuse manual steering
    manager.tick()
    assert all(c.steer.state()["target_deg"] == 0 for c in manager.corners.values())


def test_manager_watchdog_and_estop_gate_stationary_steering():
    clock = FakeClock()
    manager = _armed_manager(clock=clock)
    manager.set(0, 0, steering=.7)
    manager.tick()
    before = {n: c.steer.state()["target_deg"] for n, c in manager.corners.items()}
    clock.advance(.31)
    manager.set(0, 0, steering=-.7, received_s=0)  # queued stale intent is discarded
    manager.tick()
    assert {n: c.steer.state()["target_deg"] for n, c in manager.corners.items()} == before
    assert all(c.drive.state()["target_vel"] == 0 for c in manager.corners.values())
    manager.estop("test")
    before = {n: c.steer.state()["target_deg"] for n, c in manager.corners.items()}
    manager.set(0, 0, steering=-.7)
    manager.tick()
    assert {n: c.steer.state()["target_deg"] for n, c in manager.corners.items()} == before


def test_runtime_geometry_matches_reviewed_urdf_snapshot():
    import json
    from pathlib import Path
    evidence = json.loads((Path(__file__).resolve().parents[3] /
        "docs/reports/2026-09-09-ackermann-urdf-evidence.json").read_text())
    for wheel in kinematics.default_geometry().wheels:
        assert (wheel.x, wheel.y) == pytest.approx(
            evidence["wheels"][wheel.name]["symmetric_xy_m"], abs=1e-10)


@pytest.mark.parametrize("speed,steering", [(.4,.7),(-.4,.7),(.4,-.7)])
def test_manual_wheel_targets_roundtrip_through_odometry(speed, steering):
    from chassis.odometry import WheelObservation, solve_twist
    geom = kinematics.default_geometry()
    commands = kinematics.solve_steering(geom, speed, steering)
    estimate = solve_twist(geom, [WheelObservation(w.name, w.drive_mps, w.steer_deg)
                                for w in commands.wheels.values()])
    mid_x = next(w.x for w in geom.wheels if w.name == "mid_left")
    assert estimate.vx == pytest.approx(speed)
    assert estimate.vy == pytest.approx(-commands.omega_applied * mid_x)
    assert estimate.omega == pytest.approx(commands.omega_applied)
    assert estimate.residual_mps == pytest.approx(0, abs=1e-12)
