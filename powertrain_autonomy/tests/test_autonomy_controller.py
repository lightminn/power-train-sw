from __future__ import annotations

import dataclasses
from dataclasses import FrozenInstanceError
import math

import pytest

from powertrain_autonomy.controller import (
    AutonomyController,
    AutonomyControllerConfig,
    CARRYING_LOCKED,
    EMPTY_STOWED,
    DriveDiagnostics,
    DriveProfile,
    MotionState,
    ProfileGate,
    assist_correction_from_terrain,
    profile_by_name,
    validate_carrying_profile_invariant,
)
from powertrain_autonomy.terrain import TerrainEstimate


def terrain(stamp_s=0.0, **overrides):
    values = {
        "stamp_s": stamp_s,
        "path_offset_m": 0.0,
        "heading_error_rad": 0.0,
        "left_wheel_clearance_m": 0.40,
        "right_wheel_clearance_m": 0.40,
        "bank_angle_rad": 0.0,
        "longitudinal_slope_rad": 0.0,
        "roughness_m": 0.0,
        "confidence": 0.90,
        "degradation_reasons": (),
        "reject_reasons": (),
        "path_available": True,
    }
    values.update(overrides)
    return TerrainEstimate(**values)


def motion(stamp_s=0.0, **overrides):
    values = {
        "stamp_s": stamp_s,
        "forward_m_s": 0.0,
        "yaw_rate_rad_s": 0.0,
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
    }
    values.update(overrides)
    return MotionState(**values)


def gate(stamp_s=0.0, status="STOWED_LOCKED"):
    return ProfileGate(stamp_s=stamp_s, status=status)


def test_assist_correction_requires_an_available_finite_path():
    config = AutonomyControllerConfig()

    assert assist_correction_from_terrain(None, config) is None
    assert assist_correction_from_terrain(
        terrain(path_available=False),
        config,
    ) is None
    assert assist_correction_from_terrain(
        terrain(path_offset_m=math.nan),
        config,
    ) is None


def test_assist_correction_uses_separate_yaw_clamp_and_empty_speed_cap():
    config = AutonomyControllerConfig()
    estimate = terrain(
        path_offset_m=1.0,
        heading_error_rad=1.0,
        left_wheel_clearance_m=0.175,
        right_wheel_clearance_m=0.175,
        bank_angle_rad=math.radians(11.0),
        longitudinal_slope_rad=math.radians(12.0),
        confidence=0.425,
    )

    correction = assist_correction_from_terrain(estimate, config)

    assert correction is not None
    omega, speed_cap, confidence = correction
    assert omega == pytest.approx(0.4)
    assert speed_cap == pytest.approx(0.096)
    assert confidence == pytest.approx(0.425)


def test_assist_speed_cap_is_empty_stowed_max_on_clear_confident_path():
    omega, speed_cap, confidence = assist_correction_from_terrain(
        terrain(path_offset_m=-0.1, heading_error_rad=0.05),
        AutonomyControllerConfig(),
    )

    assert omega == pytest.approx(-0.02)
    assert speed_cap == pytest.approx(EMPTY_STOWED.max_speed_m_s)
    assert confidence == pytest.approx(0.9)


def diagnostics(stamp_s=0.0, **overrides):
    values = {
        "stamp_s": stamp_s,
        "slip_candidate": False,
        "stuck_candidate": False,
        "speed_cap_m_s": math.inf,
    }
    values.update(overrides)
    return DriveDiagnostics(**values)


def decide_fresh(controller, now_s, *, estimate=None, state=None, arm_gate=None, diag=None):
    estimate = terrain(now_s) if estimate is None else dataclasses.replace(estimate, stamp_s=now_s)
    state = motion(now_s) if state is None else dataclasses.replace(state, stamp_s=now_s)
    arm_gate = gate(now_s, controller.profile.required_arm_status) if arm_gate is None else arm_gate
    if diag is not None:
        diag = dataclasses.replace(diag, stamp_s=now_s)
    return controller.decide(
        now_s,
        terrain=estimate,
        motion=state,
        gate=arm_gate,
        diagnostics=diag,
    )


def steady_decision(*, profile=EMPTY_STOWED, estimate=None, state=None, diag=None):
    controller = AutonomyController(profile)
    decide_fresh(controller, 0.0, estimate=estimate, state=state, diag=diag)
    return decide_fresh(controller, 10.0, estimate=estimate, state=state, diag=diag)


def test_profiles_are_frozen_exact_provisional_presets():
    assert EMPTY_STOWED == DriveProfile(
        name="EMPTY_STOWED",
        required_arm_status="STOWED_LOCKED",
        max_speed_m_s=0.8,
        max_accel_m_s2=0.5,
        max_decel_m_s2=0.8,
        max_yaw_rate_rad_s=0.8,
        max_yaw_accel_rad_s2=1.5,
        max_bank_rad=math.radians(15.0),
        soft_bank_rad=math.radians(8.0),
        max_slope_rad=math.radians(15.0),
        soft_slope_rad=math.radians(10.0),
    )
    assert CARRYING_LOCKED == DriveProfile(
        name="CARRYING_LOCKED",
        required_arm_status="CARRYING_LOCKED",
        max_speed_m_s=0.5,
        max_accel_m_s2=0.3,
        max_decel_m_s2=0.6,
        max_yaw_rate_rad_s=0.5,
        max_yaw_accel_rad_s2=1.0,
        max_bank_rad=math.radians(10.0),
        soft_bank_rad=math.radians(5.0),
        max_slope_rad=math.radians(12.0),
        soft_slope_rad=math.radians(8.0),
    )
    with pytest.raises(FrozenInstanceError):
        EMPTY_STOWED.max_speed_m_s = 1.0
    assert profile_by_name("EMPTY_STOWED") is EMPTY_STOWED
    assert profile_by_name("CARRYING_LOCKED") is CARRYING_LOCKED
    with pytest.raises(ValueError, match="drive_profile"):
        profile_by_name("UNKNOWN")


def test_carrying_profile_invariant_checks_every_motion_and_tilt_limit():
    validate_carrying_profile_invariant(EMPTY_STOWED, CARRYING_LOCKED)
    constrained_fields = (
        "max_speed_m_s",
        "max_accel_m_s2",
        "max_decel_m_s2",
        "max_yaw_rate_rad_s",
        "max_yaw_accel_rad_s2",
        "max_bank_rad",
        "soft_bank_rad",
        "max_slope_rad",
        "soft_slope_rad",
    )
    for field in constrained_fields:
        unsafe = dataclasses.replace(
            CARRYING_LOCKED,
            **{field: getattr(EMPTY_STOWED, field) + 0.01},
        )
        with pytest.raises(ValueError, match=field):
            validate_carrying_profile_invariant(EMPTY_STOWED, unsafe)


def test_central_path_tracks_forward_without_yaw():
    decision = steady_decision()
    assert decision.state == "TRACKING"
    assert decision.v_m_s > 0.0
    assert decision.omega_rad_s == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("offset", "heading", "expected_sign"),
    ((0.10, 0.10, 1), (0.10, -0.02, 1), (-0.10, 0.02, -1), (-0.10, -0.10, -1)),
)
def test_offset_and_heading_signs_steer_toward_positive_left_path(offset, heading, expected_sign):
    decision = steady_decision(
        estimate=terrain(path_offset_m=offset, heading_error_rad=heading)
    )
    assert math.copysign(1.0, decision.omega_rad_s) == expected_sign


@pytest.mark.parametrize(
    ("kind", "variant", "reason", "expected_state"),
    (
        ("terrain", "missing", "terrain_missing", "CONTROLLED_HOLD"),
        ("terrain", "stale", "terrain_stale", "CONTROLLED_HOLD"),
        ("terrain", "future", "terrain_future", "CONTROLLED_HOLD"),
        ("motion", "missing", "motion_missing", "CONTROLLED_HOLD"),
        ("motion", "stale", "motion_stale", "CONTROLLED_HOLD"),
        ("motion", "future", "motion_future", "CONTROLLED_HOLD"),
        ("gate", "missing", "gate_missing", "BLOCKED"),
        ("gate", "stale", "gate_stale", "BLOCKED"),
        ("gate", "future", "gate_future", "BLOCKED"),
    ),
)
def test_required_input_loss_matrix(kind, variant, reason, expected_state):
    now_s = 2.0
    values = {
        "terrain": terrain(now_s),
        "motion": motion(now_s),
        "gate": gate(now_s),
        "diagnostics": None,
    }
    if variant == "missing":
        values[kind] = None
    elif variant == "stale":
        timeout = {"terrain": 0.45, "motion": 0.30, "gate": 0.50}[kind]
        values[kind] = dataclasses.replace(values[kind], stamp_s=now_s - timeout - 0.01)
    else:
        values[kind] = dataclasses.replace(values[kind], stamp_s=now_s + 0.11)

    decision = AutonomyController(EMPTY_STOWED).decide(now_s, **values)

    assert decision.state == expected_state
    assert reason in decision.reasons
    assert decision.v_m_s == 0.0
    assert decision.omega_rad_s == 0.0


def test_controlled_hold_decelerates_to_zero_and_recovers_with_slew_limits():
    controller = AutonomyController(EMPTY_STOWED)
    for now_s in (0.0, 1.0, 2.0):
        decision = decide_fresh(controller, now_s)
    assert decision.v_m_s == pytest.approx(0.8)

    stale = terrain(1.0)
    previous = decision.v_m_s
    previous_now = 2.0
    for now_s in (2.1, 2.2, 2.5, 3.0, 3.5):
        decision = controller.decide(
            now_s,
            terrain=stale,
            motion=motion(now_s),
            gate=gate(now_s),
            diagnostics=None,
        )
        assert decision.state == "CONTROLLED_HOLD"
        assert 0.0 <= previous - decision.v_m_s <= EMPTY_STOWED.max_decel_m_s2 * (now_s - previous_now) + 1e-12
        previous = decision.v_m_s
        previous_now = now_s
    assert decision.v_m_s == 0.0

    recovered = decide_fresh(controller, 3.6)
    assert recovered.state == "TRACKING"
    assert 0.0 < recovered.v_m_s <= EMPTY_STOWED.max_accel_m_s2 * 0.1 + 1e-12


def test_blocked_is_immediate_and_resets_slew_origin():
    controller = AutonomyController(EMPTY_STOWED)
    for now_s in (0.0, 1.0, 2.0):
        moving = decide_fresh(controller, now_s)
    assert moving.v_m_s == pytest.approx(0.8)

    blocked = controller.decide(
        2.1,
        terrain=terrain(2.1),
        motion=motion(2.1),
        gate=gate(2.1, "EXECUTING"),
        diagnostics=None,
    )
    assert blocked.state == "BLOCKED"
    assert blocked.v_m_s == blocked.omega_rad_s == 0.0
    assert "arm_status_mismatch" in blocked.reasons

    resumed = decide_fresh(controller, 2.2)
    assert 0.0 < resumed.v_m_s <= EMPTY_STOWED.max_accel_m_s2 * 0.1 + 1e-12


@pytest.mark.parametrize(
    ("field", "full", "slow", "hold", "slow_reason"),
    (
        ("clearance", 0.31, 0.175, 0.049, "clearance_slow"),
        ("bank", 0.0, math.radians(11.0), math.radians(15.1), "bank_slow"),
        ("slope", 0.0, math.radians(12.0), math.radians(15.1), "slope_slow"),
        ("confidence", 0.61, 0.40, 0.24, "confidence_slow"),
    ),
)
def test_terrain_speed_scales_are_monotonic_and_hold_beyond_boundary(field, full, slow, hold, slow_reason):
    def configured(value):
        if field == "clearance":
            return terrain(left_wheel_clearance_m=value, right_wheel_clearance_m=value)
        if field == "bank":
            return terrain(bank_angle_rad=value)
        if field == "slope":
            return terrain(longitudinal_slope_rad=value)
        return terrain(confidence=value)

    fast = steady_decision(estimate=configured(full))
    reduced = steady_decision(estimate=configured(slow))
    stopped = steady_decision(estimate=configured(hold))

    assert 0.0 < reduced.v_m_s < fast.v_m_s
    assert slow_reason in reduced.reasons
    assert stopped.state == "CONTROLLED_HOLD"
    assert stopped.v_m_s == 0.0


def test_measured_roll_alone_can_trigger_controlled_hold():
    decision = steady_decision(
        estimate=terrain(bank_angle_rad=0.0),
        state=motion(roll_rad=EMPTY_STOWED.max_bank_rad + 0.01),
    )
    assert decision.state == "CONTROLLED_HOLD"
    assert "roll_limit" in decision.reasons


def test_fresh_diagnostics_hold_scale_and_cap_but_stale_diagnostics_are_ignored():
    baseline = steady_decision()
    stuck = steady_decision(diag=diagnostics(stuck_candidate=True))
    slipped = steady_decision(diag=diagnostics(slip_candidate=True))
    capped = steady_decision(diag=diagnostics(speed_cap_m_s=0.25))

    assert stuck.state == "CONTROLLED_HOLD"
    assert "stuck_candidate" in stuck.reasons
    assert slipped.v_m_s == pytest.approx(baseline.v_m_s * 0.5)
    assert "slip_candidate" in slipped.reasons
    assert capped.v_m_s == pytest.approx(0.25)
    assert "speed_cap" in capped.reasons

    controller = AutonomyController(EMPTY_STOWED)
    controller.decide(
        0.0,
        terrain=terrain(0.0),
        motion=motion(0.0),
        gate=gate(0.0),
        diagnostics=diagnostics(-1.01, slip_candidate=True, speed_cap_m_s=0.1),
    )
    stale = controller.decide(
        10.0,
        terrain=terrain(10.0),
        motion=motion(10.0),
        gate=gate(10.0),
        diagnostics=diagnostics(8.99, slip_candidate=True, speed_cap_m_s=0.1),
    )
    assert stale.v_m_s == pytest.approx(baseline.v_m_s)
    assert "slip_candidate" not in stale.reasons
    assert "speed_cap" not in stale.reasons


def test_carrying_profile_is_never_faster_for_the_same_geometry_inputs():
    estimate = terrain(path_offset_m=0.08, heading_error_rad=0.05, confidence=0.45)
    empty = steady_decision(profile=EMPTY_STOWED, estimate=estimate)
    carrying = steady_decision(profile=CARRYING_LOCKED, estimate=estimate)
    assert carrying.v_m_s <= empty.v_m_s
    assert abs(carrying.omega_rad_s) <= abs(empty.omega_rad_s)


def test_outputs_remain_finite_nonnegative_and_decisions_are_frozen():
    invalid = terrain(path_offset_m=math.nan)
    decision = steady_decision(estimate=invalid)
    assert decision.state == "CONTROLLED_HOLD"
    assert "terrain_nonfinite" in decision.reasons
    assert decision.v_m_s >= 0.0
    assert all(math.isfinite(value) for value in (decision.stamp_s, decision.v_m_s, decision.omega_rad_s))
    with pytest.raises(FrozenInstanceError):
        decision.v_m_s = -1.0


def test_same_input_sequence_is_deterministic_and_time_regression_holds_slew():
    sequence = (0.0, 0.1, 0.2, 0.15, 0.3)

    def run():
        controller = AutonomyController(EMPTY_STOWED)
        return tuple(
            decide_fresh(
                controller,
                stamp,
                estimate=terrain(path_offset_m=0.05, heading_error_rad=0.02),
            )
            for stamp in sequence
        )

    first = run()
    second = run()
    assert first == second
    assert first[3].v_m_s == first[2].v_m_s
    assert first[3].omega_rad_s == first[2].omega_rad_s


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("terrain_stale_s", 0.0),
        ("motion_stale_s", -1.0),
        ("gate_stale_s", math.inf),
        ("diagnostics_stale_s", math.nan),
        ("kp_heading", -0.1),
        ("kp_offset", -0.1),
        ("curvature_slow_k", -0.1),
        ("clearance_hold_m", -0.1),
        ("clearance_full_m", 0.04),
        ("min_confidence", -0.1),
        ("full_confidence", 0.20),
        ("confidence_floor_scale", 0.0),
        ("confidence_floor_scale", 1.1),
        ("slip_scale", 0.0),
        ("slip_scale", 1.1),
    ),
)
def test_invalid_controller_config_raises_value_error(field, value):
    with pytest.raises(ValueError):
        AutonomyControllerConfig(**{field: value})


def test_nonfinite_now_is_rejected_before_a_nonfinite_decision_can_escape():
    with pytest.raises(ValueError, match="now_s"):
        AutonomyController(EMPTY_STOWED).decide(
            math.nan,
            terrain=terrain(),
            motion=motion(),
            gate=gate(),
            diagnostics=None,
        )
