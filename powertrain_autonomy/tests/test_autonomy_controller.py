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
    clearance_m = (
        config.clearance_hold_m + config.clearance_full_m
    ) / 2.0
    estimate = terrain(
        path_offset_m=1.0,
        heading_error_rad=1.0,
        left_wheel_clearance_m=clearance_m,
        right_wheel_clearance_m=clearance_m,
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
    decision = decide_fresh(
        controller,
        0.0,
        estimate=estimate,
        state=state,
        diag=diag,
    )
    for tick in range(1, 41):
        decision = decide_fresh(
            controller,
            tick * 0.25,
            estimate=estimate,
            state=state,
            diag=diag,
        )
    return decision


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


def test_sustained_turn_intent_opens_measured_yaw_rate_damping_gate():
    estimate = terrain(path_offset_m=0.10, heading_error_rad=0.20)
    state = motion(yaw_rate_rad_s=0.30)

    def decision_for(kd_yaw):
        controller = AutonomyController(
            EMPTY_STOWED,
            AutonomyControllerConfig(kd_yaw=kd_yaw),
        )
        decision = decide_fresh(
            controller,
            0.0,
            estimate=estimate,
            state=state,
        )
        for tick in range(1, 21):
            decision = decide_fresh(
                controller,
                tick * 0.25,
                estimate=estimate,
                state=state,
            )
        return decision

    proportional = decision_for(0.0)
    damped = decision_for(0.4)

    assert proportional.state == damped.state == "TRACKING"
    assert proportional.omega_rad_s == pytest.approx(0.32)
    assert damped.omega_rad_s == pytest.approx(0.20)
    assert 0.0 < damped.omega_rad_s < proportional.omega_rad_s


def test_curvature_governor_uses_turn_intent_when_damping_cancels_yaw_command():
    controller = AutonomyController(
        EMPTY_STOWED,
        AutonomyControllerConfig(kd_yaw=0.5),
    )
    estimate = terrain(path_offset_m=0.10, heading_error_rad=0.20)
    state = motion(yaw_rate_rad_s=0.64)

    for tick in range(21):
        decision = decide_fresh(
            controller,
            tick * 0.25,
            estimate=estimate,
            state=state,
        )

    assert decision.state == "TRACKING"
    assert decision.omega_rad_s == pytest.approx(0.0)
    assert decision.v_m_s == pytest.approx(0.6060606060606061)
    assert "curvature_slow" in decision.reasons


def test_nearly_straight_turn_intent_preserves_terrain_induced_yaw_rate():
    estimate = terrain(heading_error_rad=0.001)
    state = motion(yaw_rate_rad_s=0.30)

    def decision_for(kd_yaw):
        controller = AutonomyController(
            EMPTY_STOWED,
            AutonomyControllerConfig(kd_yaw=kd_yaw),
        )
        decision = decide_fresh(
            controller,
            0.0,
            estimate=estimate,
            state=state,
        )
        for tick in range(1, 41):
            decision = decide_fresh(
                controller,
                tick * 0.25,
                estimate=estimate,
                state=state,
            )
        return decision

    proportional = decision_for(0.0)
    damped = decision_for(0.5)

    assert proportional.state == damped.state == "TRACKING"
    assert proportional.omega_rad_s == pytest.approx(0.0012)
    assert damped.omega_rad_s == pytest.approx(
        proportional.omega_rad_s,
        abs=1.0e-3,
    )


def test_blocked_state_clears_sustained_turn_activity():
    controller = AutonomyController(
        EMPTY_STOWED,
        AutonomyControllerConfig(kd_yaw=0.5),
    )
    turning = terrain(path_offset_m=0.10, heading_error_rad=0.20)
    yawing = motion(yaw_rate_rad_s=0.30)
    for tick in range(21):
        decide_fresh(
            controller,
            tick * 0.25,
            estimate=turning,
            state=yawing,
        )

    blocked = controller.decide(
        5.25,
        terrain=terrain(5.25),
        motion=motion(5.25),
        gate=gate(5.25, "EXECUTING"),
        diagnostics=None,
    )
    resumed = decide_fresh(
        controller,
        5.50,
        estimate=terrain(heading_error_rad=0.001),
        state=yawing,
    )

    assert blocked.state == "BLOCKED"
    assert resumed.state == "TRACKING"
    assert resumed.omega_rad_s == pytest.approx(0.0012, abs=1.0e-3)


def test_controlled_hold_clears_sustained_turn_activity():
    controller = AutonomyController(
        EMPTY_STOWED,
        AutonomyControllerConfig(kd_yaw=0.5),
    )
    turning = terrain(path_offset_m=0.10, heading_error_rad=0.20)
    yawing = motion(yaw_rate_rad_s=0.30)
    for tick in range(21):
        decide_fresh(
            controller,
            tick * 0.25,
            estimate=turning,
            state=yawing,
        )

    held = decide_fresh(
        controller,
        5.25,
        estimate=terrain(path_available=False),
        state=yawing,
    )
    nearly_straight = terrain(heading_error_rad=0.001)
    assert decide_fresh(
        controller,
        5.50,
        estimate=nearly_straight,
        state=yawing,
    ).state == "CONTROLLED_HOLD"
    assert decide_fresh(
        controller,
        5.75,
        estimate=nearly_straight,
        state=yawing,
    ).state == "CONTROLLED_HOLD"
    resumed = decide_fresh(
        controller,
        6.00,
        estimate=nearly_straight,
        state=yawing,
    )

    assert held.state == "CONTROLLED_HOLD"
    assert resumed.state == "TRACKING"
    assert resumed.omega_rad_s == pytest.approx(0.0012, abs=1.0e-3)


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
    for tick in range(9):
        now_s = tick * 0.25
        decision = decide_fresh(controller, now_s)
    assert decision.v_m_s == pytest.approx(0.8)

    stale = terrain(1.0)
    previous = decision.v_m_s
    previous_now = 2.0
    for now_s in (2.1, 2.2, 2.5, 3.0, 3.5, 3.75):
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

    dwell_one = decide_fresh(controller, 3.85)
    dwell_two = decide_fresh(controller, 3.95)
    recovered = decide_fresh(controller, 4.05)
    assert dwell_one.state == dwell_two.state == "CONTROLLED_HOLD"
    assert dwell_one.v_m_s == dwell_two.v_m_s == 0.0
    assert recovered.state == "TRACKING"
    assert 0.0 < recovered.v_m_s <= EMPTY_STOWED.max_accel_m_s2 * 0.1 + 1e-12


def test_controlled_hold_recovery_requires_three_consecutive_fresh_ticks():
    controller = AutonomyController(EMPTY_STOWED)
    assert decide_fresh(controller, 0.0).state == "TRACKING"
    held = controller.decide(
        1.0,
        terrain=terrain(0.0),
        motion=motion(1.0),
        gate=gate(1.0),
        diagnostics=None,
    )
    assert held.state == "CONTROLLED_HOLD"
    assert "terrain_stale" in held.reasons

    first = decide_fresh(controller, 1.1)
    second = decide_fresh(controller, 1.2)
    third = decide_fresh(controller, 1.3)

    assert first.state == second.state == "CONTROLLED_HOLD"
    assert first.reasons == second.reasons == ("recovery_dwell",)
    assert third.state == "TRACKING"


def test_recovery_dwell_reduces_threshold_flap_transitions():
    def states(recovery_ticks):
        controller = AutonomyController(
            EMPTY_STOWED,
            # A3 dwell의 시간·표본 조건은 중립화 — 이 테스트는 틱 수 효과만
            # 격리해 비교한다(신규 조건은 전용 테스트가 커버).
            AutonomyControllerConfig(
                recovery_ticks=recovery_ticks,
                recovery_min_elapsed_s=0.0,
                recovery_min_samples=1,
            ),
        )
        result = []
        for index, age_s in enumerate((0.451, 0.449) * 4):
            now_s = 1.0 + index * 0.05
            decision = controller.decide(
                now_s,
                terrain=terrain(now_s - age_s),
                motion=motion(now_s),
                gate=gate(now_s),
                diagnostics=None,
            )
            result.append(decision.state)
        return result

    immediate = states(1)
    conservative = states(3)
    immediate_transitions = sum(a != b for a, b in zip(immediate, immediate[1:]))
    conservative_transitions = sum(a != b for a, b in zip(conservative, conservative[1:]))

    assert conservative_transitions < immediate_transitions
    assert set(conservative) == {"CONTROLLED_HOLD"}


def test_blocked_recovery_semantics_remain_immediate_without_hold_dwell():
    controller = AutonomyController(EMPTY_STOWED)
    controller.decide(
        1.0,
        terrain=terrain(0.0),
        motion=motion(1.0),
        gate=gate(1.0),
        diagnostics=None,
    )
    blocked = controller.decide(
        1.1,
        terrain=terrain(1.1),
        motion=motion(1.1),
        gate=gate(1.1, "EXECUTING"),
        diagnostics=None,
    )
    resumed = decide_fresh(controller, 1.2)

    assert blocked.state == "BLOCKED"
    assert resumed.state == "TRACKING"


def test_blocked_is_immediate_and_resets_slew_origin():
    controller = AutonomyController(EMPTY_STOWED)
    for tick in range(9):
        now_s = tick * 0.25
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


def test_recentering_slack_matches_one_clearance_quantisation_step():
    """보고 clearance 는 0.05 m 격자 에지의 중앙값이라 0.025 m 단위로 계단진다.
    슬랙이 그보다 작으면 첫 양자화 하강에 창이 래치돼 재정렬이 불가능해진다 —
    실측에서 개방 0.40 s 뒤 drop 0.0250 으로 닫힌 뒤 78 틱 내내 닫혀 있었다.
    한 스텝 하락은 통과하고 두 스텝은 잡아야 한다.
    """
    config = AutonomyControllerConfig()
    step_m = 0.025

    assert config.recentring_margin_slack_m == pytest.approx(step_m)
    assert not step_m > config.recentring_margin_slack_m
    assert 2 * step_m > config.recentring_margin_slack_m


def test_curvature_slow_is_reported_only_when_it_actually_costs_speed():
    """조향이 0 이 아니기만 하면 붙던 사유가 감속량과 무관해져, 실측 2415/3000 틱을
    병목으로 오독하게 만들었다. 실제 감속이 1% 를 넘을 때만 붙어야 한다.
    """
    config = AutonomyControllerConfig()

    negligible = steady_decision(estimate=terrain(path_offset_m=0.001))
    substantial = steady_decision(estimate=terrain(path_offset_m=0.5))

    # 0.8 * 0.001 = 0.0008 -> 나눗수 1.0008, 0.08% 감속
    assert "curvature_slow" not in negligible.reasons
    # 0.8 * 0.5 = 0.4 -> 나눗수 1.4, 29% 감속
    assert "curvature_slow" in substantial.reasons
    # 보고 임계일 뿐이므로 감속식 자체는 그대로다.
    assert substantial.v_m_s == pytest.approx(
        negligible.v_m_s / (1.0 + config.kp_offset * 0.5),
        rel=0.05,
    )


def test_measured_course_corridor_keeps_at_least_quarter_profile_speed():
    """0.95 m was measured from course.stl at the frozen real-course pose,
    matching its ground-truth corridor width of 0.950 m exactly.
    """
    corridor_width_m = 0.95
    as_built_v2_footprint_width_m = 0.789
    clearance_m = (
        corridor_width_m - as_built_v2_footprint_width_m
    ) / 2.0

    decision = steady_decision(
        estimate=terrain(
            left_wheel_clearance_m=clearance_m,
            right_wheel_clearance_m=clearance_m,
        ),
    )

    assert decision.v_m_s >= 0.25 * EMPTY_STOWED.max_speed_m_s


def test_measured_offcentre_corridor_opens_guarded_recentering_arc():
    with pytest.raises(ValueError, match="recentring_speed_m_s"):
        AutonomyControllerConfig(recentring_speed_m_s=True)

    config = AutonomyControllerConfig()
    controller = AutonomyController(EMPTY_STOWED, config)
    estimate = terrain(
        path_offset_m=-0.075,
        heading_error_rad=0.20,
        left_wheel_clearance_m=0.0055,
        right_wheel_clearance_m=0.1555,
    )

    for tick in range(5):
        decide_fresh(controller, tick * 0.25)
    decision = decide_fresh(controller, 1.25, estimate=estimate)

    assert decision.state != "CONTROLLED_HOLD"
    assert 0.0 < decision.v_m_s <= config.recentring_speed_m_s
    assert "recentring" in decision.reasons
    assert decision.omega_rad_s < 0.0


def test_recentering_rejects_corridor_that_does_not_fit():
    decision = decide_fresh(
        AutonomyController(EMPTY_STOWED),
        0.0,
        estimate=terrain(
            left_wheel_clearance_m=0.01,
            right_wheel_clearance_m=0.01,
        ),
    )

    assert decision.state == "CONTROLLED_HOLD"
    assert "clearance_low" in decision.reasons
    assert decision.v_m_s == 0.0


def test_recentering_rejects_wheel_outside_support():
    decision = decide_fresh(
        AutonomyController(EMPTY_STOWED),
        0.0,
        estimate=terrain(
            left_wheel_clearance_m=-0.02,
            right_wheel_clearance_m=0.30,
        ),
    )

    assert decision.state == "CONTROLLED_HOLD"
    assert "clearance_low" in decision.reasons
    assert decision.v_m_s == 0.0


def test_recentering_falling_margin_pauses_window_during_cooldown():
    """The tick after the violation still holds because it is inside the cooldown."""
    config = AutonomyControllerConfig()
    controller = AutonomyController(EMPTY_STOWED, config)
    initial = terrain(
        left_wheel_clearance_m=0.04,
        right_wheel_clearance_m=0.12,
    )
    eroded = terrain(
        left_wheel_clearance_m=0.04 - config.recentring_margin_slack_m - 0.001,
        right_wheel_clearance_m=0.14,
    )

    decide_fresh(controller, 0.0, estimate=initial)
    moving = decide_fresh(controller, 0.1, estimate=initial)
    paused = decide_fresh(controller, 0.2, estimate=eroded)
    still_paused = decide_fresh(controller, 0.3, estimate=eroded)

    assert moving.state != "CONTROLLED_HOLD"
    assert moving.v_m_s > 0.0
    assert paused.state == "CONTROLLED_HOLD"
    assert "clearance_low" in paused.reasons
    assert still_paused.state == "CONTROLLED_HOLD"
    assert "clearance_low" in still_paused.reasons


def test_recentering_window_reopens_after_cooldown():
    """A clock rollback must start a new cooldown epoch, not preserve the old one."""
    config = AutonomyControllerConfig()
    controller = AutonomyController(EMPTY_STOWED, config)
    initial = terrain(
        left_wheel_clearance_m=0.04,
        right_wheel_clearance_m=0.12,
    )
    eroded = terrain(
        left_wheel_clearance_m=0.04 - config.recentring_margin_slack_m - 0.001,
        right_wheel_clearance_m=0.14,
    )

    decide_fresh(controller, 0.0, estimate=initial)
    decide_fresh(controller, 0.1, estimate=initial)
    decide_fresh(controller, 0.2, estimate=eroded)
    decide_fresh(controller, 0.3, estimate=eroded)
    decide_fresh(controller, 2.21, estimate=eroded)
    decide_fresh(controller, 2.31, estimate=eroded)
    reopened = decide_fresh(controller, 2.41, estimate=eroded)

    assert reopened.state != "CONTROLLED_HOLD"
    assert reopened.v_m_s > 0.0
    assert "recentring" in reopened.reasons

    rollback_controller = AutonomyController(EMPTY_STOWED, config)
    decide_fresh(rollback_controller, 100.0, estimate=initial)
    decide_fresh(rollback_controller, 100.1, estimate=initial)
    decide_fresh(rollback_controller, 100.2, estimate=eroded)
    rolled_back = decide_fresh(rollback_controller, 50.0, estimate=eroded)
    decide_fresh(rollback_controller, 100.21, estimate=eroded)
    decide_fresh(rollback_controller, 100.31, estimate=eroded)
    reopened_after_rollback = decide_fresh(
        rollback_controller,
        100.41,
        estimate=eroded,
    )

    assert rolled_back.state == "CONTROLLED_HOLD"
    assert "clearance_low" in rolled_back.reasons
    assert reopened_after_rollback.state != "CONTROLLED_HOLD"
    assert reopened_after_rollback.v_m_s > 0.0
    assert "recentring" in reopened_after_rollback.reasons


def test_recentering_window_does_not_reopen_before_cooldown():
    config = AutonomyControllerConfig()
    controller = AutonomyController(EMPTY_STOWED, config)
    initial = terrain(
        left_wheel_clearance_m=0.04,
        right_wheel_clearance_m=0.12,
    )
    eroded = terrain(
        left_wheel_clearance_m=0.04 - config.recentring_margin_slack_m - 0.001,
        right_wheel_clearance_m=0.14,
    )

    decide_fresh(controller, 0.0, estimate=initial)
    decide_fresh(controller, 0.1, estimate=initial)
    decide_fresh(controller, 0.2, estimate=eroded)
    held = decide_fresh(controller, 2.199, estimate=eroded)

    assert held.state == "CONTROLLED_HOLD"
    assert "clearance_low" in held.reasons
    with pytest.raises(ValueError, match="recentring_cooldown_s"):
        AutonomyControllerConfig(recentring_cooldown_s=True)


def test_recentering_wheel_outside_support_holds_after_cooldown():
    config = AutonomyControllerConfig()
    controller = AutonomyController(EMPTY_STOWED, config)
    initial = terrain(
        left_wheel_clearance_m=0.04,
        right_wheel_clearance_m=0.12,
    )
    eroded = terrain(
        left_wheel_clearance_m=0.04 - config.recentring_margin_slack_m - 0.001,
        right_wheel_clearance_m=0.14,
    )

    decide_fresh(controller, 0.0, estimate=initial)
    decide_fresh(controller, 0.1, estimate=initial)
    decide_fresh(controller, 0.2, estimate=eroded)
    held = decide_fresh(
        controller,
        2.21,
        estimate=terrain(
            left_wheel_clearance_m=-0.02,
            right_wheel_clearance_m=0.30,
        ),
    )

    assert held.state == "CONTROLLED_HOLD"
    assert "clearance_low" in held.reasons


def test_recentering_margin_slack_tolerates_quantisation_noise():
    with pytest.raises(ValueError, match="recentring_margin_slack_m"):
        AutonomyControllerConfig(recentring_margin_slack_m=True)

    config = AutonomyControllerConfig()
    controller = AutonomyController(EMPTY_STOWED, config)
    # 슬랙이 한 양자화 스텝(0.025)으로 커지면서 기존 기저값 0.02 는 dither 뒤
    # 부동소수점상 음수가 되어 "바퀴가 support 밖" 가드에 먼저 걸렸다. 의도는
    # 그대로 두고 기저값만 올려 dither 뒤에도 양수이면서 정지 임계 아래에 남게 한다.
    initial = terrain(
        left_wheel_clearance_m=0.04,
        right_wheel_clearance_m=0.12,
    )
    dithered = terrain(
        left_wheel_clearance_m=0.04 - 0.8 * config.recentring_margin_slack_m,
        right_wheel_clearance_m=0.14,
    )

    decide_fresh(controller, 0.0, estimate=initial)
    moving = decide_fresh(controller, 0.1, estimate=initial)
    decision = decide_fresh(controller, 0.2, estimate=dithered)

    assert moving.v_m_s > 0.0
    assert decision.state != "CONTROLLED_HOLD"
    assert "recentring" in decision.reasons


def test_recentering_window_times_out_and_fails_closed_on_time_rollback():
    config = AutonomyControllerConfig()
    controller = AutonomyController(EMPTY_STOWED, config)
    estimate = terrain(
        left_wheel_clearance_m=0.02,
        right_wheel_clearance_m=0.12,
    )

    decide_fresh(controller, 0.0, estimate=estimate)
    moving = decide_fresh(controller, 0.1, estimate=estimate)
    timed_out = decide_fresh(
        controller,
        config.recentring_timeout_s + 0.01,
        estimate=estimate,
    )

    assert moving.v_m_s > 0.0
    assert timed_out.state == "CONTROLLED_HOLD"
    assert "clearance_low" in timed_out.reasons

    rollback_controller = AutonomyController(EMPTY_STOWED, config)
    decide_fresh(rollback_controller, 100.0, estimate=estimate)
    assert decide_fresh(
        rollback_controller,
        100.1,
        estimate=estimate,
    ).v_m_s > 0.0
    assert decide_fresh(
        rollback_controller,
        104.9,
        estimate=estimate,
    ).v_m_s > 0.0
    rolled_back = decide_fresh(
        rollback_controller,
        104.0,
        estimate=estimate,
    )

    assert rolled_back.state == "CONTROLLED_HOLD"
    assert "clearance_low" in rolled_back.reasons
    with pytest.raises(ValueError, match="recentring_timeout_s"):
        AutonomyControllerConfig(recentring_timeout_s=True)


@pytest.mark.parametrize(
    ("field", "full", "slow", "hold", "slow_reason"),
    (
        ("clearance", None, None, None, "clearance_slow"),
        ("bank", 0.0, math.radians(11.0), math.radians(15.1), "bank_slow"),
        ("slope", 0.0, math.radians(12.0), math.radians(15.1), "slope_slow"),
        ("confidence", 0.61, 0.40, 0.24, "confidence_slow"),
    ),
)
def test_terrain_speed_scales_are_monotonic_and_hold_beyond_boundary(field, full, slow, hold, slow_reason):
    if field == "clearance":
        config = AutonomyControllerConfig()
        ramp_width = config.clearance_full_m - config.clearance_hold_m
        full = config.clearance_full_m + ramp_width
        slow = (config.clearance_hold_m + config.clearance_full_m) / 2.0
        hold = config.clearance_hold_m - 0.001

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
    for tick in range(41):
        now_s = tick * 0.25
        stale = controller.decide(
            now_s,
            terrain=terrain(now_s),
            motion=motion(now_s),
            gate=gate(now_s),
            diagnostics=diagnostics(
                now_s - 1.01,
                slip_candidate=True,
                speed_cap_m_s=0.1,
            ),
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


def test_time_rollback_never_moves_dt_origin_back_and_recovery_dt_is_clamped():
    controller = AutonomyController(EMPTY_STOWED)

    initial = decide_fresh(controller, 0.0)
    advanced = decide_fresh(controller, 0.2)
    rollback = decide_fresh(controller, 0.0)
    recovered = decide_fresh(controller, 2.0)

    assert initial.v_m_s == 0.0
    assert rollback.v_m_s == advanced.v_m_s
    assert recovered.v_m_s - rollback.v_m_s <= (
        EMPTY_STOWED.max_accel_m_s2 * 0.25 + 1e-12
    )


def test_blocked_rollback_does_not_move_slew_origin_back():
    controller = AutonomyController(EMPTY_STOWED)
    decide_fresh(controller, 0.0)
    decide_fresh(controller, 0.2)

    blocked = controller.decide(
        0.0,
        terrain=terrain(0.0),
        motion=motion(0.0),
        gate=gate(0.0, "EXECUTING"),
        diagnostics=None,
    )
    recovered_before_origin = decide_fresh(controller, 0.1)

    assert blocked.state == "BLOCKED"
    assert recovered_before_origin.v_m_s == 0.0


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
        ("yaw_damp_gate_rad_s", 0.0),
        ("yaw_damp_gate_rad_s", -0.1),
        ("yaw_damp_tau_s", 0.0),
        ("yaw_damp_tau_s", -0.1),
        ("clearance_hold_m", -0.1),
        ("clearance_full_m", 0.04),
        ("min_confidence", -0.1),
        ("full_confidence", 0.20),
        ("confidence_floor_scale", 0.0),
        ("confidence_floor_scale", 1.1),
        ("slip_scale", 0.0),
        ("slip_scale", 1.1),
        ("recovery_ticks", 0),
        ("recovery_ticks", -1),
        ("recovery_ticks", 1.5),
        ("recovery_ticks", True),
    ),
)
def test_invalid_controller_config_raises_value_error(field, value):
    with pytest.raises(ValueError):
        AutonomyControllerConfig(**{field: value})


@pytest.mark.parametrize("kd_yaw", (-0.1, math.nan, math.inf, True))
def test_invalid_kd_yaw_raises_specific_value_error(kd_yaw):
    with pytest.raises(
        ValueError,
        match="^kd_yaw must be finite and non-negative$",
    ):
        AutonomyControllerConfig(kd_yaw=kd_yaw)


@pytest.mark.parametrize("kd_yaw", (0.0, 0.4))
def test_nonnegative_finite_kd_yaw_is_accepted(kd_yaw):
    assert AutonomyControllerConfig(kd_yaw=kd_yaw).kd_yaw == kd_yaw


def test_nonfinite_now_is_rejected_before_a_nonfinite_decision_can_escape():
    with pytest.raises(ValueError, match="now_s"):
        AutonomyController(EMPTY_STOWED).decide(
            math.nan,
            terrain=terrain(),
            motion=motion(),
            gate=gate(),
            diagnostics=None,
        )


def test_recovery_dwell_requires_min_elapsed_time_not_just_ticks():
    """A3: 부하로 틱이 빨리 돌아도 최소 경과 시간(0.15 s) 전엔 복귀 금지."""
    controller = AutonomyController(EMPTY_STOWED)
    decide_fresh(controller, 0.0)
    held = controller.decide(
        1.0, terrain=terrain(0.0), motion=motion(1.0), gate=gate(1.0),
        diagnostics=None,
    )
    assert held.state == "CONTROLLED_HOLD"

    # 0.02 s 간격 4틱: ticks(3)·samples(3) 충족, 경과 0.08 s < 0.15 s
    for step in range(1, 5):
        decision = decide_fresh(controller, 1.0 + 0.02 * step)
        assert decision.state == "CONTROLLED_HOLD"
        assert decision.reasons == ("recovery_dwell",)

    # 첫 fresh 틱(1.02) 기준 경과 0.14 s → 아직 dwell
    assert decide_fresh(controller, 1.16).state == "CONTROLLED_HOLD"
    # 경과 0.16 s → 복귀
    assert decide_fresh(controller, 1.18).state == "TRACKING"


def test_recovery_dwell_requires_tick_count_even_when_time_elapsed():
    controller = AutonomyController(EMPTY_STOWED)
    decide_fresh(controller, 0.0)
    controller.decide(
        1.0, terrain=terrain(0.0), motion=motion(1.0), gate=gate(1.0),
        diagnostics=None,
    )

    # 0.2 s 간격 2틱: 경과는 충족하나 ticks 2 < 3
    assert decide_fresh(controller, 1.2).state == "CONTROLLED_HOLD"
    assert decide_fresh(controller, 1.4).state == "CONTROLLED_HOLD"
    assert decide_fresh(controller, 1.6).state == "TRACKING"


def test_recovery_dwell_requires_distinct_terrain_samples():
    """같은 terrain 스탬프 재사용은 표본으로 안 센다 — 데이터 정체 시 복귀 금지."""
    controller = AutonomyController(EMPTY_STOWED)
    decide_fresh(controller, 0.0)
    controller.decide(
        1.0, terrain=terrain(0.0), motion=motion(1.0), gate=gate(1.0),
        diagnostics=None,
    )

    frozen = terrain(1.5)
    for step in range(1, 11):          # 10틱·경과 0.2 s, 표본은 1개뿐
        decision = controller.decide(
            1.5 + 0.02 * step,
            terrain=frozen,
            motion=motion(1.5 + 0.02 * step),
            gate=gate(1.5 + 0.02 * step),
            diagnostics=None,
        )
    assert decision.state == "CONTROLLED_HOLD"

    # 신선한 표본 2개 더 → 전 조건 충족 → 복귀
    decide_fresh(controller, 1.72)
    assert decide_fresh(controller, 1.74).state == "TRACKING"


def test_recovery_dwell_all_conditions_met_returns_tracking():
    controller = AutonomyController(
        EMPTY_STOWED,
        AutonomyControllerConfig(
            recovery_ticks=2,
            recovery_min_elapsed_s=0.05,
            recovery_min_samples=2,
        ),
    )
    decide_fresh(controller, 0.0)
    controller.decide(
        1.0, terrain=terrain(0.0), motion=motion(1.0), gate=gate(1.0),
        diagnostics=None,
    )
    assert decide_fresh(controller, 1.1).state == "CONTROLLED_HOLD"
    assert decide_fresh(controller, 1.2).state == "TRACKING"
