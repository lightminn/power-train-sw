"""Deterministic contracts for the pure WP9 degradation FSM."""

from powertrain_autonomy.degradation import (
    DegradationConfig,
    DegradationFsm,
    DegradationOutput,
    DegradationStage,
)


class FakeClock:
    def __init__(self, now_s=0.0):
        self.now_s = float(now_s)

    def __call__(self):
        return self.now_s


def _fsm(config=None, *, clock=None):
    return DegradationFsm(config, clock=clock or FakeClock())


def _update(
    fsm,
    now_s,
    *,
    depth_quality=0.0,
    slip_candidate=False,
    stuck_candidate=False,
    traveled_m=0.0,
):
    return fsm.update(
        depth_quality=depth_quality,
        slip_candidate=slip_candidate,
        stuck_candidate=stuck_candidate,
        traveled_m=traveled_m,
        now_s=now_s,
    )


def test_normal_output_contract():
    output = _update(_fsm(), 1.0)

    assert output == DegradationOutput(
        stage=DegradationStage.NORMAL,
        speed_scale=1.0,
        request_hold=False,
        handover_wait=False,
        reasons=(),
    )


def test_depth_dropout_enters_slowdown_at_threshold():
    output = _update(_fsm(), 1.0, depth_quality=0.35)

    assert output.stage is DegradationStage.SLOWDOWN
    assert output.speed_scale == 0.5
    assert not output.request_hold
    assert output.reasons == ("depth_dropout",)


def test_slip_candidate_enters_slowdown_without_depth_dropout():
    output = _update(_fsm(), 1.0, slip_candidate=True)

    assert output.stage is DegradationStage.SLOWDOWN
    assert output.speed_scale == 0.5
    assert output.reasons == ("slip_candidate",)


def test_depth_hysteresis_holds_between_thresholds_then_returns_normal():
    fsm = _fsm()

    entered = _update(fsm, 1.0, depth_quality=0.40)
    held = _update(fsm, 2.0, depth_quality=0.25)
    recovered = _update(fsm, 3.0, depth_quality=0.20)

    assert entered.stage is DegradationStage.SLOWDOWN
    assert held.stage is DegradationStage.SLOWDOWN
    assert recovered.stage is DegradationStage.NORMAL
    assert recovered.reasons == ()


def test_consecutive_stuck_ticks_enter_hold_and_accumulate_reasons():
    config = DegradationConfig(stuck_enter_ticks=3)
    fsm = _fsm(config)

    first = _update(
        fsm,
        1.0,
        slip_candidate=True,
        stuck_candidate=True,
    )
    second = _update(
        fsm,
        2.0,
        slip_candidate=True,
        stuck_candidate=True,
    )
    held = _update(
        fsm,
        3.0,
        slip_candidate=True,
        stuck_candidate=True,
    )

    assert first.stage is DegradationStage.SLOWDOWN
    assert second.stage is DegradationStage.SLOWDOWN
    assert held.stage is DegradationStage.HOLD_RECOVERY
    assert held.speed_scale == 0.0
    assert held.request_hold
    assert not held.handover_wait
    assert held.reasons == ("slip_candidate", "stuck_candidate")


def test_unavailable_depth_escalates_slowdown_to_hold_then_recovers_in_stages():
    fsm = _fsm()

    slowed = _update(fsm, 1.0, depth_quality=0.40)
    held = _update(fsm, 2.0, depth_quality=None)
    recovery_slowdown = _update(fsm, 3.0, depth_quality=0.0)
    normal = _update(fsm, 4.0, depth_quality=0.0)

    assert slowed.stage is DegradationStage.SLOWDOWN
    assert held.stage is DegradationStage.HOLD_RECOVERY
    assert held.request_hold
    assert set(held.reasons) == {"depth_dropout", "depth_unavailable"}
    assert recovery_slowdown.stage is DegradationStage.SLOWDOWN
    assert normal.stage is DegradationStage.NORMAL


def test_recovery_attempt_budget_exhaustion_enters_handover_wait():
    config = DegradationConfig(
        stuck_enter_ticks=1,
        recovery_attempts_max=2,
        recovery_time_budget_s=100.0,
        recovery_distance_budget_m=100.0,
    )
    fsm = _fsm(config)

    assert _update(fsm, 0.0, slip_candidate=True, stuck_candidate=True).stage \
        is DegradationStage.SLOWDOWN
    assert _update(fsm, 1.0, slip_candidate=True, stuck_candidate=True).stage \
        is DegradationStage.HOLD_RECOVERY
    assert _update(fsm, 2.0).stage is DegradationStage.SLOWDOWN
    assert _update(fsm, 3.0, slip_candidate=True, stuck_candidate=True).stage \
        is DegradationStage.HOLD_RECOVERY
    assert _update(fsm, 4.0).stage is DegradationStage.SLOWDOWN

    exhausted = _update(
        fsm,
        5.0,
        slip_candidate=True,
        stuck_candidate=True,
    )

    assert exhausted.stage is DegradationStage.HANDOVER_WAIT
    assert exhausted.handover_wait
    assert exhausted.request_hold
    assert "recovery_attempt_budget_exhausted" in exhausted.reasons


def test_recovery_time_budget_exhaustion_enters_handover_wait():
    config = DegradationConfig(
        stuck_enter_ticks=1,
        recovery_attempts_max=10,
        recovery_time_budget_s=2.0,
        recovery_distance_budget_m=100.0,
    )
    fsm = _fsm(config)
    _update(fsm, 0.0, slip_candidate=True, stuck_candidate=True)
    assert _update(fsm, 1.0, slip_candidate=True, stuck_candidate=True).stage \
        is DegradationStage.HOLD_RECOVERY

    before_limit = _update(
        fsm,
        2.99,
        slip_candidate=True,
        stuck_candidate=True,
    )
    exhausted = _update(
        fsm,
        3.0,
        slip_candidate=True,
        stuck_candidate=True,
    )

    assert before_limit.stage is DegradationStage.HOLD_RECOVERY
    assert exhausted.stage is DegradationStage.HANDOVER_WAIT
    assert "recovery_time_budget_exhausted" in exhausted.reasons


def test_recovery_distance_budget_exhaustion_enters_handover_wait():
    config = DegradationConfig(
        stuck_enter_ticks=1,
        recovery_attempts_max=10,
        recovery_time_budget_s=100.0,
        recovery_distance_budget_m=1.5,
    )
    fsm = _fsm(config)
    _update(
        fsm,
        0.0,
        slip_candidate=True,
        stuck_candidate=True,
        traveled_m=10.0,
    )
    assert _update(
        fsm,
        1.0,
        slip_candidate=True,
        stuck_candidate=True,
        traveled_m=10.0,
    ).stage is DegradationStage.HOLD_RECOVERY

    before_limit = _update(
        fsm,
        2.0,
        slip_candidate=True,
        stuck_candidate=True,
        traveled_m=11.49,
    )
    exhausted = _update(
        fsm,
        3.0,
        slip_candidate=True,
        stuck_candidate=True,
        traveled_m=11.5,
    )

    assert before_limit.stage is DegradationStage.HOLD_RECOVERY
    assert exhausted.stage is DegradationStage.HANDOVER_WAIT
    assert "recovery_distance_budget_exhausted" in exhausted.reasons


def test_handover_is_sticky_until_operator_reset_and_inputs_are_deterministic():
    config = DegradationConfig(
        stuck_enter_ticks=1,
        recovery_attempts_max=1,
        recovery_time_budget_s=100.0,
        recovery_distance_budget_m=100.0,
    )
    first = _fsm(config, clock=FakeClock(10.0))
    second = _fsm(config, clock=FakeClock(10.0))
    sequence = (
        dict(now_s=0.0, slip_candidate=True, stuck_candidate=True),
        dict(now_s=1.0, slip_candidate=True, stuck_candidate=True),
        dict(now_s=2.0),
        dict(now_s=3.0, slip_candidate=True, stuck_candidate=True),
    )

    first_outputs = [_update(first, **item) for item in sequence]
    second_outputs = [_update(second, **item) for item in sequence]

    assert first_outputs == second_outputs
    assert first_outputs[-1].stage is DegradationStage.HANDOVER_WAIT
    still_waiting = _update(first, 100.0, depth_quality=0.0)
    assert still_waiting.stage is DegradationStage.HANDOVER_WAIT

    first.operator_reset()
    reset = _update(first, 101.0, depth_quality=0.0)

    assert reset.stage is DegradationStage.NORMAL
    assert reset.reasons == ()
