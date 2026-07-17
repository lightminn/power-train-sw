from motor_control.laptop.haptic_arbiter import (
    HapticArbiter,
    PRIORITY,
    Rumble,
    STALE_S,
)


class FakeClock:
    def __init__(self, now=0.0):
        self.now = float(now)

    def __call__(self):
        return self.now


def _state(**overrides):
    state = {
        "authority_mode": "IDLE",
        "gateway_state": "DRIVE",
        "estop_latched": False,
        "active_estop_sources": [],
        "safety_distance_mm": None,
        "assist_bypass": False,
    }
    state.update(overrides)
    return state


def test_stale_state_forces_link_loss_over_every_positive_or_alarm_pattern():
    clock = FakeClock()
    arbiter = HapticArbiter(clock=clock)
    arbiter.feed_ops_state(
        _state(
            authority_mode="TELEOP",
            estop_latched=True,
            safety_distance_mm=50,
            assist_bypass=True,
        ),
        received_s=clock.now,
    )

    clock.now = STALE_S + 0.001

    assert PRIORITY == (
        "estop",
        "authority",
        "link_loss",
        "proximity",
        "bypass",
    )
    assert arbiter.decide() == Rumble(low=0.55, high=0.0, duration_ms=180)


def test_estop_pattern_has_priority_over_an_authority_transition():
    clock = FakeClock()
    arbiter = HapticArbiter(clock=clock)
    arbiter.feed_ops_state(_state(), received_s=clock.now)
    assert arbiter.decide() is None

    clock.now = 0.1
    arbiter.feed_ops_state(
        _state(authority_mode="AUTONOMY", estop_latched=True),
        received_s=clock.now,
    )

    assert arbiter.decide() == Rumble(low=1.0, high=1.0, duration_ms=250)


def test_authority_transition_emits_exactly_one_pulse():
    clock = FakeClock()
    arbiter = HapticArbiter(clock=clock)
    arbiter.feed_ops_state(_state(authority_mode="IDLE"), received_s=0.0)

    clock.now = 0.1
    arbiter.feed_ops_state(_state(authority_mode="TELEOP"), received_s=0.1)

    assert arbiter.decide() == Rumble(low=0.2, high=0.65, duration_ms=140)
    assert arbiter.decide() is None

    clock.now = 0.2
    arbiter.feed_ops_state(_state(authority_mode="TELEOP"), received_s=0.2)
    assert arbiter.decide() is None


def test_proximity_strength_increases_as_distance_decreases():
    clock = FakeClock()
    far = HapticArbiter(clock=clock)
    near = HapticArbiter(clock=clock)
    far.feed_ops_state(_state(safety_distance_mm=350), received_s=0.0)
    near.feed_ops_state(_state(safety_distance_mm=100), received_s=0.0)

    far_pattern = far.decide()
    near_pattern = near.decide()

    assert far_pattern is not None
    assert near_pattern is not None
    assert 0.0 < far_pattern.low < near_pattern.low <= 1.0
    assert far_pattern.high < near_pattern.high


def test_bypass_alone_is_a_low_intensity_continuous_cue():
    clock = FakeClock()
    arbiter = HapticArbiter(clock=clock)
    # Current broker pushes omit both optional fields; the arbiter must accept
    # the sparse shape before a later producer adds them.
    arbiter.feed_ops_state(
        {"authority_mode": "TELEOP", "assist_bypass": True},
        received_s=0.0,
    )

    pattern = arbiter.decide()

    assert pattern == Rumble(low=0.12, high=0.0, duration_ms=80)


def test_chord_progress_and_ack_have_distinct_one_shot_patterns():
    clock = FakeClock()
    arbiter = HapticArbiter(clock=clock)
    arbiter.feed_ops_state(_state(), received_s=0.0)

    arbiter.feed_event("chord_progress", "arm:begin")
    progress = arbiter.decide()
    assert progress == Rumble(low=0.0, high=0.35, duration_ms=100)
    assert arbiter.decide() is None

    arbiter.feed_event("ack", "FINAL_SUCCESS")
    ack = arbiter.decide()
    assert ack == Rumble(low=0.15, high=0.8, duration_ms=160)
    assert ack != progress
    assert arbiter.decide() is None


def test_lightbar_maps_authority_hold_and_estop_colors():
    clock = FakeClock()
    arbiter = HapticArbiter(clock=clock)

    for state, expected in (
        (_state(authority_mode="AUTONOMY"), (0, 0, 255)),
        (_state(authority_mode="TELEOP"), (255, 255, 255)),
        (_state(authority_mode="MOTION_HOLD"), (255, 191, 0)),
        (_state(estop_latched=True), (255, 0, 0)),
    ):
        arbiter.feed_ops_state(state, received_s=clock.now)
        assert arbiter.lightbar() == expected

    clock.now = STALE_S + 0.001
    assert arbiter.lightbar() is None
