from dataclasses import FrozenInstanceError

import pytest

from l515_dashboard.network_profiles import (
    CONGESTED,
    EMERGENCY_REMOTE,
    NORMAL,
    ProfileConfig,
    ProfileStateMachine,
    validate_profiles,
)
from l515_dashboard.receiver_feedback import ChannelAvailability


NS = 1_000_000_000


def healthy(loss_percent=0.0):
    return ChannelAvailability(
        available=True,
        verdict=None,
        reason="receiver_report_healthy",
        loss_percent=loss_percent,
    )


def unavailable(verdict="REMOTE_DRIVE_VIDEO_UNAVAILABLE"):
    return ChannelAvailability(
        available=False,
        verdict=verdict,
        reason="receiver_report_stale",
        loss_percent=None,
    )


def tick(machine, seconds, *, l515=None, d435i=None, hint=False, override=None):
    return machine.tick(
        int(seconds * NS),
        l515=healthy() if l515 is None else l515,
        d435i=healthy() if d435i is None else d435i,
        sender_drop_hint=hint,
        operator_override=override,
    )


def test_static_profiles_preserve_resolution_and_30_fps():
    assert validate_profiles() is None
    assert (NORMAL.l515.width, NORMAL.l515.height, NORMAL.l515.fps) == (
        1280,
        720,
        30,
    )
    assert (NORMAL.d435i.width, NORMAL.d435i.height, NORMAL.d435i.fps) == (
        848,
        480,
        30,
    )
    for profile in (CONGESTED, EMERGENCY_REMOTE):
        assert (profile.l515.width, profile.l515.height, profile.l515.fps) == (
            NORMAL.l515.width,
            NORMAL.l515.height,
            NORMAL.l515.fps,
        )
        assert (profile.d435i.width, profile.d435i.height, profile.d435i.fps) == (
            NORMAL.d435i.width,
            NORMAL.d435i.height,
            NORMAL.d435i.fps,
        )


def test_static_profiles_use_monotonic_bitrates_and_expected_flags():
    assert [profile.l515.bitrate_kbps for profile in (
        NORMAL,
        CONGESTED,
        EMERGENCY_REMOTE,
    )] == [3000, 1800, 1800]
    assert [profile.d435i.bitrate_kbps for profile in (
        NORMAL,
        CONGESTED,
        EMERGENCY_REMOTE,
    )] == [2000, 1200, 1200]
    assert all(
        profile.metadata_best_effort
        for profile in (NORMAL, CONGESTED, EMERGENCY_REMOTE)
    )
    assert [profile.l515_rgb_locked for profile in (
        NORMAL,
        CONGESTED,
        EMERGENCY_REMOTE,
    )] == [False, False, True]


def test_profiles_are_frozen():
    with pytest.raises(FrozenInstanceError):
        NORMAL.name = "changed"
    with pytest.raises(FrozenInstanceError):
        NORMAL.l515.fps = 15


def test_l515_unavailable_enters_emergency_immediately():
    machine = ProfileStateMachine(ProfileConfig())
    assert tick(machine, 0).profile is NORMAL

    decision = tick(machine, 0.1, l515=unavailable())

    assert decision.profile is EMERGENCY_REMOTE
    assert decision.changed is True
    assert decision.reason == "receiver_unavailable:l515_rgb"
    assert decision.entered_monotonic_ns == int(0.1 * NS)


def test_d435i_unavailable_also_enters_emergency_immediately():
    machine = ProfileStateMachine(ProfileConfig())
    tick(machine, 0)

    decision = tick(
        machine,
        1,
        d435i=unavailable("REMOTE_ARM_VIDEO_UNAVAILABLE"),
    )

    assert decision.profile is EMERGENCY_REMOTE
    assert decision.reason == "receiver_unavailable:d435i_rgb"


def test_emergency_recovers_to_normal_after_recovery_and_minimum_dwells():
    machine = ProfileStateMachine(
        ProfileConfig(normal_exit_dwell_s=2.0, min_dwell_s=3.0)
    )
    tick(machine, 0)
    tick(machine, 1, l515=unavailable())

    assert tick(machine, 2).profile is EMERGENCY_REMOTE
    assert tick(machine, 3.9).profile is EMERGENCY_REMOTE
    decision = tick(machine, 4)

    assert decision.profile is NORMAL
    assert decision.changed is True
    assert decision.reason == "receiver_recovered"


def test_congested_entry_and_normal_recovery_use_loss_hysteresis():
    machine = ProfileStateMachine(
        ProfileConfig(
            loss_enter_percent=5.0,
            loss_exit_percent=1.0,
            congested_enter_dwell_s=1.0,
            normal_exit_dwell_s=2.0,
            min_dwell_s=3.0,
        )
    )
    tick(machine, 0)

    tick(machine, 1, l515=healthy(6.0))
    assert tick(machine, 2, l515=healthy(6.0)).profile is NORMAL
    entered = tick(machine, 3, l515=healthy(6.0))
    assert entered.profile is CONGESTED
    assert entered.reason == "receiver_loss_high"

    tick(machine, 4, l515=healthy(0.5))
    assert tick(machine, 5.9, l515=healthy(0.5)).profile is CONGESTED
    recovered = tick(machine, 6, l515=healthy(0.5))
    assert recovered.profile is NORMAL
    assert recovered.reason == "receiver_loss_recovered"


def test_sender_drop_hint_alone_never_downgrades():
    machine = ProfileStateMachine(
        ProfileConfig(congested_enter_dwell_s=0.0, min_dwell_s=0.0)
    )

    for second in range(5):
        assert tick(machine, second, hint=True).profile is NORMAL


def test_sender_drop_hint_advances_downgrade_only_with_receiver_loss_evidence():
    machine = ProfileStateMachine(
        ProfileConfig(
            loss_enter_percent=5.0,
            loss_exit_percent=1.0,
            congested_enter_dwell_s=1.0,
            min_dwell_s=0.0,
        )
    )
    tick(machine, 0)

    assert tick(machine, 1, l515=healthy(2.0), hint=True).profile is NORMAL
    decision = tick(machine, 2, l515=healthy(2.0), hint=True)

    assert decision.profile is CONGESTED
    assert decision.reason == "receiver_loss_with_sender_hint"


def test_operator_override_is_immediate_fixed_and_release_resumes_automatic_mode():
    machine = ProfileStateMachine(
        ProfileConfig(normal_exit_dwell_s=2.0, min_dwell_s=3.0)
    )
    tick(machine, 0)

    forced = tick(machine, 1, override="CONGESTED")
    assert forced.profile is CONGESTED
    assert forced.changed is True
    assert forced.reason == "operator_override"
    assert tick(machine, 5, l515=unavailable(), override="CONGESTED").profile is CONGESTED

    assert tick(machine, 6).profile is CONGESTED
    released = tick(machine, 8)
    assert released.profile is NORMAL
    assert released.reason == "receiver_loss_recovered"


def test_operator_override_rejects_unknown_profile_name():
    machine = ProfileStateMachine(ProfileConfig())

    with pytest.raises(ValueError, match="operator_override"):
        tick(machine, 0, override="LOW_RESOLUTION")


def test_minimum_dwell_prevents_flapping_after_congested_entry():
    machine = ProfileStateMachine(
        ProfileConfig(
            congested_enter_dwell_s=0.0,
            normal_exit_dwell_s=2.0,
            min_dwell_s=3.0,
        )
    )
    tick(machine, 0)
    entered = tick(machine, 3, l515=healthy(6.0))
    assert entered.profile is CONGESTED

    tick(machine, 3.1, l515=healthy(0.0))
    tick(machine, 4.0, l515=healthy(6.0))
    tick(machine, 4.5, l515=healthy(0.0))
    assert tick(machine, 6.4, l515=healthy(0.0)).profile is CONGESTED
    assert tick(machine, 6.5, l515=healthy(0.0)).profile is NORMAL


def test_same_tick_sequence_produces_identical_decisions():
    config = ProfileConfig(
        congested_enter_dwell_s=1.0,
        normal_exit_dwell_s=1.0,
        min_dwell_s=1.0,
    )

    def run():
        machine = ProfileStateMachine(config)
        return [
            tick(machine, 0),
            tick(machine, 1, l515=healthy(8.0)),
            tick(machine, 2, l515=healthy(8.0)),
            tick(machine, 3, d435i=unavailable()),
            tick(machine, 4),
            tick(machine, 5),
        ]

    assert run() == run()


def test_tick_rejects_time_reversal():
    machine = ProfileStateMachine(ProfileConfig())
    tick(machine, 2)

    with pytest.raises(ValueError, match="monotonic"):
        tick(machine, 1)
