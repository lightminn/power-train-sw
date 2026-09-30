"""Jog lifetime: deadman TTL, latest-value coalescing, surviving stop intents."""
from __future__ import annotations

import pytest

from operator_console.arm_ops import contract as K
from operator_console.arm_ops import jog as J


AXIS = J.arm_axis_channel(2)
TOOL = J.tool_channel("left", "TOOL-D1", 8)


def registry(ttl_s: float = 1.0, refresh_interval_s: float = 0.2) -> J.JogRegistry:
    return J.JogRegistry(ttl_s=ttl_s, refresh_interval_s=refresh_interval_s)


# --- construction guards ----------------------------------------------------
def test_a_refresh_interval_at_or_above_the_ttl_is_rejected():
    # Refreshing no more often than the deadline means every hold dies.
    with pytest.raises(ValueError):
        J.JogRegistry(ttl_s=0.5, refresh_interval_s=0.5)
    with pytest.raises(ValueError):
        J.JogRegistry(ttl_s=0.5, refresh_interval_s=0.0)


def test_the_shipped_defaults_leave_room_for_a_missed_refresh():
    assert J.DEFAULT_REFRESH_INTERVAL_S * 2 < J.DEFAULT_JOG_TTL_S


# --- deadman ----------------------------------------------------------------
def test_every_jog_request_carries_a_future_deadline():
    reg = registry(ttl_s=0.6)
    reg.press(AXIS, direction=1, now=10.0)

    request, = reg.due(10.0)

    assert request.action == K.ACTION_JOG
    assert request.params["expires_at_s"] == pytest.approx(10.6)
    assert request.params["expires_at_s"] > 10.0


def test_a_hold_that_is_not_refreshed_expires_into_a_stop():
    reg = registry(ttl_s=0.5)
    reg.press(AXIS, direction=1, now=10.0)

    assert reg.expire(10.4) == ()
    stops = reg.expire(10.6)

    assert [stop.reason for stop in stops] == [K.STOP_REASON_EXPIRED]
    assert reg.is_active(AXIS) is False


def test_refreshing_keeps_a_hold_alive_past_the_original_deadline():
    reg = registry(ttl_s=0.5)
    reg.press(AXIS, direction=1, now=10.0)
    reg.press(AXIS, direction=1, now=10.4)

    assert reg.expire(10.6) == ()
    assert reg.is_active(AXIS) is True


# --- latest value, not a queue ----------------------------------------------
def test_pressing_again_replaces_the_value_instead_of_stacking():
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)
    reg.press(AXIS, direction=-1, now=10.05)

    requests = reg.due(10.05)

    assert len(requests) == 1
    assert requests[0].params["direction"] == -1
    assert len(reg.active_channels) == 1


def test_only_one_request_is_outstanding_at_a_time():
    # The ops broker serialises mutations; a second in-flight jog would come
    # back "busy: mutation in flight".
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)

    first = reg.due(10.0)
    assert len(first) == 1

    reg.press(AXIS, direction=-1, now=10.05)
    assert reg.due(10.05) == ()          # still outstanding

    reg.settle(first[0])
    second = reg.due(10.06)
    assert len(second) == 1
    assert second[0].params["direction"] == -1


def test_at_most_one_refresh_per_pump_even_with_several_holds():
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)
    reg.press(TOOL, direction=1, now=10.0)

    assert len(reg.due(10.0)) == 1


def test_a_settled_request_frees_the_slot_regardless_of_outcome():
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)
    request, = reg.due(10.0)

    # OUTCOME_UNKNOWN settles too: one lost response must not wedge the hold.
    reg.settle(request)

    reg.press(AXIS, direction=1, now=10.3)
    assert len(reg.due(10.3)) == 1


def test_a_quiet_hold_is_refreshed_on_the_interval_not_every_pump():
    reg = registry(ttl_s=1.0, refresh_interval_s=0.2)
    reg.press(AXIS, direction=1, now=10.0)
    first, = reg.due(10.0)
    reg.settle(first)

    assert reg.due(10.1) == ()           # below the refresh interval
    assert len(reg.due(10.25)) == 1


# --- stops survive ----------------------------------------------------------
def test_an_explicit_release_produces_a_pending_stop():
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)

    stop = reg.release(AXIS, now=10.1)

    assert stop is not None
    assert stop.reason == K.STOP_REASON_EXPLICIT
    assert reg.pending_stops == (stop,)
    assert reg.is_active(AXIS) is False


def test_a_pending_stop_stays_pending_until_it_is_settled():
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)
    reg.release(AXIS, now=10.1)

    # Two pumps that never deliver: the intent must not evaporate.
    assert len(reg.pending_stops) == 1
    assert len(reg.pending_stops) == 1

    request = reg.pending_stops[0].request()
    reg.settle(request)
    assert reg.pending_stops == ()


def test_releasing_a_channel_that_is_not_held_produces_nothing():
    assert registry().release(AXIS, now=1.0) is None


@pytest.mark.parametrize("reason", [
    K.STOP_REASON_TOOL_CHANGED,
    K.STOP_REASON_AUTHORITY_LOST,
    K.STOP_REASON_LINK_LOST,
    K.STOP_REASON_FOCUS_LOST,
    K.STOP_REASON_CALIBRATION,
])
def test_every_external_event_can_end_every_hold_with_its_reason(reason):
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)
    reg.press(TOOL, direction=-1, now=10.0)

    stops = reg.release_all(now=10.2, reason=reason)

    assert len(stops) == 2
    assert {stop.reason for stop in stops} == {reason}
    assert reg.active_channels == ()
    assert len(reg.pending_stops) == 2


def test_an_unknown_stop_reason_is_rejected():
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)
    with pytest.raises(ValueError):
        reg.release(AXIS, now=10.1, reason="whatever")


def test_the_first_reason_wins_when_a_stop_is_queued_twice():
    # An explicit release followed by a link drop is still an explicit release:
    # the first reason is the one that actually ended the motion.
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)
    reg.release(AXIS, now=10.1, reason=K.STOP_REASON_EXPLICIT)
    reg.press(AXIS, direction=1, now=10.2)
    reg.release(AXIS, now=10.3, reason=K.STOP_REASON_LINK_LOST)

    assert len(reg.pending_stops) == 1
    assert reg.pending_stops[0].reason == K.STOP_REASON_EXPLICIT


def test_stop_requests_carry_the_reason_and_the_subject():
    reg = registry()
    reg.press(TOOL, direction=1, now=10.0)
    reg.release_all(now=10.1, reason=K.STOP_REASON_TOOL_CHANGED)

    request = reg.pending_stops[0].request()

    assert request.action == K.ACTION_TOOL_JOG_STOP
    assert request.params["reason"] == K.STOP_REASON_TOOL_CHANGED
    assert request.params["tool_id"] == "TOOL-D1"
    assert request.params["tool_generation"] == 8
    assert request.is_stop is True


def test_delivery_attempts_are_counted_so_a_stuck_stop_is_visible():
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)
    reg.release(AXIS, now=10.1)

    reg.record_stop_attempt(AXIS)
    reg.record_stop_attempt(AXIS)

    assert reg.pending_stops[0].attempts == 2


def test_releasing_outstanding_slots_is_not_a_stop():
    """Freeing in-flight bookkeeping must never be mistaken for ending a hold."""
    reg = registry()
    reg.press(AXIS, direction=1, now=10.0)
    reg.due(10.0)

    reg.release_outstanding()

    assert reg.is_active(AXIS) is True
    assert reg.pending_stops == ()
    assert len(reg.due(10.3)) == 1


# --- channels ---------------------------------------------------------------
def test_channels_are_equal_when_their_subject_is_equal():
    assert J.arm_axis_channel(2) == J.arm_axis_channel(2)
    assert J.arm_axis_channel(2) != J.arm_axis_channel(3)
    assert J.tool_channel("left", "T", 1) != J.tool_channel("left", "T", 2)


def test_channels_for_tool_finds_everything_bound_to_one_tool():
    reg = registry()
    reg.press(J.tool_channel("left", "TOOL-D1", 8), direction=1, now=10.0)
    reg.press(J.tool_channel("right", "TOOL-D1", 8), direction=1, now=10.0)
    reg.press(AXIS, direction=1, now=10.0)

    bound = J.channels_for_tool(reg.active_channels, "TOOL-D1")

    assert len(bound) == 2


def test_an_invalid_direction_is_rejected_at_the_press():
    with pytest.raises(ValueError):
        registry().press(AXIS, direction=0, now=1.0)
