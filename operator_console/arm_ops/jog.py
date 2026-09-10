"""Press-and-hold jog lifetime: deadman TTL, latest-value only, surviving stop.

Three properties this file exists to guarantee, none of which the UI can
provide on its own:

**A jog expires by itself.**  Every jog request carries ``expires_at_s``.  If
the console stops refreshing — the process wedged, the link died, the operator's
laptop went to sleep — the backend must end the motion on its own.  The
console's explicit stop is the fast path, not the safety net.

**A jog never queues up.**  The ops broker serialises mutations and answers
``busy: mutation in flight`` to anything sent while another is outstanding, so
a 20 Hz hold sent naively would either be rejected or pile into a backlog that
outlives the press.  A hold is therefore *latest-value*: at most one jog
request is outstanding at a time, and a newer value replaces an unsent older
one instead of following it.

**A stop intent outlives its transport.**  When a hold is ended by something
other than the operator — tool change, lost grant, dead link, the tab going
away — the stop is recorded as *pending* and stays pending until a transport
confirms it was delivered.  A stop that could not be sent is not a stop that
did not need sending, so it is retried on reconnect rather than dropped.

Timing values here are console-side defaults and explicitly provisional: the
real refresh rate has to come from measured ops round-trip and rate limits
(``BACKEND_REQUIREMENTS['jog_rate_and_serialization']``).  They are constructor
arguments so a measurement can replace them without touching this logic.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Iterable, Mapping

from . import contract as K


# Provisional. A hold must be refreshed at least this often or the backend
# stops it.  Long enough to survive one missed round trip, short enough that an
# unnoticed freeze does not keep a joint moving.
DEFAULT_JOG_TTL_S = 0.6
# Provisional.  Must stay comfortably below the TTL so one lost refresh does
# not end an intended hold.
DEFAULT_REFRESH_INTERVAL_S = 0.2


@dataclass(frozen=True)
class JogChannel:
    """Identifies one hold: which jog action, on which subject.

    Two holds with the same channel are the same hold.  Different channels can
    be live at once (an arm axis and a gripper, say), and each expires and
    stops independently.
    """

    action: str
    stop_action: str
    # Identifying params only — no lifetime fields.  For an arm jog this is
    # ``{"axis": 2}``; for a tool jog the target plus the tool identity it was
    # issued against.
    subject: tuple[tuple[str, Any], ...] = ()

    @classmethod
    def make(cls, action: str, stop_action: str, subject: Mapping[str, Any]) -> "JogChannel":
        return cls(action, stop_action, tuple(sorted(subject.items())))

    @property
    def subject_params(self) -> dict[str, Any]:
        return dict(self.subject)


@dataclass(frozen=True)
class PendingStop:
    """A stop that has been decided but not yet confirmed delivered."""

    channel: JogChannel
    reason: str
    sequence: int
    created_s: float
    attempts: int = 0

    def request(self) -> K.ArmOpsRequest:
        params = dict(self.channel.subject_params)
        params["reason"] = self.reason
        params["sequence"] = self.sequence
        return K.ArmOpsRequest(
            action=self.channel.stop_action,
            params=params,
            issued_s=self.created_s,
            intent=f"jog stop ({K.STOP_REASON_KOREAN.get(self.reason, self.reason)})",
        )


@dataclass
class _ActiveJog:
    channel: JogChannel
    direction: int
    speed_level: int | None
    started_s: float
    refreshed_s: float
    sequence: int
    # Set when a value has been produced but not yet settled by the transport.
    outstanding: bool = False
    # The newest value the operator produced while a request was outstanding.
    dirty: bool = False


class JogRegistry:
    """All live holds plus the stop intents that must still be delivered.

    Pure and clock-injected: every method takes ``now`` rather than reading a
    clock, so expiry, coalescing and retry are all directly testable.
    """

    def __init__(
        self,
        *,
        ttl_s: float = DEFAULT_JOG_TTL_S,
        refresh_interval_s: float = DEFAULT_REFRESH_INTERVAL_S,
    ) -> None:
        if not 0.0 < refresh_interval_s < ttl_s:
            raise ValueError("refresh interval must be positive and below the TTL")
        self._ttl_s = float(ttl_s)
        self._refresh_interval_s = float(refresh_interval_s)
        self._active: dict[JogChannel, _ActiveJog] = {}
        self._pending_stops: list[PendingStop] = []
        self._sequence = 0

    # --- introspection --------------------------------------------------
    @property
    def ttl_s(self) -> float:
        return self._ttl_s

    @property
    def active_channels(self) -> tuple[JogChannel, ...]:
        return tuple(self._active)

    @property
    def pending_stops(self) -> tuple[PendingStop, ...]:
        return tuple(self._pending_stops)

    def is_active(self, channel: JogChannel) -> bool:
        return channel in self._active

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence

    # --- operator gestures ----------------------------------------------
    def press(
        self,
        channel: JogChannel,
        *,
        direction: int,
        now: float,
        speed_level: int | None = None,
    ) -> None:
        """Start a hold, or update an existing one with the newest value.

        Pressing an already-held channel does not stack: it replaces the value
        and refreshes the deadline.  That is the whole latest-value contract.
        """
        if direction not in K.DIRECTIONS:
            raise ValueError(f"invalid jog direction: {direction!r}")
        existing = self._active.get(channel)
        if existing is None:
            self._active[channel] = _ActiveJog(
                channel=channel,
                direction=direction,
                speed_level=speed_level,
                started_s=float(now),
                refreshed_s=float(now),
                sequence=self._next_sequence(),
                dirty=True,
            )
            return
        existing.direction = direction
        existing.speed_level = speed_level
        existing.refreshed_s = float(now)
        existing.dirty = True

    def release(
        self,
        channel: JogChannel,
        *,
        now: float,
        reason: str = K.STOP_REASON_EXPLICIT,
    ) -> PendingStop | None:
        """End one hold.  Returns the stop intent, or ``None`` if not held.

        The stop is queued as pending rather than returned-and-forgotten: the
        caller may fail to deliver it, and this registry is what remembers.
        """
        active = self._active.pop(channel, None)
        if active is None:
            return None
        return self._queue_stop(channel, reason, now)

    def release_all(
        self, *, now: float, reason: str,
    ) -> tuple[PendingStop, ...]:
        """End every hold at once — tool change, lost grant, dead link, blur."""
        stops = []
        for channel in tuple(self._active):
            stop = self.release(channel, now=now, reason=reason)
            if stop is not None:
                stops.append(stop)
        return tuple(stops)

    def _queue_stop(
        self, channel: JogChannel, reason: str, now: float,
    ) -> PendingStop:
        if reason not in K.STOP_REASONS:
            raise ValueError(f"unknown stop reason: {reason!r}")
        # Collapse repeats for the same channel: one stop is one stop.  The
        # first reason is kept, because it is the one that actually ended the
        # motion; a later "link_lost" must not rewrite an explicit release.
        for existing in self._pending_stops:
            if existing.channel == channel:
                return existing
        stop = PendingStop(
            channel=channel,
            reason=reason,
            sequence=self._next_sequence(),
            created_s=float(now),
        )
        self._pending_stops.append(stop)
        return stop

    # --- expiry ----------------------------------------------------------
    def expire(self, now: float) -> tuple[PendingStop, ...]:
        """End holds whose deadline has passed.

        This is the console mirroring the deadman it asked the backend to
        honour.  Both sides stopping is correct; only one side stopping is the
        bug this guards against.
        """
        expired = [
            channel for channel, active in self._active.items()
            if float(now) - active.refreshed_s > self._ttl_s
        ]
        stops = []
        for channel in expired:
            self._active.pop(channel, None)
            stops.append(self._queue_stop(channel, K.STOP_REASON_EXPIRED, now))
        return tuple(stops)

    # --- outbound work ----------------------------------------------------
    def due(self, now: float) -> tuple[K.ArmOpsRequest, ...]:
        """The jog *refresh* that should go out now, if any.

        Stops are not included: the caller sends :attr:`pending_stops` first,
        because with a serialising broker letting a refresh take the single
        available slot would delay the very command that ends the motion.  At
        most one refresh is produced per call, and only when nothing is
        outstanding.
        """
        requests: list[K.ArmOpsRequest] = []
        for active in self._active.values():
            if active.outstanding:
                continue
            if not active.dirty and (
                float(now) - active.refreshed_s < self._refresh_interval_s
            ):
                continue
            requests.append(self._jog_request(active, now))
            active.dirty = False
            active.outstanding = True
            break                       # one outstanding mutation at a time
        return tuple(requests)

    def _jog_request(self, active: _ActiveJog, now: float) -> K.ArmOpsRequest:
        params = dict(active.channel.subject_params)
        params["direction"] = active.direction
        if active.speed_level is not None:
            params["speed_level"] = active.speed_level
        params["expires_at_s"] = float(now) + self._ttl_s
        params["sequence"] = active.sequence
        return K.ArmOpsRequest(
            action=active.channel.action,
            params=params,
            issued_s=float(now),
            intent="jog refresh",
        )

    def settle(self, request: K.ArmOpsRequest) -> None:
        """Mark one produced request as no longer outstanding.

        Called when the transport reports a final outcome — success, rejection
        or OUTCOME_UNKNOWN alike.  An unknown outcome must free the slot too,
        or a single lost response would freeze the hold forever.
        """
        if request.action in K.STOP_ACTIONS:
            self._settle_stop(request)
            return
        for active in self._active.values():
            if active.channel.action == request.action and all(
                request.params.get(key) == value
                for key, value in active.channel.subject
            ):
                active.outstanding = False
                return

    def _settle_stop(self, request: K.ArmOpsRequest) -> None:
        remaining = []
        for stop in self._pending_stops:
            matches = stop.channel.stop_action == request.action and all(
                request.params.get(key) == value
                for key, value in stop.channel.subject
            )
            if not matches:
                remaining.append(stop)
        self._pending_stops = remaining

    def record_stop_attempt(self, channel: JogChannel) -> None:
        """Count a delivery attempt so a permanently failing stop is visible."""
        self._pending_stops = [
            replace(stop, attempts=stop.attempts + 1)
            if stop.channel == channel else stop
            for stop in self._pending_stops
        ]

    def release_outstanding(self) -> None:
        """Forget which requests were in flight, without ending any hold.

        Used when the connection drops: the previous requests will never be
        answered, so their slots must be freed.  The holds themselves are ended
        separately (by ``release_all(reason=link_lost)``) — this method alone
        must never be mistaken for a stop.
        """
        for active in self._active.values():
            active.outstanding = False


def arm_axis_channel(axis: int) -> JogChannel:
    return JogChannel.make(
        K.ACTION_JOG, K.ACTION_JOG_STOP, {"axis": int(axis)},
    )


def tool_channel(target: str, tool_id: str, tool_generation: int) -> JogChannel:
    return JogChannel.make(
        K.ACTION_TOOL_JOG, K.ACTION_TOOL_JOG_STOP,
        {
            "target": str(target),
            "tool_id": str(tool_id),
            "tool_generation": int(tool_generation),
        },
    )


def calibration_channel(
    target: str, tool_id: str, tool_generation: int,
) -> JogChannel:
    return JogChannel.make(
        K.ACTION_CALIBRATION_JOG, K.ACTION_CALIBRATION_JOG_STOP,
        {
            "target": str(target),
            "tool_id": str(tool_id),
            "tool_generation": int(tool_generation),
        },
    )


def channels_for_tool(
    channels: Iterable[JogChannel], tool_id: str,
) -> tuple[JogChannel, ...]:
    """Every channel bound to one tool — what a tool change has to end."""
    return tuple(
        channel for channel in channels
        if dict(channel.subject).get("tool_id") == tool_id
    )
