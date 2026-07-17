"""Pure, pygame-free arbitration for laptop DualSense feedback."""

from collections import deque
from dataclasses import dataclass
import math
import threading


PRIORITY = ("estop", "authority", "link_loss", "proximity", "bypass")
STALE_S = 0.5


@dataclass(frozen=True)
class Rumble:
    low: float
    high: float
    duration_ms: int


_ESTOP = Rumble(low=1.0, high=1.0, duration_ms=250)
_AUTHORITY = Rumble(low=0.2, high=0.65, duration_ms=140)
_LINK_LOSS = Rumble(low=0.55, high=0.0, duration_ms=180)
_BYPASS = Rumble(low=0.12, high=0.0, duration_ms=80)
_EVENT_PATTERNS = {
    "chord_progress": Rumble(low=0.0, high=0.35, duration_ms=100),
    "ack": Rumble(low=0.15, high=0.8, duration_ms=160),
    "nack": Rumble(low=0.8, high=0.8, duration_ms=240),
}


class HapticArbiter:
    """Select at most one haptic pattern from the latest ops state."""

    def __init__(self, *, clock):
        self._clock = clock
        self._state = None
        self._received_s = None
        self._authority = None
        self._authority_transition = False
        self._events = deque(maxlen=8)
        self._lock = threading.Lock()

    def feed_ops_state(self, state: dict, received_s: float):
        """Record a locally timestamped snapshot without trusting its shape."""
        snapshot = dict(state) if isinstance(state, dict) else {}
        try:
            received = float(received_s)
        except (TypeError, ValueError, OverflowError):
            received = float("nan")
        if not math.isfinite(received):
            received = None

        with self._lock:
            authority = snapshot.get("authority_mode")
            if authority is not None:
                authority = str(authority).strip().upper() or None
            if (
                self._authority is not None
                and authority is not None
                and authority != self._authority
            ):
                self._authority_transition = True
            self._authority = authority
            self._state = snapshot
            self._received_s = received

    def feed_event(self, kind: str, detail: str = ""):
        """Queue one bounded transient cue; unknown event kinds are ignored."""
        del detail
        pattern = _EVENT_PATTERNS.get(str(kind).strip().lower())
        if pattern is not None:
            with self._lock:
                self._events.append(pattern)

    def _is_stale(self):
        if self._state is None or self._received_s is None:
            return True
        try:
            age_s = float(self._clock()) - self._received_s
        except (TypeError, ValueError, OverflowError):
            return True
        return not math.isfinite(age_s) or age_s > STALE_S

    def _estop_active(self):
        if self._state is None:
            return False
        return bool(
            self._state.get("estop_latched", False)
            or self._state.get("active_estop_sources")
        )

    def _proximity(self):
        if self._state is None:
            return None
        distance = self._state.get("safety_distance_mm")
        if distance is None or isinstance(distance, bool):
            return None
        try:
            distance = float(distance)
        except (TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(distance) or distance >= 400.0:
            return None
        strength = max(0.0, min(1.0, (400.0 - distance) / 400.0))
        return Rumble(
            low=strength,
            high=0.5 * strength,
            duration_ms=100,
        )

    def _clear_transients(self):
        self._authority_transition = False
        self._events.clear()

    def decide(self) -> Rumble | None:
        """Return the highest-priority current pattern, if any."""
        with self._lock:
            # Stale data cannot authorize a positive or historical cue.
            if self._is_stale():
                self._clear_transients()
                return _LINK_LOSS

            if self._estop_active():
                self._clear_transients()
                return _ESTOP

            if self._authority_transition:
                self._authority_transition = False
                return _AUTHORITY

            if self._events:
                return self._events.popleft()

            proximity = self._proximity()
            if proximity is not None:
                return proximity

            if self._state is not None and bool(
                self._state.get("assist_bypass", False)
            ):
                return _BYPASS
            return None

    def lightbar(self) -> tuple[int, int, int] | None:
        """Return the Tier-2 state color, or neutral for missing/stale state."""
        with self._lock:
            if self._is_stale() or self._state is None:
                return None
            if self._estop_active():
                return (255, 0, 0)
            authority = str(
                self._state.get("authority_mode") or ""
            ).upper()
            gateway = str(
                self._state.get("gateway_state") or ""
            ).upper()
            if authority == "MOTION_HOLD" or gateway == "MOTION_HOLD":
                return (255, 191, 0)
            if authority == "AUTONOMY":
                return (0, 0, 255)
            if authority == "TELEOP":
                return (255, 255, 255)
            return None
