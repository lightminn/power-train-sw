"""Pure fail-closed enforcement for versioned ``/section/state`` payloads."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math

from chassis.section_profiles import SectionConfig


_FUTURE_STAMP_TOLERANCE_S = 0.5


def _finite(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{label} must be numeric") from None
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


@dataclass(frozen=True)
class EnforcementDecision:
    v_cap: float | None
    force_hold: bool
    reason: str


class SectionEnforcer:
    """Validate section state and return its final drive constraint."""

    def __init__(self, config, clock):
        if not isinstance(config, SectionConfig):
            raise ValueError("config must be SectionConfig")
        if not callable(clock):
            raise ValueError("clock must be callable")
        local_ttl_s = _finite(config.state_ttl_s, "state_ttl_s")
        if local_ttl_s <= 0.0:
            raise ValueError("state_ttl_s must be positive")
        self.config = config
        self._clock = clock
        self._local_ttl_s = local_ttl_s
        self._state = None
        self._received_s = None
        self._session_id = None
        self._sequence = None

    def feed(self, payload: dict, received_s: float):
        """Accept valid state; invalid or replayed input fails closed."""
        try:
            if not isinstance(payload, Mapping):
                raise ValueError("payload must be a mapping")
            received_s = _finite(received_s, "received_s")
            now_s = _finite(self._clock(), "clock")
            if received_s < 0.0:
                raise ValueError("received_s must be non-negative")

            schema_version = payload["schema_version"]
            if (
                isinstance(schema_version, bool)
                or schema_version != 1
            ):
                raise ValueError("schema_version must be 1")

            session_id = payload["session_id"]
            if not isinstance(session_id, str) or not session_id.strip():
                raise ValueError("session_id must be a non-empty string")
            session_id = session_id.strip()

            sequence = payload["sequence"]
            if (
                isinstance(sequence, bool)
                or not isinstance(sequence, int)
                or sequence < 0
            ):
                raise ValueError("sequence must be a non-negative integer")
            if (
                session_id == self._session_id
                and self._sequence is not None
                and sequence <= self._sequence
            ):
                raise ValueError("sequence must increase within a session")

            stamp_s = _finite(payload["stamp_s"], "stamp_s")
            if stamp_s < 0.0:
                raise ValueError("stamp_s must be non-negative")
            if stamp_s - now_s > _FUTURE_STAMP_TOLERANCE_S:
                raise ValueError("stamp_s is too far in the future")

            ttl_s = _finite(payload["ttl_s"], "ttl_s")
            if ttl_s <= 0.0:
                raise ValueError("ttl_s must be positive")

            enabled = payload["enabled"]
            hold_hint = payload["hold_hint"]
            if not isinstance(enabled, bool):
                raise ValueError("enabled must be boolean")
            if not isinstance(hold_hint, bool):
                raise ValueError("hold_hint must be boolean")

            speed_hint = payload["speed_hint"]
            if speed_hint is not None:
                speed_hint = _finite(speed_hint, "speed_hint")
        except (KeyError, TypeError, ValueError):
            self._state = None
            self._received_s = None
            return False

        self._session_id = session_id
        self._sequence = sequence
        self._received_s = received_s
        self._state = {
            "enabled": enabled,
            "hold_hint": hold_hint,
            "speed_hint": speed_hint,
            "ttl_s": min(ttl_s, self._local_ttl_s),
        }
        return True

    def decide(
        self,
        now_s: float,
        *,
        floor_v_m_s: float = 0.0,
    ) -> EnforcementDecision:
        now_s = _finite(now_s, "now_s")
        floor_v_m_s = _finite(floor_v_m_s, "floor_v_m_s")
        if floor_v_m_s < 0.0:
            raise ValueError("floor_v_m_s must be non-negative")
        if self._state is None or self._received_s is None:
            return EnforcementDecision(None, True, "stale")

        age_s = now_s - self._received_s
        if age_s < 0.0 or age_s > self._state["ttl_s"]:
            return EnforcementDecision(None, True, "stale")
        if not self._state["enabled"]:
            return EnforcementDecision(None, False, "disabled")
        if self._state["hold_hint"]:
            return EnforcementDecision(None, True, "hold_hint")

        speed_hint = self._state["speed_hint"]
        if speed_hint is not None and speed_hint > 0.0:
            if floor_v_m_s > 0.0 and speed_hint < floor_v_m_s:
                return EnforcementDecision(
                    None,
                    True,
                    "hint_below_floor",
                )
            return EnforcementDecision(speed_hint, False, "speed_hint")
        return EnforcementDecision(None, False, "no_speed_hint")
