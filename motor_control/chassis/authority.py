"""Pure single-owner command authority with qualified stop handover.

``chassis_node`` owns this core.  It has no ROS or wheel-predicate imports;
the physical stop decision and its qualification state are injected as
callables.
"""

from dataclasses import dataclass
import math


# Public legacy request strings remain accepted by existing services/tests.
MANUAL = "MANUAL"
AUTO = "AUTO"

# Canonical WP5.2 states.
IDLE = "IDLE"
AUTONOMY = "AUTONOMY"
STOPPING_FOR_HANDOVER = "STOPPING_FOR_HANDOVER"
TELEOP = "TELEOP"
MOTION_HOLD = "MOTION_HOLD"
MODES = (
    IDLE,
    AUTONOMY,
    STOPPING_FOR_HANDOVER,
    TELEOP,
    MOTION_HOLD,
)

MANUAL_SOURCE = "teleop"
AUTO_SOURCE = "auto"

_REQUEST_ALIASES = {
    MANUAL: TELEOP,
    AUTO: AUTONOMY,
    TELEOP: TELEOP,
    AUTONOMY: AUTONOMY,
    IDLE: IDLE,
}
_SOURCE_BY_STATE = {
    TELEOP: MANUAL_SOURCE,
    AUTONOMY: AUTO_SOURCE,
}


@dataclass
class AuthorityConfig:
    stale_s: float = 0.3
    neutral_v: float = 0.02
    neutral_omega: float = 0.05
    handover_timeout_s: float = 2.0


@dataclass(frozen=True)
class Command:
    v: float = 0.0
    omega: float = 0.0
    ok: bool = False
    reason: str = ""


@dataclass(frozen=True)
class TransitionResult:
    accepted: bool
    reason: str
    state: str


class CommandAuthority:
    """Select one source and stop physically before a moving handover."""

    def __init__(
        self,
        cfg: AuthorityConfig = None,
        *,
        wheel_stopped=None,
        wheel_stop_qualified=None,
    ):
        self.cfg = cfg or AuthorityConfig()
        if (
            not math.isfinite(self.cfg.handover_timeout_s)
            or self.cfg.handover_timeout_s <= 0.0
        ):
            raise ValueError("handover_timeout_s must be finite and positive")
        self.mode = IDLE
        self._src = {}
        self._armed = False
        self._wheel_stopped = wheel_stopped
        self._wheel_stop_qualified = wheel_stop_qualified
        self._pending_mode = None
        self._stopping_started_s = None
        self._stopping_zero_emitted = False
        self._last_output = (0.0, 0.0)
        self._last_select_t = 0.0
        self.last_transition_reason = "initialized in IDLE"

    def submit(self, source: str, v: float, omega: float, t: float) -> None:
        self._src[source] = (float(v), float(omega), float(t))

    def _transition_result(self, accepted, reason):
        self.last_transition_reason = reason
        return TransitionResult(bool(accepted), reason, self.mode)

    @staticmethod
    def _call_flag(value):
        if callable(value):
            return bool(value())
        return bool(value)

    def _qualified_stop_available(self):
        if self._wheel_stop_qualified is not None:
            return self._call_flag(self._wheel_stop_qualified)
        if self._wheel_stopped is not None:
            return self._call_flag(
                getattr(self._wheel_stopped, "qualified", False)
            )
        return False

    def _output_is_nonzero(self):
        return self._last_output != (0.0, 0.0)

    def request_mode(self, mode: str, t: float = None) -> TransitionResult:
        target = _REQUEST_ALIASES.get(mode)
        if target is None:
            return self._transition_result(False, "invalid mode: %s" % mode)

        if self.mode == MOTION_HOLD:
            return self._transition_result(
                False,
                "MOTION_HOLD requires clear_hold() acknowledgement",
            )

        if target == IDLE:
            self.mode = IDLE
            self._pending_mode = None
            self._stopping_started_s = None
            self._stopping_zero_emitted = False
            self._armed = False
            self._last_output = (0.0, 0.0)
            return self._transition_result(True, "mode=IDLE")

        if self.mode == STOPPING_FOR_HANDOVER:
            if target == self._pending_mode:
                return self._transition_result(
                    True,
                    "handover already stopping for %s" % target,
                )
            return self._transition_result(
                False,
                "handover already stopping for %s" % self._pending_mode,
            )

        if target == self.mode:
            return self._transition_result(True, "mode already %s" % target)

        if self.mode in _SOURCE_BY_STATE and self._output_is_nonzero():
            try:
                qualified = self._qualified_stop_available()
            except Exception as exc:
                return self._transition_result(
                    False,
                    "wheel-stop qualification error: %s" % exc,
                )
            if not qualified:
                return self._transition_result(
                    False,
                    "wheel-stop predicate unqualified",
                )
            when = self._last_select_t if t is None else float(t)
            if not math.isfinite(when):
                return self._transition_result(False, "invalid request time")
            self.mode = STOPPING_FOR_HANDOVER
            self._pending_mode = target
            self._stopping_started_s = when
            self._stopping_zero_emitted = False
            self._armed = False
            return self._transition_result(
                True,
                "stopping for handover to %s" % target,
            )

        # IDLE→source and already-zero source→source retain the transitional
        # neutral gate, without requiring a physical-stop predicate.
        self.mode = target
        self._armed = False
        return self._transition_result(
            True,
            "mode=%s; neutral confirmation required" % target,
        )

    def set_mode(self, mode: str, t: float = None) -> bool:
        """Backward-compatible boolean wrapper around ``request_mode``."""
        return self.request_mode(mode, t=t).accepted

    def clear_hold(self) -> bool:
        """Acknowledge a hold and return to IDLE; never resume a source."""
        if self.mode != MOTION_HOLD:
            return False
        self.mode = IDLE
        self._pending_mode = None
        self._stopping_started_s = None
        self._stopping_zero_emitted = False
        self._armed = False
        self._last_output = (0.0, 0.0)
        self.last_transition_reason = "MOTION_HOLD cleared to IDLE"
        return True

    def _zero(self, reason):
        self._last_output = (0.0, 0.0)
        return Command(0.0, 0.0, True, reason)

    def _select_stopping(self, t):
        elapsed = t - self._stopping_started_s
        if elapsed < 0.0:
            self.mode = MOTION_HOLD
            self._pending_mode = None
            self._armed = False
            return self._zero("clock rollback during handover → MOTION_HOLD")
        if elapsed + 1e-12 >= self.cfg.handover_timeout_s:
            self.mode = MOTION_HOLD
            self._pending_mode = None
            self._armed = False
            return self._zero("handover timeout → MOTION_HOLD")

        # Always emit at least one explicit zero after accepting a handover;
        # a predicate result latched before that request must not complete it.
        if not self._stopping_zero_emitted:
            self._stopping_zero_emitted = True
            return self._zero("STOPPING_FOR_HANDOVER — zero commanded")

        try:
            stopped = self._call_flag(self._wheel_stopped)
        except Exception as exc:
            self.mode = MOTION_HOLD
            self._pending_mode = None
            self._armed = False
            return self._zero(
                "wheel-stop callback error → MOTION_HOLD: %s" % exc
            )
        if not stopped:
            return self._zero("STOPPING_FOR_HANDOVER — wheel stop pending")

        target = self._pending_mode
        self.mode = target
        self._pending_mode = None
        self._stopping_started_s = None
        self._stopping_zero_emitted = False
        self._armed = False
        return self._zero("wheel stop confirmed — mode=%s" % target)

    def select(self, t: float) -> Command:
        t = float(t)
        self._last_select_t = t
        if self.mode == STOPPING_FOR_HANDOVER:
            return self._select_stopping(t)
        if self.mode == MOTION_HOLD:
            return self._zero("MOTION_HOLD — clear_hold() required")
        if self.mode == IDLE:
            return Command(reason="IDLE — 아무도 조종하지 않음")

        name = _SOURCE_BY_STATE[self.mode]
        entry = self._src.get(name)
        if entry is None:
            return Command(reason=f"{name} 명령 없음")

        v, omega, ts = entry
        age = t - ts
        if age < 0.0:
            self.mode = MOTION_HOLD
            self._armed = False
            return Command(reason=f"{name} future timestamp → MOTION_HOLD")
        if age > self.cfg.stale_s:
            self.mode = MOTION_HOLD
            self._armed = False
            return Command(reason=f"{name} stale ({age:.2f}s) → MOTION_HOLD")

        if not self._armed:
            if self._is_neutral(v, omega):
                self._armed = True
                return self._zero(f"{name} 중립 확인 — 권한 인계")
            return Command(
                reason=f"{name} 중립 대기 (v={v:+.2f} ω={omega:+.2f})"
            )

        self._last_output = (v, omega)
        return Command(v, omega, True, name)

    def _is_neutral(self, v, omega):
        return (
            abs(v) <= self.cfg.neutral_v
            and abs(omega) <= self.cfg.neutral_omega
        )
