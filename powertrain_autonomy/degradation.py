"""Pure WP9 degradation state machine.

``depth_quality`` is the invalid/dropout ratio: larger values are worse and
``None`` means that no usable depth-quality observation is available.  One
entry into ``HOLD_RECOVERY`` consumes one recovery attempt.  Time and distance
budgets span the whole recovery episode until a stable return to ``NORMAL`` or
an explicit operator reset from ``HANDOVER_WAIT``.
"""

from dataclasses import dataclass
from enum import Enum
import math


class DegradationStage(str, Enum):
    NORMAL = "NORMAL"
    SLOWDOWN = "SLOWDOWN"
    HOLD_RECOVERY = "HOLD_RECOVERY"
    HANDOVER_WAIT = "HANDOVER_WAIT"


@dataclass(frozen=True)
class DegradationConfig:
    slowdown_scale: float = 0.5
    enter_depth_dropout: float = 0.35
    exit_depth_dropout: float = 0.20
    stuck_enter_ticks: int = 5
    recovery_attempts_max: int = 3
    recovery_time_budget_s: float = 8.0
    recovery_distance_budget_m: float = 1.5

    def __post_init__(self):
        ratios = (
            self.slowdown_scale,
            self.enter_depth_dropout,
            self.exit_depth_dropout,
        )
        if any(
            isinstance(value, bool)
            or not math.isfinite(float(value))
            or not 0.0 <= float(value) <= 1.0
            for value in ratios
        ):
            raise ValueError("degradation scales and ratios must be within [0, 1]")
        if self.exit_depth_dropout >= self.enter_depth_dropout:
            raise ValueError("exit_depth_dropout must be below enter_depth_dropout")
        for name, value in (
            ("stuck_enter_ticks", self.stuck_enter_ticks),
            ("recovery_attempts_max", self.recovery_attempts_max),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        for name, value in (
            ("recovery_time_budget_s", self.recovery_time_budget_s),
            ("recovery_distance_budget_m", self.recovery_distance_budget_m),
        ):
            if (
                isinstance(value, bool)
                or not math.isfinite(float(value))
                or float(value) <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class DegradationOutput:
    stage: DegradationStage
    speed_scale: float
    request_hold: bool
    handover_wait: bool
    reasons: tuple


class DegradationFsm:
    def __init__(self, config=None, *, clock):
        if config is not None and not isinstance(config, DegradationConfig):
            raise ValueError("config must be DegradationConfig")
        if not callable(clock):
            raise ValueError("clock must be callable")
        self.config = config or DegradationConfig()
        self._clock = clock
        self._stage = DegradationStage.NORMAL
        self._depth_degraded = False
        self._stuck_ticks = 0
        self._recovery_attempts = 0
        self._recovery_started_s = None
        self._recovery_started_m = None
        self._reasons = []

    def update(
        self,
        *,
        depth_quality: float | None,
        slip_candidate: bool,
        stuck_candidate: bool,
        traveled_m: float,
        now_s: float,
    ) -> DegradationOutput:
        if self._stage is DegradationStage.HANDOVER_WAIT:
            return self._output()

        now_s = self._finite(
            self._clock() if now_s is None else now_s,
            "now_s",
        )
        traveled_m = self._finite(traveled_m, "traveled_m")
        if traveled_m < 0.0:
            raise ValueError("traveled_m must be non-negative")
        if depth_quality is None:
            depth_unavailable = True
            self._depth_degraded = True
            self._add_reason("depth_dropout")
            self._add_reason("depth_unavailable")
        else:
            depth_unavailable = False
            depth_quality = self._finite(depth_quality, "depth_quality")
            if not 0.0 <= depth_quality <= 1.0:
                raise ValueError("depth_quality must be within [0, 1]")
            if depth_quality >= self.config.enter_depth_dropout:
                self._depth_degraded = True
                self._add_reason("depth_dropout")
            elif depth_quality <= self.config.exit_depth_dropout:
                self._depth_degraded = False

        if slip_candidate:
            self._add_reason("slip_candidate")
        if stuck_candidate:
            self._stuck_ticks += 1
            self._add_reason("stuck_candidate")
        else:
            self._stuck_ticks = 0

        if self._stage is DegradationStage.NORMAL:
            if self._depth_degraded or slip_candidate:
                self._stage = DegradationStage.SLOWDOWN
            return self._output()

        if self._stage is DegradationStage.SLOWDOWN:
            if (
                depth_unavailable
                or self._stuck_ticks >= self.config.stuck_enter_ticks
            ):
                self._enter_recovery(now_s, traveled_m)
            elif (
                not self._depth_degraded
                and not slip_candidate
                and not stuck_candidate
            ):
                self._reset_to_normal()
            return self._output()

        self._apply_running_budgets(now_s, traveled_m)
        if self._stage is DegradationStage.HANDOVER_WAIT:
            return self._output()
        if (
            not self._depth_degraded
            and not slip_candidate
            and not stuck_candidate
        ):
            self._stage = DegradationStage.SLOWDOWN
        return self._output()

    def operator_reset(self):
        if self._stage is DegradationStage.HANDOVER_WAIT:
            self._reset_to_normal()

    @staticmethod
    def _finite(value, label):
        try:
            result = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{label} must be numeric") from None
        if not math.isfinite(result):
            raise ValueError(f"{label} must be finite")
        return result

    def _enter_recovery(self, now_s, traveled_m):
        if self._recovery_started_s is None:
            self._recovery_started_s = now_s
            self._recovery_started_m = traveled_m
        if self._recovery_attempts >= self.config.recovery_attempts_max:
            self._add_reason("recovery_attempt_budget_exhausted")
            self._stage = DegradationStage.HANDOVER_WAIT
            return
        self._recovery_attempts += 1
        self._stage = DegradationStage.HOLD_RECOVERY

    def _apply_running_budgets(self, now_s, traveled_m):
        elapsed_s = now_s - self._recovery_started_s
        distance_m = max(0.0, traveled_m - self._recovery_started_m)
        exhausted = False
        if elapsed_s >= self.config.recovery_time_budget_s:
            self._add_reason("recovery_time_budget_exhausted")
            exhausted = True
        if distance_m >= self.config.recovery_distance_budget_m:
            self._add_reason("recovery_distance_budget_exhausted")
            exhausted = True
        if exhausted:
            self._stage = DegradationStage.HANDOVER_WAIT

    def _reset_to_normal(self):
        self._stage = DegradationStage.NORMAL
        self._depth_degraded = False
        self._stuck_ticks = 0
        self._recovery_attempts = 0
        self._recovery_started_s = None
        self._recovery_started_m = None
        self._reasons = []

    def _add_reason(self, reason):
        if reason not in self._reasons:
            self._reasons.append(reason)

    def _output(self):
        if self._stage is DegradationStage.NORMAL:
            speed_scale = 1.0
        elif self._stage is DegradationStage.SLOWDOWN:
            speed_scale = self.config.slowdown_scale
        else:
            speed_scale = 0.0
        return DegradationOutput(
            stage=self._stage,
            speed_scale=speed_scale,
            request_hold=self._stage in (
                DegradationStage.HOLD_RECOVERY,
                DegradationStage.HANDOVER_WAIT,
            ),
            handover_wait=self._stage is DegradationStage.HANDOVER_WAIT,
            reasons=tuple(self._reasons),
        )
