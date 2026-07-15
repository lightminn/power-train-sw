"""Pure qualified per-wheel stop predicate for WP5.2 handovers.

The module deliberately has no ROS imports.  ``chassis_node`` converts a
``WheelStates`` message into the primitive dataclasses below before calling
``WheelStopPredicate.update``.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Sequence

import yaml


DEFAULT_SAMPLE_TIMEOUT_S = 0.1
REQUIRED_WHEEL_COUNT = 6


@dataclass(frozen=True)
class WheelStopConfig:
    thresholds_rev_s: Mapping[str, float]
    dwell_ms: int = 300
    qualified: bool = False


@dataclass(frozen=True)
class WheelStopWheel:
    name: str
    drive_turns_per_s: float
    drive_stale: bool
    steer_stale: bool
    drive_axis_error: int
    steer_fault: int


@dataclass(frozen=True)
class WheelStopSample:
    stamp_s: float
    healthy: bool
    wheels: Sequence[WheelStopWheel]
    authority_v: float
    authority_omega: float


def load_wheel_stop_config(path) -> WheelStopConfig:
    """Load the small non-ROS YAML contract used by the stop predicate."""
    source = Path(path)
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("wheel-stop config must be a mapping")

    qualified = data.get("qualified", False)
    thresholds = data.get("thresholds_rev_s", {})
    dwell_ms = data.get("dwell_ms", 300)
    if not isinstance(qualified, bool):
        raise ValueError("qualified must be a boolean")
    if not isinstance(thresholds, dict):
        raise ValueError("thresholds_rev_s must be a mapping")
    if isinstance(dwell_ms, bool) or not isinstance(dwell_ms, int):
        raise ValueError("dwell_ms must be an integer")
    return WheelStopConfig(
        thresholds_rev_s=dict(thresholds),
        dwell_ms=dwell_ms,
        qualified=qualified,
    )


class WheelStopPredicate:
    """Require every qualified wheel to remain stopped for one dwell."""

    def __init__(
        self,
        config: WheelStopConfig,
        *,
        sample_timeout_s: float = DEFAULT_SAMPLE_TIMEOUT_S,
    ):
        self.config = config
        self.sample_timeout_s = float(sample_timeout_s)
        if (
            not math.isfinite(self.sample_timeout_s)
            or self.sample_timeout_s <= 0.0
        ):
            raise ValueError("sample_timeout_s must be finite and positive")
        if config.dwell_ms < 0:
            raise ValueError("dwell_ms must be non-negative")

        self._thresholds = self._validated_thresholds(
            config.thresholds_rev_s
        )
        self._last_stamp_s = None
        self._last_now_s = None
        self._dwell_start_s = None
        self.confirmed = False
        self.last_reject_reason = "not_evaluated"

    @staticmethod
    def _validated_thresholds(thresholds):
        if not isinstance(thresholds, Mapping):
            return None
        if len(thresholds) != REQUIRED_WHEEL_COUNT:
            return None

        result = {}
        for name, value in thresholds.items():
            if not isinstance(name, str) or not name:
                return None
            try:
                threshold = float(value)
            except (TypeError, ValueError):
                return None
            # exact-zero thresholds are forbidden before encoder-floor HIL.
            if not math.isfinite(threshold) or threshold <= 0.0:
                return None
            result[name] = threshold
        return result

    @property
    def qualified(self) -> bool:
        return bool(self.config.qualified and self._thresholds is not None)

    def _reject(self, reason):
        self._dwell_start_s = None
        self.confirmed = False
        self.last_reject_reason = reason
        return False

    def update(self, sample: WheelStopSample, now_s: float) -> bool:
        if not self.config.qualified:
            return self._reject("unqualified")
        if self._thresholds is None:
            return self._reject("unqualified_threshold_map")

        try:
            now_s = float(now_s)
        except (TypeError, ValueError):
            return self._reject("invalid_now")
        if not math.isfinite(now_s):
            return self._reject("invalid_now")
        if self._last_now_s is not None and now_s < self._last_now_s:
            self._last_now_s = now_s
            self._last_stamp_s = None
            return self._reject("clock_not_monotonic")
        self._last_now_s = now_s

        try:
            stamp_s = float(sample.stamp_s)
        except (TypeError, ValueError):
            return self._reject("invalid_header_stamp")
        if not math.isfinite(stamp_s):
            return self._reject("invalid_header_stamp")
        age_s = now_s - stamp_s
        if age_s < 0.0:
            return self._reject("header_from_future")
        if age_s > self.sample_timeout_s:
            return self._reject("header_stale")
        if self._last_stamp_s is not None and stamp_s <= self._last_stamp_s:
            return self._reject("header_not_monotonic")
        self._last_stamp_s = stamp_s

        wheels = tuple(sample.wheels)
        if len(wheels) != REQUIRED_WHEEL_COUNT:
            return self._reject("wheel_count_not_6")
        names = [wheel.name for wheel in wheels]
        if len(set(names)) != REQUIRED_WHEEL_COUNT:
            return self._reject("duplicate_wheel_name")
        if set(names) != set(self._thresholds):
            return self._reject("wheel_name_threshold_mismatch")
        if not sample.healthy:
            return self._reject("sample_unhealthy")

        try:
            authority_v = float(sample.authority_v)
            authority_omega = float(sample.authority_omega)
        except (TypeError, ValueError):
            return self._reject("authority_output_nonfinite")
        if not math.isfinite(authority_v) or not math.isfinite(
            authority_omega
        ):
            return self._reject("authority_output_nonfinite")
        if authority_v != 0.0 or authority_omega != 0.0:
            return self._reject("authority_output_nonzero")

        for wheel in wheels:
            try:
                speed = float(wheel.drive_turns_per_s)
            except (TypeError, ValueError):
                return self._reject("wheel_speed_nonfinite:%s" % wheel.name)
            if not math.isfinite(speed):
                return self._reject("wheel_speed_nonfinite:%s" % wheel.name)
            if wheel.drive_stale:
                return self._reject("drive_stale:%s" % wheel.name)
            if wheel.steer_stale:
                return self._reject("steer_stale:%s" % wheel.name)
            if int(wheel.drive_axis_error) != 0:
                return self._reject("drive_axis_error:%s" % wheel.name)
            if int(wheel.steer_fault) != 0:
                return self._reject("steer_fault:%s" % wheel.name)
            if abs(speed) > self._thresholds[wheel.name]:
                return self._reject(
                    "wheel_above_threshold:%s" % wheel.name
                )

        if self._dwell_start_s is None:
            self._dwell_start_s = now_s
        dwell_s = self.config.dwell_ms / 1000.0
        if now_s - self._dwell_start_s < dwell_s:
            self.confirmed = False
            self.last_reject_reason = "dwell_not_met"
            return False

        self.confirmed = True
        self.last_reject_reason = ""
        return True
