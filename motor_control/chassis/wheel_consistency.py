"""Pure wheel-command/encoder consistency diagnostics.

The monitor consumes cached values only.  It never imports ROS, opens CAN, or
applies a speed limit; the caller decides whether to use the proposed terrain
profile cap.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from chassis.kinematics import ChassisGeometry


@dataclass(frozen=True)
class WheelConsistencyConfig:
    same_side_delta_turns_per_s: float = 0.75
    yaw_mismatch_rad_s: float = 0.5
    spin_turns_per_s: float = 1.0
    stopped_turns_per_s: float = 0.1
    active_command_turns_per_s: float = 0.5
    min_response_ratio: float = 0.4
    max_response_ratio: float = 1.6
    warn_speed_cap: float = 0.5

    def __post_init__(self):
        nonnegative = (
            self.same_side_delta_turns_per_s,
            self.yaw_mismatch_rad_s,
            self.spin_turns_per_s,
            self.stopped_turns_per_s,
            self.active_command_turns_per_s,
            self.min_response_ratio,
            self.max_response_ratio,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in nonnegative):
            raise ValueError("wheel consistency thresholds must be finite and nonnegative")
        if self.min_response_ratio > self.max_response_ratio:
            raise ValueError("min_response_ratio must not exceed max_response_ratio")
        if not math.isfinite(self.warn_speed_cap) or not 0.0 <= self.warn_speed_cap <= 1.0:
            raise ValueError("warn_speed_cap must be finite and within [0, 1]")


@dataclass(frozen=True)
class WheelConsistencySample:
    name: str
    command_turns_per_s: float
    measured_turns_per_s: float
    steer_deg: float = 0.0
    stale: bool = False


@dataclass(frozen=True)
class WheelConsistencyWarning:
    code: str
    wheels: Tuple[str, ...]
    value: float
    threshold: float


@dataclass(frozen=True)
class WheelConsistencyResult:
    warnings: Tuple[WheelConsistencyWarning, ...]
    terrain_speed_cap: float
    wheel_yaw_rate_rad_s: Optional[float]
    imu_yaw_rate_rad_s: Optional[float]


class WheelConsistencyMonitor:
    def __init__(self, geometry: ChassisGeometry, config: WheelConsistencyConfig):
        self.geometry = geometry
        self.config = config
        self._geometry_by_name = {wheel.name: wheel for wheel in geometry.wheels}

    def evaluate(self, samples, *, imu_yaw_rate_rad_s=None) -> WheelConsistencyResult:
        valid = {}
        for sample in samples:
            if sample.name not in self._geometry_by_name or sample.stale:
                continue
            values = (
                sample.command_turns_per_s,
                sample.measured_turns_per_s,
                sample.steer_deg,
            )
            if not all(math.isfinite(float(value)) for value in values):
                continue
            valid[sample.name] = sample

        warnings = []
        self._same_side_warnings(valid, warnings)
        wheel_yaw_rate = self._wheel_yaw_rate(valid)
        imu_yaw_rate = self._finite_optional(imu_yaw_rate_rad_s)
        if (
            wheel_yaw_rate is not None
            and imu_yaw_rate is not None
            and abs(wheel_yaw_rate - imu_yaw_rate) > self.config.yaw_mismatch_rad_s
        ):
            warnings.append(WheelConsistencyWarning(
                code="yaw_mismatch",
                wheels=tuple(sorted(valid)),
                value=abs(wheel_yaw_rate - imu_yaw_rate),
                threshold=self.config.yaw_mismatch_rad_s,
            ))
        self._single_wheel_warnings(valid, warnings)
        self._response_ratio_warnings(valid, warnings)

        return WheelConsistencyResult(
            warnings=tuple(warnings),
            terrain_speed_cap=self.config.warn_speed_cap if warnings else 1.0,
            wheel_yaw_rate_rad_s=wheel_yaw_rate,
            imu_yaw_rate_rad_s=imu_yaw_rate,
        )

    @staticmethod
    def _finite_optional(value):
        if value is None:
            return None
        value = float(value)
        return value if math.isfinite(value) else None

    def _same_side_warnings(self, valid, warnings):
        for suffix in ("_left", "_right"):
            side = [
                sample
                for name, sample in valid.items()
                if name.endswith(suffix)
            ]
            if len(side) < 2:
                continue
            response_errors = [
                sample.measured_turns_per_s - sample.command_turns_per_s
                for sample in side
            ]
            delta = max(response_errors) - min(response_errors)
            if delta > self.config.same_side_delta_turns_per_s:
                warnings.append(WheelConsistencyWarning(
                    code="same_side_delta",
                    wheels=tuple(sample.name for sample in side),
                    value=delta,
                    threshold=self.config.same_side_delta_turns_per_s,
                ))

    def _wheel_yaw_rate(self, valid):
        circumference = 2.0 * math.pi * self.geometry.wheel_radius_m
        estimates = []
        for axle in ("front", "mid", "rear"):
            left = valid.get(f"{axle}_left")
            right = valid.get(f"{axle}_right")
            if left is None or right is None:
                continue
            left_geometry = self._geometry_by_name[left.name]
            right_geometry = self._geometry_by_name[right.name]
            track_m = abs(left_geometry.y - right_geometry.y)
            if track_m <= 0.0:
                continue
            left_mps = left.measured_turns_per_s * circumference
            right_mps = right.measured_turns_per_s * circumference
            estimates.append((right_mps - left_mps) / track_m)
        if not estimates:
            return None
        return sum(estimates) / len(estimates)

    def _single_wheel_warnings(self, valid, warnings):
        values = list(valid.values())
        spin = [
            sample
            for sample in values
            if abs(sample.command_turns_per_s) < self.config.active_command_turns_per_s
            and abs(sample.measured_turns_per_s) >= self.config.spin_turns_per_s
        ]
        if (
            len(spin) == 1
            and all(
                other is spin[0]
                or abs(other.measured_turns_per_s) <= self.config.stopped_turns_per_s
                for other in values
            )
        ):
            warnings.append(WheelConsistencyWarning(
                code="single_wheel_spin",
                wheels=(spin[0].name,),
                value=abs(spin[0].measured_turns_per_s),
                threshold=self.config.spin_turns_per_s,
            ))

        active = [
            sample
            for sample in values
            if abs(sample.command_turns_per_s) >= self.config.active_command_turns_per_s
        ]
        stopped = [
            sample
            for sample in active
            if abs(sample.measured_turns_per_s) <= self.config.stopped_turns_per_s
        ]
        moving = [sample for sample in active if sample not in stopped]
        if len(stopped) == 1 and len(moving) >= 2:
            warnings.append(WheelConsistencyWarning(
                code="single_wheel_stop",
                wheels=(stopped[0].name,),
                value=abs(stopped[0].measured_turns_per_s),
                threshold=self.config.stopped_turns_per_s,
            ))

    def _response_ratio_warnings(self, valid, warnings):
        for sample in valid.values():
            command = abs(sample.command_turns_per_s)
            if command < self.config.active_command_turns_per_s:
                continue
            ratio = abs(sample.measured_turns_per_s) / command
            if (
                ratio < self.config.min_response_ratio
                or ratio > self.config.max_response_ratio
            ):
                threshold = (
                    self.config.min_response_ratio
                    if ratio < self.config.min_response_ratio
                    else self.config.max_response_ratio
                )
                warnings.append(WheelConsistencyWarning(
                    code="response_ratio",
                    wheels=(sample.name,),
                    value=ratio,
                    threshold=threshold,
                ))
