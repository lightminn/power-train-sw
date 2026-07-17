"""Run and record one headless MuJoCo fast-mode scenario."""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np

from chassis.kinematics import ChassisGeometry, default_geometry
from powertrain_ros.state_estimation import (
    StateEstimator,
    StateEstimatorConfig,
    StateSnapshot,
)

from ..fixtures import DepthFrame, GroundTruthFrame, _motion
from ..recording import RunWriter
from ..scenario import Scenario
from .model_builder import WHEEL_HALF_WIDTH_M
from .plant import MujocoFastPlant
from .sensors import FastSensorSuite


CommandSource = Callable[[float, StateSnapshot | None], tuple[float, float]]
HoldStateSource = Callable[[float, StateSnapshot | None], tuple[bool, bool]]
DepthTap = Callable[[DepthFrame], None]


def _apply_depth_degradation(
    frame: DepthFrame,
    *,
    elapsed_s: float,
    faults,
    rng: np.random.Generator,
) -> DepthFrame:
    """Apply active ramped dropout/noise faults without mutating sensor RNG state."""
    active = tuple(
        fault
        for fault in faults
        if fault["start_s"] <= elapsed_s < fault["end_s"]
    )
    if not active:
        return frame

    scale_m = float(frame.depth_scale_m)
    depth_m = np.asarray(frame.depth_roi, dtype=float).copy() * scale_m
    for fault in active:
        progress = (float(elapsed_s) - float(fault["start_s"])) / (
            float(fault["end_s"]) - float(fault["start_s"])
        )
        dropout_ratio = float(fault["dropout_ratio_start"]) + progress * (
            float(fault["dropout_ratio_end"])
            - float(fault["dropout_ratio_start"])
        )
        valid_flat = np.flatnonzero(depth_m.ravel() > 0.0)
        dropout_count = min(
            len(valid_flat),
            max(0, int(round(dropout_ratio * len(valid_flat)))),
        )
        if dropout_count:
            dropped = rng.choice(valid_flat, size=dropout_count, replace=False)
            depth_m.ravel()[dropped] = 0.0

        noise_std_m = float(fault["noise_std_m"])
        valid = depth_m > 0.0
        if noise_std_m and np.any(valid):
            depth_m[valid] += rng.normal(
                0.0,
                noise_std_m,
                size=np.count_nonzero(valid),
            )

    raw = np.rint(
        np.clip(depth_m / scale_m, 0.0, np.iinfo(np.uint16).max)
    ).astype(np.uint16)
    raw.setflags(write=False)
    return DepthFrame(
        stamp_s=frame.stamp_s,
        depth_roi=raw,
        depth_scale_m=frame.depth_scale_m,
        intrinsics=frame.intrinsics,
        frame_id=frame.frame_id,
    )


@dataclass(frozen=True)
class MetricsReport:
    scenario_id: str
    completion_ratio: float
    min_wheel_clearance_m: float
    edge_overrun_count: int
    false_hold_count: int
    fail_open_count: int
    max_recovery_time_s: float
    wall_clock_runtime_s: float
    max_estimator_runtime_ms: float
    distance_error_ratio: float
    yaw_error_ratio: float
    passed: bool
    reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        output = asdict(self)
        output["reasons"] = list(self.reasons)
        for key, value in output.items():
            # error ratio는 추정치가 없으면 inf — JSON(allow_nan=False)에서 None으로.
            if isinstance(value, float) and not math.isfinite(value):
                output[key] = None
        return output

    def summary(self) -> str:
        status = "PASS" if self.passed else "FAIL"
        reasons = "none" if not self.reasons else "; ".join(self.reasons)
        return (
            f"MetricsReport[{status}] scenario={self.scenario_id} "
            f"completion={self.completion_ratio:.4f} "
            f"clearance={self.min_wheel_clearance_m:.4f}m "
            f"overrun={self.edge_overrun_count} "
            f"false_hold={self.false_hold_count} fail_open={self.fail_open_count} "
            f"recovery={self.max_recovery_time_s:.4f}s "
            f"runtime={self.wall_clock_runtime_s:.4f}s "
            f"estimator_max={self.max_estimator_runtime_ms:.4f}ms "
            f"distance_error={100.0 * self.distance_error_ratio:.3f}% "
            f"yaw_error={100.0 * self.yaw_error_ratio:.3f}% "
            f"reasons={reasons}"
        )


class HoldMetricsTracker:
    """Count hold-policy episodes without treating allowed recovery as false hold."""

    def __init__(self) -> None:
        self.false_hold_count = 0
        self.fail_open_count = 0
        self.max_recovery_time_s = 0.0
        self._false_hold_active = False
        self._fail_open_active = False
        self._previous_should_hold: bool | None = None
        self._recovery_started_s: float | None = None

    def observe(self, t_s: float, *, actual_hold: bool, should_hold: bool) -> None:
        if not math.isfinite(t_s) or t_s < 0.0:
            raise ValueError("hold metric time must be finite and nonnegative")
        if not isinstance(actual_hold, bool) or not isinstance(should_hold, bool):
            raise ValueError("hold metric states must be boolean")

        released = self._previous_should_hold is True and not should_hold
        if released:
            self._recovery_started_s = float(t_s) if actual_hold else None
        if should_hold:
            self._recovery_started_s = None
        elif self._recovery_started_s is not None and not actual_hold:
            recovery = float(t_s) - self._recovery_started_s
            self.max_recovery_time_s = max(self.max_recovery_time_s, recovery)
            self._recovery_started_s = None

        false_hold = actual_hold and not should_hold and self._recovery_started_s is None
        fail_open = should_hold and not actual_hold
        if false_hold and not self._false_hold_active:
            self.false_hold_count += 1
        if fail_open and not self._fail_open_active:
            self.fail_open_count += 1
        self._false_hold_active = false_hold
        self._fail_open_active = fail_open
        self._previous_should_hold = should_hold


@dataclass(frozen=True)
class _Projection:
    station_m: float
    lateral_m: float
    width_m: float
    left_drop: bool
    right_drop: bool


class _TrackProjector:
    def __init__(self, scenario: Scenario) -> None:
        self.scenario = scenario
        self.points = np.asarray(scenario.track.centerline_m, dtype=float)
        self.segments = np.diff(self.points, axis=0)
        self.lengths = np.linalg.norm(self.segments, axis=1)
        self.stations = np.concatenate(([0.0], np.cumsum(self.lengths)))

    @property
    def length_m(self) -> float:
        return float(self.stations[-1])

    def project(self, point_world: tuple[float, float, float] | np.ndarray) -> _Projection:
        point = np.asarray(point_world, dtype=float)
        best: tuple[float, int, float, np.ndarray] | None = None
        for index, (start, segment, length) in enumerate(
            zip(self.points, self.segments, self.lengths)
        ):
            fraction = float(np.clip(np.dot(point - start, segment) / (length * length), 0.0, 1.0))
            nearest = start + fraction * segment
            distance_squared = float(np.dot(point - nearest, point - nearest))
            candidate = (distance_squared, index, fraction, nearest)
            if best is None or candidate[0] < best[0]:
                best = candidate
        assert best is not None
        _, index, fraction, nearest = best
        tangent_xy = self.segments[index, :2]
        tangent_xy = tangent_xy / np.linalg.norm(tangent_xy)
        delta_xy = point[:2] - nearest[:2]
        lateral = float(
            tangent_xy[0] * delta_xy[1] - tangent_xy[1] * delta_xy[0]
        )
        width = float(
            self.scenario.track.width_m[index]
            + fraction
            * (
                self.scenario.track.width_m[index + 1]
                - self.scenario.track.width_m[index]
            )
        )
        left = (
            self.scenario.track.drop_boundaries[index].left
            or self.scenario.track.drop_boundaries[index + 1].left
        )
        right = (
            self.scenario.track.drop_boundaries[index].right
            or self.scenario.track.drop_boundaries[index + 1].right
        )
        return _Projection(
            station_m=float(self.stations[index] + fraction * self.lengths[index]),
            lateral_m=lateral,
            width_m=width,
            left_drop=left,
            right_drop=right,
        )


class _MetricsAccumulator:
    def __init__(self, scenario: Scenario, geometry: ChassisGeometry) -> None:
        self.scenario = scenario
        self.geometry = geometry
        self.projector = _TrackProjector(scenario)
        self.max_station_m = 0.0
        self.min_clearance_m = math.inf
        self.edge_overrun_count = 0
        self._overrun = {wheel.name: False for wheel in geometry.wheels}
        self._first_truth: GroundTruthFrame | None = None
        self._previous_truth: GroundTruthFrame | None = None
        self.truth_distance_m = 0.0

    def observe(
        self,
        truth: GroundTruthFrame,
        wheel_contacts: dict[str, tuple[float, float, float]],
    ) -> None:
        if self._first_truth is None:
            self._first_truth = truth
        if self._previous_truth is not None:
            self.truth_distance_m += math.hypot(
                truth.x_m - self._previous_truth.x_m,
                truth.y_m - self._previous_truth.y_m,
            )
        self._previous_truth = truth
        body_projection = self.projector.project((truth.x_m, truth.y_m, truth.z_m))
        self.max_station_m = max(self.max_station_m, body_projection.station_m)

        for wheel in self.geometry.wheels:
            projection = self.projector.project(wheel_contacts[wheel.name])
            if projection.lateral_m >= 0.0:
                clearance = (
                    projection.width_m / 2.0 - projection.lateral_m
                    if projection.left_drop
                    else math.inf
                )
            else:
                clearance = (
                    projection.width_m / 2.0 + projection.lateral_m
                    if projection.right_drop
                    else math.inf
                )
            self.min_clearance_m = min(self.min_clearance_m, clearance)
            overrun = clearance <= WHEEL_HALF_WIDTH_M
            if overrun and not self._overrun[wheel.name]:
                self.edge_overrun_count += 1
            self._overrun[wheel.name] = overrun

    @property
    def completion_ratio(self) -> float:
        return float(np.clip(self.max_station_m / self.projector.length_m, 0.0, 1.0))

    def error_ratios(self, snapshot: StateSnapshot | None) -> tuple[float, float]:
        if snapshot is None or self._first_truth is None or self._previous_truth is None:
            return math.inf, math.inf
        if self.truth_distance_m <= 1e-9:
            distance_error = 0.0
        else:
            distance_error = abs(snapshot.distance_m - self.truth_distance_m) / self.truth_distance_m
        truth_yaw = math.atan2(
            math.sin(self._previous_truth.yaw_rad - self._first_truth.yaw_rad),
            math.cos(self._previous_truth.yaw_rad - self._first_truth.yaw_rad),
        )
        if abs(truth_yaw) <= 1e-9:
            yaw_error = 0.0
        else:
            yaw_error = abs(
                math.atan2(
                    math.sin(snapshot.pose.yaw_rad - truth_yaw),
                    math.cos(snapshot.pose.yaw_rad - truth_yaw),
                )
            ) / abs(truth_yaw)
        return distance_error, yaw_error


def _comparison_reasons(
    scenario: Scenario,
    *,
    completion_ratio: float,
    min_clearance_m: float,
    edge_overrun_count: int,
    hold: HoldMetricsTracker,
    max_estimator_runtime_ms: float,
) -> tuple[str, ...]:
    expected = scenario.expected_metrics
    reasons = []
    completed = completion_ratio >= 0.95
    if completed != bool(expected["completion"]):
        reasons.append(
            f"completion {completion_ratio:.4f} does not match expected {bool(expected['completion'])}"
        )
    if min_clearance_m < float(expected["min_clearance_m"]):
        reasons.append(
            f"clearance {min_clearance_m:.4f}m below {float(expected['min_clearance_m']):.4f}m"
        )
    for label, actual in (
        ("edge_overrun_count", edge_overrun_count),
        ("false_hold_count", hold.false_hold_count),
        ("fail_open_count", hold.fail_open_count),
    ):
        if actual > int(expected[label]):
            reasons.append(f"{label} {actual} exceeds {int(expected[label])}")
    if hold.max_recovery_time_s > float(expected["max_recovery_time_s"]):
        reasons.append(
            f"recovery {hold.max_recovery_time_s:.4f}s exceeds "
            f"{float(expected['max_recovery_time_s']):.4f}s"
        )
    if max_estimator_runtime_ms > float(expected["max_estimator_runtime_ms"]):
        reasons.append(
            f"estimator runtime {max_estimator_runtime_ms:.4f}ms exceeds "
            f"{float(expected['max_estimator_runtime_ms']):.4f}ms"
        )
    return tuple(reasons)


def run_scenario(
    scenario: Scenario,
    run_directory: str | Path,
    *,
    command_source: CommandSource | None = None,
    hold_state_source: HoldStateSource | None = None,
    depth_tap: DepthTap | None = None,
    geometry: ChassisGeometry | None = None,
) -> MetricsReport:
    """Execute, record, replay-compatible sample, and score one fast-mode run."""
    started_ns = time.perf_counter_ns()
    geometry = geometry or default_geometry()
    plant = MujocoFastPlant(scenario, geometry=geometry)
    sensors = FastSensorSuite(scenario, plant)
    # A jumped stream is derived from the scenario seed but isolated from the
    # established FastSensorSuite draw order. Existing families consume no new
    # draws when depth_degradation is absent.
    degradation_rng = np.random.Generator(
        np.random.PCG64(scenario.prng.seed).jumped()
    )
    estimator = StateEstimator(geometry, StateEstimatorConfig(bias_samples=0))
    metrics = _MetricsAccumulator(scenario, geometry)
    hold = HoldMetricsTracker()
    latest_estimate: StateSnapshot | None = None
    max_estimator_runtime_ms = 0.0
    output = Path(run_directory)

    with RunWriter(output, run_id=scenario.scenario_id) as writer:
        for index in range(scenario.clock.sample_count):
            elapsed_s = index * scenario.clock.dt_s
            if command_source is None:
                v_m_s, omega_rad_s = _motion(scenario, elapsed_s)
            else:
                command = command_source(elapsed_s, latest_estimate)
                if not isinstance(command, tuple) or len(command) != 2:
                    raise ValueError("command_source must return (v_m_s, omega_rad_s)")
                v_m_s, omega_rad_s = command
            plant.apply_command(float(v_m_s), float(omega_rad_s))

            wheel = sensors.sample_wheel(index)
            imu = sensors.sample_imu(index)
            depth = sensors.sample_depth(index)
            if depth is not None:
                depth = _apply_depth_degradation(
                    depth,
                    elapsed_s=elapsed_s,
                    faults=scenario.faults.get("depth_degradation", ()),
                    rng=degradation_rng,
                )
            truth = sensors.sample_ground_truth(index)
            if wheel is not None:
                writer.write_wheel(wheel)
                estimator_started_ns = time.perf_counter_ns()
                decision = estimator.update_wheels(wheel, now_s=wheel.stamp_s)
                estimator_elapsed_ms = (time.perf_counter_ns() - estimator_started_ns) / 1e6
                max_estimator_runtime_ms = max(
                    max_estimator_runtime_ms,
                    estimator_elapsed_ms,
                )
                if not decision.accepted:
                    raise RuntimeError(f"production estimator rejected wheel sample: {decision.reason}")
            if imu is not None:
                writer.write_imu(imu)
                estimator_started_ns = time.perf_counter_ns()
                decision = estimator.update_imu(imu, now_s=imu.stamp_s)
                estimator_elapsed_ms = (time.perf_counter_ns() - estimator_started_ns) / 1e6
                max_estimator_runtime_ms = max(
                    max_estimator_runtime_ms,
                    estimator_elapsed_ms,
                )
                if not decision.accepted:
                    raise RuntimeError(f"production estimator rejected IMU sample: {decision.reason}")
            if depth is not None:
                writer.write_depth(depth)
                if depth_tap is not None:
                    depth_tap(depth)
            writer.write_ground_truth(truth)

            latest_estimate = estimator.snapshot(now_s=truth.stamp_s)
            if hold_state_source is None:
                actual_hold, should_hold = False, False
            else:
                actual_hold, should_hold = hold_state_source(elapsed_s, latest_estimate)
            hold.observe(
                elapsed_s,
                actual_hold=actual_hold,
                should_hold=should_hold,
            )
            metrics.observe(truth, plant.wheel_contact_points_world())
            if index + 1 < scenario.clock.sample_count:
                plant.step_clock_interval()

    runtime_s = (time.perf_counter_ns() - started_ns) / 1e9
    distance_error, yaw_error = metrics.error_ratios(latest_estimate)
    clearance = metrics.min_clearance_m
    reasons = _comparison_reasons(
        scenario,
        completion_ratio=metrics.completion_ratio,
        min_clearance_m=clearance,
        edge_overrun_count=metrics.edge_overrun_count,
        hold=hold,
        max_estimator_runtime_ms=max_estimator_runtime_ms,
    )
    report = MetricsReport(
        scenario_id=scenario.scenario_id,
        completion_ratio=metrics.completion_ratio,
        min_wheel_clearance_m=clearance,
        edge_overrun_count=metrics.edge_overrun_count,
        false_hold_count=hold.false_hold_count,
        fail_open_count=hold.fail_open_count,
        max_recovery_time_s=hold.max_recovery_time_s,
        wall_clock_runtime_s=runtime_s,
        max_estimator_runtime_ms=max_estimator_runtime_ms,
        distance_error_ratio=distance_error,
        yaw_error_ratio=yaw_error,
        passed=not reasons,
        reasons=reasons,
    )
    (output / "metrics.json").write_text(
        json.dumps(report.to_dict(), sort_keys=True, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report


__all__ = (
    "CommandSource",
    "DepthTap",
    "HoldMetricsTracker",
    "HoldStateSource",
    "MetricsReport",
    "run_scenario",
)
