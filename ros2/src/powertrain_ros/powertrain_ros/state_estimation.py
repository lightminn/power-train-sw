"""ROS-free WP6-A wheel and IMU state-estimation core.

Adapters inject primitive, timestamped values.  This module combines the
existing chassis odometry solver with the existing wheel-consistency monitor;
it does not duplicate either wheel model or its slip/stuck thresholds.

The accumulated odometry is short-horizon relative state.  It must not be used
as the sole mission-arrival condition.
"""

from dataclasses import dataclass, field
import math
from typing import Optional, Sequence, Tuple

from chassis.odometry import (
    OdometryConfig,
    WheelObservation,
    solve_twist,
)
from chassis.wheel_consistency import (
    WheelConsistencyConfig,
    WheelConsistencyMonitor,
    WheelConsistencyResult,
    WheelConsistencySample,
)


@dataclass(frozen=True)
class WheelValue:
    name: str
    command_turns_per_s: float
    measured_turns_per_s: float
    steer_deg: float = 0.0
    stale: bool = False


@dataclass(frozen=True)
class WheelSample:
    stamp_s: float
    wheels: Sequence[WheelValue]


@dataclass(frozen=True)
class ImuSample:
    stamp_s: float
    gyro_x_rad_s: float
    gyro_y_rad_s: float
    gyro_z_rad_s: float
    accel_x_m_s2: float
    accel_y_m_s2: float
    accel_z_m_s2: float


@dataclass(frozen=True)
class StateEstimatorConfig:
    sample_timeout_s: float = 0.25
    bias_samples: int = 200
    accel_lpf_alpha: float = 0.8
    complementary_alpha: float = 0.98
    stationary_command_turns_per_s: float = 0.05
    stationary_measured_turns_per_s: float = 0.05
    # bias 후보 상한: 이보다 큰 |gyro - bias| 는 정지 중에도 실회전으로 적분한다
    # (빙판 슬립·외력 회전을 bias 로 삼키지 않기 위한 타당성 게이트).
    max_bias_rad_s: float = 0.05
    terrain_profile: str = "default"
    use_imu_yaw: bool = True
    odometry: OdometryConfig = field(default_factory=OdometryConfig)
    wheel_consistency: WheelConsistencyConfig = field(
        default_factory=WheelConsistencyConfig
    )


@dataclass(frozen=True)
class SampleDecision:
    accepted: bool
    reason: str = ""
    reinitialized: bool = False


@dataclass(frozen=True)
class PoseSnapshot:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class VelocitySnapshot:
    forward_m_s: float
    lateral_m_s: float
    yaw_rate_rad_s: float


@dataclass(frozen=True)
class TiltSnapshot:
    roll_rad: float
    pitch_rad: float


@dataclass(frozen=True)
class DiagnosticSnapshot:
    slip_candidate: bool
    stuck_candidate: bool
    one_wheel_mismatch: bool
    warning_codes: Tuple[str, ...]
    affected_wheels: Tuple[str, ...]
    terrain_profile: str
    terrain_speed_cap: float
    wheel_yaw_rate_rad_s: Optional[float]
    imu_yaw_rate_rad_s: Optional[float]


@dataclass(frozen=True)
class StateSnapshot:
    pose: PoseSnapshot
    velocity: VelocitySnapshot
    tilt: TiltSnapshot
    distance_m: float
    diagnostics: DiagnosticSnapshot
    stale: bool
    wheel_stale: bool
    imu_stale: bool
    initialized: bool
    reinitialized: bool
    reconnect_count: int
    yaw_source: str
    gyro_bias_rad_s: Tuple[float, float, float]


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class StateEstimator:
    """Fuse qualified wheel and IMU values into one immutable snapshot."""

    def __init__(self, geometry, config: StateEstimatorConfig = None):
        self.geometry = geometry
        self.config = config or StateEstimatorConfig()
        self._validate_config()
        self._geometry_names = tuple(wheel.name for wheel in geometry.wheels)
        self._consistency_monitor = WheelConsistencyMonitor(
            geometry,
            self.config.wheel_consistency,
        )
        self._consistency = self._consistency_monitor.evaluate(())

        self._x_m = 0.0
        self._y_m = 0.0
        self._yaw_rad = 0.0
        self._distance_m = 0.0
        self._roll_rad = 0.0
        self._pitch_rad = 0.0
        self._tilt_initialized = False

        self._last_wheel_stamp_s = None
        self._last_imu_stamp_s = None
        # 수신 시계(now_s) 도메인의 마지막 관측 시각 — stamp 도메인과 혼용 비교 금지.
        self._last_wheel_seen_s = None
        self._last_imu_seen_s = None
        self._last_twist = None
        self._velocity = (0.0, 0.0)
        self._imu_rate_rad_s = (0.0, 0.0, 0.0)
        self._imu_valid = False
        self._accel_filtered = None

        self._gyro_bias = [0.0, 0.0, 0.0]
        self._bias_accumulator = [0.0, 0.0, 0.0]
        self._bias_count = 0
        self._wheel_stationary = None

        self._frozen = False
        self._wheel_reinitialized = False
        self._imu_reinitialized = False
        self._reconnect_count = 0
        self._yaw_source = "none"

    def _validate_config(self):
        cfg = self.config
        if not _finite(cfg.sample_timeout_s) or cfg.sample_timeout_s <= 0.0:
            raise ValueError("sample_timeout_s must be finite and positive")
        if (
            isinstance(cfg.bias_samples, bool)
            or not isinstance(cfg.bias_samples, int)
            or cfg.bias_samples < 0
        ):
            raise ValueError("bias_samples must be a nonnegative integer")
        for label, value in (
            ("accel_lpf_alpha", cfg.accel_lpf_alpha),
            ("complementary_alpha", cfg.complementary_alpha),
        ):
            if not _finite(value) or not 0.0 <= float(value) <= 1.0:
                raise ValueError("%s must be finite and within [0, 1]" % label)
        for label, value in (
            (
                "stationary_command_turns_per_s",
                cfg.stationary_command_turns_per_s,
            ),
            (
                "stationary_measured_turns_per_s",
                cfg.stationary_measured_turns_per_s,
            ),
            ("max_bias_rad_s", cfg.max_bias_rad_s),
        ):
            if not _finite(value) or float(value) < 0.0:
                raise ValueError("%s must be finite and nonnegative" % label)
        if not isinstance(cfg.terrain_profile, str) or not cfg.terrain_profile:
            raise ValueError("terrain_profile must be a nonempty string")
        if not isinstance(cfg.use_imu_yaw, bool):
            raise ValueError("use_imu_yaw must be boolean")

    @staticmethod
    def _stamp_decision(stamp_s, now_s, last_stamp_s):
        if not _finite(stamp_s):
            return SampleDecision(False, "stamp_invalid")
        stamp_s = float(stamp_s)
        if stamp_s == 0.0:
            return SampleDecision(False, "stamp_zero")
        if stamp_s < 0.0:
            return SampleDecision(False, "stamp_invalid")
        if not _finite(now_s):
            return SampleDecision(False, "now_invalid")
        now_s = float(now_s)
        if stamp_s > now_s:
            return SampleDecision(False, "stamp_future")
        if last_stamp_s is not None and stamp_s <= last_stamp_s:
            return SampleDecision(False, "stamp_not_monotonic")
        return SampleDecision(True)

    def update_wheels(self, sample: WheelSample, *, now_s: float) -> SampleDecision:
        decision = self._stamp_decision(
            sample.stamp_s,
            now_s,
            self._last_wheel_stamp_s,
        )
        if not decision.accepted:
            return decision
        stamp_s = float(sample.stamp_s)
        wheels = tuple(sample.wheels)
        if not self._valid_wheel_set(wheels):
            return SampleDecision(False, "wheel_set_invalid")

        previous_stamp_s = self._last_wheel_stamp_s
        gap = (
            previous_stamp_s is not None
            and stamp_s - previous_stamp_s > self.config.sample_timeout_s
        )
        reconnecting = bool(gap or self._frozen)

        observations = tuple(
            WheelObservation.from_turns_per_s(
                wheel.name,
                float(wheel.measured_turns_per_s),
                float(wheel.steer_deg),
                wheel_radius_m=self.geometry.wheel_radius_m,
                valid=not wheel.stale,
            )
            for wheel in wheels
        )
        twist = solve_twist(self.geometry, observations, self.config.odometry)
        consistency_samples = tuple(
            WheelConsistencySample(
                name=wheel.name,
                command_turns_per_s=float(wheel.command_turns_per_s),
                measured_turns_per_s=float(wheel.measured_turns_per_s),
                steer_deg=float(wheel.steer_deg),
                stale=bool(wheel.stale),
            )
            for wheel in wheels
        )
        imu_rate = self._fresh_imu_rate(stamp_s)
        self._consistency = self._consistency_monitor.evaluate(
            consistency_samples,
            imu_yaw_rate_rad_s=imu_rate,
        )
        self._update_stationary(wheels)

        self._last_wheel_stamp_s = stamp_s
        self._last_wheel_seen_s = float(now_s)
        if previous_stamp_s is None or reconnecting:
            self._last_twist = twist
            self._velocity = (twist.vx, twist.vy)
            self._frozen = False
            self._wheel_reinitialized = reconnecting
            if reconnecting:
                self._reconnect_count += 1
            self._yaw_source = "imu" if imu_rate is not None else "wheel"
            return SampleDecision(True, reinitialized=reconnecting)

        dt = stamp_s - previous_stamp_s
        previous = self._last_twist or twist
        vx = 0.5 * (previous.vx + twist.vx)
        vy = 0.5 * (previous.vy + twist.vy)
        if imu_rate is None:
            yaw_rate = 0.5 * (previous.omega + twist.omega)
            midpoint_yaw = self._yaw_rad + 0.5 * yaw_rate * dt
            self._yaw_rad = _wrap(self._yaw_rad + yaw_rate * dt)
            self._yaw_source = "wheel"
        else:
            midpoint_yaw = self._yaw_rad
            self._yaw_source = "imu"

        cosine = math.cos(midpoint_yaw)
        sine = math.sin(midpoint_yaw)
        self._x_m += (vx * cosine - vy * sine) * dt
        self._y_m += (vx * sine + vy * cosine) * dt
        self._distance_m += math.hypot(vx, vy) * dt
        self._last_twist = twist
        self._velocity = (twist.vx, twist.vy)
        self._wheel_reinitialized = False
        return SampleDecision(True)

    def _valid_wheel_set(self, wheels):
        if len(wheels) != len(self._geometry_names):
            return False
        if {wheel.name for wheel in wheels} != set(self._geometry_names):
            return False
        for wheel in wheels:
            if not isinstance(wheel.name, str) or not wheel.name:
                return False
            if not isinstance(wheel.stale, bool):
                return False
            values = (
                wheel.command_turns_per_s,
                wheel.measured_turns_per_s,
                wheel.steer_deg,
            )
            if not wheel.stale and not all(_finite(value) for value in values):
                return False
        return True

    def _update_stationary(self, wheels):
        valid = tuple(wheel for wheel in wheels if not wheel.stale)
        stationary = len(valid) == len(self._geometry_names) and all(
            abs(float(wheel.command_turns_per_s))
            <= self.config.stationary_command_turns_per_s
            and abs(float(wheel.measured_turns_per_s))
            <= self.config.stationary_measured_turns_per_s
            for wheel in valid
        )
        if stationary and self._wheel_stationary is False:
            self._bias_accumulator = [0.0, 0.0, 0.0]
            self._bias_count = 0
        self._wheel_stationary = stationary

    def update_imu(self, sample: ImuSample, *, now_s: float) -> SampleDecision:
        decision = self._stamp_decision(
            sample.stamp_s,
            now_s,
            self._last_imu_stamp_s,
        )
        if not decision.accepted:
            return decision
        values = (
            sample.gyro_x_rad_s,
            sample.gyro_y_rad_s,
            sample.gyro_z_rad_s,
            sample.accel_x_m_s2,
            sample.accel_y_m_s2,
            sample.accel_z_m_s2,
        )
        if not all(_finite(value) for value in values):
            self._imu_valid = False
            self._yaw_source = "wheel" if self._last_wheel_stamp_s else "none"
            return SampleDecision(False, "imu_nonfinite")

        stamp_s = float(sample.stamp_s)
        previous_stamp_s = self._last_imu_stamp_s
        gap = (
            previous_stamp_s is not None
            and stamp_s - previous_stamp_s > self.config.sample_timeout_s
        )
        if self._last_wheel_seen_s is not None and (
            float(now_s) - self._last_wheel_seen_s
            > self.config.sample_timeout_s
        ):
            self._freeze()

        gyro = (
            float(sample.gyro_x_rad_s),
            float(sample.gyro_y_rad_s),
            float(sample.gyro_z_rad_s),
        )
        accel = (
            float(sample.accel_x_m_s2),
            float(sample.accel_y_m_s2),
            float(sample.accel_z_m_s2),
        )
        self._update_accel_filter(accel)

        startup_learning = (
            self._bias_count < self.config.bias_samples
            and self._wheel_stationary is not False
        )
        stationary_learning = self._wheel_stationary is True
        bias_plausible = all(
            abs(gyro[index] - self._gyro_bias[index])
            <= self.config.max_bias_rad_s
            for index in range(3)
        )
        learning = (startup_learning or stationary_learning) and bias_plausible
        if learning:
            self._learn_bias(gyro)
        corrected = tuple(gyro[index] - self._gyro_bias[index] for index in range(3))
        self._imu_rate_rad_s = corrected
        self._imu_valid = True
        self._last_imu_stamp_s = stamp_s
        self._last_imu_seen_s = float(now_s)
        if gap:
            self._imu_reinitialized = True
            self._reconnect_count += 1
        elif previous_stamp_s is not None:
            self._imu_reinitialized = False

        dt = None if previous_stamp_s is None else stamp_s - previous_stamp_s
        self._update_tilt(corrected, dt, reinitialize=gap)
        if (
            dt is not None
            and not gap
            and not self._frozen
            and not learning
            and self.config.use_imu_yaw
        ):
            self._yaw_rad = _wrap(self._yaw_rad + corrected[2] * dt)
        self._yaw_source = (
            "imu" if self.config.use_imu_yaw else "wheel"
        )
        return SampleDecision(True, reinitialized=gap)

    def _learn_bias(self, gyro):
        for index, value in enumerate(gyro):
            self._bias_accumulator[index] += value
        self._bias_count += 1
        self._gyro_bias = [
            total / self._bias_count for total in self._bias_accumulator
        ]

    def _update_accel_filter(self, accel):
        if self._accel_filtered is None:
            self._accel_filtered = accel
            return
        alpha = float(self.config.accel_lpf_alpha)
        self._accel_filtered = tuple(
            alpha * previous + (1.0 - alpha) * current
            for previous, current in zip(self._accel_filtered, accel)
        )

    def _update_tilt(self, corrected_gyro, dt, *, reinitialize):
        accel_x, accel_y, accel_z = self._accel_filtered
        roll_acc = math.atan2(accel_y, accel_z)
        pitch_acc = math.atan2(-accel_x, math.hypot(accel_y, accel_z))
        if not self._tilt_initialized or dt is None or reinitialize:
            self._roll_rad = roll_acc
            self._pitch_rad = pitch_acc
            self._tilt_initialized = True
            return
        alpha = float(self.config.complementary_alpha)
        self._roll_rad = (
            alpha * (self._roll_rad + corrected_gyro[0] * dt)
            + (1.0 - alpha) * roll_acc
        )
        self._pitch_rad = (
            alpha * (self._pitch_rad + corrected_gyro[1] * dt)
            + (1.0 - alpha) * pitch_acc
        )

    def _fresh_imu_rate(self, stamp_s):
        if (
            not self.config.use_imu_yaw
            or not self._imu_valid
            or self._last_imu_stamp_s is None
        ):
            return None
        if stamp_s - self._last_imu_stamp_s > self.config.sample_timeout_s:
            return None
        return self._imu_rate_rad_s[2]

    def _freeze(self):
        self._frozen = True
        self._velocity = (0.0, 0.0)

    def reset(self):
        """Explicitly reset pose and every integration/freshness baseline."""
        self._x_m = 0.0
        self._y_m = 0.0
        self._yaw_rad = 0.0
        self._distance_m = 0.0
        self._last_wheel_stamp_s = None
        self._last_imu_stamp_s = None
        # 수신 시계(now_s) 도메인의 마지막 관측 시각 — stamp 도메인과 혼용 비교 금지.
        self._last_wheel_seen_s = None
        self._last_imu_seen_s = None
        self._last_twist = None
        self._velocity = (0.0, 0.0)
        self._imu_valid = False
        self._imu_rate_rad_s = (0.0, 0.0, 0.0)
        self._frozen = False
        self._wheel_reinitialized = False
        self._imu_reinitialized = False
        self._reconnect_count = 0
        self._wheel_stationary = None
        self._consistency = self._consistency_monitor.evaluate(())
        self._yaw_source = "none"

    def pose(self):
        """Compatibility view for the former ``OdometryIntegrator`` owner."""
        return (self._x_m, self._y_m, self._yaw_rad)

    @property
    def last_twist(self):
        return self._last_twist

    @property
    def bias_count(self):
        return self._bias_count

    @property
    def tilt_initialized(self):
        return self._tilt_initialized

    @property
    def corrected_gyro_rad_s(self):
        return self._imu_rate_rad_s

    @property
    def filtered_accel_m_s2(self):
        return self._accel_filtered

    def snapshot(self, *, now_s: float) -> StateSnapshot:
        if not _finite(now_s):
            raise ValueError("now_s must be finite")
        now_s = float(now_s)
        wheel_stale = (
            self._last_wheel_seen_s is None
            or now_s - self._last_wheel_seen_s > self.config.sample_timeout_s
        )
        imu_stale = (
            not self._imu_valid
            or self._last_imu_seen_s is None
            or now_s - self._last_imu_seen_s > self.config.sample_timeout_s
        )
        if wheel_stale and self._last_wheel_seen_s is not None:
            self._freeze()

        if not imu_stale and self.config.use_imu_yaw:
            yaw_source = "imu"
            yaw_rate = self._imu_rate_rad_s[2]
        elif not wheel_stale and self._last_twist is not None:
            yaw_source = "wheel"
            yaw_rate = self._last_twist.omega
        else:
            yaw_source = "none"
            yaw_rate = 0.0

        diagnostics = self._diagnostic_snapshot(self._consistency)
        return StateSnapshot(
            pose=PoseSnapshot(self._x_m, self._y_m, self._yaw_rad),
            velocity=VelocitySnapshot(
                self._velocity[0],
                self._velocity[1],
                yaw_rate,
            ),
            tilt=TiltSnapshot(self._roll_rad, self._pitch_rad),
            distance_m=self._distance_m,
            diagnostics=diagnostics,
            stale=wheel_stale,
            wheel_stale=wheel_stale,
            imu_stale=imu_stale,
            initialized=self._last_wheel_stamp_s is not None,
            reinitialized=(
                self._wheel_reinitialized or self._imu_reinitialized
            ),
            reconnect_count=self._reconnect_count,
            yaw_source=yaw_source,
            gyro_bias_rad_s=tuple(self._gyro_bias),
        )

    def _diagnostic_snapshot(self, result: WheelConsistencyResult):
        warnings = tuple(result.warnings)
        warning_codes = tuple(sorted({warning.code for warning in warnings}))
        one_wheel = tuple(
            warning
            for warning in warnings
            if warning.code in ("single_wheel_spin", "single_wheel_stop")
        )
        relevant = one_wheel or warnings
        affected_wheels = tuple(sorted({
            wheel
            for warning in relevant
            for wheel in warning.wheels
        }))
        stuck_candidate = any(
            warning.code == "single_wheel_stop"
            or (
                warning.code == "response_ratio"
                and warning.value < warning.threshold
            )
            for warning in warnings
        )
        return DiagnosticSnapshot(
            slip_candidate=bool(warnings),
            stuck_candidate=stuck_candidate,
            one_wheel_mismatch=bool(one_wheel),
            warning_codes=warning_codes,
            affected_wheels=affected_wheels,
            terrain_profile=self.config.terrain_profile,
            terrain_speed_cap=result.terrain_speed_cap,
            wheel_yaw_rate_rad_s=result.wheel_yaw_rate_rad_s,
            imu_yaw_rate_rad_s=result.imu_yaw_rate_rad_s,
        )
