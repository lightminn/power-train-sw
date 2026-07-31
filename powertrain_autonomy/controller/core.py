"""Deterministic ROS-free WP6-C terrain-path controller."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

from ..validation import (
    require_all_finite, require_int_at_least, require_non_negative,
    require_ordered,
)
from .profiles import EMPTY_STOWED, DriveProfile

if TYPE_CHECKING:
    from ..terrain import TerrainEstimate


_FUTURE_TOLERANCE_S = 0.1
# curvature_slow 를 보고할 최소 감속 — 속도를 1% 미만으로 깎는 조향은 사유로
# 남기지 않는다(보고 임계일 뿐 감속식 자체는 바뀌지 않는다).
_CURVATURE_SLOW_REPORT_FRACTION = 0.01
ASSIST_MAX_OMEGA_CORRECTION_RAD_S = 0.4


@dataclass(frozen=True)
class MotionState:
    stamp_s: float
    forward_m_s: float
    yaw_rate_rad_s: float
    roll_rad: float
    pitch_rad: float


@dataclass(frozen=True)
class DriveDiagnostics:
    stamp_s: float
    slip_candidate: bool
    stuck_candidate: bool
    speed_cap_m_s: float


@dataclass(frozen=True)
class ProfileGate:
    stamp_s: float
    status: str


@dataclass(frozen=True)
class AutonomyControllerConfig:
    terrain_stale_s: float = 0.45
    motion_stale_s: float = 0.30
    gate_stale_s: float = 0.50
    diagnostics_stale_s: float = 1.0
    recovery_ticks: int = 3
    # A3(스펙 r6 §4.4): 복귀는 틱 수 AND 최소 경과시간 AND 신선 표본 수를
    # 모두 요구한다 — 부하로 틱 주기가 변해도 dwell 의미가 보존된다.
    recovery_min_elapsed_s: float = 0.15
    recovery_min_samples: int = 3
    kp_heading: float = 1.2
    kp_offset: float = 0.8
    curvature_slow_k: float = 1.0
    # 0.0 preserves the pure-P baseline for tests/backward compatibility.
    # Production enables the clothoid fix through autonomy_controller_node's
    # kd_yaw=0.5 parameter; direct construction without a config is undamped.
    kd_yaw: float = 0.0
    yaw_damp_gate_rad_s: float = 0.25
    yaw_damp_tau_s: float = 0.7
    min_confidence: float = 0.25
    full_confidence: float = 0.6
    confidence_floor_scale: float = 0.4
    slip_scale: float = 0.5

    def __post_init__(self) -> None:
        require_int_at_least(self.recovery_ticks, 1, "recovery_ticks must be a positive integer")
        samples_message = "recovery_min_samples must be a positive integer"
        require_int_at_least(self.recovery_min_samples, 1, samples_message)
        recovery_elapsed_message = "recovery_min_elapsed_s must be a finite non-negative number"
        if not isinstance(self.recovery_min_elapsed_s, (int, float)):
            raise ValueError(recovery_elapsed_message)
        require_non_negative(self.recovery_min_elapsed_s, recovery_elapsed_message)
        positive = (
            "terrain_stale_s", "motion_stale_s", "gate_stale_s",
            "diagnostics_stale_s", "kp_heading", "kp_offset",
            "curvature_slow_k", "yaw_damp_gate_rad_s", "yaw_damp_tau_s",
            "min_confidence", "full_confidence",
        )
        for name in positive:
            value = getattr(self, name)
            message = f"{name} must be finite and positive"
            require_all_finite((value,), message)
            require_ordered(0.0, value, message)
        kd_yaw_message = "kd_yaw must be finite and non-negative"
        if not isinstance(self.kd_yaw, (int, float)):
            raise ValueError(kd_yaw_message)
        require_non_negative(self.kd_yaw, kd_yaw_message)
        confidence_message = "confidence thresholds must be ordered within 0..1"
        require_ordered(self.min_confidence, self.full_confidence, confidence_message)
        if self.full_confidence > 1.0:
            raise ValueError(confidence_message)
        for name in ("confidence_floor_scale", "slip_scale"):
            value = getattr(self, name)
            message = f"{name} must be within (0, 1]"
            require_all_finite((value,), message)
            if not 0.0 < value <= 1.0:
                raise ValueError(message)


@dataclass(frozen=True)
class ControllerDecision:
    stamp_s: float
    v_m_s: float
    omega_rad_s: float
    state: str
    reasons: tuple[str, ...]


def _finite(values) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def _clamp(value: float, low: float, high: float) -> float:
    return min(high, max(low, value))


def _scale_down(value: float, soft: float, hard: float) -> float:
    if value <= soft:
        return 1.0
    if value >= hard:
        return 0.0
    return (hard - value) / (hard - soft)


def _slew(current: float, target: float, rise_rate: float, fall_rate: float, dt: float) -> float:
    if target >= current:
        return min(target, current + rise_rate * dt)
    return max(target, current - fall_rate * dt)


def assist_correction_from_terrain(
    terrain: TerrainEstimate | None,
    config: AutonomyControllerConfig,
) -> tuple[float, float, float] | None:
    """Return bounded heading correction, conservative cap, and confidence."""
    if terrain is None or not terrain.path_available:
        return None
    values = (
        terrain.stamp_s,
        terrain.path_offset_m,
        terrain.heading_error_rad,
        terrain.bank_angle_rad,
        terrain.longitudinal_slope_rad,
        terrain.confidence,
    )
    if not _finite(values) or not 0.0 <= terrain.confidence <= 1.0:
        return None

    bank_scale = _scale_down(
        abs(terrain.bank_angle_rad),
        EMPTY_STOWED.soft_bank_rad,
        EMPTY_STOWED.max_bank_rad,
    )
    slope_scale = _scale_down(
        abs(terrain.longitudinal_slope_rad),
        EMPTY_STOWED.soft_slope_rad,
        EMPTY_STOWED.max_slope_rad,
    )
    confidence_scale = 1.0
    if terrain.confidence < config.full_confidence:
        fraction = (
            terrain.confidence - config.min_confidence
        ) / (config.full_confidence - config.min_confidence)
        confidence_scale = config.confidence_floor_scale + (
            1.0 - config.confidence_floor_scale
        ) * _clamp(fraction, 0.0, 1.0)

    omega_raw = (
        config.kp_heading * terrain.heading_error_rad
        + config.kp_offset * terrain.path_offset_m
    )
    omega_correction = _clamp(
        omega_raw,
        -ASSIST_MAX_OMEGA_CORRECTION_RAD_S,
        ASSIST_MAX_OMEGA_CORRECTION_RAD_S,
    )
    # Manual assist has no payload profile; EMPTY_STOWED is the conservative
    # fixed basis until braking, bank, and slope HIL qualifies another cap.
    speed_cap = EMPTY_STOWED.max_speed_m_s * (
        bank_scale
        * slope_scale
        * confidence_scale
    )
    return omega_correction, speed_cap, terrain.confidence


class AutonomyController:
    def __init__(
        self,
        profile: DriveProfile,
        config: AutonomyControllerConfig | None = None,
    ) -> None:
        self.profile = profile
        self.config = config or AutonomyControllerConfig()
        self._last_stamp_s: float | None = None
        self._v_m_s = 0.0
        self._omega_rad_s = 0.0
        self._turn_activity = 0.0
        self._recovering_from_hold = False
        self._recovery_fresh_ticks = 0
        self._recovery_started_s: float | None = None
        self._recovery_samples = 0
        self._recovery_last_sample_stamp: float | None = None

    def _dt(self, now_s: float) -> float:
        if self._last_stamp_s is None:
            self._last_stamp_s = now_s
            return 0.0
        dt = min(0.25, max(0.0, now_s - self._last_stamp_s))
        self._last_stamp_s = max(self._last_stamp_s, now_s)
        return dt

    def _decision(self, now_s: float, state: str, reasons) -> ControllerDecision:
        return ControllerDecision(
            stamp_s=now_s,
            v_m_s=max(0.0, self._v_m_s),
            omega_rad_s=self._omega_rad_s,
            state=state,
            reasons=tuple(dict.fromkeys(reasons)),
        )

    def _blocked(self, now_s: float, reasons) -> ControllerDecision:
        self._v_m_s = 0.0
        self._omega_rad_s = 0.0
        self._turn_activity = 0.0
        self._recovering_from_hold = False
        self._recovery_fresh_ticks = 0
        self._recovery_started_s = None
        self._recovery_samples = 0
        self._recovery_last_sample_stamp = None
        if self._last_stamp_s is None:
            self._last_stamp_s = now_s
        else:
            self._last_stamp_s = max(self._last_stamp_s, now_s)
        return self._decision(now_s, "BLOCKED", reasons)

    def _controlled_hold(
        self,
        now_s: float,
        reasons,
        dt: float,
        *,
        reset_recovery: bool,
    ) -> ControllerDecision:
        self._turn_activity = 0.0
        if reset_recovery:
            self._recovering_from_hold = True
            self._recovery_fresh_ticks = 0
            self._recovery_started_s = None
            self._recovery_samples = 0
            self._recovery_last_sample_stamp = None
        self._v_m_s = _slew(
            self._v_m_s,
            0.0,
            self.profile.max_accel_m_s2,
            self.profile.max_decel_m_s2,
            dt,
        )
        self._omega_rad_s = _slew(
            self._omega_rad_s,
            0.0,
            self.profile.max_yaw_accel_rad_s2,
            self.profile.max_yaw_accel_rad_s2,
            dt,
        )
        return self._decision(now_s, "CONTROLLED_HOLD", reasons)

    def decide(
        self,
        now_s: float,
        *,
        terrain: TerrainEstimate | None,
        motion: MotionState | None,
        gate: ProfileGate | None,
        diagnostics: DriveDiagnostics | None,
    ) -> ControllerDecision:
        if not _finite((now_s,)):
            raise ValueError("now_s must be finite")

        blocked_reasons: list[str] = []
        if gate is None:
            blocked_reasons.append("gate_missing")
        else:
            if not _finite((gate.stamp_s,)):
                blocked_reasons.append("gate_nonfinite")
            elif gate.stamp_s > now_s + _FUTURE_TOLERANCE_S:
                blocked_reasons.append("gate_future")
            elif now_s - gate.stamp_s > self.config.gate_stale_s:
                blocked_reasons.append("gate_stale")
            if gate.status != self.profile.required_arm_status:
                blocked_reasons.append("arm_status_mismatch")
        if blocked_reasons:
            return self._blocked(now_s, blocked_reasons)

        dt = self._dt(now_s)
        hold_reasons: list[str] = []
        terrain_valid = terrain is not None
        if terrain is None:
            hold_reasons.append("terrain_missing")
        else:
            terrain_values = (
                terrain.stamp_s,
                terrain.path_offset_m,
                terrain.heading_error_rad,
                terrain.bank_angle_rad,
                terrain.longitudinal_slope_rad,
                terrain.confidence,
            )
            if not _finite(terrain_values):
                hold_reasons.append("terrain_nonfinite")
                terrain_valid = False
            elif terrain.stamp_s > now_s + _FUTURE_TOLERANCE_S:
                hold_reasons.append("terrain_future")
            elif now_s - terrain.stamp_s > self.config.terrain_stale_s:
                hold_reasons.append("terrain_stale")
            if terrain_valid:
                if not terrain.path_available:
                    hold_reasons.append("path_unavailable")
                if terrain.confidence < self.config.min_confidence:
                    hold_reasons.append("low_confidence")
                if abs(terrain.bank_angle_rad) > self.profile.max_bank_rad:
                    hold_reasons.append("bank_limit")
                if abs(terrain.longitudinal_slope_rad) > self.profile.max_slope_rad:
                    hold_reasons.append("slope_limit")

        motion_valid = motion is not None
        if motion is None:
            hold_reasons.append("motion_missing")
        else:
            if not _finite(
                (
                    motion.stamp_s,
                    motion.forward_m_s,
                    motion.yaw_rate_rad_s,
                    motion.roll_rad,
                    motion.pitch_rad,
                )
            ):
                hold_reasons.append("motion_nonfinite")
                motion_valid = False
            elif motion.stamp_s > now_s + _FUTURE_TOLERANCE_S:
                hold_reasons.append("motion_future")
            elif now_s - motion.stamp_s > self.config.motion_stale_s:
                hold_reasons.append("motion_stale")
            if motion_valid:
                if abs(motion.roll_rad) > self.profile.max_bank_rad:
                    hold_reasons.append("roll_limit")
                if abs(motion.pitch_rad) > self.profile.max_slope_rad:
                    hold_reasons.append("pitch_limit")

        diagnostics_fresh = bool(
            diagnostics is not None
            and _finite((diagnostics.stamp_s,))
            and diagnostics.stamp_s <= now_s + _FUTURE_TOLERANCE_S
            and now_s - diagnostics.stamp_s <= self.config.diagnostics_stale_s
        )
        if diagnostics_fresh and diagnostics.stuck_candidate:
            hold_reasons.append("stuck_candidate")

        if hold_reasons:
            return self._controlled_hold(
                now_s,
                hold_reasons,
                dt,
                reset_recovery=True,
            )

        if self._recovering_from_hold:
            self._recovery_fresh_ticks += 1
            if self._recovery_started_s is None:
                self._recovery_started_s = now_s
            if terrain.stamp_s != self._recovery_last_sample_stamp:
                self._recovery_last_sample_stamp = terrain.stamp_s
                self._recovery_samples += 1
            elapsed_ok = (
                now_s - self._recovery_started_s
                >= self.config.recovery_min_elapsed_s
            )
            if not (
                self._recovery_fresh_ticks >= self.config.recovery_ticks
                and elapsed_ok
                and self._recovery_samples >= self.config.recovery_min_samples
            ):
                return self._controlled_hold(
                    now_s,
                    ("recovery_dwell",),
                    dt,
                    reset_recovery=False,
                )
            self._recovering_from_hold = False
            self._recovery_fresh_ticks = 0
            self._recovery_started_s = None
            self._recovery_samples = 0
            self._recovery_last_sample_stamp = None

        reasons: list[str] = []
        bank = max(abs(terrain.bank_angle_rad), abs(motion.roll_rad))
        bank_scale = _scale_down(
            bank,
            self.profile.soft_bank_rad,
            self.profile.max_bank_rad,
        )
        if bank_scale < 1.0:
            reasons.append("bank_slow")

        slope = max(
            abs(terrain.longitudinal_slope_rad),
            abs(motion.pitch_rad),
        )
        slope_scale = _scale_down(
            slope,
            self.profile.soft_slope_rad,
            self.profile.max_slope_rad,
        )
        if slope_scale < 1.0:
            reasons.append("slope_slow")

        confidence_scale = 1.0
        if terrain.confidence < self.config.full_confidence:
            fraction = (
                terrain.confidence - self.config.min_confidence
            ) / (
                self.config.full_confidence - self.config.min_confidence
            )
            confidence_scale = self.config.confidence_floor_scale + (
                1.0 - self.config.confidence_floor_scale
            ) * _clamp(fraction, 0.0, 1.0)
            reasons.append("confidence_slow")

        scales = [bank_scale, slope_scale, confidence_scale]
        if diagnostics_fresh and diagnostics.slip_candidate:
            scales.append(self.config.slip_scale)
            reasons.append("slip_candidate")
        v_lim = self.profile.max_speed_m_s * min(scales)
        if diagnostics_fresh and math.isfinite(diagnostics.speed_cap_m_s):
            speed_cap = max(0.0, diagnostics.speed_cap_m_s)
            if speed_cap < v_lim:
                reasons.append("speed_cap")
                v_lim = speed_cap
        omega_p = (
            self.config.kp_heading * terrain.heading_error_rad
            + self.config.kp_offset * terrain.path_offset_m
        )
        if self.config.kd_yaw > 0.0:
            alpha = dt / (self.config.yaw_damp_tau_s + dt)
            self._turn_activity += (
                abs(omega_p) - self._turn_activity
            ) * alpha
            gate = _clamp(
                self._turn_activity / self.config.yaw_damp_gate_rad_s,
                0.0,
                1.0,
            )
            omega_raw = (
                omega_p
                - self.config.kd_yaw * motion.yaw_rate_rad_s * gate
            )
        else:
            omega_raw = omega_p
        if not math.isfinite(omega_raw):
            return self._controlled_hold(
                now_s,
                ("control_nonfinite",),
                dt,
                reset_recovery=True,
            )
        omega_target = _clamp(
            omega_raw,
            -self.profile.max_yaw_rate_rad_s,
            self.profile.max_yaw_rate_rad_s,
        )
        if omega_target != omega_raw:
            reasons.append("yaw_rate_limited")
        curvature_divisor = 1.0 + self.config.curvature_slow_k * abs(omega_p)
        # 다른 *_slow 사유는 각자의 soft 임계 아래에서 스케일이 정확히 1.0 이라
        # 조용하지만, 곡률 항에는 데드존이 없어 omega_p 가 0 이 아니기만 하면
        # 붙었다. 그래서 조향 중 거의 모든 틱에 사유가 찍히고 실제 감속량과
        # 무관해진다 — 실측 2415/3000 틱에서 감속은 2% 였고, 그 수를 병목으로
        # 오독하게 만들었다. 속도를 실제로 깎을 때만 보고한다.
        if curvature_divisor > 1.0 + _CURVATURE_SLOW_REPORT_FRACTION:
            reasons.append("curvature_slow")
        v_target = max(0.0, v_lim / curvature_divisor)

        self._v_m_s = _slew(
            self._v_m_s,
            v_target,
            self.profile.max_accel_m_s2,
            self.profile.max_decel_m_s2,
            dt,
        )
        self._omega_rad_s = _slew(
            self._omega_rad_s,
            omega_target,
            self.profile.max_yaw_accel_rad_s2,
            self.profile.max_yaw_accel_rad_s2,
            dt,
        )
        return self._decision(now_s, "TRACKING", reasons)
