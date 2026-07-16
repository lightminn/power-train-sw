"""Deterministic ROS-free WP6-C terrain-path controller."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING

from .profiles import DriveProfile

if TYPE_CHECKING:
    from ..terrain import TerrainEstimate


_FUTURE_TOLERANCE_S = 0.1


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
    kp_heading: float = 1.2
    kp_offset: float = 0.8
    curvature_slow_k: float = 1.0
    clearance_hold_m: float = 0.05
    clearance_full_m: float = 0.30
    min_confidence: float = 0.25
    full_confidence: float = 0.6
    confidence_floor_scale: float = 0.4
    slip_scale: float = 0.5

    def __post_init__(self) -> None:
        positive = (
            "terrain_stale_s",
            "motion_stale_s",
            "gate_stale_s",
            "diagnostics_stale_s",
            "kp_heading",
            "kp_offset",
            "curvature_slow_k",
            "clearance_hold_m",
            "clearance_full_m",
            "min_confidence",
            "full_confidence",
        )
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.clearance_hold_m >= self.clearance_full_m:
            raise ValueError("clearance_hold_m must be below clearance_full_m")
        if not self.min_confidence < self.full_confidence <= 1.0:
            raise ValueError("confidence thresholds must be ordered within 0..1")
        for name in ("confidence_floor_scale", "slip_scale"):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise ValueError(f"{name} must be within (0, 1]")


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

    def _dt(self, now_s: float) -> float:
        dt = 0.0
        if self._last_stamp_s is not None and now_s >= self._last_stamp_s:
            dt = now_s - self._last_stamp_s
        self._last_stamp_s = now_s
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
        self._last_stamp_s = now_s
        return self._decision(now_s, "BLOCKED", reasons)

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
                terrain.left_wheel_clearance_m,
                terrain.right_wheel_clearance_m,
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
                if min(
                    terrain.left_wheel_clearance_m,
                    terrain.right_wheel_clearance_m,
                ) < self.config.clearance_hold_m:
                    hold_reasons.append("clearance_low")
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
            return self._decision(now_s, "CONTROLLED_HOLD", hold_reasons)

        reasons: list[str] = []
        clearance = min(
            terrain.left_wheel_clearance_m,
            terrain.right_wheel_clearance_m,
        )
        clearance_scale = _clamp(
            (clearance - self.config.clearance_hold_m)
            / (self.config.clearance_full_m - self.config.clearance_hold_m),
            0.0,
            1.0,
        )
        if clearance_scale < 1.0:
            reasons.append("clearance_slow")

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

        scales = [clearance_scale, bank_scale, slope_scale, confidence_scale]
        if diagnostics_fresh and diagnostics.slip_candidate:
            scales.append(self.config.slip_scale)
            reasons.append("slip_candidate")
        v_lim = self.profile.max_speed_m_s * min(scales)
        if diagnostics_fresh and math.isfinite(diagnostics.speed_cap_m_s):
            speed_cap = max(0.0, diagnostics.speed_cap_m_s)
            if speed_cap < v_lim:
                reasons.append("speed_cap")
                v_lim = speed_cap

        omega_raw = (
            self.config.kp_heading * terrain.heading_error_rad
            + self.config.kp_offset * terrain.path_offset_m
        )
        if not math.isfinite(omega_raw):
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
            return self._decision(now_s, "CONTROLLED_HOLD", ("control_nonfinite",))
        omega_target = _clamp(
            omega_raw,
            -self.profile.max_yaw_rate_rad_s,
            self.profile.max_yaw_rate_rad_s,
        )
        if omega_target != omega_raw:
            reasons.append("yaw_rate_limited")
        if omega_raw:
            reasons.append("curvature_slow")
        v_target = max(
            0.0,
            v_lim / (1.0 + self.config.curvature_slow_k * abs(omega_raw)),
        )

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
