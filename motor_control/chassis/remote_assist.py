"""Pure remote-assist composition after teleop authority selection."""

from dataclasses import dataclass
import math


_FUTURE_TOLERANCE_S = 0.1
_NEUTRAL_LINEAR_M_S = 0.02
_NEUTRAL_ANGULAR_RAD_S = 0.05


@dataclass(frozen=True)
class AssistCorrection:
    stamp_s: float
    omega_correction_rad_s: float
    speed_cap_m_s: float
    confidence: float


@dataclass(frozen=True)
class AssistConfig:
    correction_stale_s: float = 0.5
    min_confidence: float = 0.25
    max_omega_correction_rad_s: float = 0.4
    degraded_speed_scale: float = 0.6
    bypass_stale_s: float = 0.5


@dataclass(frozen=True)
class AssistResult:
    v_m_s: float
    omega_rad_s: float
    applied: bool
    reasons: tuple[str, ...]


def _raw(operator_v, operator_omega, reason):
    return AssistResult(
        v_m_s=operator_v,
        omega_rad_s=operator_omega,
        applied=False,
        reasons=(reason,),
    )


def _signed_cap(value, cap):
    magnitude = min(abs(value), max(0.0, cap))
    return math.copysign(magnitude, value)


def _degraded(operator_v, operator_omega, config, profile_max_speed, reason):
    if config.degraded_speed_scale == 0.0:
        cap = 0.0
    else:
        cap = profile_max_speed * config.degraded_speed_scale
    return AssistResult(
        v_m_s=_signed_cap(operator_v, cap),
        omega_rad_s=operator_omega,
        applied=False,
        reasons=(reason,),
    )


def _validate_inputs(operator_v, operator_omega, now_s, config, profile_max_speed):
    values = {
        "operator_v": operator_v,
        "operator_omega": operator_omega,
    }
    for name, value in values.items():
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ValueError("%s must be finite" % name)
        if not math.isfinite(float(value)):
            raise ValueError("%s must be finite" % name)

    if not isinstance(now_s, (int, float)) or not math.isfinite(float(now_s)):
        raise ValueError("now_s must be finite")

    config_values = {
        "correction_stale_s": config.correction_stale_s,
        "min_confidence": config.min_confidence,
        "max_omega_correction_rad_s": config.max_omega_correction_rad_s,
        "degraded_speed_scale": config.degraded_speed_scale,
        "bypass_stale_s": config.bypass_stale_s,
    }
    for name, value in config_values.items():
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise ValueError("config.%s must be finite" % name)
    if config.correction_stale_s < 0.0 or config.bypass_stale_s < 0.0:
        raise ValueError("stale thresholds must be non-negative")
    if not 0.0 <= config.min_confidence <= 1.0:
        raise ValueError("config.min_confidence must be in [0, 1]")
    if config.max_omega_correction_rad_s < 0.0:
        raise ValueError("config.max_omega_correction_rad_s must be non-negative")
    if not 0.0 <= config.degraded_speed_scale <= 1.0:
        raise ValueError("config.degraded_speed_scale must be in [0, 1]")

    if not isinstance(profile_max_speed, (int, float)) or isinstance(
        profile_max_speed,
        bool,
    ):
        raise ValueError("profile_max_speed_m_s must be non-negative")
    if math.isnan(float(profile_max_speed)) or profile_max_speed < 0.0:
        raise ValueError("profile_max_speed_m_s must be non-negative")


def _correction_invalid(correction):
    numeric = (
        correction.stamp_s,
        correction.omega_correction_rad_s,
        correction.speed_cap_m_s,
        correction.confidence,
    )
    if any(
        not isinstance(value, (int, float)) or isinstance(value, bool)
        for value in numeric
    ):
        return True
    if not math.isfinite(float(correction.stamp_s)):
        return True
    if not math.isfinite(float(correction.omega_correction_rad_s)):
        return True
    if math.isnan(float(correction.speed_cap_m_s)):
        return True
    if correction.speed_cap_m_s == -math.inf:
        return True
    if not math.isfinite(float(correction.confidence)):
        return True
    return not 0.0 <= correction.confidence <= 1.0


def compose(
    operator_v,
    operator_omega,
    *,
    now_s,
    correction,
    bypass_active,
    bypass_stamp_s,
    enabled,
    config,
    profile_max_speed_m_s,
):
    """Compose bounded terrain assistance into an already-selected teleop command."""
    _validate_inputs(
        operator_v,
        operator_omega,
        now_s,
        config,
        profile_max_speed_m_s,
    )
    operator_v = float(operator_v)
    operator_omega = float(operator_omega)
    now_s = float(now_s)
    profile_max_speed_m_s = float(profile_max_speed_m_s)

    if not enabled:
        return _raw(operator_v, operator_omega, "assist_disabled")

    if bypass_stamp_s is None:
        return _raw(operator_v, operator_omega, "bypass_unknown")
    try:
        bypass_stamp_s = float(bypass_stamp_s)
    except (TypeError, ValueError):
        return _raw(operator_v, operator_omega, "bypass_unknown")
    if not math.isfinite(bypass_stamp_s):
        return _raw(operator_v, operator_omega, "bypass_unknown")
    bypass_age_s = now_s - bypass_stamp_s
    if (
        bypass_age_s > config.bypass_stale_s
        or bypass_age_s < -_FUTURE_TOLERANCE_S
    ):
        return _raw(operator_v, operator_omega, "bypass_unknown")
    if bypass_active:
        return _raw(operator_v, operator_omega, "assist_bypass")

    if (
        abs(operator_v) <= _NEUTRAL_LINEAR_M_S
        and abs(operator_omega) <= _NEUTRAL_ANGULAR_RAD_S
    ):
        return _raw(operator_v, operator_omega, "operator_neutral")

    if correction is None:
        return _degraded(
            operator_v,
            operator_omega,
            config,
            profile_max_speed_m_s,
            "correction_missing",
        )
    if _correction_invalid(correction):
        return _degraded(
            operator_v,
            operator_omega,
            config,
            profile_max_speed_m_s,
            "correction_stale",
        )

    correction_age_s = now_s - float(correction.stamp_s)
    if (
        correction_age_s > config.correction_stale_s
        or correction_age_s < -_FUTURE_TOLERANCE_S
    ):
        return _degraded(
            operator_v,
            operator_omega,
            config,
            profile_max_speed_m_s,
            "correction_stale",
        )
    if correction.confidence < config.min_confidence:
        return _degraded(
            operator_v,
            operator_omega,
            config,
            profile_max_speed_m_s,
            "low_confidence",
        )

    correction_limit = config.max_omega_correction_rad_s
    bounded_correction = max(
        -correction_limit,
        min(correction_limit, float(correction.omega_correction_rad_s)),
    )
    speed_cap = min(
        max(0.0, float(correction.speed_cap_m_s)),
        profile_max_speed_m_s,
    )
    return AssistResult(
        v_m_s=_signed_cap(operator_v, speed_cap),
        omega_rad_s=operator_omega + bounded_correction,
        applied=True,
        reasons=(),
    )
