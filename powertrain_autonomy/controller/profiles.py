"""Provisional WP6-C drive profiles with no ROS dependency."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DriveProfile:
    name: str
    required_arm_status: str
    max_speed_m_s: float
    max_accel_m_s2: float
    max_decel_m_s2: float
    max_yaw_rate_rad_s: float
    max_yaw_accel_rad_s2: float
    max_bank_rad: float
    soft_bank_rad: float
    max_slope_rad: float
    soft_slope_rad: float


# PROVISIONAL before braking, bank, slope, and payload HIL qualification.
EMPTY_STOWED = DriveProfile(
    name="EMPTY_STOWED",
    required_arm_status="STOWED_LOCKED",
    max_speed_m_s=0.8,
    max_accel_m_s2=0.5,
    max_decel_m_s2=0.8,
    max_yaw_rate_rad_s=0.8,
    max_yaw_accel_rad_s2=1.5,
    max_bank_rad=math.radians(15.0),
    soft_bank_rad=math.radians(8.0),
    max_slope_rad=math.radians(15.0),
    soft_slope_rad=math.radians(10.0),
)

# PROVISIONAL before braking, bank, slope, and payload HIL qualification.
CARRYING_LOCKED = DriveProfile(
    name="CARRYING_LOCKED",
    required_arm_status="CARRYING_LOCKED",
    max_speed_m_s=0.5,
    max_accel_m_s2=0.3,
    max_decel_m_s2=0.6,
    max_yaw_rate_rad_s=0.5,
    max_yaw_accel_rad_s2=1.0,
    max_bank_rad=math.radians(10.0),
    soft_bank_rad=math.radians(5.0),
    max_slope_rad=math.radians(12.0),
    soft_slope_rad=math.radians(8.0),
)


_PROFILE_FIELDS = (
    "max_speed_m_s",
    "max_accel_m_s2",
    "max_decel_m_s2",
    "max_yaw_rate_rad_s",
    "max_yaw_accel_rad_s2",
    "max_bank_rad",
    "soft_bank_rad",
    "max_slope_rad",
    "soft_slope_rad",
)


def validate_carrying_profile_invariant(
    empty: DriveProfile = EMPTY_STOWED,
    carrying: DriveProfile = CARRYING_LOCKED,
) -> None:
    """Raise when a carrying limit is less conservative than the empty limit."""
    for field in _PROFILE_FIELDS:
        if getattr(carrying, field) > getattr(empty, field):
            raise ValueError(f"carrying profile violates {field}")


def profile_by_name(name: str) -> DriveProfile:
    profiles = {
        EMPTY_STOWED.name: EMPTY_STOWED,
        CARRYING_LOCKED.name: CARRYING_LOCKED,
    }
    try:
        return profiles[name]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "drive_profile must be EMPTY_STOWED or CARRYING_LOCKED"
        ) from exc


validate_carrying_profile_invariant()
