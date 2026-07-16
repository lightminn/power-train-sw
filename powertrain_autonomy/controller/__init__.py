"""Public WP6-C controller contract."""

from .core import (
    AutonomyController,
    AutonomyControllerConfig,
    ControllerDecision,
    DriveDiagnostics,
    MotionState,
    ProfileGate,
    assist_correction_from_terrain,
)
from .profiles import (
    CARRYING_LOCKED,
    EMPTY_STOWED,
    DriveProfile,
    profile_by_name,
    validate_carrying_profile_invariant,
)

__all__ = [
    "AutonomyController",
    "AutonomyControllerConfig",
    "CARRYING_LOCKED",
    "ControllerDecision",
    "DriveDiagnostics",
    "DriveProfile",
    "EMPTY_STOWED",
    "MotionState",
    "ProfileGate",
    "assist_correction_from_terrain",
    "profile_by_name",
    "validate_carrying_profile_invariant",
]
