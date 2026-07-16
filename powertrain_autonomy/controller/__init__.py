"""Public WP6-C controller contract."""

from .core import (
    AutonomyController,
    AutonomyControllerConfig,
    ControllerDecision,
    DriveDiagnostics,
    MotionState,
    ProfileGate,
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
    "profile_by_name",
    "validate_carrying_profile_invariant",
]
