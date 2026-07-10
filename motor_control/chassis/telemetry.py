from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class WheelSnapshot:
    name: str
    corner_mode: str
    drive_turns_per_s: float
    steer_deg: float
    drive_current_a: float
    steer_current_a: float
    drive_stale: bool
    steer_stale: bool
    drive_axis_error: int
    steer_fault: int


@dataclass(frozen=True)
class ChassisSnapshot:
    chassis_mode: str
    stop_state: str
    healthy: bool
    wheels: Tuple[WheelSnapshot, ...]
