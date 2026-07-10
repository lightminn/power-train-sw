from dataclasses import dataclass
from typing import Optional

CHECKING = "CHECKING"
VALID = "VALID"
INVALID_READING = "INVALID_READING"
NO_RESPONSE = "NO_RESPONSE"


@dataclass(frozen=True)
class SensorReading:
    status: str
    distance_mm: Optional[float]
    detail: str = ""


@dataclass(frozen=True)
class Verdict:
    status: str
    distance_mm: Optional[float]
    estop_required: bool
    consecutive_failures: int
    detail: str = ""
