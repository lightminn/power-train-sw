from dataclasses import dataclass
from typing import Optional

SAFE = "safe"
WARN = "warn"
STOP = "stop"


@dataclass(frozen=True)
class Verdict:
    level: str
    distance_mm: Optional[float]
