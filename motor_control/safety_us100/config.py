from dataclasses import dataclass


@dataclass
class SafetyConfig:
    warn_mm: float = 400.0
    stop_mm: float = 200.0
    hysteresis_mm: float = 30.0
    fail_stop_count: int = 3
    port: str = "/dev/ttyTHS1"
    baud: int = 9600
