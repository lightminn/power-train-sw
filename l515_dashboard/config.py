"""Configuration contract for the L515 dashboard runtime."""

from dataclasses import dataclass
import math
from numbers import Real


ENCODERS = ("x264", "openh264")


@dataclass(frozen=True)
class DashboardConfig:
    """Immutable runtime settings shared by dashboard components."""

    port: int = 5000
    latency_ms: int = 60
    encoder: str = "x264"
    width: int = 640
    height: int = 480
    fps: int = 30
    bitrate_kbps: int = 3000
    startup_timeout_s: float = 10.0
    graceful_timeout_s: float = 3.0
    termination_timeout_s: float = 2.0

    def __post_init__(self) -> None:
        if isinstance(self.port, bool) or not isinstance(self.port, int):
            raise ValueError("port must be an integer")
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if self.encoder not in ENCODERS:
            raise ValueError(f"encoder must be one of {ENCODERS}: {self.encoder!r}")

        positive_integer_fields = (
            "latency_ms",
            "width",
            "height",
            "fps",
            "bitrate_kbps",
        )
        for name in positive_integer_fields:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")

        positive_real_fields = (
            "startup_timeout_s",
            "graceful_timeout_s",
            "termination_timeout_s",
        )
        for name in positive_real_fields:
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} must be a finite positive number")
