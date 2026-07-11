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
    width: int = 1280
    height: int = 720
    fps: int = 30
    bitrate_kbps: int = 3000
    startup_timeout_s: float = 10.0
    graceful_timeout_s: float = 3.0
    termination_timeout_s: float = 2.0
    socket_path: str = "/run/l515-gateway/gateway.sock"
    lock_path: str = "/run/l515-gateway/l515.lock"
    color_width: int = 1280
    color_height: int = 720
    depth_width: int = 640
    depth_height: int = 480
    overlay_alpha: float = 0.5
    reconnect_interval_s: float = 2.0
    max_message_bytes: int = 65536

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
            "color_width",
            "color_height",
            "depth_width",
            "depth_height",
            "max_message_bytes",
        )
        for name in positive_integer_fields:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if (self.width, self.height) != (1280, 720):
            raise ValueError("width and height must be fixed at 1280x720")
        if (self.color_width, self.color_height) != (1280, 720):
            raise ValueError("color profile must be fixed at 1280x720")
        if (self.depth_width, self.depth_height) != (640, 480):
            raise ValueError("depth profile must be fixed at 640x480")

        positive_real_fields = (
            "startup_timeout_s",
            "graceful_timeout_s",
            "termination_timeout_s",
            "reconnect_interval_s",
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
        if (
            isinstance(self.overlay_alpha, bool)
            or not isinstance(self.overlay_alpha, Real)
            or not math.isfinite(self.overlay_alpha)
            or not 0 < self.overlay_alpha <= 1
        ):
            raise ValueError("overlay_alpha must be in (0, 1]")
        for name in ("socket_path", "lock_path"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")
