"""Pinhole camera model shared by the detector and the projector."""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(v) for v in values) or self.fx <= 0 or self.fy <= 0:
            raise ValueError("camera intrinsics must be finite with positive focal length")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera size must be positive")

    @classmethod
    def from_k(cls, k, width: int, height: int) -> "CameraModel":
        """Build from a row-major 3x3 ROS CameraInfo ``k``."""
        return cls(
            fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]),
            width=int(width), height=int(height),
        )
