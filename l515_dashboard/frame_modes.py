"""Video rendering and bounded latest-frame handoff for the L515 dashboard."""

from enum import Enum
from threading import Lock
from typing import Optional

import cv2
import numpy as np


_MAX_DEPTH_MM = 5000


class FrameMode(str, Enum):
    COLOR = "color"
    DEPTH = "depth"
    SIDE_BY_SIDE = "side_by_side"


def _sized(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    if frame.shape[:2] == (height, width):
        return frame
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_NEAREST)


def _render_depth(depth: np.ndarray, width: int, height: int) -> np.ndarray:
    sized = _sized(depth, width, height)
    clipped = np.clip(sized, 0, _MAX_DEPTH_MM)
    normalized = np.rint(
        clipped.astype(np.float32) * (255.0 / _MAX_DEPTH_MM)
    )
    colored = cv2.applyColorMap(
        normalized.astype(np.uint8), cv2.COLORMAP_TURBO
    )
    colored[sized == 0] = 0
    return colored


def render_frame(
    mode: FrameMode,
    color: Optional[np.ndarray],
    depth: Optional[np.ndarray],
    width: int,
    height: int,
) -> Optional[np.ndarray]:
    """Render the selected output, or ``None`` when an input is absent."""
    mode = FrameMode(mode)
    if mode is FrameMode.COLOR:
        if color is None:
            return None
        return np.ascontiguousarray(_sized(color, width, height))

    if mode is FrameMode.DEPTH:
        if depth is None:
            return None
        return np.ascontiguousarray(
            _render_depth(depth, width, height)
        )

    if color is None or depth is None:
        return None
    rendered_color = np.ascontiguousarray(_sized(color, width, height))
    rendered_depth = _render_depth(depth, width, height)
    return np.ascontiguousarray(np.hstack((rendered_color, rendered_depth)))


class LatestVideoFrames:
    """Thread-safe color/depth slots that are consumed at most once."""

    def __init__(self, width: int = 640, height: int = 480) -> None:
        self._width = width
        self._height = height
        self._color: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None
        self._lock = Lock()

    def put_color(self, frame: np.ndarray) -> None:
        with self._lock:
            self._color = frame.copy()

    def put_depth(self, frame: np.ndarray) -> None:
        with self._lock:
            self._depth = frame.copy()

    def take(self, mode: FrameMode) -> Optional[np.ndarray]:
        mode = FrameMode(mode)
        with self._lock:
            color = self._color
            depth = self._depth
            self._color = None
            self._depth = None

        return render_frame(mode, color, depth, self._width, self._height)
