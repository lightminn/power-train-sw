"""Video rendering and bounded latest-frame handoff for the L515 dashboard."""

from enum import Enum
from threading import Lock
from typing import Optional

import cv2
import numpy as np


_MAX_DEPTH_MM = 5000


class FrameMode(str, Enum):
    COLOR = "rgb"
    DEPTH = "depth"
    OVERLAY = "overlay"


def _require_color(frame: np.ndarray, width: int, height: int) -> None:
    if frame.shape != (height, width, 3) or frame.dtype != np.uint8:
        raise ValueError(f"color must be uint8 BGR {width}x{height}")
    if not frame.flags.c_contiguous:
        raise ValueError("color must be contiguous")


def _require_aligned_depth(frame: np.ndarray, width: int, height: int) -> None:
    if frame.shape != (height, width) or frame.dtype != np.uint16:
        raise ValueError(f"aligned depth must be uint16 {width}x{height}")
    if not frame.flags.c_contiguous:
        raise ValueError("aligned depth must be contiguous")


def _render_depth(depth: np.ndarray, width: int, height: int) -> np.ndarray:
    _require_aligned_depth(depth, width, height)
    clipped = np.clip(depth, 0, _MAX_DEPTH_MM)
    normalized = np.rint(
        clipped.astype(np.float32) * (255.0 / _MAX_DEPTH_MM)
    )
    colored = cv2.applyColorMap(
        normalized.astype(np.uint8), cv2.COLORMAP_TURBO
    )
    colored[depth == 0] = 0
    return colored


def render_frame(
    mode: FrameMode,
    color: Optional[np.ndarray],
    depth: Optional[np.ndarray],
    width: int,
    height: int,
    overlay_alpha: float = 0.5,
) -> Optional[np.ndarray]:
    """Render the selected output, or ``None`` when an input is absent."""
    mode = FrameMode(mode)
    if (
        isinstance(overlay_alpha, bool)
        or not isinstance(overlay_alpha, (int, float))
        or not 0 < overlay_alpha <= 1
    ):
        raise ValueError("overlay_alpha must be in (0, 1]")
    if mode is FrameMode.COLOR:
        if color is None:
            return None
        _require_color(color, width, height)
        return color

    if mode is FrameMode.DEPTH:
        if depth is None:
            return None
        return np.ascontiguousarray(
            _render_depth(depth, width, height)
        )

    if color is None or depth is None:
        return None
    _require_color(color, width, height)
    rendered_color = color
    rendered_depth = _render_depth(depth, width, height)
    return np.ascontiguousarray(
        cv2.addWeighted(
            rendered_color, 1.0 - overlay_alpha,
            rendered_depth, overlay_alpha, 0,
        )
    )


class LatestVideoFrames:
    """Thread-safe latest color plus reusable timestamped aligned depth."""

    def __init__(self, width: int = 1280, height: int = 720,
                 overlay_alpha: float = 0.5) -> None:
        self._width = width
        self._height = height
        self._overlay_alpha = overlay_alpha
        self._color: Optional[np.ndarray] = None
        self._color_timestamp_ns: Optional[int] = None
        self._depth: Optional[np.ndarray] = None
        self._depth_timestamp_ns: Optional[int] = None
        self._lock = Lock()

    def put_color(self, frame: np.ndarray, timestamp_ns: int) -> None:
        with self._lock:
            self._color = frame.copy()
            self._color_timestamp_ns = timestamp_ns

    def put_depth(self, frame: np.ndarray, timestamp_ns: int) -> None:
        with self._lock:
            self._depth = frame.copy()
            self._depth_timestamp_ns = timestamp_ns

    def depth_age_ns(self, now_ns: int) -> Optional[int]:
        with self._lock:
            if self._depth_timestamp_ns is None:
                return None
            return max(0, now_ns - self._depth_timestamp_ns)

    def take(
        self, mode: FrameMode, now_ns: int, max_depth_age_ns: int
    ) -> Optional[np.ndarray]:
        mode = FrameMode(mode)
        with self._lock:
            color = self._color
            depth = self._depth
            self._color = None
            self._color_timestamp_ns = None
            depth_timestamp_ns = self._depth_timestamp_ns

        if mode in (FrameMode.DEPTH, FrameMode.OVERLAY):
            if (
                depth_timestamp_ns is None
                or max(0, now_ns - depth_timestamp_ns) > max_depth_age_ns
            ):
                depth = None
            # Even depth-only video is paced by a newly consumed RGB sample.
            if color is None:
                return None

        return render_frame(
            mode, color, depth, self._width, self._height,
            overlay_alpha=self._overlay_alpha,
        )
