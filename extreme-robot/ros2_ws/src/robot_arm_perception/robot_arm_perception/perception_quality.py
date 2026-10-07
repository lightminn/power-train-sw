"""Pure quality gates shared by the D435i perception runtime and tests."""
from __future__ import annotations

import math
import statistics


def is_usable_bbox(
    box: tuple[int, int, int, int],
    *,
    frame_width: int,
    frame_height: int,
    border_margin_px: int = 4,
    max_border_contacts: int = 2,
) -> bool:
    """Reject degenerate or heavily clipped detections.

    A target touching three or four image borders is not sufficiently visible
    for a reliable mask/depth estimate.  This also rejects the observed false
    positive spanning almost the complete left half of the D435i image.
    """
    x1, y1, x2, y2 = box
    if x2 <= x1 or y2 <= y1:
        return False
    contacts = sum((
        x1 <= border_margin_px,
        y1 <= border_margin_px,
        x2 >= frame_width - border_margin_px,
        y2 >= frame_height - border_margin_px,
    ))
    return contacts <= max_border_contacts


def robust_depth_m(
    raw_depth_values,
    *,
    depth_scale: float,
    minimum_pixels: int = 20,
    minimum_m: float = 0.1,
    maximum_m: float = 10.0,
) -> float | None:
    """Return a robust metric depth after invalid/outlier rejection."""
    values = [
        float(value) for value in raw_depth_values
        if math.isfinite(float(value)) and float(value) > 0.0
    ]
    if len(values) < minimum_pixels:
        return None
    values_m = [
        value * float(depth_scale) for value in values
        if minimum_m <= value * float(depth_scale) <= maximum_m
    ]
    if len(values_m) < minimum_pixels:
        return None
    median = float(statistics.median(values_m))
    deviations = [abs(value - median) for value in values_m]
    mad = float(statistics.median(deviations))
    if math.isfinite(mad) and mad > 0.0:
        limit = 3.5 * 1.4826 * mad
        values_m = [
            value for value, deviation in zip(values_m, deviations)
            if deviation <= limit
        ]
        if len(values_m) < minimum_pixels:
            return None
    return float(statistics.median(values_m))
