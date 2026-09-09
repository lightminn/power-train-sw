"""Shared predicates for validating autonomy configuration values."""
from __future__ import annotations

from collections.abc import Iterable
import math


def require_positive(value: float, message: str) -> None:
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0.0:
        raise ValueError(message)


def require_non_negative(value: float, message: str) -> None:
    if isinstance(value, bool) or not math.isfinite(value) or value < 0.0:
        raise ValueError(message)


def require_int_at_least(value: int, minimum: int, message: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(message)


def require_all_finite(values: Iterable[float], message: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(message)


def require_ordered(low: float, high: float, message: str) -> None:
    if not low < high:
        raise ValueError(message)
