"""Ground-plane tilt for display (spec r11). Pure numpy; operator aid only.

Two values, never mixed:

- ``plane_tilt`` — how the plane the detector chose (usually the near floor; a ramp
  further ahead may not be in it) tilts against the **body** axes.
- gravity values — the same plane's slope and the body's own tilt against
  **gravity**, only from a still window ending at the depth stamp: accelerometer
  and gyro quiet and every valid wheel commanded and measured at rest.  While the
  rover moves, acceleration cannot be told from gravity, so there is no value.
"""
from __future__ import annotations

from collections import deque
import math

import numpy as np

from .config import MapConfig

G = 9.80665
WHEEL_HZ = 50.0
IMU_HZ = 100.0
_KEEP_S = 2.5


def _deg(x: float) -> float:
    return math.degrees(x)


def plane_tilt(normal_b: np.ndarray, up_b: np.ndarray) -> tuple[float, float, float] | None:
    """(pitch, roll, total) degrees of a plane with unit-ish normal ``normal_b`` (body axes)
    against the 'up' direction ``up_b``: pitch + = rising ahead, roll + = left side higher."""
    n = np.asarray(normal_b, float)
    u = np.asarray(up_b, float)
    n = n / np.linalg.norm(n)
    u = u / np.linalg.norm(u)
    nu = float(n @ u)
    if nu <= 0.0:
        return None
    x = np.array([1.0, 0.0, 0.0])
    f = x - (x @ u) * u
    if np.linalg.norm(f) < 1e-3:
        return None                                    # heading points along 'up': no forward direction
    f = f / np.linalg.norm(f)
    left = np.cross(u, f)
    pitch = math.atan2(-float(n @ f), nu)
    roll = math.atan2(-float(n @ left), nu)
    return _deg(pitch), _deg(roll), _deg(math.acos(min(1.0, max(-1.0, nu))))


def body_tilt(up_b: np.ndarray) -> tuple[float, float]:
    """Nose-up and left-side-up angles of the body axes above the horizontal (not an Euler pair)."""
    u = np.asarray(up_b, float) / np.linalg.norm(up_b)
    return _deg(math.asin(min(1.0, max(-1.0, float(u[0]))))), _deg(math.asin(min(1.0, max(-1.0, float(u[1])))))


class StillGravity:
    """Still-window gravity at a depth stamp, from samples stamped up to that stamp only."""

    def __init__(self, cfg: MapConfig) -> None:
        self.cfg = cfg
        self._accel: deque[tuple[float, np.ndarray]] = deque()
        self._gyro: deque[tuple[float, float]] = deque()
        self._wheels: deque[tuple[float, bool]] = deque()

    def clear(self) -> None:
        """Epoch change: samples never cross a continuity break."""
        self._accel.clear()
        self._gyro.clear()
        self._wheels.clear()

    def _add(self, buf: deque, stamp: float, value) -> None:
        if not math.isfinite(stamp):
            return
        if buf and stamp < buf[-1][0] - self.cfg.stamp_regression_epoch_s:
            self.clear()                               # clock restart
        buf.append((stamp, value))
        while buf and buf[-1][0] - buf[0][0] > _KEEP_S:
            buf.popleft()

    def add_accel(self, stamp: float, accel_b: np.ndarray) -> None:
        a = np.asarray(accel_b, float)
        if np.all(np.isfinite(a)):
            self._add(self._accel, stamp, a)

    def add_gyro(self, stamp: float, rate_norm: float) -> None:
        if math.isfinite(rate_norm):
            self._add(self._gyro, stamp, float(rate_norm))

    def add_wheels(self, stamp: float, stationary: bool) -> None:
        self._add(self._wheels, stamp, bool(stationary))

    def _window(self, buf: deque, t: float) -> list:
        lo = t - self.cfg.still_window_s
        return [(s, v) for s, v in buf if lo <= s <= t]

    def _covered(self, samples: list, t: float, rate_hz: float) -> bool:
        w = self.cfg.still_window_s
        if len(samples) < math.ceil(0.8 * w * rate_hz):
            return False
        stamps = [s for s, _ in sorted(samples, key=lambda e: e[0])]
        gaps = np.diff([t - w] + stamps + [t])
        return bool(gaps.max() <= self.cfg.wheel_gap_invalid_s)

    def up_at(self, t: float) -> np.ndarray | None:
        cfg = self.cfg
        accel, gyro, wheels = self._window(self._accel, t), self._window(self._gyro, t), self._window(self._wheels, t)
        if not (self._covered(accel, t, IMU_HZ) and self._covered(gyro, t, IMU_HZ)
                and self._covered(wheels, t, WHEEL_HZ)):
            return None
        if not all(still for _, still in wheels):
            return None
        if max(rate for _, rate in gyro) >= cfg.still_max_gyro_rad_s:
            return None
        a = np.array([v for _, v in accel])
        if float(a.std(axis=0).max()) >= cfg.still_max_accel_std:
            return None
        mean = a.mean(axis=0)
        magnitude = float(np.linalg.norm(mean))
        if abs(magnitude / G - 1.0) > cfg.gravity_tolerance:
            return None
        return mean / magnitude
