"""Rover motion simulator for odometry tests — wheel messages from the real kinematics.

Wheel readings come from ``chassis.kinematics`` (the same solver the chassis
uses), the truth pose from integrating the body twist on a fine grid.  The
odometry under test never sees the truth.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

from chassis.kinematics import default_geometry, skid_geometry, solve, solve_steering
from terrain_map.odometry import GyroSample, WheelLayout, WheelReading, WheelSample

GEOMETRY = default_geometry()
CIRCUMFERENCE = 2.0 * math.pi * GEOMETRY.wheel_radius_m
LAYOUT = WheelLayout({w.name: (w.x, w.y) for w in GEOMETRY.wheels}, GEOMETRY.wheel_radius_m)
MID_X = sum(w.x for w in GEOMETRY.wheels if not w.steerable) / 2.0


@dataclass(frozen=True)
class Command:
    """What the rover is doing at one instant."""
    mode: str            # "ackermann" | "skid" | "stop"
    v: float = 0.0
    steering: float = 0.0   # ackermann [-1, 1]
    omega: float = 0.0      # skid


def wheels_for(cmd: Command):
    """(readings without validity, body vx, vy, omega)."""
    if cmd.mode == "stop":
        result = solve(skid_geometry(), 0.0, 0.0)
        omega, vx, vy = 0.0, 0.0, 0.0
    elif cmd.mode == "ackermann":
        result = solve_steering(GEOMETRY, cmd.v, cmd.steering)
        omega = result.omega_applied
        vx, vy = cmd.v, -omega * MID_X
    else:
        result = solve(skid_geometry(), cmd.v, cmd.omega)
        omega, vx, vy = result.omega_applied, cmd.v, 0.0
    readings = []
    for w in GEOMETRY.wheels:
        c = result.wheels[w.name]
        turns = c.drive_turns_per_s
        readings.append((w.name, turns * CIRCUMFERENCE, math.radians(c.steer_deg), turns, turns))
    return readings, vx, vy, omega


def simulate(
    schedule: Callable[[float], Command],
    duration_s: float,
    *,
    wheel_hz: float = 50.0,
    gyro_hz: float = 100.0,
    gyro_bias: float = 0.0,
    wheel_delay_s: float = 0.0,
    gyro_delay_s: float = 0.0,
    gyro_rate: Callable[[float], float] | None = None,
    stale: Callable[[float, str], bool] | None = None,
    drop_wheels: Callable[[float], bool] | None = None,
    drop_gyro: Callable[[float], bool] | None = None,
    start_s: float = 100.0,
):
    """Return (events sorted by receive time, truth(t) -> (x, y, yaw))."""
    fine = 0.001
    steps = int(duration_s / fine) + 1
    truth = [(0.0, 0.0, 0.0)]
    x = y = yaw = 0.0
    for i in range(steps):
        t = i * fine
        _, vx, vy, omega = wheels_for(schedule(t))
        if gyro_rate is not None:
            omega = gyro_rate(t)
        c, s = math.cos(yaw + 0.5 * omega * fine), math.sin(yaw + 0.5 * omega * fine)
        x += (c * vx - s * vy) * fine
        y += (s * vx + c * vy) * fine
        yaw += omega * fine
        truth.append((x, y, yaw))

    def truth_at(stamp: float):
        i = min(len(truth) - 1, max(0, int(round((stamp - start_s) / fine))))
        return truth[i]

    events = []
    for i in range(int(duration_s * wheel_hz) + 1):
        t = i / wheel_hz
        if drop_wheels is not None and drop_wheels(t):
            continue
        readings, *_ = wheels_for(schedule(t))
        wheels = tuple(
            WheelReading(name, speed, steer, cmd, meas, not (stale and stale(t, name)))
            for name, speed, steer, cmd, meas in readings
        )
        events.append((start_s + t + wheel_delay_s, WheelSample(start_s + t, start_s + t + wheel_delay_s, wheels)))
    for i in range(int(duration_s * gyro_hz) + 1):
        t = i / gyro_hz
        if drop_gyro is not None and drop_gyro(t):
            continue
        _, _, _, omega = wheels_for(schedule(t))
        if gyro_rate is not None:
            omega = gyro_rate(t)
        events.append((start_s + t + gyro_delay_s, GyroSample(start_s + t, start_s + t + gyro_delay_s, omega + gyro_bias)))
    events.sort(key=lambda e: (e[0], 0 if isinstance(e[1], WheelSample) else 1))
    return events, truth_at


def feed(odometry, events):
    """Feed in receive order; return all emitted pose samples."""
    out = []
    for rx, sample in events:
        if isinstance(sample, WheelSample):
            out += odometry.add_wheels(sample)
        else:
            out += odometry.add_gyro(sample)
        out += odometry.poll(rx)
    return out
