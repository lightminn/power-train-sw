"""Synthetic inputs for the terrain map node tests (domain 77).

Wheels come from ``chassis.kinematics`` through ``terrain_map.tests.odometry_sim``,
depth from the independent world renderer, and the gyro is expressed in the
depth optical frame by rotating base axes with the mount — not with
``terrain_map.geometry.base_to_optical`` — so the node's conversion is checked
against an independent construction.
"""
from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np
from sensor_msgs.msg import CameraInfo, Image, Imu

from powertrain_msgs.msg import WheelState, WheelStates
from terrain_map import odometry as O
from terrain_map.tests.odometry_sim import Command, simulate
from terrain_map.tests.world_render import BasePose, Mount, flat, render_depth, rot_x, rot_y, rot_z, with_box
from terrain_overlay.camera import CameraModel

CAM = CameraModel(fx=115.0, fy=115.0, cx=80.0, cy=60.0, width=160, height=120)
BOX = with_box(flat(), 2.4, 2.7, -0.2, 0.2, 0.15)
BOX_CENTRE_X = 2.55
START = 1000.0
DEPTH_TOPIC = "/l515/depth/image_rect_raw"
INFO_TOPIC = "/l515/depth/camera_info"
GYRO_TOPIC = "/l515/gyro/sample"
WHEEL_TOPIC = "/wheel_states"
NODE_PARAMS = {"pixel_stride": 1}            # the fixture renders 160x120 depth


class Event(NamedTuple):
    stamp_s: float
    topic: str
    message: object


def optical_axes_in_base(pitch_down_deg=20.0, roll_deg=0.0, yaw_deg=0.0) -> np.ndarray:
    """Rows: optical x (right), y (down), z (forward) in base coordinates."""
    m = rot_z(math.radians(yaw_deg)) @ rot_y(math.radians(pitch_down_deg)) @ rot_x(math.radians(roll_deg))
    return np.stack([m @ np.array([0.0, -1.0, 0.0]), m @ np.array([0.0, 0.0, -1.0]), m @ np.array([1.0, 0.0, 0.0])])


def set_stamp(message, stamp_s: float):
    sec = math.floor(stamp_s)
    message.header.stamp.sec = int(sec)
    message.header.stamp.nanosec = int(round((stamp_s - sec) * 1e9)) % 1_000_000_000
    return message


def info_message(camera=CAM, stamp_s=START):
    message = CameraInfo()
    message.width, message.height = camera.width, camera.height
    message.k = [camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0]
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    return set_stamp(message, stamp_s)


def depth_message(array, stamp_s):
    message = Image()
    message.height, message.width = array.shape
    message.encoding, message.is_bigendian = "16UC1", 0
    message.step = array.shape[1] * 2
    message.data = array.tobytes()
    return set_stamp(message, stamp_s)


def gyro_message(stamp_s, wz_body, mount=(20.0, 0.0, 0.0)):
    w = optical_axes_in_base(*mount) @ np.array([0.0, 0.0, wz_body])
    message = Imu()
    message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = (float(v) for v in w)
    return set_stamp(message, stamp_s)


def wheel_message(sample: O.WheelSample, stale=False):
    message = WheelStates()
    message.chassis_mode = "ackermann"
    message.healthy = True
    for r in sample.wheels:
        w = WheelState()
        w.name = r.name
        w.drive_turns_per_s = float(r.meas_turns_per_s)
        w.command_turns_per_s = float(r.cmd_turns_per_s)
        w.steer_deg = float(math.degrees(r.steer_rad))
        w.drive_stale = bool(stale or not r.valid)
        message.wheels.append(w)
    return set_stamp(message, sample.stamp_s)


def drive_then(motion, stop_s=1.5):
    return lambda t: Command("stop") if t < stop_s else motion


def scenario(schedule, duration_s, *, surface=BOX, mount=(20.0, 0.0, 0.0), depth_from_s=1.6,
             stale_wheels_after_s=None, gyro_rate=None) -> tuple[list[Event], object]:
    """All messages in stamp order, plus truth(stamp) -> (x, y, yaw)."""
    events, truth = simulate(schedule, duration_s, start_s=START, gyro_rate=gyro_rate)
    out = []
    for _, sample in events:
        if isinstance(sample, O.WheelSample):
            stale = stale_wheels_after_s is not None and sample.stamp_s - START > stale_wheels_after_s
            out.append(Event(sample.stamp_s, WHEEL_TOPIC, wheel_message(sample, stale)))
        else:
            out.append(Event(sample.stamp_s, GYRO_TOPIC, gyro_message(sample.stamp_s, sample.wz, mount)))
    if surface is not None:
        pitch, roll, yaw = mount
        for k in range(int(depth_from_s * 10), int(duration_s * 10)):
            t = START + k / 10
            x, y, heading = truth(t)
            array = render_depth(CAM, BasePose(x=x, y=y, yaw_deg=math.degrees(heading)),
                                 Mount(pitch_down_deg=pitch, roll_deg=roll, yaw_deg=yaw), surface)
            out.append(Event(t, INFO_TOPIC, info_message(CAM, t)))
            out.append(Event(t, DEPTH_TOPIC, depth_message(array, t)))
    order = {INFO_TOPIC: 0, WHEEL_TOPIC: 1, GYRO_TOPIC: 2, DEPTH_TOPIC: 3}
    out.sort(key=lambda e: (e.stamp_s, order[e.topic]))
    return out, truth
