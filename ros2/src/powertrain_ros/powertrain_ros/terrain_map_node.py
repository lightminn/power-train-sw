"""L515 2.5D terrain map → operator console UDP :5010.

Operator visual aid only: this node subscribes to the Gateway's depth,
CameraInfo and gyro and to ``/wheel_states``; it publishes nothing to ROS and
gives nothing to control, stop or speed limits.  Every decision lives in the
ROS-free ``terrain_map.runtime`` core.  Spec:
docs/specs/2026-10-17-l515-terrain-map-design.md
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import socket
import sys
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

from powertrain_msgs.msg import WheelStates
from powertrain_runtime.telemetry import send_datagram
from powertrain_ros.terrain_overlay_node import depth_array
from terrain_map.config import MapConfig
from terrain_map.odometry import WheelLayout, WheelReading
from terrain_map.runtime import MapRuntime
from terrain_overlay.camera import CameraModel
from terrain_overlay.runtime import SenderSession

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.kinematics import default_geometry  # noqa: E402

DEPTH_TOPIC = "/l515/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/l515/depth/camera_info"
GYRO_TOPIC = "/l515/gyro/sample"
ACCEL_TOPIC = "/l515/accel/sample"          # display only: still-window gravity for the plane tilt (r11)
WHEEL_TOPIC = "/wheel_states"
# Gateway-compatible (best effort, volatile); a few frames so the scheduler can choose by stamp.
DEPTH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=5,
    reliability=qos_profile_sensor_data.reliability,
    durability=qos_profile_sensor_data.durability,
)
OPTIONAL_FLOATS = ("support_base_height_m", "support_base_variation_m")   # NaN parameter = None


def stamp_s(message) -> float:
    return int(message.header.stamp.sec) + int(message.header.stamp.nanosec) * 1e-9


def wheel_layout() -> WheelLayout:
    geometry = default_geometry()
    return WheelLayout({w.name: (w.x, w.y) for w in geometry.wheels}, geometry.wheel_radius_m)


def wheel_readings(message: WheelStates, radius_m: float) -> tuple[WheelReading, ...]:
    """WheelStates → readings; ground speed = drive turns/s × 2πr (wheel frame, + = forward)."""
    circumference = 2.0 * math.pi * radius_m
    return tuple(
        WheelReading(w.name, float(w.drive_turns_per_s) * circumference, math.radians(float(w.steer_deg)),
                     float(w.command_turns_per_s), float(w.drive_turns_per_s),
                     not (w.drive_stale or w.steer_stale))
        for w in message.wheels
    )


def map_config_from(get) -> MapConfig:
    values = {}
    for f in dataclasses.fields(MapConfig):
        value = get(f.name)
        if f.name in OPTIONAL_FLOATS:
            value = None if value is None or (isinstance(value, float) and math.isnan(value)) else float(value)
        values[f.name] = value
    return MapConfig(**values)


def _parameter_default(f: dataclasses.Field):
    if f.name in OPTIONAL_FLOATS:
        return float("nan")
    return f.default


class TerrainMapNode(Node):
    def __init__(self, *, session: SenderSession | None = None, **node_kwargs) -> None:
        super().__init__("terrain_map", **node_kwargs)
        self.declare_parameter("operator_host", "")
        self.declare_parameter("operator_port", 5010)
        self.declare_parameter("processed_log_path", "")
        for f in dataclasses.fields(MapConfig):
            self.declare_parameter(f.name, _parameter_default(f))
        value = lambda name: self.get_parameter(name).value  # noqa: E731
        host = str(value("operator_host"))
        port = int(value("operator_port"))
        if not host and not os.environ.get("POWERTRAIN_OPERATOR_SESSION_FILE"):
            raise ValueError("operator_host is required without POWERTRAIN_OPERATOR_SESSION_FILE")
        if not 1 <= port <= 65535:
            raise ValueError("operator_port must be 1-65535")
        self.config = map_config_from(value)              # range/relation violations raise here
        self._endpoint = (host, port)
        log_path = str(value("processed_log_path"))
        self._log_file = open(log_path, "a", encoding="utf-8") if log_path else None
        self._runtime = MapRuntime(
            self.config, session or SenderSession.create(), wheel_layout(),
            log_error=self._log_error, log_overrun=self._log_overrun,
            decisions=self._write_decision if self._log_file else None,
        )
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_errors = 0
        self._info_logged = False
        # Depth, odometry inputs and ticks each get a group: processing never delays the
        # receive times the odometry's stall rule reads (3-thread executor, see main).
        depth_group = MutuallyExclusiveCallbackGroup()
        input_group = MutuallyExclusiveCallbackGroup()
        tick_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, DEPTH_QOS, callback_group=depth_group)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self._on_info, qos_profile_sensor_data,
                                 callback_group=input_group)
        self.create_subscription(Imu, GYRO_TOPIC, self._on_gyro, qos_profile_sensor_data, callback_group=input_group)
        self.create_subscription(Imu, ACCEL_TOPIC, self._on_accel, qos_profile_sensor_data, callback_group=input_group)
        self.create_subscription(WheelStates, WHEEL_TOPIC, self._on_wheels, 10, callback_group=input_group)
        self.create_timer(1.0 / (2.0 * self.config.process_hz), self._runtime.process_tick, callback_group=tick_group)
        self.create_timer(1.0 / self.config.status_hz, self._send, callback_group=tick_group)
        session_file = os.environ.get("POWERTRAIN_OPERATOR_SESSION_FILE")
        target = f"operator session file {session_file} (fallback {host or '-'})" if session_file else host
        s = self._runtime.session
        self.get_logger().info(f"terrain map → {target}:{port} · session {s.boot_id[:8]}/{s.start_mono_ns}/{s.instance}")

    def _log_error(self, message: str) -> None:
        self.get_logger().error(message, throttle_duration_sec=5.0)

    def _log_overrun(self, message: str) -> None:
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def _write_decision(self, entry: dict) -> None:
        self._log_file.write(json.dumps(entry, separators=(",", ":")) + "\n")

    # --- inputs ---------------------------------------------------------------------
    def _on_depth(self, message: Image) -> None:
        received = time.monotonic()
        array = depth_array(message)
        if array is None:
            self._runtime.on_bad_depth(received)
            self.get_logger().warning(
                f"depth dropped: encoding={message.encoding} bigendian={message.is_bigendian} "
                f"step={message.step} width={message.width}", throttle_duration_sec=5.0)
            return
        self._runtime.on_depth(array, stamp_s(message), received)

    def _on_info(self, message: CameraInfo) -> None:
        try:
            camera = CameraModel.from_k(message.k, message.width, message.height)
        except (ValueError, IndexError) as exc:
            self.get_logger().warning(f"depth camera_info ignored: {exc}", throttle_duration_sec=5.0)
            return
        if not self._info_logged:
            self._info_logged = True
            self.get_logger().info(f"depth camera_info {message.width}x{message.height} (pinhole used)")
        self._runtime.on_camera_info(camera)

    def _on_gyro(self, message: Imu) -> None:
        v = message.angular_velocity
        self._runtime.on_gyro(stamp_s(message), time.monotonic(), (v.x, v.y, v.z))

    def _on_accel(self, message: Imu) -> None:
        a = message.linear_acceleration
        self._runtime.on_accel(stamp_s(message), time.monotonic(), (a.x, a.y, a.z))

    def _on_wheels(self, message: WheelStates) -> None:
        self._runtime.on_wheels(stamp_s(message), time.monotonic(),
                                wheel_readings(message, self.config.wheel_radius_m))

    # --- output ---------------------------------------------------------------------
    def _send(self) -> None:
        for raw in self._runtime.send_tick():
            try:
                send_datagram(self._udp, raw, self._endpoint)
            except OSError as exc:
                self.send_errors += 1
                self.get_logger().warning(f"terrain map UDP send failed ({self.send_errors} total): {exc}",
                                          throttle_duration_sec=5.0)

    def processing_summary(self) -> str:
        return self._runtime.processing_summary()

    def destroy_node(self) -> bool:
        self.get_logger().info(self.processing_summary())
        if self._log_file is not None:
            self._log_file.close()
        self._udp.close()
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = TerrainMapNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
