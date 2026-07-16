"""One-process WP6-B terrain estimator and WP6-C controller ROS adapter.

This node only converts timestamped ROS messages to immutable autonomy-core
values.  Chassis command authority and final drive gating remain exclusively in
``chassis_node``.
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

from robot_arm_msgs.msg import ArmStatus

from powertrain_ros import contract

sys.path.insert(0, os.environ.get("AUTONOMY_PATH", "/workspace"))
sys.path.insert(
    0,
    os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"),
)

from powertrain_autonomy.controller import (  # noqa: E402
    AutonomyController,
    AutonomyControllerConfig,
    DriveDiagnostics,
    MotionState,
    ProfileGate,
    assist_correction_from_terrain,
    profile_by_name,
)
from powertrain_autonomy.terrain import (  # noqa: E402
    BaseToCameraExtrinsic,
    BodyTilt,
    OdometryDelta,
    TerrainEstimator,
    TerrainFrame,
)
from powertrain_autonomy.terrain.depth_quality import (  # noqa: E402
    CameraIntrinsics,
)


_TARGET_HEIGHT = 60
_TARGET_WIDTH = 80


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _rpy(quaternion) -> tuple[float, float, float]:
    roll = math.atan2(
        2.0 * (
            quaternion.w * quaternion.x
            + quaternion.y * quaternion.z
        ),
        1.0 - 2.0 * (
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
        ),
    )
    pitch = math.asin(
        max(
            -1.0,
            min(
                1.0,
                2.0 * (
                    quaternion.w * quaternion.y
                    - quaternion.z * quaternion.x
                ),
            ),
        )
    )
    yaw = math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )
    return roll, pitch, yaw


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class AutonomyControllerNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            "autonomy_controller",
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("enabled", False)
        self.declare_parameter("drive_profile", "EMPTY_STOWED")
        self.declare_parameter("tick_hz", 20.0)
        # PROVISIONAL: unmeasured candidates; never production-completion proof.
        self.declare_parameter("camera_height_m", 0.60)
        self.declare_parameter("camera_pitch_down_deg", 25.0)
        self.declare_parameter("camera_x_m", 0.0)
        self.declare_parameter("camera_yaw_deg", 0.0)
        self.declare_parameter("min_confidence", 0.25)

        tick_hz = float(self.get_parameter("tick_hz").value)
        if not math.isfinite(tick_hz) or tick_hz <= 0.0:
            raise ValueError("tick_hz must be finite and positive")
        profile = profile_by_name(
            str(self.get_parameter("drive_profile").value)
        )
        controller_config = AutonomyControllerConfig(
            min_confidence=float(
                self.get_parameter("min_confidence").value
            )
        )
        self.controller = AutonomyController(profile, controller_config)
        self.estimator = TerrainEstimator()
        self.extrinsic = BaseToCameraExtrinsic(
            x_m=float(self.get_parameter("camera_x_m").value),
            z_m=float(self.get_parameter("camera_height_m").value),
            pitch_down_rad=math.radians(
                float(
                    self.get_parameter("camera_pitch_down_deg").value
                )
            ),
            yaw_rad=math.radians(
                float(self.get_parameter("camera_yaw_deg").value)
            ),
        )
        self._enabled = bool(self.get_parameter("enabled").value)

        self._grid_source_shape: tuple[int, int] | None = None
        self._row_indices: np.ndarray | None = None
        self._col_indices: np.ndarray | None = None
        self._intrinsics: CameraIntrinsics | None = None
        self._tilt: BodyTilt | None = None
        self._imu_stamp_s: float | None = None
        self._odom = None
        self._previous_depth_pose = None
        self._terrain = None
        self._terrain_seen = False
        self._terrain_update_count = 0
        self._gate: ProfileGate | None = None
        self._diagnostics: DriveDiagnostics | None = None

        self.pub_cmd = self.create_publisher(
            Twist,
            "/autonomy/cmd_vel",
            10,
        )
        self.pub_controller_state = self.create_publisher(
            String,
            "/autonomy/controller_state",
            10,
        )
        self.pub_terrain_state = self.create_publisher(
            String,
            "/autonomy/terrain_state",
            10,
        )
        self.pub_assist_correction = self.create_publisher(
            String,
            "/autonomy/assist_correction",
            10,
        )

        self.create_subscription(
            CameraInfo,
            "/l515/depth/camera_info",
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/l515/depth/image_rect_raw",
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/imu/filtered",
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            10,
        )
        arm_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            ArmStatus,
            contract.TOPIC_ARM_STATUS,
            self._on_arm_status,
            arm_qos,
        )
        self.create_subscription(
            String,
            "/odom_diagnostics",
            self._on_diagnostics,
            10,
        )
        self.create_timer(1.0 / tick_hz, self._tick)

        mode = "ON" if self._enabled else "OFF (diagnostics only)"
        self.get_logger().info(
            "autonomy_controller profile=%s proposal=%s"
            % (profile.name, mode)
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_terrain_unavailable(self, reason: str) -> None:
        self.pub_terrain_state.publish(
            String(data="False|0.000000|0.000000|0.000000|" + reason)
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        shape = (int(message.height), int(message.width))
        if self._grid_source_shape is not None:
            if shape != self._grid_source_shape:
                self.get_logger().warning(
                    "CameraInfo resolution changed %s -> %s; fixed grid retained"
                    % (self._grid_source_shape, shape)
                )
            return
        height, width = shape
        stride = min(height // _TARGET_HEIGHT, width // _TARGET_WIDTH)
        if stride < 1:
            self.get_logger().error(
                "depth CameraInfo %dx%d is smaller than fixed 80x60"
                % (width, height)
            )
            return
        crop_height = _TARGET_HEIGHT * stride
        crop_width = _TARGET_WIDTH * stride
        row_start = (height - crop_height) // 2
        col_start = (width - crop_width) // 2
        try:
            intrinsics = CameraIntrinsics(
                fx=float(message.k[0]) / stride,
                fy=float(message.k[4]) / stride,
                cx=(float(message.k[2]) - col_start) / stride,
                cy=(float(message.k[5]) - row_start) / stride,
            )
        except (IndexError, TypeError, ValueError) as exc:
            self.get_logger().error("invalid depth CameraInfo: %s" % exc)
            return
        self._grid_source_shape = shape
        self._row_indices = row_start + stride * np.arange(_TARGET_HEIGHT)
        self._col_indices = col_start + stride * np.arange(_TARGET_WIDTH)
        self._intrinsics = intrinsics

    def _on_imu(self, message: Imu) -> None:
        roll, pitch, _ = _rpy(message.orientation)
        self._tilt = BodyTilt(roll_rad=roll, pitch_rad=pitch)
        self._imu_stamp_s = _stamp_s(message.header.stamp)

    def _on_odom(self, message: Odometry) -> None:
        _, _, yaw = _rpy(message.pose.pose.orientation)
        self._odom = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            yaw,
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
            _stamp_s(message.header.stamp),
        )

    def _on_arm_status(self, message: ArmStatus) -> None:
        self._gate = ProfileGate(
            stamp_s=_stamp_s(message.header.stamp),
            status=str(message.status),
        )

    def _on_diagnostics(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            raw_cap = payload["speed_cap_m_s"]
            speed_cap = math.inf if raw_cap is None else float(raw_cap)
            self._diagnostics = DriveDiagnostics(
                stamp_s=float(payload["stamp_s"]),
                slip_candidate=bool(payload["slip_candidate"]),
                stuck_candidate=bool(payload["stuck_candidate"]),
                speed_cap_m_s=speed_cap,
            )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                "invalid /odom_diagnostics ignored: %s" % exc
            )

    @staticmethod
    def _decode_depth(message: Image) -> np.ndarray:
        if message.encoding not in ("16UC1", "mono16"):
            raise ValueError("depth encoding must be 16UC1 or mono16")
        if message.step < message.width * 2 or message.step % 2:
            raise ValueError("depth step is inconsistent with uint16 width")
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        expected = int(message.height) * int(message.step)
        if len(message.data) < expected:
            raise ValueError("depth data is shorter than height*step")
        row_width = int(message.step) // 2
        array = np.frombuffer(message.data, dtype=dtype, count=expected // 2)
        array = array.reshape(int(message.height), row_width)
        return array[:, : int(message.width)].astype(np.uint16, copy=False)

    def _odometry_delta(self) -> OdometryDelta:
        # 기준 pose 전진은 estimator.update 성공 후에만 한다 — 실패 프레임의
        # 이동이 grid 이력 warp에서 누락되면 캐리 셀이 조용히 어긋난다.
        x_m, y_m, yaw_rad = self._odom[:3]
        if self._previous_depth_pose is None:
            return OdometryDelta(0.0, 0.0, 0.0)
        prev_x, prev_y, prev_yaw = self._previous_depth_pose
        global_x = x_m - prev_x
        global_y = y_m - prev_y
        cosine = math.cos(prev_yaw)
        sine = math.sin(prev_yaw)
        return OdometryDelta(
            dx_m=cosine * global_x + sine * global_y,
            dy_m=-sine * global_x + cosine * global_y,
            dyaw_rad=_wrap(yaw_rad - prev_yaw),
        )

    def _on_depth(self, message: Image) -> None:
        if self._grid_source_shape is None:
            self._publish_terrain_unavailable("waiting_camera_info")
            return
        shape = (int(message.height), int(message.width))
        if shape != self._grid_source_shape:
            self.get_logger().warning(
                "depth resolution %s ignored; fixed source is %s"
                % (shape, self._grid_source_shape)
            )
            return
        if self._tilt is None or self._odom is None:
            self._publish_terrain_unavailable("waiting_motion")
            return
        try:
            raw = self._decode_depth(message)
        except ValueError as exc:
            self._terrain = None
            self._publish_terrain_unavailable("invalid_depth")
            self.get_logger().warning("depth frame ignored: %s" % exc)
            return
        sampled = raw[np.ix_(self._row_indices, self._col_indices)].copy()
        frame = TerrainFrame(
            depth_roi=sampled,
            depth_scale_m=0.001,
            intrinsics=self._intrinsics,
            stamp_s=_stamp_s(message.header.stamp),
        )
        try:
            estimate = self.estimator.update(
                frame,
                tilt=self._tilt,
                extrinsic=self.extrinsic,
                odometry_delta=self._odometry_delta(),
                now_s=self._now_s(),
            )
        except ValueError as exc:
            self._terrain = None
            self._publish_terrain_unavailable("value_error")
            self.get_logger().error("terrain update rejected: %s" % exc)
            return
        self._previous_depth_pose = (
            float(self._odom[0]),
            float(self._odom[1]),
            float(self._odom[2]),
        )
        self._terrain = estimate
        self._terrain_seen = True
        self._terrain_update_count += 1
        reject = ",".join(estimate.reject_reasons)
        self.pub_terrain_state.publish(
            String(
                data="%s|%.6f|%.6f|%.6f|%s"
                % (
                    bool(estimate.path_available),
                    estimate.path_offset_m,
                    estimate.heading_error_rad,
                    estimate.confidence,
                    reject,
                )
            )
        )

    def _motion_state(self, now_s: float) -> MotionState | None:
        if self._odom is None or self._tilt is None or self._imu_stamp_s is None:
            return None
        stamps = (self._odom[5], self._imu_stamp_s)
        if not all(math.isfinite(stamp) for stamp in stamps):
            stamp_s = math.nan
        elif max(stamps) > now_s + 0.1:
            stamp_s = max(stamps)
        else:
            stamp_s = min(stamps)
        return MotionState(
            stamp_s=stamp_s,
            forward_m_s=self._odom[3],
            yaw_rate_rad_s=self._odom[4],
            roll_rad=self._tilt.roll_rad,
            pitch_rad=self._tilt.pitch_rad,
        )

    def _tick(self) -> None:
        now_s = self._now_s()
        assist = assist_correction_from_terrain(
            self._terrain,
            self.controller.config,
        )
        if assist is not None:
            omega_correction, speed_cap, confidence = assist
            self.pub_assist_correction.publish(
                String(
                    data=json.dumps(
                        {
                            "stamp_s": float(self._terrain.stamp_s),
                            "omega_correction_rad_s": omega_correction,
                            "speed_cap_m_s": speed_cap,
                            "confidence": confidence,
                        },
                        allow_nan=False,
                        separators=(",", ":"),
                        sort_keys=True,
                    )
                )
            )
        decision = self.controller.decide(
            now_s,
            terrain=self._terrain,
            motion=self._motion_state(now_s),
            gate=self._gate,
            diagnostics=self._diagnostics,
        )
        self.pub_controller_state.publish(
            String(
                data="%s|%.6f|%.6f|%s"
                % (
                    decision.state,
                    decision.v_m_s,
                    decision.omega_rad_s,
                    ",".join(decision.reasons),
                )
            )
        )
        if not self._enabled or not self._terrain_seen:
            return
        command = Twist()
        command.linear.x = decision.v_m_s
        command.angular.z = decision.omega_rad_s
        self.pub_cmd.publish(command)


def main():
    rclpy.init()
    node = AutonomyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
