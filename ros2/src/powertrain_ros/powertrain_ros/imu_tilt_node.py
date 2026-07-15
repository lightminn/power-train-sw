"""Thin L515 IMU adapter for the ROS-free WP6-A estimator core.

Raw optical-frame accel/gyro messages are converted to REP-103 values and
injected into ``StateEstimator``.  The existing ``/imu/filtered`` and optional
TF contracts remain unchanged.
"""

import math
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from powertrain_msgs.msg import WheelStates

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.kinematics import default_geometry  # noqa: E402
from powertrain_ros.state_estimation import (  # noqa: E402
    ImuSample,
    StateEstimator,
    StateEstimatorConfig,
    WheelSample,
    WheelValue,
)


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def opt_to_body(vector):
    """L515 optical axes to REP-103 body axes."""
    return (vector.z, -vector.x, -vector.y)


class ImuTiltNode(Node):
    def __init__(self):
        super().__init__("imu_tilt")
        self.declare_parameter("alpha", 0.98)
        self.declare_parameter("accel_lpf_alpha", 0.8)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("bias_samples", 200)
        self.declare_parameter("sample_timeout_s", 0.25)
        self.declare_parameter("mount_x", 0.30)
        self.declare_parameter("mount_y", 0.0)
        self.declare_parameter("mount_z", 0.35)
        self.declare_parameter("publish_static_tf", True)
        self.declare_parameter("publish_odom_tf", True)

        self.geom = default_geometry()
        self.estimator = self._new_estimator()
        self.roll = self.pitch = self.yaw = 0.0
        self._accel = None
        self._t_prev = None
        self._settled = False
        self._bias = [0.0, 0.0, 0.0]
        self._bias_acc = [0.0, 0.0, 0.0]
        self._bias_n = 0
        self._gyro_body = (0.0, 0.0, 0.0)

        self._own_odom_tf = bool(
            self.get_parameter("publish_odom_tf").value
        )
        self.tf = TransformBroadcaster(self)
        self.imu_pub = self.create_publisher(
            Imu,
            "/imu/filtered",
            qos_profile_sensor_data,
        )
        if bool(self.get_parameter("publish_static_tf").value):
            self.static_tf = StaticTransformBroadcaster(self)
            self._publish_static()
        else:
            self.get_logger().info(
                "정적 TF 는 robot_state_publisher(URDF)가 발행 — 생략"
            )
        if not self._own_odom_tf:
            self.get_logger().info("odom→base_link 는 오도메트리 노드가 발행 — 생략")

        self.create_subscription(
            Imu,
            "/l515/accel/sample",
            self._on_accel,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/l515/gyro/sample",
            self._on_gyro,
            qos_profile_sensor_data,
        )
        # Stationary wheel evidence keeps bias estimation active after startup.
        self.create_subscription(
            WheelStates,
            "/wheel_states",
            self._on_wheels,
            10,
        )
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self._publish_tf)
        self.create_timer(2.0, self._log)
        self.get_logger().info("imu_tilt 시작 — WP6-A core")

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _new_estimator(self):
        return StateEstimator(
            self.geom,
            StateEstimatorConfig(
                sample_timeout_s=float(
                    self.get_parameter("sample_timeout_s").value
                ),
                bias_samples=int(self.get_parameter("bias_samples").value),
                accel_lpf_alpha=float(
                    self.get_parameter("accel_lpf_alpha").value
                ),
                complementary_alpha=float(
                    self.get_parameter("alpha").value
                ),
            ),
        )

    def _refresh_prefirst_config(self):
        """Honor parameter overrides made before the first IMU sample."""
        if self._t_prev is not None or self.estimator.bias_count:
            return
        expected = (
            int(self.get_parameter("bias_samples").value),
            float(self.get_parameter("accel_lpf_alpha").value),
            float(self.get_parameter("alpha").value),
            float(self.get_parameter("sample_timeout_s").value),
        )
        actual = (
            self.estimator.config.bias_samples,
            self.estimator.config.accel_lpf_alpha,
            self.estimator.config.complementary_alpha,
            self.estimator.config.sample_timeout_s,
        )
        if expected != actual:
            self.estimator = self._new_estimator()

    def _publish_static(self):
        mount = TransformStamped()
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.header.frame_id = "base_link"
        mount.child_frame_id = "l515_link"
        mount.transform.translation.x = float(
            self.get_parameter("mount_x").value
        )
        mount.transform.translation.y = float(
            self.get_parameter("mount_y").value
        )
        mount.transform.translation.z = float(
            self.get_parameter("mount_z").value
        )
        mount.transform.rotation.w = 1.0

        optical = TransformStamped()
        optical.header.stamp = mount.header.stamp
        optical.header.frame_id = "l515_link"
        optical.child_frame_id = "l515_depth_optical_frame"
        qx, qy, qz, qw = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)
        optical.transform.rotation.x = qx
        optical.transform.rotation.y = qy
        optical.transform.rotation.z = qz
        optical.transform.rotation.w = qw
        self.static_tf.sendTransform([mount, optical])

    def _on_accel(self, msg: Imu):
        # The gyro header is the integration stamp.  Accel is the latest-only
        # low-pass input paired at the next gyro callback.
        self._accel = opt_to_body(msg.linear_acceleration)

    def _on_wheels(self, msg: WheelStates):
        stamp_s = self._stamp_s(msg.header.stamp)
        values = tuple(
            WheelValue(
                name=wheel.name,
                command_turns_per_s=float(
                    getattr(
                        wheel,
                        "command_turns_per_s",
                        wheel.drive_turns_per_s,
                    )
                ),
                measured_turns_per_s=float(wheel.drive_turns_per_s),
                steer_deg=float(wheel.steer_deg),
                stale=bool(
                    getattr(wheel, "drive_stale", False)
                    or getattr(wheel, "steer_stale", False)
                ),
            )
            for wheel in msg.wheels
        )
        self.estimator.update_wheels(
            WheelSample(stamp_s=stamp_s, wheels=values),
            now_s=self._now_s(),
        )

    def _on_gyro(self, msg: Imu):
        if self._accel is None:
            return
        self._refresh_prefirst_config()
        stamp_s = self._stamp_s(msg.header.stamp)
        gyro = opt_to_body(msg.angular_velocity)
        decision = self.estimator.update_imu(
            ImuSample(
                stamp_s=stamp_s,
                gyro_x_rad_s=gyro[0],
                gyro_y_rad_s=gyro[1],
                gyro_z_rad_s=gyro[2],
                accel_x_m_s2=self._accel[0],
                accel_y_m_s2=self._accel[1],
                accel_z_m_s2=self._accel[2],
            ),
            now_s=self._now_s(),
        )
        if not decision.accepted:
            return
        self._t_prev = stamp_s
        self._sync_from_core(stamp_s)

    def _sync_from_core(self, stamp_s):
        state = self.estimator.snapshot(now_s=stamp_s)
        self.roll = state.tilt.roll_rad
        self.pitch = state.tilt.pitch_rad
        self.yaw = state.pose.yaw_rad
        self._bias = list(state.gyro_bias_rad_s)
        self._bias_n = self.estimator.bias_count
        self._bias_acc = [value * self._bias_n for value in self._bias]
        self._settled = self.estimator.tilt_initialized
        self._gyro_body = self.estimator.corrected_gyro_rad_s

    def _publish_tf(self):
        now = self.get_clock().now()
        state = self.estimator.snapshot(now_s=now.nanoseconds * 1e-9)
        if not self._settled or state.imu_stale:
            return
        qx, qy, qz, qw = quat_from_rpy(
            state.tilt.roll_rad,
            state.tilt.pitch_rad,
            state.pose.yaw_rad,
        )

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = "base_link"
        imu.orientation.x, imu.orientation.y = qx, qy
        imu.orientation.z, imu.orientation.w = qz, qw
        imu.angular_velocity.x = self._gyro_body[0]
        imu.angular_velocity.y = self._gyro_body[1]
        imu.angular_velocity.z = self._gyro_body[2]
        accel = self.estimator.filtered_accel_m_s2
        if accel is not None:
            imu.linear_acceleration.x = accel[0]
            imu.linear_acceleration.y = accel[1]
            imu.linear_acceleration.z = accel[2]
        self.imu_pub.publish(imu)

        if not self._own_odom_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = imu.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf.sendTransform(transform)

    def _log(self):
        if self._settled:
            state = self.estimator.snapshot(now_s=self._now_s())
            self.get_logger().info(
                "roll=%+6.1f° pitch=%+6.1f° yaw=%+6.1f° imu_stale=%s"
                % (
                    math.degrees(state.tilt.roll_rad),
                    math.degrees(state.tilt.pitch_rad),
                    math.degrees(state.pose.yaw_rad),
                    state.imu_stale,
                )
            )


def main():
    rclpy.init()
    node = ImuTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
