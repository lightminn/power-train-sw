"""Thin ROS adapter for the WP6-A wheel+IMU estimator.

The ROS-free ``state_estimation`` core owns freshness, reconnect handling,
wheel odometry, yaw-source selection, and immutable state.  This node only
converts messages and preserves the existing ``/odom``, TF, and ``~/reset``
contracts.  Relative odometry is never a sole mission-arrival condition.
"""

import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

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


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry")
        self.declare_parameter("use_imu_yaw", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("sample_timeout_s", 0.25)

        self.geom = default_geometry()
        self.estimator = StateEstimator(
            self.geom,
            StateEstimatorConfig(
                sample_timeout_s=float(
                    self.get_parameter("sample_timeout_s").value
                ),
                # /imu/filtered is already bias-corrected by imu_tilt_node.
                bias_samples=0,
                accel_lpf_alpha=0.0,
                complementary_alpha=0.0,
                use_imu_yaw=bool(
                    self.get_parameter("use_imu_yaw").value
                ),
            ),
        )
        # Existing tests and bench helpers use ``node.odo.pose()``.
        self.odo = self.estimator
        self._roll = self._pitch = 0.0
        self._yaw_rate = None
        self._t_prev = None
        self._last = None
        self._n = 0

        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.pub_diagnostics = self.create_publisher(
            String,
            "/odom_diagnostics",
            10,
        )
        self.create_subscription(
            WheelStates,
            "/wheel_states",
            self._on_wheels,
            10,
        )
        self.create_subscription(
            Imu,
            "/imu/filtered",
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(
            1.0 / float(self.get_parameter("publish_hz").value),
            self._publish,
        )
        self.create_timer(5.0, self._log)
        self.create_service(Trigger, "~/reset", self._srv_reset)
        self.get_logger().info(
            "odometry 시작 — WP6-A core, 바퀴 %d개" % len(self.geom.wheels)
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_s(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self._roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )
        self._pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        )
        self._yaw_rate = float(msg.angular_velocity.z)
        stamp_s = self._stamp_s(msg.header.stamp)

        # Reconstruct the gravity direction represented by /imu/filtered.
        gravity = 9.81
        accel_x = -gravity * math.sin(self._pitch)
        accel_y = gravity * math.sin(self._roll) * math.cos(self._pitch)
        accel_z = gravity * math.cos(self._roll) * math.cos(self._pitch)
        self.estimator.update_imu(
            ImuSample(
                stamp_s=stamp_s,
                gyro_x_rad_s=float(msg.angular_velocity.x),
                gyro_y_rad_s=float(msg.angular_velocity.y),
                gyro_z_rad_s=self._yaw_rate,
                accel_x_m_s2=accel_x,
                accel_y_m_s2=accel_y,
                accel_z_m_s2=accel_z,
            ),
            now_s=self._now_s(),
        )

    def _on_wheels(self, msg: WheelStates):
        stamp_s = self._stamp_s(msg.header.stamp)
        values = tuple(
            WheelValue(
                name=wheel.name,
                # WheelState currently has no command field.  A future additive
                # field is consumed automatically; until then measurement is the
                # neutral fallback so this adapter cannot invent a slip warning.
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
        decision = self.estimator.update_wheels(
            WheelSample(stamp_s=stamp_s, wheels=values),
            now_s=self._now_s(),
        )
        if not decision.accepted:
            return
        self._t_prev = stamp_s
        self._last = self.estimator.last_twist
        self._n += 1

    def _publish(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        state = self.estimator.snapshot(now_s=now_s)
        if state.stale:
            self.get_logger().warning(
                "wheel snapshot stale — /odom, diagnostics, TF 발행 생략",
                throttle_duration_sec=1.0,
            )
            return
        speed_cap = state.diagnostics.terrain_speed_cap
        self.pub_diagnostics.publish(
            String(
                data=json.dumps(
                    {
                        "stamp_s": now_s,
                        "slip_candidate": state.diagnostics.slip_candidate,
                        "stuck_candidate": state.diagnostics.stuck_candidate,
                        "terrain_profile": state.diagnostics.terrain_profile,
                        "speed_cap_m_s": (
                            speed_cap if math.isfinite(speed_cap) else None
                        ),
                    },
                    separators=(",", ":"),
                    allow_nan=False,
                )
            )
        )
        qx, qy, qz, qw = _quat(
            state.tilt.roll_rad,
            state.tilt.pitch_rad,
            state.pose.yaw_rad,
        )

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = state.pose.x_m
        odom.pose.pose.position.y = state.pose.y_m
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = state.velocity.forward_m_s
        odom.twist.twist.linear.y = state.velocity.lateral_m_s
        odom.twist.twist.angular.z = state.velocity.yaw_rate_rad_s
        self.pub.publish(odom)

        if not bool(self.get_parameter("publish_tf").value):
            return
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = state.pose.x_m
        transform.transform.translation.y = state.pose.y_m
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf.sendTransform(transform)

    def _srv_reset(self, _request, response):
        self.estimator.reset()
        self._t_prev = None
        self._last = None
        self.get_logger().info("오도메트리 pose 리셋 → (0, 0, 0)")
        response.success = True
        response.message = "odometry reset"
        return response

    def _log(self):
        if self._n == 0:
            self.get_logger().info("/wheel_states 없음 — 모터가 꺼져 있다")
            return
        state = self.estimator.snapshot(now_s=self._now_s())
        warning_codes = ",".join(state.diagnostics.warning_codes) or "-"
        self.get_logger().info(
            "pose x=%+.2f y=%+.2f yaw=%+.1f° v=%+.2f m/s "
            "yaw_source=%s stale=%s diagnostics=%s"
            % (
                state.pose.x_m,
                state.pose.y_m,
                math.degrees(state.pose.yaw_rad),
                state.velocity.forward_m_s,
                state.yaw_source,
                state.stale,
                warning_codes,
            )
        )


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
