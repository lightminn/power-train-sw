"""4WS 오도메트리 ROS 래퍼 (WP6 Step 2).

    /wheel_states ─┐
                   ├─→ [이 노드] ─→ /odom (nav_msgs/Odometry)
    /imu/filtered ─┘                └─→ TF odom→base_link (병진 + 회전)

**순수 Python 코어 + 얇은 rclpy 래퍼** — 레포 원칙 그대로다. 계산은 전부
`motor_control/chassis/odometry.py` 가 하고(하드웨어·ROS 의존 0, pytest 22종),
여기서는 메시지를 코어의 자료형으로 옮기고 결과를 다시 메시지로 옮길 뿐이다.

────────────────────────────────────────────────────────────────────────
원칙: **바퀴 = 거리 · IMU = 회전**
────────────────────────────────────────────────────────────────────────
· **병진(x, y)** = 휠 6 + 조향 4 를 최소자승으로 푼 차체 속도를 적분. 슬립 바퀴는
  코어가 아웃라이어로 배제한다.
· **회전(yaw)** = **IMU 자이로**를 쓴다(`OdometryIntegrator(yaw_rate=...)`). 휠에서 푼
  ω 를 안 쓰는 이유는 실측으로 확인됐다 — 제자리 피벗에서 조향이 ±45° 로 클램프되면
  휠 기반 ω 가 5% 이상 과소추정된다(축거 1018mm 에서 피벗은 63° 를 요구한다).
  자이로는 그 왜곡을 겪지 않는다.
· **roll/pitch** = IMU 자세를 TF 회전에 그대로 싣는다 → RViz 에서 경사에 기울어진다.

⚠️ EKF 를 쓰지 않는다(상보 필터). 절대 보정 소스(GPS·지자기)가 없어 공분산 튜닝 이득이
   작고, "바퀴=거리 / IMU=회전" 분리가 깨끗하기 때문이다. 전역 드리프트 보정(`map→odom`)은
   우리 범위 밖이다.

⚠️ **절대 정확도는 미검증이다.** 기하(윤거)와 바퀴 반경(하중에 눌린 실효값)이 아직
   플레이스홀더다. 조립 후 지상 캘리브레이션(직진 줄자 → 실효반경 / 원주행 → 윤거·축거)
   으로만 확정된다. 그때 `ChassisGeometry` 숫자만 바꾸면 이 노드는 그대로 유효하다.
"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from powertrain_msgs.msg import WheelStates

# chassis_node 와 동일한 방식으로 순수 Python 코어를 얹는다
sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.kinematics import default_geometry           # noqa: E402
from chassis.odometry import (                            # noqa: E402
    OdometryConfig, OdometryIntegrator, WheelObservation, solve_twist,
)


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry")
        self.declare_parameter("use_imu_yaw", True)     # 원칙: IMU = 회전
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_hz", 50.0)      # TF/odom 발행 주기 (제어루프와 동일)

        self.geom = default_geometry()
        self.cfg = OdometryConfig()
        self.odo = OdometryIntegrator()

        self._roll = self._pitch = 0.0
        self._yaw_rate = None                            # IMU 자이로 z (없으면 휠 ω 사용)
        self._t_prev = None
        self._n = 0

        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(WheelStates, "/wheel_states", self._on_wheels, 10)
        self.create_subscription(Imu, "/imu/filtered", self._on_imu, qos_profile_sensor_data)
        # ★ TF 는 **고정 주기로** 쏜다. 바퀴 콜백에서만 쏘면 모터가 꺼졌을 때
        #   `odom→base_link` 가 아무데서도 안 나와 **TF 체인이 끊긴다** — 로봇 모델도
        #   포인트클라우드도 렌더링되지 않는다. 이 노드가 그 TF 의 단독 소유자이므로
        #   바퀴가 없어도 IMU 자세만으로 계속 발행할 책임이 있다(병진은 그대로 유지).
        self.create_timer(1.0 / float(self.get_parameter("publish_hz").value), self._publish)
        self.create_timer(5.0, self._log)
        self.get_logger().info(
            f"odometry 시작 — 바퀴 {len(self.geom.wheels)}개, "
            f"축거 {(max(w.x for w in self.geom.wheels) - min(w.x for w in self.geom.wheels))*1000:.0f}mm")

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self._roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                                1 - 2 * (q.x * q.x + q.y * q.y))
        self._pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self._yaw_rate = msg.angular_velocity.z

    def _on_wheels(self, msg: WheelStates):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t_prev is None:
            self._t_prev = t
            return
        dt = t - self._t_prev
        self._t_prev = t
        if not (0.0 < dt < 0.5):                          # 튄 타임스탬프는 버린다
            return

        obs = [
            WheelObservation.from_turns_per_s(
                w.name, w.drive_turns_per_s, w.steer_deg,
                wheel_radius_m=self.geom.wheel_radius_m,
                valid=not (w.drive_stale or w.steer_stale),
            )
            for w in msg.wheels
        ]
        est = solve_twist(self.geom, obs, self.cfg)

        yaw_rate = (self._yaw_rate
                    if self._yaw_rate is not None
                    and bool(self.get_parameter("use_imu_yaw").value)
                    else None)
        self.odo.update(est, dt, yaw_rate=yaw_rate)
        self._n += 1
        self._last = est

    def _publish(self):
        """고정 주기 발행 — 바퀴가 없어도 IMU 자세로 TF 를 유지한다."""
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.odo.pose()
        est = getattr(self, "_last", None)
        qx, qy, qz, qw = _quat(self._roll, self._pitch, yaw)   # 기울임까지 실어보낸다

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        if est is not None:                            # 바퀴가 없으면 속도는 0 으로 둔다
            odom.twist.twist.linear.x = est.vx
            odom.twist.twist.linear.y = est.vy
            odom.twist.twist.angular.z = est.omega
        self.pub.publish(odom)

        if not bool(self.get_parameter("publish_tf").value):
            return
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

    def _log(self):
        if self._n == 0:
            self.get_logger().info("/wheel_states 없음 — 모터가 꺼져 있다")
            return
        x, y, th = self.odo.pose()
        e = self._last
        rej = ",".join(e.rejected) if e.rejected else "-"
        self.get_logger().info(
            f"pose x={x:+.2f} y={y:+.2f} yaw={math.degrees(th):+.1f}°  "
            f"v={e.vx:+.2f} m/s ω={e.omega:+.2f}  잔차={e.residual_mps:.3f}  슬립배제={rej}")


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


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
