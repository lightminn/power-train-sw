"""앞 로봇 추종 ROS 래퍼 (WP9 · 국방 ⑤구간).

    /detected_objects ─→ [이 노드] ─→ /follow/state
                                   ├─→ /follow/active  (→ chassis_node 가 FOLLOW_LEAD 모드로)
                                   └─→ /cmd_vel/auto   (⚠️ `enabled:=true` 일 때만)

계산은 순수 코어(`motor_control/chassis/follow.py`, pytest 14종)가 한다.

🛑 `/cmd_vel` 을 직접 쓰지 않는다 — `command_authority` 만 쓴다.
⚠️ **레인·벽 추종과 동시에 켜지 않는다** — 셋 다 `/cmd_vel/auto` 를 쓴다.
⚠️ 추종 중에는 `/chassis_mode` 가 **FOLLOW_LEAD** 가 돼야 팔이 자세를 락한다
   (앞 차 급정거 시 팔이 흔들린다). `/follow/active` 로 chassis_node 에 알린다.
"""
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from powertrain_ros import contract
from robot_arm_msgs.msg import DetectedObjectArray

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.follow import FollowConfig, LeadFollower    # noqa: E402


class LeadFollowerNode(Node):
    def __init__(self):
        super().__init__("lead_follower")
        self.declare_parameter("enabled", False)          # 🛑 /cmd_vel/auto 제안 여부
        self.declare_parameter("class_name", "robot")
        self.declare_parameter("target_m", 1.5)
        self.declare_parameter("min_m", 0.8)              # ★ 이 안이면 무조건 정지
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("kd", 1.2)                 # ★ 접근 속도 → 추돌 방지

        self.cfg = FollowConfig(
            class_name=str(self.get_parameter("class_name").value),
            target_m=float(self.get_parameter("target_m").value),
            min_m=float(self.get_parameter("min_m").value),
            kp=float(self.get_parameter("kp").value),
            kd=float(self.get_parameter("kd").value),
        )
        self.follower = LeadFollower(self.cfg)
        self._allow_drive = True
        self._last = None

        self.create_subscription(DetectedObjectArray, contract.TOPIC_DETECTED,
                                 self._on_detections, 10)
        self.create_subscription(Bool, "/mission/allow_drive",
                                 lambda m: setattr(self, "_allow_drive", m.data), 10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel/auto", 10)
        self.pub_state = self.create_publisher(Float32MultiArray, "/follow/state", 10)
        # 추종 중임을 알린다 → chassis_node 가 /chassis_mode = FOLLOW_LEAD 로 (팔 자세 락)
        self.pub_active = self.create_publisher(Bool, "/follow/active", 10)
        self.create_timer(2.0, self._log)
        self.get_logger().info(
            f"lead_follower 시작 — '{self.cfg.class_name}' 추종, 목표 {self.cfg.target_m} m, "
            f"최소 {self.cfg.min_m} m")

    def _on_detections(self, msg: DetectedObjectArray):
        # ⚠️ pose 좌표계가 계약에 미명시 — 카메라(광학) 기준 가정 (z=전방, x=횡).
        dets = [(o.class_name, float(o.confidence),
                 float(o.pose.position.z), float(o.pose.position.x))
                for o in msg.objects]
        r = self.follower.update(dets, self.get_clock().now().nanoseconds * 1e-9)
        self._last = r

        self.pub_state.publish(Float32MultiArray(data=[
            1.0 if r.ok else 0.0, r.v, r.omega, r.distance_m, r.closing_mps]))
        self.pub_active.publish(Bool(data=bool(r.ok)))

        # ⚠️ 놓치면 아무것도 발행하지 않는다 — 없는 로봇을 따라 계속 달리면 안 된다.
        if r.ok and self._allow_drive and bool(self.get_parameter("enabled").value):
            cmd = Twist()
            cmd.linear.x = r.v
            cmd.angular.z = r.omega
            self.pub_cmd.publish(cmd)

    def _log(self):
        if self._last is None:
            self.get_logger().info("검출 대기 중 (/detected_objects)")
            return
        r = self._last
        if r.ok:
            self.get_logger().info(
                f"추종  거리 {r.distance_m:.2f} m (목표 {self.cfg.target_m})  "
                f"접근 {r.closing_mps:+.2f} m/s  → v={r.v:.2f} ω={r.omega:+.2f}  ({r.reason})")
        else:
            self.get_logger().warn(f"{r.reason}")


def main():
    rclpy.init()
    node = LeadFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
