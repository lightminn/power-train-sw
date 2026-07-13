"""명령 권한 ROS 래퍼 — `/cmd_vel` 의 단일 작성자 (WP5.2-T).

    /cmd_vel/teleop ─┐
                     ├─→ [이 노드] ─→ /cmd_vel  (여기만 쓴다)
    /cmd_vel/auto  ──┘        └─→ /command_authority/state

    서비스: ~/manual  ~/auto  ~/idle   (모드 전환)

계산은 순수 코어(`motor_control/chassis/authority.py`, pytest 9종)가 한다.

────────────────────────────────────────────────────────────────────────
🛑 `/cmd_vel` 은 **이 노드만** 쓴다
────────────────────────────────────────────────────────────────────────
다른 노드(lane_follower · mission_sequencer · 텔레옵)는 `/cmd_vel/auto` 또는
`/cmd_vel/teleop` 로 **제안**만 하고, 실제 발행은 여기서 한 번만 한다.
ROS 는 복수 작성자를 막지 않는다 — 마지막 도착 메시지가 이긴다. 그러면 자율과 원격이
서로 싸우거나, **꺼진 줄 알았던 자율이 계속 조종**한다.

★ **zero-confirmed handover**: 모드를 바꿔도 **새 소스가 중립(≈0)을 한 번 보내기 전까지
  아무것도 전달하지 않는다.** 전환 순간 살아 있던 전속 명령이 즉시 나가는 걸 막는다.

⚠️ 안전 게이트가 아니다 — E-stop·충돌방지는 `SafetyInterlock` + US-100 이 한다.
⚠️ 소스가 stale 하면 **아무것도 발행하지 않는다.** `chassis_node` 의 명령 워치독(300 ms)이
   자연히 구동을 0 으로 내린다. 여기서 0 을 계속 쏘면 그 워치독이 영영 안 터진다.
"""
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.authority import (                          # noqa: E402
    AUTO, AUTO_SOURCE, IDLE, MANUAL, MANUAL_SOURCE,
    AuthorityConfig, CommandAuthority,
)


class CommandAuthorityNode(Node):
    def __init__(self):
        super().__init__("command_authority")
        self.declare_parameter("stale_s", 0.3)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("start_mode", IDLE)        # 기본은 아무도 조종 안 함

        self.auth = CommandAuthority(AuthorityConfig(
            stale_s=float(self.get_parameter("stale_s").value)))
        self.auth.set_mode(str(self.get_parameter("start_mode").value))

        self.create_subscription(
            Twist, "/cmd_vel/teleop",
            lambda m: self._submit(MANUAL_SOURCE, m), 10)
        self.create_subscription(
            Twist, "/cmd_vel/auto",
            lambda m: self._submit(AUTO_SOURCE, m), 10)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/command_authority/state", 10)

        self.create_service(Trigger, "~/manual", lambda q, r: self._mode(MANUAL, r))
        self.create_service(Trigger, "~/auto", lambda q, r: self._mode(AUTO, r))
        self.create_service(Trigger, "~/idle", lambda q, r: self._mode(IDLE, r))

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self.create_timer(2.0, self._log)
        self._last_reason = ""
        self.get_logger().info(f"command_authority 시작 — mode={self.auth.mode}")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _submit(self, source, msg: Twist):
        self.auth.submit(source, msg.linear.x, msg.angular.z, self._now())

    def _mode(self, mode, response):
        self.auth.set_mode(mode)
        self.get_logger().warn(
            f"모드 → {mode} (중립 확인 전까지 명령 전달 안 함)")
        response.success = True
        response.message = f"mode={mode}"
        return response

    def _tick(self):
        cmd = self.auth.select(self._now())
        self._last_reason = cmd.reason
        self.pub_state.publish(String(data=f"{self.auth.mode}|{cmd.reason}"))
        if not cmd.ok:
            return                                        # ★ 아무것도 안 쏜다 (워치독이 처리)
        t = Twist()
        t.linear.x = cmd.v
        t.angular.z = cmd.omega
        self.pub.publish(t)

    def _log(self):
        self.get_logger().info(f"mode={self.auth.mode}  {self._last_reason}")


def main():
    rclpy.init()
    node = CommandAuthorityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
