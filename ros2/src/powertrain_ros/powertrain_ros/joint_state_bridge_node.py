"""`/wheel_states` → `/joint_states` 변환 (WP6 Step 1).

`robot_state_publisher` 는 URDF + `/joint_states` 를 받아 `base_link → 각 바퀴/센서` TF 를
**자동으로** 만들어준다. 우리가 할 일은 실기 피드백을 그 표준 형식으로 옮기는 것뿐이다.

    ChassisManager (50 Hz) ─→ /wheel_states ─→ [이 노드] ─→ /joint_states
                                                                  ↓
                                                       robot_state_publisher ─→ TF

**진실의 소스를 하나로 유지한다** — 바퀴 상태는 이미 `/wheel_states`(WP5.1)로 나오고 있으므로
CAN 을 다시 열지 않는다(`can0` 단일 소유권). 여기서는 단위 변환만 한다.
  · 조향: `steer_deg` [deg] → 조향 관절 [rad]
  · 구동: `drive_turns_per_s` [rev/s] → 시간 적분 → 바퀴 회전각 [rad]
    (엔코더 절대각이 아니라 **속도의 적분**이다. 시각화용으로 충분하고, 바퀴가 도는 게
     화면에서 보이면 된다. 오도메트리는 별개로 `/odom` 이 담당한다.)

**모터가 꺼져 있어도** 0 으로 채운 `/joint_states` 를 계속 발행한다 → RViz 에 로봇이
그대로 뜬다. 벤치에서 URDF·TF 만 확인할 때 모터를 돌릴 필요가 없다.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from powertrain_msgs.msg import WheelStates
except ImportError:                                   # 메시지 미빌드 환경 — 0 발행만 한다
    WheelStates = None

STEERABLE = ("front_left", "front_right", "rear_left", "rear_right")
ALL_WHEELS = ("front_left", "front_right", "mid_left", "mid_right",
              "rear_left", "rear_right")


class JointStateBridge(Node):
    def __init__(self):
        super().__init__("joint_state_bridge")
        self.declare_parameter("publish_hz", 30.0)

        self._steer = {n: 0.0 for n in STEERABLE}      # rad
        self._angle = {n: 0.0 for n in ALL_WHEELS}     # rad (누적 회전각)
        self._rev_s = {n: 0.0 for n in ALL_WHEELS}     # rev/s (최신 속도)
        self._t_prev = None
        self._have_data = False

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        if WheelStates is not None:
            self.create_subscription(WheelStates, "/wheel_states", self._on_wheels, 10)
        else:
            self.get_logger().warn("powertrain_msgs 없음 — 0 으로만 발행한다")

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self._publish)
        self.create_timer(5.0, self._log)
        self.get_logger().info("joint_state_bridge 시작 — /joint_states 발행")

    def _on_wheels(self, msg):
        self._have_data = True
        for w in msg.wheels:
            if w.name in self._rev_s:
                self._rev_s[w.name] = w.drive_turns_per_s
            if w.name in self._steer:
                self._steer[w.name] = math.radians(w.steer_deg)

    def _publish(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._t_prev is not None:
            dt = t - self._t_prev
            if 0.0 < dt < 1.0:
                for n, rev in self._rev_s.items():     # rev/s → rad, 적분
                    self._angle[n] = _wrap(self._angle[n] + rev * 2.0 * math.pi * dt)
        self._t_prev = t

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ([f"{n}_steer_joint" for n in STEERABLE]
                   + [f"{n}_wheel_joint" for n in ALL_WHEELS])
        js.position = ([self._steer[n] for n in STEERABLE]
                       + [self._angle[n] for n in ALL_WHEELS])
        self.pub.publish(js)

    def _log(self):
        if not self._have_data:
            self.get_logger().info("/wheel_states 없음 — 0 자세로 발행 중 (모터 꺼짐)")


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    rclpy.init()
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
