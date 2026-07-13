"""가짜 `/wheel_states` 발행기 — **벤치 전용**. 모터 없이 오도메트리 파이프라인을 검증한다.

    ros2 run powertrain_ros fake_wheels --ros-args -p course:=square

🛑 **실차에서 절대 쓰지 말 것.** 이 노드는 바퀴가 명령대로 완벽히 굴렀다고 **가정**한
   값을 만들어낸다. 실제 하드웨어가 무엇을 하든 상관없이 그럴듯한 텔레메트리가 나오므로,
   실기와 함께 띄우면 "잘 도는 것처럼 보이는" 착시를 만든다. `chassis_node` 와 동시에
   띄우면 `/wheel_states` 를 두 노드가 발행해 서로 싸운다.

무엇을 검증하나:
    차체 명령 (v, ω) → kinematics.solve() → 바퀴 명령 → **여기서 /wheel_states 로 발행**
                                                              ↓
                                    odometry_node → solve_twist → /odom + TF
                                                              ↓
                                                    RViz 에서 로봇이 실제로 움직인다

즉 **ROS 배관 전체**(메시지 왕복 · 좌표계 · TF · 시각화)를 모터 없이 통과시킨다.
계산의 정합성은 이미 `motor_control/chassis/tests/test_odometry.py`(22종)와
`chassis/odom_sim.py`(시각 검증)가 담보한다. 여기서 보는 것은 **배관**이다.
"""
import math
import os
import sys

import rclpy
from rclpy.node import Node

from powertrain_msgs.msg import WheelState, WheelStates

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.kinematics import default_geometry, solve      # noqa: E402

RATE_HZ = 50.0                                              # 실기 제어루프와 동일

# 코스 = [(지속 s, v m/s, ω rad/s), ...]
COURSES = {
    "straight": [(6.0, 0.4, 0.0)],
    "circle": [(2 * math.pi / 0.3, 0.4, 0.3)],
    "square": sum(([(4.0, 0.4, 0.0), (2.6, 0.4, math.pi / 4)] for _ in range(4)), []),
    "figure8": [(2 * math.pi / 0.35, 0.4, 0.35), (2 * math.pi / 0.35, 0.4, -0.35)],
    "pivot": [(4.0, 0.0, 0.4)],
}


class FakeWheels(Node):
    def __init__(self):
        super().__init__("fake_wheels")
        self.declare_parameter("course", "square")
        self.declare_parameter("loop", True)

        name = str(self.get_parameter("course").value)
        self.course = COURSES.get(name, COURSES["square"])
        self.geom = default_geometry()
        self._i = 0                                          # 코스 구간 인덱스
        self._t = 0.0                                        # 구간 내 경과시간

        self.pub = self.create_publisher(WheelStates, "/wheel_states", 10)
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().warn(
            f"🛑 벤치 전용 가짜 /wheel_states 발행 (코스={name}) — 실차 금지")

    def _tick(self):
        dur, v, omega = self.course[self._i]
        self._t += 1.0 / RATE_HZ
        if self._t >= dur:
            self._t = 0.0
            self._i += 1
            if self._i >= len(self.course):
                if not bool(self.get_parameter("loop").value):
                    self.get_logger().info("코스 종료")
                    raise SystemExit
                self._i = 0

        cmd = solve(self.geom, v, omega)                     # 실제 레포 역기구학
        msg = WheelStates()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.chassis_mode = "FAKE"
        msg.stop_state = "RUN"
        msg.healthy = True
        for w in self.geom.wheels:
            wc = cmd.wheels[w.name]
            ws = WheelState()
            ws.name = w.name
            ws.corner_mode = "FAKE"
            ws.drive_turns_per_s = wc.drive_turns_per_s      # 명령대로 굴렀다고 가정
            ws.steer_deg = wc.steer_deg
            msg.wheels.append(ws)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeWheels()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
