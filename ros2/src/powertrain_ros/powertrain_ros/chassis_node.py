"""WP5 chassis_node — ROS2 ↔ 10모터 4WS 다리.

설계원칙: **ROS 는 껍데기.** 제어는 `chassis.ChassisManager`(pytest 검증, motor_control).
이 노드는 토픽/서비스를 그 위에 붙이기만 한다.

  구독 /cmd_vel (geometry_msgs/Twist)  → cm.set(linear.x, angular.z)     [loop_hz tick]
  발행 /chassis_mode (ChassisMode)     ← 모드 인텐트(param `mode`, 기본 DRIVING)
  발행 /chassis_state (ChassisMode 재사용: mode 필드에 "MODE v=.. w=.. cor=..") — 진단
  구독 /arm_status (ArmStatus)         → 로그 + DONE 훅(WP8 시퀀서가 재출발에 사용)
  서비스 ~/arm ~/disarm ~/estop (std_srvs/Trigger)

안전: ① ChassisManager 내장 워치독(set() 이 watchdog_ms 무입력 → 구동 0) ②노드 cmd_vel
타임아웃(cmd_timeout 초 무수신 → set(0,0)) ③CAN 웻지 워치독은 별도 상주 서비스(canwatchdog).
⚠️ 실기 구동은 estop·바퀴 자유·좀비 프로세스 정리 등 HIL 안전수칙(kickoff 계획 WP1 참조).

실행 (powertrain_ros 컨테이너, /workspace/ros2 빌드·소스 후):
  ros2 run powertrain_ros chassis                              # 실기(can0)
  ros2 run powertrain_ros chassis --ros-args -p fake:=true     # 무하드웨어 배선검증
  ros2 run powertrain_ros chassis --ros-args -p channel:=can0 -p min_rev:=1.0 -p v_max:=1.5
"""
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from robot_arm_msgs.msg import ArmStatus, ChassisMode, ArrivalStatus
from std_msgs.msg import Header

from powertrain_ros import contract

# motor_control 은 ROS 패키지가 아님 — 레포가 컨테이너 /workspace 에 마운트됨.
sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))


class ChassisNode(Node):
    def __init__(self):
        super().__init__("chassis_node")

        self.declare_parameter("fake", False)
        self.declare_parameter("channel", "can0")
        self.declare_parameter("min_rev", 1.0)          # 저속 코깅존 회피 플로어(turns/s)
        self.declare_parameter("v_max", 1.5)            # 전진속도 상한 m/s
        self.declare_parameter("cmd_timeout", 0.5)      # /cmd_vel 무수신 → 정지(s)
        self.declare_parameter("mode", contract.MODE_DRIVING)

        fake = self.get_parameter("fake").value
        channel = self.get_parameter("channel").value
        min_rev = float(self.get_parameter("min_rev").value)
        v_max = float(self.get_parameter("v_max").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        from chassis.chassis_manager import ChassisManager, ChassisConfig, build_real_corners

        cfg = ChassisConfig(min_drive_turns_per_s=min_rev)
        cfg.geometry.drive_limit_mps = max(v_max, cfg.geometry.drive_limit_mps)

        if fake:
            corners = self._build_fake_corners(cfg)
            self.get_logger().warn("⚠️ FAKE 모드 — 실제 모터 구동하지 않음 (배선 검증용)")
        else:
            corners = build_real_corners(channel)
            self.get_logger().info("실기 모드 — 코너 6개 on %s" % channel)

        self.cm = ChassisManager(corners, cfg, monitor=None)
        self.cm.connect()

        # 구독/발행/서비스
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(ArmStatus, contract.TOPIC_ARM_STATUS, self._on_arm_status, 10)
        self.pub_mode = self.create_publisher(ChassisMode, contract.TOPIC_CHASSIS_MODE, 10)
        self.pub_arrival = self.create_publisher(ArrivalStatus, contract.TOPIC_ARRIVAL, 10)
        self.pub_state = self.create_publisher(ChassisMode, "/chassis_state", 10)  # 진단

        self.create_service(Trigger, "~/arm", self._srv_arm)
        self.create_service(Trigger, "~/disarm", self._srv_disarm)
        self.create_service(Trigger, "~/estop", self._srv_estop)

        self._last_cmd_ms = None
        self._last_arm_status = None

        period = 1.0 / self.cm.cfg.loop_hz
        self.create_timer(period, self._tick)           # 제어 루프
        self.create_timer(0.5, self._publish_mode)      # /chassis_mode 2Hz
        self.create_timer(1.0, self._publish_state)      # 진단 1Hz

        self.get_logger().info(
            "chassis_node 시작 (loop %.0fHz, min_rev %.1f, v_max %.1f). "
            "arm: ros2 service call /chassis_node/arm std_srvs/srv/Trigger"
            % (self.cm.cfg.loop_hz, min_rev, v_max))

    # ── 빌더 ──
    def _build_fake_corners(self, cfg):
        from corner_module.corner_module import CornerModule
        from corner_module.fake import FakeSteer, FakeDrive
        from corner_module.null_steer import NullSteer
        corners = {}
        for w in cfg.geometry.wheels:
            steer = FakeSteer() if w.steerable else NullSteer()
            corners[w.name] = CornerModule(steer, FakeDrive(), cfg.corner)
        return corners

    def _now_ms(self):
        return self.get_clock().now().nanoseconds / 1e6

    # ── 제어 루프 ──
    def _on_cmd_vel(self, msg: Twist):
        self.cm.set(msg.linear.x, msg.angular.z)
        self._last_cmd_ms = self._now_ms()

    def _tick(self):
        # cmd_vel 타임아웃 → 정지 (ChassisManager 내장 워치독과 이중 안전)
        if (self._last_cmd_ms is not None
                and (self._now_ms() - self._last_cmd_ms) > self._cmd_timeout * 1000.0):
            self.cm.set(0.0, 0.0)
        try:
            self.cm.tick()
        except Exception as e:                          # CAN 에러 등에 루프 안 죽게
            self.get_logger().warn("tick 예외: %s" % e, throttle_duration_sec=2.0)

    # ── 발행 ──
    def _header(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "base_link"
        return h

    def _publish_mode(self):
        m = ChassisMode()
        m.header = self._header()
        m.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.pub_mode.publish(m)

    def _publish_state(self):
        st = self.cm.state()
        m = ChassisMode()
        m.header = self._header()
        m.mode = "%s v=%.2f w=%.2f" % (st["mode"], st["v"], st["omega"])
        self.pub_state.publish(m)

    def publish_arrival(self, mission_id: int, status: str):
        """미션 도착 통지 (WP8 시퀀서가 정차 완료 시 호출)."""
        a = ArrivalStatus()
        a.header = self._header()
        a.mission_id = int(mission_id)
        a.status = status
        self.pub_arrival.publish(a)
        self.get_logger().info("→ /arrival_status mission=%d status=%s" % (a.mission_id, status))

    # ── 콜백 ──
    def _on_arm_status(self, msg: ArmStatus):
        if msg.status != self._last_arm_status:
            self.get_logger().info("← /arm_status mission=%d status=%s" % (msg.mission_id, msg.status))
            if msg.status == contract.ARM_DONE:
                self.get_logger().info("  ⇒ DONE — 재출발 가능 (WP8 시퀀서 훅)")
            self._last_arm_status = msg.status

    # ── 서비스 ──
    def _srv_arm(self, req, resp):
        self.cm.arm()
        resp.success = self.cm.mode == "ARMED"
        resp.message = "mode=%s" % self.cm.mode
        self.get_logger().info("arm → %s" % self.cm.mode)
        return resp

    def _srv_disarm(self, req, resp):
        self.cm.disarm()
        resp.success = True
        resp.message = "mode=%s" % self.cm.mode
        return resp

    def _srv_estop(self, req, resp):
        self.cm.estop()
        resp.success = True
        resp.message = "estop — mode=%s" % self.cm.mode
        self.get_logger().warn("estop!")
        return resp


def main(argv=None):
    rclpy.init(args=argv)
    node = ChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cm.disarm()
        node.cm.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
