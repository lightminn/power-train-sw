"""WP4 파워트레인 ROS2 브릿지 — 브링업 스켈레톤 (모터 로직 없음).

설계원칙: **ROS2 는 껍데기**. 여기선 로봇팔 팀 실물 FSM 과 메시지 왕복만 성립시킨다.
WP5 에서 이 노드에 `chassis.ChassisManager` 를 물려 (v, ω)·핸드셰이크를 실제 구동에 연결.

구독(팔→우리): `/arm_status`(ArmStatus) · `/detected_objects`(DetectedObjectArray)
발행(우리→팔): `/chassis_mode`(ChassisMode, 2 Hz 하트비트) · `/arrival_status`(ArrivalStatus, 이벤트)

계약 문자열 = `contract.py`. 실행:
    ros2 run powertrain_ros bringup
    ros2 run powertrain_ros bringup --ros-args -p mode:=CORNERING     # 락 모드로 발행
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from robot_arm_msgs.msg import ArmStatus, ArrivalStatus, ChassisMode, DetectedObjectArray

from powertrain_ros import contract


class BringupNode(Node):
    def __init__(self):
        super().__init__("powertrain_ros")

        # WP5 에서 이 값을 ChassisManager 상태(DRIVING/CORNERING/…)로 대체
        self.declare_parameter("mode", contract.MODE_DRIVING)
        self.declare_parameter("mode_hz", 2.0)

        # 구독 (팔 → 우리)
        self.create_subscription(ArmStatus, contract.TOPIC_ARM_STATUS, self._on_arm_status, 10)
        self.create_subscription(
            DetectedObjectArray, contract.TOPIC_DETECTED, self._on_detections, 10)
        # 발행 (우리 → 팔)
        self.pub_mode = self.create_publisher(ChassisMode, contract.TOPIC_CHASSIS_MODE, 10)
        self.pub_arrival = self.create_publisher(ArrivalStatus, contract.TOPIC_ARRIVAL, 10)

        hz = self.get_parameter("mode_hz").get_parameter_value().double_value
        self.create_timer(1.0 / max(hz, 0.1), self._publish_mode)

        self._last_arm_status = None
        self._det_count = 0
        self.create_timer(2.0, self._log_detections)     # 30fps 스로틀

        mode = self.get_parameter("mode").get_parameter_value().string_value
        self.get_logger().info(
            "powertrain_ros 브링업 (WP4 스켈레톤, 모터 로직 없음) — "
            "발행 mode=%s @%.0fHz, /arm_status·/detected_objects 구독" % (mode, hz))

    # ── 헬퍼 ──
    def _header(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "base_link"
        return h

    def _publish_mode(self):
        m = ChassisMode()
        m.header = self._header()
        m.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.pub_mode.publish(m)

    def publish_arrival(self, mission_id: int, status: str):
        """미션 도착 통지 (WP5: ChassisManager 정차 완료 시 호출)."""
        a = ArrivalStatus()
        a.header = self._header()
        a.mission_id = int(mission_id)
        a.status = status
        self.pub_arrival.publish(a)
        self.get_logger().info("→ /arrival_status mission=%d status=%s" % (a.mission_id, status))

    # ── 콜백 (팔 → 우리) ──
    def _on_arm_status(self, msg: ArmStatus):
        if msg.status != self._last_arm_status:          # 상태 전이 때만 로그
            self.get_logger().info(
                "← /arm_status mission=%d status=%s" % (msg.mission_id, msg.status))
            if msg.status == contract.ARM_DONE:
                self.get_logger().info("  ⇒ DONE 수신 — 재출발 신호 (WP5: ChassisManager.resume)")
            self._last_arm_status = msg.status

    def _on_detections(self, msg: DetectedObjectArray):
        self._det_count = len(msg.objects)

    def _log_detections(self):
        if self._det_count:
            self.get_logger().info("← /detected_objects 최근 프레임 %d개 물체" % self._det_count)


def main(argv=None):
    rclpy.init(args=argv)
    node = BringupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
