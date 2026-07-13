"""미션 시퀀서 ROS 래퍼 (WP8) — 로봇팔 핸드셰이크.

────────────────────────────────────────────────────────────────────────
⚠️ v1 계약 기준 프로토타입
────────────────────────────────────────────────────────────────────────
WP5.2 Task 5의 chassis_node 소유 순수 mission_supervisor로 흡수 예정.
contract_output_enabled=true는 mock 팔 시험 전용 — 실물 팔과 병행 실행 금지.

    /detected_objects ─┐                      ┌─→ /chassis_mode   (팔 자세 락 / MISSION_STOP)
    /arm_status ───────┼─→ [이 노드] ─────────┼─→ /arrival_status (도착 알림)
    /odom ─────────────┘                      └─→ /mission/state
                                              └─→ ~/arrive, ~/reset (서비스)

계산은 순수 코어(`motor_control/chassis/mission.py`, pytest 14종)가 한다.

────────────────────────────────────────────────────────────────────────
🛑 정지는 **여기서 명령하지 않는다**
────────────────────────────────────────────────────────────────────────
`allow_drive=False` 여도 이 노드는 `/cmd_vel` 을 쓰지 않는다 — 그건 `chassis_node`에
내장된 command authority의 몫이다. 대신 `/mission/allow_drive` 로 알리고, **레인 추종이
그걸 보고 제안을 멈춘다.**
그러면 `chassis_node` 의 명령 워치독(300 ms)이 구동을 0 으로 내린다.
(여기서 0 을 계속 쏘면 그 워치독이 영영 안 터진다.)

⚠️ **팔이 `MISSION_STOP` 을 지켜줄 거라고 믿지 않는다.** 팔 팀 코드에 아직 그 모드가
   없다(무시한다). 계약은 우리가 지키되, **우리가 실제로 정지했는지는 우리가 확인**한다
   (`/odom` 의 속도로).
"""
import os
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from powertrain_ros import contract
from robot_arm_msgs.msg import ArmStatus, ArrivalStatus, DetectedObjectArray

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.mission import (                              # noqa: E402
    FAILED, MissionConfig, MissionSequencer,
)
from chassis.mission_trigger import (                      # noqa: E402
    MissionTrigger, TriggerConfig, TriggerRule,
)


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission")
        self.declare_parameter("done_timeout_s", 15.0)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("stop_settle_s", 0.3)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("contract_output_enabled", False)
        # ── 도착 자동 판정 (YOLO) ──
        self.declare_parameter("auto_trigger", True)
        self.declare_parameter("pickup_class", "box")
        self.declare_parameter("drop_class", "dropzone")
        self.declare_parameter("pickup_stop_m", 1.0)
        self.declare_parameter("drop_stop_m", 1.2)
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("consecutive", 5)      # ★ 한 프레임 깜빡임으로 급정거 금지
        self.declare_parameter("cooldown_s", 10.0)    # ★ 끝난 미션이 다시 트리거되면 무한루프

        self.seq = MissionSequencer(MissionConfig(
            done_timeout_s=float(self.get_parameter("done_timeout_s").value),
            max_retries=int(self.get_parameter("max_retries").value),
            stop_settle_s=float(self.get_parameter("stop_settle_s").value),
        ))
        self._speed = 0.0
        self._pending = None            # 서비스로 받은 도착 요청
        self._contract_output_enabled = bool(
            self.get_parameter("contract_output_enabled").value
        )

        self.trigger = MissionTrigger(TriggerConfig(
            rules=[
                TriggerRule(str(self.get_parameter("pickup_class").value),
                            contract.ARRIVED_PICKUP,
                            float(self.get_parameter("pickup_stop_m").value),
                            float(self.get_parameter("min_confidence").value)),
                TriggerRule(str(self.get_parameter("drop_class").value),
                            contract.ARRIVED_DROP,
                            float(self.get_parameter("drop_stop_m").value),
                            float(self.get_parameter("min_confidence").value)),
            ],
            consecutive=int(self.get_parameter("consecutive").value),
            cooldown_s=float(self.get_parameter("cooldown_s").value),
        ))
        self._active_class = None

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        # 인식은 **로봇팔 팀 단일 소스** — 우리는 결과만 구독한다.
        self.create_subscription(DetectedObjectArray, contract.TOPIC_DETECTED,
                                 self._on_detections, 10)
        self.create_subscription(ArmStatus, contract.TOPIC_ARM_STATUS,
                                 self._on_arm, 10)

        # ⚠️ `/chassis_mode` 를 **직접 쓰지 않는다.** 그건 팔과의 계약 토픽이고
        #    `chassis_node` 가 단독 소유한다. 두 노드가 쓰면 팔이 번갈아 받아서,
        #    **우리가 정차 중인데 DRIVING 을 보고 팔이 움직인다.**
        #    여기서는 "정차해달라"고 **요청**만 한다.
        self.pub_mode = self.create_publisher(String, "/mission/chassis_mode", 10)
        if self._contract_output_enabled:
            self.pub_arrival = self.create_publisher(
                ArrivalStatus,
                contract.TOPIC_ARRIVAL,
                10,
            )
        else:
            self.pub_arrival = None
        self.pub_allow = self.create_publisher(Bool, "/mission/allow_drive", 10)
        self.pub_state = self.create_publisher(String, "/mission/state", 10)

        # 도착 트리거 — 지금은 수동(서비스). 인지 연동(신호등·마커)은 다음 단계.
        self.create_service(Trigger, "~/arrive_pickup",
                            lambda q, r: self._arrive(contract.ARRIVED_PICKUP, r))
        self.create_service(Trigger, "~/arrive_drop",
                            lambda q, r: self._arrive(contract.ARRIVED_DROP, r))
        self.create_service(Trigger, "~/reset", self._srv_reset)

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self.create_timer(2.0, self._log)
        self._mission_counter = 0
        self._last = None
        if self._contract_output_enabled:
            self.get_logger().warning(
                "contract_output_enabled=true: mock 팔 시험 전용; "
                "실물 팔과 병행 실행 금지"
            )
        self.get_logger().info("mission 시작 — 도착 트리거: ~/arrive_pickup, ~/arrive_drop")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry):
        self._speed = msg.twist.twist.linear.x

    def _on_detections(self, msg: DetectedObjectArray):
        """YOLO 검출 → 도착 판정. ⚠️ `pose` 의 **좌표계가 계약에 명시돼 있지 않다** —
        지금은 카메라(광학) 기준으로 보고 z(전방거리)를 쓴다. 로봇팔 팀과 확정 필요.
        base_link 기준이면 x 를 써야 하고, 잘못 쓰면 **엉뚱한 거리에서 멈춘다.**"""
        if not bool(self.get_parameter("auto_trigger").value):
            return
        dets = [(o.class_name, float(o.confidence), float(o.pose.position.z))
                for o in msg.objects]
        hit = self.trigger.on_detections(dets, self._now())
        if hit is None:
            return
        status, cls = hit
        self._mission_counter += 1
        if self.seq.arrive(self._mission_counter, status, self._now()):
            self._active_class = cls
            self.get_logger().warn(
                f"🎯 자동 도착 판정 — {cls} → {status} (mission_id={self._mission_counter}). "
                "MISSION_STOP → 정지 확인 후 ArrivalStatus")
        else:
            self._mission_counter -= 1                 # 이미 미션 중 — 카운터 되돌림

    def _on_arm(self, msg: ArmStatus):
        why = self.seq.on_arm_status(msg.mission_id, msg.status, self._now())
        if "무시" in why:
            self.get_logger().warn(f"팔 상태 무시: {why}")
        elif "유효한 DONE" in why:
            self.get_logger().info(f"팔: {why}")
            if self._active_class:
                # ★ 재출발할 때 그 물체가 아직 눈앞에 있다 → 쿨다운 없으면 무한 루프
                self.trigger.mission_finished(self._active_class, self._now())
                self._active_class = None

    def _arrive(self, status, response):
        self._mission_counter += 1
        ok = self.seq.arrive(self._mission_counter, status, self._now())
        response.success = ok
        response.message = (f"mission {self._mission_counter} {status}" if ok
                            else f"이미 미션 처리 중 (state={self.seq.state})")
        if ok:
            self.get_logger().warn(
                f"미션 도착 — mission_id={self._mission_counter} {status}. "
                "MISSION_STOP 발행 → 정지 확인 후 ArrivalStatus")
        return response

    def _srv_reset(self, _request, response):
        self.seq.reset(self._now())
        self.get_logger().warn("미션 시퀀서 리셋 — 사람이 상황을 확인했다고 가정한다")
        response.success = True
        response.message = "reset"
        return response

    def _tick(self):
        d = self.seq.update(self._now(), self._speed)
        self._last = d

        self.pub_mode.publish(String(data=d.chassis_mode))      # 요청 (chassis_node 가 최종 결정)
        self.pub_allow.publish(Bool(data=d.allow_drive))
        self.pub_state.publish(String(data=f"{d.state}|{d.reason}"))

        if self._contract_output_enabled and d.publish_arrival is not None:
            mid, status = d.publish_arrival
            a = ArrivalStatus()
            a.header.stamp = self.get_clock().now().to_msg()
            a.mission_id = int(mid)
            a.status = status
            self.pub_arrival.publish(a)
            self.get_logger().warn(
                f"ArrivalStatus 발행 — mission_id={mid} {status}  ({d.reason})")

    def _log(self):
        if self._last is None:
            return
        d = self._last
        log = self.get_logger().error if d.state == FAILED else self.get_logger().info
        log(f"[{d.state}] {d.reason}  (drive={'ON' if d.allow_drive else 'OFF'}, "
            f"v={self._speed:+.2f})")


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
