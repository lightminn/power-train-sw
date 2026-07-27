# ros2/src/powertrain_ros/powertrain_ros/approach_controller_node.py
"""능동 최종접근 정렬 behavior 노드 (비전 자동 ARRIVED 트리거).

    /detected_objects ─┐
    /odom ─────────────┼─→ [이 노드: ApproachController] ─→ /autonomy/cmd_vel (제안)
    TF(base_link) ─────┘                                 ├─→ /approach/active (Bool)
    /arm_status ────────────────────────────────────────┼─→ /approach/state  (String)
                                                          └─→ chassis/mission_arrive_pickup/_drop (Trigger)

🛑 `/cmd_vel` 을 직접 쓰지 않는다 — authority가 내장된 chassis_node만 받는다.
   여기서는 `/autonomy/cmd_vel` 로 **제안**만 한다(`enabled:=true` 일 때만).
⚠️ pose 프레임이 TF로 base_link에 해석 안 되면 SEARCHING 유지(엉뚱한 거리 크립 금지).
설계: docs/superpowers/specs/2026-07-27-vision-arrival-approach-design.md
"""
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from powertrain_ros import contract
from powertrain_ros.section_supervisor_node import _apply_tf  # DRY: 동일 quaternion 규칙 재사용
from robot_arm_msgs.msg import ArmStatus, DetectedObjectArray

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.approach import (                              # noqa: E402
    ApproachConfig, ApproachController, Target, ARRIVED_PICKUP,
)

_CFG_FIELDS = (
    "engage_m", "stop_m", "lat_tol", "dist_tol", "k_yaw", "k_dist",
    "omega_max", "v_approach_max", "v_settle", "backoff_creep",
    "backoff_time_s", "align_timeout_s", "max_retries", "consecutive",
    "cooldown_s", "min_confidence", "lost_frames", "pose_stale_s",
    "pickup_class", "drop_class",
)


class ApproachControllerNode(Node):
    def __init__(self):
        super().__init__("approach_controller")
        self.declare_parameter("enabled", False)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("base_frame", "base_link")
        defaults = ApproachConfig()
        for f in _CFG_FIELDS:
            self.declare_parameter(f, getattr(defaults, f))
        cfg = ApproachConfig(**{f: self.get_parameter(f).value for f in _CFG_FIELDS})
        self.ctl = ApproachController(cfg)
        self._base = str(self.get_parameter("base_frame").value)
        self._enabled = bool(self.get_parameter("enabled").value)
        self._speed = 0.0
        self._targets = []
        self._targets_s = 0.0          # 마지막 detection 수신 시각(freshness)
        self._arm_done_prev = False

        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)

        self.create_subscription(DetectedObjectArray, contract.TOPIC_DETECTED,
                                 self._on_detections, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(ArmStatus, contract.TOPIC_ARM_STATUS,
                                 self._on_arm, 10)

        self.pub_cmd = self.create_publisher(Twist, "/autonomy/cmd_vel", 10)
        self.pub_active = self.create_publisher(Bool, "/approach/active", 10)
        self.pub_state = self.create_publisher(String, "/approach/state", 10)

        # ⚠️ chassis 노드명은 "chassis_node" → 서비스 절대경로.
        self.cli_pickup = self.create_client(
            Trigger, "/chassis_node/mission_arrive_pickup")
        self.cli_drop = self.create_client(
            Trigger, "/chassis_node/mission_arrive_drop")

        hz = float(self.get_parameter("control_hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            "approach_controller 시작 — 제안 %s" % ("ON" if self._enabled else "OFF"))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry):
        self._speed = abs(msg.twist.twist.linear.x)

    def _on_arm(self, msg: ArmStatus):
        done = (str(msg.status) == contract.ARM_DONE)
        if done and not self._arm_done_prev:
            self.ctl.on_mission_done(self._now())
        self._arm_done_prev = done

    def _on_detections(self, msg: DetectedObjectArray):
        frame = msg.header.frame_id
        if not frame:
            self._targets = []
            self.get_logger().warn("detected_objects frame_id 공백 — SEARCHING 유지",
                                   throttle_duration_sec=2.0)
            return
        try:
            tf = self._tf.lookup_transform(self._base, frame, Time())
        except Exception as exc:                            # TF 미해석 → 안전 실패
            self._targets = []
            self.get_logger().warn("TF %s→%s 실패(%s) — SEARCHING 유지"
                                   % (frame, self._base, type(exc).__name__),
                                   throttle_duration_sec=2.0)
            return
        out = []
        for o in msg.objects:
            x, y, _z = _apply_tf(o.pose.position, tf)       # 3-튜플 (x,y,z), x=전방·y=횡
            out.append(Target(str(o.class_name), float(o.confidence), x, y))
        self._targets = out
        self._targets_s = self._now()

    def _tick(self):
        # pose freshness: 오래된 검출로 크립 금지(§4.3). stale이면 빈 리스트→lost/정지.
        now = self._now()
        stale = (now - self._targets_s) > self.ctl.cfg.pose_stale_s
        targets = [] if stale else self._targets
        d = self.ctl.update(targets, self._speed, now)
        self.pub_active.publish(Bool(data=bool(d.active)))
        self.pub_state.publish(String(data="%s|%s" % (d.state, d.reason)))
        if self._enabled and d.active:
            cmd = Twist()
            cmd.linear.x = float(d.v)
            cmd.angular.z = float(d.omega)
            self.pub_cmd.publish(cmd)
        if d.fire is not None:
            self._call_arrive(d.fire)

    def _call_arrive(self, status):
        cli = self.cli_pickup if status == ARRIVED_PICKUP else self.cli_drop
        if not cli.service_is_ready():
            self.get_logger().warn("mission_arrive 서비스 미준비 — ACK 거부 처리")
            self.ctl.on_service_ack(False, self._now())
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(self._on_arrive_resp)

    def _on_arrive_resp(self, fut):
        try:
            resp = fut.result()
            ok = bool(resp.success)
        except Exception:
            ok = False
        self.ctl.on_service_ack(ok, self._now())
        self.get_logger().warn("mission_arrive ACK success=%s" % ok)


def main():
    rclpy.init()
    node = ApproachControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
