# ros2/src/powertrain_ros/powertrain_ros/approach_controller_node.py
"""능동 최종접근 정렬 behavior 노드 (비전 자동 ARRIVED 트리거).

    /detected_objects ─┐
    /odom ─────────────┼─→ [이 노드: ApproachController] ─→ /autonomy/cmd_vel (제안)
    TF(base_link) ─────┘                                 ├─→ /approach/active (Bool)
    /arrival_status + /arm_status ──────────────────────┼─→ /approach/state  (String)
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
from robot_arm_msgs.msg import ArmStatus, ArrivalStatus, DetectedObjectArray

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.approach import (                              # noqa: E402
    ALIGNED, ARRIVED_PICKUP, ApproachConfig, ApproachController, Target,
)

_CFG_FIELDS = (
    "engage_m", "stop_m", "lat_tol", "dist_tol", "k_yaw", "k_dist",
    "omega_max", "v_approach_max", "v_settle", "backoff_creep",
    "backoff_time_s", "align_timeout_s", "max_retries", "consecutive",
    "cooldown_s", "min_confidence", "lost_frames", "pose_stale_s",
    "pickup_class", "drop_class",
)
_INT_CFG_FIELDS = {"max_retries", "consecutive", "lost_frames"}


class ApproachControllerNode(Node):
    def __init__(self):
        super().__init__("approach_controller")
        self.declare_parameter("enabled", False)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_stale_s", 0.3)
        defaults = ApproachConfig()
        for f in _CFG_FIELDS:
            default = getattr(defaults, f)
            if f in _INT_CFG_FIELDS:
                default = float(default)
            self.declare_parameter(f, default)
        cfg_values = {f: self.get_parameter(f).value for f in _CFG_FIELDS}
        for f in _INT_CFG_FIELDS:
            cfg_values[f] = int(cfg_values[f])
        cfg = ApproachConfig(**cfg_values)
        self.ctl = ApproachController(cfg)
        self._base = str(self.get_parameter("base_frame").value)
        self._enabled = bool(self.get_parameter("enabled").value)
        self._odom_stale_s = float(self.get_parameter("odom_stale_s").value)
        self._speed = 0.0
        self._speed_s = 0.0
        self._targets = []
        self._targets_stamp_s = 0.0
        self._active_mission_id = None
        self._resume_prev = False
        self._pending_fire = None
        self._call_inflight = False

        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)

        self.create_subscription(DetectedObjectArray, contract.TOPIC_DETECTED,
                                 self._on_detections, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(ArrivalStatus, contract.TOPIC_ARRIVAL,
                                 self._on_arrival, 10)
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
        self.create_service(Trigger, "~/reset", self._srv_reset)

        hz = float(self.get_parameter("control_hz").value)
        if not (hz > 0.0):
            self.get_logger().warn(
                "control_hz가 0 이하이거나 유효하지 않음 — 20.0 Hz 사용")
            hz = 20.0
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            "approach_controller 시작 — 제안 %s" % ("ON" if self._enabled else "OFF"))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry):
        self._speed = abs(msg.twist.twist.linear.x)
        self._speed_s = self._now()

    def _on_arrival(self, msg: ArrivalStatus):
        # chassis MissionSupervisor가 미션 진입 시 발행하는 mission_id를 학습한다.
        mission_id = int(msg.mission_id)
        if mission_id > 0:
            self._active_mission_id = mission_id

    def _on_arm(self, msg: ArmStatus):
        # v2 재출발 권위 = DRIVE_READY_STATUSES{STOWED_LOCKED, CARRYING_LOCKED} 진입.
        # ⚠️ 팔은 idle에도 STOWED_LOCKED를 상시 발행하므로 mission_id 일치 + rising edge로만
        #    "우리가 시킨 그 미션의 완료"를 인정한다(DONE은 v1-레거시라 안 쓴다).
        status = str(msg.status)
        drive_ready = status in contract.DRIVE_READY_STATUSES
        matched = (
            self._active_mission_id is not None
            and int(msg.mission_id) == self._active_mission_id
        )
        cond = drive_ready and matched
        if cond and not self._resume_prev:
            self.ctl.on_mission_done(self._now())
            self._active_mission_id = None
        self._resume_prev = cond

    def _on_detections(self, msg: DetectedObjectArray):
        self._targets_stamp_s = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
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

    def _tick(self):
        # pose freshness: 오래된 검출로 크립 금지(§4.3). stale이면 빈 리스트→lost/정지.
        now = self._now()
        age = now - self._targets_stamp_s
        stale = age > self.ctl.cfg.pose_stale_s or age < -0.1
        targets = [] if stale else self._targets
        odom_fresh = (now - self._speed_s) <= self._odom_stale_s
        speed = self._speed if odom_fresh else 1e9
        d = self.ctl.update(targets, speed, now)
        self.pub_active.publish(Bool(data=bool(d.active and self._enabled)))
        self.pub_state.publish(String(data="%s|%s" % (d.state, d.reason)))
        if self._enabled and d.active:
            cmd = Twist()
            cmd.linear.x = float(d.v)
            cmd.angular.z = float(d.omega)
            self.pub_cmd.publish(cmd)
        if self._enabled and d.fire is not None:
            self._pending_fire = d.fire
        if (
            self._pending_fire is not None
            and d.state != ALIGNED
            and not self._call_inflight
        ):
            self._pending_fire = None
        if (
            self._enabled
            and self._pending_fire is not None
            and not self._call_inflight
        ):
            self._call_arrive(self._pending_fire)

    def _call_arrive(self, status):
        cli = self.cli_pickup if status == ARRIVED_PICKUP else self.cli_drop
        if not cli.service_is_ready():
            self.get_logger().warn(
                "mission_arrive 서비스 미준비 — 다음 tick 재시도",
                throttle_duration_sec=2.0,
            )
            return
        fut = cli.call_async(Trigger.Request())
        self._call_inflight = True
        fut.add_done_callback(self._on_arrive_resp)

    def _on_arrive_resp(self, fut):
        self._call_inflight = False
        self._pending_fire = None
        try:
            resp = fut.result()
            ok = bool(resp.success)
        except Exception:
            ok = False
        self.ctl.on_service_ack(ok, self._now())
        self.get_logger().warn("mission_arrive ACK success=%s" % ok)

    def _srv_reset(self, request, response):
        del request
        self.ctl.reset(self._now())
        self._active_mission_id = None
        self._resume_prev = False
        self._pending_fire = None
        response.success = True
        response.message = "reset"
        return response


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
