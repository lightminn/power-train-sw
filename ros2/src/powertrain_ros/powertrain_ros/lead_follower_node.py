"""앞 로봇 추종 ROS 래퍼 (WP7 · 국방 ⑤구간).

    /detected_objects ─→ [이 노드] ─→ /follow/state
                                   ├─→ /follow/active  (→ chassis_node 가 FOLLOW_LEAD 모드로)
                                   └─→ /autonomy/cmd_vel (⚠️ `enabled:=true` 일 때만)

계산은 순수 코어(`motor_control/chassis/follow.py`)가 한다.

🛑 `/cmd_vel` 을 직접 쓰지 않는다 — authority가 내장된 `chassis_node`만 받는다.
⚠️ **레인·벽 추종과 동시에 켜지 않는다** — 셋 다 `/autonomy/cmd_vel` 를 쓴다.
모드 전환: `ros2 service call /chassis_node/authority_auto std_srvs/srv/Trigger`.
⚠️ 추종 중에는 `/chassis_mode` 가 **FOLLOW_LEAD** 가 돼야 팔이 자세를 락한다
   (앞 차 급정거 시 팔이 흔들린다). `/follow/active` 로 chassis_node 에 알린다.
"""
from dataclasses import replace
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import Buffer, TransformListener

from powertrain_ros import contract
from robot_arm_msgs.msg import DetectedObjectArray

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.follow import FollowConfig, FollowResult, LeadFollower    # noqa: E402


FOLLOW_STATE_TRACKING = 1.0
FOLLOW_STATE_PREDICTING = 2.0
FOLLOW_STATE_REACQUIRING = 3.0
FOLLOW_STATE_LOST = 4.0
_FOLLOW_STATE_CODES = {
    "TRACKING": FOLLOW_STATE_TRACKING,
    "PREDICTING": FOLLOW_STATE_PREDICTING,
    "REACQUIRING": FOLLOW_STATE_REACQUIRING,
    "LOST": FOLLOW_STATE_LOST,
}


def _apply_tf(position, tf):
    """Transform one xyz point without making the pure follow core depend on ROS."""
    q = tf.transform.rotation
    # Quaternion-vector rotation: p' = p + w(2q×p) + q×(2q×p).
    px, py, pz = float(position.x), float(position.y), float(position.z)
    tx = 2.0 * (q.y * pz - q.z * py)
    ty = 2.0 * (q.z * px - q.x * pz)
    tz = 2.0 * (q.x * py - q.y * px)
    rx = px + q.w * tx + (q.y * tz - q.z * ty)
    ry = py + q.w * ty + (q.z * tx - q.x * tz)
    rz = pz + q.w * tz + (q.x * ty - q.y * tx)
    translation = tf.transform.translation
    return rx + translation.x, ry + translation.y, rz + translation.z


class LeadFollowerNode(Node):
    def __init__(self, **kwargs):
        super().__init__("lead_follower", **kwargs)
        self.declare_parameter("enabled", False)          # 🛑 autonomy 제안 여부
        self.declare_parameter("class_name", "robot")
        self.declare_parameter("target_m", 2.0)
        self.declare_parameter("min_m", 1.5)              # ★ 이 안이면 무조건 정지
        self.declare_parameter("band_min_m", 1.5)
        self.declare_parameter("band_max_m", 2.5)
        self.declare_parameter("band_gain", 0.2)
        self.declare_parameter("max_m", 6.0)
        self.declare_parameter("kp", 0.8)
        self.declare_parameter("kd", 1.2)                 # ★ 접근 속도 → 추돌 방지
        self.declare_parameter("lost_grace_s", 0.5)
        self.declare_parameter("predict_decay", 0.5)
        self.declare_parameter("predict_limit_s", 1.0)
        self.declare_parameter("reacquire_pos_m", 1.0)
        self.declare_parameter("reacquire_size_min", 0.5)
        self.declare_parameter("reacquire_size_max", 2.0)
        self.declare_parameter("reacquire_max_gap_s", 3.0)
        self.declare_parameter("reacquire_confirm_n", 2)
        self.declare_parameter("tf_stale_s", 0.5)

        # L515 지형 자격 게이트는 여기 걸지 않는다. 이 노드는 L515 를 전혀 쓰지
        # 않는다 — /detected_objects(팔 D435i)와 TF 만 소비하므로 잠정 L515
        # mount/ROI 값이 이 노드의 출력에 영향을 줄 수 없다. 게이트는 L515 기하를
        # 실제로 소비하는 autonomy_controller(/l515/depth/*)와
        # wall_follower(/l515/points)에만 건다.

        self.cfg = FollowConfig(
            class_name=str(self.get_parameter("class_name").value),
            target_m=float(self.get_parameter("target_m").value),
            min_m=float(self.get_parameter("min_m").value),
            band_m=(float(self.get_parameter("band_min_m").value),
                    float(self.get_parameter("band_max_m").value)),
            band_gain=float(self.get_parameter("band_gain").value),
            max_m=float(self.get_parameter("max_m").value),
            kp=float(self.get_parameter("kp").value),
            kd=float(self.get_parameter("kd").value),
            lost_grace_s=float(self.get_parameter("lost_grace_s").value),
            predict_decay=float(self.get_parameter("predict_decay").value),
            predict_limit_s=float(self.get_parameter("predict_limit_s").value),
            reacquire_pos_m=float(self.get_parameter("reacquire_pos_m").value),
            reacquire_size_ratio=(
                float(self.get_parameter("reacquire_size_min").value),
                float(self.get_parameter("reacquire_size_max").value),
            ),
            reacquire_max_gap_s=float(
                self.get_parameter("reacquire_max_gap_s").value
            ),
            reacquire_confirm_n=int(
                self.get_parameter("reacquire_confirm_n").value
            ),
        )
        self.follower = LeadFollower(self.cfg)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self._allow_drive = True
        self._last = None
        self._command_was_publishable = False

        self.create_subscription(DetectedObjectArray, contract.TOPIC_DETECTED,
                                 self._on_detections, 10)
        self.create_subscription(Bool, "/mission/allow_drive",
                                 lambda m: setattr(self, "_allow_drive", m.data), 10)

        self.pub_cmd = self.create_publisher(Twist, "/autonomy/cmd_vel", 10)
        self.pub_state = self.create_publisher(Float32MultiArray, "/follow/state", 10)
        # 추종 중임을 알린다 → chassis_node 가 /chassis_mode = FOLLOW_LEAD 로 (팔 자세 락)
        self.pub_active = self.create_publisher(Bool, "/follow/active", 10)
        self.create_timer(2.0, self._log)
        self.get_logger().info(
            f"lead_follower 시작 — '{self.cfg.class_name}' 추종, 목표 {self.cfg.target_m} m, "
            f"최소 {self.cfg.min_m} m")

    def _on_detections(self, msg: DetectedObjectArray):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        frame_id = msg.header.frame_id.strip()
        if not frame_id:
            self._reject_frame("frame_id 없음 — target 명령 차단", now_s)
            return

        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        age_s = now_s - stamp_s
        stale_s = float(self.get_parameter("tf_stale_s").value)
        if stamp_s <= 0.0:
            self._reject_frame("stamp 없음 — target 명령 차단", now_s)
            return
        if age_s > stale_s:
            self._reject_frame(
                f"detection stamp stale ({age_s:.3f} s) — target 명령 차단", now_s
            )
            return
        if age_s < 0.0:
            self._reject_frame("detection stamp 미래 — target 명령 차단", now_s)
            return

        try:
            tf = self.tf_buf.lookup_transform(
                "base_link", frame_id, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.0),
            )
        except Exception:
            self._reject_frame(
                f"TF 없음 ({frame_id} → base_link) — target 명령 차단", now_s
            )
            return

        dets = []
        for detected in msg.objects:
            forward, left, _ = _apply_tf(detected.pose.position, tf)
            area = float(detected.bbox.width) * float(detected.bbox.height)
            dets.append((detected.class_name, float(detected.confidence),
                         forward, left, area))

        r = self.follower.update(dets, stamp_s)
        self._publish_result(r)

    def _reject_frame(self, reason, now_s):
        # 내부에는 loss를 알려 다음 valid 검출이 연속성 심사를 거치게 하되, 이 frame
        # 자체의 예측 명령은 절대 외부로 내보내지 않는다.
        lost = self.follower.update([], now_s)
        r = replace(lost, ok=False, v=0.0, omega=0.0,
                    reason=reason, state="LOST")
        self._publish_result(r, allow_command=False)

    def _publish_result(self, r: FollowResult, allow_command=True):
        self._last = r

        self.pub_state.publish(Float32MultiArray(data=[
            1.0 if r.ok else 0.0, r.v, r.omega, r.distance_m, r.closing_mps,
            _FOLLOW_STATE_CODES.get(r.state, FOLLOW_STATE_LOST),
        ]))
        self.pub_active.publish(Bool(data=bool(r.ok)))

        command_publishable = bool(
            allow_command
            and r.ok
            and self._allow_drive
            and bool(self.get_parameter("enabled").value)
        )
        if command_publishable:
            cmd = Twist()
            cmd.linear.x = r.v
            cmd.angular.z = r.omega
            self.pub_cmd.publish(cmd)
            self._command_was_publishable = True
        elif self._command_was_publishable:
            self.pub_cmd.publish(Twist())
            self._command_was_publishable = False

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
