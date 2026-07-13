"""벽 추종 ROS 래퍼 (WP7).

    /l515/points ─→ [이 노드] ─→ /wall/state   (거리·각도 · 항상 발행)
                              └─→ /wall/marker (RViz — 추정된 벽)
                              └─→ /cmd_vel/auto (⚠️ `enabled:=true` 일 때만)

계산은 순수 코어(`motor_control/vision/wall.py`, pytest 22종)가 한다.

🛑 `/cmd_vel` 을 직접 쓰지 않는다 — `command_authority` 만 쓴다. 여기서는 `/cmd_vel/auto`
   로 **제안**만 한다.
⚠️ **레인 추종과 동시에 켜면 안 된다** — 둘 다 `/cmd_vel/auto` 를 쓴다. 상위(미션
   시퀀서)가 구간에 따라 하나만 고른다. 레인(흰 선)이 있으면 레인, 없으면(복도·터널) 벽.
⚠️ 벽을 못 보면 **아무것도 발행하지 않는다.** 마지막 명령을 반복하면 벽을 잃은 채
   계속 달린다.
"""
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from vision.wall import (                                    # noqa: E402
    LEFT, RIGHT, WallConfig, WallFollower, detect_wall,
)


def _apply_tf(pts, tf):
    q, t = tf.transform.rotation, tf.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)
    return pts @ R.T + np.array([t.x, t.y, t.z], dtype=np.float32)


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower")
        self.declare_parameter("enabled", False)         # 🛑 /cmd_vel/auto 제안 여부
        self.declare_parameter("side", RIGHT)
        self.declare_parameter("target_m", 0.6)
        self.declare_parameter("kp", 1.2)
        self.declare_parameter("kh", 1.4)                # 각도항 — S자 진동을 잡는다
        self.declare_parameter("v_nominal", 0.5)

        self.cfg = WallConfig(
            side=str(self.get_parameter("side").value),
            target_m=float(self.get_parameter("target_m").value),
            kp=float(self.get_parameter("kp").value),
            kh=float(self.get_parameter("kh").value),
            v_nominal=float(self.get_parameter("v_nominal").value),
        )
        self.follower = WallFollower(self.cfg)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self._allow_drive = True
        self._n = 0
        self._last = None

        # depth=1 — 늦은 프레임보다 최신 프레임이 옳다 (obstacle_zones 와 같은 이유)
        self.create_subscription(
            PointCloud2, "/l515/points", self._on_cloud,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(Bool, "/mission/allow_drive",
                                 lambda m: setattr(self, "_allow_drive", m.data), 10)

        self.pub_state = self.create_publisher(Float32MultiArray, "/wall/state", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel/auto", 10)
        self.pub_marker = self.create_publisher(Marker, "/wall/marker", 10)
        self.create_timer(2.0, self._log)

        self.get_logger().info(
            f"wall_follower 시작 — {self.cfg.side} 벽, 목표 {self.cfg.target_m} m, "
            f"제안 {'ON' if bool(self.get_parameter('enabled').value) else 'OFF'}")

    def _on_cloud(self, msg: PointCloud2):
        try:
            tf = self.tf_buf.lookup_transform(
                "base_link", msg.header.frame_id, msg.header.stamp)
        except Exception:
            try:
                tf = self.tf_buf.lookup_transform(
                    "base_link", msg.header.frame_id, rclpy.time.Time())
            except Exception:
                return

        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        res = detect_wall(_apply_tf(pts, tf), self.cfg)
        self._last = res
        self._n += 1

        self.pub_state.publish(Float32MultiArray(data=[
            1.0 if res.ok else 0.0, res.distance_m, res.heading_rad,
            res.residual_m, float(res.n_points),
        ]))
        self._publish_marker(res)

        v, omega, ok = self.follower.update(res)
        # ⚠️ 못 보면 아무것도 발행하지 않는다. 미션이 정차를 명령해도 마찬가지.
        if ok and self._allow_drive and bool(self.get_parameter("enabled").value):
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = omega
            self.pub_cmd.publish(cmd)

    def _publish_marker(self, res):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()   # ⚠️ ROS 시계 (Gateway 드리프트 회피)
        m.header.frame_id = "base_link"
        m.ns = "wall"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD if res.ok else Marker.DELETE
        m.scale.x = 0.05
        m.color.a = 1.0
        m.color.b = 1.0
        m.pose.orientation.w = 1.0
        if res.ok:
            a = math.tan(res.heading_rad)
            sign = -1.0 if self.cfg.side == RIGHT else 1.0
            b = sign * res.distance_m * math.sqrt(1.0 + a * a)
            for i in range(11):
                x = self.cfg.x_min + i * (self.cfg.x_max - self.cfg.x_min) / 10.0
                m.points.append(Point(x=float(x), y=float(a * x + b), z=0.3))
        self.pub_marker.publish(m)

    def _log(self):
        if self._last is None:
            self.get_logger().info("점군 대기 중 (/l515/points)")
            return
        r = self._last
        if r.ok:
            v, omega, _ = self.follower.update(r)
            self.get_logger().info(
                f"벽 OK  거리 {r.distance_m:.2f} m (목표 {self.cfg.target_m})  "
                f"각도 {math.degrees(r.heading_rad):+.1f}°  잔차 {r.residual_m:.3f}  "
                f"점 {r.n_points}  → ω={omega:+.2f}")
        else:
            self.get_logger().warn(f"벽 못 봄 ({r.detail}) — 조향하지 않는다")


def main():
    rclpy.init()
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
