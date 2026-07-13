"""레인 추종 ROS 래퍼 (WP7).

    /l515/color/image_raw ─┐
    /l515/color/camera_info ├─→ [이 노드] ─→ /lane/state  (인식 결과 · 항상 발행)
    /imu/filtered ─────────┘                └─→ /autonomy/cmd_vel (⚠️ `enabled:=true` 일 때만)
                                                 → chassis_node authority → 모터

계산은 전부 순수 코어(`motor_control/vision/lane.py`, pytest 17종)가 한다. 여기서는
메시지를 옮기고 주기를 맞출 뿐이다 — 레포 원칙 그대로.

────────────────────────────────────────────────────────────────────────
🛑 `/cmd_vel` 을 **직접 쓰지 않는다**
────────────────────────────────────────────────────────────────────────
`/cmd_vel` 은 **단일 입력 경로**여야 한다 — authority가 내장된 `chassis_node`만 받는다.
여기서는 `/autonomy/cmd_vel` 로 **제안**만 하고, 실제 전달 여부는 authority가 정한다.
  · **항상** `/lane/state` 로 인식 결과를 내보낸다 (디버깅·튜닝용)
  · `enabled:=true` 일 때만 `/autonomy/cmd_vel` 로 제안한다
  · authority 가 AUTO 모드이고 **중립을 확인**해야 비로소 모터로 간다
  · 모드 전환: `ros2 service call /chassis_node/authority_auto std_srvs/srv/Trigger`

⚠️ **레인을 못 보면 아무것도 발행하지 않는다.** 마지막 명령을 반복하면 로봇이 레인을
   잃은 채로 계속 달린다. 상위(미션 시퀀서)가 정지·복구를 결정해야 한다.
   `chassis_node` 의 명령 워치독(300 ms)이 자연히 구동을 0 으로 내린다.

⚠️ obstacle zone의 `/diagnostics/obstacle/speed_scale`은 BENCH/RViz 진단 전용이며
   production `chassis_node`에 연결하지 않는다. 실제 충돌 안전은 US-100이 담당한다.
"""
import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import Marker

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from vision.lane import (                                   # noqa: E402
    LaneConfig, LaneFollower, bev_size, binarize, ground_homography, lane_center,
)
import cv2                                                  # noqa: E402


class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__("lane_follower")
        self.declare_parameter("enabled", False)            # 🛑 /cmd_vel 발행 여부
        self.declare_parameter("use_imu_tilt", True)        # 경사에서 지면 보정
        self.declare_parameter("cam_height_m", 0.35)        # ⚠️ 미실측 플레이스홀더
        self.declare_parameter("cam_pitch_deg", 20.0)       # ⚠️ 미실측 플레이스홀더
        self.declare_parameter("cam_x_m", 0.30)
        self.declare_parameter("v_nominal", 0.5)
        self.declare_parameter("kp", 1.6)
        self.declare_parameter("kd", 0.25)

        self.cfg = LaneConfig(
            cam_height_m=float(self.get_parameter("cam_height_m").value),
            cam_pitch_deg=float(self.get_parameter("cam_pitch_deg").value),
            cam_x_m=float(self.get_parameter("cam_x_m").value),
            v_nominal=float(self.get_parameter("v_nominal").value),
            kp=float(self.get_parameter("kp").value),
            kd=float(self.get_parameter("kd").value),
        )
        self.follower = LaneFollower(self.cfg)

        self._K = None
        self._M = None                                       # 호모그래피 캐시
        self._tilt = (0.0, 0.0)
        self._t_prev = None
        self._n = 0

        self.create_subscription(CameraInfo, "/l515/color/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/l515/color/image_raw",
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/filtered", self._on_imu,
                                 qos_profile_sensor_data)
        # 미션 시퀀서가 정차를 명령하면(팔 작업 중) **제안을 멈춘다.**
        # 여기서 0 을 계속 쏘면 chassis_node 워치독(300ms)이 영영 안 터진다.
        self._allow_drive = True
        self.create_subscription(Bool, "/mission/allow_drive",
                                 lambda m: setattr(self, "_allow_drive", m.data), 10)

        self.pub_state = self.create_publisher(Float32MultiArray, "/lane/state", 10)
        self.pub_cmd = self.create_publisher(Twist, "/autonomy/cmd_vel", 10)
        # RViz 로 보는 인식 결과 — 로그 숫자만으론 튜닝이 안 된다
        self.pub_bev = self.create_publisher(Image, "/lane/bev", qos_profile_sensor_data)
        self.pub_marker = self.create_publisher(Marker, "/lane/marker", 10)
        self.create_timer(2.0, self._log)

        if bool(self.get_parameter("enabled").value):
            self.get_logger().warn(
                "/autonomy/cmd_vel 제안 ON — 실제 주행은 chassis_node authority가 "
                "AUTO 모드일 때만"
            )
        else:
            self.get_logger().info(
                "/autonomy/cmd_vel 제안 OFF (인식 결과만 /lane/state 로 발행)"
            )

    def _on_info(self, msg: CameraInfo):
        K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        if K != self._K:
            self._K = K
            self._M = None                                   # 파라미터 바뀜 → 재계산

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self._tilt = (roll, pitch)

    def _on_image(self, msg: Image):
        if self._K is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self._t_prev is None else t - self._t_prev
        self._t_prev = t

        roll, pitch = self._tilt if bool(
            self.get_parameter("use_imu_tilt").value) else (0.0, 0.0)
        # 기울임이 바뀌면 호모그래피를 다시 만든다(경사에서 지면이 안 틀어지게)
        self._M = ground_homography(self._K, self.cfg, roll, pitch)

        bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        mask = binarize(bgr, self.cfg)
        bev = cv2.warpPerspective(mask, self._M, bev_size(self.cfg))
        res = lane_center(bev, self.cfg)

        self.pub_state.publish(Float32MultiArray(data=[
            1.0 if res.ok else 0.0, res.offset_m, res.heading_rad,
            res.curvature, float(res.n_bands),
        ]))

        v, omega, ok = self.follower.update(res, dt)
        self._last = (res, v, omega)
        self._n += 1
        # ⚠️ Gateway 타임스탬프는 ROS 시계와 드리프트한다(l515_adapter.TimeMapper).
        #    그대로 쓰면 RViz 가 TF 캐시 범위 밖이라며 전부 버린다 → ROS 시계로 찍는다.
        self._publish_debug(self.get_clock().now().to_msg(), bev, res)

        # ⚠️ 못 보면 **아무것도 발행하지 않는다** — 마지막 명령을 반복하지 않는다.
        #    미션 시퀀서가 정차를 명령해도 마찬가지다(팔이 뻗어 있을 수 있다).
        if ok and self._allow_drive and bool(self.get_parameter("enabled").value):
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = omega
            self.pub_cmd.publish(cmd)

    def _publish_debug(self, stamp, bev, res):
        """버드아이 마스크 + 추정된 레인 중심선을 RViz 로 보낸다.

        숫자 로그만으론 "왜 저기를 레인이라고 하는지" 알 수 없다. 눈으로 봐야 튜닝된다.
        """
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = "base_link"
        img.height, img.width = bev.shape[:2]
        img.encoding = "mono8"
        img.step = img.width
        img.data = bev.tobytes()
        self.pub_bev.publish(img)

        # 추정된 레인 중심선을 지면 위 선으로 (base_link 기준, 미터 단위)
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = "base_link"
        m.ns = "lane"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD if res.ok else Marker.DELETE
        m.scale.x = 0.04
        m.color.a = 1.0
        m.color.g = 1.0 if res.ok else 0.0
        m.color.r = 0.0 if res.ok else 1.0
        m.pose.orientation.w = 1.0
        if res.ok:
            slope = math.tan(res.heading_rad)
            b = res.offset_m - slope * self.cfg.lookahead_m
            for i in range(11):
                x = self.cfg.look_near_m + i * (
                    self.cfg.look_far_m - self.cfg.look_near_m) / 10.0
                m.points.append(Point(
                    x=float(x), y=float(slope * x + b), z=0.02))
        self.pub_marker.publish(m)

    def _log(self):
        if self._n == 0 or not hasattr(self, "_last"):
            self.get_logger().info("영상 대기 중 (/l515/color/image_raw)")
            return
        res, v, omega = self._last
        if res.ok:
            self.get_logger().info(
                f"레인 OK  횡오차 {res.offset_m:+.3f} m  헤딩 "
                f"{math.degrees(res.heading_rad):+.1f}°  곡률 {res.curvature:+.3f}  "
                f"띠 {res.n_bands}  → v={v:.2f} ω={omega:+.2f}")
        else:
            self.get_logger().warn(f"레인 못 봄 ({res.detail}) — 조향하지 않는다")


def main():
    rclpy.init()
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
