"""L515 depth 이미지 → PointCloud2 (WP6 Step 4).

    실행 (powertrain_ros 컨테이너 안):
        source /opt/ros/humble/setup.bash
        python3 /workspace/ros2/src/powertrain_ros/powertrain_ros/l515_cloud_node.py

Gateway 는 **PointCloud2 를 의도적으로 발행하지 않는다**(계약: color/depth/camera_info/
accel/gyro 6토픽만). 그 계약을 깨지 않기 위해 Gateway 를 건드리지 않고, depth 이미지를
구독해 여기서 클라우드를 만든다. CPU 도 분리된다.

────────────────────────────────────────────────────────────────────────
핵심 수치 (실측으로 확인함 — 추측 아님)
────────────────────────────────────────────────────────────────────────
· **depth_scale = 0.00025 m/unit** (L515 는 1/4000. D400 계열의 1 mm 가 **아니다**).
  raw 중앙값 12792 → 0.25 mm 가정 시 3.2 m(방 크기, 타당) / 1 mm 가정 시 12.8 m
  (L515 최대사거리 9 m 초과 → 불가능). raw max 32470 → 8.1 m 로 사거리 안.
  ⚠️ 이걸 틀리면 모든 거리가 4배로 나온다.
· Gateway 는 raw **Z16 을 `16UC1` 로 그대로** 발행한다(스케일 변환 안 함).
· 토픽 QoS = **BEST_EFFORT**(sensor data). RELIABLE 로 구독하면 한 프레임도 못 받는다.
· 내부파라미터는 `/l515/depth/camera_info` 의 K 에서 읽는다(하드코딩 금지).
· depth 원본 10 Hz (Gateway 정책).

프레임: 결과 클라우드는 `l515_depth_optical_frame`(광학 규약: x=오른쪽, y=아래, z=앞)에
그대로 둔다. odom/base_link 로의 변환은 TF 가 한다 → `imu_tilt_node.py` 가 TF 를 쏜다.
"""
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

DEPTH_SCALE_M = 0.00025          # L515 (1/4000). ⚠️ D400 의 0.001 이 아니다.


class L515CloudNode(Node):
    def __init__(self):
        super().__init__("l515_cloud")
        self.declare_parameter("stride", 2)          # 픽셀 간격 — 2면 320×240 (≈4만점)
        self.declare_parameter("min_range_m", 0.20)
        self.declare_parameter("max_range_m", 9.00)  # L515 사거리 상한
        self.declare_parameter("depth_scale_m", DEPTH_SCALE_M)
        # ⚠️ Gateway 의 타임스탬프를 **믿지 않는다** — 아래 설명 참조
        self.declare_parameter("restamp_ros_time", True)

        self._k = None                               # (fx, fy, cx, cy)
        self._uv = None                              # 픽셀 격자 캐시
        self._shape = None

        self.create_subscription(CameraInfo, "/l515/depth/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/l515/depth/image_rect_raw",
                                 self._on_depth, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, "/l515/points",
                                         qos_profile_sensor_data)
        self._n = 0
        self.get_logger().info("l515_cloud 시작 — /l515/points 발행")

    def _on_info(self, msg: CameraInfo):
        self._k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])     # fx, fy, cx, cy

    def _grid(self, h, w, stride):
        """픽셀 좌표 격자를 캐시 (매 프레임 다시 만들지 않는다)."""
        if self._uv is None or self._shape != (h, w, stride):
            vs, us = np.mgrid[0:h:stride, 0:w:stride]
            self._uv = (us.astype(np.float32), vs.astype(np.float32))
            self._shape = (h, w, stride)
        return self._uv

    def _on_depth(self, msg: Image):
        if self._k is None:
            return
        stride = int(self.get_parameter("stride").value)
        scale = float(self.get_parameter("depth_scale_m").value)
        lo = float(self.get_parameter("min_range_m").value)
        hi = float(self.get_parameter("max_range_m").value)

        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        d = depth[::stride, ::stride].astype(np.float32) * scale       # → m
        us, vs = self._grid(msg.height, msg.width, stride)

        ok = (d > lo) & (d < hi)
        if not ok.any():
            return
        z = d[ok]
        fx, fy, cx, cy = self._k
        # 역투영: 픽셀 + 깊이 → 3D (광학 프레임: x=오른쪽, y=아래, z=앞)
        x = (us[ok] - cx) * z / fx
        y = (vs[ok] - cy) * z / fy

        pts = np.stack([x, y, z], axis=1).astype(np.float32)

        header = msg.header
        if bool(self.get_parameter("restamp_ros_time").value):
            # ★ Gateway 의 타임스탬프를 ROS 시계로 다시 찍는다.
            #   `l515_adapter.TimeMapper` 는 장치시계→ROS시계 오프셋을 **처음 한 번만**
            #   잡고 다시 보정하지 않는다. 두 시계는 크리스털이 달라 속도가 미세하게
            #   다르므로 시간이 갈수록 선형 드리프트한다. 게다가 `_offset_ns` 를 **모든
            #   스트림이 공유**하는데 RealSense 의 color/depth/IMU 는 시계 도메인이 다를
            #   수 있어, 한 스트림의 재앵커가 다른 스트림의 오프셋까지 갈아엎는다.
            #   실측: TF 대비 +1.4초 → −3.7초로 점프. 그러면 TF 조회가 캐시 범위를
            #   벗어나 **RViz 가 모든 프레임을 버린다**.
            #   depth 실지연(~30–100 ms)은 이 드리프트에 비하면 무시할 수준이라
            #   발행 시각으로 다시 찍는 편이 훨씬 정확하다.
            #   ⚠️ 근본 수정은 Gateway 쪽(주기적 재앵커 + 스트림별 오프셋)이다.
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id
        self.pub.publish(self._cloud_msg(header, pts))

        self._n += 1
        if self._n % 50 == 0:
            self.get_logger().info(
                f"{self._n} 프레임  점 {pts.shape[0]}개  "
                f"거리 {z.min():.2f}~{z.max():.2f} m (중앙 {np.median(z):.2f})")

    @staticmethod
    def _cloud_msg(header, pts):
        msg = PointCloud2()
        msg.header = header                    # frame_id = l515_depth_optical_frame 유지
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * pts.shape[0]
        msg.is_dense = True
        msg.data = pts.tobytes()
        return msg


def main():
    rclpy.init()
    node = L515CloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
