"""BENCH/RViz 전용 L515 장애물 구역 실험. production command authority가 아니다.

    /l515/points ─→ [이 노드] ─→ /diagnostics/obstacle/points
                                 ├→ /diagnostics/obstacle/state
                                 └→ /diagnostics/obstacle/speed_scale

2026-07-07 L515 1차 검증(윈도우 PC)의 좌/중/우 3분할 GO·SLOW·STOP 판정을 ROS 로 옮기되,
그때 미해결로 남았던 두 문제를 푼다.

────────────────────────────────────────────────────────────────────────
① 바닥을 장애물로 오인하는 문제 — **바닥 평면을 추정해서 푼다**
────────────────────────────────────────────────────────────────────────
1차 검증은 카메라 기준 거리만 봤다. 그러면 카메라가 조금만 아래를 봐도 **바닥이 가까운
장애물로 잡힌다**(1차 검증 문서의 확인 항목 "바닥을 너무 장애물로 잡지 않는가").

순진한 해법 "높이 z 가 크면 장애물"도 두 상황에서 깨진다:
  · **경사로** — 앞의 오르막 바닥이 통째로 높아져 벽으로 오인된다.
  · **로커보기 관절** — 바퀴는 땅에 있는데 **차체만 기울어진다**. 차체 기준 z 가 틀어진다.

그래서 **바닥 평면을 점구름에서 직접 추정**한다(z = a·x + b·y + c). 경사로면 평면이 같이
기울고, 차체가 기울어도 평면이 따라간다. 높이는 **그 평면으로부터** 잰다.
장애물 점이 평면 추정을 오염시키지 않도록 **MAD 기반 아웃라이어 제거**를 반복한다
(`chassis/odometry.py` 의 슬립 배제와 같은 발상 — 계통 오차와 이상치를 구분한다).

────────────────────────────────────────────────────────────────────────
② 경계에서 덜컥거리는 문제 — **히스테리시스**
────────────────────────────────────────────────────────────────────────
고정 임계값이면 0.79 m → SLOW, 0.81 m → GO 가 반복돼 속도 명령이 떨린다
(1차 검증 "11.2 히스테리시스 적용" 계획). 상태별로 진입·이탈 임계를 다르게 둔다:
      GO→SLOW 0.75 이하 / SLOW→GO 0.90 이상
      SLOW→STOP 0.40 이하 / STOP→SLOW 0.50 이상

🛑 **이 노드는 안전 최종 게이트가 아니다.** 최종 게이팅은 US-100(독립 초음파) +
   `SafetyInterlock` 이 한다. 여기는 **감속 힌트**를 주는 인지 계층이다. depth 는
   검은 물체·반사체·얇은 물체에서 구멍이 나므로(1차 검증 한계 항목) 단독으로 신뢰하지
   않는다. 값이 없으면 `UNKNOWN` → 보수적으로 감속(speed_scale 0.3)한다.
"""
import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

GROUND_RGB = struct.unpack("f", struct.pack("I", 0x00808088))[0]   # 회색
OBSTACLE_RGB = struct.unpack("f", struct.pack("I", 0x00E03C28))[0]  # 빨강


class ObstacleZones(Node):
    def __init__(self):
        super().__init__("obstacle_zones")
        # 바닥/장애물
        self.declare_parameter("ground_band_m", 0.06)     # 평면 ±이 값 = 바닥
        self.declare_parameter("obstacle_min_h", 0.08)    # 이보다 높아야 장애물
        self.declare_parameter("obstacle_max_h", 1.20)    # 이보다 높으면 무시(천장·간판)
        # 관심 영역 (차체 기준)
        self.declare_parameter("x_min", 0.15)
        self.declare_parameter("x_max", 4.00)
        self.declare_parameter("zone_half_w", 0.35)       # 중앙 구역 반폭
        self.declare_parameter("side_w", 0.45)            # 좌/우 구역 폭
        # 판정 임계 (히스테리시스)
        self.declare_parameter("go_to_slow", 0.75)
        self.declare_parameter("slow_to_go", 0.90)
        self.declare_parameter("slow_to_stop", 0.40)
        self.declare_parameter("stop_to_slow", 0.50)
        self.declare_parameter("min_obstacle_points", 25)  # 노이즈 몇 점으로 STOP 금지

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.state = "UNKNOWN"
        self._plane = None
        self._n = 0

        self.pub_cloud = self.create_publisher(PointCloud2, "/diagnostics/obstacle/points",
                                               qos_profile_sensor_data)
        self.pub_state = self.create_publisher(String, "/diagnostics/obstacle/state", 10)
        self.pub_scale = self.create_publisher(
            Float32, "/diagnostics/obstacle/speed_scale", 10
        )
        # ★ depth=1 — **항상 최신 프레임만 처리하고 밀린 것은 버린다.**
        #   처리가 입력(10 Hz)보다 조금만 느려도 큐가 무한히 쌓여 발행 타임스탬프가 점점
        #   과거가 된다(실측: 0.9초 → 13초까지 벌어짐). 그러면 RViz 가 TF 캐시 범위를
        #   벗어난 메시지로 보고 전부 버린다. 센서 인지에서는 **늦은 프레임보다 최신
        #   프레임이 옳다** — 오래된 장애물 정보로 감속 판단을 내리면 안 된다.
        self.create_subscription(
            PointCloud2, "/l515/points", self._on_cloud,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_timer(2.0, self._log)
        self.get_logger().info("obstacle_zones 시작 — 바닥평면 추정 + 좌/중/우 판정")

    # ── 바닥 평면 추정 ───────────────────────────────────────────────────

    def _fit_ground(self, pts):
        """z = a·x + b·y + c 를 강건 최소자승으로 추정. 장애물 점은 아웃라이어로 배제.

        차체 앞 근거리 대역만 후보로 쓴다 — 멀수록 depth 노이즈가 커지고, 먼 벽이
        평면을 끌어당긴다. 배제는 절대 임계가 아니라 **중앙값 + k·MAD** 로 한다
        (계통 오차와 이상치를 구분 — odometry.py 와 같은 원리).
        """
        m = (pts[:, 0] > 0.25) & (pts[:, 0] < 2.5) & (np.abs(pts[:, 1]) < 1.0)
        cand = pts[m]
        if cand.shape[0] < 100:
            return None

        for _ in range(3):
            A = np.column_stack([cand[:, 0], cand[:, 1], np.ones(len(cand))])
            sol, *_ = np.linalg.lstsq(A, cand[:, 2], rcond=None)
            resid = np.abs(A @ sol - cand[:, 2])
            med = float(np.median(resid))
            mad = float(np.median(np.abs(resid - med)))
            keep = resid <= max(0.03, med + 3.0 * mad)
            if keep.all() or keep.sum() < 100:
                break
            cand = cand[keep]
        return sol                                    # (a, b, c)

    # ── 메인 ─────────────────────────────────────────────────────────────

    def _on_cloud(self, msg: PointCloud2):
        # 광학 프레임 → 차체 프레임. **촬영 시각의 TF** 로 조회해야 메시지와 정합한다
        # (최신 TF 로 변환해놓고 옛 타임스탬프를 붙이면 좌표와 시각이 어긋난다).
        try:
            tf = self.tf_buf.lookup_transform(
                "base_link", msg.header.frame_id, msg.header.stamp)
        except Exception:
            try:                                      # 아직 그 시각 TF 가 없으면 최신으로
                tf = self.tf_buf.lookup_transform(
                    "base_link", msg.header.frame_id, rclpy.time.Time())
            except Exception:
                return                                # TF 자체가 아직 안 옴

        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)
        pts = _apply_tf(pts, tf)

        plane = self._fit_ground(pts)
        if plane is None:
            self._emit("UNKNOWN", 0.3, {})            # 바닥을 못 찾음 → 보수적 감속
            return
        self._plane = plane

        a, b, c = plane
        height = pts[:, 2] - (a * pts[:, 0] + b * pts[:, 1] + c)

        band = float(self.get_parameter("ground_band_m").value)
        h_min = float(self.get_parameter("obstacle_min_h").value)
        h_max = float(self.get_parameter("obstacle_max_h").value)
        x_min = float(self.get_parameter("x_min").value)
        x_max = float(self.get_parameter("x_max").value)

        in_roi = (pts[:, 0] > x_min) & (pts[:, 0] < x_max)
        is_ground = in_roi & (np.abs(height) < band)
        is_obst = in_roi & (height > h_min) & (height < h_max)

        dists = self._zone_distances(pts, is_obst)
        state, scale = self._decide(dists.get("center"))
        self._emit(state, scale, dists)
        self._publish_cloud(msg, pts, is_ground, is_obst)
        self._n += 1

    def _zone_distances(self, pts, is_obst):
        """좌·중·우 각 구역에서 **가장 가까운 장애물까지의 전방거리**."""
        half = float(self.get_parameter("zone_half_w").value)
        side = float(self.get_parameter("side_w").value)
        need = int(self.get_parameter("min_obstacle_points").value)

        y, x = pts[:, 1], pts[:, 0]
        zones = {
            "left":   is_obst & (y >= half) & (y < half + side),
            "center": is_obst & (np.abs(y) < half),
            "right":  is_obst & (y <= -half) & (y > -(half + side)),
        }
        out = {}
        for name, m in zones.items():
            if int(m.sum()) >= need:                  # 노이즈 몇 점으로 STOP 하지 않는다
                out[name] = float(np.percentile(x[m], 5))   # 최소값 대신 5% 분위 = 강건
        return out

    def _decide(self, center):
        """중앙 거리 → 상태. **히스테리시스**로 경계에서 덜컥거리지 않게 한다."""
        if center is None:
            self.state = "GO"                         # 중앙에 장애물 없음
            return "GO", 1.0

        g2s = float(self.get_parameter("go_to_slow").value)
        s2g = float(self.get_parameter("slow_to_go").value)
        s2p = float(self.get_parameter("slow_to_stop").value)
        p2s = float(self.get_parameter("stop_to_slow").value)

        s = self.state
        if s == "STOP":
            s = "SLOW" if center >= p2s else "STOP"
        elif s == "SLOW":
            s = "STOP" if center <= s2p else ("GO" if center >= s2g else "SLOW")
        else:                                          # GO / UNKNOWN
            s = "STOP" if center <= s2p else ("SLOW" if center <= g2s else "GO")
        self.state = s
        return s, {"GO": 1.0, "SLOW": 0.3, "STOP": 0.0}[s]

    def _emit(self, state, scale, dists):
        self.pub_state.publish(String(data=state))
        self.pub_scale.publish(Float32(data=float(scale)))
        self._last = (state, scale, dists)

    def _publish_cloud(self, src, pts, is_ground, is_obst):
        """바닥=회색 / 장애물=빨강 으로 색을 입혀 RViz 로 보낸다."""
        sel = is_ground | is_obst
        if not sel.any():
            return
        xyz = pts[sel]
        rgb = np.where(is_obst[sel], OBSTACLE_RGB, GROUND_RGB).astype(np.float32)
        data = np.column_stack([xyz, rgb]).astype(np.float32)

        msg = PointCloud2()
        msg.header.stamp = src.header.stamp
        msg.header.frame_id = "base_link"             # 이미 차체 기준으로 변환했다
        msg.height = 1
        msg.width = data.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * data.shape[0]
        msg.is_dense = True
        msg.data = data.tobytes()
        self.pub_cloud.publish(msg)

    def _log(self):
        if self._n == 0 or not hasattr(self, "_last"):
            self.get_logger().info("점구름 대기 중 (/l515/points)")
            return
        state, scale, d = self._last
        zs = "  ".join(f"{k[0].upper()}={v:.2f}m" for k, v in sorted(d.items())) or "장애물 없음"
        a, b, c = self._plane if self._plane is not None else (0, 0, 0)
        self.get_logger().info(
            f"{state:7s} scale={scale:.1f}  {zs}  "
            f"| 바닥평면 기울기 {math.degrees(math.atan(a)):+.1f}° (전후)")


def _apply_tf(pts, tf):
    """PointCloud 를 TF 로 변환 (쿼터니언 회전 + 병진)."""
    q = tf.transform.rotation
    t = tf.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)
    return pts @ R.T + np.array([t.x, t.y, t.z], dtype=np.float32)


def main():
    rclpy.init()
    node = ObstacleZones()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
