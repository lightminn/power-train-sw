"""L515 내장 IMU → 차체 기울임 TF (WP6 Step 3).

    실행 (powertrain_ros 컨테이너 안):
        source /opt/ros/humble/setup.bash
        python3 /workspace/ros2/src/powertrain_ros/powertrain_ros/imu_tilt_node.py

**맵도, 오도메트리도, URDF 도 없이** RViz 에서 로봇(과 포인트클라우드)이 기울어지게
만드는 최소 배관. `odom→base_link` TF 의 **회전 성분**에 IMU 자세를 넣으면 끝이다.

────────────────────────────────────────────────────────────────────────
상보 필터 (EKF 안 씀)
────────────────────────────────────────────────────────────────────────
· **자이로**: 각속도. 적분하면 각도가 되지만 **서서히 드리프트**한다. 단기엔 매우 정확.
· **가속도계**: 정지 시 중력 방향을 알려줘 roll/pitch 의 **절대 기준**이 된다. 대신
  로봇이 가속하면 흔들려 노이즈가 크다.
· 섞는다:  각도 = α·(자이로 적분) + (1−α)·(가속도계가 말하는 각도),  α≈0.98
  → 자이로의 단기 정확성 + 가속도계의 장기 무드리프트.

**yaw 는 절대 기준이 없다**(중력은 수평 회전을 모른다). 자이로 적분뿐이라 계속 드리프트한다.
지자기 센서도 GPS 도 없으므로 이건 원리적 한계다. WP6 오도메트리에서 휠 회전과 섞어 쓰고
(원칙: **바퀴=거리, IMU=회전**), 전역 보정은 우리 범위 밖이다.

────────────────────────────────────────────────────────────────────────
좌표 변환 — 여기서 틀리면 화면이 엉뚱하게 돈다
────────────────────────────────────────────────────────────────────────
L515 IMU 는 **광학 규약**(x=오른쪽, y=아래, z=앞)으로 값을 준다. 실측 확인:
정지 상태 accel = (0.21, **−9.61**, −0.04) → 중력이 −y 로 읽힘 → **y 가 아래**가 맞다.

REP-103 차체 규약(x=앞, y=왼쪽, z=위)으로 바꾸면:
        body_x = +opt_z        (앞    = 광학의 앞)
        body_y = −opt_x        (왼쪽  = 광학 오른쪽의 반대)
        body_z = −opt_y        (위    = 광학 아래의 반대)
검산: accel(0.21, −9.61, −0.04) → body(−0.04, −0.21, **+9.61**) → z 축(위)에 중력 ✓

TF 트리:
    odom ──(IMU 자세: roll/pitch/yaw)── base_link ──(마운트)── l515_link
                                                        └─(광학회전)─ l515_depth_optical_frame

⚠️ `base_link→l515_link` 마운트 값은 **아직 실측 전이다**(WP6 남은 커미셔닝). 아래 기본값은
플레이스홀더 — 마운트 조립 후 실측해서 파라미터로 넘겨라:
    --ros-args -p mount_x:=0.30 -p mount_z:=0.35
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,      # x
            cr * sp * cy + sr * cp * sy,      # y
            cr * cp * sy - sr * sp * cy,      # z
            cr * cp * cy + sr * sp * sy)      # w


def opt_to_body(v):
    """광학 규약 벡터 → REP-103 차체 규약 (위 docstring 의 검산 참조)."""
    return (v.z, -v.x, -v.y)


class ImuTiltNode(Node):
    def __init__(self):
        super().__init__("imu_tilt")
        self.declare_parameter("alpha", 0.98)          # 상보 필터 가중 (자이로 쪽)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("bias_samples", 200)    # 기동 시 자이로 편향 측정 샘플 수
        # ⚠️ 마운트 = 미실측 플레이스홀더 (WP6 남은 커미셔닝)
        self.declare_parameter("mount_x", 0.30)
        self.declare_parameter("mount_y", 0.0)
        self.declare_parameter("mount_z", 0.35)
        # URDF + robot_state_publisher 를 쓰면 base_link→l515_link 와 광학회전을 그쪽이
        # 발행한다. **같은 TF 를 두 곳에서 쏘면 충돌**하므로 그때는 false 로 끈다.
        self.declare_parameter("publish_static_tf", True)
        # 오도메트리 노드(WP6 Step 2)가 붙으면 `odom→base_link` 는 **그쪽이 소유**한다
        # (병진 + 회전을 한 변환에 담아야 하므로). 그때는 false 로 끄고, 여기서는
        # `/imu/filtered` 로 자세만 넘긴다.
        self.declare_parameter("publish_odom_tf", True)

        self.roll = self.pitch = self.yaw = 0.0
        self._accel = None                             # 최신 가속도 (body 규약)
        self._t_prev = None
        self._settled = False
        # 자이로 편향(bias): 가만히 있어도 0 이 아닌 값이 나온다 → 적분하면 yaw 가 흘러간다.
        # 기동 직후 **로봇이 정지해 있다고 보고** 평균을 재서 빼준다.
        # ⚠️ 기동 중 로봇이 움직이고 있으면 그 움직임을 편향으로 잘못 배운다.
        self._bias = [0.0, 0.0, 0.0]
        self._bias_acc = [0.0, 0.0, 0.0]
        self._bias_n = 0

        self._own_odom_tf = bool(self.get_parameter("publish_odom_tf").value)
        self.tf = TransformBroadcaster(self)
        # 필터링된 자세 + 편향보정된 각속도 — 오도메트리 노드가 회전 소스로 쓴다
        self.imu_pub = self.create_publisher(Imu, "/imu/filtered", qos_profile_sensor_data)
        self._gyro_body = (0.0, 0.0, 0.0)

        if bool(self.get_parameter("publish_static_tf").value):
            self.static_tf = StaticTransformBroadcaster(self)
            self._publish_static()
        else:
            self.get_logger().info("정적 TF 는 robot_state_publisher(URDF)가 발행 — 생략")
        if not self._own_odom_tf:
            self.get_logger().info("odom→base_link 는 오도메트리 노드가 발행 — 생략")

        self.create_subscription(Imu, "/l515/accel/sample", self._on_accel,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, "/l515/gyro/sample", self._on_gyro,
                                 qos_profile_sensor_data)
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self._publish_tf)
        self.create_timer(2.0, self._log)
        self.get_logger().info("imu_tilt 시작 — odom→base_link TF 발행 (회전만)")

    # ── 정적 TF: 마운트 + 광학 회전 ──────────────────────────────────────

    def _publish_static(self):
        mx = float(self.get_parameter("mount_x").value)
        my = float(self.get_parameter("mount_y").value)
        mz = float(self.get_parameter("mount_z").value)

        mount = TransformStamped()
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.header.frame_id = "base_link"
        mount.child_frame_id = "l515_link"
        mount.transform.translation.x = mx
        mount.transform.translation.y = my
        mount.transform.translation.z = mz
        mount.transform.rotation.w = 1.0               # 카메라가 정면을 본다고 가정

        # l515_link(REP-103) → 광학 프레임: 표준 카메라 회전 RPY(−90°, 0, −90°)
        optical = TransformStamped()
        optical.header.stamp = mount.header.stamp
        optical.header.frame_id = "l515_link"
        optical.child_frame_id = "l515_depth_optical_frame"
        qx, qy, qz, qw = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)
        optical.transform.rotation.x = qx
        optical.transform.rotation.y = qy
        optical.transform.rotation.z = qz
        optical.transform.rotation.w = qw

        self.static_tf.sendTransform([mount, optical])

    # ── IMU ──────────────────────────────────────────────────────────────

    def _on_accel(self, msg: Imu):
        self._accel = opt_to_body(msg.linear_acceleration)

    def _on_gyro(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t_prev is None:
            self._t_prev = t
            return
        dt = t - self._t_prev
        self._t_prev = t
        if not (0.0 < dt < 0.5):                       # 튄 타임스탬프는 버린다
            return
        if self._accel is None:
            return

        wx, wy, wz = opt_to_body(msg.angular_velocity)

        # ── 기동 시 자이로 편향 측정 (정지 가정) ──
        need = int(self.get_parameter("bias_samples").value)
        if self._bias_n < need:
            for i, w in enumerate((wx, wy, wz)):
                self._bias_acc[i] += w
            self._bias_n += 1
            if self._bias_n == need:
                self._bias = [a / need for a in self._bias_acc]
                self.get_logger().info(
                    "자이로 편향 보정 완료 "
                    f"({', '.join(f'{math.degrees(b):+.3f}°/s' for b in self._bias)})")
            return

        wx -= self._bias[0]
        wy -= self._bias[1]
        wz -= self._bias[2]
        ax, ay, az = self._accel

        # 가속도계가 말하는 절대 자세 (중력 방향)
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.hypot(ay, az))

        if not self._settled:                          # 첫 샘플은 가속도계로 초기화
            self.roll, self.pitch, self._settled = roll_acc, pitch_acc, True
            return

        a = float(self.get_parameter("alpha").value)
        self.roll = a * (self.roll + wx * dt) + (1 - a) * roll_acc
        self.pitch = a * (self.pitch + wy * dt) + (1 - a) * pitch_acc
        self.yaw += wz * dt                            # ⚠️ 절대기준 없음 → 드리프트
        self._gyro_body = (wx, wy, wz)                 # 편향 보정 후 (body 규약)

    # ── 출력 ─────────────────────────────────────────────────────────────

    def _publish_tf(self):
        if not self._settled:
            return
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat_from_rpy(self.roll, self.pitch, self.yaw)

        # 자세 + 각속도를 토픽으로 — 오도메트리 노드의 회전 소스
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"          # REP-103 으로 변환된 값이다
        imu.orientation.x, imu.orientation.y = qx, qy
        imu.orientation.z, imu.orientation.w = qz, qw
        imu.angular_velocity.x = self._gyro_body[0]
        imu.angular_velocity.y = self._gyro_body[1]
        imu.angular_velocity.z = self._gyro_body[2]
        self.imu_pub.publish(imu)

        if not self._own_odom_tf:
            return                                  # 오도메트리 노드가 TF 를 소유
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        # 병진은 0 — 오도메트리(WP6 Step 2)가 붙으면 그쪽이 채운다
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

    def _log(self):
        if self._settled:
            self.get_logger().info(
                f"roll={math.degrees(self.roll):+6.1f}°  "
                f"pitch={math.degrees(self.pitch):+6.1f}°  "
                f"yaw={math.degrees(self.yaw):+6.1f}° (드리프트)")


def main():
    rclpy.init()
    node = ImuTiltNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
