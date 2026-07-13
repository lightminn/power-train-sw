"""imu_tilt_node — 좌표 변환 · 상보 필터 · 자이로 편향 보정.

여기서 틀리면 RViz 에서 로봇이 **엉뚱하게 기울어진다**. 부호 하나가 전부다.
"""
import math

import pytest
import rclpy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

from powertrain_ros.imu_tilt_node import ImuTiltNode, opt_to_body, quat_from_rpy


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = ImuTiltNode()
    yield n
    n.destroy_node()


def _v(x, y, z):
    return Vector3(x=float(x), y=float(y), z=float(z))


def _imu(accel=None, gyro=None, t=0.0):
    m = Imu()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int((t % 1.0) * 1e9)
    if accel is not None:
        m.linear_acceleration = _v(*accel)
    if gyro is not None:
        m.angular_velocity = _v(*gyro)
    return m


# ── 좌표 변환 (광학 → REP-103) ────────────────────────────────────────────

def test_opt_to_body_gravity():
    """★ 실측 검산. L515 를 수평으로 놓았을 때 accel = (0.21, −9.61, −0.04).

    광학 규약(x=오른쪽, y=아래, z=앞) → REP-103(x=앞, y=왼쪽, z=위) 변환 결과
    **중력이 body z(위) 축에 +9.61 로 떨어져야** 한다. 부호가 틀리면 로봇이 뒤집힌다.
    """
    bx, by, bz = opt_to_body(_v(0.2059, -9.6105, -0.0392))
    assert bz == pytest.approx(+9.6105, abs=1e-3)     # 위쪽에 중력
    assert abs(bx) < 0.1 and abs(by) < 0.3            # 수평 성분은 작다


def test_opt_to_body_axes():
    assert opt_to_body(_v(0, 0, 1)) == (1.0, 0.0, -0.0)    # 광학 앞  → body 앞
    assert opt_to_body(_v(1, 0, 0)) == (0.0, -1.0, -0.0)   # 광학 오른쪽 → body -y
    assert opt_to_body(_v(0, 1, 0)) == (0.0, -0.0, -1.0)   # 광학 아래 → body -z


def test_quat_from_rpy_identity():
    assert quat_from_rpy(0, 0, 0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quat_from_rpy_norm():
    q = quat_from_rpy(0.3, -0.2, 1.1)
    assert sum(v * v for v in q) == pytest.approx(1.0)


# ── 자이로 편향 보정 ──────────────────────────────────────────────────────

def test_gyro_bias_is_learned_and_removed(node):
    """정지 상태에서 자이로가 일정한 오프셋을 내면 그만큼 빼야 한다.

    안 빼면 적분된 yaw 가 계속 흘러간다(실측 0.12°/s → 보정 후 ~0.01°/s).
    """
    node.set_parameters([rclpy.parameter.Parameter(
        "bias_samples", rclpy.Parameter.Type.INTEGER, 10)])
    node._on_accel(_imu(accel=(0.0, -9.81, 0.0)))

    drift = 0.02                                   # rad/s, 광학 z (= body 앞축 roll)
    t = 0.0
    for _ in range(12):                            # 편향 학습 + 그 이후 몇 샘플
        t += 0.01
        node._on_gyro(_imu(gyro=(0.0, 0.0, drift), t=t))

    # 학습된 편향이 실제 오프셋과 일치해야 한다 (body x = opt z)
    assert node._bias[0] == pytest.approx(drift, abs=1e-6)

    # 편향을 뺀 뒤에는 같은 입력이 자세를 흔들지 않는다
    roll_before = node.roll
    for _ in range(50):
        t += 0.01
        node._on_gyro(_imu(gyro=(0.0, 0.0, drift), t=t))
    assert node.roll == pytest.approx(roll_before, abs=1e-3)


def test_bad_dt_is_ignored(node):
    """타임스탬프가 튀면(dt<0 또는 너무 큼) 그 샘플은 버린다 — 적분이 폭발한다."""
    node.set_parameters([rclpy.parameter.Parameter(
        "bias_samples", rclpy.Parameter.Type.INTEGER, 1)])
    node._on_accel(_imu(accel=(0.0, -9.81, 0.0)))
    node._on_gyro(_imu(gyro=(0, 0, 0), t=100.0))
    node._on_gyro(_imu(gyro=(0, 0, 0), t=100.01))
    pitch = node.pitch
    node._on_gyro(_imu(gyro=(0, 0, 5.0), t=50.0))      # 과거로 점프 → 무시
    assert node.pitch == pytest.approx(pitch, abs=1e-9)


# ── 상보 필터 ────────────────────────────────────────────────────────────

def test_accel_defines_absolute_tilt(node):
    """가속도계는 중력으로 **절대 자세**를 준다 — 드리프트가 없어야 한다."""
    node.set_parameters([rclpy.parameter.Parameter(
        "bias_samples", rclpy.Parameter.Type.INTEGER, 1)])
    # 광학 accel 이 (0, -cos, -sin) → body 로 가면 pitch 가 생긴다
    tilt = math.radians(10.0)
    node._on_accel(_imu(accel=(0.0, -9.81 * math.cos(tilt), 9.81 * math.sin(tilt))))
    t = 0.0
    for _ in range(400):                            # 4초간 자이로는 0 (안 움직임)
        t += 0.01
        node._on_gyro(_imu(gyro=(0, 0, 0), t=t))
    # 자이로가 0 이어도 가속도계가 잡아당겨 실제 기울기로 수렴한다
    assert node.pitch == pytest.approx(-tilt, abs=math.radians(1.0))


def test_yaw_has_no_absolute_reference(node):
    """⚠️ yaw 는 중력으로 못 잡는다 — 자이로 적분뿐이라 **원리적으로 드리프트**한다.

    이 테스트는 버그가 아니라 **한계를 문서화**한다. 전역 보정(map→odom)은 범위 밖.
    """
    node.set_parameters([rclpy.parameter.Parameter(
        "bias_samples", rclpy.Parameter.Type.INTEGER, 2)])
    node._on_accel(_imu(accel=(0.0, -9.81, 0.0)))
    t = 0.0
    # ⚠️ 첫 유효 샘플들이 **편향으로 학습**된다 — 정지 상태(0)를 먼저 먹여야 한다.
    #    여기에 회전을 넣으면 그 회전이 편향으로 흡수돼 yaw 가 0 으로 남는다.
    for _ in range(4):
        t += 0.01
        node._on_gyro(_imu(gyro=(0, 0, 0), t=t))
    for _ in range(100):                            # body z(yaw) = 광학 −y
        t += 0.01
        node._on_gyro(_imu(gyro=(0.0, -0.1, 0.0), t=t))
    assert node.yaw == pytest.approx(0.1 * 1.0, rel=0.05)   # 1초간 0.1 rad/s → 0.1 rad
