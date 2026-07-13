"""obstacle_zones_node — 바닥 평면 추정 · 구역 판정 · 히스테리시스.

🛑 이 노드는 **감속 힌트**지 안전 게이트가 아니다(게이트 = US-100 + SafetyInterlock).
   테스트도 그 전제로 읽는다.
"""
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped

from powertrain_ros.obstacle_zones_node import ObstacleZones, _apply_tf


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = ObstacleZones()
    yield n
    n.destroy_node()


def _tf(x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 1.0)):
    t = TransformStamped()
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
    t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
    return t


# ── TF 적용 ──────────────────────────────────────────────────────────────

def test_apply_tf_translation():
    pts = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    out = _apply_tf(pts, _tf(x=10.0, z=-1.0))
    assert out[0] == pytest.approx([11.0, 2.0, 2.0])


def test_apply_tf_rotation_90deg_z():
    """z축 +90° 회전: x축이 y축으로 간다."""
    q = (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    out = _apply_tf(np.array([[1.0, 0.0, 0.0]], dtype=np.float32), _tf(q=q))
    assert out[0] == pytest.approx([0.0, 1.0, 0.0], abs=1e-6)


# ── 바닥 평면 추정 ───────────────────────────────────────────────────────

def _plane_cloud(a=0.0, b=0.0, c=0.0, n=800, seed=0):
    """z = a·x + b·y + c 평면 위의 점들 (관심 대역 안에)."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.3, 2.4, n)
    y = rng.uniform(-0.9, 0.9, n)
    z = a * x + b * y + c
    return np.column_stack([x, y, z]).astype(np.float32)


def test_fit_ground_flat(node):
    pts = _plane_cloud(c=-0.35)                       # 수평 바닥, 35 cm 아래
    a, b, c = node._fit_ground(pts)
    assert (a, b) == pytest.approx((0.0, 0.0), abs=1e-3)
    assert c == pytest.approx(-0.35, abs=1e-3)


def test_fit_ground_slope(node):
    """★ 경사로도 따라간다 — 이게 '높이 z 가 크면 장애물' 순진한 방식과의 차이다.

    오르막에서는 앞의 바닥이 통째로 높아지는데, 평면을 추정하면 그걸 바닥으로 인식한다.
    """
    pts = _plane_cloud(a=0.2, c=-0.35)                # 앞으로 갈수록 올라감
    a, b, c = node._fit_ground(pts)
    assert a == pytest.approx(0.2, abs=0.01)


def test_fit_ground_ignores_obstacles(node):
    """★ 장애물 점이 평면 추정을 오염시키면 안 된다 (MAD 아웃라이어 제거)."""
    ground = _plane_cloud(c=-0.35, n=800)
    obstacle = np.column_stack([                      # 앞 1.2 m 에 선 벽
        np.full(300, 1.2), np.linspace(-0.4, 0.4, 300), np.linspace(-0.3, 0.6, 300),
    ]).astype(np.float32)
    a, b, c = node._fit_ground(np.vstack([ground, obstacle]))
    assert c == pytest.approx(-0.35, abs=0.03)        # 벽에 끌려가지 않음
    assert abs(a) < 0.05


def test_fit_ground_too_few_points(node):
    assert node._fit_ground(_plane_cloud(n=10)) is None


# ── 히스테리시스 ─────────────────────────────────────────────────────────
#  GO→SLOW 0.75 / SLOW→GO 0.90 / SLOW→STOP 0.40 / STOP→SLOW 0.50

def test_hysteresis_go_to_slow_and_back(node):
    """★ 고정 임계값이면 0.79↔0.81 에서 상태가 덜컥거린다. 그래서 진입·이탈을 다르게 둔다."""
    assert node._decide(1.20)[0] == "GO"
    assert node._decide(0.80)[0] == "GO"        # 0.75 초과 → 아직 GO
    assert node._decide(0.70)[0] == "SLOW"      # 0.75 이하 → SLOW
    assert node._decide(0.80)[0] == "SLOW"      # ★ 0.90 미만이면 SLOW 유지 (안 튄다)
    assert node._decide(0.95)[0] == "GO"        # 0.90 이상 → GO 복귀


def test_hysteresis_slow_to_stop_and_back(node):
    node._decide(0.70)                           # SLOW 진입
    assert node._decide(0.35)[0] == "STOP"       # 0.40 이하
    assert node._decide(0.45)[0] == "STOP"       # ★ 0.50 미만이면 STOP 유지
    assert node._decide(0.55)[0] == "SLOW"       # 0.50 이상 → SLOW 복귀


def test_no_obstacle_is_go(node):
    assert node._decide(None) == ("GO", 1.0)


def test_speed_scale_values(node):
    assert node._decide(1.5)[1] == 1.0           # GO
    assert node._decide(0.60)[1] == 0.3          # SLOW
    assert node._decide(0.30)[1] == 0.0          # STOP


# ── 구역 판정 ────────────────────────────────────────────────────────────

def test_zone_distances_center(node):
    pts = np.column_stack([
        np.full(60, 1.5), np.zeros(60), np.full(60, 0.3)]).astype(np.float32)
    d = node._zone_distances(pts, np.ones(60, dtype=bool))
    assert d["center"] == pytest.approx(1.5, abs=0.01)
    assert "left" not in d and "right" not in d


def test_zone_distances_left_right(node):
    left = np.column_stack([np.full(40, 2.0), np.full(40, 0.5), np.zeros(40)])
    right = np.column_stack([np.full(40, 1.0), np.full(40, -0.5), np.zeros(40)])
    pts = np.vstack([left, right]).astype(np.float32)
    d = node._zone_distances(pts, np.ones(80, dtype=bool))
    assert d["left"] == pytest.approx(2.0, abs=0.01)
    assert d["right"] == pytest.approx(1.0, abs=0.01)
    assert "center" not in d


def test_few_points_do_not_trigger(node):
    """★ 노이즈 몇 점으로 STOP 하면 안 된다 (min_obstacle_points)."""
    pts = np.column_stack([
        np.full(5, 0.3), np.zeros(5), np.full(5, 0.3)]).astype(np.float32)
    assert node._zone_distances(pts, np.ones(5, dtype=bool)) == {}
