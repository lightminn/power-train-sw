# live_overlay 7판 부품 시험: 바닥 검출기(ground_est)가 합성 바닥에서 높이·숙임을 되찾고 천장을 바닥으로 받지 않음,
# 비스듬히 보기(live_map.oblique_image)가 시점 범위 안에서 같은 크기로 그리고 모든 지도 줄이 화면에 보임(음성 대조 포함).
import os, sys

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("cv2")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ground_est import GroundDetector  # noqa: E402
import live_map  # noqa: E402

H, W, F = 270, 480, 340.0


def scene(h=0.72, pitch_deg=19.5, ceiling=None):
    """카메라 높이 h, 숙임 pitch의 바닥(+상자·벽) 깊이(uint16, 0.25 mm 단위)와 카메라 좌표 위 방향."""
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    xn, yn = (u - W / 2) / F, (v - H / 2) / F
    t = np.radians(pitch_deg)
    up = np.array([0.0, -np.cos(t), -np.sin(t)])
    den = yn * np.cos(t) + np.sin(t)
    z = np.where(den > 0.05, h / np.maximum(den, 1e-6), 0.0)
    z[(np.abs(xn) < 0.15) & (yn > 0.0) & (yn < 0.15)] = 1.4
    if ceiling is not None:                                   # 위쪽 화면 = 카메라 위 ceiling m 천장
        den_c = -(yn * np.cos(t) + np.sin(t))
        zc = np.where(den_c > 0.05, ceiling / np.maximum(den_c, 1e-6), 0.0)
        z = np.where(zc > 0, zc, z)
    z = np.where(z > 6, 0, z) + np.random.default_rng(0).normal(0, 0.005, z.shape) * (z > 0)
    return (z / 0.00025).astype(np.uint16), up


def detector(up):
    d = GroundDetector(F, F, W / 2, H / 2, 0.00025, shape=(H, W), max_dist=5.0, hmin=0.12, hmax=1.6)
    d.up = up
    return d


def test_ground_height_and_tilt():
    depth, up = scene()
    d = detector(up)
    for _ in range(3):
        r = d.process(depth)
    assert r["status"] == "OK"
    assert abs(r["camera_height_m"] - 0.72) < 0.01
    assert abs(r["camera_tilt_deg"] - 19.5) < 0.5
    assert d.P is not None and d.P.shape == (H, W, 3) and d.valid.shape == (H, W)


def test_ceiling_not_taken_as_floor():
    depth, up = scene(ceiling=1.2)                            # 천장이 바닥보다 가까워도 '가장 낮은 면' = 바닥
    d = detector(up)
    for _ in range(3):
        r = d.process(depth)
    assert r["status"] == "OK"
    assert abs(r["camera_height_m"] - 0.72) < 0.02


def _grid(nf=60, nl=50):
    Z = np.zeros((nf, nl))
    Z[20:25, 10:20] = 0.1
    known = np.ones((nf, nl), bool)
    C = np.full((nf, nl, 3), (60, 160, 60), np.uint8)
    return Z, C, known


@pytest.mark.parametrize("tilt", [0.0, 0.35, 1.0, -1.0, 2.0])
def test_oblique_shape_and_bounds(tilt):
    Z, C, known = _grid()
    img = live_map.oblique_image(Z, C, known, W=640, Hh=240, fwd_max=1.8, tilt=tilt)
    assert img.shape == (240, 640, 3) and img.dtype == np.uint8


def _stripes(img, W):
    """가운데 열에서 칠한 구간 안 색 경계 수."""
    col = img[:, W // 2].astype(int)
    ys = np.nonzero((col[:, 1] > 90) & (col[:, 0] < 90))[0]
    seg = col[ys[0]:ys[-1] + 1]
    return int(np.sum(np.any(np.diff(seg, axis=0) != 0, axis=1)))


def _striped(nf=117, nl=100):
    Z, known = np.zeros((nf, nl)), np.ones((nf, nl), bool)
    C = np.zeros((nf, nl, 3), np.uint8)
    C[0::2], C[1::2] = (60, 160, 60), (60, 100, 200)        # 지도 줄마다 색을 번갈아
    return Z, C, known


@pytest.mark.parametrize("tilt", [0.0, 0.35, 0.7, 1.0])
def test_oblique_every_map_row_visible(tilt):
    """실제 지도 크기(3.5 m ÷ 3 cm 줄)·화면 높이 300에서 모든 지도 줄이 화면에 남음(경계 = 줄 수 − 1)."""
    Z, C, known = _striped()
    img = live_map.oblique_image(Z, C, known, W=960, Hh=300, fwd_max=3.5, tilt=tilt)
    assert _stripes(img, 960) == Z.shape[0] - 1


def test_oblique_row_check_detects_drop():
    """음성 대조: 화면이 너무 낮아 줄 하나가 1 px보다 얇으면 위 시험 조건이 실패해야 함(조건 자체가 줄 빠짐을 잡는지)."""
    Z, C, known = _striped()
    img = live_map.oblique_image(Z, C, known, W=960, Hh=110, fwd_max=3.5, tilt=0.0)
    assert _stripes(img, 960) < Z.shape[0] - 1


def test_dip_without_trench_rule_is_not_pass():
    """trench_depth_m이 없는 옛 기준값(v0.2)에서도 6.5 cm 꺼진 칸이 '통과'로 보이지 않음(턱 기준 보수 대체 — step_down_m 15/30 cm로 보면 통과가 됨)."""
    import json
    rules = json.load(open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "obstacle_criteria_v0.2.json")))["rules"]
    assert "trench_depth_m" not in rules
    Z = np.full((3, 3), -0.065)
    lv, dip = live_map.classify_heights(Z, np.ones((3, 3), bool), rules)
    assert (lv[1, 1] >= 2) and dip[1, 1]
