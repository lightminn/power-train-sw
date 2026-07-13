"""벽 추종 — 합성 점군으로 검증한다 (정답을 우리가 안다).

★ 핵심: **거리만 보면 안 되고 각도도 봐야 한다.** 거리 하나로 PID 를 걸면 벽에
  비스듬히 다가가는 중인지 평행한지 구분을 못 해 **S자로 진동**한다.
"""
import math

import numpy as np
import pytest

from vision.wall import (
    LEFT, RIGHT, WallConfig, WallFollower, WallResult, detect_wall,
)


def wall_points(distance=0.6, angle_deg=0.0, side=RIGHT, n=300,
                x_range=(0.3, 2.3), height=(0.2, 0.8), noise=0.0, seed=0):
    """차체 기준 벽 점군을 만든다.

    distance : 벽까지의 수직 거리 [m]
    angle_deg: 벽이 차체 방향과 이루는 각 (+ = 앞으로 갈수록 벌어짐)
    """
    rng = np.random.default_rng(seed)
    x = rng.uniform(*x_range, n)
    a = math.tan(math.radians(angle_deg))
    sign = -1.0 if side == RIGHT else +1.0
    # 수직거리 d 인 직선: y = a·x + b,  |b|/sqrt(1+a²) = d
    b = sign * distance * math.sqrt(1.0 + a * a)
    y = a * x + b
    if noise:
        y = y + rng.normal(0.0, noise, n)
    z = rng.uniform(*height, n)
    return np.column_stack([x, y, z])


# ── 검출 ─────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("d", [0.4, 0.6, 1.0])
def test_distance_is_measured_in_meters(d):
    res = detect_wall(wall_points(distance=d), WallConfig(side=RIGHT))
    assert res.ok
    assert res.distance_m == pytest.approx(d, abs=0.02)


@pytest.mark.parametrize("ang", [-15.0, -5.0, 0.0, 5.0, 15.0])
def test_heading_is_measured(ang):
    """★ 각도를 못 재면 S자 진동을 못 잡는다."""
    res = detect_wall(wall_points(distance=0.6, angle_deg=ang), WallConfig(side=RIGHT))
    assert res.ok
    assert math.degrees(res.heading_rad) == pytest.approx(ang, abs=1.5)


def test_left_and_right_sides():
    cfg_r, cfg_l = WallConfig(side=RIGHT), WallConfig(side=LEFT)
    right = detect_wall(wall_points(0.7, side=RIGHT), cfg_r)
    left = detect_wall(wall_points(0.7, side=LEFT), cfg_l)
    assert right.ok and left.ok
    assert right.distance_m == pytest.approx(0.7, abs=0.02)
    assert left.distance_m == pytest.approx(0.7, abs=0.02)


def test_wrong_side_sees_nothing():
    """오른쪽 벽만 있는데 왼쪽을 보라고 하면 못 본다."""
    res = detect_wall(wall_points(0.6, side=RIGHT), WallConfig(side=LEFT))
    assert not res.ok


def test_floor_and_ceiling_are_excluded():
    """★ 바닥을 벽으로 잡으면 거리가 엉망이 된다."""
    wall = wall_points(0.6, side=RIGHT, n=200, height=(0.2, 0.8))
    floor = np.column_stack([                       # 바닥 (z≈0) — 옆으로 넓게 퍼짐
        np.random.default_rng(1).uniform(0.3, 2.3, 400),
        np.random.default_rng(2).uniform(-1.4, -0.1, 400),
        np.zeros(400),
    ])
    res = detect_wall(np.vstack([wall, floor]), WallConfig(side=RIGHT))
    assert res.ok
    assert res.distance_m == pytest.approx(0.6, abs=0.03)   # 바닥에 안 끌려간다


def test_corner_is_rejected():
    """★ 모서리에서는 직선 모델이 안 맞는다 → **조향하면 안 된다.**

    벽이 꺾이는 지점에서 억지로 직선을 맞추면 엉뚱한 각도가 나와 벽에 박는다.
    잔차로 "이건 직선이 아니다"를 판정해 `ok=False` 로 돌려준다.
    """
    # 앞으로 가다가 x=1.2 에서 오른쪽으로 꺾이는 벽 (ㄱ자 모서리)
    a = np.column_stack([np.linspace(0.3, 1.2, 150),
                         np.full(150, -0.6), np.full(150, 0.5)])
    b = np.column_stack([np.full(150, 1.2),
                         np.linspace(-0.6, -1.4, 150), np.full(150, 0.5)])
    res = detect_wall(np.vstack([a, b]), WallConfig(side=RIGHT))
    assert not res.ok
    assert "직선이 아님" in res.detail


def test_gentle_curve_within_roi_is_accepted():
    """관심영역 안에서 거의 직선인 완만한 곡선은 받아들인다 — 과하게 까다로우면 못 쓴다."""
    x = np.linspace(0.3, 2.3, 300)
    y = -0.6 - 0.02 * (x - 0.3) ** 2
    res = detect_wall(np.column_stack([x, y, np.full(300, 0.5)]),
                      WallConfig(side=RIGHT))
    assert res.ok


def test_too_few_points():
    res = detect_wall(wall_points(0.6, n=10), WallConfig(side=RIGHT))
    assert not res.ok
    assert "부족" in res.detail


def test_noise_is_tolerated():
    res = detect_wall(wall_points(0.6, noise=0.01), WallConfig(side=RIGHT))
    assert res.ok
    assert res.distance_m == pytest.approx(0.6, abs=0.03)


def test_bad_shape_is_rejected():
    assert not detect_wall(np.zeros((10, 2)), WallConfig()).ok


# ── 추종 ─────────────────────────────────────────────────────────────────

def test_too_far_steers_toward_wall():
    """오른쪽 벽에서 멀어지면 오른쪽으로 (ω < 0)."""
    f = WallFollower(WallConfig(side=RIGHT, target_m=0.6, kh=0.0))
    _, w, ok = f.update(WallResult(True, distance_m=0.9))
    assert ok and w < 0


def test_too_close_steers_away():
    f = WallFollower(WallConfig(side=RIGHT, target_m=0.6, kh=0.0))
    _, w, _ = f.update(WallResult(True, distance_m=0.3))
    assert w > 0


def test_left_side_signs_are_mirrored():
    f = WallFollower(WallConfig(side=LEFT, target_m=0.6, kh=0.0))
    _, w, _ = f.update(WallResult(True, distance_m=0.9))
    assert w > 0                                    # 왼쪽 벽에서 멀면 왼쪽으로


def test_heading_term_damps_oscillation():
    """★ 각도항이 있어야 S자 진동이 잡힌다.

    거리는 맞는데(오차 0) 벽 쪽으로 **비스듬히 다가가는 중**이면, 거리항만으로는
    ω=0 이라 그대로 벽에 박는다. 각도항이 그걸 되돌린다.
    """
    approaching = WallResult(True, distance_m=0.6,      # 거리 오차 0
                             heading_rad=math.radians(-10.0))  # 벽 쪽으로 기울어짐

    only_distance = WallFollower(WallConfig(side=RIGHT, target_m=0.6, kh=0.0))
    _, w0, _ = only_distance.update(approaching)
    assert w0 == pytest.approx(0.0)                  # ← 아무것도 안 한다 (벽에 박는다)

    with_heading = WallFollower(WallConfig(side=RIGHT, target_m=0.6, kh=1.4))
    _, w1, _ = with_heading.update(approaching)
    assert w1 < 0                                    # 각도항이 되돌린다


def test_omega_is_clamped():
    f = WallFollower(WallConfig(kp=100.0, omega_max=1.0))
    _, w, _ = f.update(WallResult(True, distance_m=5.0))
    assert abs(w) <= 1.0 + 1e-9


def test_no_wall_no_steering():
    """★ 못 보면 조향하지 않는다 — 마지막 명령을 반복하지도 않는다."""
    f = WallFollower(WallConfig())
    assert f.update(WallResult(False)) == (0.0, 0.0, False)
