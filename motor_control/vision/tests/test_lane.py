"""레인 추종 코어 검증 — 합성 영상으로 '성질'을 테스트한다.

실제 트랙 없이도 검증 가능하다: **지면 위에 레인을 그려놓고, 그걸 카메라로 투영해서**
입력 영상을 만든다(= 우리 호모그래피의 역방향). 그러면 정답(횡오차·헤딩)을 우리가 안다.

⚠️ 이 테스트가 증명하는 것과 못 하는 것
  · 증명함 : 원근 유도·중심선 추출·PID 의 **기하학적 정합성**. 못 보면 조향 안 하는 것.
  · 못 함  : **실제 조명·재질·트랙에서의 이진화 성능.** 흰 레인 가정(HSV V>170, S<80)이
             대회 트랙에서 통하는지는 현장 튜닝이 필요하다.
"""
import math

import cv2
import numpy as np
import pytest

from vision.lane import (
    LaneConfig, LaneFollower, _ground_to_pixel, _rot_x, _rot_y,
    binarize, detect, ground_homography, lane_center,
)

# L515 depth 실측 내부파라미터 (640×480)
K = (464.52, 464.25, 351.89, 245.49)
IMG = (480, 640)          # h, w


def cfg(**kw):
    c = LaneConfig(cam_pitch_deg=20.0)      # 아래로 20° 숙임 (바닥이 보이게)
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def _R(c: LaneConfig):
    R_opt2body = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
    return _rot_y(math.radians(c.cam_pitch_deg)) @ R_opt2body


def render_lane(c: LaneConfig, lane_y_at, width_m=0.08, bg=40, fg=245):
    """지면 위 레인을 카메라 영상으로 **투영**해서 합성 입력을 만든다.

    lane_y_at(x) : 전방거리 x 에서 레인 중심의 횡위치 [m] (+ = 왼쪽)
    """
    img = np.full((*IMG, 3), bg, dtype=np.uint8)
    C = np.array([c.cam_x_m, 0.0, c.cam_height_m])
    R = _R(c)
    pts = []
    for x in np.linspace(c.look_near_m, c.look_far_m, 120):
        y = lane_y_at(x)
        for dy in (-width_m / 2, +width_m / 2):
            try:
                u, v = _ground_to_pixel((x, y + dy), K, R, C)
            except ValueError:
                continue
            if 0 <= u < IMG[1] and 0 <= v < IMG[0]:
                pts.append((u, v, x, dy))
    # 레인 폭만큼 굵게 그린다
    for x in np.linspace(c.look_near_m, c.look_far_m, 240):
        y = lane_y_at(x)
        try:
            uL, vL = _ground_to_pixel((x, y + width_m / 2), K, R, C)
            uR, vR = _ground_to_pixel((x, y - width_m / 2), K, R, C)
        except ValueError:
            continue
        cv2.line(img, (int(round(uL)), int(round(vL))),
                 (int(round(uR)), int(round(vR))), (fg, fg, fg), 2)
    return img


# ── 호모그래피 ───────────────────────────────────────────────────────────

def test_ground_homography_is_invertible():
    M = ground_homography(K, cfg())
    assert abs(np.linalg.det(M)) > 1e-9


def test_camera_must_look_at_ground():
    """카메라가 수평이면 look_far 지면점이 지평선 근처라 투영이 불안정하다.
    아래로 숙여야 한다 — 마운트 각도가 잘못되면 여기서 걸린다."""
    with pytest.raises(ValueError):
        c = cfg(cam_pitch_deg=-30.0)          # 위를 본다
        ground_homography(K, c)


# ── 중심선 추출 (정답을 아는 합성 영상) ──────────────────────────────────

def test_straight_lane_centered():
    c = cfg()
    img = render_lane(c, lambda x: 0.0)              # 정중앙 직선
    res = detect(img, K, c)
    assert res.ok
    assert abs(res.offset_m) < 0.03                  # 3 cm 이내
    assert abs(res.heading_rad) < math.radians(2.0)


@pytest.mark.parametrize("y0", [+0.20, -0.20, +0.35, -0.35])
def test_offset_lane_is_measured_in_meters(y0):
    """★ 출력이 **미터 단위**다 — 4점 눈대중 방식은 픽셀만 준다."""
    c = cfg()
    img = render_lane(c, lambda x: y0)
    res = detect(img, K, c)
    assert res.ok
    assert res.offset_m == pytest.approx(y0, abs=0.04)


def test_sign_convention_left_is_positive():
    c = cfg()
    left = detect(render_lane(c, lambda x: +0.25), K, c)
    right = detect(render_lane(c, lambda x: -0.25), K, c)
    assert left.offset_m > 0 and right.offset_m < 0


def test_heading_error_of_slanted_lane():
    """레인이 비스듬하면 헤딩오차가 잡혀야 한다 (기울기 = tan θ)."""
    c = cfg()
    slope = 0.20                                     # 1 m 전방마다 0.2 m 왼쪽
    img = render_lane(c, lambda x: slope * (x - c.lookahead_m))
    res = detect(img, K, c)
    assert res.ok
    assert res.heading_rad == pytest.approx(math.atan(slope), abs=math.radians(3.0))
    assert abs(res.offset_m) < 0.05                  # lookahead 지점에서는 중앙


def test_curved_lane_has_curvature():
    c = cfg()
    img = render_lane(c, lambda x: 0.12 * x * x)     # 왼쪽으로 휨
    res = detect(img, K, c)
    assert res.ok
    assert res.curvature > 0.05


# ── 못 볼 때 ─────────────────────────────────────────────────────────────

def test_empty_image_is_not_ok():
    """★ 레인을 못 보면 `ok=False` — **조향하면 안 된다.**"""
    c = cfg()
    img = np.full((*IMG, 3), 30, dtype=np.uint8)     # 아무것도 없음
    res = detect(img, K, c)
    assert not res.ok
    assert res.n_bands < c.min_bands


def test_partial_lane_below_min_bands():
    """레인이 몇 조각만 보이면 `ok=False` — 조각으로 직선을 맞추면 엉뚱한 데로 간다."""
    from vision.lane import bev_size
    c = cfg(bands=6, min_bands=4)
    w, h = bev_size(c)
    mask = np.zeros((h, w), dtype=np.uint8)
    band_h = h // c.bands
    for i in (0, 1):                                  # 6개 띠 중 2개만 채운다
        mask[i * band_h:(i + 1) * band_h, w // 2 - 5:w // 2 + 5] = 255
    res = lane_center(mask, c)
    assert not res.ok
    assert res.n_bands == 2


def test_binarize_ignores_dark_and_colored():
    c = cfg()
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    img[0:5] = (30, 30, 30)                           # 어두움
    img[5:8] = (0, 0, 220)                            # 빨강 (채도 높음)
    img[8:10] = (250, 250, 250)                       # 흰색
    mask = binarize(img, c)
    assert mask[9, 5] > 0                             # 흰색만 잡힌다
    assert mask[1, 5] == 0 and mask[6, 5] == 0


# ── 추종 (PID) ───────────────────────────────────────────────────────────

def test_follower_steers_toward_lane():
    f = LaneFollower(cfg())
    c = cfg()
    left = detect(render_lane(c, lambda x: +0.3), K, c)
    v, w, ok = f.update(left, 0.02)
    assert ok and w > 0                               # 레인이 왼쪽 → 왼쪽으로 (ω>0)

    f.reset()
    right = detect(render_lane(c, lambda x: -0.3), K, c)
    _, w2, _ = f.update(right, 0.02)
    assert w2 < 0


def test_follower_omega_is_clamped():
    f = LaneFollower(cfg(kp=100.0))
    from vision.lane import LaneResult
    _, w, _ = f.update(LaneResult(True, offset_m=1.0), 0.02)
    assert abs(w) <= f.cfg.omega_max + 1e-9


def test_follower_stops_when_lane_lost():
    """★ 못 보면 **마지막 명령을 반복하지 않는다** — 상위가 결정하게 한다."""
    f = LaneFollower(cfg())
    from vision.lane import LaneResult
    f.update(LaneResult(True, offset_m=0.3), 0.02)
    v, w, ok = f.update(LaneResult(False), 0.02)
    assert (v, w, ok) == (0.0, 0.0, False)


def test_follower_integral_resets_on_loss():
    f = LaneFollower(cfg(ki=1.0))
    from vision.lane import LaneResult
    for _ in range(20):
        f.update(LaneResult(True, offset_m=0.2), 0.02)
    assert f._i > 0
    f.update(LaneResult(False), 0.02)
    assert f._i == 0.0                                # 적분 와인드업이 남으면 안 된다


# ── 오검출 방지 (벽·바닥) ────────────────────────────────────────────────

def test_wide_bright_area_is_not_a_lane():
    """★ 흰 벽·밝은 바닥을 레인으로 착각하면 안 된다.

    폭 검사가 없으면 넓게 퍼진 밝은 영역의 **무게중심이 잡혀 '완벽히 중앙인 레인'**
    이 되어버린다. 실제로 레인 없는 사무실에서 계속 '레인 OK' 가 떴다.
    """
    from vision.lane import bev_size
    c = cfg()
    w, h = bev_size(c)
    mask = np.full((h, w), 255, dtype=np.uint8)      # 전면이 하얗다 (벽)
    res = lane_center(mask, c)
    assert not res.ok
    assert "넓" in res.detail


def test_narrow_stripe_is_a_lane():
    """좁은 띠(실제 레인)는 통과해야 한다 — 폭 검사가 과하면 안 된다."""
    from vision.lane import bev_size
    c = cfg()
    w, h = bev_size(c)
    mask = np.zeros((h, w), dtype=np.uint8)
    stripe = int(0.08 * c.px_per_m)                  # 8 cm 폭 레인
    mask[:, w // 2 - stripe // 2: w // 2 + stripe // 2] = 255
    res = lane_center(mask, c)
    assert res.ok
    assert abs(res.offset_m) < 0.02


def test_lane_width_threshold():
    """max_lane_width_m 경계에서 갈린다."""
    from vision.lane import bev_size
    c = cfg(max_lane_width_m=0.30)
    w, h = bev_size(c)

    def _mask(width_m):
        m = np.zeros((h, w), dtype=np.uint8)
        px = int(width_m * c.px_per_m)
        m[:, w // 2 - px // 2: w // 2 + px // 2] = 255
        return m

    assert lane_center(_mask(0.20), c).ok           # 20 cm → 레인
    assert not lane_center(_mask(0.60), c).ok       # 60 cm → 레인 아님
