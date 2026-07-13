"""레인 추종 — 순수 계산 코어 (WP7).

    RGB → 이진화 → 버드아이(원근보정) → 중심선 추출 → 횡오차·헤딩오차 → PID → ω

하드웨어·ROS 의존 없음(numpy + cv2 만). ROS 래퍼는 `powertrain_ros/lane_follower_node.py`이며
제어 제안은 `/autonomy/cmd_vel` 로 보낸다.
레포 원칙: **순수 Python 코어 + 얇은 rclpy 래퍼.**

────────────────────────────────────────────────────────────────────────
버드아이 변환 — 4점을 손으로 찍지 않는다
────────────────────────────────────────────────────────────────────────
흔한 방식은 이미지에서 사다리꼴 4점을 눈대중으로 찍어 원근을 편다. 그러면 카메라를
조금만 옮겨도 다시 찍어야 하고, **결과가 미터 단위가 아니다**(픽셀 오프셋만 나온다).

우리는 **카메라 내부 파라미터 K 와 지면 평면**을 이미 갖고 있으므로 호모그래피를
**해석적으로 유도**한다. 그러면:
  · 마운트가 바뀌면 **숫자만 바꾸면 된다**(`base_link→l515_link` 실측이 오면 그대로).
  · 출력이 **미터 단위**라 PID 게인이 물리적 의미를 갖고, 조향 기하와 바로 맞물린다.

유도: 광학 프레임의 픽셀 (u,v) → 카메라 좌표 광선 d = K⁻¹·(u,v,1).
차체 좌표로 회전(R)시킨 뒤, 지면(z = −h, h=카메라 높이)과 만나는 점을 구한다:
        p = C + t·(R·d),   t = (−h − C_z) / (R·d)_z
지면 위 점 (x_forward, y_left) 가 바로 버드아이 좌표다. 이 대응을 4점만 잡아
`cv2.getPerspectiveTransform` 으로 3×3 행렬을 만든다(지면은 평면이므로 정확하다).

⚠️ 지면이 **평면이라는 가정**이 깔린다. 경사로 진입·요철에서는 틀어진다. 차체 기울임
   (IMU roll/pitch)을 R 에 넣어 보정할 수 있게 파라미터로 받는다.
"""
from dataclasses import dataclass, field
import math

import cv2
import numpy as np


# ── 설정 ─────────────────────────────────────────────────────────────────


@dataclass
class LaneConfig:
    """레인 인식·추종 파라미터. 대회 트랙에 맞춰 조정한다."""

    # ── 카메라 (base_link 기준 마운트) ──
    # ⚠️ 미실측 플레이스홀더 — `base_link→l515_link` 실측이 오면 교체
    cam_height_m: float = 0.35          # 지면 위 카메라 높이
    cam_pitch_deg: float = 0.0          # 아래로 숙인 각도(+ = 아래)
    cam_x_m: float = 0.30               # base_link 기준 전방 오프셋

    # ── 버드아이 출력 영역 (차체 기준, 미터) ──
    look_near_m: float = 0.4            # 이 앞부터
    look_far_m: float = 2.5             # 여기까지 본다
    half_width_m: float = 1.0           # 좌우 폭
    px_per_m: float = 100.0             # 버드아이 해상도

    # ── 이진화 ──
    # 흰색 레인 기준. 트랙이 다르면 HSV 범위를 바꾼다.
    v_min: int = 170                    # HSV V 하한 (밝기)
    s_max: int = 80                     # HSV S 상한 (무채색 = 흰색)
    blur: int = 5

    # ── 중심선 추출 ──
    bands: int = 6                      # 버드아이를 몇 개 가로띠로 나눌지
    min_px_per_band: int = 30           # 띠당 최소 픽셀 (미달 = 못 봄)
    min_bands: int = 3                  # 이만큼은 봐야 신뢰
    # ★ 레인은 **좁은 띠**다. 이보다 넓게 퍼진 밝은 영역은 레인이 아니라 **벽·바닥**이다.
    #   이 검사가 없으면 흰 벽의 무게중심이 잡혀 "완벽히 중앙인 레인"으로 보인다
    #   (실측: 레인 없는 사무실에서 계속 '레인 OK' 가 떴다).
    max_lane_width_m: float = 0.30
    max_fill_ratio: float = 0.45        # 띠의 이 비율 넘게 채워지면 레인 아님

    # ── 추종 (PID) ──
    lookahead_m: float = 1.2            # 이 거리의 횡오차로 조향한다
    kp: float = 1.6
    ki: float = 0.0
    kd: float = 0.25
    omega_max: float = 1.2              # rad/s
    v_nominal: float = 0.5              # m/s (감속 힌트가 따로 곱해진다)
    i_clamp: float = 0.5


@dataclass(frozen=True)
class LaneResult:
    """레인 인식 결과. `ok=False` 면 조향하지 말 것."""
    ok: bool
    offset_m: float = 0.0               # lookahead 거리에서의 횡오차 (+ = 레인이 왼쪽)
    heading_rad: float = 0.0            # 레인 방향과 차체 방향의 각도차 (+ = 왼쪽)
    curvature: float = 0.0              # 1/m (부호 = 선회 방향)
    n_bands: int = 0                    # 몇 개 띠에서 레인을 봤나
    detail: str = ""


# ── 버드아이 호모그래피 (해석적 유도) ────────────────────────────────────


def ground_homography(K, cfg: LaneConfig, roll=0.0, pitch=0.0):
    """이미지 픽셀 → 버드아이 픽셀 변환 행렬 (3×3).

    K    : (fx, fy, cx, cy) — `/l515/depth/camera_info` 의 값
    roll, pitch : 차체 기울임(rad). IMU 를 물리면 경사에서도 지면이 안 틀어진다.
    """
    fx, fy, cx, cy = K
    Kinv = np.array([[1 / fx, 0, -cx / fx],
                     [0, 1 / fy, -cy / fy],
                     [0, 0, 1]], dtype=np.float64)

    # 광학(x=오른쪽, y=아래, z=앞) → 차체(x=앞, y=왼쪽, z=위)
    R_opt2body = np.array([[0, 0, 1],
                           [-1, 0, 0],
                           [0, -1, 0]], dtype=np.float64)
    R = _rot_y(math.radians(cfg.cam_pitch_deg)) @ R_opt2body   # 카메라 숙임
    R = _rot_x(roll) @ _rot_y(pitch) @ R                        # 차체 기울임 보정

    C = np.array([cfg.cam_x_m, 0.0, cfg.cam_height_m])          # base_link 기준 카메라 위치

    # 지면 위 4점을 잡아 대응 픽셀을 역산한다 (지면은 평면 → 호모그래피가 정확)
    ground = np.array([
        [cfg.look_near_m, +cfg.half_width_m],
        [cfg.look_near_m, -cfg.half_width_m],
        [cfg.look_far_m,  -cfg.half_width_m],
        [cfg.look_far_m,  +cfg.half_width_m],
    ], dtype=np.float64)

    src = np.array([_ground_to_pixel(p, K, R, C) for p in ground], dtype=np.float32)
    dst = np.array([_ground_to_bev(p, cfg) for p in ground], dtype=np.float32)
    return cv2.getPerspectiveTransform(src, dst)


def _ground_to_pixel(p_ground, K, R, C):
    """지면 위 점(x_forward, y_left, z=0) → 이미지 픽셀."""
    fx, fy, cx, cy = K
    p = np.array([p_ground[0], p_ground[1], 0.0])
    d_body = p - C                               # 카메라 → 지면점 (차체 좌표)
    d_opt = R.T @ d_body                         # 광학 좌표로
    if d_opt[2] <= 1e-6:                         # 카메라 뒤 → 투영 불가
        raise ValueError("지면점이 카메라 뒤에 있다 — look_near_m/마운트 확인")
    u = fx * d_opt[0] / d_opt[2] + cx
    v = fy * d_opt[1] / d_opt[2] + cy
    return (u, v)


def _ground_to_bev(p_ground, cfg: LaneConfig):
    """지면 위 점 → 버드아이 픽셀. 위=먼쪽, 왼쪽=차체 왼쪽."""
    x, y = p_ground
    col = (cfg.half_width_m - y) * cfg.px_per_m
    row = (cfg.look_far_m - x) * cfg.px_per_m
    return (col, row)


def bev_size(cfg: LaneConfig):
    w = int(round(2 * cfg.half_width_m * cfg.px_per_m))
    h = int(round((cfg.look_far_m - cfg.look_near_m) * cfg.px_per_m))
    return (w, h)


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


# ── 이진화 ───────────────────────────────────────────────────────────────


def binarize(bgr, cfg: LaneConfig):
    """흰 레인 마스크. 밝고(V 높음) 무채색(S 낮음)인 픽셀."""
    if cfg.blur > 1:
        bgr = cv2.GaussianBlur(bgr, (cfg.blur | 1, cfg.blur | 1), 0)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, (0, 0, cfg.v_min), (179, cfg.s_max, 255))


# ── 중심선 추출 ──────────────────────────────────────────────────────────


def lane_center(mask_bev, cfg: LaneConfig) -> LaneResult:
    """버드아이 마스크 → 횡오차·헤딩오차·곡률 (미터·라디안).

    가로띠마다 레인 픽셀의 무게중심을 구하고, (x=전방거리, y=횡위치) 점들에 **직선을
    맞춘다**. 기울기가 헤딩오차, lookahead 거리의 값이 횡오차다.
    ⚠️ 띠가 `min_bands` 미만이면 `ok=False` — **못 보면 조향하지 않는다.**
    """
    h, w = mask_bev.shape[:2]
    band_h = max(1, h // cfg.bands)
    pts = []
    rejected_wide = 0
    for i in range(cfg.bands):
        r0, r1 = i * band_h, min(h, (i + 1) * band_h)
        band = mask_bev[r0:r1]
        cols = np.nonzero(band)[1]
        if cols.size < cfg.min_px_per_band:
            continue

        # ★ 레인은 좁은 띠다. 넓게 퍼져 있으면 **벽·바닥이지 레인이 아니다.**
        #   이 검사가 없으면 흰 벽의 무게중심이 잡혀 '완벽히 중앙인 레인'이 되어버린다.
        span_m = (cols.max() - cols.min() + 1) / cfg.px_per_m
        fill = cols.size / float(band.size)
        if span_m > cfg.max_lane_width_m or fill > cfg.max_fill_ratio:
            rejected_wide += 1
            continue

        col = float(cols.mean())
        row = 0.5 * (r0 + r1)
        x = cfg.look_far_m - row / cfg.px_per_m          # 전방거리 [m]
        y = cfg.half_width_m - col / cfg.px_per_m        # 횡위치 [m] (+ = 왼쪽)
        pts.append((x, y))

    if len(pts) < cfg.min_bands:
        why = "너무 넓음(벽/바닥)" if rejected_wide else "레인 픽셀 부족"
        return LaneResult(False, n_bands=len(pts), detail=why)

    arr = np.array(pts)
    # y = m·x + b  (x=전방거리) — 기울기 m 이 곧 tan(헤딩오차)
    m, b = np.polyfit(arr[:, 0], arr[:, 1], 1)
    offset = m * cfg.lookahead_m + b
    heading = math.atan(m)

    curv = 0.0
    if len(pts) >= 3:                                    # 2차항이 있으면 곡률
        a2, a1, a0 = np.polyfit(arr[:, 0], arr[:, 1], 2)
        curv = 2.0 * a2 / (1.0 + a1 ** 2) ** 1.5

    return LaneResult(True, float(offset), float(heading), float(curv), len(pts))


# ── 추종 (PID) ───────────────────────────────────────────────────────────


class LaneFollower:
    """횡오차 → ω. 순수 PID (상태는 적분항뿐).

    ⚠️ **레인을 못 보면 조향하지 않는다** — 마지막 명령을 반복하지도 않는다.
       `update()` 가 `(v=0, omega=0, ok=False)` 를 돌려주고, 상위(미션 시퀀서)가
       어떻게 할지(정지·직진 유지·복구 탐색) 결정한다. 여기서 임의로 추측하지 않는다.
    """

    def __init__(self, cfg: LaneConfig = None):
        self.cfg = cfg or LaneConfig()
        self._i = 0.0
        self._prev = None

    def reset(self):
        self._i = 0.0
        self._prev = None

    def update(self, res: LaneResult, dt: float):
        """→ (v_mps, omega_rad_s, ok)"""
        c = self.cfg
        if not res.ok or dt <= 0.0:
            self.reset()
            return 0.0, 0.0, False

        err = res.offset_m                       # + = 레인이 왼쪽 → 왼쪽으로 틀어야
        self._i = _clamp(self._i + err * dt, -c.i_clamp, c.i_clamp)
        d = 0.0 if self._prev is None else (err - self._prev) / dt
        self._prev = err

        omega = c.kp * err + c.ki * self._i + c.kd * d
        return c.v_nominal, _clamp(omega, -c.omega_max, c.omega_max), True


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ── 파이프라인 한 번에 ───────────────────────────────────────────────────


def detect(bgr, K, cfg: LaneConfig, roll=0.0, pitch=0.0):
    """RGB 한 장 → LaneResult. (호모그래피는 매번 만들지 말고 캐시하는 게 좋다)"""
    M = ground_homography(K, cfg, roll, pitch)
    mask = binarize(bgr, cfg)
    bev = cv2.warpPerspective(mask, M, bev_size(cfg))
    return lane_center(bev, cfg)
