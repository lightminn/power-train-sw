"""벽 추종 — depth 로 벽과의 거리·각도를 재서 일정 간격을 유지한다 (WP7).

레인(흰 선)이 없는 구간 — 복도·터널·좁은 통로 — 에서 쓴다. 레인 추종의 **대체**지
보완이 아니다(둘 다 조향을 낸다). 상위(미션 시퀀서)가 어느 쪽을 쓸지 고른다.

하드웨어·ROS 의존 없음(numpy 만). ROS 래퍼는 `powertrain_ros/wall_follower_node.py`.

────────────────────────────────────────────────────────────────────────
★ 벽까지의 '거리'만 보면 안 된다 — **각도**도 봐야 한다
────────────────────────────────────────────────────────────────────────
초음파처럼 거리 하나만 재서 PID 를 걸면, 벽에 **비스듬히 다가가는 중**인지 **평행하게
가는 중**인지 구분을 못 한다. 그래서 벽에 부딪히거나 좌우로 계속 흔들린다(S자 주행).

depth 는 벽 위의 **여러 점**을 준다 → 직선을 맞추면 **거리와 각도를 동시에** 얻는다.
    거리 오차 = (목표 간격 − 실제 간격)
    각도 오차 = 벽의 방향과 차체 방향의 차이
    ω = kp·거리오차 + kh·각도오차          ← 각도항이 S자 진동을 잡는다

⚠️ **점군을 차체 좌표로 변환해서 넣어야 한다.** 광학 좌표를 그대로 넣으면 축이 뒤집힌다.
⚠️ 벽이 **직선이라는 가정**이다. 곡선 벽·모서리에서는 잔차가 커진다 → `residual` 로
   신뢰도를 판단하고, 크면 `ok=False`.
"""
from dataclasses import dataclass
import math

import numpy as np

LEFT = "left"
RIGHT = "right"


@dataclass
class WallConfig:
    side: str = RIGHT               # 어느 쪽 벽을 따라갈지
    target_m: float = 0.6           # 유지할 간격 [m]
    # 관심 영역 (차체 기준)
    x_min: float = 0.2              # 이 앞부터
    x_max: float = 2.5              # 여기까지의 벽을 본다
    y_max: float = 1.5              # 이보다 먼 옆은 벽이 아니다
    z_min: float = 0.10             # 바닥(z≈0)은 뺀다
    z_max: float = 1.00             # 천장·간판은 뺀다
    # 신뢰도
    min_points: int = 60
    max_residual_m: float = 0.08    # 직선 잔차가 이보다 크면 벽이 아니다(모서리·곡선)
    # 제어
    kp: float = 1.2                 # 거리 오차 → ω
    kh: float = 1.4                 # 각도 오차 → ω  (S자 진동을 잡는 항)
    omega_max: float = 1.0
    v_nominal: float = 0.5


@dataclass(frozen=True)
class WallResult:
    ok: bool
    distance_m: float = 0.0         # 벽까지의 수직 거리 (항상 양수)
    heading_rad: float = 0.0        # 벽 방향 − 차체 방향 (+ = 벽이 왼쪽으로 벌어짐)
    residual_m: float = 0.0         # 직선 맞춤 잔차 = 벽이 얼마나 직선인가
    n_points: int = 0
    detail: str = ""


def detect_wall(points_body, cfg: WallConfig) -> WallResult:
    """차체 좌표 점군 (N,3) → 벽까지의 거리·각도.

    points_body : base_link 기준 (x=앞, y=왼쪽, z=위). **광학 좌표를 그대로 넣지 말 것.**
    """
    p = np.asarray(points_body, dtype=np.float64)
    if p.ndim != 2 or p.shape[1] != 3:
        return WallResult(False, detail="점군 형식 오류")

    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    side_ok = (y > 0) if cfg.side == LEFT else (y < 0)
    m = (
        side_ok
        & (x > cfg.x_min) & (x < cfg.x_max)
        & (np.abs(y) < cfg.y_max)
        & (z > cfg.z_min) & (z < cfg.z_max)     # 바닥·천장 제거
    )
    if int(m.sum()) < cfg.min_points:
        return WallResult(False, n_points=int(m.sum()), detail="벽 점 부족")

    xs, ys = x[m], y[m]

    # 벽에 직선을 맞춘다: y = a·x + b   (x=전방거리)
    # ⚠️ 벽이 차체와 거의 나란하므로 x 로 회귀하는 게 안정적이다(y 로 하면 발산).
    a, b = np.polyfit(xs, ys, 1)
    resid = float(np.sqrt(np.mean((a * xs + b - ys) ** 2)))
    if resid > cfg.max_residual_m:
        # 모서리·곡선 벽·장애물이 섞였다 → 직선 모델이 안 맞는다
        return WallResult(False, residual_m=resid, n_points=int(m.sum()),
                          detail=f"벽이 직선이 아님 (잔차 {resid:.3f} m)")

    # 차체 원점에서 직선까지의 수직 거리 = |b| / sqrt(1 + a²)
    distance = abs(b) / math.sqrt(1.0 + a * a)
    heading = math.atan(a)                  # 벽의 기울기 = 각도 오차

    return WallResult(True, float(distance), float(heading), resid, int(m.sum()))


class WallFollower:
    """거리 + **각도** → ω. 각도항이 S자 진동을 잡는다.

    ⚠️ 벽을 못 보면 조향하지 않는다 — 마지막 명령을 반복하지도 않는다.
    """

    def __init__(self, cfg: WallConfig = None):
        self.cfg = cfg or WallConfig()

    def update(self, res: WallResult):
        """→ (v_mps, omega_rad_s, ok)"""
        c = self.cfg
        if not res.ok:
            return 0.0, 0.0, False

        # 거리 오차: 목표보다 **멀면** 벽 쪽으로 붙어야 한다
        d_err = res.distance_m - c.target_m
        # 오른쪽 벽을 따라갈 때: 멀면(+) 오른쪽으로 = ω 음수
        sign = -1.0 if c.side == RIGHT else +1.0

        omega = sign * c.kp * d_err + c.kh * res.heading_rad
        return c.v_nominal, _clamp(omega, -c.omega_max, c.omega_max), True


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))
