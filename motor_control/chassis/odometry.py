"""4WS 오도메트리 — 바퀴 실측값을 차체 운동으로 되푸는 순기구학 (WP6).

`kinematics.py` 의 **역방향**이다.
    kinematics.solve()  : 차체 (v, ω)      → 바퀴별 (조향각, 속도)      [명령]
    odometry.solve_twist(): 바퀴별 (조향각, 속도) → 차체 (vx, vy, ω)   [추정]

순수 계산 모듈: 하드웨어·ROS·numpy 의존 없음(stdlib `math` 만). 기하는 `kinematics`
의 `ChassisGeometry` 를 그대로 재사용하므로 **코드는 실제 치수에 무관** — CAD/실측
확정 시 `default_geometry()` 숫자만 바꾸면 여기도 따라간다.

좌표계 : REP-103 (x=앞, y=왼쪽), 차체 중심 원점. 단위 = m·rad·s. ω>0 = 좌회전(CCW).

────────────────────────────────────────────────────────────────────────
모델 — 왜 최소자승인가
────────────────────────────────────────────────────────────────────────
차체가 평면 트위스트 (vx, vy, ω)로 움직이면 바퀴 i(위치 xᵢ,yᵢ)의 접지점 속도는
        (vx − ω·yᵢ,  vy + ω·xᵢ)                    # v_center + ω ẑ × rᵢ
바퀴는 자기가 향한 방향으로만 구르므로, 측정된 (조향각 δᵢ, 굴림속도 sᵢ)는

  · 조향 바퀴 → 접지점 속도벡터가 곧 sᵢ·(cos δᵢ, sin δᵢ) 이어야 함  → **식 2개**
        vx − ω·yᵢ = sᵢ·cos δᵢ
        vy + ω·xᵢ = sᵢ·sin δᵢ
  · 고정 바퀴 → 전진성분만 관측 가능(측면성분은 스크럽으로 미끄러짐) → **식 1개**
        vx − ω·yᵢ = sᵢ

조향4 × 2 + 고정2 × 1 = **식 10개**, 미지수 3개(vx, vy, ω) → 과결정(overdetermined).
이상적인 세계라면 10개가 한 점을 가리키지만, 실제로는 슬립·스크럽·HALL 노이즈로
제각각 다른 답을 낸다. 그래서 **"10개 식을 가장 적게 어기는 (vx,vy,ω)"** 을 고른다
= 가중 최소자승. 미지수가 3개뿐이라 정규방정식(3×3)을 직접 풀면 되고 numpy 불필요.

방어 2겹:
  1) **가중치** — 중간 고정륜은 스크럽 때문에 원래 덜 믿는다(`mid_weight`).
     저속 구간은 HALL 코깅존(<0.3 rev/s 실측)이라 속도 피드백이 튄다 → 비중 하향
     (`hall_trust_rev_s` 미만이면 `low_speed_weight` 배).
  2) **아웃라이어 제거** — 일단 푼 답으로 각 바퀴가 냈어야 할 속도를 되계산해,
     실측과 크게 어긋나는 바퀴는 '미끄러졌다'고 보고 빼고 다시 푼다(`slip_tol_mps`).

⚠️ **정확도 한계 (정직하게)**: 이 모듈이 보증하는 것은 *구조적 정합성*이지 *절대
정확도*가 아니다. `wheel_radius_m`(공칭 0.10 — 50 kg 하중에서 눌리면 실효 반경은
더 작다)과 바퀴 좌표는 아직 플레이스홀더다. 절대 정확도는 **차체 조립 후 지상
캘리브레이션**(직진 실측 → 실효 반경 / 피벗·원주행 → 윤거·축거 / UMBmark 양방향
사각형 → 계통오차·랜덤오차 분리)으로만 확정된다. 그때 `ChassisGeometry` 숫자만
교체하면 이 코드는 그대로 유효하다.
"""
from dataclasses import dataclass
import math

from chassis.kinematics import ChassisGeometry


# ── 데이터 구조 ──────────────────────────────────────────────────────────


@dataclass(frozen=True)
class WheelObservation:
    """바퀴 하나의 **실측값** (`/wheel_states` 의 WheelState 에 대응).

    drive_mps : 접지 선속도[m/s] (부호 = 굴림 방향). ODrive 는 rev/s 로 주므로
                `from_turns_per_s()` 로 변환해 넣는다.
    steer_deg : 조향각[deg]. 고정 바퀴는 0.
    valid     : False 면 방정식에서 아예 제외 (WheelState.drive_stale 등).
    """
    name: str
    drive_mps: float
    steer_deg: float = 0.0
    valid: bool = True

    @staticmethod
    def from_turns_per_s(name, turns_per_s, steer_deg=0.0, *, wheel_radius_m, valid=True):
        circ = 2.0 * math.pi * wheel_radius_m
        return WheelObservation(name, turns_per_s * circ, steer_deg, valid)


@dataclass
class OdometryConfig:
    """추정기 튜닝값. 전부 실측 근거가 있는 값으로만 기본값을 둔다."""
    mid_weight: float = 0.3          # 중간 고정륜 = 스크럽 발생 → 덜 믿음
    hall_trust_rev_s: float = 0.3    # 이 미만 = HALL 코깅존(실측) → 속도값 신뢰 하락
    low_speed_weight: float = 0.2    # 코깅존 바퀴에 곱할 가중치
    slip_tol_mps: float = 0.05       # 배제의 **절대 하한** — 이보다 작은 잔차는 노이즈
    mad_k: float = 3.0               # 배제의 **상대 기준** — 중앙값 + k·MAD 를 넘어야 이상치
    min_wheels: int = 3              # 이보다 적게 남으면 배제 중단 (해가 불안정)
    max_reject: int = 2              # 한 틱에 배제할 수 있는 최대 바퀴 수


@dataclass(frozen=True)
class TwistEstimate:
    """차체 운동 추정 결과."""
    vx: float                        # 전진속도[m/s]
    vy: float                        # 횡속도[m/s] — 우리 IK는 0만 명령. 0이 아니면 슬립/크랩 신호
    omega: float                     # 요레이트[rad/s], >0 = 좌회전
    rejected: tuple = ()             # 슬립으로 배제된 바퀴 이름
    residual_mps: float = 0.0        # 채택된 바퀴들의 RMS 잔차 — 추정 신뢰도 지표
    used: int = 0                    # 방정식에 실제로 쓰인 바퀴 수


# ── 선형대수 (3×3 이면 손으로 푸는 게 낫다) ───────────────────────────────


def _solve3(m, r):
    """3×3 선형계 m·x = r 를 부분 피벗 가우스 소거로 푼다. 특이하면 None."""
    a = [list(m[i]) + [r[i]] for i in range(3)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda i: abs(a[i][col]))
        if abs(a[piv][col]) < 1e-12:
            return None                       # 특이 — 관측이 부족해 자세를 못 정함
        a[col], a[piv] = a[piv], a[col]
        for i in range(col + 1, 3):
            f = a[i][col] / a[col][col]
            for j in range(col, 4):
                a[i][j] -= f * a[col][j]
    x = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        s = a[i][3] - sum(a[i][j] * x[j] for j in range(i + 1, 3))
        x[i] = s / a[i][i]
    return x


def _rows(geom, obs_map, cfg):
    """각 바퀴의 관측을 선형식 행(row)들로 전개.

    반환: list[(wheel_name, [a0,a1,a2], b, weight)] — a·(vx,vy,ω) = b
    """
    circ = 2.0 * math.pi * geom.wheel_radius_m
    rows = []
    for w in geom.wheels:
        o = obs_map.get(w.name)
        if o is None or not o.valid:
            continue

        weight = 1.0 if w.steerable else cfg.mid_weight          # 스크럽 보정
        if abs(o.drive_mps) / circ < cfg.hall_trust_rev_s:        # HALL 코깅존
            weight *= cfg.low_speed_weight

        if w.steerable:
            d = math.radians(o.steer_deg)
            # 접지점 속도벡터 = s·(cos δ, sin δ)
            rows.append((w.name, [1.0, 0.0, -w.y], o.drive_mps * math.cos(d), weight))
            rows.append((w.name, [0.0, 1.0,  w.x], o.drive_mps * math.sin(d), weight))
        else:
            # 고정륜은 전진성분만 관측 — 측면성분은 스크럽이라 방정식에 넣지 않는다
            rows.append((w.name, [1.0, 0.0, -w.y], o.drive_mps, weight))
    return rows


def _median(xs):
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _reject_threshold(res, cfg):
    """이 잔차를 넘는 바퀴만 '이상치'로 본다.

    ★ 핵심: 이상치는 **절대적으로 큰 것**이 아니라 **다른 바퀴들에 비해 유독 튀는 것**이다.
    절대 기준만 쓰면, 기하 오차·조향 클램프처럼 **모든 바퀴가 똑같이 어긋나는 계통 오차**를
    슬립으로 오인해 멀쩡한 바퀴를 버린다. 그러면 좌우 대칭이 깨지면서 **없던 속도가
    생겨난다**(피벗에서 존재하지 않는 전진속도 +0.09 m/s 가 실제로 관측됐다).

    그래서 중앙값 + k·MAD(중앙값 절대편차) 라는 **상대 기준**을 쓰고, 노이즈를 슬립으로
    보지 않도록 `slip_tol_mps` 를 **절대 하한**으로 깐다. 전부 비슷하게 어긋나 있으면
    MAD 가 작아도 중앙값이 높아져 아무도 배제되지 않는다 = 계통 오차는 그냥 감수하고
    잔차(신뢰도 지표)로만 보고한다. 한 바퀴만 튀면 중앙값이 낮게 유지되어 그놈만 걸린다.

    설계값과 실제 제작치수가 다를 수밖에 없으므로(공차·조립오차·하중 변형), 이 여유가
    없으면 기하가 조금만 틀려도 추정이 무너진다.
    """
    vals = list(res.values())
    med = _median(vals)
    mad = _median([abs(v - med) for v in vals])
    return max(cfg.slip_tol_mps, med + cfg.mad_k * mad)


def _weighted_lsq(rows):
    """가중 최소자승 → (vx, vy, ω). 정규방정식 Σw·aaᵀ·x = Σw·a·b."""
    m = [[0.0] * 3 for _ in range(3)]
    r = [0.0, 0.0, 0.0]
    for _, a, b, w in rows:
        for i in range(3):
            r[i] += w * a[i] * b
            for j in range(3):
                m[i][j] += w * a[i] * a[j]
    return _solve3(m, r)


# ── 메인 ─────────────────────────────────────────────────────────────────


def solve_twist(geom: ChassisGeometry, observations, cfg: OdometryConfig = None) -> TwistEstimate:
    """바퀴 실측값 → 차체 트위스트 (vx, vy, ω). 슬립 바퀴는 자동 배제.

    observations : Iterable[WheelObservation] (geom.wheels 이름과 매칭)
    """
    cfg = cfg or OdometryConfig()
    obs_map = {o.name: o for o in observations}
    rejected = []

    for _ in range(cfg.max_reject + 1):
        rows = _rows(geom, obs_map, cfg)
        names = {n for n, *_ in rows}
        if len(names) < cfg.min_wheels:
            break
        x = _weighted_lsq(rows)
        if x is None:
            break

        # 바퀴별 잔차 = "이 답이 맞다면 이 바퀴는 이렇게 굴렀어야 한다" 와 실측의 차이
        res = {}
        for n, a, b, _w in rows:
            e = sum(a[i] * x[i] for i in range(3)) - b
            res[n] = math.hypot(res.get(n, 0.0), e)

        worst = max(res, key=res.get)
        if len(rejected) < cfg.max_reject and len(names) - 1 >= cfg.min_wheels \
                and res[worst] > _reject_threshold(res, cfg):
            rejected.append(worst)                    # 슬립으로 판정 → 빼고 재계산
            obs_map.pop(worst, None)
            continue

        rms = math.sqrt(sum(v * v for v in res.values()) / len(res)) if res else 0.0
        return TwistEstimate(x[0], x[1], x[2], tuple(rejected), rms, len(names))

    return TwistEstimate(0.0, 0.0, 0.0, tuple(rejected), 0.0, 0)   # fail-safe: 정지로 본다


# ── 적분기 (트위스트 → 위치) ──────────────────────────────────────────────


class OdometryIntegrator:
    """트위스트를 시간 적분해 odom 프레임 pose (x, y, θ) 를 누적한다.

    ⚠️ 오도메트리는 **원리적으로 드리프트한다** — 미끄러진 만큼은 영영 모르고 오차가
    누적된다. 버그가 아니다. 짧은 구간의 상대 이동·시각화·제어 피드백용으로 쓰고,
    절대 위치로 신뢰하지 않는다(전역 보정은 우리 범위 밖 = `map→odom`).

    `yaw_rate` 를 주면 회전을 그 값으로 대체한다 → **"바퀴=거리, IMU=회전"** 원칙.
    (WP6 Step 3에서 L515 내장 IMU 자이로를 여기에 물린다.)
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta

    def update(self, twist: TwistEstimate, dt: float, yaw_rate: float = None):
        if dt <= 0.0:
            return self.pose()
        omega = twist.omega if yaw_rate is None else yaw_rate
        # 중점(midpoint) 적분 — dt 구간의 평균 방위로 변위를 적분해 원호 오차를 줄인다
        mid = self.theta + 0.5 * omega * dt
        c, s = math.cos(mid), math.sin(mid)
        self.x += (twist.vx * c - twist.vy * s) * dt
        self.y += (twist.vx * s + twist.vy * c) * dt
        self.theta = _wrap(self.theta + omega * dt)
        return self.pose()

    def pose(self):
        return (self.x, self.y, self.theta)

    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta


def _wrap(a):
    """각도를 (−π, π] 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))
