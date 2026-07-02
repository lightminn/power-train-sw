"""4WS(4륜 조향) 키네마틱스 — 차체 명령을 각 바퀴 (조향각, 구동속도)로 변환.

순수 계산 모듈: 하드웨어·ROS 의존 없음(stdlib `math` 만). 기하(바퀴 위치·조향 가능
여부)는 `ChassisGeometry` 설정으로 주입하므로 **코드는 실제 치수에 무관** — CAD 확정 시
`default_geometry()` 의 숫자만 바꾸면 된다. `tests/test_kinematics.py` 가 숫자와 무관한
'관계'(직진=0°, 안쪽이 더 꺾임, 바깥이 더 빠름 …)로 검증한다.

좌표계 : REP-103 (x=앞, y=왼쪽), 차체 중심 원점. 단위 = m·rad·s. 로봇팔 팀 TF 규약과 동일.

입력 : (v, ω)  — 전진속도[m/s] + 요레이트[rad/s] (ω>0 = 좌회전/CCW).
       ROS `geometry_msgs/Twist` 의 (linear.x, angular.z) 와 그대로 매칭.
       회전반경 R = v/ω, 곡률 κ = ω/v. (v, κ) 대신 (v, ω)를 쓰는 이유 = **피벗(제자리
       회전)이 v=0·ω≠0 으로 자연히 표현**됨 — κ는 v=0에서 발산.

모델 : 차체가 (v 전진 + ω 요회전)하면 바퀴 i(위치 xᵢ,yᵢ)의 접지점 속도는
           (vx, vy) = (v − ω·yᵢ,  ω·xᵢ)          # v_center + ω ẑ × rᵢ
       - 조향 바퀴 : 이 벡터 방향으로 바퀴를 틀고(조향각 δ=atan2(vy,vx)), 크기만큼 굴린다.
       - 고정 바퀴(중간) : 틀 수 없으니 전진성분(vx=v−ω·yᵢ)만 굴리고 측면성분(vy=ω·xᵢ)은
         스크럽(미끄럼). **중간 바퀴를 x≈0(차체 중심축)에 두면 vy=ω·xᵢ≈0 → 스크럽 0**
         → 이것이 6륜 로커보기에서 중간 2바퀴에 조향모터를 안 다는 설계 근거.
"""
from dataclasses import dataclass, field
import math

# ── 데이터 구조 ──────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Wheel:
    """바퀴 하나의 기하. x=앞(+)/y=왼(+) [m], steerable=조향 가능 여부."""
    name: str
    x: float
    y: float
    steerable: bool


@dataclass
class ChassisGeometry:
    """차체 기하 + 한계. CAD 확정 시 이 값만 교체하면 kinematics 전체가 따라감."""
    wheels: list                       # list[Wheel]
    wheel_radius_m: float = 0.10       # BL70200 인휠 R_w=100mm
    steer_limit_deg: float = 45.0      # AK 조향 출력축 ±한계 (corner config 와 동일)
    drive_limit_mps: float = 0.80      # 바퀴 선속도 상한 (v4 최적화 v_max=0.80 m/s)


@dataclass(frozen=True)
class WheelCommand:
    """바퀴 하나에 내릴 명령. 고정 바퀴는 steer_deg=0."""
    name: str
    steer_deg: float
    drive_mps: float                   # 접지 선속도[m/s] (부호 = 굴림 방향)
    drive_turns_per_s: float           # ODrive 입력용 (=drive_mps / 원주)


@dataclass(frozen=True)
class SolveResult:
    wheels: dict                       # dict[str, WheelCommand]
    omega_applied: float               # 실제 적용된 ω (조향한계로 축소됐을 수 있음)
    steer_clamped: bool                # 조향 한계 초과 → 곡률/조향 제한됨
    speed_clamped: bool                # 속도 한계 초과 → 전체 감속됨


# ── 내부 유틸 ────────────────────────────────────────────────────────────


def _normalize(delta: float, speed: float):
    """조향각을 [-π/2, π/2] 로 정규화. 넘으면 180° 뒤집고 속도 부호를 반전
    (바퀴는 뒤로도 구를 수 있으므로 δ와 δ±180°+역속도는 등가)."""
    if delta > math.pi / 2:
        return delta - math.pi, -speed
    if delta < -math.pi / 2:
        return delta + math.pi, -speed
    return delta, speed


def _peak_steer(geom: ChassisGeometry, v: float, omega: float) -> float:
    """주어진 (v, ω)에서 조향 바퀴들의 최대 |조향각| [rad]."""
    peak = 0.0
    for w in geom.wheels:
        if not w.steerable:
            continue
        delta, _ = _normalize(math.atan2(omega * w.x, v - omega * w.y), 1.0)
        peak = max(peak, abs(delta))
    return peak


def _limit_omega(geom: ChassisGeometry, v: float, omega: float, steer_max: float):
    """조향 한계를 넘으면 ω를 줄여(=선회를 완만하게) 최대 조향각이 딱 한계가 되게 한다.
    반환 (적용ω, 클램프여부). 피벗(v≈0)은 조향각이 ω 크기와 무관 → ω로 못 풀고
    solve() 에서 바퀴별 개별 클램프(스크럽 감수)."""
    if omega == 0.0 or _peak_steer(geom, v, omega) <= steer_max + 1e-9:
        return omega, False
    if abs(v) < 1e-6:
        return omega, True             # 순수 피벗 — solve()가 per-wheel 클램프
    # feasible 영역(조향각<90°)에서 |ω|↑ → 최대조향각↑ 단조 → 이분탐색으로 경계 찾기
    lo, hi = 0.0, abs(omega)
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _peak_steer(geom, v, math.copysign(mid, omega)) <= steer_max:
            lo = mid
        else:
            hi = mid
    return math.copysign(lo, omega), True


# ── 메인 ─────────────────────────────────────────────────────────────────


def solve(geom: ChassisGeometry, v_mps: float, omega_rad_s: float) -> SolveResult:
    """차체 명령 (v, ω) → 바퀴별 명령. 조향/속도 한계를 초과하면 자동 제한."""
    steer_max = math.radians(geom.steer_limit_deg)

    # 1) 조향 한계로 ω 축소 (선회 완만화)
    omega, steer_clamped = _limit_omega(geom, v_mps, omega_rad_s, steer_max)

    # 2) 바퀴별 (조향각, 선속도)
    raw = []                           # (Wheel, delta_rad, speed_mps)
    for w in geom.wheels:
        vx = v_mps - omega * w.y
        vy = omega * w.x
        if w.steerable:
            delta, speed = _normalize(math.atan2(vy, vx), math.hypot(vx, vy))
            if abs(delta) > steer_max:              # 피벗 등 잔여 초과 → 개별 클램프
                delta = math.copysign(steer_max, delta)
                steer_clamped = True
        else:
            delta, speed = 0.0, vx                  # 고정 바퀴 = 전진성분만
        raw.append((w, delta, speed))

    # 3) 속도 한계로 전체 스케일 (경로 형상 유지, 느리게만)
    peak = max((abs(s) for _, _, s in raw), default=0.0)
    scale = geom.drive_limit_mps / peak if peak > geom.drive_limit_mps else 1.0
    speed_clamped = scale < 1.0

    circ = 2.0 * math.pi * geom.wheel_radius_m
    wheels = {}
    for w, delta, speed in raw:
        speed *= scale
        wheels[w.name] = WheelCommand(
            name=w.name,
            steer_deg=math.degrees(delta),
            drive_mps=speed,
            drive_turns_per_s=speed / circ,
        )
    return SolveResult(wheels, omega, steer_clamped, speed_clamped)


# ── 기본 기하 (⚠️ 잠정 플레이스홀더) ─────────────────────────────────────


def default_geometry() -> ChassisGeometry:
    """⚠️ 잠정 플레이스홀더 — CAD 확정·실물 실측 후 숫자 교체할 것.

    6륜 로커보기: 앞·뒤 4바퀴 조향(AK), 중간 2바퀴 고정('M2 조향 브라켓 없음' 설계).
      - 윤거: v4 최적화 하부폭 W_bot≈520mm → 좌우 y=±0.26 m  [잠정]
      - 축거: 미확정 → 앞 x=+0.30 / 중간 x=0 / 뒤 x=−0.30 m 로 가정(축거 600mm)  [잠정]
      - 중간 바퀴 x=0 → 선회·피벗 시 측면 스크럽 0 (조향 불필요 근거).
    실제 로커보기 바퀴 위치는 parameter_calc `wpos.py` + 최적 pkl 로 계산 예정.
    """
    y = 0.26
    xf, xr = 0.30, -0.30
    return ChassisGeometry(wheels=[
        Wheel("front_left",  xf,  y, True),
        Wheel("front_right", xf, -y, True),
        Wheel("mid_left",   0.0,  y, False),
        Wheel("mid_right",  0.0, -y, False),
        Wheel("rear_left",  xr,   y, True),
        Wheel("rear_right", xr,  -y, True),
    ])
