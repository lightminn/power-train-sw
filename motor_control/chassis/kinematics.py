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
    """6륜 로커보기 바퀴 배치 — **설계팀 CAD URDF 실제 제작 치수**.

    출처: `scripts/extract_geometry_from_cad_urdf.py` 가 설계팀 CAD 익스포트
    (`rover/urdf_2.urdf`, 2026-07-11)에서 도출한다. 숫자를 손으로 고치지 말고 그
    스크립트를 다시 돌려라. base_link = 축거중점 · 차체중심선 · 지면.

      축거(앞−뒤) 875.5 mm   |   윤거: 앞 705.0 / 중간 879.0 / 뒤 585.0 mm

    ⚠️ **윤거가 세 축 모두 다르다.** 앞 705 / 중간 879 / 뒤 585 mm — 중간이 가장 넓고
       뒤가 가장 좁다. CAD 상 실측이며 오독이 아니다(조향 4륜의 타이어 링크 원점이
       킹핀 축과 Δ0.0 mm 로 일치 = 스크럽 반경 0 설계 → 타이어 좌표 = 바퀴 중심면).
       **설계 의도인지 CAD 오류인지 설계팀 확인 필요.** 키네마틱스는 바퀴별 (x, y)를
       개별로 받으므로 값이 바뀌어도 코드는 그대로다.

    ⚠️ **중간 바퀴가 축거 중심에서 60.3 mm 뒤로 치우쳐 있다.** x≈0 이면 선회·피벗 시
       측면 스크럽이 0 인데(그게 중간 2륜에 조향모터를 안 다는 근거였다), 실제로는
       −60.3 mm 라 **스크럽이 남는다**. v4 최적화 설계값은 −11.4 mm 로 거의 중앙이었다.

    ⚠️ **제자리 피벗은 현 조향한계로 불가능**: 필요 |δ| = 90° − atan(|y| ÷ |x|) →
       **앞 51.2° · 뒤 56.2°** 로 AK 한계 ±45° 를 넘는다. `solve()` 가 45° 로 클램프하고
       스크럽을 감수한다(설계된 동작). 피벗 명령 시 바퀴들이 서로 모순된 값을 보고하므로
       오도메트리 잔차가 뜨고 ω 가 과소추정된다.

    참고 — **v4 최적화 설계값과 다르다**(제작 과정에서 바뀜):
       v4: 축거 1018 mm, 앞 +509.0 / 중간 −11.4 / 뒤 −509.0 mm (윤거는 v4 범위 밖)
       CAD 도출은 `parameter_calc/python_gpu_triangle/export_chassis_geometry.py`.
    """
    return ChassisGeometry(wheels=[
        # CAD URDF 실측 (scripts/extract_geometry_from_cad_urdf.py, 좌우 대칭화 적용)
        Wheel("front_left",  +0.4377, +0.3525, True),
        Wheel("front_right", +0.4377, -0.3525, True),
        Wheel("mid_left",    -0.0603, +0.4395, False),
        Wheel("mid_right",   -0.0603, -0.4395, False),
        Wheel("rear_left",   -0.4377, +0.2925, True),
        Wheel("rear_right",  -0.4377, -0.2925, True),
    ])


def four_wheel_geometry() -> ChassisGeometry:
    """🛠️ **중륜 2개를 뺀 4륜 구성** — 중간 ODrive 보드(node 13/14)를 부하모터(다이나모)에
    쓰고 있을 때의 임시 구성이다.

    ⚠️ **임시다.** 중륜 없이 정상 운용하는 설계가 아니다. 보드가 돌아오면 6륜으로 되돌린다.

    ⚠️ **바퀴는 여전히 땅에 닿아 있다.** 구동만 안 될 뿐 물리적으로는 붙어 있으므로,
       지상 주행 시 **끌려다니며 저항·스크럽**을 만든다(인휠 BLDC 라 코깅 드래그도 있다).
       → **바퀴를 띄운 벤치 테스트**에서 쓴다. 지상 주행은 별도 판단.

    ⚠️ **오도메트리 정확도가 떨어진다.** 방정식이 10개 → 8개로 줄고, 중륜은 조향이 없어
       **회전을 직접 관측하던 유일한 소스**였다(피벗에서 특히). 슬립 배제의 여유도 준다.

    기하 자체는 `default_geometry()` 와 같은 CAD 실측치를 쓴다 — 앞뒤 좌표·윤거 그대로.
    """
    full = default_geometry()
    return ChassisGeometry(
        wheels=[w for w in full.wheels if w.steerable],   # 조향 4륜만 (중륜은 고정륜)
        wheel_radius_m=full.wheel_radius_m,
        steer_limit_deg=full.steer_limit_deg,
        drive_limit_mps=full.drive_limit_mps,
    )
