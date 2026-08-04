"""4WS 오도메트리 검증 — 역기구학과의 왕복(round-trip) + 슬립 배제 성질.

⚠️ **이 테스트가 증명하는 것과 못 하는 것**
  · 증명함 : `odometry` 가 `kinematics` 와 **같은 물리모델의 역함수**로 올바로 구현됐다
             (부호·인덱스·고정륜 처리·최소자승 구성의 버그 없음). 슬립 바퀴 배제 동작.
  · 못 함  : **절대 정확도**. 두 모듈이 같은 기하(플레이스홀더 치수, 공칭 바퀴반경)를
             쓰므로, 기하가 현실과 어긋나면 왕복은 여전히 통과한다. 절대 정확도는
             차체 조립 후 **지상 캘리브레이션**으로만 확정된다(`odometry.py` 상단 참조).
  두 모듈은 서로의 코드를 대수적으로 뒤집은 게 아니라 물리모델에서 **독립 유도**했다.
  그래서 왕복 일치는 자기충족이 아니라 교차검증이다.

실행: motor_control/ 에서  `python -m pytest chassis/tests/ -q`
"""
import math
import pytest

from chassis.kinematics import Wheel, ChassisGeometry, solve, default_geometry
from chassis.odometry import (
    WheelObservation, OdometryConfig, OdometryIntegrator, solve_twist,
)


def g():
    return default_geometry()


def observe(geom, result, slip=None):
    """역기구학 명령 결과를 '완벽히 추종한 바퀴 실측값'으로 변환.

    slip = {바퀴이름: 배율} — 그 바퀴가 헛도는(또는 끌리는) 상황을 주입한다.
    """
    slip = slip or {}
    return [
        WheelObservation(
            name=n,
            drive_mps=wc.drive_mps * slip.get(n, 1.0),
            steer_deg=wc.steer_deg,
        )
        for n, wc in result.wheels.items()
    ]


def roundtrip(geom, v, omega, **kw):
    """(v, ω) → 역기구학 → 실측값 → 순기구학 → 추정 트위스트."""
    r = solve(geom, v, omega)
    return r, solve_twist(geom, observe(geom, r, **kw))


# ── 왕복: 명령이 그대로 복원되는가 ────────────────────────────────────────

@pytest.mark.parametrize("v,omega", [
    (0.5, 0.0),      # 직진
    (-0.4, 0.0),     # 후진
    (0.4, 0.4),      # 좌선회
    (0.4, -0.4),     # 우선회
    (0.2, 0.15),     # 완만한 선회
])
def test_roundtrip_recovers_command(v, omega):
    """조향 한계 안쪽(feasible)에서는 명령이 **정확히** 복원돼야 한다."""
    geom = g()
    r, est = roundtrip(geom, v, omega)
    assert not r.steer_clamped and not r.speed_clamped   # feasible 케이스임을 못박음
    assert est.vx == pytest.approx(v, abs=1e-6)
    assert est.omega == pytest.approx(omega, abs=1e-6)
    assert est.vy == pytest.approx(0.0, abs=1e-6)     # 우리 IK는 횡속도를 명령하지 않는다
    assert est.rejected == ()
    assert est.residual_mps < 1e-6                    # 슬립 없으면 모든 식이 한 점을 가리킴


def test_zero_command_zero_twist():
    geom = g()
    _, est = roundtrip(geom, 0.0, 0.0)
    assert est.vx == pytest.approx(0.0, abs=1e-9)
    assert est.omega == pytest.approx(0.0, abs=1e-9)


# ── 성질: 부호와 방향 ─────────────────────────────────────────────────────

def test_left_turn_positive_omega():
    _, est = roundtrip(g(), 0.4, 0.4)
    assert est.omega > 0                      # ω>0 = 좌회전(CCW)


def test_right_turn_negative_omega():
    _, est = roundtrip(g(), 0.4, -0.4)
    assert est.omega < 0


def test_reverse_negative_vx():
    _, est = roundtrip(g(), -0.3, 0.0)
    assert est.vx < 0


def test_pivot_pure_rotation():
    """피벗 = 병진 0, 회전만. 단 **정확 복원이 아니라 최적합**이다.

    CAD 실측 기하에서 제자리 회전에 필요한 조향각은 **앞 |δ|=58.1° · 뒤 64.1°** 인데 AK
    한계는 ±45° → `solve()` 가 45°로 클램프하고 스크럽을 감수한다(설계된 동작). 그 결과
    6바퀴 실측이 **물리적으로 서로 모순**되고, 어떤 (vx,vy,ω)도 전부를 만족시킬 수 없다.
    최소자승은 '가장 덜 어기는 답'을 내놓는다 — 이것이 정상이며, 0 이 아닌 잔차가
    바로 "이 명령은 스크럽 중"이라는 신호다.

    ⚠️ **vy 가 정확히 0 이 아니다.** CAD 기하는 앞뒤가 비대칭(윤거 545 vs 425 mm, 중간
    바퀴가 60.3 mm 뒤로 치우침)이라 클램프된 피벗이 **미세한 횡방향 드리프트**를 만든다.
    좌우는 여전히 대칭이므로 vx 는 정확히 0 이다.

    ⚠️ **as-built v2 의 피벗 충실도는 실제로 더 나빠졌다.** 이전 넓은 기하는 필요
    조향각이 앞 51.2° / 뒤 56.2°라 추정 ω=0.494514 rad/s(명령 대비 −1.10%)였지만,
    더 좁은 v2는 필요각이 58.1° / 64.1°로 ±45° 클램프에서 더 멀어져
    **ω=0.480453 rad/s(−3.91%)**만 복원한다. 이는 테스트 오차가 아니라 증가한
    타이어 스크럽의 모델 결과다. 따라서 명령 대비 허용 한계는 측정 손실 다음의
    정수 공학 여유인 5%로 두되, 측정값 자체도 별도로 회귀 고정한다.

    ★ 회귀 방지: 예전 절대기준 배제 로직은 **좌측 두 바퀴만 골라 버려** 좌우 대칭을 깨고
    존재하지 않는 전진속도 +0.09 m/s 를 만들어냈다. 계통 오차는 배제 대상이 아니다.
    """
    r, est = roundtrip(g(), 0.0, 0.5)
    assert r.steer_clamped                     # 피벗은 조향 한계에 걸린다 (설계된 동작)
    assert est.rejected == ()                  # ★ 계통 오차 — 아무도 배제하면 안 된다
    assert est.vx == pytest.approx(0.0, abs=1e-9)   # ★ 유령 전진속도가 없어야 한다 (좌우 대칭)
    assert abs(est.vy) < 0.02                  # 앞뒤 비대칭 → 미세 횡드리프트 (0 은 아님)
    assert est.omega == pytest.approx(0.480453, abs=1e-5)  # v2 결정론적 모델값
    assert est.omega == pytest.approx(0.5, rel=0.05)  # 실제 3.91% 손실 + 공학 여유
    assert est.residual_mps > 0.0              # 모순의 크기 = 스크럽 지표


def test_systematic_error_is_not_slip():
    """모든 바퀴가 똑같이 어긋나면(계통 오차) 아무도 배제하지 않는다.

    설계값 ↔ 실제 제작치수 차이(공차·조립·하중 변형)는 전 바퀴에 고르게 나타난다.
    이걸 슬립으로 오인해 버리기 시작하면 대칭이 깨져 추정이 오히려 망가진다.
    """
    geom = g()
    # 전 바퀴 8% 과다 (예: 실효 바퀴반경이 공칭보다 작아 rev/s→m/s 환산이 어긋난 경우)
    _, est = roundtrip(geom, 0.4, 0.0, slip={w.name: 1.08 for w in geom.wheels})
    assert est.rejected == ()                  # 전원 유지
    assert est.vx == pytest.approx(0.4 * 1.08, rel=1e-6)   # 스케일 오차는 그대로 반영(정직)


# ── 슬립 배제 ────────────────────────────────────────────────────────────

def test_slipping_wheel_is_rejected():
    geom = g()
    # front_left 가 2배로 헛돎(구동륜 공회전) → 아웃라이어로 배제되어야 한다
    _, est = roundtrip(geom, 0.4, 0.0, slip={"front_left": 2.0})
    assert "front_left" in est.rejected
    assert est.vx == pytest.approx(0.4, abs=1e-6)     # 나머지 바퀴로 정답 복원


def test_rejection_keeps_estimate_sane():
    geom = g()
    # 슬립을 배제하지 못하면 vx 가 위로 끌려간다 — 배제가 그걸 막는지 확인
    _, est = roundtrip(geom, 0.4, 0.0, slip={"rear_right": 1.8})
    assert est.vx < 0.45


def test_no_rejection_when_within_tolerance():
    geom = g()
    # 3% 오차는 슬립이 아니라 노이즈 — 배제하지 말고 흡수해야 한다
    _, est = roundtrip(geom, 0.4, 0.0, slip={"front_left": 1.03})
    assert est.rejected == ()
    assert est.vx == pytest.approx(0.4, abs=0.02)


def test_invalid_wheel_excluded():
    geom = g()
    r = solve(geom, 0.4, 0.0)
    obs = observe(geom, r)
    obs = [WheelObservation(o.name, o.drive_mps, o.steer_deg,
                            valid=(o.name != "front_left")) for o in obs]
    est = solve_twist(geom, obs)
    assert est.used == 5                       # stale 바퀴 1개 제외
    assert est.vx == pytest.approx(0.4, abs=1e-6)


def test_all_wheels_invalid_is_failsafe_zero():
    geom = g()
    obs = [WheelObservation(w.name, 0.0, 0.0, valid=False) for w in geom.wheels]
    est = solve_twist(geom, obs)
    assert (est.vx, est.vy, est.omega) == (0.0, 0.0, 0.0)
    assert est.used == 0


# ── HALL 코깅존 가중치 ────────────────────────────────────────────────────

def test_low_speed_wheels_downweighted():
    geom = g()
    cfg = OdometryConfig()
    circ = 2.0 * math.pi * geom.wheel_radius_m
    slow = cfg.hall_trust_rev_s * circ * 0.5           # 코깅존 한복판
    # 전 바퀴가 코깅존이어도 해는 나와야 한다(가중치가 0이 아니라 낮을 뿐)
    obs = [WheelObservation(w.name, slow, 0.0) for w in geom.wheels]
    est = solve_twist(geom, obs, cfg)
    assert est.vx == pytest.approx(slow, abs=1e-6)


# ── 적분기 ───────────────────────────────────────────────────────────────

def test_integrator_straight_line():
    geom = g()
    odo = OdometryIntegrator()
    _, est = roundtrip(geom, 0.5, 0.0)
    for _ in range(100):                       # 0.02 s × 100 = 2 s → 1.0 m
        odo.update(est, 0.02)
    x, y, th = odo.pose()
    assert x == pytest.approx(1.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert th == pytest.approx(0.0, abs=1e-9)


def test_integrator_pivot_turns_in_place():
    """피벗 → 제자리 회전 (as-built v2 스크럽 손실만큼 못 미침).

    ⚠️ ω 를 키우면 바퀴 선속도가 `drive_limit_mps` 를 넘어 `solve()` 가 전체를 스케일
    다운한다. 그런데 `SolveResult.omega_applied` 는 그 스케일을 반영하지 않으므로
    (조향한계만 반영) 실제 회전은 보고값보다 느리다. 여기서는 상한에 걸리지 않는
    ω 를 써서 그 교란을 배제한다.

    이전 넓은 기하보다 좁은 v2는 피벗 필요 조향각이 앞 51.2°→58.1°,
    뒤 56.2°→64.1°로 커졌지만 AK 한계는 그대로 ±45°다. 따라서 클램프 후
    스크럽이 증가해 2 s × 0.5 rad/s 명령의 적분 결과가 **0.960907 rad
    (−3.91%)**로 실제 저하됐다. 명령 대비 5% 물리 계약과 이 측정값을 함께
    고정해, 더 느슨한 한계를 단순 테스트 우회로 오해하지 않게 한다.
    """
    geom = g()
    r = solve(geom, 0.0, 0.5)
    assert not r.speed_clamped                   # 속도 상한 미도달 구간임을 못박음
    odo = OdometryIntegrator()
    _, est = roundtrip(geom, 0.0, 0.5)
    for _ in range(100):                         # 2 s → 1.0 rad
        odo.update(est, 0.02)
    x, y, th = odo.pose()
    # 좌우 대칭이라 전진 병진은 0. 앞뒤 비대칭 때문에 횡방향으로 아주 조금 밀린다.
    assert math.hypot(x, y) < 0.03               # 2 초 피벗에 3 cm 이내
    assert th == pytest.approx(0.960907, abs=1e-5)  # v2 결정론적 2초 적분값
    assert th == pytest.approx(1.0, rel=0.05)    # 실제 3.91% 손실 + 공학 여유


def test_integrator_circle_returns_to_start():
    """일정 (v, ω)로 한 바퀴 = 원 → 시작점 복귀. 중점적분의 원호 정확도 확인."""
    geom = g()
    odo = OdometryIntegrator()
    v, omega = 0.4, 0.4                          # 반경 1 m
    _, est = roundtrip(geom, v, omega)
    dt = 0.001
    steps = int(round(2 * math.pi / omega / dt))
    for _ in range(steps):
        odo.update(est, dt)
    x, y, _ = odo.pose()
    assert math.hypot(x, y) < 0.01               # 1 cm 이내 복귀


def test_integrator_yaw_rate_override():
    """IMU yaw 를 주면 회전은 그쪽을 따른다 ('바퀴=거리, IMU=회전')."""
    geom = g()
    odo = OdometryIntegrator()
    _, est = roundtrip(geom, 0.4, 0.4)
    odo.update(est, 0.1, yaw_rate=0.0)           # IMU: 안 돌았다고 말함
    _, _, th = odo.pose()
    assert th == pytest.approx(0.0, abs=1e-9)    # 휠이 뭐라 하든 IMU 를 따름


# ── 기하 무관 (숫자 바뀌어도 유효) ────────────────────────────────────────

def test_arbitrary_geometry_roundtrip():
    geom = ChassisGeometry(wheels=[
        Wheel("fl", 0.5, 0.4, True), Wheel("fr", 0.5, -0.4, True),
        Wheel("rl", -0.5, 0.4, True), Wheel("rr", -0.5, -0.4, True),
    ], wheel_radius_m=0.08)
    _, est = roundtrip(geom, 0.3, 0.3)
    assert est.vx == pytest.approx(0.3, abs=1e-6)
    assert est.omega == pytest.approx(0.3, abs=1e-6)


# ── 🛠️ 4륜 구성 ──────────────────────────────────────────────────────────

def test_four_wheel_odometry_still_solves():
    """방정식이 10개 → 8개로 줄어도 (vx, vy, ω) 3개를 푸는 데는 충분하다."""
    from chassis.kinematics import four_wheel_geometry
    geom = four_wheel_geometry()
    for v, om in [(0.4, 0.0), (0.4, 0.4), (-0.3, 0.2)]:
        r = solve(geom, v, om)
        est = solve_twist(geom, observe(geom, r))
        assert est.vx == pytest.approx(v, abs=1e-6)
        assert est.omega == pytest.approx(om, abs=1e-6)


def test_four_wheel_pivot_degrades_slightly():
    """⚠️ 중륜은 **회전을 직접 관측하던 유일한 소스**였다(조향이 없어 순수 회전 지표).

    빼면 피벗 추정이 조금 나빠진다 — 그래도 실용 범위(수 %)다.
    """
    from chassis.kinematics import four_wheel_geometry
    geom = four_wheel_geometry()
    r = solve(geom, 0.0, 0.5)
    est = solve_twist(geom, observe(geom, r))
    assert est.omega == pytest.approx(0.5, rel=0.05)
    assert est.residual_mps > 0.0            # 조향 클램프 → 스크럽 (6륜과 동일)


from chassis.kinematics import skid_geometry, solve


def _observations_from_command(geom, v_mps, omega_rad_s):
    """명령을 그대로 실측이라고 가정한 관측 — 슬립 0 인 이상적 케이스."""
    result = solve(geom, v_mps, omega_rad_s)
    return [
        WheelObservation(name=name, drive_mps=wc.drive_mps, steer_deg=wc.steer_deg)
        for name, wc in result.wheels.items()
    ]


def test_skid_twist_is_recovered_from_wheel_speeds():
    """수정 전에는 정규방정식이 특이해 (0,0,0) fail-safe 가 나왔다 — 음성 대조."""
    geom = skid_geometry()
    observations = _observations_from_command(geom, 0.3, 0.5)

    twist = solve_twist(geom, observations)

    assert twist.vx == pytest.approx(0.3, abs=1e-6)
    assert twist.omega == pytest.approx(0.5, abs=1e-6)
    assert twist.vy == pytest.approx(0.0, abs=1e-6)
    assert twist.used == 6


def test_skid_pivot_twist_is_recovered():
    geom = skid_geometry()
    observations = _observations_from_command(geom, 0.0, -0.8)

    twist = solve_twist(geom, observations)

    assert twist.vx == pytest.approx(0.0, abs=1e-6)
    assert twist.omega == pytest.approx(-0.8, abs=1e-6)


def test_vy_prior_is_not_counted_as_a_wheel():
    """사전분포 행이 used·rejected·잔차 집계에 새면 신뢰도 지표가 오염된다."""
    geom = skid_geometry()
    observations = _observations_from_command(geom, 0.3, 0.5)

    twist = solve_twist(geom, observations)

    assert twist.used == 6
    assert twist.rejected == ()
    assert twist.residual_mps == pytest.approx(0.0, abs=1e-6)


def test_ackermann_rows_are_unchanged_by_the_prior():
    """조향륜이 하나라도 있으면 사전분포 행을 넣지 않는다 (4WS 회귀 0)."""
    from chassis.odometry import OdometryConfig, _rows

    geom = default_geometry()
    observations = _observations_from_command(geom, 0.3, 0.3)
    obs_map = {o.name: o for o in observations}

    rows = _rows(geom, obs_map, OdometryConfig())

    assert all(name is not None for name, *_ in rows)
    # 조향 4륜 × 2행 + 고정 2륜 × 1행
    assert len(rows) == 10


def test_no_observations_produces_no_lone_prior_row():
    from chassis.odometry import OdometryConfig, _rows

    geom = skid_geometry()

    assert _rows(geom, {}, OdometryConfig()) == []
