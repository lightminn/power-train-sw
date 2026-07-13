"""4WS 키네마틱스 관계 검증 — 숫자가 아니라 '성질'을 테스트한다
(직진=0°, 안쪽이 더 꺾임, 바깥이 더 빠름, 앞뒤 역위상, 한계 클램프, 피벗).
따라서 CAD 치수가 바뀌어도(default_geometry 숫자 교체) 테스트는 그대로 유효하다.
실행: motor_control/ 에서  `python -m pytest chassis/tests/ -q`
"""
import math
import pytest
from chassis.kinematics import (
    Wheel, ChassisGeometry, WheelCommand, SolveResult, solve, default_geometry,
)

TURN = dict(v_mps=0.4, omega_rad_s=0.4)   # 좌회전 공통 케이스 (한계 미도달)


def g():
    return default_geometry()


# ── 직진 / 정지 ──────────────────────────────────────────────────────────

def test_straight_zero_steer_equal_speed():
    r = solve(g(), v_mps=0.5, omega_rad_s=0.0)
    for wc in r.wheels.values():
        assert wc.steer_deg == pytest.approx(0.0, abs=1e-9)
        assert wc.drive_mps == pytest.approx(0.5)
    assert not r.steer_clamped and not r.speed_clamped


def test_zero_command_full_stop():
    r = solve(g(), 0.0, 0.0)
    for wc in r.wheels.values():
        assert wc.drive_mps == 0.0 and wc.steer_deg == 0.0


# ── 좌회전 Ackermann 성질 ────────────────────────────────────────────────

def test_left_turn_all_steer_left():
    r = solve(g(), **TURN)             # ω>0 = 좌회전 → 앞바퀴 좌(+)로
    assert r.wheels["front_left"].steer_deg > 0
    assert r.wheels["front_right"].steer_deg > 0


def test_inner_wheel_steers_more():
    r = solve(g(), **TURN)             # 좌회전 → 안쪽=좌측이 더 많이 꺾임
    assert abs(r.wheels["front_left"].steer_deg) > abs(r.wheels["front_right"].steer_deg)


def test_front_rear_opposite_phase():
    """4WS 협조: 뒤축은 앞축과 **반대 부호**로 꺾인다.

    ⚠️ 크기까지 같지는 않다. CAD 실측 기하는 **앞 윤거 705 mm ≠ 뒤 윤거 585 mm** 이므로
    같은 선회에서 요구되는 조향각이 앞뒤가 다르다(예: 앞 +34.1° / 뒤 −31.7°).
    구 기하(앞뒤 윤거 동일)에서는 정확한 거울상이었다.
    """
    r = solve(g(), **TURN)
    front = r.wheels["front_left"].steer_deg
    rear = r.wheels["rear_left"].steer_deg
    assert front > 0 and rear < 0                      # 반대 위상
    assert abs(rear) == pytest.approx(abs(front), rel=0.25)   # 크기는 비슷하되 다름


def test_outer_wheels_faster():
    r = solve(g(), **TURN)             # 좌회전 → 우측(바깥)이 더 빠름
    assert abs(r.wheels["front_right"].drive_mps) > abs(r.wheels["front_left"].drive_mps)
    assert abs(r.wheels["mid_right"].drive_mps) > abs(r.wheels["mid_left"].drive_mps)


def test_mid_wheels_fixed_zero_steer():
    r = solve(g(), **TURN)
    assert r.wheels["mid_left"].steer_deg == 0.0
    assert r.wheels["mid_right"].steer_deg == 0.0


def test_right_turn_mirrors_left():
    left = solve(g(), 0.4, 0.4)
    right = solve(g(), 0.4, -0.4)      # 우회전 = 좌회전의 좌우 대칭
    assert right.wheels["front_right"].steer_deg == pytest.approx(
        -left.wheels["front_left"].steer_deg, abs=1e-6)
    assert right.wheels["front_left"].steer_deg == pytest.approx(
        -left.wheels["front_right"].steer_deg, abs=1e-6)


# ── 한계 클램프 ──────────────────────────────────────────────────────────

def test_steer_limit_never_exceeded():
    r = solve(g(), v_mps=0.3, omega_rad_s=5.0)     # 과도한 급선회
    assert r.steer_clamped
    assert abs(r.omega_applied) < 5.0              # ω가 줄어듦
    for wc in r.wheels.values():
        assert abs(wc.steer_deg) <= 45.0 + 1e-6


def test_speed_limit_scales_uniformly():
    geo = g()
    r = solve(geo, v_mps=2.0, omega_rad_s=0.0)     # v_max 초과
    assert r.speed_clamped
    peak = max(abs(wc.drive_mps) for wc in r.wheels.values())
    assert peak == pytest.approx(geo.drive_limit_mps)


def test_turns_per_s_conversion():
    geo = g()
    wc = solve(geo, 0.5, 0.0).wheels["front_left"]
    assert wc.drive_turns_per_s == pytest.approx(0.5 / (2 * math.pi * geo.wheel_radius_m))


# ── 피벗(제자리 회전) ─────────────────────────────────────────────────────

def test_pivot_mid_wheels_counter_rotate():
    r = solve(g(), v_mps=0.0, omega_rad_s=0.5)     # 제자리 좌회전
    # 중앙축(x=0) 중간바퀴는 좌우가 정확히 반대속도로 굴러 회전 생성
    assert r.wheels["mid_left"].drive_mps == pytest.approx(
        -r.wheels["mid_right"].drive_mps, abs=1e-9)
    # 조향바퀴는 접선 방향으로 꺾임(0 아님)
    assert abs(r.wheels["front_left"].steer_deg) > 1e-3


def test_pivot_no_net_translation():
    # 피벗은 순수 회전 — 좌우 대칭이라 전진 구동성분 합이 0에 가까움
    r = solve(g(), v_mps=0.0, omega_rad_s=0.3)
    fwd = sum(wc.drive_mps * math.cos(math.radians(wc.steer_deg))
              for wc in r.wheels.values())
    assert fwd == pytest.approx(0.0, abs=1e-9)


# ── 커스텀 기하로도 동작 (숫자 무관 증명) ─────────────────────────────────

def test_arbitrary_geometry_symmetry():
    geo = ChassisGeometry(wheels=[
        Wheel("fl", 0.5, 0.4, True), Wheel("fr", 0.5, -0.4, True),
        Wheel("rl", -0.5, 0.4, True), Wheel("rr", -0.5, -0.4, True),
    ])
    r = solve(geo, 0.3, 0.3)
    # 좌우 대칭 기하 → 좌회전에서 fl/fr 조향 부호 같고 크기 다름
    assert r.wheels["fl"].steer_deg > 0 and r.wheels["fr"].steer_deg > 0
    assert abs(r.wheels["fl"].steer_deg) > abs(r.wheels["fr"].steer_deg)


# ── 🛠️ 4륜 구성 (중간 ODrive 보드를 부하모터에 쓸 때) ────────────────────

def test_four_wheel_geometry_has_only_steerables():
    """중륜 2개(고정륜)를 뺀다 — node 13/14 가 버스에 없으면 stale → 전체 estop 이 뜬다."""
    from chassis.kinematics import four_wheel_geometry
    g = four_wheel_geometry()
    assert len(g.wheels) == 4
    assert all(w.steerable for w in g.wheels)
    assert {w.name for w in g.wheels} == {
        "front_left", "front_right", "rear_left", "rear_right"}


def test_four_wheel_commands_match_six_wheel():
    """★ 남는 4바퀴의 명령은 6륜일 때와 **완전히 같아야** 한다.

    중륜은 고정륜이라 다른 바퀴의 조향·구동 명령에 영향을 주지 않는다. 달라진다면
    기하를 잘못 자른 것이다.
    """
    from chassis.kinematics import four_wheel_geometry
    g6, g4 = default_geometry(), four_wheel_geometry()
    for v, om in [(0.4, 0.0), (0.4, 0.4), (0.0, 0.5), (-0.3, -0.2)]:
        r6, r4 = solve(g6, v, om), solve(g4, v, om)
        for n in r4.wheels:
            assert r4.wheels[n].steer_deg == pytest.approx(
                r6.wheels[n].steer_deg, abs=1e-9), f"{n} 조향 불일치 (v={v}, ω={om})"
            assert r4.wheels[n].drive_mps == pytest.approx(
                r6.wheels[n].drive_mps, abs=1e-9), f"{n} 구동 불일치 (v={v}, ω={om})"


def test_four_wheel_map_matches_geometry():
    """★ 기하와 모터 매핑의 **바퀴 이름이 어긋나면 KeyError** 로 죽는다."""
    from chassis.chassis_manager import FOUR_WHEEL_MAP
    from chassis.kinematics import four_wheel_geometry
    assert {m.wheel for m in FOUR_WHEEL_MAP} == {
        w.name for w in four_wheel_geometry().wheels}
    assert {m.drive_node_id for m in FOUR_WHEEL_MAP} == {11, 12, 15, 16}   # 13/14 제외
