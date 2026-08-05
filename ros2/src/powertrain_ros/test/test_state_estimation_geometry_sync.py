"""스키드 주행 중 추정 노드가 애커만 기하로 남으면 추정이 오염된다.

⚠️ 초판은 "요레이트가 0 으로 눌린다"고 적었으나 2026-08-05 실측에서 틀린 것으로
확인됐다. 측면식은 경성 제약이 아니라 가중최소자승의 한 항이라, 종방향 식들과
타협한다. 실제 증상은 구성마다 다르다 (설계문서 §1.5 표):

  · 6륜 기본 설정 — ω 는 살아나지만(이상치 배제가 뒷바퀴 2개를 버리면서 모순되는
    측면식도 함께 빠진다) **유령 횡속도 −0.35 m/s** 와 **정상 바퀴 2개 오배제**가
    남아 슬립 감지와 잔차 신뢰도가 무력화된다.
  · 배제 여유가 없는 구성(4륜 등) — ω 가 **39~49 % 로 과소추정**된다.
"""
import pytest

from chassis.kinematics import (
    default_geometry, four_wheel_geometry, skid_geometry, solve,
)
from chassis.odometry import OdometryConfig, WheelObservation, solve_twist
from powertrain_ros.state_estimation import (
    StateEstimator, geometry_for_steering_mode,
)


def _skid_observations(v_mps, omega_rad_s, base=None):
    geom = skid_geometry(base=base)
    result = solve(geom, v_mps, omega_rad_s)
    return [
        WheelObservation(name=name, drive_mps=wc.drive_mps, steer_deg=0.0)
        for name, wc in result.wheels.items()
    ]


def test_ackermann_geometry_invents_lateral_velocity_for_skid_observations():
    """결함 B ① — 있지도 않은 횡속도가 생긴다 (스키드는 vy 를 명령하지 않는다)."""
    twist = solve_twist(default_geometry(), _skid_observations(0.0, 0.8))

    assert abs(twist.vy) > 0.1          # 실측 −0.35016
    assert solve_twist(skid_geometry(), _skid_observations(0.0, 0.8)).vy == \
        pytest.approx(0.0, abs=1e-9)


def test_ackermann_geometry_wrongly_rejects_healthy_wheels():
    """결함 B ② — 멀쩡한 바퀴가 슬립으로 배제되어 슬립 감지가 무력화된다."""
    ackermann = solve_twist(default_geometry(), _skid_observations(0.0, 0.8))
    skid = solve_twist(skid_geometry(), _skid_observations(0.0, 0.8))

    assert len(ackermann.rejected) == 2 and ackermann.used == 4
    assert skid.rejected == () and skid.used == 6


def test_ackermann_geometry_underestimates_yaw_without_rejection_headroom():
    """결함 B ③ — 배제가 못 구해주는 구성에서는 ω 자체가 과소추정된다.

    6륜 기본 설정에서 ω 가 살아나는 것은 이상치 배제가 뒷바퀴를 버려준 덕이다.
    배제를 끄면(또는 4륜처럼 여유가 없으면) 그 우연이 사라진다.
    """
    twist = solve_twist(
        default_geometry(),
        _skid_observations(0.0, 0.8),
        OdometryConfig(max_reject=0),
    )

    assert twist.omega < 0.8 * 0.6      # 실측 0.39093 = 49 %

    four = solve_twist(
        four_wheel_geometry(),
        _skid_observations(0.0, 0.8, base=four_wheel_geometry()),
    )
    assert four.omega < 0.8 * 0.6       # 실측 0.31577 = 39 %


def test_skid_geometry_recovers_yaw_for_the_same_observations():
    twist = solve_twist(skid_geometry(), _skid_observations(0.0, 0.8))

    assert twist.omega == pytest.approx(0.8, abs=1e-6)


def test_geometry_for_steering_mode_maps_both_modes():
    assert any(w.steerable for w in geometry_for_steering_mode("ackermann").wheels)
    assert not any(w.steerable for w in geometry_for_steering_mode("skid").wheels)


def test_geometry_for_steering_mode_applies_the_track_gain():
    wheels = {w.name: w for w in geometry_for_steering_mode("skid", 1.4).wheels}

    assert wheels["front_left"].y == pytest.approx(0.2725 * 1.4)


def test_geometry_for_steering_mode_falls_back_to_ackermann():
    """알 수 없는 값이 오면 조용히 스키드로 바꾸지 않는다."""
    assert any(w.steerable for w in geometry_for_steering_mode("crab").wheels)


def test_estimator_geometry_can_be_swapped():
    estimator = StateEstimator(default_geometry())

    estimator.set_geometry(skid_geometry())

    assert not any(w.steerable for w in estimator.geometry.wheels)
