"""앞 로봇 추종 (WP9) — 거리 제어 + **추돌 방지**.

★ 단순 PID 로 목표 거리를 맞추면, 앞 로봇이 급정거할 때 우리는 관성으로 밀고 들어간다.
"""
import math

import pytest

from chassis.follow import FollowConfig, LeadFollower


def _f(**kw):
    return LeadFollower(FollowConfig(**kw))


def _det(dist, lat=0.0, conf=0.9, name="robot", area=1200.0):
    return [(name, conf, dist, lat, area)]


def _settle(f, dist, t0=0.0, n=6, dt=0.1, lat=0.0):
    """평활이 수렴하도록 같은 거리를 몇 번 먹인다."""
    r = None
    for i in range(n):
        r = f.update(_det(dist, lat), t0 + i * dt)
    return r, t0 + (n - 1) * dt


# ── 거리 제어 ────────────────────────────────────────────────────────────

def test_wp7_default_spacing_contract():
    c = FollowConfig()
    assert c.target_m == 2.0
    assert c.min_m == 1.5
    assert c.band_m == (1.5, 2.5)
    assert c.max_m == 6.0
    assert c.predict_decay == 0.5
    assert c.predict_limit_s == 1.0
    assert c.reacquire_pos_m == 1.0
    assert c.reacquire_size_ratio == (0.5, 2.0)
    assert c.reacquire_max_gap_s == 3.0
    assert c.reacquire_confirm_n == 2


def test_far_speeds_up():
    f = _f(target_m=1.5)
    r, _ = _settle(f, 3.0)
    assert r.ok and r.v > 0.3


def test_at_target_holds():
    f = _f(target_m=1.5, kd=0.0)
    r, _ = _settle(f, 1.5)
    assert r.ok
    assert r.v == pytest.approx(0.0, abs=0.05)


def test_spacing_band_uses_weak_gain():
    f = _f(kd=0.0, kp=1.0, band_gain=0.2)
    inside, _ = _settle(f, 2.4)
    outside, _ = _settle(_f(kd=0.0, kp=1.0, band_gain=0.2), 2.6)
    assert inside.ok and outside.ok
    assert inside.v == pytest.approx(0.08)
    assert outside.v == pytest.approx(0.6)


def test_fake_stationary_target_stream_converges_to_2_m():
    f = _f(ema=1.0, kd=0.0)
    distance = 4.0
    for i in range(500):
        r = f.update(_det(distance), i * 0.1)
        distance -= r.v * 0.1
    assert distance == pytest.approx(2.0, abs=0.01)
    assert r.state == "TRACKING"


def test_no_reverse():
    """앞 로봇이 뒤로 오면? **후진하지 않는다** — 뒤가 안 보인다."""
    f = _f(target_m=1.5, min_m=0.3)
    r, _ = _settle(f, 0.9)
    assert r.v >= 0.0


# ── ★ 추돌 방지 ─────────────────────────────────────────────────────────

def test_min_distance_forces_stop():
    """★ 최소 거리 안 = **무조건 정지.** PID 를 믿지 않는다."""
    f = _f(target_m=1.5, min_m=0.8)
    r, _ = _settle(f, 0.5)
    assert r.ok
    assert r.v == 0.0
    assert "최소 거리" in r.reason


def test_default_minimum_distance_is_1_5_m():
    f = _f(ema=1.0)
    f.update(_det(2.0), 0.0)
    r = f.update(_det(1.5), 0.1)
    assert r.ok
    assert r.v == 0.0
    assert "최소 거리" in r.reason


def test_single_raw_detection_inside_minimum_stops_before_ema_catches_up():
    f = _f(ema=0.4, kd=0.0)
    f.update(_det(3.0), 0.0)
    r = f.update(_det(1.0), 0.1)
    assert r.distance_m > f.cfg.min_m             # EMA는 아직 멀다고 보더라도
    assert r.ok and r.v == 0.0                    # raw hard-stop이 먼저 이겨야 한다
    assert "최소 거리" in r.reason


def test_closing_fast_slows_down():
    """★★ 앞 로봇이 급정거 → 우리가 빠르게 다가간다 → **미리 감속**해야 한다.

    거리항만 보면 아직 목표(1.5 m)보다 멀어서(2.0 m) **가속**한다 — 그대로 들이받는다.
    접근 속도(kd) 항이 그걸 막는다.
    """
    approaching = [(3.0, 0.0), (2.6, 0.1), (2.2, 0.2), (2.0, 0.3)]   # 빠르게 접근

    no_damping = _f(target_m=1.5, kd=0.0, band_m=(1.0, 1.8))
    v_no = None
    for d, t in approaching:
        v_no = no_damping.update(_det(d), t).v

    with_damping = _f(target_m=1.5, kd=1.2, band_m=(1.0, 1.8))
    v_with = None
    for d, t in approaching:
        v_with = with_damping.update(_det(d), t).v

    assert v_no > 0.2                    # 거리항만: 아직 멀다고 가속
    assert v_with < v_no                 # ★ 접근 속도항이 감속시킨다


def test_closing_rate_is_reported():
    f = _f()
    f.update(_det(3.0), 0.0)
    r = f.update(_det(2.0), 0.5)
    assert r.closing_mps > 0             # + = 가까워지는 중


# ── 조향 ─────────────────────────────────────────────────────────────────

def test_lateral_offset_steers():
    f = _f()
    left, _ = _settle(f, 2.0, lat=+0.5)      # base_link +y = 왼쪽
    assert left.omega > 0                     # 왼쪽으로 튼다

    f2 = _f()
    right, _ = _settle(f2, 2.0, lat=-0.5)
    assert right.omega < 0


def test_omega_clamped():
    f = _f(k_yaw=100.0, omega_max=1.0)
    r, _ = _settle(f, 2.0, lat=1.0)
    assert abs(r.omega) <= 1.0 + 1e-9


# ── 가림 예측·놓침 ───────────────────────────────────────────────────────

def test_brief_loss_predicts_with_tick_decay():
    """한 프레임 가림은 마지막 명령을 그대로 반복하지 않고 매 tick 감속한다."""
    f = _f(lost_grace_s=0.5, predict_decay=0.5, kd=0.0)
    tracked, last_t = _settle(f, 3.0, n=5)

    first = f.update([], last_t + 0.05)
    second = f.update([], last_t + 0.10)

    assert first.ok and second.ok
    assert first.state == second.state == "PREDICTING"
    assert first.v == pytest.approx(tracked.v * 0.5)
    assert second.v == pytest.approx(first.v * 0.5)
    assert "예측" in first.reason


def test_occlusion_extrapolates_distance_from_closing_rate():
    f = _f(ema=1.0, kd=0.0, lost_grace_s=0.5)
    f.update(_det(3.0), 0.0)
    tracked = f.update(_det(2.8), 0.1)
    predicted = f.update([], 0.2)
    assert tracked.closing_mps == pytest.approx(2.0)
    assert predicted.distance_m == pytest.approx(2.6)
    assert predicted.closing_mps == pytest.approx(2.0)


def test_prediction_limit_publishes_one_stop_then_gives_up():
    """예측 한계 tick은 0을 발행하고, 그 뒤에는 ok=False로 명령을 만들지 않는다."""
    f = _f(lost_grace_s=2.0, predict_limit_s=0.25, kd=0.0)
    _, last_t = _settle(f, 3.0, n=5)
    predicting = f.update([], last_t + 0.10)
    stopped = f.update([], last_t + 0.30)
    lost = f.update([], last_t + 0.40)

    assert predicting.ok and predicting.v > 0.0
    assert stopped.ok and (stopped.v, stopped.omega) == (0.0, 0.0)
    assert stopped.state == "LOST"
    assert "예측 한계" in stopped.reason
    assert not lost.ok and (lost.v, lost.omega) == (0.0, 0.0)
    assert lost.state == "LOST"
    assert "놓침" in lost.reason


# ── 재검출 연속성 gate ──────────────────────────────────────────────────

def test_reacquire_requires_two_frames_then_reinitializes_at_zero():
    f = _f(kd=0.0, lost_grace_s=1.0, reacquire_confirm_n=2)
    _, last_t = _settle(f, 3.0, n=5)
    f.update([], last_t + 0.1)

    first = f.update(_det(3.0), last_t + 0.2)
    accepted = f.update(_det(3.0), last_t + 0.3)
    tracking = f.update(_det(3.0), last_t + 0.4)

    assert not first.ok and first.v == 0.0
    assert first.state == "REACQUIRING"
    assert "1/2" in first.reason
    assert not accepted.ok and accepted.v == 0.0
    assert accepted.state == "REACQUIRING"
    assert "수락" in accepted.reason
    assert tracking.ok and tracking.state == "TRACKING"


def test_detection_stream_gap_also_requires_reacquire_confirmation():
    f = _f(kd=0.0, lost_grace_s=0.5, reacquire_confirm_n=2)
    _, last_t = _settle(f, 3.0, n=5)

    first = f.update(_det(3.0), last_t + 0.6)
    accepted = f.update(_det(3.0), last_t + 0.7)
    tracking = f.update(_det(3.0), last_t + 0.8)

    assert not first.ok and "1/2" in first.reason
    assert not accepted.ok and "수락" in accepted.reason
    assert tracking.ok and tracking.state == "TRACKING"


def test_reacquire_rejects_position_jump_and_resets_confirmation():
    f = _f(kd=0.0, lost_grace_s=1.0, reacquire_pos_m=1.0)
    _, last_t = _settle(f, 3.0, n=5)
    f.update([], last_t + 0.1)

    first = f.update(_det(3.1), last_t + 0.2)
    jump = f.update(_det(4.2), last_t + 0.3)
    first_again = f.update(_det(3.1), last_t + 0.4)

    assert not first.ok and "1/2" in first.reason
    assert not jump.ok and "위치" in jump.reason
    assert not first_again.ok and "1/2" in first_again.reason


def test_reacquire_alternating_candidate_identities_never_confirm():
    follower = _f(
        kd=0.0,
        lost_grace_s=1.0,
        reacquire_pos_m=0.5,
        reacquire_confirm_n=2,
    )
    _, last_t = _settle(follower, 3.0, n=5)
    follower.update([], last_t + 0.1)

    results = [
        follower.update(_det(3.0, lat=lat), last_t + stamp_offset)
        for lat, stamp_offset in (
            (-0.4, 0.2),
            (0.4, 0.3),
            (-0.4, 0.4),
            (0.4, 0.5),
        )
    ]

    assert all(not result.ok for result in results)
    assert all(result.state == "REACQUIRING" for result in results)
    assert all("1/2" in result.reason for result in results)


@pytest.mark.parametrize("area", [500.0, 2500.0])
def test_reacquire_rejects_bbox_size_jump(area):
    f = _f(kd=0.0, lost_grace_s=1.0)
    _, last_t = _settle(f, 3.0, n=5)
    f.update([], last_t + 0.1)
    r = f.update(_det(3.0, area=area), last_t + 0.2)
    assert not r.ok and r.v == 0.0
    assert r.state == "REACQUIRING"
    assert "크기" in r.reason


def test_reacquire_ignores_higher_confidence_wrong_class():
    f = _f(kd=0.0, lost_grace_s=1.0)
    _, last_t = _settle(f, 3.0, n=5)
    f.update([], last_t + 0.1)
    detections = [
        ("사람", 0.99, 2.9, 0.0, 1200.0),
        ("robot", 0.80, 3.0, 0.0, 1200.0),
    ]
    first = f.update(detections, last_t + 0.2)
    accepted = f.update(detections, last_t + 0.3)
    assert "1/2" in first.reason
    assert "수락" in accepted.reason
    assert accepted.distance_m == pytest.approx(3.0)


def test_reacquire_rejects_wrong_class_at_zero_speed():
    f = _f(kd=0.0, lost_grace_s=1.0)
    _, last_t = _settle(f, 3.0, n=5)
    f.update([], last_t + 0.1)
    r = f.update(_det(3.0, name="사람"), last_t + 0.2)
    assert not r.ok and r.v == 0.0
    assert r.state == "REACQUIRING"
    assert "class" in r.reason


def test_reacquire_after_max_gap_is_a_fresh_zero_speed_acquisition():
    f = _f(kd=0.0, lost_grace_s=1.0, reacquire_max_gap_s=3.0)
    _, last_t = _settle(f, 3.0, n=5)
    f.update([], last_t + 0.1)

    fresh = f.update(_det(4.8, area=400.0), last_t + 3.1)
    tracking = f.update(_det(4.8, area=400.0), last_t + 3.2)

    assert not fresh.ok and fresh.v == 0.0
    assert fresh.state == "REACQUIRING"
    assert "초기화" in fresh.reason
    assert tracking.ok and tracking.state == "TRACKING"


def test_fake_target_stream_is_deterministic():
    stream = [
        (_det(3.0), 0.0),
        (_det(2.9), 0.1),
        ([], 0.2),
        ([], 0.3),
        (_det(2.7), 0.4),
        (_det(2.7), 0.5),
        (_det(2.7), 0.6),
    ]

    def run():
        f = _f(ema=1.0, kd=0.0, lost_grace_s=1.0)
        return [f.update(detections, t) for detections, t in stream]

    assert run() == run()


def test_too_far_is_not_tracked():
    f = _f(max_m=6.0)
    r = f.update(_det(9.0), 0.0)
    assert not r.ok


def test_low_confidence_ignored():
    f = _f(min_confidence=0.5)
    r = f.update(_det(2.0, conf=0.2), 0.0)
    assert not r.ok


@pytest.mark.parametrize("field", ["conf", "dist", "lat", "area"])
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_nonfinite_detection_fields_never_create_a_target(field, value):
    values = {
        "name": "robot",
        "conf": 0.9,
        "dist": 3.0,
        "lat": 0.2,
        "area": 1200.0,
    }
    values[field] = value
    detection = [(
        values["name"],
        values["conf"],
        values["dist"],
        values["lat"],
        values["area"],
    )]

    result = _f().update(detection, 0.0)

    assert result.ok is False
    assert result.state == "LOST"
    assert (result.v, result.omega) == (0.0, 0.0)


@pytest.mark.parametrize("field", ["conf", "dist", "lat", "area"])
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_nonfinite_detection_does_not_contaminate_existing_target(field, value):
    follower = _f(ema=1.0, kd=0.0)
    follower.update(_det(3.0, lat=0.2, area=1200.0), 0.0)
    tracked = follower.update(_det(3.0, lat=0.2, area=1200.0), 0.1)
    target_state = (
        follower._d,
        follower._t,
        follower._t_seen,
        follower._lat,
        follower._area,
    )
    values = {
        "name": "robot",
        "conf": 0.9,
        "dist": 3.0,
        "lat": 0.2,
        "area": 1200.0,
    }
    values[field] = value

    rejected = follower.update([(
        values["name"],
        values["conf"],
        values["dist"],
        values["lat"],
        values["area"],
    )], 0.2)

    assert rejected.state != "TRACKING"
    assert (follower._d, follower._t, follower._t_seen,
            follower._lat, follower._area) == target_state
    assert all(math.isfinite(number) for number in (
        tracked.v,
        tracked.omega,
        follower._d,
        follower._lat,
        follower._area,
    ))


def test_wrong_class_ignored():
    f = _f(class_name="robot")
    r = f.update(_det(2.0, name="사람"), 0.0)
    assert not r.ok


def test_nearest_robot_wins():
    f = _f()
    for i in range(6):
        r = f.update([
            ("robot", 0.9, 3.0, 0.0, 800.0),
            ("robot", 0.9, 1.8, 0.0, 1200.0),
        ], i * 0.1)
    assert r.distance_m == pytest.approx(1.8, abs=0.2)
