"""앞 로봇 추종 (WP9) — 거리 제어 + **추돌 방지**.

★ 단순 PID 로 목표 거리를 맞추면, 앞 로봇이 급정거할 때 우리는 관성으로 밀고 들어간다.
"""
import pytest

from chassis.follow import FollowConfig, LeadFollower


def _f(**kw):
    return LeadFollower(FollowConfig(**kw))


def _det(dist, lat=0.0, conf=0.9, name="robot"):
    return [(name, conf, dist, lat)]


def _settle(f, dist, t0=0.0, n=6, dt=0.1, lat=0.0):
    """평활이 수렴하도록 같은 거리를 몇 번 먹인다."""
    r = None
    for i in range(n):
        r = f.update(_det(dist, lat), t0 + i * dt)
    return r, t0 + (n - 1) * dt


# ── 거리 제어 ────────────────────────────────────────────────────────────

def test_far_speeds_up():
    f = _f(target_m=1.5)
    r, _ = _settle(f, 3.0)
    assert r.ok and r.v > 0.3


def test_at_target_holds():
    f = _f(target_m=1.5, kd=0.0)
    r, _ = _settle(f, 1.5)
    assert r.ok
    assert r.v == pytest.approx(0.0, abs=0.05)


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


def test_closing_fast_slows_down():
    """★★ 앞 로봇이 급정거 → 우리가 빠르게 다가간다 → **미리 감속**해야 한다.

    거리항만 보면 아직 목표(1.5 m)보다 멀어서(2.0 m) **가속**한다 — 그대로 들이받는다.
    접근 속도(kd) 항이 그걸 막는다.
    """
    approaching = [(3.0, 0.0), (2.6, 0.1), (2.2, 0.2), (2.0, 0.3)]   # 빠르게 접근

    no_damping = _f(target_m=1.5, kd=0.0)
    v_no = None
    for d, t in approaching:
        v_no = no_damping.update(_det(d), t).v

    with_damping = _f(target_m=1.5, kd=1.2)
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
    left, _ = _settle(f, 2.0, lat=-0.5)      # 앞 로봇이 왼쪽(광학 x 음수)
    assert left.omega > 0                     # 왼쪽으로 튼다

    f2 = _f()
    right, _ = _settle(f2, 2.0, lat=+0.5)
    assert right.omega < 0


def test_omega_clamped():
    f = _f(k_yaw=100.0, omega_max=1.0)
    r, _ = _settle(f, 2.0, lat=-1.0)
    assert abs(r.omega) <= 1.0 + 1e-9


# ── 놓침 ─────────────────────────────────────────────────────────────────

def test_brief_loss_is_tolerated():
    """★ 한 프레임 놓쳤다고 급정거하면 안 된다 — YOLO 는 흔들린다."""
    f = _f(lost_grace_s=0.5)
    _settle(f, 2.0, n=5)
    r = f.update([], 0.45)                    # 잠깐 놓침 (유예 안)
    assert not r.ok
    assert "유예" in r.reason


def test_long_loss_gives_up():
    """★ 오래 놓치면 **따라가지 않는다** — 아무것도 없는 곳으로 계속 달리면 안 된다."""
    f = _f(lost_grace_s=0.5)
    _settle(f, 2.0, n=5)
    r = f.update([], 2.0)
    assert not r.ok
    assert "놓침" in r.reason
    assert (r.v, r.omega) == (0.0, 0.0)


def test_too_far_is_not_tracked():
    f = _f(max_m=6.0)
    r = f.update(_det(9.0), 0.0)
    assert not r.ok


def test_low_confidence_ignored():
    f = _f(min_confidence=0.5)
    r = f.update(_det(2.0, conf=0.2), 0.0)
    assert not r.ok


def test_wrong_class_ignored():
    f = _f(class_name="robot")
    r = f.update(_det(2.0, name="사람"), 0.0)
    assert not r.ok


def test_nearest_robot_wins():
    f = _f()
    for i in range(6):
        r = f.update([("robot", 0.9, 3.0, 0.0), ("robot", 0.9, 1.8, 0.0)], i * 0.1)
    assert r.distance_m == pytest.approx(1.8, abs=0.2)
