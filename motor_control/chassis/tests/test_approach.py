import pytest
from chassis.approach import (
    ApproachConfig, Target, ApproachDecision, creep_cmd,
    ARRIVED_PICKUP, SEARCHING, APPROACHING, ALIGNED, ARRIVED_FIRED,
    DONE, BACKOFF, FAILED_HOLD,
)
from chassis.approach import ApproachController


def test_creep_forward_and_center():
    cfg = ApproachConfig(stop_m=1.0, k_dist=0.8, v_approach_max=0.15,
                         k_yaw=1.5, omega_max=0.4)
    # 멀고(전방 3m) 오른쪽으로 치우침(y=-0.2) → 전진 + 좌회전(omega>0)
    v, omega = creep_cmd(cfg, x=3.0, y=-0.2)
    assert 0.0 < v <= 0.15                      # 전진, 상한 클램프
    assert omega > 0.0                          # y<0(우측) → +omega(좌회전)로 중앙 복귀
    assert abs(omega) <= 0.4


def test_creep_no_reverse_past_stop():
    cfg = ApproachConfig(stop_m=1.0, k_dist=0.8, v_approach_max=0.15)
    # 이미 stop_m 안쪽(x=0.8) → 전진 명령 0 (후진 없음)
    v, omega = creep_cmd(cfg, x=0.8, y=0.0)
    assert v == 0.0


def test_creep_yaw_clamped():
    cfg = ApproachConfig(k_yaw=100.0, omega_max=0.4)
    v, omega = creep_cmd(cfg, x=2.0, y=1.0)
    assert omega == pytest.approx(-0.4)         # 큰 오차라도 omega_max로 클램프


def _targets(cls="box", conf=0.9, x=1.8, y=0.0):
    return [Target(cls, conf, x, y)]


def _run(ctl, targets_seq, speed=0.0, t0=0.0, dt=0.1):
    """프레임 시퀀스를 먹이고 마지막 Decision 반환."""
    d = None
    for i, tg in enumerate(targets_seq):
        d = ctl.update(tg, speed, t0 + i * dt)
    return d


def test_lock_after_debounce_enters_approaching():
    cfg = ApproachConfig(engage_m=2.0, consecutive=5)
    ctl = ApproachController(cfg)
    # 4프레임은 아직 lock 안 됨(디바운스)
    d = _run(ctl, [_targets(x=1.8)] * 4)
    assert d.state == SEARCHING
    assert d.active is False
    # 5번째 프레임 → lock
    d = ctl.update(_targets(x=1.8), 0.0, 0.5)
    assert d.state == APPROACHING
    assert d.active is True
    assert d.v > 0.0                            # 전진 크립


def test_aligned_fires_once_when_stopped_and_centered():
    cfg = ApproachConfig(engage_m=2.0, stop_m=1.0, lat_tol=0.05,
                         dist_tol=0.05, v_settle=0.03, consecutive=1)
    ctl = ApproachController(cfg)
    ctl.update(_targets(x=1.8), 0.0, 0.0)       # lock (consecutive=1)
    # 정렬+정지: x≈stop_m, y≈0, speed<v_settle
    d = ctl.update(_targets(x=1.0, y=0.0), 0.0, 0.1)
    assert d.state == ALIGNED
    assert d.fire == ARRIVED_PICKUP
    assert d.v == 0.0
    # 다음 tick은 중복 발사 금지
    d2 = ctl.update(_targets(x=1.0, y=0.0), 0.0, 0.2)
    assert d2.fire is None


def test_not_aligned_while_moving():
    cfg = ApproachConfig(stop_m=1.0, v_settle=0.03, consecutive=1,
                         dist_tol=0.05, lat_tol=0.05)
    ctl = ApproachController(cfg)
    ctl.update(_targets(x=1.8), 0.0, 0.0)
    # 위치는 맞지만 아직 움직이는 중(speed=0.1>v_settle) → 발사 금지
    d = ctl.update(_targets(x=1.0, y=0.0), 0.1, 0.1)
    assert d.state == APPROACHING
    assert d.fire is None


def test_timeout_retries_then_failed_hold():
    cfg = ApproachConfig(engage_m=2.0, stop_m=1.0, align_timeout_s=1.0,
                         max_retries=1, backoff_time_s=0.2, consecutive=1,
                         lat_tol=0.001, dist_tol=0.001)
    ctl = ApproachController(cfg)
    ctl.update(_targets(x=1.8), 0.0, 0.0)       # lock, enter_s=0
    # 정렬 불가(항상 x=1.8, 오차 큼)로 시간만 흐름 → timeout at t>1.0
    d = ctl.update(_targets(x=1.8), 0.0, 1.2)   # retry 1 → backoff
    assert ctl.retries == 1
    assert d.state == BACKOFF
    # backoff 후에도 못 맞추고 다시 timeout → 재시도 소진 → FAILED_HOLD
    ctl.update(_targets(x=1.8), 0.0, 1.5)       # backoff 끝 → approaching (enter_s=1.5)
    d = ctl.update(_targets(x=1.8), 0.0, 2.8)   # timeout again, retries>=max
    assert d.state == FAILED_HOLD
    assert d.v == 0.0
    assert d.active is True


def test_lost_target_backoff():
    cfg = ApproachConfig(engage_m=2.0, lost_frames=3, backoff_creep=0.1,
                         consecutive=1, backoff_time_s=1.0)
    ctl = ApproachController(cfg)
    ctl.update(_targets(x=1.8), 0.0, 0.0)       # lock
    # 대상 사라짐 3프레임 → BACKOFF
    ctl.update([], 0.0, 0.1)
    ctl.update([], 0.0, 0.2)
    d = ctl.update([], 0.0, 0.3)
    assert d.state == BACKOFF
    assert d.v == pytest.approx(-0.1)           # 뒤로 크립
