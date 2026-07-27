import pytest
from chassis.approach import (
    ApproachConfig, Target, ApproachDecision, creep_cmd,
    SEARCHING, APPROACHING, ALIGNED, ARRIVED_FIRED, DONE, BACKOFF, FAILED_HOLD,
)


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
