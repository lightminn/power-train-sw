"""미션 시퀀서 — 로봇팔 핸드셰이크 (WP8).

착수계획서 §3.1 에 "남은 검증"으로 적혀 있던 3가지를 여기서 못박는다:
  ① MISSION_STOP → (정지 확인) → ArrivalStatus 순서
  ② DONE 누락/중복 시 타임아웃·재시도 — **타임아웃은 재출발 사유가 아니다**
  ③ 유효한 DONE 이후에만 재출발
"""
import pytest

from chassis.mission import (
    ARM_DONE, DRIVE, FAILED, MODE_DRIVING, MODE_MISSION_STOP, STOPPING, WAIT_ARM,
    MissionConfig, MissionSequencer,
)


def _seq(**kw):
    return MissionSequencer(MissionConfig(**kw))


def _drive_to_stop(s, t0=0.0, mid=7, status="ARRIVED_PICKUP"):
    """도착 → 정차 확인 → ArrivalStatus 발행까지 진행. 발행 시각을 돌려준다."""
    s.arrive(mid, status, t=t0)
    t = t0
    for _ in range(20):
        t += 0.1
        d = s.update(t, speed_mps=0.0)
        if d.publish_arrival is not None:
            return t, d
    raise AssertionError("ArrivalStatus 가 발행되지 않았다")


# ── ① 순서: MISSION_STOP 먼저, ArrivalStatus 는 정지 확인 후 ─────────────

def test_mission_stop_is_published_before_arrival():
    """★ 팔은 ArrivalStatus 를 받으면 '작업 시작해도 된다'고 이해한다.

    그때 차체가 **아직 굴러가고 있으면 팔이 뻗은 채로 로봇이 움직인다** — 팔이 부러진다.
    그래서 MISSION_STOP 이 먼저고, ArrivalStatus 는 **실제 정지를 확인한 뒤**다.
    """
    s = _seq()
    s.arrive(7, "ARRIVED_PICKUP", t=0.0)

    d = s.update(0.1, speed_mps=0.5)                 # 아직 굴러간다
    assert d.chassis_mode == MODE_MISSION_STOP       # ★ 모드는 이미 MISSION_STOP
    assert d.publish_arrival is None                 # ★ 그러나 ArrivalStatus 는 아직
    assert d.allow_drive is False
    assert d.state == STOPPING


def test_arrival_only_after_speed_settles():
    """정지가 **settle 시간 동안 유지**돼야 한다 — 한 순간 0 이었다고 보내면 안 된다."""
    s = _seq(stop_settle_s=0.3)
    s.arrive(7, "ARRIVED_PICKUP", t=0.0)

    assert s.update(0.1, 0.0).publish_arrival is None    # 방금 멈춤
    assert s.update(0.2, 0.0).publish_arrival is None
    assert s.update(0.3, 0.0).publish_arrival is None    # 0.3s 경과 직전
    d = s.update(0.45, 0.0)
    assert d.publish_arrival == (7, "ARRIVED_PICKUP")    # 이제 보낸다
    assert d.state == WAIT_ARM


def test_moving_again_restarts_the_settle_timer():
    s = _seq(stop_settle_s=0.3)
    s.arrive(7, "ARRIVED_PICKUP", t=0.0)
    s.update(0.1, 0.0)
    s.update(0.2, 0.4)                                   # 다시 움직였다
    assert s.update(0.4, 0.0).publish_arrival is None    # 타이머 재시작
    assert s.update(0.8, 0.0).publish_arrival is not None


def test_stop_failure_goes_to_failed():
    """정차가 안 되면(브레이크 고장?) ArrivalStatus 를 보내면 안 된다."""
    s = _seq(stop_timeout_s=1.0)
    s.arrive(7, "ARRIVED_PICKUP", t=0.0)
    d = None
    for i in range(1, 30):
        d = s.update(i * 0.1, speed_mps=0.5)             # 계속 굴러간다
        if d.state == FAILED:
            break                                        # 전환 시점의 사유를 본다
    assert d.state == FAILED
    assert d.allow_drive is False
    assert "정차 실패" in d.reason


# ── ③ 재출발 게이트: 유효한 DONE 만 ──────────────────────────────────────

def test_valid_done_resumes():
    s = _seq()
    t, _ = _drive_to_stop(s, mid=7)
    assert "유효한 DONE" in s.on_arm_status(7, ARM_DONE, t + 1.0)
    d = s.update(t + 1.1, 0.0)
    assert d.allow_drive is True
    assert d.chassis_mode == MODE_DRIVING
    assert d.state == DRIVE


def test_wrong_mission_id_is_ignored():
    """★ 이전 미션의 뒤늦은 DONE 으로 재출발하면 안 된다."""
    s = _seq()
    t, _ = _drive_to_stop(s, mid=7)
    why = s.on_arm_status(6, ARM_DONE, t + 1.0)          # 이전 미션
    assert "mission_id 불일치" in why
    assert s.update(t + 1.1, 0.0).allow_drive is False   # 여전히 정지


def test_done_before_arrival_is_ignored():
    """★ ArrivalStatus 를 보내기 **전에** 온 DONE = 재생·잔류 메시지."""
    s = _seq()
    s.arrive(7, "ARRIVED_PICKUP", t=0.0)
    why = s.on_arm_status(7, ARM_DONE, t=0.05)           # 아직 STOPPING
    assert "잔류/재생" in why

    for i in range(1, 10):                               # 정차 확인 → WAIT_ARM 진입
        s.update(0.1 * i, 0.0)
    assert s.state == WAIT_ARM
    # ★ 그 DONE 은 무효였으므로 재출발하지 않는다 — 진짜 DONE 을 기다린다
    assert s.update(1.0, 0.0).allow_drive is False


def test_duplicate_done_after_resume_is_ignored():
    s = _seq()
    t, _ = _drive_to_stop(s, mid=7)
    s.on_arm_status(7, ARM_DONE, t + 1.0)
    s.update(t + 1.1, 0.0)                               # 재출발
    assert s.state == DRIVE
    why = s.on_arm_status(7, ARM_DONE, t + 1.5)          # 중복 DONE
    assert "무시" in why
    assert s.update(t + 1.6, 0.4).allow_drive is True    # 계속 주행 (영향 없음)


def test_non_done_status_is_ignored():
    s = _seq()
    t, _ = _drive_to_stop(s, mid=7)
    s.on_arm_status(7, "EXECUTING", t + 1.0)
    assert s.update(t + 1.1, 0.0).allow_drive is False   # 아직 대기


# ── ② 타임아웃 · 재시도 ──────────────────────────────────────────────────

def test_arrival_is_resent_on_timeout():
    """ArrivalStatus 가 유실됐을 수 있다 → 다시 보낸다."""
    s = _seq(done_timeout_s=2.0, max_retries=2)
    t, _ = _drive_to_stop(s, mid=7)

    assert s.update(t + 1.0, 0.0).publish_arrival is None   # 아직 대기
    d = s.update(t + 2.1, 0.0)
    assert d.publish_arrival == (7, "ARRIVED_PICKUP")       # 재전송 1
    assert "재전송 (1/2)" in d.reason


def test_timeout_is_not_a_reason_to_resume():
    """★★ 가장 중요한 규칙 — 타임아웃으로 재출발하면 **팔이 부러진다.**

    DONE 이 끝내 안 오면 FAILED 로 가서 **정지 상태로 사람을 기다린다.**
    """
    s = _seq(done_timeout_s=1.0, max_retries=1)
    t, _ = _drive_to_stop(s, mid=7)

    s.update(t + 1.1, 0.0)                              # 재전송 1
    d = s.update(t + 2.2, 0.0)                          # 재시도 소진
    assert d.state == FAILED
    assert d.allow_drive is False                       # ★ 절대 재출발 안 함
    assert "사람 개입" in d.reason


def test_failed_stays_stopped_until_reset():
    s = _seq(done_timeout_s=1.0, max_retries=0)
    t, _ = _drive_to_stop(s, mid=7)
    s.update(t + 1.1, 0.0)
    assert s.state == FAILED

    for i in range(10):                                 # 시간이 지나도 안 풀린다
        d = s.update(t + 2.0 + i, 0.0)
        assert d.allow_drive is False

    s.reset(t + 20.0)                                   # 사람이 확인 후 리셋
    assert s.update(t + 20.1, 0.0).allow_drive is True


def test_done_arriving_during_retry_still_works():
    s = _seq(done_timeout_s=1.0, max_retries=2)
    t, _ = _drive_to_stop(s, mid=7)
    s.update(t + 1.1, 0.0)                              # 재전송 1
    s.on_arm_status(7, ARM_DONE, t + 1.5)               # 그 사이 DONE 도착
    assert s.update(t + 1.6, 0.0).allow_drive is True


# ── 상태 격리 ────────────────────────────────────────────────────────────

def test_arrive_is_ignored_while_busy():
    """미션 처리 중에 또 도착 신호가 와도 무시한다 (한 번에 하나)."""
    s = _seq()
    assert s.arrive(7, "ARRIVED_PICKUP", t=0.0) is True
    assert s.arrive(8, "ARRIVED_DROP", t=0.1) is False
    assert s.mission_id == 7
