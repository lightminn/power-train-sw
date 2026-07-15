"""미션 시퀀서 — 로봇팔 핸드셰이크 (WP8).

착수계획서 §3.1 에 "남은 검증"으로 적혀 있던 3가지를 여기서 못박는다:
  ① MISSION_STOP → (정지 확인) → ArrivalStatus 순서
  ② DONE 누락/중복 시 타임아웃·재시도 — **타임아웃은 재출발 사유가 아니다**
  ③ 유효한 DONE 이후에만 재출발
"""
import pytest
import chassis.mission as mission_module
import ast
from pathlib import Path
from types import SimpleNamespace

from chassis.mission import (
    ARM_DONE, DRIVE, FAILED, MODE_DRIVING, MODE_MISSION_STOP, STOPPING, WAIT_ARM,
    MissionConfig, MissionSequencer,
)
from chassis.mission import (
    ARM_CARRYING_LOCKED,
    ARM_EXECUTING,
    ARM_FAILED,
    ARM_GRIP_LOST,
    ARM_PERCEIVING,
    ARM_PLANNING,
    ARM_STOWED_LOCKED,
    ARM_WORK,
    ARM_WORK_READY,
    ARRIVED_DROP,
    ARRIVED_PICKUP,
    COMPLETE,
    DROP,
    DIAGNOSTIC_FAILURES,
    EVENT_HOLD,
    FAILED_HOLD,
    GRIP_LOST_HOLD,
    MODE_STOW_REQUEST,
    PICKUP,
    READY,
    RESUME,
    STOP_REQUESTED,
    STOW_VERIFY,
    SUPERVISOR_STATES,
    MissionSupervisor,
)


def test_v2_mission_supervisor_exists_in_team_owned_pure_core():
    assert hasattr(mission_module, "MissionSupervisor")


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


# ── WP5.2 contract-v2 MissionSupervisor ────────────────────────────────────


class _Store:
    def __init__(self, *mission_ids):
        self.ids = list(mission_ids or (1,))
        self.calls = 0

    def allocate(self):
        self.calls += 1
        if not self.ids:
            return SimpleNamespace(
                accepted=False,
                mission_id=None,
                hold_reason="mission_id_store:exhausted_fixture",
            )
        return SimpleNamespace(
            accepted=True,
            mission_id=self.ids.pop(0),
            hold_reason="",
        )


class _WheelStop:
    def __init__(self, *, qualified=True, confirmed=True):
        self.qualified = qualified
        self.confirmed = confirmed
        self.last_reject_reason = "fixture_pending"


def _supervisor(
    *,
    mission_ids=(1,),
    qualified=True,
    confirmed=True,
    authority_zero=True,
    clear_grip_lost=None,
):
    store = _Store(*mission_ids)
    stop = _WheelStop(qualified=qualified, confirmed=confirmed)
    supervisor = MissionSupervisor(
        store,
        wheel_stop=stop,
        authority_output_zero=lambda: authority_zero,
        clear_grip_lost=clear_grip_lost,
    )
    return supervisor, store, stop


def _locked(supervisor, status, *, mission_id=77, t=10.0):
    return supervisor.on_arm_status(status, mission_id, t, t)


def _request_and_publish(
    supervisor,
    arrival=ARRIVED_PICKUP,
    *,
    posture=ARM_STOWED_LOCKED,
    t=10.0,
):
    _locked(supervisor, posture, mission_id=77, t=t)
    requested = supervisor.request_work(arrival, t + 0.01)
    assert requested.accepted is True
    published = supervisor.tick(t + 0.02)
    assert published.publish_arrival is not None
    return published


def _start_arm_work(
    supervisor,
    arrival=ARRIVED_PICKUP,
    *,
    posture=ARM_STOWED_LOCKED,
    t=10.0,
):
    published = _request_and_publish(
        supervisor,
        arrival,
        posture=posture,
        t=t,
    )
    mission_id = published.publish_arrival[0]
    result = supervisor.on_arm_status(
        ARM_WORK_READY,
        mission_id,
        t + 0.03,
        t + 0.03,
    )
    assert result.state == ARM_WORK
    return mission_id


def test_supervisor_state_vocabulary_is_exactly_contract_v2():
    assert set(SUPERVISOR_STATES) == {
        READY,
        DRIVE,
        STOP_REQUESTED,
        EVENT_HOLD,
        ARM_WORK,
        STOW_VERIFY,
        RESUME,
        COMPLETE,
        FAILED_HOLD,
        GRIP_LOST_HOLD,
    }


def test_unqualified_stop_predicate_rejects_work_without_allocating_id():
    supervisor, store, _stop = _supervisor(qualified=False)
    _locked(supervisor, ARM_STOWED_LOCKED)

    result = supervisor.request_work(ARRIVED_PICKUP, 10.01)

    assert result.accepted is False
    assert result.state == EVENT_HOLD
    assert "unqualified" in result.hold_reason
    assert result.publish_arrival is None
    assert store.calls == 0


def test_one_unstopped_wheel_keeps_stop_pending_without_work_permit():
    supervisor, store, stop = _supervisor(confirmed=False)
    _locked(supervisor, ARM_STOWED_LOCKED)
    assert supervisor.request_work(ARRIVED_PICKUP, 10.01).accepted is True

    pending = supervisor.tick(10.02)

    assert pending.state == STOP_REQUESTED
    assert pending.publish_arrival is None
    assert pending.allow_drive is False
    assert store.calls == 0
    stop.confirmed = True
    published = supervisor.tick(10.03)
    assert published.publish_arrival == (1, ARRIVED_PICKUP)


def test_authority_final_output_must_be_zero_before_mode_and_arrival():
    zero = SimpleNamespace(value=False)
    supervisor, store, _stop = _supervisor()
    supervisor.authority_output_zero = lambda: zero.value
    _locked(supervisor, ARM_STOWED_LOCKED)
    supervisor.request_work(ARRIVED_PICKUP, 10.01)

    pending = supervisor.tick(10.02)

    assert pending.publish_arrival is None
    assert pending.mode_intent == MODE_STOW_REQUEST
    assert store.calls == 0
    zero.value = True
    published = supervisor.tick(10.03)
    assert published.mode_intent == MODE_MISSION_STOP
    assert published.publish_arrival == (1, ARRIVED_PICKUP)


def test_pickup_and_drop_require_fresh_payload_specific_locked_posture():
    pickup, pickup_store, _ = _supervisor()
    _locked(pickup, ARM_CARRYING_LOCKED)
    rejected_pickup = pickup.request_work(ARRIVED_PICKUP, 10.01)
    assert rejected_pickup.accepted is False
    assert pickup_store.calls == 0

    drop, drop_store, _ = _supervisor()
    _locked(drop, ARM_STOWED_LOCKED)
    rejected_drop = drop.request_work(ARRIVED_DROP, 10.01)
    assert rejected_drop.accepted is False
    assert drop_store.calls == 0

    stale, stale_store, _ = _supervisor()
    _locked(stale, ARM_STOWED_LOCKED, t=1.0)
    rejected_stale = stale.request_work(ARRIVED_PICKUP, 2.0)
    assert rejected_stale.accepted is False
    assert "fresh" in rejected_stale.hold_reason
    assert stale_store.calls == 0


def test_idle_locked_heartbeat_id_is_not_interpreted_for_new_work():
    supervisor, _store, _stop = _supervisor(mission_ids=(88,))
    _locked(supervisor, ARM_STOWED_LOCKED, mission_id=41)

    result = supervisor.request_work(ARRIVED_PICKUP, 10.01)
    published = supervisor.tick(10.02)

    assert result.accepted is True
    assert published.publish_arrival == (88, ARRIVED_PICKUP)


def test_arrival_republishes_at_two_hz_for_less_than_two_seconds_then_holds():
    supervisor, _store, _stop = _supervisor()
    initial = _request_and_publish(supervisor)
    assert initial.publish_arrival == (1, ARRIVED_PICKUP)

    assert supervisor.tick(10.51).publish_arrival is None
    assert supervisor.tick(10.52).publish_arrival == (1, ARRIVED_PICKUP)
    assert supervisor.tick(11.02).publish_arrival == (1, ARRIVED_PICKUP)
    assert supervisor.tick(11.52).publish_arrival == (1, ARRIVED_PICKUP)

    timeout = supervisor.tick(12.02)
    assert timeout.state == EVENT_HOLD
    assert timeout.publish_arrival is None
    assert supervisor.arrival_republish_active is False
    assert "operator" in timeout.operator_notice
    assert supervisor.tick(30.0).publish_arrival is None


@pytest.mark.parametrize(
    "accepted_status",
    (ARM_WORK_READY, ARM_PERCEIVING, ARM_PLANNING, ARM_EXECUTING),
)
def test_same_id_work_accepted_status_stops_arrival_republish(accepted_status):
    supervisor, _store, _stop = _supervisor()
    initial = _request_and_publish(supervisor)
    mission_id = initial.publish_arrival[0]

    accepted = supervisor.on_arm_status(
        accepted_status,
        mission_id,
        10.2,
        10.2,
    )

    assert accepted.state == ARM_WORK
    assert supervisor.arrival_republish_active is False
    assert supervisor.work_start_count == 1
    assert supervisor.tick(10.6).publish_arrival is None


def test_duplicate_work_ack_for_same_id_starts_exactly_one_arm_job():
    supervisor, _store, _stop = _supervisor()
    initial = _request_and_publish(supervisor)
    mission_id = initial.publish_arrival[0]

    supervisor.on_arm_status(ARM_WORK_READY, mission_id, 10.2, 10.2)
    supervisor.on_arm_status(ARM_EXECUTING, mission_id, 10.3, 10.3)

    assert supervisor.state == ARM_WORK
    assert supervisor.work_start_count == 1


def test_wrong_or_previous_id_status_never_starts_or_resumes_work():
    supervisor, _store, _stop = _supervisor()
    initial = _request_and_publish(supervisor)
    mission_id = initial.publish_arrival[0]

    supervisor.on_arm_status(ARM_WORK_READY, mission_id - 1, 10.2, 10.2)
    supervisor.on_arm_status(ARM_CARRYING_LOCKED, mission_id - 1, 10.3, 10.3)

    assert supervisor.state == EVENT_HOLD
    assert supervisor.work_start_count == 0
    assert supervisor.tick(10.52).allow_drive is False


def test_done_is_diagnostic_only_and_locked_success_ack_drives_resume():
    supervisor, _store, _stop = _supervisor()
    mission_id = _start_arm_work(supervisor)

    done = supervisor.on_arm_status(ARM_DONE, mission_id, 10.1, 10.1)
    assert done.state == ARM_WORK
    assert done.allow_drive is False
    assert supervisor.diagnostic_events[-1][0] == ARM_DONE

    locked = supervisor.on_arm_status(
        ARM_CARRYING_LOCKED,
        mission_id,
        10.2,
        10.2,
    )
    assert locked.state == STOW_VERIFY
    assert supervisor.tick(10.21).state == RESUME
    assert supervisor.tick(10.22).state == COMPLETE
    driving = supervisor.tick(10.23)
    assert driving.state == DRIVE
    assert driving.allow_drive is True
    assert supervisor.last_completed_mission_id == mission_id


def test_drop_success_requires_same_id_fresh_stowed_locked():
    supervisor, _store, _stop = _supervisor()
    mission_id = _start_arm_work(
        supervisor,
        ARRIVED_DROP,
        posture=ARM_CARRYING_LOCKED,
    )

    supervisor.on_arm_status(ARM_CARRYING_LOCKED, mission_id, 10.2, 10.2)
    assert supervisor.state == ARM_WORK
    supervisor.on_arm_status(ARM_STOWED_LOCKED, mission_id, 10.3, 10.3)
    assert supervisor.state == STOW_VERIFY


def test_failed_hold_preserves_wire_evidence_and_pickup_operation_latch():
    supervisor, _store, _stop = _supervisor()
    mission_id = _start_arm_work(supervisor)

    result = supervisor.on_arm_status(ARM_FAILED, mission_id, 10.4, 10.4)

    assert result.state == FAILED_HOLD
    assert result.mode_intent == MODE_STOW_REQUEST
    assert supervisor.arrival_republish_active is False
    assert supervisor.failure.wire_status == ARM_FAILED
    assert supervisor.failure.mission_id == mission_id
    assert supervisor.failure.stamp_s == 10.4
    assert supervisor.failure.last_locked_posture == ARM_STOWED_LOCKED
    assert supervisor.failure.operation == PICKUP
    assert supervisor.failure.arm_latched is True


@pytest.mark.parametrize("wire_status", sorted(DIAGNOSTIC_FAILURES))
def test_optional_diagnostic_failure_causes_same_hold_and_preserves_wire_status(
    wire_status,
):
    supervisor, _store, _stop = _supervisor()
    mission_id = _start_arm_work(supervisor)

    result = supervisor.on_arm_status(wire_status, mission_id, 10.4, 10.4)

    assert result.state == FAILED_HOLD
    assert supervisor.failure.wire_status == wire_status
    assert supervisor.failure.operation == PICKUP


def test_failed_during_arrival_ack_window_stops_republication_immediately():
    supervisor, _store, _stop = _supervisor()
    initial = _request_and_publish(supervisor)
    mission_id = initial.publish_arrival[0]

    result = supervisor.on_arm_status(ARM_FAILED, mission_id, 10.2, 10.2)

    assert result.state == FAILED_HOLD
    assert supervisor.arrival_republish_active is False
    assert supervisor.tick(10.52).publish_arrival is None


def test_pickup_failure_closes_only_after_release_to_stowed_and_explicit_skip():
    supervisor, _store, _stop = _supervisor()
    mission_id = _start_arm_work(supervisor)
    supervisor.on_arm_status(ARM_FAILED, mission_id, 10.4, 10.4)

    supervisor.on_arm_status(ARM_CARRYING_LOCKED, mission_id, 10.5, 10.5)
    rejected = supervisor.resolve_failure("skip", 10.51)
    assert rejected.state == FAILED_HOLD
    assert rejected.accepted is False

    supervisor.on_arm_status(ARM_STOWED_LOCKED, mission_id, 10.6, 10.6)
    assert supervisor.state == FAILED_HOLD
    closed = supervisor.resolve_failure("skip", 10.61)
    assert closed.accepted is True
    assert closed.state == COMPLETE


def test_drop_failure_never_collapses_to_empty_stowed_locked():
    supervisor, _store, _stop = _supervisor()
    mission_id = _start_arm_work(
        supervisor,
        ARRIVED_DROP,
        posture=ARM_CARRYING_LOCKED,
    )
    supervisor.on_arm_status(ARM_FAILED, mission_id, 10.4, 10.4)
    assert supervisor.failure.operation == DROP

    supervisor.on_arm_status(ARM_STOWED_LOCKED, mission_id, 10.5, 10.5)
    rejected = supervisor.resolve_failure("skip", 10.51)
    assert rejected.accepted is False
    assert rejected.state == FAILED_HOLD

    supervisor.on_arm_status(ARM_CARRYING_LOCKED, mission_id, 10.6, 10.6)
    closed = supervisor.resolve_failure("skip", 10.61)
    assert closed.accepted is True
    assert closed.state == COMPLETE


def test_failure_retry_allocates_a_new_id_and_never_replays_old_arrival():
    supervisor, store, _stop = _supervisor(mission_ids=(5, 6))
    mission_id = _start_arm_work(supervisor)
    supervisor.on_arm_status(ARM_FAILED, mission_id, 10.4, 10.4)
    supervisor.on_arm_status(ARM_STOWED_LOCKED, mission_id, 10.5, 10.5)

    retry = supervisor.resolve_failure("retry", 10.51)

    assert retry.accepted is True
    assert retry.state == STOP_REQUESTED
    assert retry.publish_arrival is None
    new_attempt = supervisor.tick(10.52)
    assert new_attempt.publish_arrival == (6, ARRIVED_PICKUP)
    assert store.calls == 2


def test_grip_lost_requires_explicit_bounded_regrasp_transition():
    clear_calls = []
    supervisor, _store, _stop = _supervisor(
        clear_grip_lost=lambda authorized=False: clear_calls.append(authorized) or authorized,
    )
    mission_id = _start_arm_work(supervisor)
    supervisor.on_arm_status(ARM_GRIP_LOST, mission_id, 10.4, 10.4)
    assert supervisor.state == GRIP_LOST_HOLD

    supervisor.on_arm_status(ARM_CARRYING_LOCKED, mission_id, 10.5, 10.5)
    assert supervisor.state == GRIP_LOST_HOLD
    assert supervisor.tick(99.0).state == GRIP_LOST_HOLD
    supervisor.on_arm_status(ARM_CARRYING_LOCKED, mission_id, 99.1, 99.1)
    assert supervisor.state == GRIP_LOST_HOLD

    recovered = supervisor.confirm_regrasp(99.11)
    assert recovered.accepted is True
    assert recovered.state == STOW_VERIFY
    assert clear_calls == [True]


def test_bounded_regrasp_cannot_exit_if_interlock_clear_fails():
    supervisor, _store, _stop = _supervisor(
        clear_grip_lost=lambda authorized=False: False,
    )
    mission_id = _start_arm_work(supervisor)
    supervisor.on_arm_status(ARM_GRIP_LOST, mission_id, 10.4, 10.4)
    supervisor.on_arm_status(ARM_CARRYING_LOCKED, mission_id, 10.5, 10.5)

    rejected = supervisor.confirm_regrasp(10.51)

    assert rejected.accepted is False
    assert supervisor.state == GRIP_LOST_HOLD
    assert "interlock_clear" in rejected.hold_reason


def test_grip_lost_during_carrying_drive_enters_supervisor_latch():
    supervisor, _store, _stop = _supervisor(
        clear_grip_lost=lambda authorized=False: authorized,
    )
    mission_id = _start_arm_work(supervisor)
    supervisor.on_arm_status(ARM_CARRYING_LOCKED, mission_id, 10.2, 10.2)
    supervisor.tick(10.21)
    supervisor.tick(10.22)
    assert supervisor.tick(10.23).state == DRIVE
    assert supervisor.active_mission_id is None

    lost = supervisor.on_arm_status(ARM_GRIP_LOST, mission_id, 10.3, 10.3)

    assert lost.state == GRIP_LOST_HOLD
    assert supervisor.active_mission_id == mission_id
    assert supervisor.failure.operation == PICKUP
    assert supervisor.tick(20.0).state == GRIP_LOST_HOLD


def test_operator_grip_clear_requires_authorization_interlock_and_typed_lock():
    clear_calls = []
    supervisor, _store, _stop = _supervisor(
        clear_grip_lost=lambda authorized=False: clear_calls.append(authorized) or authorized,
    )
    mission_id = _start_arm_work(supervisor)
    supervisor.on_arm_status(ARM_GRIP_LOST, mission_id, 10.4, 10.4)
    supervisor.on_arm_status(ARM_STOWED_LOCKED, mission_id, 10.5, 10.5)

    unauthorized = supervisor.operator_clear_grip_lost(False, 10.51)
    assert unauthorized.accepted is False
    assert clear_calls == []
    assert supervisor.state == GRIP_LOST_HOLD

    cleared = supervisor.operator_clear_grip_lost(True, 10.52)
    assert cleared.accepted is True
    assert clear_calls == [True]
    assert cleared.state == FAILED_HOLD


def test_arm_restart_or_delayed_heartbeat_cannot_revive_pending_work():
    old, _store, _stop = _supervisor()
    initial = _request_and_publish(old)
    mission_id = initial.publish_arrival[0]
    old.tick(12.02)
    old.on_arm_status(ARM_WORK_READY, mission_id, 12.03, 12.03)
    assert old.state == EVENT_HOLD
    assert old.arrival_republish_active is False

    restarted, _new_store, _new_stop = _supervisor(mission_ids=(2,))
    restarted.on_arm_status(ARM_WORK_READY, mission_id, 12.04, 12.04)
    assert restarted.state == READY
    assert restarted.arrival_republish_active is False


def test_override_aborts_active_mission_stops_republish_then_requests_stow():
    supervisor, _store, _stop = _supervisor()
    _request_and_publish(supervisor)

    assert supervisor.abort_for_override(10.2) is True
    assert supervisor.arrival_republish_active is False
    assert supervisor.mode_intent == MODE_STOW_REQUEST
    assert supervisor.state == FAILED_HOLD
    assert supervisor.failure.wire_status == "ABORT_OVERRIDE"
    assert supervisor.tick(10.6).publish_arrival is None


def test_override_before_id_allocation_cannot_revive_pending_arrival():
    supervisor, store, stop = _supervisor(confirmed=False)
    _locked(supervisor, ARM_STOWED_LOCKED)
    assert supervisor.request_work(ARRIVED_PICKUP, 10.01).accepted is True
    assert supervisor.state == STOP_REQUESTED

    assert supervisor.abort_for_override(10.02) is True
    stop.confirmed = True
    later = supervisor.tick(20.0)

    assert later.state == FAILED_HOLD
    assert later.publish_arrival is None
    assert supervisor.arrival_republish_active is False
    assert store.calls == 0


def test_callback_and_allocator_exceptions_become_hold_results_not_raises():
    class _ExplodingStop:
        @property
        def qualified(self):
            raise RuntimeError("qualification exploded")

    supervisor, _store, _stop = _supervisor()
    supervisor.wheel_stop = _ExplodingStop()
    _locked(supervisor, ARM_STOWED_LOCKED)
    rejected = supervisor.request_work(ARRIVED_PICKUP, 10.01)
    assert rejected.state == EVENT_HOLD
    assert "exception" in rejected.hold_reason

    allocation, _store, _stop = _supervisor()
    _locked(allocation, ARM_STOWED_LOCKED)
    allocation.request_work(ARRIVED_PICKUP, 10.01)
    allocation.mission_id_store.allocate = lambda: (_ for _ in ()).throw(
        OSError("store exploded")
    )
    held = allocation.tick(10.02)
    assert held.state == EVENT_HOLD
    assert "mission_id_store_exception" in held.hold_reason


def test_mission_supervisor_core_has_no_ros_imports():
    source = Path(mission_module.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".")[0])

    assert "rclpy" not in imported_roots
    assert "robot_arm_msgs" not in imported_roots
