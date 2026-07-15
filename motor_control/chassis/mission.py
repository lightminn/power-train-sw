"""미션 시퀀서 — 로봇팔 핸드셰이크 (WP8). 순수 계산 코어.

    주행 → 미션 지점 도착 → 정차 → 팔에 알림 → 팔 작업 → DONE → 재출발

하드웨어·ROS 의존 없음. ROS 래퍼는 `powertrain_ros/mission_node.py`.

────────────────────────────────────────────────────────────────────────
★ 미결이던 3가지를 여기서 못박는다 (착수계획서 §3.1 "남은 검증")
────────────────────────────────────────────────────────────────────────

**① 순서: `MISSION_STOP` 먼저, `ArrivalStatus` 나중.**
   팔은 `ArrivalStatus` 를 받으면 "작업 시작해도 된다"고 이해한다. 그런데 그때 차체가
   **아직 굴러가고 있으면** 팔이 뻗은 채로 로봇이 움직인다 — 팔이 부러진다.
   그래서 순서를 뒤집을 수 없다:
       ① `/chassis_mode` = MISSION_STOP  (차체가 이제 안 움직인다)
       ② **실제로 정지했는지 확인**(|v| ≤ eps 가 settle 시간 동안 유지)
       ③ `/arrival_status` = ARRIVED_*   (이제 작업해도 된다)
   ⚠️ 팔 팀 코드에 `MISSION_STOP` 이 **아직 없다**(그들은 무시한다). 그래도 우리는
      보낸다 — 계약은 우리가 지킨다. **팔이 지켜줄 거라고 믿고 움직이면 안 된다.**

**② `DONE` 이 안 오면: 재시도 → 실패. 절대 그냥 재출발하지 않는다.**
   `ArrivalStatus` 가 유실됐을 수 있으니 타임아웃 후 **다시 보낸다**(최대 N회).
   그래도 안 오면 `FAILED` — **정지 상태로 사람을 기다린다.** 팔이 뻗어 있는데 재출발하면
   팔이 부러지거나 로봇이 넘어진다. **타임아웃은 재출발 사유가 아니다.**

**③ 재출발 게이트: '유효한 DONE' 만 인정한다.**
   다음은 전부 **무시**한다 —
     · `mission_id` 불일치 (이전 미션의 뒤늦은 DONE)
     · 우리가 `ArrivalStatus` 를 보내기 **전에** 도착한 DONE (재생·잔류 메시지)
     · 이미 재출발한 뒤의 중복 DONE
"""
from dataclasses import dataclass, field
import math

from chassis.mission_id_store import MissionIdStore

# 상태
DRIVE = "DRIVE"                 # 주행 중
STOPPING = "STOPPING"           # 미션 지점 도착 → 정차 중 (MISSION_STOP 발행됨)
WAIT_ARM = "WAIT_ARM"           # ArrivalStatus 보냈고 DONE 대기
FAILED = "FAILED"               # DONE 이 끝내 안 옴 → 사람 개입 필요 (정지 유지)
STATES = (DRIVE, STOPPING, WAIT_ARM, FAILED)

# 팔 상태 (robot_arm_msgs 계약)
ARM_DONE = "DONE"
ARM_FAILED = "FAILED"

# 차체 모드 (contract.py 와 동일)
MODE_DRIVING = "DRIVING"
MODE_MISSION_STOP = "MISSION_STOP"


@dataclass
class MissionConfig:
    stop_speed_eps: float = 0.02    # 이 이하면 '정지'로 본다 [m/s]
    stop_settle_s: float = 0.3      # 이만큼 계속 정지해야 진짜 정지
    done_timeout_s: float = 15.0    # DONE 을 이만큼 기다린다
    max_retries: int = 2            # ArrivalStatus 재전송 횟수
    stop_timeout_s: float = 5.0     # 정차 자체가 이만큼 안 되면 이상 (브레이크 고장?)


@dataclass(frozen=True)
class Decision:
    """이번 틱에 무엇을 할지."""
    allow_drive: bool               # False = 구동 0 (상위가 /cmd_vel 을 0 으로)
    chassis_mode: str = MODE_DRIVING
    publish_arrival: tuple = None   # (mission_id, status) — 보낼 때만
    state: str = DRIVE
    reason: str = ""


class MissionSequencer:
    """미션 지점 도착 → 팔 핸드셰이크 → 재출발."""

    def __init__(self, cfg: MissionConfig = None):
        self.cfg = cfg or MissionConfig()
        self.state = DRIVE
        self.mission_id = None
        self.arrival_status = None
        self._t_state = 0.0             # 현재 상태 진입 시각
        self._t_stopped = None          # 정지가 시작된 시각
        self._t_arrival = None          # ArrivalStatus 를 마지막으로 보낸 시각
        self._retries = 0
        self._done_seen = False

    # ── 이벤트 ───────────────────────────────────────────────────────────

    def arrive(self, mission_id: int, status: str, t: float) -> bool:
        """미션 지점에 도착했다(상위 인지가 판단). 정차 시퀀스를 시작한다."""
        if self.state != DRIVE:
            return False                # 이미 미션 처리 중 — 무시
        self.mission_id = int(mission_id)
        self.arrival_status = status
        self._enter(STOPPING, t)
        self._t_stopped = None
        self._retries = 0
        self._done_seen = False
        return True

    def on_arm_status(self, mission_id: int, status: str, t: float) -> str:
        """팔 상태 수신. **유효한 DONE 만** 재출발 사유가 된다.

        반환: 사유 문자열(무시했으면 왜 무시했는지). 디버깅용.
        """
        if status != ARM_DONE:
            return f"DONE 아님({status}) — 무시"

        # ③ 재출발 게이트
        if self.state != WAIT_ARM:
            # ArrivalStatus 를 보내기 전에 온 DONE = 재생·잔류 메시지
            return f"WAIT_ARM 아님({self.state}) — 잔류/재생 DONE 무시"
        if int(mission_id) != self.mission_id:
            return (f"mission_id 불일치 (받은 {mission_id}, 기대 {self.mission_id}) "
                    "— 이전 미션의 뒤늦은 DONE 무시")
        if self._t_arrival is None or t < self._t_arrival:
            return "ArrivalStatus 이전의 DONE — 무시"

        self._done_seen = True
        return "유효한 DONE — 재출발"

    # ── 매 틱 ────────────────────────────────────────────────────────────

    def update(self, t: float, speed_mps: float) -> Decision:
        c = self.cfg

        if self.state == DRIVE:
            return Decision(True, MODE_DRIVING, state=DRIVE, reason="주행")

        if self.state == FAILED:
            # 정지 유지. 사람이 reset() 해야 풀린다.
            return Decision(False, MODE_MISSION_STOP, state=FAILED,
                            reason="DONE 미수신 — 사람 개입 필요 (재출발 금지)")

        if self.state == STOPPING:
            # ① MISSION_STOP 은 이미 발행 중(chassis_mode). 실제 정지를 확인한다.
            if abs(speed_mps) <= c.stop_speed_eps:
                if self._t_stopped is None:
                    self._t_stopped = t
                if t - self._t_stopped >= c.stop_settle_s:
                    # ② 정지 확인됨 → 이제서야 ArrivalStatus 를 보낸다
                    self._enter(WAIT_ARM, t)
                    self._t_arrival = t
                    return Decision(
                        False, MODE_MISSION_STOP,
                        publish_arrival=(self.mission_id, self.arrival_status),
                        state=WAIT_ARM, reason="정지 확인 → ArrivalStatus 발행")
            else:
                self._t_stopped = None      # 다시 움직였다 → 처음부터

            if t - self._t_state > c.stop_timeout_s:
                self._enter(FAILED, t)
                return Decision(False, MODE_MISSION_STOP, state=FAILED,
                                reason=f"정차 실패 (|v|={abs(speed_mps):.2f} m/s) — 브레이크 확인")
            return Decision(False, MODE_MISSION_STOP, state=STOPPING,
                            reason="정차 중 (아직 ArrivalStatus 안 보냄)")

        if self.state == WAIT_ARM:
            if self._done_seen:
                self._enter(DRIVE, t)
                self.mission_id = None
                self._done_seen = False
                return Decision(True, MODE_DRIVING, state=DRIVE,
                                reason="유효한 DONE → 재출발")

            if t - self._t_arrival >= c.done_timeout_s:
                if self._retries < c.max_retries:
                    # ArrivalStatus 가 유실됐을 수 있다 → 다시 보낸다
                    self._retries += 1
                    self._t_arrival = t
                    return Decision(
                        False, MODE_MISSION_STOP,
                        publish_arrival=(self.mission_id, self.arrival_status),
                        state=WAIT_ARM,
                        reason=f"DONE 타임아웃 → ArrivalStatus 재전송 "
                               f"({self._retries}/{c.max_retries})")
                # ⚠️ 타임아웃은 **재출발 사유가 아니다.** 팔이 뻗어 있을 수 있다.
                self._enter(FAILED, t)
                return Decision(False, MODE_MISSION_STOP, state=FAILED,
                                reason="DONE 미수신 (재시도 소진) — 정지 유지, 사람 개입")

            return Decision(False, MODE_MISSION_STOP, state=WAIT_ARM,
                            reason=f"팔 작업 대기 ({t - self._t_arrival:.1f}s)")

        return Decision(False, MODE_MISSION_STOP, state=self.state, reason="알 수 없는 상태")

    # ── 기타 ─────────────────────────────────────────────────────────────

    def reset(self, t: float = 0.0):
        """FAILED 를 풀고 주행 상태로. **사람이 상황을 확인한 뒤** 부른다."""
        self._enter(DRIVE, t)
        self.mission_id = None
        self._done_seen = False
        self._retries = 0

    def _enter(self, state: str, t: float):
        self.state = state
        self._t_state = t


# ── WP5.2 contract-v2 supervisor ───────────────────────────────────────────

READY = "READY"
STOP_REQUESTED = "STOP_REQUESTED"
EVENT_HOLD = "EVENT_HOLD"
ARM_WORK = "ARM_WORK"
STOW_VERIFY = "STOW_VERIFY"
RESUME = "RESUME"
COMPLETE = "COMPLETE"
FAILED_HOLD = "FAILED_HOLD"
GRIP_LOST_HOLD = "GRIP_LOST_HOLD"
SUPERVISOR_STATES = (
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
)

PICKUP = "PICKUP"
DROP = "DROP"
ARRIVED_PICKUP = "ARRIVED_PICKUP"
ARRIVED_DROP = "ARRIVED_DROP"
ARM_WORK_READY = "WORK_READY"
ARM_PERCEIVING = "PERCEIVING"
ARM_PLANNING = "PLANNING"
ARM_EXECUTING = "EXECUTING"
ARM_STOWED_LOCKED = "STOWED_LOCKED"
ARM_CARRYING_LOCKED = "CARRYING_LOCKED"
ARM_GRIP_LOST = "GRIP_LOST"
MODE_STOW_REQUEST = "STOW_REQUEST"
WORK_ACCEPTED_STATUSES = {
    ARM_WORK_READY,
    ARM_PERCEIVING,
    ARM_PLANNING,
    ARM_EXECUTING,
}
DIAGNOSTIC_FAILURES = {
    "IK_FAILURE",
    "TRAJECTORY_FAILURE",
    "SELF_COLLISION",
    "BASE_COLLISION",
    "JOINT_OVERCURRENT",
    "GRIP_UNCERTAIN",
    "STOW_FAILURE",
    "ACTION_TIMEOUT",
}


@dataclass(frozen=True)
class SupervisorConfig:
    arrival_period_s: float = 0.5
    arrival_window_s: float = 2.0
    arm_status_timeout_s: float = 0.5


@dataclass(frozen=True)
class SupervisorResult:
    state: str
    accepted: bool = False
    allow_drive: bool = False
    mode_intent: str = MODE_STOW_REQUEST
    publish_arrival: tuple = None
    hold_reason: str = ""
    operator_notice: str = ""


@dataclass(frozen=True)
class FailureRecord:
    wire_status: str
    mission_id: int
    stamp_s: float
    last_locked_posture: str
    operation: str
    arm_latched: bool


class MissionSupervisor:
    """Contract-v2 mission FSM owned by ``chassis_node``.

    All public transitions return a value object instead of raising.  ROS
    publication and chassis-mode mutation stay in the owning adapter.
    """

    def __init__(
        self,
        mission_id_store=None,
        *,
        wheel_stop=None,
        authority_output_zero=None,
        clear_grip_lost=None,
        cfg=None,
    ):
        self.cfg = cfg or SupervisorConfig()
        self.mission_id_store = mission_id_store or MissionIdStore()
        self.wheel_stop = wheel_stop
        self.authority_output_zero = authority_output_zero
        self.clear_grip_lost_callback = clear_grip_lost
        self.state = READY
        self.mode_intent = MODE_STOW_REQUEST
        self.active_mission_id = None
        self.operation = None
        self.arrival_status = None
        self.arrival_republish_active = False
        self.operator_notice = ""
        self.failure = None
        self.last_completed_mission_id = None
        self.work_start_count = 0
        self.diagnostic_events = []
        self._last_locked_posture = None
        self._last_locked_mission_id = None
        self._last_locked_stamp_s = None
        self._last_arm_stamp_s = None
        self._arrival_started_s = None
        self._last_arrival_publish_s = None
        self._arrival_timed_out = False
        self._grip_lost_stamp_s = None

    def request_work(self, arrival_status, now_s):
        now_s = self._time(now_s)
        if now_s is None:
            return self._enter_hold("invalid_request_time")
        if self.state not in (READY, DRIVE):
            return self._result(hold_reason="mission_busy:%s" % self.state)

        operation = {
            ARRIVED_PICKUP: PICKUP,
            ARRIVED_DROP: DROP,
        }.get(str(arrival_status))
        if operation is None:
            return self._result(hold_reason="unsupported_arrival_status")

        try:
            qualified = bool(self.wheel_stop is not None and self.wheel_stop.qualified)
        except Exception as exc:
            return self._enter_hold(
                "wheel_stop_qualification_exception:%s" % type(exc).__name__
            )
        if not qualified:
            return self._enter_hold("wheel_stop_unqualified")

        expected = (
            ARM_STOWED_LOCKED if operation == PICKUP else ARM_CARRYING_LOCKED
        )
        if not self._locked_is_fresh(expected, now_s):
            return self._result(
                hold_reason="fresh_%s_required" % expected.lower()
            )

        self.operation = operation
        self.arrival_status = str(arrival_status)
        self.active_mission_id = None
        self.failure = None
        self.operator_notice = ""
        self.arrival_republish_active = False
        self._arrival_timed_out = False
        self._arrival_started_s = None
        self._last_arrival_publish_s = None
        self.mode_intent = MODE_STOW_REQUEST
        self.state = STOP_REQUESTED
        return self._result(accepted=True)

    def tick(self, now_s):
        now_s = self._time(now_s)
        if now_s is None:
            return self._enter_hold("invalid_tick_time")

        if self.state == READY:
            if self._locked_is_fresh(None, now_s):
                self.state = DRIVE
                self.mode_intent = MODE_DRIVING
                return self._result(allow_drive=True)
            self.mode_intent = MODE_STOW_REQUEST
            return self._result(hold_reason="fresh_locked_posture_required")

        if self.state == DRIVE:
            self.mode_intent = MODE_DRIVING
            return self._result(allow_drive=True)

        if self.state == STOP_REQUESTED:
            return self._tick_stop_requested(now_s)

        if self.state == EVENT_HOLD:
            return self._tick_event_hold(now_s)

        if self.state == ARM_WORK:
            self.mode_intent = MODE_MISSION_STOP
            return self._result(hold_reason="arm_work_active")

        if self.state == STOW_VERIFY:
            expected = self._success_posture()
            if self._locked_is_fresh(
                expected,
                now_s,
                mission_id=self.active_mission_id,
            ):
                self.last_completed_mission_id = self.active_mission_id
                self.state = RESUME
                self.mode_intent = MODE_DRIVING
                return self._result(allow_drive=True)
            self.mode_intent = MODE_MISSION_STOP
            return self._result(hold_reason="fresh_success_lock_required")

        if self.state == RESUME:
            self.state = COMPLETE
            self.mode_intent = MODE_DRIVING
            return self._result(allow_drive=True)

        if self.state == COMPLETE:
            self.state = DRIVE
            self.mode_intent = MODE_DRIVING
            self.active_mission_id = None
            self.operation = None
            self.arrival_status = None
            self.failure = None
            return self._result(allow_drive=True)

        if self.state in (FAILED_HOLD, GRIP_LOST_HOLD):
            self.mode_intent = MODE_STOW_REQUEST
            return self._result(
                hold_reason=self.operator_notice or self.state.lower(),
                operator_notice=self.operator_notice,
            )

        return self._enter_hold("unknown_state:%s" % self.state)

    def on_arm_status(self, status, mission_id, stamp_s, now_s):
        status = str(status)
        try:
            mission_id = int(mission_id)
        except (TypeError, ValueError):
            return self._result(hold_reason="invalid_arm_mission_id")
        stamp_s = self._time(stamp_s)
        now_s = self._time(now_s)
        if stamp_s is None or now_s is None:
            return self._result(hold_reason="invalid_arm_stamp")

        if status == ARM_DONE:
            self.diagnostic_events.append((status, mission_id, stamp_s))
            return self._result(hold_reason="done_diagnostic_only")

        if not self._sample_fresh(stamp_s, now_s):
            return self._result(hold_reason="arm_status_not_fresh")
        if self._last_arm_stamp_s is not None and stamp_s < self._last_arm_stamp_s:
            return self._result(hold_reason="arm_status_stamp_regression")
        self._last_arm_stamp_s = stamp_s

        if status in (ARM_STOWED_LOCKED, ARM_CARRYING_LOCKED):
            self._last_locked_posture = status
            self._last_locked_mission_id = mission_id
            self._last_locked_stamp_s = stamp_s

        if status == ARM_GRIP_LOST:
            if (
                self.active_mission_id is not None
                and mission_id != self.active_mission_id
            ):
                return self._result(
                    hold_reason="arm_status_previous_mission_ignored"
                )
            if self.active_mission_id is None:
                # A pickup mission is complete before the carrying DRIVE
                # segment starts, but loss of that payload still belongs to
                # the completed pickup ID and must enter the same latch.
                self.active_mission_id = mission_id
                if self.operation is None:
                    self.operation = PICKUP
                    self.arrival_status = ARRIVED_PICKUP
            self._grip_lost_stamp_s = stamp_s
            self.arrival_republish_active = False
            self.mode_intent = MODE_STOW_REQUEST
            self.operator_notice = "grip_lost_operator_action_required"
            self.failure = self._failure_record(status, mission_id, stamp_s)
            self.state = GRIP_LOST_HOLD
            return self._result(
                hold_reason="grip_lost_latched",
                operator_notice=self.operator_notice,
            )

        if self.active_mission_id is None:
            return self._result()
        if mission_id != self.active_mission_id:
            return self._result(hold_reason="arm_status_previous_mission_ignored")

        if status == ARM_FAILED or status in DIAGNOSTIC_FAILURES:
            return self._enter_failure(status, mission_id, stamp_s)

        if (
            self.state == EVENT_HOLD
            and self.arrival_republish_active
            and status in WORK_ACCEPTED_STATUSES
        ):
            self.arrival_republish_active = False
            self.state = ARM_WORK
            self.work_start_count += 1
            self.mode_intent = MODE_MISSION_STOP
            return self._result(accepted=True)

        if self.state == ARM_WORK and status == self._success_posture():
            self.state = STOW_VERIFY
            self.mode_intent = MODE_MISSION_STOP
            return self._result(accepted=True)

        # Locked heartbeats in a failure/latch state are evidence only.  They
        # never perform an implicit transition.
        return self._result()

    def resolve_failure(self, action, now_s):
        now_s = self._time(now_s)
        if now_s is None:
            return self._result(hold_reason="invalid_resolution_time")
        if self.state != FAILED_HOLD or self.failure is None:
            return self._result(hold_reason="failed_hold_required")
        if action not in ("skip", "retry"):
            return self._result(hold_reason="operator_skip_or_retry_required")

        expected = self._failure_posture()
        if not self._locked_is_fresh(
            expected,
            now_s,
            mission_id=self.active_mission_id,
            newer_than=self.failure.stamp_s,
        ):
            return self._result(
                hold_reason="fresh_failure_%s_required" % expected.lower()
            )

        if action == "retry":
            self.active_mission_id = None
            self.failure = None
            self.operator_notice = ""
            self.arrival_republish_active = False
            self._arrival_timed_out = False
            self.state = STOP_REQUESTED
            self.mode_intent = MODE_STOW_REQUEST
            return self._result(accepted=True)

        self.last_completed_mission_id = self.active_mission_id
        self.operator_notice = ""
        self.state = COMPLETE
        self.mode_intent = MODE_STOW_REQUEST
        return self._result(accepted=True)

    def confirm_regrasp(self, now_s):
        now_s = self._time(now_s)
        if now_s is None or self.state != GRIP_LOST_HOLD:
            return self._result(hold_reason="grip_lost_hold_required")
        if not self._locked_is_fresh(
            ARM_CARRYING_LOCKED,
            now_s,
            mission_id=self.active_mission_id,
            newer_than=self._grip_lost_stamp_s,
        ):
            return self._result(hold_reason="fresh_regrasp_lock_required")
        cleared, reason = self._clear_grip_interlock()
        if not cleared:
            return self._result(hold_reason=reason)

        self.operator_notice = ""
        self.failure = None
        if self.operation == PICKUP:
            self.state = STOW_VERIFY
        else:
            self.state = ARM_WORK
        self.mode_intent = MODE_MISSION_STOP
        return self._result(accepted=True)

    def operator_clear_grip_lost(self, authorized, now_s):
        now_s = self._time(now_s)
        if now_s is None or self.state != GRIP_LOST_HOLD:
            return self._result(hold_reason="grip_lost_hold_required")
        if not authorized:
            return self._result(hold_reason="operator_authorization_required")
        expected = self._failure_posture()
        if not self._locked_is_fresh(
            expected,
            now_s,
            mission_id=self.active_mission_id,
            newer_than=self._grip_lost_stamp_s,
        ):
            return self._result(
                hold_reason="fresh_operator_%s_required" % expected.lower()
            )
        cleared, reason = self._clear_grip_interlock()
        if not cleared:
            return self._result(hold_reason=reason)

        grip_failure = self.failure
        self.state = FAILED_HOLD
        self.mode_intent = MODE_STOW_REQUEST
        self.operator_notice = "grip_lost_cleared_skip_or_retry_required"
        self.failure = FailureRecord(
            wire_status=grip_failure.wire_status,
            mission_id=grip_failure.mission_id,
            stamp_s=grip_failure.stamp_s,
            last_locked_posture=grip_failure.last_locked_posture,
            operation=grip_failure.operation,
            arm_latched=False,
        )
        return self._result(accepted=True, operator_notice=self.operator_notice)

    def abort_for_override(self, now_s=0.0):
        now_s = self._time(now_s)
        if now_s is None:
            return False
        self.arrival_republish_active = False
        self._arrival_timed_out = True
        self.mode_intent = MODE_STOW_REQUEST
        self.operator_notice = "override_abort_skip_or_retry_required"
        pending = self.operation is not None and self.state not in (
            READY,
            DRIVE,
            COMPLETE,
        )
        if pending:
            self.failure = self._failure_record(
                "ABORT_OVERRIDE",
                self.active_mission_id or 0,
                now_s,
            )
            self.state = FAILED_HOLD
        return self.mode_intent == MODE_STOW_REQUEST and not self.arrival_republish_active

    def _tick_stop_requested(self, now_s):
        try:
            qualified = bool(self.wheel_stop is not None and self.wheel_stop.qualified)
        except Exception as exc:
            return self._enter_hold(
                "wheel_stop_qualification_exception:%s" % type(exc).__name__
            )
        if not qualified:
            return self._enter_hold("wheel_stop_unqualified")

        try:
            authority_zero = bool(
                self.authority_output_zero is not None
                and self.authority_output_zero()
            )
        except Exception as exc:
            return self._enter_hold(
                "authority_output_exception:%s" % type(exc).__name__
            )
        if not authority_zero:
            self.mode_intent = MODE_STOW_REQUEST
            return self._result(hold_reason="authority_output_nonzero")

        try:
            stopped = bool(self.wheel_stop.confirmed)
        except Exception as exc:
            return self._enter_hold(
                "wheel_stop_exception:%s" % type(exc).__name__
            )
        if not stopped:
            detail = getattr(self.wheel_stop, "last_reject_reason", "pending")
            self.mode_intent = MODE_STOW_REQUEST
            return self._result(hold_reason="wheel_stop_pending:%s" % detail)

        try:
            allocation = self.mission_id_store.allocate()
        except Exception as exc:
            return self._enter_hold(
                "mission_id_store_exception:%s" % type(exc).__name__
            )
        if not getattr(allocation, "accepted", False):
            return self._enter_hold(
                getattr(allocation, "hold_reason", "mission_id_store_rejected")
            )

        mission_id = getattr(allocation, "mission_id", None)
        if (
            isinstance(mission_id, bool)
            or not isinstance(mission_id, int)
            or not 0 < mission_id <= 2_147_483_647
        ):
            return self._enter_hold("mission_id_store_invalid_result")

        self.active_mission_id = mission_id
        self.state = EVENT_HOLD
        self.mode_intent = MODE_MISSION_STOP
        self.arrival_republish_active = True
        self._arrival_timed_out = False
        self._arrival_started_s = now_s
        self._last_arrival_publish_s = now_s
        return self._result(
            accepted=True,
            publish_arrival=(mission_id, self.arrival_status),
        )

    def _tick_event_hold(self, now_s):
        self.mode_intent = MODE_MISSION_STOP
        if not self.arrival_republish_active:
            return self._result(
                hold_reason=(
                    "arrival_ack_timeout" if self._arrival_timed_out else "event_hold"
                ),
                operator_notice=self.operator_notice,
            )

        elapsed = now_s - self._arrival_started_s
        if elapsed < 0.0:
            self.arrival_republish_active = False
            self._arrival_timed_out = True
            self.operator_notice = "operator_required:arrival_clock_rollback"
            return self._result(
                hold_reason="arrival_clock_rollback",
                operator_notice=self.operator_notice,
            )
        if elapsed + 1e-12 >= self.cfg.arrival_window_s:
            self.arrival_republish_active = False
            self._arrival_timed_out = True
            self.operator_notice = "operator_required:arrival_ack_timeout"
            return self._result(
                hold_reason="arrival_ack_timeout",
                operator_notice=self.operator_notice,
            )
        if now_s - self._last_arrival_publish_s + 1e-12 >= self.cfg.arrival_period_s:
            self._last_arrival_publish_s = now_s
            return self._result(
                publish_arrival=(self.active_mission_id, self.arrival_status)
            )
        return self._result(hold_reason="arrival_ack_pending")

    def _enter_failure(self, status, mission_id, stamp_s):
        self.arrival_republish_active = False
        self._arrival_timed_out = True
        self.mode_intent = MODE_STOW_REQUEST
        self.failure = self._failure_record(status, mission_id, stamp_s)
        self.operator_notice = "arm_failure_skip_or_retry_required"
        self.state = FAILED_HOLD
        return self._result(
            hold_reason="arm_failure:%s" % status,
            operator_notice=self.operator_notice,
        )

    def _enter_hold(self, reason):
        self.arrival_republish_active = False
        self.mode_intent = MODE_STOW_REQUEST
        self.operator_notice = "operator_required:%s" % reason
        self.state = EVENT_HOLD
        return self._result(
            hold_reason=reason,
            operator_notice=self.operator_notice,
        )

    def _failure_record(self, status, mission_id, stamp_s):
        return FailureRecord(
            wire_status=str(status),
            mission_id=int(mission_id),
            stamp_s=float(stamp_s),
            last_locked_posture=self._last_locked_posture or "",
            operation=self.operation or "",
            arm_latched=True,
        )

    def _clear_grip_interlock(self):
        if self.clear_grip_lost_callback is None:
            return False, "interlock_clear_unavailable"
        try:
            cleared = bool(self.clear_grip_lost_callback(authorized=True))
        except Exception as exc:
            return (
                False,
                "interlock_clear_exception:%s" % type(exc).__name__,
            )
        if not cleared:
            return False, "interlock_clear_rejected"
        return True, ""

    def _success_posture(self):
        return (
            ARM_CARRYING_LOCKED
            if self.operation == PICKUP
            else ARM_STOWED_LOCKED
        )

    def _failure_posture(self):
        return (
            ARM_STOWED_LOCKED
            if self.operation == PICKUP
            else ARM_CARRYING_LOCKED
        )

    def _locked_is_fresh(
        self,
        expected,
        now_s,
        *,
        mission_id=None,
        newer_than=None,
    ):
        if self._last_locked_stamp_s is None:
            return False
        if expected is not None and self._last_locked_posture != expected:
            return False
        if mission_id is not None and self._last_locked_mission_id != mission_id:
            return False
        if newer_than is not None and self._last_locked_stamp_s <= newer_than:
            return False
        return self._sample_fresh(self._last_locked_stamp_s, now_s)

    def _sample_fresh(self, stamp_s, now_s):
        age_s = now_s - stamp_s
        return 0.0 <= age_s <= self.cfg.arm_status_timeout_s

    @staticmethod
    def _time(value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return value

    def _result(self, **overrides):
        values = {
            "state": self.state,
            "accepted": False,
            "allow_drive": False,
            "mode_intent": self.mode_intent,
            "publish_arrival": None,
            "hold_reason": "",
            "operator_notice": "",
        }
        values.update(overrides)
        return SupervisorResult(**values)
