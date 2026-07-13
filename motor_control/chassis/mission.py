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
