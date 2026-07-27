# 비전 자동 도착 판정 + 능동 정렬 접근 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 파워트레인이 픽업/드롭 대상을 스스로 감지·정렬 접근하고 정렬 완료 시 기존 MissionSupervisor에 자동으로 도착을 통보(`ARRIVED_PICKUP/DROP`)하는 능동 최종접근 파이프라인을 만든다.

**Architecture:** 신규 순수 코어 `approach.py`(정렬 상태기계 + 제어법, ROS 무관)와 이를 감싸는 `approach_controller_node`(behavior 노드 1개)를 추가한다. 노드는 `/detected_objects`를 TF로 base_link에 투영해 폐루프 크립을 `/autonomy/cmd_vel`로 제안하고, 정렬 완료 시 chassis_node의 기존 `~/mission_arrive_pickup/_drop` 서비스를 자동 호출한다. 대상 감지·디바운스·쿨다운은 기존 `MissionTrigger`를 재사용하고, 하류(MISSION_STOP→ArrivalStatus→핸드셰이크)는 무변경으로 재사용한다. `lane_follower_node`는 `/approach/active`를 보고 접근 중 제안을 양보한다.

**Tech Stack:** Python 3.10, ROS 2 Humble (rclpy), `robot_arm_msgs`(DetectedObjectArray/ArmStatus/ArrivalStatus), `geometry_msgs/Twist`, `std_msgs/Bool·String`, `std_srvs/Trigger`, `tf2_ros`, pytest.

## Global Constraints

- 순수 코어(`motor_control/chassis/approach.py`)는 **하드웨어·ROS 의존 0** — `import rclpy` 금지. 계산만.
- 접근 노드는 `/cmd_vel`을 직접 쓰지 않는다. `/autonomy/cmd_vel`로 **제안**만 하고 authority/hold_source를 소유하지 않는다.
- 인식은 로봇팔 팀 단일 소스 — `/detected_objects`를 구독만 한다. 자체 카메라 인식 금지.
- `DetectedObject.pose` 프레임이 TF로 base_link에 해석되지 않으면 **APPROACHING 진입 거부**(SEARCHING 유지 + 경고) — 엉뚱한 거리 크립 금지.
- ARRIVED 서비스 호출은 **실제 정지 확인(`v_settle`) 후에만**. 한 프레임 깜빡임 급정거 금지.
- base_link 좌표 규약(REP-103): x=전방, y=좌(+). pose 투영 시 x=전방거리, y=횡오차.
- 테스트: 순수 pytest + 젯슨 rclpy 스모크(도메인77 격리) + E2E(모터 무전원) 모두 필수. 순수 스위트 green + diff 리뷰만으로 완료 선언 금지.
- 실제 구현은 Codex 위임, 검증은 Claude(diff+테스트 재실행+E2E+음성대조).

---

### Task 1: 순수 코어 스캐폴딩 — 설정·데이터·제어법

**Files:**
- Create: `motor_control/chassis/approach.py`
- Test: `motor_control/chassis/tests/test_approach.py`

**Interfaces:**
- Consumes: 없음(신규).
- Produces:
  - 상태 상수 `SEARCHING, APPROACHING, ALIGNED, ARRIVED_FIRED, DONE, BACKOFF, FAILED_HOLD` (str).
  - `ApproachConfig` (dataclass, 아래 필드).
  - `Target(class_name: str, confidence: float, x: float, y: float)` (dataclass).
  - `ApproachDecision(state: str, active: bool, v: float, omega: float, fire: str|None, reason: str, retries: int)` (dataclass).
  - `creep_cmd(cfg: ApproachConfig, x: float, y: float) -> tuple[float, float]` — `(v, omega)` 클램프 계산.

- [ ] **Step 1: Write the failing test**

```python
# motor_control/chassis/tests/test_approach.py
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd motor_control && python -m pytest chassis/tests/test_approach.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'chassis.approach'`

- [ ] **Step 3: Write minimal implementation**

```python
# motor_control/chassis/approach.py
"""능동 최종접근 정렬 — 대상 pose(base_link)를 보고 폐루프로 크립·정렬한다.

    /detected_objects(팔 YOLO, TF→base_link) → [이것] → /autonomy/cmd_vel + ARRIVED 서비스

하드웨어·ROS 의존 없음. 인식은 로봇팔 팀 단일 소스, 우리는 결과만 받는다.
설계: docs/superpowers/specs/2026-07-27-vision-arrival-approach-design.md
"""
from dataclasses import dataclass

from chassis.mission_trigger import MissionTrigger, TriggerConfig, TriggerRule

# 계약 어휘 (contract.py와 동일 문자열; 순수 코어라 하드코딩)
ARRIVED_PICKUP = "ARRIVED_PICKUP"
ARRIVED_DROP = "ARRIVED_DROP"

# 상태
SEARCHING = "SEARCHING"
APPROACHING = "APPROACHING"
ALIGNED = "ALIGNED"
ARRIVED_FIRED = "ARRIVED_FIRED"
DONE = "DONE"
BACKOFF = "BACKOFF"
FAILED_HOLD = "FAILED_HOLD"


@dataclass
class ApproachConfig:
    engage_m: float = 2.0          # 이 안에 들어오면 lock(접근 시작)
    stop_m: float = 1.0            # 이 거리에서 정지(팔 리치, HIL 캘리브)
    lat_tol: float = 0.05          # 횡오차 허용(정렬 판정)
    dist_tol: float = 0.05         # 거리오차 허용(정렬 판정)
    k_yaw: float = 1.5             # 횡오차 → omega 이득
    k_dist: float = 0.8            # 거리오차 → v 이득
    omega_max: float = 0.4         # rad/s
    v_approach_max: float = 0.15   # m/s (저속 정밀 크립)
    v_settle: float = 0.03         # m/s 미만이면 "정지"로 인정
    backoff_creep: float = 0.10    # m/s (뒤로 물러나는 속도)
    backoff_time_s: float = 1.0
    align_timeout_s: float = 8.0
    max_retries: int = 2
    consecutive: int = 5           # lock 디바운스 프레임
    cooldown_s: float = 10.0       # 미션 완료 후 같은 클래스 무시
    min_confidence: float = 0.6
    lost_frames: int = 5           # 연속 이만큼 못 보면 lost
    pose_stale_s: float = 0.5      # 노드가 freshness 게이트에 참고
    pickup_class: str = "box"
    drop_class: str = "dropzone"


@dataclass
class Target:
    """base_link로 투영된 검출 1개."""
    class_name: str
    confidence: float
    x: float                       # 전방거리 m
    y: float                       # 횡오차 m (+좌)


@dataclass
class ApproachDecision:
    state: str
    active: bool                   # /approach/active
    v: float                       # /autonomy/cmd_vel linear.x 제안
    omega: float                   # angular.z 제안
    fire: str = None               # ARRIVED_* (서비스 호출) or None
    reason: str = ""
    retries: int = 0


def creep_cmd(cfg: ApproachConfig, x: float, y: float) -> tuple:
    """대상 상대위치(x 전방, y 횡) → (v, omega). 후진 없음, 전부 클램프."""
    omega = max(-cfg.omega_max, min(cfg.omega_max, -cfg.k_yaw * y))
    v = cfg.k_dist * (x - cfg.stop_m)
    v = max(0.0, min(cfg.v_approach_max, v))
    return v, omega
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd motor_control && python -m pytest chassis/tests/test_approach.py -v`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add motor_control/chassis/approach.py motor_control/chassis/tests/test_approach.py
git commit -m "feat(approach): pure-core config, target, decision, creep control law"
```

---

### Task 2: ApproachController 상태기계 — 감지→접근→정렬 + lost/타임아웃/재시도

**Files:**
- Modify: `motor_control/chassis/approach.py` (append `ApproachController` class)
- Test: `motor_control/chassis/tests/test_approach.py` (append)

**Interfaces:**
- Consumes: Task 1의 `ApproachConfig`, `Target`, `ApproachDecision`, `creep_cmd`, 상태 상수; `MissionTrigger`.
- Produces:
  - `ApproachController(cfg: ApproachConfig=None, trigger: MissionTrigger=None)`.
  - `.update(targets: list[Target], speed_mps: float, now_s: float) -> ApproachDecision` — 매 tick 호출.
  - 속성 `.state: str`, `.retries: int`, `.active_status: str|None`(ARRIVED_PICKUP/DROP), `.active_class: str|None`.

**동작 규칙:**
- `SEARCHING`: `MissionTrigger.on_detections`로 lock 판정(디바운스+쿨다운+engage_m 거리). lock 시 APPROACHING, active=True, `active_status`/`active_class` 저장, `enter_s=now`.
- `APPROACHING`: 매칭 대상(active_class) 찾음 → `creep_cmd`로 v,omega 제안. 정렬 판정 `|y|<lat_tol AND |x-stop_m|<dist_tol AND speed<v_settle` → ALIGNED(active=True, v=0, omega=0, fire=active_status). 매칭 대상 없음(연속 `lost_frames`) → BACKOFF. `now-enter_s > align_timeout_s` → 재시도++: 재시도<max_retries면 BACKOFF, 아니면 FAILED_HOLD.
- `BACKOFF`: v=-backoff_creep, omega=0, active=True. `backoff_time_s` 후 대상 재획득되면 APPROACHING(enter_s 갱신), 아니면 SEARCHING 복귀(단 재시도 소진 시 FAILED_HOLD 유지).
- `ALIGNED`: v=0, omega=0, active=True, fire=active_status를 **한 번만** 방출(노드가 서비스 호출). 다음 tick부터 fire=None(중복 호출 금지) — `on_service_ack` 대기.
- `FAILED_HOLD`: v=0, omega=0, active=True, reason="align_failed". 정지 유지, 노드가 콘솔 경고. `reset()`까지 유지.

- [ ] **Step 1: Write the failing test**

```python
# append to test_approach.py
from chassis.approach import ApproachController


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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd motor_control && python -m pytest chassis/tests/test_approach.py -v -k "lock or aligned or moving or timeout or lost"`
Expected: FAIL with `ImportError` / `AttributeError: ApproachController`

- [ ] **Step 3: Write minimal implementation**

```python
# append to approach.py

class ApproachController:
    """감지→능동 접근→정렬→발사. 상태와 카운터만 보유(순수)."""

    def __init__(self, cfg: ApproachConfig = None, trigger: MissionTrigger = None):
        self.cfg = cfg or ApproachConfig()
        c = self.cfg
        self.trigger = trigger or MissionTrigger(TriggerConfig(
            rules=[
                TriggerRule(c.pickup_class, ARRIVED_PICKUP, c.engage_m, c.min_confidence),
                TriggerRule(c.drop_class, ARRIVED_DROP, c.engage_m, c.min_confidence),
            ],
            consecutive=c.consecutive,
            cooldown_s=c.cooldown_s,
        ))
        self.state = SEARCHING
        self.retries = 0
        self.active_status = None       # ARRIVED_PICKUP / ARRIVED_DROP
        self.active_class = None
        self._enter_s = 0.0
        self._backoff_until = 0.0
        self._lost = 0
        self._fired = False

    def _match(self, targets):
        """active_class에 맞는 가장 가까운 유효 대상."""
        best = None
        for t in targets:
            if t.class_name != self.active_class:
                continue
            if t.confidence < self.cfg.min_confidence:
                continue
            if t.x <= 0.0:
                continue
            if best is None or t.x < best.x:
                best = t
        return best

    def _result(self, state, active, v, omega, fire=None, reason=""):
        self.state = state
        return ApproachDecision(state, active, v, omega, fire, reason, self.retries)

    def update(self, targets, speed_mps, now_s) -> ApproachDecision:
        c = self.cfg
        if self.state in (SEARCHING,):
            dets = [(t.class_name, t.confidence, t.x) for t in targets]
            hit = self.trigger.on_detections(dets, now_s)
            if hit is None:
                return self._result(SEARCHING, False, 0.0, 0.0, reason="searching")
            status, cls = hit
            self.active_status, self.active_class = status, cls
            self.retries = 0
            self._fired = False
            self._enter_s = now_s
            self._lost = 0
            # 즉시 첫 크립
            return self._enter_approaching(targets, speed_mps, now_s)

        if self.state == APPROACHING:
            return self._tick_approaching(targets, speed_mps, now_s)

        if self.state == BACKOFF:
            m = self._match(targets)
            if now_s >= self._backoff_until:
                if m is not None:
                    self._enter_s = now_s
                    self._lost = 0
                    return self._enter_approaching(targets, speed_mps, now_s)
                # 재획득 실패 → 다시 SEARCHING (쿨다운/디바운스 재적용)
                self.active_status = self.active_class = None
                return self._result(SEARCHING, False, 0.0, 0.0, reason="backoff_gaveup")
            return self._result(BACKOFF, True, -c.backoff_creep, 0.0, reason="backoff")

        if self.state == ALIGNED:
            # 발사는 이미 방출됨 → 서비스 ACK 대기(노드가 on_service_ack 호출)
            return self._result(ALIGNED, True, 0.0, 0.0, reason="await_ack")

        if self.state == ARRIVED_FIRED:
            return self._result(ARRIVED_FIRED, True, 0.0, 0.0, reason="arm_working")

        if self.state == FAILED_HOLD:
            return self._result(FAILED_HOLD, True, 0.0, 0.0, reason="align_failed")

        if self.state == DONE:
            return self._result(SEARCHING, False, 0.0, 0.0, reason="resumed")

        return self._result(self.state, False, 0.0, 0.0, reason="unknown")

    def _enter_approaching(self, targets, speed_mps, now_s):
        self.state = APPROACHING
        return self._tick_approaching(targets, speed_mps, now_s)

    def _tick_approaching(self, targets, speed_mps, now_s):
        c = self.cfg
        m = self._match(targets)
        if m is None:
            self._lost += 1
            if self._lost >= c.lost_frames:
                self._backoff_until = now_s + c.backoff_time_s
                return self._result(BACKOFF, True, -c.backoff_creep, 0.0, reason="lost")
            return self._result(APPROACHING, True, 0.0, 0.0, reason="lost_grace")
        self._lost = 0
        # 정렬 판정
        aligned = (abs(m.y) < c.lat_tol
                   and abs(m.x - c.stop_m) < c.dist_tol
                   and abs(speed_mps) < c.v_settle)
        if aligned and not self._fired:
            self._fired = True
            return self._result(ALIGNED, True, 0.0, 0.0,
                                fire=self.active_status, reason="aligned")
        # 타임아웃 → 재시도/실패
        if now_s - self._enter_s > c.align_timeout_s:
            self.retries += 1
            if self.retries > c.max_retries:
                return self._result(FAILED_HOLD, True, 0.0, 0.0, reason="align_failed")
            self._backoff_until = now_s + c.backoff_time_s
            return self._result(BACKOFF, True, -c.backoff_creep, 0.0, reason="retry")
        v, omega = creep_cmd(c, m.x, m.y)
        return self._result(APPROACHING, True, v, omega, reason="approaching")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd motor_control && python -m pytest chassis/tests/test_approach.py -v`
Expected: 모든 테스트 passed (Task 1의 3 + Task 2의 5)

- [ ] **Step 5: Commit**

```bash
git add motor_control/chassis/approach.py motor_control/chassis/tests/test_approach.py
git commit -m "feat(approach): ApproachController FSM — lock, closed-loop approach, align, lost/timeout/retry"
```

---

### Task 3: 발사 ACK·팔 DONE·쿨다운·리셋

**Files:**
- Modify: `motor_control/chassis/approach.py` (append methods to `ApproachController`)
- Test: `motor_control/chassis/tests/test_approach.py` (append)

**Interfaces:**
- Consumes: Task 2의 `ApproachController` 내부 상태.
- Produces:
  - `.on_service_ack(success: bool, now_s: float) -> None` — ALIGNED에서 서비스 응답 반영. success → ARRIVED_FIRED. 실패 → 재시도(BACKOFF) 또는 FAILED_HOLD.
  - `.on_mission_done(now_s: float) -> None` — 팔 DONE 관측 시 ARRIVED_FIRED→DONE, 쿨다운 설정(`MissionTrigger.mission_finished`), active_class/status 정리.
  - `.reset(now_s: float) -> None` — 어느 상태에서든 SEARCHING으로. 운용자 개입 후.

- [ ] **Step 1: Write the failing test**

```python
# append to test_approach.py

def test_service_ack_success_to_arrived_fired():
    cfg = ApproachConfig(stop_m=1.0, v_settle=0.03, consecutive=1,
                         lat_tol=0.05, dist_tol=0.05)
    ctl = ApproachController(cfg)
    ctl.update(_targets(x=1.8), 0.0, 0.0)
    d = ctl.update(_targets(x=1.0, y=0.0), 0.0, 0.1)
    assert d.fire == ARRIVED_PICKUP
    ctl.on_service_ack(True, 0.2)
    assert ctl.state == ARRIVED_FIRED
    # 팔 작업 중엔 계속 정지 + active
    d = ctl.update(_targets(x=1.0, y=0.0), 0.0, 0.3)
    assert d.state == ARRIVED_FIRED
    assert d.v == 0.0 and d.active is True


def test_service_ack_reject_retries():
    cfg = ApproachConfig(stop_m=1.0, v_settle=0.03, consecutive=1,
                         lat_tol=0.05, dist_tol=0.05, max_retries=1,
                         backoff_time_s=0.2)
    ctl = ApproachController(cfg)
    ctl.update(_targets(x=1.8), 0.0, 0.0)
    ctl.update(_targets(x=1.0, y=0.0), 0.0, 0.1)     # ALIGNED, fire
    ctl.on_service_ack(False, 0.2)                    # 서버 거부(예: wheel_stop_unqualified)
    assert ctl.state == BACKOFF
    assert ctl.retries == 1


def test_mission_done_sets_cooldown_and_resumes():
    cfg = ApproachConfig(stop_m=1.0, v_settle=0.03, consecutive=1,
                         lat_tol=0.05, dist_tol=0.05, cooldown_s=10.0)
    ctl = ApproachController(cfg)
    ctl.update(_targets(x=1.8), 0.0, 0.0)
    ctl.update(_targets(x=1.0, y=0.0), 0.0, 0.1)
    ctl.on_service_ack(True, 0.2)
    ctl.on_mission_done(0.3)
    assert ctl.state == DONE
    # 재출발: 다음 update는 SEARCHING, 그리고 쿨다운으로 같은 박스 재트리거 안 됨
    d = ctl.update(_targets(x=1.0, y=0.0), 0.0, 0.4)
    assert d.state == SEARCHING
    assert d.active is False
    # 쿨다운 동안 계속 봐도 lock 안 됨
    for i in range(10):
        d = ctl.update(_targets(x=1.0, y=0.0), 0.0, 0.5 + i * 0.1)
    assert d.state == SEARCHING


def test_reset_from_failed_hold():
    cfg = ApproachConfig(align_timeout_s=1.0, max_retries=0, consecutive=1,
                         lat_tol=0.001, dist_tol=0.001)
    ctl = ApproachController(cfg)
    ctl.update(_targets(x=1.8), 0.0, 0.0)
    d = ctl.update(_targets(x=1.8), 0.0, 1.2)         # timeout, retries>max → FAILED_HOLD
    assert d.state == FAILED_HOLD
    ctl.reset(2.0)
    assert ctl.state == SEARCHING
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd motor_control && python -m pytest chassis/tests/test_approach.py -v -k "ack or done or reset"`
Expected: FAIL with `AttributeError: 'ApproachController' object has no attribute 'on_service_ack'`

- [ ] **Step 3: Write minimal implementation**

```python
# append methods inside ApproachController

    def on_service_ack(self, success: bool, now_s: float) -> None:
        if self.state != ALIGNED:
            return
        if success:
            self.state = ARRIVED_FIRED
            return
        # 서버 거부 → 재시도 또는 실패
        self._fired = False
        self.retries += 1
        if self.retries > self.cfg.max_retries:
            self.state = FAILED_HOLD
        else:
            self._backoff_until = now_s + self.cfg.backoff_time_s
            self.state = BACKOFF

    def on_mission_done(self, now_s: float) -> None:
        if self.state not in (ARRIVED_FIRED, ALIGNED):
            return
        if self.active_class is not None:
            self.trigger.mission_finished(self.active_class, now_s)
        self.active_status = self.active_class = None
        self._fired = False
        self.retries = 0
        self.state = DONE

    def reset(self, now_s: float) -> None:
        self.active_status = self.active_class = None
        self._fired = False
        self.retries = 0
        self._lost = 0
        self.state = SEARCHING
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd motor_control && python -m pytest chassis/tests/test_approach.py -v`
Expected: 전체 passed (3 + 5 + 4)

- [ ] **Step 5: 음성대조(negative control) — 정지 게이트 증명**

정렬 게이트가 실제로 급발진을 막는지 증명한다. 임시로 `v_settle` 조건을 코어에서 제거하면
`test_not_aligned_while_moving`가 FAIL해야 한다.

Run: `cd motor_control && python -m pytest chassis/tests/test_approach.py::test_not_aligned_while_moving -v`
게이트 제거 시 Expected: FAIL(움직이는 중 발사) → 게이트 복원 후 PASS 확인. (코드 원복 커밋하지 않음.)

- [ ] **Step 6: Commit**

```bash
git add motor_control/chassis/approach.py motor_control/chassis/tests/test_approach.py
git commit -m "feat(approach): service-ack, arm-done cooldown resume, reset"
```

---

### Task 4: `approach_controller_node` ROS 래퍼

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/approach_controller_node.py`
- Modify: `ros2/src/powertrain_ros/setup.py:53` (console_scripts에 entry 추가)
- Test: (노드 스모크는 Task 6; 여기선 import·구성만 순수 검증)

**Interfaces:**
- Consumes: Task 1–3의 `ApproachController`, `ApproachConfig`, `Target`, 상태 상수; `contract.TOPIC_DETECTED`, `contract.TOPIC_ARM_STATUS`, `contract.ARRIVED_PICKUP/DROP`.
- Produces: 설치 엔트리포인트 `approach_controller = powertrain_ros.approach_controller_node:main`.

**노드 계약:**
- 파라미터: `enabled`(bool, 기본 False), 그리고 `ApproachConfig`의 모든 필드를 `declare_parameter`.
- 구독:
  - `contract.TOPIC_DETECTED` (`DetectedObjectArray`) → TF(header.frame_id→base_link) 투영 후 `Target` 리스트 구성. TF 실패/`frame_id` 공백 시 **빈 리스트 + 경고**(안전 실패).
  - `/odom` (`Odometry`) → `speed_mps = abs(twist.twist.linear.x)`.
  - `contract.TOPIC_ARM_STATUS` (`ArmStatus`) → `status == contract.ARM_DONE`(DONE 상수, contract.py 확인) 에지에서 `ctl.on_mission_done(now)`.
- 발행:
  - `/autonomy/cmd_vel` (`Twist`) — `enabled AND decision.active` 일 때만 `v/omega` 제안. 아니면 미발행.
  - `/approach/active` (`Bool`) — `decision.active`.
  - `/approach/state` (`String`) — `f"{decision.state}|{decision.reason}"`.
- 서비스 클라이언트: `chassis_node`의 `chassis/mission_arrive_pickup`·`chassis/mission_arrive_drop`(`Trigger`). `decision.fire`가 나오면 비동기 호출, 응답 콜백에서 `ctl.on_service_ack(resp.success, now)`.
- 타이머: `control_hz`(기본 20.0)로 `_tick`: 최신 targets 스냅샷 + speed + now로 `ctl.update` → 위 발행·서비스.
- TF 투영은 `section_supervisor_node`의 `_apply_tf` 패턴을 따른다(tf2_ros Buffer/TransformListener, `lookup_transform("base_link", frame_id, Time())`, 실패 시 스킵).

- [ ] **Step 1: Write the node (참조 패턴 명시)**

```python
# ros2/src/powertrain_ros/powertrain_ros/approach_controller_node.py
"""능동 최종접근 정렬 behavior 노드 (비전 자동 ARRIVED 트리거).

    /detected_objects ─┐
    /odom ─────────────┼─→ [이 노드: ApproachController] ─→ /autonomy/cmd_vel (제안)
    TF(base_link) ─────┘                                 ├─→ /approach/active (Bool)
    /arm_status ────────────────────────────────────────┼─→ /approach/state  (String)
                                                          └─→ chassis/mission_arrive_pickup/_drop (Trigger)

🛑 `/cmd_vel` 을 직접 쓰지 않는다 — authority가 내장된 chassis_node만 받는다.
   여기서는 `/autonomy/cmd_vel` 로 **제안**만 한다(`enabled:=true` 일 때만).
⚠️ pose 프레임이 TF로 base_link에 해석 안 되면 SEARCHING 유지(엉뚱한 거리 크립 금지).
설계: docs/superpowers/specs/2026-07-27-vision-arrival-approach-design.md
"""
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from powertrain_ros import contract
from powertrain_ros.section_supervisor_node import _apply_tf  # DRY: 동일 quaternion 규칙 재사용
from robot_arm_msgs.msg import ArmStatus, DetectedObjectArray

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from chassis.approach import (                              # noqa: E402
    ApproachConfig, ApproachController, Target, ARRIVED_PICKUP,
)

_CFG_FIELDS = (
    "engage_m", "stop_m", "lat_tol", "dist_tol", "k_yaw", "k_dist",
    "omega_max", "v_approach_max", "v_settle", "backoff_creep",
    "backoff_time_s", "align_timeout_s", "max_retries", "consecutive",
    "cooldown_s", "min_confidence", "lost_frames", "pose_stale_s",
    "pickup_class", "drop_class",
)


class ApproachControllerNode(Node):
    def __init__(self):
        super().__init__("approach_controller")
        self.declare_parameter("enabled", False)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("base_frame", "base_link")
        defaults = ApproachConfig()
        for f in _CFG_FIELDS:
            self.declare_parameter(f, getattr(defaults, f))
        cfg = ApproachConfig(**{f: self.get_parameter(f).value for f in _CFG_FIELDS})
        self.ctl = ApproachController(cfg)
        self._base = str(self.get_parameter("base_frame").value)
        self._enabled = bool(self.get_parameter("enabled").value)
        self._speed = 0.0
        self._targets = []
        self._targets_s = 0.0          # 마지막 detection 수신 시각(freshness)
        self._arm_done_prev = False

        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)

        self.create_subscription(DetectedObjectArray, contract.TOPIC_DETECTED,
                                 self._on_detections, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(ArmStatus, contract.TOPIC_ARM_STATUS,
                                 self._on_arm, 10)

        self.pub_cmd = self.create_publisher(Twist, "/autonomy/cmd_vel", 10)
        self.pub_active = self.create_publisher(Bool, "/approach/active", 10)
        self.pub_state = self.create_publisher(String, "/approach/state", 10)

        # ⚠️ chassis 노드명은 "chassis_node" → 서비스 절대경로.
        self.cli_pickup = self.create_client(
            Trigger, "/chassis_node/mission_arrive_pickup")
        self.cli_drop = self.create_client(
            Trigger, "/chassis_node/mission_arrive_drop")

        hz = float(self.get_parameter("control_hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            "approach_controller 시작 — 제안 %s" % ("ON" if self._enabled else "OFF"))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry):
        self._speed = abs(msg.twist.twist.linear.x)

    def _on_arm(self, msg: ArmStatus):
        done = (str(msg.status) == contract.ARM_DONE)
        if done and not self._arm_done_prev:
            self.ctl.on_mission_done(self._now())
        self._arm_done_prev = done

    def _on_detections(self, msg: DetectedObjectArray):
        frame = msg.header.frame_id
        if not frame:
            self._targets = []
            self.get_logger().warn("detected_objects frame_id 공백 — SEARCHING 유지",
                                   throttle_duration_sec=2.0)
            return
        try:
            tf = self._tf.lookup_transform(self._base, frame, Time())
        except Exception as exc:                            # TF 미해석 → 안전 실패
            self._targets = []
            self.get_logger().warn("TF %s→%s 실패(%s) — SEARCHING 유지"
                                   % (frame, self._base, type(exc).__name__),
                                   throttle_duration_sec=2.0)
            return
        out = []
        for o in msg.objects:
            x, y, _z = _apply_tf(o.pose.position, tf)       # 3-튜플 (x,y,z), x=전방·y=횡
            out.append(Target(str(o.class_name), float(o.confidence), x, y))
        self._targets = out
        self._targets_s = self._now()

    def _tick(self):
        # pose freshness: 오래된 검출로 크립 금지(§4.3). stale이면 빈 리스트→lost/정지.
        now = self._now()
        stale = (now - self._targets_s) > self.ctl.cfg.pose_stale_s
        targets = [] if stale else self._targets
        d = self.ctl.update(targets, self._speed, now)
        self.pub_active.publish(Bool(data=bool(d.active)))
        self.pub_state.publish(String(data="%s|%s" % (d.state, d.reason)))
        if self._enabled and d.active:
            cmd = Twist()
            cmd.linear.x = float(d.v)
            cmd.angular.z = float(d.omega)
            self.pub_cmd.publish(cmd)
        if d.fire is not None:
            self._call_arrive(d.fire)

    def _call_arrive(self, status):
        cli = self.cli_pickup if status == ARRIVED_PICKUP else self.cli_drop
        if not cli.service_is_ready():
            self.get_logger().warn("mission_arrive 서비스 미준비 — ACK 거부 처리")
            self.ctl.on_service_ack(False, self._now())
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(self._on_arrive_resp)

    def _on_arrive_resp(self, fut):
        try:
            resp = fut.result()
            ok = bool(resp.success)
        except Exception:
            ok = False
        self.ctl.on_service_ack(ok, self._now())
        self.get_logger().warn("mission_arrive ACK success=%s" % ok)


def main():
    rclpy.init()
    node = ApproachControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

> 확인 완료(계획 작성 시 대조): `contract.ARM_DONE = "DONE"`(contract.py:43), `contract.TOPIC_DETECTED`/`TOPIC_ARM_STATUS`/`ARRIVED_PICKUP` 존재, chassis 노드명 = `"chassis_node"`(서비스 절대경로 `/chassis_node/mission_arrive_*`), `section_supervisor_node._apply_tf(position, transform)` = 3-튜플 반환(재사용).

- [ ] **Step 2: Register entry point**

`ros2/src/powertrain_ros/setup.py`의 `console_scripts` 리스트에 추가:

```python
            "approach_controller = powertrain_ros.approach_controller_node:main",
```

- [ ] **Step 3: Build + import smoke (젯슨 or dev 컨테이너)**

Run(젯슨 컨테이너 `/workspace/ros2`):
```bash
colcon build --packages-select powertrain_ros --symlink-install
source install/setup.bash
python -c "from powertrain_ros import approach_controller_node as m; print('import ok', m.main)"
ros2 pkg executables powertrain_ros | grep approach_controller
```
Expected: `import ok ...` + `powertrain_ros approach_controller`

- [ ] **Step 4: Commit**

```bash
git add ros2/src/powertrain_ros/powertrain_ros/approach_controller_node.py ros2/src/powertrain_ros/setup.py
git commit -m "feat(approach): approach_controller_node ROS wrapper + entry point"
```

---

### Task 5: `lane_follower` 접근 양보

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/lane_follower_node.py:107-109`(구독 추가), `:190-191`(발행 게이트)
- Test: `motor_control/chassis` 순수 테스트 대상 아님 — 노드 동작이라 Task 6 스모크에서 검증. 여기선 게이트 논리 diff만.

**Interfaces:**
- Consumes: `/approach/active` (`Bool`) from Task 4.
- Produces: 접근 활성 동안 `/autonomy/cmd_vel` 제안 억제.

- [ ] **Step 1: 구독 추가** (line 107 `self._allow_drive = True` 아래)

```python
        self._approach_active = False
        self.create_subscription(Bool, "/approach/active",
                                 lambda m: setattr(self, "_approach_active", m.data), 10)
```

- [ ] **Step 2: 발행 게이트 수정** (line 190)

기존:
```python
        if ok and imu_is_fresh and self._allow_drive and bool(
            self.get_parameter("enabled").value
```
수정:
```python
        if ok and imu_is_fresh and self._allow_drive and not self._approach_active and bool(
            self.get_parameter("enabled").value
```

- [ ] **Step 3: import 확인**

`Bool`이 이미 import돼 있는지 확인(`from std_msgs.msg import ...`). 없으면 추가.

Run(dev 컨테이너):
```bash
cd motor_control && python -c "import ast; ast.parse(open('../ros2/src/powertrain_ros/powertrain_ros/lane_follower_node.py').read()); print('syntax ok')"
```
Expected: `syntax ok`

- [ ] **Step 4: Commit**

```bash
git add ros2/src/powertrain_ros/powertrain_ros/lane_follower_node.py
git commit -m "feat(lane): yield /autonomy/cmd_vel while /approach/active"
```

---

### Task 6: 젯슨 rclpy 노드 스모크 (도메인77 격리)

**Files:**
- Create: `ros2/src/powertrain_ros/test/test_approach_controller_smoke.py` (또는 스크립트형; 기존 스모크 관례를 따름)

**Interfaces:**
- Consumes: 설치된 `approach_controller` 엔트리포인트, `robot_arm_msgs`, `tf2_ros`.
- Produces: fixture 발행 → 실응답 단언 스모크.

**시나리오:** 별도 rclpy 노드가 (a) `base_link←arm_cam` 정적 TF, (b) `/detected_objects`(box, x·y 시퀀스로 접근→정렬), (c) `/odom`(speed→0)을 발행하고, `chassis/mission_arrive_pickup` **Trigger 서비스를 mock 서버**로 띄운다. `approach_controller`를 `enabled:=true`로 실기동해:
1. `/approach/active`가 true가 되는지,
2. `/autonomy/cmd_vel`에 전진 크립(linear.x>0)이 나오는지,
3. 정렬+정지 프레임 주입 시 mock `mission_arrive_pickup`이 **실제 호출**되는지
를 단언하고 `killpg`로 정리.

- [ ] **Step 1: Write the smoke test**

```python
# ros2/src/powertrain_ros/test/test_approach_controller_smoke.py
"""approach_controller 설치 엔트리포인트 실기동 스모크 (도메인77 격리).

fixture: 정적 TF + /detected_objects + /odom 발행 + mock mission_arrive_pickup 서버.
단언: /approach/active=true, /autonomy/cmd_vel 전진 크립, 정렬 시 서비스 호출.
"""
import os
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist, TransformStamped          # noqa: E402
from nav_msgs.msg import Odometry                              # noqa: E402
from std_msgs.msg import Bool                                  # noqa: E402
from std_srvs.srv import Trigger                               # noqa: E402
from tf2_ros import StaticTransformBroadcaster                 # noqa: E402
from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray  # noqa: E402


def _spawn_node():
    env = dict(os.environ, ROS_DOMAIN_ID="77")
    return subprocess.Popen(
        ["ros2", "run", "powertrain_ros", "approach_controller",
         "--ros-args", "-p", "enabled:=true", "-p", "consecutive:=2",
         "-p", "stop_m:=1.0", "-p", "engage_m:=2.0",
         "-p", "lat_tol:=0.08", "-p", "dist_tol:=0.08", "-p", "v_settle:=0.05"],
        env=env, preexec_fn=os.setsid,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


@pytest.mark.timeout(40)
def test_approach_controller_fires_arrive_on_alignment():
    os.environ["ROS_DOMAIN_ID"] = "77"
    rclpy.init()
    node = rclpy.create_node("approach_smoke_fixture")

    got = {"active": False, "creep": False, "called": False}
    node.create_subscription(Bool, "/approach/active",
                             lambda m: got.__setitem__("active", got["active"] or m.data), 10)
    node.create_subscription(
        Twist, "/autonomy/cmd_vel",
        lambda m: got.__setitem__("creep", got["creep"] or m.linear.x > 0.0), 10)

    def _srv(req, resp):
        got["called"] = True
        resp.success = True
        resp.message = "mock"
        return resp
    node.create_service(Trigger, "/chassis_node/mission_arrive_pickup", _srv)
    node.create_service(Trigger, "/chassis_node/mission_arrive_drop", _srv)

    stf = StaticTransformBroadcaster(node)
    t = TransformStamped()
    t.header.frame_id = "base_link"
    t.child_frame_id = "arm_cam"
    t.transform.rotation.w = 1.0                # identity: pose 그대로 base_link
    stf.sendTransform(t)

    pub_det = node.create_publisher(DetectedObjectArray, "/detected_objects", 10)
    pub_odom = node.create_publisher(Odometry, "/odom", 10)

    proc = _spawn_node()
    try:
        t0 = time.time()
        # 단계: 먼 접근(x=1.8) 다수 → 정렬(x=1.0,y=0) 다수, speed=0
        stage = "approach"
        while time.time() - t0 < 30 and not got["called"]:
            arr = DetectedObjectArray()
            arr.header.frame_id = "arm_cam"
            o = DetectedObject()
            o.class_name = "box"
            o.confidence = 0.9
            if got["active"] and got["creep"]:
                stage = "align"
            o.pose.position.x = 1.0 if stage == "align" else 1.8
            o.pose.position.y = 0.0
            arr.objects = [o]
            pub_det.publish(arr)
            od = Odometry()
            od.twist.twist.linear.x = 0.0
            pub_odom.publish(od)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)
        assert got["active"], "/approach/active 가 true가 되지 않음"
        assert got["creep"], "/autonomy/cmd_vel 전진 크립 없음"
        assert got["called"], "정렬 후 mission_arrive_pickup 미호출"
    finally:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Run on Jetson (도메인77)**

Run(젯슨 컨테이너 `/workspace/ros2`, 빌드·소싱 후):
```bash
ROS_DOMAIN_ID=77 python -m pytest src/powertrain_ros/test/test_approach_controller_smoke.py -v
```
Expected: 1 passed (active + creep + 서비스 호출 관측)

- [ ] **Step 3: Commit**

```bash
git add ros2/src/powertrain_ros/test/test_approach_controller_smoke.py
git commit -m "test(approach): Jetson rclpy smoke — align triggers mission_arrive service"
```

---

### Task 7: E2E 체인 검증 (모터 무전원) + 배포

**Files:**
- Create: `scratchpad/e2e_approach.py` (검증 스크립트; 레포 커밋 아님 — 실행 로그만 문서화)
- Modify: (필요 시) `docker/docker-compose.jetson.yml` 또는 `wp5_control.launch.py`에 `approach_controller` 기동 추가 — 배포 방식은 실행 시점 확인.

**Interfaces:**
- Consumes: 배포된 `chassis`(MissionSupervisor 활성) + `approach_controller` + fixture.
- Produces: 자동 도착 → MISSION_STOP → `ArrivalStatus=ARRIVED_PICKUP` 전 체인 관측 로그.

**시나리오(무전원):** 실배포 스택에서 fixture로 정적 TF + `/detected_objects`(접근→정렬) + `/odom`(0)을 주입하고, `MissionSupervisor`가 요구하는 선행 조건(wheel_stop qualified + `ARM_STOWED_LOCKED` fresh — mock `/arm_status` 발행)을 만족시킨 뒤:
1. `approach_controller`가 `/chassis_node/mission_arrive_pickup`을 자동 호출,
2. `chassis`가 `/chassis_mode=MISSION_STOP` 발행,
3. 정지 확인 후 `/arrival_status`에 `ARRIVED_PICKUP` 발행
을 순서대로 관측한다. CAN 바퀴출력만 미관측(모터 무전원).

- [ ] **Step 1: Write E2E driver** — `scratchpad/e2e_approach.py`

`scratchpad/e2e_drive.py`의 도메인·비차단 소켓·순서 함정(us100 OFF→reset→clear_hold→arm) 패턴을 계승하고, 추가로 mock `/arm_status`(ARM_STOWED_LOCKED fresh)와 `/detected_objects`(box 접근→정렬) 발행 + `/chassis_mode`·`/arrival_status` 구독 단언을 넣는다. (전체 스크립트는 실행 시점에 배포 토폴로지 확인 후 작성.)

- [ ] **Step 2: 배포**

Run(젯슨):
```bash
docker compose -f docker/docker-compose.jetson.yml up -d --force-recreate powertrain_control powertrain_chassis
# approach_controller가 launch에 포함됐는지 확인, 없으면 임시 ros2 run으로 기동
docker exec powertrain_ros ros2 node list | grep -E "chassis|approach_controller"
```
Expected: 두 노드 모두 목록에 존재.

- [ ] **Step 3: Run E2E**

Run(젯슨 컨테이너):
```bash
ROS_DOMAIN_ID=<deploy> python scratchpad/e2e_approach.py
```
Expected 출력(순서): `approach_active=true` → `mission_arrive_pickup called success=true` → `chassis_mode=MISSION_STOP` → `arrival_status=ARRIVED_PICKUP`.

- [ ] **Step 4: 배포 무크래시 확인**

Run(젯슨):
```bash
docker exec powertrain_ros bash -c "ros2 node list | grep approach_controller" && \
docker logs --since 2m powertrain_ros 2>&1 | grep -iE "traceback|error" || echo "no crash"
```
Expected: 노드 생존 + traceback 없음.

- [ ] **Step 5: Commit(문서) + 메모리**

E2E 실행 로그를 `docs/superpowers/plans/2026-07-27-vision-arrival-approach.md` 하단 검증 섹션 또는
`docs/reports/`에 요약하고, 메모리 `robot-arm-team-resources`의 ARRIVED 항목을 "비전 자동 트리거
구현 완료(E2E PASS)"로 갱신.

```bash
git add docs/
git commit -m "docs(approach): E2E verification log — auto ARRIVED chain PASS (motors unpowered)"
```

---

## 실행 후 검증 (Claude 담당)

- 순수 스위트: `cd motor_control && python -m pytest chassis/tests/test_approach.py -v` 전수 green + 음성대조 재확인.
- 젯슨 rclpy 스모크(Task 6) 실기동 PASS.
- E2E(Task 7) 전 체인 관측 + 배포 무크래시.
- 기존 스위트 회귀 없음: `chassis/tests` 전체 + `powertrain_ros` 변경영역 rclpy.
- Cross-team relay: `DetectedObject.pose` 프레임 계약(`header.frame_id` + base_link TF) 팔팀 확정 — 미확정 시 안전 실패 동작만 검증하고 실주행 정렬은 HIL 게이트로 보류.

## 미결/후속 (이 계획 밖)

- **operator_console FAILED_HOLD 배너**: `/approach/state`는 Task 4에서 발행되므로 데이터는
  이미 관측 가능(logs·토픽). 콘솔 GTK 패널에 접근 상태·FAILED 배너를 그리는 것은 별
  서브시스템 변경이라 후속(구현 시 `operator_console` 패널 구조 정독 + `runtime_smoke` 필요).
  이 계획의 실패 UX는 `/approach/state=FAILED_HOLD` 발행 + 노드 경고 로그로 우선 충족.
- `stop_m`·`lat_tol` 실서보 HIL 캘리브(WP5.2 Task 7 서보 세션과 합류).
- `section_supervisor` 통합 arbiter(현재는 협조 양보 래치).
- 레거시 `mission_node.py` 자동트리거 정리.
- 실주행(모터 전원 + 팔 실물) 정렬 정확도 검증.
