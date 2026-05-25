# 코너 모듈 컨트롤러 (Corner Module Controller) — 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 로커보기 코너 1개(조향 AK40 + 구동 ODrive 3.6)를 협조 제어하는 재사용 라이브러리 + DualSense 텔레옵 데모를 만든다.

**Architecture:** 트랜스포트 무관 `Actuator` 추상화(조향/구동) 위에 `CornerModule` 협조 제어기를 올린다. 순수 로직(설정·상태머신·안전·협조·입력매핑)은 Fake 액추에이터로 TDD하고, 하드웨어 드라이버(AK40·ODrive USB)는 얇은 래퍼로 두어 HIL로 검증한다. `motor_control`은 `motor_gui`를 import하지 않는다(역의존 금지).

**Tech Stack:** Python 3, `python-can`(socketcan, 조향), `odrive`(USB, 구동), `pygame` 또는 기존 DualSense 입력 방식(데모), `pytest`(단위 테스트). 설계 근거: `docs/specs/2026-05-25-corner-module-controller-design.md`.

---

## 사전 안내 (구현자 필독)

- **단위:** 조향 = 출력축 도(°), 구동 = turns/s (ODrive 네이티브). m/s 변환은 본 범위 밖.
- **테스트 실행 환경:** 로컬 파이썬 테스트는 x86 dev 컨테이너(`powertrain_dev`) 안에서 실행한다. pip 의존성 추가가 필요하면 `docker/Dockerfile`에 넣는다. 모든 `pytest` 명령은 `motor_control/`을 작업 디렉터리로 가정한다(`cd /home/light/Defence_Robot/motor_control`).
- **하드웨어 드라이버 태스크(Task 5,6,9 일부):** 실모터가 필요해 `pytest`로 검증 불가. 각 태스크에 명시된 **수동 HIL 스모크 절차**로 검증하고, 단위 테스트는 작성하지 않는다. 이 드라이버 모듈들은 `odrive`/`ak_control` 등을 **모듈 최상단에서 import**하되, 순수-로직 단위 테스트는 이 모듈들을 import하지 않으므로 무하드웨어 환경에서도 테스트가 돈다.
- **Fake 기반 로직 태스크(Task 1~4, 8):** 완전 TDD. 하드웨어 의존 없음.
- 기존 참조 코드: 조향 `motor_control/steering/ak_control.py`(`AK40`, `CANSession`), 구동 USB `motor_control/drive/x2212_test/init_odrive.py` · `odrive_dualsense_vel_test.py`, 구동 CAN `odrive_can_drive.py`, CAN 셋업 `scripts/can_setup.sh`.

---

## 파일 구조 (File Structure)

```
motor_control/corner_module/
├── __init__.py            # 패키지 마커 (빈 파일)
├── config.py              # CornerConfig 데이터클래스 + clamp() 유틸          [Task 1]
├── actuator.py            # Actuator/SteerActuator/DriveActuator ABC          [Task 2]
├── fake.py                # FakeSteer/FakeDrive 테스트 더블                    [Task 2]
├── corner_module.py       # CornerModule 협조 제어기 (핵심)              [Task 3,4]
├── steer_ak40.py          # AK40 백엔드 SteerActuator (HW)                    [Task 5]
├── drive_odrive_usb.py    # ODrive USB 백엔드 DriveActuator (HW)              [Task 6]
├── drive_odrive_can.py    # ODrive CAN 백엔드 (인터페이스 예약 스텁)          [Task 7]
├── teleop_dualsense.py    # 입력매핑 + 데모 앱                            [Task 8,9]
├── README.md              # 실행법                                            [Task 9]
└── tests/
    └── test_corner_module.py   # 순수 로직 단위 테스트                  [Task 1~4,8]
```

각 파일 책임: `config`=값/한계, `actuator`=인터페이스 계약, `fake`=테스트 더블, `corner_module`=상태머신·안전·협조, `steer_ak40`/`drive_odrive_usb`/`drive_odrive_can`=하드웨어 어댑터, `teleop_dualsense`=입력원+실행 진입점.

---

## Task 1: 패키지 스캐폴드 + CornerConfig + clamp 유틸

**Files:**
- Create: `motor_control/corner_module/__init__.py` (빈 파일)
- Create: `motor_control/corner_module/config.py`
- Create: `motor_control/corner_module/tests/test_corner_module.py`

- [ ] **Step 1: 빈 패키지 마커 생성**

`motor_control/corner_module/__init__.py` 를 빈 파일로 생성.

- [ ] **Step 2: 실패하는 테스트 작성**

`motor_control/corner_module/tests/test_corner_module.py`:

```python
from corner_module.config import CornerConfig, clamp


def test_default_config_values():
    c = CornerConfig()
    assert c.steer_min_deg == -45.0
    assert c.steer_max_deg == 45.0
    assert c.drive_vel_limit == 5.0
    assert c.watchdog_ms == 300.0
    assert c.loop_hz == 50.0
    assert c.steer_gate is False
    assert c.gate_deg == 10.0
    assert c.stale_ms == 200.0


def test_clamp_bounds():
    assert clamp(100.0, -45.0, 45.0) == 45.0
    assert clamp(-100.0, -45.0, 45.0) == -45.0
    assert clamp(12.0, -45.0, 45.0) == 12.0
```

- [ ] **Step 3: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'corner_module.config'`

- [ ] **Step 4: config.py 구현**

`motor_control/corner_module/config.py`:

```python
"""코너 모듈 설정값·한계 및 공용 유틸."""
from dataclasses import dataclass


def clamp(value: float, lo: float, hi: float) -> float:
    """value 를 [lo, hi] 로 제한."""
    return max(lo, min(hi, value))


@dataclass
class CornerConfig:
    steer_min_deg: float = -45.0    # 조향 출력축 최소각
    steer_max_deg: float = 45.0     # 조향 출력축 최대각
    drive_vel_limit: float = 5.0    # 구동 최대속도 (turns/s)
    watchdog_ms: float = 300.0      # 텔레옵 입력 타임아웃 (ms)
    loop_hz: float = 50.0           # 제어 루프 주기
    steer_gate: bool = False        # 협조 로직 on/off (기본 OFF)
    gate_deg: float = 10.0          # 협조 감속 시작 조향오차
    stale_ms: float = 200.0         # AK status 미수신 stale 임계
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: PASS (2 passed)

- [ ] **Step 6: 커밋**

```bash
cd /home/light/Defence_Robot
git add motor_control/corner_module/__init__.py motor_control/corner_module/config.py motor_control/corner_module/tests/test_corner_module.py
git commit -m "feat(corner_module): CornerConfig + clamp 유틸 + 패키지 스캐폴드"
```

---

## Task 2: 액추에이터 ABC + Fake 테스트 더블

**Files:**
- Create: `motor_control/corner_module/actuator.py`
- Create: `motor_control/corner_module/fake.py`
- Modify: `motor_control/corner_module/tests/test_corner_module.py` (테스트 추가)

- [ ] **Step 1: 실패하는 테스트 작성** (파일 끝에 추가)

```python
from corner_module.fake import FakeSteer, FakeDrive


def test_fake_steer_converges_to_target_when_armed():
    s = FakeSteer(start_deg=0.0)
    s.connect()
    s.arm()
    s.set_angle(20.0)
    for _ in range(30):
        s.tick()
    assert abs(s.state()["actual_deg"] - 20.0) < 0.5


def test_fake_steer_arm_is_jump_safe():
    # arm 직후 목표는 현재 실제각과 같아야(점프 방지)
    s = FakeSteer(start_deg=15.0)
    s.connect()
    s.arm()
    assert s.state()["target_deg"] == 15.0


def test_fake_drive_arm_targets_zero_velocity():
    d = FakeDrive(start_vel=2.0)
    d.connect()
    d.arm()
    assert d.state()["target_vel"] == 0.0


def test_fake_steer_state_schema():
    s = FakeSteer()
    keys = set(s.state().keys())
    assert keys == {"target_deg", "actual_deg", "cur_a", "fault", "stale"}


def test_fake_drive_state_schema():
    d = FakeDrive()
    keys = set(d.state().keys())
    assert keys == {"target_vel", "actual_vel", "cur_a"}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'corner_module.fake'`

- [ ] **Step 3: actuator.py 구현**

`motor_control/corner_module/actuator.py`:

```python
"""트랜스포트 무관 액추에이터 인터페이스.

CornerModule 은 이 인터페이스 뒤의 구체 구현(AK40/ODrive USB/CAN/Fake)을
교체해도 동작이 변하지 않는다.
"""
from abc import ABC, abstractmethod


class Actuator(ABC):
    @abstractmethod
    def connect(self) -> None:
        """버스/USB 연결."""

    @abstractmethod
    def arm(self) -> None:
        """폐루프 진입. 현재 상태로 타깃을 동기해 점프를 방지한다."""

    @abstractmethod
    def disarm(self) -> None:
        """폐루프 해제."""

    @abstractmethod
    def tick(self) -> None:
        """매 제어 루프 호출. 명령 재전송·상태 폴링 등 통신 서비스."""

    @abstractmethod
    def state(self) -> dict:
        """정규화 텔레메트리 딕셔너리."""

    @abstractmethod
    def estop(self) -> None:
        """즉시 정지."""

    @abstractmethod
    def close(self) -> None:
        """연결 해제·정리."""


class SteerActuator(Actuator):
    @abstractmethod
    def set_angle(self, deg: float) -> None:
        """출력축 목표각(도)."""


class DriveActuator(Actuator):
    @abstractmethod
    def set_velocity(self, turns_per_s: float) -> None:
        """목표 속도(turns/s)."""
```

- [ ] **Step 4: fake.py 구현**

`motor_control/corner_module/fake.py`:

```python
"""하드웨어 없는 단위 테스트용 Fake 액추에이터.

명령을 받아 1차 지연(매 tick 오차의 50% 수렴)으로 actual 이 target 에
수렴하는 단순 모델. arm 전/disarm 상태에서는 움직이지 않는다.
"""
from corner_module.actuator import SteerActuator, DriveActuator

_STEP = 0.5  # 매 tick 수렴 비율


class FakeSteer(SteerActuator):
    def __init__(self, start_deg: float = 0.0):
        self._target = start_deg
        self._actual = start_deg
        self._armed = False
        self._connected = False
        self.cur_a = 0.0
        self.fault = 0
        self.stale_flag = False

    def connect(self) -> None:
        self._connected = True

    def arm(self) -> None:
        self._armed = True
        self._target = self._actual  # 점프 방지

    def disarm(self) -> None:
        self._armed = False

    def set_angle(self, deg: float) -> None:
        self._target = deg

    def tick(self) -> None:
        if self._armed:
            self._actual += (self._target - self._actual) * _STEP

    def state(self) -> dict:
        return {
            "target_deg": self._target,
            "actual_deg": self._actual,
            "cur_a": self.cur_a,
            "fault": self.fault,
            "stale": self.stale_flag,
        }

    def estop(self) -> None:
        self._target = self._actual
        self._armed = False

    def close(self) -> None:
        self._connected = False


class FakeDrive(DriveActuator):
    def __init__(self, start_vel: float = 0.0):
        self._target = 0.0
        self._actual = start_vel
        self._armed = False
        self._connected = False
        self.cur_a = 0.0

    def connect(self) -> None:
        self._connected = True

    def arm(self) -> None:
        self._armed = True
        self._target = 0.0  # 점프 방지: 0 속도로 진입

    def disarm(self) -> None:
        self._armed = False

    def set_velocity(self, turns_per_s: float) -> None:
        self._target = turns_per_s

    def tick(self) -> None:
        if self._armed:
            self._actual += (self._target - self._actual) * _STEP
        else:
            self._actual = 0.0

    def state(self) -> dict:
        return {
            "target_vel": self._target,
            "actual_vel": self._actual,
            "cur_a": self.cur_a,
        }

    def estop(self) -> None:
        self._target = 0.0
        self._actual = 0.0
        self._armed = False

    def close(self) -> None:
        self._connected = False
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: PASS (7 passed)

- [ ] **Step 6: 커밋**

```bash
cd /home/light/Defence_Robot
git add motor_control/corner_module/actuator.py motor_control/corner_module/fake.py motor_control/corner_module/tests/test_corner_module.py
git commit -m "feat(corner_module): 액추에이터 ABC + Fake 테스트 더블"
```

---

## Task 3: CornerModule 생명주기 (connect/arm/set/disarm/close + clamp + 점프방지)

**Files:**
- Create: `motor_control/corner_module/corner_module.py`
- Modify: `motor_control/corner_module/tests/test_corner_module.py`

- [ ] **Step 1: 실패하는 테스트 작성** (파일 끝에 추가)

```python
from corner_module.corner_module import CornerModule
from corner_module.config import CornerConfig


def _make_cm(steer=None, drive=None, cfg=None, clock=None):
    return CornerModule(
        steer or FakeSteer(),
        drive or FakeDrive(),
        cfg or CornerConfig(),
        clock=clock,
    )


def test_lifecycle_modes():
    cm = _make_cm()
    assert cm.mode == "DISCONNECTED"
    cm.connect()
    assert cm.mode == "IDLE"
    cm.arm()
    assert cm.mode == "ARMED"
    cm.disarm()
    assert cm.mode == "IDLE"
    cm.close()
    assert cm.mode == "DISCONNECTED"


def test_arm_jump_prevention():
    cm = _make_cm(steer=FakeSteer(start_deg=15.0), drive=FakeDrive(start_vel=2.0))
    cm.connect()
    cm.arm()
    assert cm.state()["steer"]["target_deg"] == 15.0
    assert cm.state()["drive"]["target_vel"] == 0.0


def test_set_clamps_targets():
    cm = _make_cm(cfg=CornerConfig(steer_max_deg=45.0, drive_vel_limit=5.0))
    cm.connect()
    cm.arm()
    cm.set(100.0, 99.0)
    cm.tick()
    assert cm.state()["steer"]["target_deg"] == 45.0
    assert cm.state()["drive"]["target_vel"] == 5.0


def test_set_ignored_when_not_armed():
    cm = _make_cm()
    cm.connect()  # IDLE, not ARMED
    cm.set(30.0, 3.0)
    # IDLE 에서는 목표가 반영되지 않아야
    assert cm.state()["steer"]["target_deg"] == 0.0
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'corner_module.corner_module'`

- [ ] **Step 3: corner_module.py 구현 (생명주기 부분)**

`motor_control/corner_module/corner_module.py`:

```python
"""코너 모듈 협조 제어기.

조향 액추에이터 1 + 구동 액추에이터 1 을 묶어 (조향각, 구동속도) 명령을
안전하게 적용한다. tick() 을 외부 루프가 주기적으로 호출하거나 run() 사용.
"""
import logging
import time

from corner_module.config import CornerConfig, clamp

logger = logging.getLogger(__name__)


class CornerModule:
    def __init__(self, steer, drive, cfg: CornerConfig, clock=None):
        self.steer = steer
        self.drive = drive
        self.cfg = cfg
        self.mode = "DISCONNECTED"
        self._steer_target = 0.0
        self._drive_target = 0.0
        self._last_set_ms = None
        self._now = clock or time.monotonic  # 테스트에서 주입 가능

    def _now_ms(self) -> float:
        return self._now() * 1000.0

    def connect(self) -> None:
        self.steer.connect()
        self.drive.connect()
        self.mode = "IDLE"

    def arm(self) -> None:
        self.steer.arm()
        self.drive.arm()
        # 점프 방지: 조향 목표=현재 실제각, 구동 목표=0
        self._steer_target = self.steer.state()["actual_deg"]
        self._drive_target = 0.0
        self.steer.set_angle(self._steer_target)
        self.drive.set_velocity(0.0)
        self._last_set_ms = self._now_ms()
        self.mode = "ARMED"

    def set(self, steer_deg: float, drive_vel: float) -> None:
        if self.mode != "ARMED":
            logger.warning("set() 무시: ARMED 아님 (mode=%s)", self.mode)
            return
        self._steer_target = clamp(steer_deg, self.cfg.steer_min_deg, self.cfg.steer_max_deg)
        self._drive_target = clamp(drive_vel, -self.cfg.drive_vel_limit, self.cfg.drive_vel_limit)
        self._last_set_ms = self._now_ms()

    def state(self) -> dict:
        return {
            "mode": self.mode,
            "steer": self.steer.state(),
            "drive": self.drive.state(),
            "faults": [],
        }

    def disarm(self) -> None:
        self.drive.set_velocity(0.0)
        self.steer.disarm()
        self.drive.disarm()
        self.mode = "IDLE"

    def close(self) -> None:
        self.steer.close()
        self.drive.close()
        self.mode = "DISCONNECTED"

    def tick(self) -> None:
        # Task 4 에서 안전·협조 로직을 채운다. 지금은 ARMED 일 때 목표만 push.
        if self.mode != "ARMED":
            self.steer.tick()
            self.drive.tick()
            return
        self.steer.set_angle(self._steer_target)
        self.drive.set_velocity(self._drive_target)
        self.steer.tick()
        self.drive.tick()
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: PASS (11 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/light/Defence_Robot
git add motor_control/corner_module/corner_module.py motor_control/corner_module/tests/test_corner_module.py
git commit -m "feat(corner_module): CornerModule 생명주기 + clamp + 점프방지"
```

---

## Task 4: CornerModule tick 안전·협조 로직 (워치독·트립·stale·gate·estop)

**Files:**
- Modify: `motor_control/corner_module/corner_module.py` (tick + estop 보강)
- Modify: `motor_control/corner_module/tests/test_corner_module.py`

- [ ] **Step 1: 실패하는 테스트 작성** (파일 끝에 추가)

```python
class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, sec):
        self.t += sec


def test_watchdog_zeros_drive_on_timeout():
    clk = FakeClock()
    cm = _make_cm(cfg=CornerConfig(watchdog_ms=300.0), clock=clk)
    cm.connect()
    cm.arm()
    cm.set(10.0, 3.0)
    clk.advance(0.1)  # 100ms < 300ms
    cm.tick()
    assert cm.state()["drive"]["target_vel"] == 3.0
    clk.advance(0.5)  # 총 600ms > 300ms
    cm.tick()
    assert cm.state()["drive"]["target_vel"] == 0.0


def test_estop_stops_both_and_faults():
    cm = _make_cm()
    cm.connect()
    cm.arm()
    cm.set(30.0, 3.0)
    cm.tick()
    cm.estop()
    assert cm.mode == "FAULT"
    assert cm.state()["drive"]["target_vel"] == 0.0


def test_steer_fault_triggers_estop():
    s = FakeSteer()
    cm = _make_cm(steer=s)
    cm.connect()
    cm.arm()
    cm.set(10.0, 2.0)
    s.fault = 5
    cm.tick()
    assert cm.mode == "FAULT"


def test_steer_stale_triggers_estop():
    s = FakeSteer()
    cm = _make_cm(steer=s)
    cm.connect()
    cm.arm()
    cm.set(10.0, 2.0)
    s.stale_flag = True
    cm.tick()
    assert cm.mode == "FAULT"


def test_corner_state_schema():
    cm = _make_cm()
    cm.connect()
    st = cm.state()
    assert set(st.keys()) == {"mode", "steer", "drive", "faults"}


def test_steer_gate_holds_drive_until_settled():
    clk = FakeClock()
    cfg = CornerConfig(steer_gate=True, gate_deg=10.0, watchdog_ms=100000.0)
    cm = _make_cm(steer=FakeSteer(start_deg=0.0), cfg=cfg, clock=clk)
    cm.connect()
    cm.arm()
    cm.set(40.0, 4.0)
    cm.tick()  # 조향오차 40 > 10 → 구동 게이트
    assert cm.state()["drive"]["target_vel"] == 0.0
    for _ in range(30):  # 조향 수렴
        cm.set(40.0, 4.0)
        cm.tick()
    assert cm.state()["steer"]["actual_deg"] > 35.0
    assert cm.state()["drive"]["target_vel"] == 4.0  # 게이트 해제
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: FAIL — `test_watchdog_*`, `test_estop_*`, `test_steer_fault_*`, `test_steer_stale_*`, `test_steer_gate_*` (estop/tick 미구현)

- [ ] **Step 3: tick + estop 구현으로 교체**

`motor_control/corner_module/corner_module.py` 의 `tick` 메서드를 아래로 교체하고, `estop` 메서드를 추가한다 (`close` 위에):

```python
    def estop(self) -> None:
        self.steer.estop()
        self.drive.estop()
        self._drive_target = 0.0
        self.mode = "FAULT"

    def tick(self) -> None:
        if self.mode != "ARMED":
            self.steer.tick()
            self.drive.tick()
            return

        st = self.steer.state()

        # 1) 조향 fault/전류 트립
        if st["fault"] != 0:
            logger.error("조향 fault=%s → estop", st["fault"])
            self.estop()
            return
        # 2) CAN stale
        if st.get("stale"):
            logger.error("조향 status stale → estop")
            self.estop()
            return

        # 3) 워치독: 입력 타임아웃 시 구동 0
        drive_cmd = self._drive_target
        if self._last_set_ms is not None and (self._now_ms() - self._last_set_ms) > self.cfg.watchdog_ms:
            drive_cmd = 0.0

        # 4) 협조 로직(옵션): 조향 따라오기 전 구동 자제
        if self.cfg.steer_gate:
            err = abs(self._steer_target - st["actual_deg"])
            if err > self.cfg.gate_deg:
                drive_cmd = 0.0

        # 5) 목표 push
        self.steer.set_angle(self._steer_target)
        self.drive.set_velocity(drive_cmd)
        self.steer.tick()
        self.drive.tick()
```

(Task 3 에서 임시로 둔 `tick` 정의를 제거하고 위 버전만 남긴다.)

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: PASS (17 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/light/Defence_Robot
git add motor_control/corner_module/corner_module.py motor_control/corner_module/tests/test_corner_module.py
git commit -m "feat(corner_module): tick 안전·협조 로직 (워치독·트립·stale·gate·estop)"
```

---

## Task 5: SteerAk40 하드웨어 드라이버 (HIL 검증)

**Files:**
- Create: `motor_control/corner_module/steer_ak40.py`

**검증 방식:** 실모터(AK40 on can0)가 필요 → `pytest` 불가. 아래 수동 HIL 스모크로 검증한다.

- [ ] **Step 1: steer_ak40.py 구현**

`motor_control/corner_module/steer_ak40.py`:

```python
"""AK40-10 백엔드 SteerActuator. motor_control/steering/ak_control.py 재사용.

조향 출력축 각(도)을 그대로 받아 AK40 의 위치 명령으로 전달하고, 매 tick
status 를 폴링한다. AK40 내장 전류/fault 정보를 state 로 노출한다.
"""
import os
import sys
import time

import can

from corner_module.actuator import SteerActuator

# ak_control.py 를 import 경로에 추가 (motor_control/steering)
_STEERING_DIR = os.path.join(os.path.dirname(__file__), "..", "steering")
sys.path.insert(0, os.path.abspath(_STEERING_DIR))
from ak_control import AK40  # noqa: E402


class SteerAk40(SteerActuator):
    def __init__(self, motor_id: int = 1, channel: str = "can0", stale_ms: float = 200.0):
        self._motor_id = motor_id
        self._channel = channel
        self._stale_ms = stale_ms
        self._bus = None
        self._ak = None
        self._target_deg = 0.0
        self._last_rx_ms = None

    def connect(self) -> None:
        try:
            self._bus = can.interface.Bus(channel=self._channel, interface="socketcan")
        except OSError as e:
            raise RuntimeError(
                f"can0 열기 실패({e}). 먼저 'bash scripts/can_setup.sh' 실행하세요."
            ) from e
        self._ak = AK40(self._bus, self._motor_id, name="steer")

    def arm(self) -> None:
        # 현재각을 읽어 목표로 동기(점프 방지). poll 로 최신 상태 확보.
        self._ak.poll(timeout=0.05)
        self._target_deg = self._ak.pos_out_deg

    def disarm(self) -> None:
        self._ak.stop()

    def set_angle(self, deg: float) -> None:
        self._target_deg = deg

    def tick(self) -> None:
        self._ak.send_pos_out(self._target_deg)
        got = self._ak.poll(timeout=0.005)
        if got:
            self._last_rx_ms = time.monotonic() * 1000.0

    def state(self) -> dict:
        stale = (
            self._last_rx_ms is None
            or (time.monotonic() * 1000.0 - self._last_rx_ms) > self._stale_ms
        )
        return {
            "target_deg": self._target_deg,
            "actual_deg": self._ak.pos_out_deg if self._ak else 0.0,
            "cur_a": self._ak.cur_a if self._ak else 0.0,
            "fault": self._ak.fault if self._ak else 0,
            "stale": stale,
        }

    def estop(self) -> None:
        if self._ak:
            self._ak.stop()

    def close(self) -> None:
        if self._ak:
            self._ak.stop()
        if self._bus:
            self._bus.shutdown()
```

> **참고:** `AK40` 의 실제 메서드명(`send_pos_out`, `poll`, `stop`, 속성 `pos_out_deg`/`cur_a`/`fault`)은 `motor_control/steering/ak_control.py` 와 일치해야 한다. 구현 전 해당 파일에서 시그니처를 확인하고, `poll(timeout=...)` 의 반환값(수신 성공 여부)이 없으면 `poll` 호출 후 마지막 수신 시각을 갱신하는 방식으로 맞춘다.

- [ ] **Step 2: 임포트 스모크 (무하드웨어)**

Run: `cd /home/light/Defence_Robot/motor_control && python -c "import sys; sys.path.insert(0,'.'); from corner_module.steer_ak40 import SteerAk40; print('import OK')"`
Expected: `import OK` (python-can 미설치면 dev 컨테이너에서 실행)

- [ ] **Step 3: 수동 HIL 스모크 (실모터, Jetson)**

사전: `bash scripts/can_setup.sh` 로 can0 1Mbps 기동. AK40 종단저항 확인.

```bash
cd /home/light/Defence_Robot/motor_control
python3 -c "
import sys; sys.path.insert(0,'.')
from corner_module.steer_ak40 import SteerAk40
import time
s = SteerAk40(motor_id=1)
s.connect(); s.arm()
for _ in range(100):           # 약 2초, 20° 목표
    s.set_angle(20.0); s.tick(); time.sleep(0.02)
print('state:', s.state())
s.disarm(); s.close()
"
```
Expected: 모터가 점프 없이 ~20°로 이동, `state()` 의 `actual_deg` 가 20 부근, `stale=False`, `fault=0`. (각도 부호/스케일이 반대면 `set_angle` 매핑이 아니라 `ak_control` 영점/기어비 설정 문제 — calibrate 후 재시도.)

- [ ] **Step 4: 커밋**

```bash
cd /home/light/Defence_Robot
git add motor_control/corner_module/steer_ak40.py
git commit -m "feat(corner_module): SteerAk40 하드웨어 드라이버 (HIL 검증)"
```

---

## Task 6: DriveOdriveUsb 하드웨어 드라이버 (HIL 검증)

**Files:**
- Create: `motor_control/corner_module/drive_odrive_usb.py`

**검증 방식:** 실 ODrive 3.6(USB) + X2212 필요 → `pytest` 불가. 수동 HIL 스모크로 검증.

- [ ] **Step 1: drive_odrive_usb.py 구현**

`motor_control/corner_module/drive_odrive_usb.py`:

```python
"""ODrive 3.6(USB) 백엔드 DriveActuator. axis1, VELOCITY_CONTROL + PASSTHROUGH.

폐루프 진입 시 input_vel=0 으로 점프를 방지한다. vel_limit/current_lim 은
NVM 에 저장된 설정값을 그대로 사용(init_odrive.py 로 1회 셋업 가정).
"""
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)

from corner_module.actuator import DriveActuator


class DriveOdriveUsb(DriveActuator):
    def __init__(self, find_timeout: float = 10.0):
        self._find_timeout = find_timeout
        self._odrv = None
        self._axis = None
        self._target_vel = 0.0

    def connect(self) -> None:
        self._odrv = odrive.find_any(timeout=self._find_timeout)
        if self._odrv is None:
            raise RuntimeError("ODrive USB 미발견. 케이블/전원 확인.")
        self._axis = self._odrv.axis1

    def arm(self) -> None:
        ax = self._axis
        ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        ax.controller.input_vel = 0.0           # 점프 방지
        self._target_vel = 0.0
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def disarm(self) -> None:
        self._axis.controller.input_vel = 0.0
        self._axis.requested_state = AXIS_STATE_IDLE

    def set_velocity(self, turns_per_s: float) -> None:
        self._target_vel = turns_per_s

    def tick(self) -> None:
        self._axis.controller.input_vel = self._target_vel

    def state(self) -> dict:
        if self._axis is None:
            return {"target_vel": self._target_vel, "actual_vel": 0.0, "cur_a": 0.0}
        return {
            "target_vel": self._target_vel,
            "actual_vel": self._axis.encoder.vel_estimate,
            "cur_a": self._axis.motor.current_control.Iq_measured,
        }

    def estop(self) -> None:
        if self._axis is not None:
            self._axis.controller.input_vel = 0.0
            self._axis.requested_state = AXIS_STATE_IDLE
        self._target_vel = 0.0

    def close(self) -> None:
        if self._axis is not None:
            self._axis.requested_state = AXIS_STATE_IDLE
```

> **참고:** ODrive fw v0.5.x 의 enum/속성 경로(`encoder.vel_estimate`, `motor.current_control.Iq_measured`)는 `init_odrive.py`/`odrive_dualsense_vel_test.py` 와 동일 펌웨어 기준이다. 펌웨어 버전이 다르면 해당 스크립트의 경로에 맞춘다.

- [ ] **Step 2: 수동 HIL 스모크 (실 ODrive, 바퀴 공중)**

사전: `init_odrive.py` 로 NVM 셋업(pp=7, cpr=16384) 완료 가정. 바퀴가 땅에 안 닿게 거치.

```bash
cd /home/light/Defence_Robot/motor_control
python3 -c "
import sys; sys.path.insert(0,'.')
from corner_module.drive_odrive_usb import DriveOdriveUsb
import time
d = DriveOdriveUsb()
d.connect(); d.arm()
for _ in range(100):           # 약 2초, 1 turn/s
    d.set_velocity(1.0); d.tick(); time.sleep(0.02)
print('state:', d.state())
d.disarm(); d.close()
"
```
Expected: 폐루프 진입 시 점프 없음, 모터가 ~1 turn/s 로 회전, `state()` 의 `actual_vel` 가 1.0 부근.

- [ ] **Step 3: 커밋**

```bash
cd /home/light/Defence_Robot
git add motor_control/corner_module/drive_odrive_usb.py
git commit -m "feat(corner_module): DriveOdriveUsb 하드웨어 드라이버 (HIL 검증)"
```

---

## Task 7: DriveOdriveCan 인터페이스 예약 스텁

**Files:**
- Create: `motor_control/corner_module/drive_odrive_can.py`
- Modify: `motor_control/corner_module/tests/test_corner_module.py`

CAN-only 전환은 케이블 확보 후. 지금은 인터페이스만 예약해 미래 슬롯을 명시한다.

- [ ] **Step 1: 실패하는 테스트 작성** (파일 끝에 추가)

```python
import pytest

from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.actuator import DriveActuator


def test_odrive_can_is_drive_actuator():
    d = DriveOdriveCan()
    assert isinstance(d, DriveActuator)


def test_odrive_can_connect_not_implemented():
    d = DriveOdriveCan()
    with pytest.raises(NotImplementedError):
        d.connect()
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'corner_module.drive_odrive_can'`

- [ ] **Step 3: drive_odrive_can.py 스텁 구현**

`motor_control/corner_module/drive_odrive_can.py`:

```python
"""ODrive 3.6(CAN) 백엔드 DriveActuator — 미래 CAN-only 전환용 인터페이스 예약.

케이블(Jetson–AK–ODrive 공통 CAN) 확보 후 구현한다. 구현 시 CANSimple
(NODE_ID<<5)|cmd, fw-v0.5.6, 현재위치 동기 점프방지를 따른다
(참조: motor_control/drive/x2212_test/odrive_can_drive.py).
"""
from corner_module.actuator import DriveActuator


class DriveOdriveCan(DriveActuator):
    def __init__(self, node_id: int = 1, channel: str = "can0"):
        self._node_id = node_id
        self._channel = channel

    def connect(self) -> None:
        raise NotImplementedError("DriveOdriveCan 미구현 — CAN-only 전환 시 구현")

    def arm(self) -> None:
        raise NotImplementedError

    def disarm(self) -> None:
        raise NotImplementedError

    def set_velocity(self, turns_per_s: float) -> None:
        raise NotImplementedError

    def tick(self) -> None:
        raise NotImplementedError

    def state(self) -> dict:
        raise NotImplementedError

    def estop(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: PASS (19 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/light/Defence_Robot
git add motor_control/corner_module/drive_odrive_can.py motor_control/corner_module/tests/test_corner_module.py
git commit -m "feat(corner_module): DriveOdriveCan 인터페이스 예약 스텁"
```

---

## Task 8: 텔레옵 입력 매핑 (순수 함수, TDD)

**Files:**
- Create: `motor_control/corner_module/teleop_dualsense.py` (매핑 함수 부분)
- Modify: `motor_control/corner_module/tests/test_corner_module.py`

- [ ] **Step 1: 실패하는 테스트 작성** (파일 끝에 추가)

```python
from corner_module.teleop_dualsense import map_input


def test_map_input_neutral_is_zero():
    cfg = CornerConfig(steer_max_deg=45.0, drive_vel_limit=5.0)
    steer, drive = map_input(left_x=0.0, rt=0.0, lt=0.0, cfg=cfg)
    assert steer == 0.0
    assert drive == 0.0


def test_map_input_deadzone():
    cfg = CornerConfig()
    steer, drive = map_input(left_x=0.03, rt=0.02, lt=0.0, cfg=cfg, deadzone=0.05)
    assert steer == 0.0
    assert drive == 0.0


def test_map_input_full_steer_and_drive():
    cfg = CornerConfig(steer_max_deg=45.0, drive_vel_limit=5.0)
    steer, drive = map_input(left_x=1.0, rt=1.0, lt=0.0, cfg=cfg)
    assert steer == 45.0
    assert drive == 5.0


def test_map_input_reverse_drive():
    cfg = CornerConfig(drive_vel_limit=5.0)
    steer, drive = map_input(left_x=0.0, rt=0.0, lt=1.0, cfg=cfg)
    assert drive == -5.0
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'corner_module.teleop_dualsense'`

- [ ] **Step 3: teleop_dualsense.py 의 map_input 구현**

`motor_control/corner_module/teleop_dualsense.py` (이 단계에서는 매핑 함수만):

```python
"""DualSense 텔레옵 데모: 게임패드 입력 → CornerModule.set().

map_input 은 순수 함수로 분리해 단위 테스트한다. 실행 진입점(main)은 Task 9.
"""
from corner_module.config import CornerConfig


def map_input(left_x: float, rt: float, lt: float, cfg: CornerConfig,
              deadzone: float = 0.05):
    """좌스틱 X → 조향각, (RT−LT) → 구동속도. 데드존 적용.

    조향은 좌우 대칭(±steer_max_deg)으로 매핑한다(CornerModule 이 다시 clamp).
    """
    sx = 0.0 if abs(left_x) < deadzone else left_x
    steer_deg = sx * cfg.steer_max_deg

    trig = rt - lt
    if abs(trig) < deadzone:
        trig = 0.0
    drive_vel = trig * cfg.drive_vel_limit
    return steer_deg, drive_vel
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: PASS (23 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/light/Defence_Robot
git add motor_control/corner_module/teleop_dualsense.py motor_control/corner_module/tests/test_corner_module.py
git commit -m "feat(corner_module): 텔레옵 입력 매핑 순수 함수 (TDD)"
```

---

## Task 9: 텔레옵 데모 앱 진입점 + README

**Files:**
- Modify: `motor_control/corner_module/teleop_dualsense.py` (main 추가)
- Create: `motor_control/corner_module/README.md`

**검증 방식:** 게임패드+실모터 필요 → `pytest` 불가. 수동 실행으로 검증.

- [ ] **Step 1: teleop_dualsense.py 에 main 추가** (파일 끝에 append)

```python
import sys
import time


def main():
    import pygame  # 게임패드 입력 (기존 odrive_dualsense_* 와 동일 방식)

    from corner_module.config import CornerConfig
    from corner_module.corner_module import CornerModule
    from corner_module.steer_ak40 import SteerAk40
    from corner_module.drive_odrive_usb import DriveOdriveUsb

    cfg = CornerConfig()
    cm = CornerModule(SteerAk40(motor_id=1), DriveOdriveUsb(), cfg)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("게임패드 미연결"); sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()

    cm.connect()
    armed = False
    period = 1.0 / cfg.loop_hz
    last_print = 0.0
    print("□: arm/disarm 토글, ○: estop, Ctrl-C: 종료")
    try:
        while True:
            pygame.event.pump()
            left_x = js.get_axis(0)
            rt = (js.get_axis(5) + 1.0) / 2.0   # 트리거 [-1,1] → [0,1]
            lt = (js.get_axis(4) + 1.0) / 2.0
            if js.get_button(2):                # □ Square
                if armed:
                    cm.disarm(); armed = False
                else:
                    cm.arm(); armed = True
                time.sleep(0.3)                 # 디바운스
            if js.get_button(1):                # ○ Circle
                cm.estop(); armed = False

            if armed and cm.mode == "ARMED":
                steer, drive = map_input(left_x, rt, lt, cfg)
                cm.set(steer, drive)
            cm.tick()

            now = time.monotonic()
            if now - last_print > 1.0:          # 1Hz 상태 출력
                print(cm.state())
                last_print = now
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        cm.disarm()
        cm.close()
        print("종료")


if __name__ == "__main__":
    sys.path.insert(0, ".")
    main()
```

> **참고:** 게임패드 축/버튼 인덱스(`get_axis(0/4/5)`, `get_button(1/2)`)는 기존 `motor_control/drive/x2212_test/odrive_dualsense_vel_test.py` 의 매핑과 대조해 일치시킨다. 다르면 그 스크립트 값을 따른다.

- [ ] **Step 2: 매핑 함수 회귀 확인 (무하드웨어)**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/test_corner_module.py -v`
Expected: PASS (23 passed) — main 추가가 기존 테스트를 깨지 않음

- [ ] **Step 3: README.md 작성**

`motor_control/corner_module/README.md`:

````markdown
# corner_module — 코너 모듈 컨트롤러

로커보기 코너 1개(조향 AK40 + 구동 ODrive 3.6)의 협조 제어 라이브러리 + DualSense 데모.
설계: `docs/specs/2026-05-25-corner-module-controller-design.md`.

## 구성
- `config.py` — CornerConfig(한계·워치독·게이트), clamp
- `actuator.py` — Actuator/SteerActuator/DriveActuator 인터페이스
- `corner_module.py` — CornerModule (상태머신·안전·협조)
- `steer_ak40.py` — AK40(CAN) 조향 드라이버
- `drive_odrive_usb.py` — ODrive(USB) 구동 드라이버 (현재)
- `drive_odrive_can.py` — ODrive(CAN) 구동 (미래 CAN-only 전환 슬롯)
- `fake.py` — 무하드웨어 테스트 더블
- `teleop_dualsense.py` — DualSense 텔레옵 데모

## 단위
조향 = 출력축 도(°), 구동 = turns/s. (m/s 변환: `v = turns/s × 2π × 0.1`)

## 테스트 (x86 dev 컨테이너에서)
```bash
cd /home/light/Defence_Robot/motor_control
python -m pytest corner_module/tests/ -v
```

## 텔레옵 실행 (Jetson, 실모터)
```bash
bash scripts/can_setup.sh            # can0 1Mbps 기동 (조향)
# ODrive USB 는 init_odrive.py 로 1회 NVM 셋업 가정
cd /home/light/Defence_Robot/motor_control
python3 corner_module/teleop_dualsense.py
```
□=arm/disarm, ○=estop, 좌스틱 X=조향, RT/LT=전/후진.

## 미래 (본 라이브러리 범위 밖)
- `drive_odrive_can.py` 구현 (CAN-only 전환)
- 4WS 애커만 키네마틱스 레이어 — 여러 CornerModule 의 소비자
- motor_gui 어댑터 — `state()` dict 를 텔레메트리로 노출
````

- [ ] **Step 4: 수동 텔레옵 검증 (실모터, 바퀴 공중)**

Task 5·6 의 HIL 스모크가 통과한 뒤, 게임패드를 연결하고 위 "텔레옵 실행"을 수행한다.
Expected: □로 arm 시 점프 없음, 좌스틱으로 조향각 추종, RT/LT로 전/후진, ○로 즉시 정지,
스틱을 놓으면(중립) 구동 0, 입력 끊기면 워치독으로 구동 0.

- [ ] **Step 5: 커밋**

```bash
cd /home/light/Defence_Robot
git add motor_control/corner_module/teleop_dualsense.py motor_control/corner_module/README.md
git commit -m "feat(corner_module): DualSense 텔레옵 데모 진입점 + README"
```

---

## 전체 회귀 (모든 태스크 완료 후)

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest corner_module/tests/ -v`
Expected: PASS (23 passed) — 순수 로직 전부. 하드웨어 드라이버(Task 5,6,9)는 HIL 스모크로 별도 검증.
