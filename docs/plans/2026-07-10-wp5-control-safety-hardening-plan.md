# WP5.1 Control and Safety Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

> **실행 상태 (2026-07-11):** Tasks 1~8 소프트웨어 완료. commit `49831bb`의 Jetson
> software-only FAKE는 PASS지만 실기 증거가 아니다. production launch gate는 commit
> `b715ba7`, workspace-independent 계약시험은 commit `60a813f`에 반영됐다. Task 9의
> Phase A·B 실기 HIL과 생산 `stop_mm` 승인은 대기 중이다.

> **운영 권한 오버라이드 (2026-07-11):** 결합 launch는 기본값 없는 필수
> `stop_mm`을 요구한다. HIL 전에는 바퀴를 든 Phase A 시나리오 1~8에서 통제된 저속으로
> `stop_mm:=200`을 명시할 수 있지만, 이 값과 노드 독립 실행 기본값 `200.0`은 진단/HIL
> 후보일 뿐 생산 승인이 아니다. Phase A 완료 뒤 바퀴를 내리기 전에 별도 사용자 확인을
> 받고, 통제 주행로·단계적 저속·spotter·exclusion zone·물리 E-stop을 갖춘 Phase B
> 시나리오 9에서 50 kg 차체 제동을 실측한다. 생산 명령은 그 결과로 승인된
> `stop_mm:=<HIL-approved-mm>`을 반드시 명시한다. 아래에 남은 과거 단계 예시와 충돌하면
> 이 오버라이드와 Task 9의 Phase 구분을 따른다.

**Goal:** 10모터 차체 제어를 실제 50 Hz 비블로킹 루프로 만들고, US-100·모터 고장을 수동 reset형 E-stop으로 통합하며, WP6용 6바퀴 실측 상태를 50 Hz ROS 토픽으로 제공한다.

**Architecture:** 제어·안전 정책은 `motor_control/`의 순수 Python에 두고 pytest로 검증한다. ROS2는 별도 US-100 프로세스와 `chassis_node` 사이의 `/safety_verdict`, WP6으로 나가는 `/wheel_states`, 기존 `/cmd_vel`·로봇팔 계약을 전달하는 얇은 껍데기다. `ChassisManager`가 CAN과 실제 E-stop 집행의 유일한 소유자다.

**Tech Stack:** Python 3.10, pytest, python-can 4.6.1, pyserial, ROS2 Humble/rclpy, rosidl/ament_cmake, Docker, SocketCAN 500 kbps

## Global Constraints

- 설계 정본은 `docs/specs/2026-07-10-wp5-control-safety-hardening-design.md`다.
- 단일 `can0` 500 kbps, AK45-36 ×4 + ODrive/BL70200 ×6 구성을 변경하지 않는다.
- 제어·건강 판정은 50 Hz, 한 tick 예산은 20 ms다.
- 주기 `tick()`과 `state()`에서는 블로킹 CAN/serial 호출을 금지한다.
- US-100 물리 측정은 별도 프로세스에서 약 5~10 Hz로 수행한다.
- `MOTION_HOLD`는 자동복구, `ESTOP`은 수동 reset 후 별도 arm이다.
- `INVALID_READING`은 정상 통과하고, 유효 거리 `< stop_mm` 또는 확정 `NO_RESPONSE`만 센서 E-stop이다.
- 실기 기본은 `safety_required=true`; 우회는 FAKE·벤치에서 명시적으로만 허용한다.
- `safety_topic_timeout`의 운영 기본값·허용 최솟값은 0.75초다(최악 거리+생존 sample
  0.4초 + 스케줄링·DDS 여유 0.35초).
- ODrive 정본은 pp=10, cpr=60, bandwidth=30, vel_gain=0.12, vel_integrator_gain=0.2, node 11~16이다.
- 기존 사용자 소유 untracked 파일 `.claude/settings.json`, `.codex/`, `docs/creativeEngineering/`와 Jetson `motor_control/vision/tests/`를 건드리지 않는다.
- 테스트는 반드시 RED 실패를 확인한 뒤 최소 구현으로 GREEN을 만든다.
- Phase A 실기 HIL 전에는 사용자에게 바퀴 부양·48V 물리 E-stop·10모터·US-100 준비를
  요청한다. Phase B는 바퀴를 내리기 직전에 지상주행 조건과 사용자 승인을 새로 받는다.
- 결합 실기 launch는 `stop_mm` 생략을 허용하지 않는다. 노드 수준 `200.0` 기본값은
  진단과 통제된 저속 pre-HIL 후보 전용이며 생산값으로 승인하지 않는다.

---

## File Map

| 경로 | 책임 |
|---|---|
| `motor_control/corner_module/drive_odrive_can.py` | ODrive CAN 수신을 bounded non-blocking drain으로 변경 |
| `motor_control/corner_module/steer_ak40.py` | AK 주기 poll을 non-blocking으로 변경 |
| `motor_control/chassis/safety_interlock.py` | `RUN/MOTION_HOLD/ESTOP` latch·reset 순수 상태머신 |
| `motor_control/chassis/telemetry.py` | 6바퀴·차체 immutable snapshot 생성 |
| `motor_control/chassis/chassis_manager.py` | interlock·watchdog·외부 안전·전체 E-stop의 유일한 집행자 |
| `motor_control/corner_module/corner_module.py` | ODrive stale/axis error 검사와 component fault reset |
| `motor_control/safety_us100/verdict.py` | US-100 상태·판정 dataclass와 상수 |
| `motor_control/safety_us100/us100.py` | 거리 0x55 + 생존 0x50 트랜잭션 |
| `motor_control/safety_us100/safety_monitor.py` | 연속 실패·CHECKING·NO_RESPONSE 정책 |
| `ros2/src/powertrain_msgs/` | 내부 `SafetyVerdict`, `WheelState`, `WheelStates` ROS 계약 |
| `ros2/src/powertrain_ros/powertrain_ros/us100_safety_node.py` | 블로킹 UART를 차체와 격리해 verdict 발행 |
| `ros2/src/powertrain_ros/powertrain_ros/message_adapter.py` | 순수 snapshot/verdict를 ROS 메시지로 변환 |
| `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py` | 안전 구독·freshness·reset service·50 Hz wheel 발행 |
| `ros2/src/powertrain_ros/launch/wp5_control.launch.py` | safety+chassis 공동 기동 |

---

### Task 1: Make periodic CAN I/O non-blocking

**Files:**
- Modify: `motor_control/corner_module/drive_odrive_can.py:91-159`
- Modify: `motor_control/corner_module/steer_ak40.py:55-62`
- Modify: `motor_control/corner_module/tests/test_corner_module.py:197-410`

**Interfaces:**
- Consumes: existing `DriveOdriveCan.tick()`, `SteerAk40.tick()`, node-filtered SocketCAN sockets.
- Produces: `DriveOdriveCan._drain_available(max_frames: int = 16) -> int`; periodic `tick()` calls only `recv(timeout=0.0)`.

- [ ] **Step 1: Write failing ODrive non-blocking and bounded-drain tests**

Add receive-call recording to `_FakeCanBus` and the following tests:

```python
class _FakeCanBus:
    def __init__(self, rx=None):
        self.sent = []
        self._rx = list(rx or [])
        self.recv_timeouts = []
        self.shutdown_called = False

    def send(self, msg):
        self.sent.append(msg)

    def recv(self, timeout=0.0):
        self.recv_timeouts.append(timeout)
        return self._rx.pop(0) if self._rx else None

    def shutdown(self):
        self.shutdown_called = True


def test_can_drive_periodic_tick_only_uses_nonblocking_recv():
    node = 11
    bus = _FakeCanBus(rx=[_enc(node, 0.0, 1.25), _iq(node, 0.2, 0.3)])
    d = DriveOdriveCan(node_id=node, bus=bus)
    d.connect()
    d.tick()
    assert bus.recv_timeouts
    assert set(bus.recv_timeouts) == {0.0}
    assert d.state()["actual_vel"] == pytest.approx(1.25)


def test_can_drive_periodic_drain_is_bounded_to_16_frames():
    node = 11
    bus = _FakeCanBus(rx=[_enc(node, float(i), float(i)) for i in range(20)])
    d = DriveOdriveCan(node_id=node, bus=bus)
    d.connect()
    d.tick()
    assert len(bus.recv_timeouts) == 16
    assert len(bus._rx) == 4
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
docker run --rm -v "$PWD:/workspace" \
  -w /workspace/motor_control powertrain-sw:dev \
  python3 -m pytest corner_module/tests/test_corner_module.py \
  -k 'periodic_tick_only or periodic_drain_is_bounded' -v
```

Expected: FAIL because current `_poll(0.004)` records a blocking timeout and has no 16-frame bound.

- [ ] **Step 3: Implement bounded non-blocking ODrive drain**

Refactor receive parsing and periodic tick to:

```python
    def _handle_rx(self, m) -> bool:
        if m.is_extended_id or m.is_remote_frame:
            return False
        if (m.arbitration_id >> 5) != self._node_id:
            return False
        cmd = m.arbitration_id & 0x1F
        if cmd == _HEARTBEAT and len(m.data) >= 5:
            self._axis_error = struct.unpack("<I", m.data[0:4])[0]
            self._axis_state = m.data[4]
        elif cmd == _GET_ENCODER_ESTIMATES and len(m.data) >= 8:
            self._actual_vel = struct.unpack("<ff", m.data[0:8])[1]
        elif cmd == _GET_IQ and len(m.data) >= 8:
            self._cur_a = struct.unpack("<ff", m.data[0:8])[1]
        else:
            return False
        self._last_rx_ms = time.monotonic() * 1000.0
        return True

    def _drain_available(self, max_frames: int = 16) -> int:
        handled = 0
        for _ in range(max_frames):
            msg = self._bus.recv(timeout=0.0)
            if msg is None:
                break
            if self._handle_rx(msg):
                handled += 1
        return handled

    def tick(self) -> None:
        self._drain_available()
        self._send(_SET_INPUT_VEL, struct.pack("<ff", self._target_vel, 0.0))
        self._send(_GET_ENCODER_ESTIMATES, rtr=True)
        self._send(_GET_IQ, rtr=True)
```

Keep the blocking `_poll(0.1)` path only for `arm()` and implement it using `_handle_rx()` so parsing has one source.

- [ ] **Step 4: Write and verify the AK periodic non-blocking test**

Extend `_StubAk` and add:

```python
class _StubAk:
    def __init__(self, poll_result):
        self._poll_result = poll_result
        self.poll_timeouts = []
        self.pos_out_deg = 5.0
        self.cur_a = 0.1
        self.fault = 0

    def poll(self, timeout=0.0):
        self.poll_timeouts.append(timeout)
        return self._poll_result

    def send_pos_out(self, deg):
        self.pos_out_deg = deg


def test_steer_periodic_tick_uses_nonblocking_poll():
    from corner_module.steer_ak40 import SteerAk40
    s = SteerAk40(motor_id=1)
    s._ak = _StubAk(poll_result=True)
    s.tick()
    assert s._ak.poll_timeouts == [0.0]
```

Run the single test. Expected RED: current code records `[0.005]`. Change `SteerAk40.tick()` to
`got = self._ak.poll(timeout=0.0)`, then rerun for GREEN.

- [ ] **Step 5: Run the complete corner-module suite**

Run:

```bash
docker run --rm -v "$PWD:/workspace" \
  -w /workspace/motor_control powertrain-sw:dev \
  python3 -m pytest corner_module/tests/ -v
```

Expected: all corner tests PASS; existing arm, telemetry parsing and estop frame tests remain green.

- [ ] **Step 6: Commit Task 1**

```bash
git add motor_control/corner_module/drive_odrive_can.py \
  motor_control/corner_module/steer_ak40.py \
  motor_control/corner_module/tests/test_corner_module.py
git commit -m "perf(can): make periodic motor polling nonblocking"
```

---

### Task 2: Add the pure SafetyInterlock state machine

**Files:**
- Create: `motor_control/chassis/safety_interlock.py`
- Create: `motor_control/chassis/tests/test_safety_interlock.py`
- Modify: `motor_control/chassis/__init__.py`

**Interfaces:**
- Consumes: monotonic clock injected as `clock: Callable[[], float]`.
- Produces: constants `RUN`, `MOTION_HOLD`, `ESTOP`; immutable `SafetySnapshot`; class `SafetyInterlock` with the five spec methods.

- [ ] **Step 1: Write failing interlock lifecycle tests**

Create `test_safety_interlock.py` with:

```python
from chassis.safety_interlock import ESTOP, MOTION_HOLD, RUN, SafetyInterlock


class FakeClock:
    def __init__(self):
        self.t = 10.0

    def __call__(self):
        return self.t


def test_hold_auto_clears_without_latching():
    interlock = SafetyInterlock()
    interlock.set_motion_hold("cmd_timeout", True, "no command")
    assert interlock.snapshot().state == MOTION_HOLD
    interlock.set_motion_hold("cmd_timeout", False)
    assert interlock.snapshot().state == RUN


def test_estop_latches_first_cause_and_is_idempotent():
    clock = FakeClock()
    interlock = SafetyInterlock(clock=clock)
    interlock.trip_estop("manual", "circle button")
    clock.t = 20.0
    interlock.trip_estop("manual", "repeat")
    snap = interlock.snapshot()
    assert snap.state == ESTOP
    assert snap.first_source == "manual"
    assert snap.first_detail == "circle button"
    assert snap.tripped_at_s == 10.0


def test_active_condition_rejects_reset_then_allows_it_after_clear():
    interlock = SafetyInterlock()
    interlock.set_estop_condition("us100", True, "too close")
    assert interlock.reset_estop() is False
    interlock.set_estop_condition("us100", False)
    assert interlock.snapshot().state == ESTOP
    assert interlock.reset_estop() is True
    assert interlock.snapshot().state == RUN


def test_hold_clear_does_not_clear_estop():
    interlock = SafetyInterlock()
    interlock.set_motion_hold("mission", True)
    interlock.trip_estop("motor_fault", "node 12")
    interlock.set_motion_hold("mission", False)
    assert interlock.snapshot().state == ESTOP
```

- [ ] **Step 2: Run and verify RED**

Run `python3 -m pytest chassis/tests/test_safety_interlock.py -v` in `powertrain-sw:dev`.
Expected: collection ERROR with `ModuleNotFoundError: chassis.safety_interlock`.

- [ ] **Step 3: Implement the minimal interlock**

Create:

```python
import time
from dataclasses import dataclass
from typing import Optional, Tuple

RUN = "RUN"
MOTION_HOLD = "MOTION_HOLD"
ESTOP = "ESTOP"


@dataclass(frozen=True)
class SafetySnapshot:
    state: str
    estop_latched: bool
    first_source: Optional[str]
    first_detail: str
    tripped_at_s: Optional[float]
    active_estop_sources: Tuple[str, ...]
    hold_sources: Tuple[str, ...]


class SafetyInterlock:
    def __init__(self, clock=None):
        self._clock = clock or time.monotonic
        self._holds = {}
        self._active_estops = {}
        self._latched = False
        self._first_source = None
        self._first_detail = ""
        self._tripped_at_s = None

    def set_motion_hold(self, source, active, detail=""):
        if active:
            self._holds[source] = detail
        else:
            self._holds.pop(source, None)

    def set_estop_condition(self, source, active, detail=""):
        if active:
            self._active_estops[source] = detail
            self.trip_estop(source, detail)
        else:
            self._active_estops.pop(source, None)

    def trip_estop(self, source, detail=""):
        if not self._latched:
            self._latched = True
            self._first_source = source
            self._first_detail = detail
            self._tripped_at_s = self._clock()

    def reset_estop(self):
        if self._active_estops:
            return False
        self._latched = False
        self._first_source = None
        self._first_detail = ""
        self._tripped_at_s = None
        return True

    def snapshot(self):
        state = ESTOP if self._latched else MOTION_HOLD if self._holds else RUN
        return SafetySnapshot(
            state=state,
            estop_latched=self._latched,
            first_source=self._first_source,
            first_detail=self._first_detail,
            tripped_at_s=self._tripped_at_s,
            active_estop_sources=tuple(sorted(self._active_estops)),
            hold_sources=tuple(sorted(self._holds)),
        )
```

- [ ] **Step 4: Run GREEN and the whole chassis suite**

Run the focused file, then `python3 -m pytest chassis/tests/ -v`. Expected: both commands PASS.

- [ ] **Step 5: Commit Task 2**

```bash
git add motor_control/chassis/safety_interlock.py \
  motor_control/chassis/tests/test_safety_interlock.py \
  motor_control/chassis/__init__.py
git commit -m "feat(safety): add latched chassis interlock"
```

---

### Task 3: Model US-100 valid, invalid, checking and no-response states

**Files:**
- Modify: `motor_control/safety_us100/verdict.py`
- Modify: `motor_control/safety_us100/evaluator.py`
- Modify: `motor_control/safety_us100/us100.py`
- Modify: `motor_control/safety_us100/fake_sensor.py`
- Modify: `motor_control/safety_us100/safety_monitor.py`
- Modify: `motor_control/safety_us100/tests/test_safety.py`
- Modify: `motor_control/safety_us100/demo.py`

**Interfaces:**
- Produces: `SensorReading(status, distance_mm, detail)`; `Verdict(status, distance_mm, estop_required, consecutive_failures, detail)`; constants `CHECKING`, `VALID`, `INVALID_READING`, `NO_RESPONSE`.
- `Us100Sensor.read() -> SensorReading`; `SafetyMonitor.tick() -> None`; `SafetyMonitor.verdict() -> Verdict`.

- [ ] **Step 1: Replace the old safe/warn/stop tests with explicit state-policy tests**

Write tests that use `FakeUs100` with structured readings:

```python
from safety_us100.verdict import (
    CHECKING, INVALID_READING, NO_RESPONSE, VALID, SensorReading,
)


def test_far_valid_distance_does_not_request_estop():
    mon = SafetyMonitor(FakeUs100([SensorReading(VALID, 500.0, "distance")]), SafetyConfig())
    mon.tick()
    verdict = mon.verdict()
    assert verdict.status == VALID
    assert verdict.estop_required is False


def test_near_valid_distance_requests_estop():
    mon = SafetyMonitor(FakeUs100([SensorReading(VALID, 150.0, "distance")]), SafetyConfig())
    mon.tick()
    assert mon.verdict().estop_required is True


def test_invalid_reading_is_normal_when_liveness_responds():
    mon = SafetyMonitor(
        FakeUs100([SensorReading(INVALID_READING, None, "temperature_alive")]),
        SafetyConfig(),
    )
    mon.tick()
    verdict = mon.verdict()
    assert verdict.status == INVALID_READING
    assert verdict.estop_required is False


def test_first_two_misses_are_checking_and_third_is_no_response():
    miss = SensorReading(NO_RESPONSE, None, "liveness_timeout")
    mon = SafetyMonitor(FakeUs100([miss, miss, miss]), SafetyConfig(fail_stop_count=3))
    states = []
    for _ in range(3):
        mon.tick()
        states.append((mon.verdict().status, mon.verdict().estop_required))
    assert states == [(CHECKING, False), (CHECKING, False), (NO_RESPONSE, True)]


def test_alive_response_resets_consecutive_failures():
    miss = SensorReading(NO_RESPONSE, None, "timeout")
    alive = SensorReading(INVALID_READING, None, "temperature_alive")
    mon = SafetyMonitor(FakeUs100([miss, miss, alive, miss]), SafetyConfig())
    for _ in range(4):
        mon.tick()
    assert mon.verdict().status == CHECKING
    assert mon.verdict().consecutive_failures == 1
```

- [ ] **Step 2: Run monitor tests and verify RED**

Run `python3 -m pytest safety_us100/tests/test_safety.py -v`. Expected: import/constructor failures because structured status types do not exist.

- [ ] **Step 3: Implement the status dataclasses and monitor policy**

Use:

```python
from dataclasses import dataclass
from typing import Optional

CHECKING = "CHECKING"
VALID = "VALID"
INVALID_READING = "INVALID_READING"
NO_RESPONSE = "NO_RESPONSE"


@dataclass(frozen=True)
class SensorReading:
    status: str
    distance_mm: Optional[float]
    detail: str = ""


@dataclass(frozen=True)
class Verdict:
    status: str
    distance_mm: Optional[float]
    estop_required: bool
    consecutive_failures: int
    detail: str = ""
```

Implement the monitor with one explicit transition table:

```python
from safety_us100.verdict import (
    CHECKING, INVALID_READING, NO_RESPONSE, VALID, Verdict,
)
from safety_us100.evaluator import requires_estop


class SafetyMonitor:
    def __init__(self, sensor, cfg):
        self._sensor = sensor
        self._cfg = cfg
        self._fail_count = 0
        self._verdict = Verdict(CHECKING, None, False, 0, "startup")

    def tick(self):
        reading = self._sensor.read()
        if reading.status == NO_RESPONSE:
            self._fail_count += 1
            confirmed = self._fail_count >= self._cfg.fail_stop_count
            self._verdict = Verdict(
                NO_RESPONSE if confirmed else CHECKING,
                None,
                confirmed,
                self._fail_count,
                reading.detail,
            )
            return

        self._fail_count = 0
        too_close = requires_estop(reading.status, reading.distance_mm, self._cfg)
        self._verdict = Verdict(
            reading.status,
            reading.distance_mm,
            too_close,
            0,
            "too_close" if too_close else reading.detail,
        )

    def verdict(self):
        return self._verdict
```

Replace the old hysteresis evaluator with:

```python
from safety_us100.verdict import VALID


def requires_estop(status, distance_mm, cfg):
    return (
        status == VALID
        and distance_mm is not None
        and distance_mm < cfg.stop_mm
    )
```

Confirmed `NO_RESPONSE` is handled by the monitor's failure counter, not this distance helper.

- [ ] **Step 4: Write fake-serial distance and liveness tests**

Add an injected serial double:

```python
class FakeSerial:
    def __init__(self, responses):
        self.responses = list(responses)
        self.writes = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(bytes(data))

    def flush(self):
        pass

    def read(self, size):
        return self.responses.pop(0) if self.responses else b""


def test_sensor_returns_valid_distance_without_liveness_probe():
    ser = FakeSerial([bytes([0x01, 0xF4])])  # 500 mm
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)
    reading = sensor.read()
    assert reading == SensorReading(VALID, 500.0, "distance")
    assert ser.writes == [b"\xff" * 8 + b"\x55"]


def test_sensor_uses_temperature_as_liveness_after_distance_timeout():
    ser = FakeSerial([b"", bytes([70])])
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)
    reading = sensor.read()
    assert reading.status == INVALID_READING
    assert reading.detail == "temperature_alive"
    assert ser.writes[-1] == b"\xff" * 8 + b"\x50"


def test_sensor_reports_no_response_when_distance_and_liveness_timeout():
    ser = FakeSerial([b"", b""])
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)
    assert sensor.read().status == NO_RESPONSE
```

- [ ] **Step 5: Implement injected serial transactions**

Extend `Us100Sensor.__init__` with `serial_port=None, sleeper=time.sleep`. Add `_request(command, expected)` that resets input, writes `b"\xff" * 8 + bytes([command])`, flushes, sleeps `response_wait=0.1`, and reads the expected byte count. `read()` follows:

```python
    def read(self):
        if self._ser is None:
            return SensorReading(NO_RESPONSE, None, "port_closed")
        try:
            data = self._request(0x55, 2)
            if len(data) >= 2:
                mm = data[-2] * 256 + data[-1]
                if 20 <= mm <= 4000:
                    return SensorReading(VALID, float(mm), "distance")
                return SensorReading(INVALID_READING, None, "out_of_range")
            alive = self._request(0x50, 1)
            if len(alive) >= 1:
                return SensorReading(INVALID_READING, None, "temperature_alive")
            return SensorReading(NO_RESPONSE, None, "liveness_timeout")
        except serial.SerialException:
            return SensorReading(NO_RESPONSE, None, "serial_error")
```

- [ ] **Step 6: Run all US-100 tests and update the demo output**

Run the safety suite. Expected: PASS. Replace the demo print body with:

```python
v = monitor.verdict()
shown = "(없음)" if v.distance_mm is None else f"{int(v.distance_mm)} mm"
print(
    f"거리: {shown}\t상태: {v.status}\t"
    f"연속실패: {v.consecutive_failures}\tESTOP: {v.estop_required}"
)
```

Remove every old `.level` access from executable US-100 code.

- [ ] **Step 7: Commit Task 3**

```bash
git add motor_control/safety_us100
git commit -m "feat(us100): distinguish invalid readings from no response"
```

---

### Task 4: Enforce drive health and resettable component faults

**Files:**
- Modify: `motor_control/corner_module/corner_module.py`
- Modify: `motor_control/corner_module/fake.py`
- Modify: `motor_control/corner_module/tests/test_corner_module.py`

**Interfaces:**
- Produces: `CornerModule.reset_fault() -> bool`; normalized drive state always includes `stale` and `axis_error`.
- Consumes: `drive.state()` keys `target_vel`, `actual_vel`, `cur_a`, optional `stale`, optional `axis_error`.

- [ ] **Step 1: Write failing drive-health and reset tests**

```python
def test_drive_stale_triggers_component_fault():
    d = FakeDrive()
    cm = _make_cm(drive=d)
    cm.connect()
    cm.arm()
    d.stale_flag = True
    cm.tick()
    assert cm.mode == "FAULT"


def test_drive_axis_error_triggers_component_fault():
    d = FakeDrive()
    cm = _make_cm(drive=d)
    cm.connect()
    cm.arm()
    d.axis_error = 0x10
    cm.tick()
    assert cm.mode == "FAULT"


def test_reset_fault_only_moves_component_to_idle():
    cm = _make_cm()
    cm.connect()
    cm.arm()
    cm.estop()
    assert cm.reset_fault() is True
    assert cm.mode == "IDLE"
    assert cm.state()["drive"]["target_vel"] == 0.0
```

- [ ] **Step 2: Run and verify RED**

Run these three tests. Expected: stale/axis tests remain ARMED and `reset_fault` is missing.

- [ ] **Step 3: Normalize FakeDrive and add drive checks**

Add `stale_flag=False`, `axis_error=0` to `FakeDrive` and return them from `state()`:

```python
    def state(self) -> dict:
        return {
            "target_vel": self._target,
            "actual_vel": self._actual,
            "cur_a": self.cur_a,
            "stale": self.stale_flag,
            "axis_error": self.axis_error,
        }
```

In `CornerModule.tick()`, insert this before target push:

```python
        drive_state = self.drive.state()
        if drive_state.get("stale", False):
            logger.error("구동 status stale → estop")
            self.estop()
            return
        if drive_state.get("axis_error", 0) != 0:
            logger.error("구동 axis_error=%s → estop", drive_state["axis_error"])
            self.estop()
            return
```

Implement:

```python
    def reset_fault(self) -> bool:
        if self.mode != "FAULT":
            return False
        self._drive_target = 0.0
        self.drive.set_velocity(0.0)
        self.mode = "IDLE"
        return True
```

Do not call actuator `arm()` from reset.

- [ ] **Step 4: Run all corner tests**

Expected: suite PASS after updating the FakeDrive schema assertion to include `stale` and `axis_error`.

- [ ] **Step 5: Commit Task 4**

```bash
git add motor_control/corner_module/corner_module.py \
  motor_control/corner_module/fake.py \
  motor_control/corner_module/tests/test_corner_module.py
git commit -m "feat(corner): trip on drive health faults"
```

---

### Task 5: Integrate SafetyInterlock and immutable wheel snapshots into ChassisManager

**Files:**
- Create: `motor_control/chassis/telemetry.py`
- Modify: `motor_control/chassis/chassis_manager.py`
- Modify: `motor_control/chassis/tests/test_chassis_manager.py`
- Modify: `motor_control/chassis/__init__.py`

**Interfaces:**
- Produces: `WheelSnapshot`, `ChassisSnapshot`; `ChassisManager.snapshot()`; `update_external_safety(status, estop_required, detail)`; `set_safety_link_stale(active, detail)`; `reset_estop() -> bool`; `arm() -> bool`; `estop(source="manual", detail="")`.
- Consumes: Task 2 `SafetyInterlock`, Task 4 normalized corner state.

- [ ] **Step 1: Replace monitor-gating tests with hold/E-stop lifecycle tests**

Remove `_Monitor` from chassis tests and add:

```python
def test_cmd_watchdog_is_motion_hold_not_estop():
    clk = FakeClock()
    m = _armed_manager(cfg=ChassisConfig(watchdog_ms=300.0), clock=clk)
    m.set(0.4, 0.0)
    clk.advance(0.5)
    m.tick()
    assert m.mode == "ARMED"
    assert m.snapshot().stop_state == "MOTION_HOLD"
    assert all(d == 0.0 for d in _drive_targets(m).values())


def test_external_estop_latches_after_condition_clears():
    m = _armed_manager()
    m.update_external_safety("VALID", True, "too_close")
    m.tick()
    assert m.mode == "ESTOP"
    m.update_external_safety("VALID", False, "clear")
    assert m.reset_estop() is True
    assert m.mode == "IDLE"
    assert m.arm() is True


def test_arm_rejected_before_estop_reset():
    m = _armed_manager()
    m.estop("manual", "button")
    assert m.arm() is False
    assert m.mode == "ESTOP"


def test_active_safety_condition_rejects_reset():
    m = _armed_manager()
    m.update_external_safety("NO_RESPONSE", True, "sensor")
    m.tick()
    assert m.reset_estop() is False
    assert m.mode == "ESTOP"


class RaisingCorner:
    def __init__(self, wrapped):
        self._wrapped = wrapped
        self.mode = wrapped.mode

    def __getattr__(self, name):
        return getattr(self._wrapped, name)

    def estop(self):
        self.mode = "FAULT"
        raise RuntimeError("stop failed")


def test_estop_continues_after_one_corner_raises():
    corners = _fake_corners()
    m = ChassisManager(corners)
    m.connect()
    m.arm()
    m.corners["front_left"] = RaisingCorner(m.corners["front_left"])
    m.estop("manual")
    assert m.mode == "ESTOP"
    for name, corner in m.corners.items():
        if name != "front_left":
            assert corner.mode == "FAULT"
```

- [ ] **Step 2: Run focused tests and verify RED**

Expected failures: no `snapshot`, no new external safety API, current `arm()` returns `None`, and first corner exception aborts E-stop propagation.

- [ ] **Step 3: Implement immutable telemetry dataclasses**

Create:

```python
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class WheelSnapshot:
    name: str
    corner_mode: str
    drive_turns_per_s: float
    steer_deg: float
    drive_current_a: float
    steer_current_a: float
    drive_stale: bool
    steer_stale: bool
    drive_axis_error: int
    steer_fault: int


@dataclass(frozen=True)
class ChassisSnapshot:
    chassis_mode: str
    stop_state: str
    healthy: bool
    wheels: Tuple[WheelSnapshot, ...]
```

Build wheels in geometry order. Missing optional fake/USB health keys default to safe values.

- [ ] **Step 4: Integrate interlock and best-effort E-stop**

Refactor `ChassisManager` so:

```python
    # __init__ additions after assigning self._now; remove monitor and _verdict
    self._interlock = SafetyInterlock(clock=self._now)
    self._last_estop_error = None

    def arm(self) -> bool:
        if self._interlock.snapshot().estop_latched:
            return False
        for corner in self.corners.values():
            corner.arm()
        self._v = self._omega = 0.0
        self._last_set_ms = self._now_ms()
        self.mode = "ARMED"
        return True

    def update_external_safety(self, status, estop_required, detail=""):
        self._interlock.set_motion_hold("us100_checking", status == "CHECKING", detail)
        self._interlock.set_estop_condition("us100", bool(estop_required), detail)

    def set_safety_link_stale(self, active, detail=""):
        self._interlock.set_estop_condition("safety_topic_stale", active, detail)

    def estop(self, source="manual", detail=""):
        self._interlock.trip_estop(source, detail)
        self._v = self._omega = 0.0
        first_error = None
        for corner in self.corners.values():
            try:
                corner.estop()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        self._last_estop_error = first_error
        self.mode = "ESTOP"

    def disarm(self) -> None:
        for corner in self.corners.values():
            corner.disarm()
        if self.mode != "ESTOP":
            self.mode = "IDLE"

    def reset_estop(self) -> bool:
        if not self._interlock.reset_estop():
            return False
        for corner in self.corners.values():
            corner.reset_fault()
        self.mode = "IDLE"
        return True
```

In `tick()`, express the chassis watchdog as hold source `cmd_watchdog`; apply active E-stop before distribution; use `drive_enabled = snapshot.state == RUN`; post-tick corner `FAULT` calls `estop("corner_fault", names)`.

Use these exact state transitions inside `tick()` before kinematics:

```python
        timed_out = (
            self._last_set_ms is not None
            and self._now_ms() - self._last_set_ms > self.cfg.watchdog_ms
        )
        self._interlock.set_motion_hold("cmd_watchdog", timed_out, "set timeout")
        safety = self._interlock.snapshot()
        if safety.estop_latched:
            if self.mode != "ESTOP":
                self.estop(safety.first_source or "estop", safety.first_detail)
            return
        drive_enabled = safety.state == "RUN"
```

Delete the old synchronous `self.monitor.tick()` branch. Update `state()` to expose
`safety=self._interlock.snapshot()` and make `snapshot()` the structured WP6 source.

- [ ] **Step 5: Add fixed-order snapshot tests**

```python
def test_snapshot_has_six_wheels_in_geometry_order():
    m = _armed_manager()
    snap = m.snapshot()
    assert [wheel.name for wheel in snap.wheels] == [
        "front_left", "front_right", "mid_left",
        "mid_right", "rear_left", "rear_right",
    ]
    assert snap.healthy is True


def test_snapshot_reports_drive_axis_error_unhealthy():
    m = _armed_manager()
    m.corners["front_left"].drive.axis_error = 0x10
    snap = m.snapshot()
    assert snap.healthy is False
    assert snap.wheels[0].drive_axis_error == 0x10
```

- [ ] **Step 6: Run chassis and corner regression suites**

Run both directories in one command. Expected: all PASS after replacing old `mode == "FAULT"` chassis assertions with `mode == "ESTOP"`; component assertions remain `FAULT`.

- [ ] **Step 7: Commit Task 5**

```bash
git add motor_control/chassis motor_control/corner_module/tests/test_corner_module.py
git commit -m "feat(chassis): unify motion hold and latched estop"
```

---

### Task 6: Define local powertrain ROS messages

**Files:**
- Create: `ros2/src/powertrain_msgs/CMakeLists.txt`
- Create: `ros2/src/powertrain_msgs/package.xml`
- Create: `ros2/src/powertrain_msgs/msg/SafetyVerdict.msg`
- Create: `ros2/src/powertrain_msgs/msg/WheelState.msg`
- Create: `ros2/src/powertrain_msgs/msg/WheelStates.msg`
- Modify: `ros2/src/powertrain_ros/package.xml`

**Interfaces:**
- Produces `powertrain_msgs/msg/SafetyVerdict`, `WheelState`, `WheelStates` with the exact fields below.
- Consumes `std_msgs/Header` and nested `powertrain_msgs/WheelState`.

- [ ] **Step 1: Create the three `.msg` files below**

`SafetyVerdict.msg`:

```text
uint8 CHECKING=0
uint8 VALID=1
uint8 INVALID_READING=2
uint8 NO_RESPONSE=3

std_msgs/Header header
uint8 status
float32 distance_mm
bool estop_required
uint32 consecutive_failures
string detail
```

`WheelState.msg`:

```text
string name
string corner_mode
float32 drive_turns_per_s
float32 steer_deg
float32 drive_current_a
float32 steer_current_a
bool drive_stale
bool steer_stale
uint32 drive_axis_error
uint8 steer_fault
```

`WheelStates.msg`:

```text
std_msgs/Header header
string chassis_mode
string stop_state
bool healthy
float32 tick_duration_ms
uint32 overrun_count
WheelState[] wheels
```

- [ ] **Step 2: Create the rosidl package metadata**

`CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(powertrain_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyVerdict.msg"
  "msg/WheelState.msg"
  "msg/WheelStates.msg"
  DEPENDENCIES std_msgs
)

ament_package()
```

Create this `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>powertrain_msgs</name>
  <version>0.1.0</version>
  <description>Internal powertrain safety and wheel telemetry messages</description>
  <maintainer email="nitez0423@gmail.com">ZETIN Powertrain</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Add `powertrain_msgs` runtime dependency to `powertrain_ros`**

Insert this next to the existing message dependency in `powertrain_ros/package.xml`:

```xml
  <exec_depend>robot_arm_msgs</exec_depend>
  <exec_depend>powertrain_msgs</exec_depend>
```

- [ ] **Step 4: Build messages locally in the ROS container and inspect interfaces**

Run without CAN/HIL:

```bash
cd /home/light/ZETIN/robotics/power-train-sw
docker compose -f docker/docker-compose.jetson.yml build powertrain_ros
docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros
docker exec powertrain_ros bash -lc \
  'cd /workspace/ros2 && rm -rf build/powertrain_msgs install/powertrain_msgs && \
   source /opt/ros/humble/setup.bash && colcon build --packages-select powertrain_msgs && \
   source install/setup.bash && ros2 interface show powertrain_msgs/msg/WheelStates'
```

Expected: build succeeds and interface output contains header, chassis_mode, stop_state, healthy, timing fields and `WheelState[] wheels`.

- [ ] **Step 5: Commit Task 6**

```bash
git add ros2/src/powertrain_msgs ros2/src/powertrain_ros/package.xml
git commit -m "feat(ros2): add powertrain safety and wheel messages"
```

---

### Task 7: Add US-100 ROS adapter and harden chassis_node

> **구현 증거 오버라이드:** Task 7의 최초 구현 뒤 commit `b715ba7`에서 결합 실기 launch에
> 기본값 없는 필수 `stop_mm` gate를 추가했고, commit `60a813f`에서 계약시험을 workspace와
> 무관하게 실행하도록 고쳤다. 아래 최초 구현 서술보다 이 override와 현재 launch 코드가
> 운영 권한을 가진다.

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/message_adapter.py`
- Create: `ros2/src/powertrain_ros/powertrain_ros/us100_safety_node.py`
- Create: `ros2/src/powertrain_ros/test/test_message_adapter.py`
- Create: `ros2/src/powertrain_ros/launch/wp5_control.launch.py`
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py`
- Modify: `ros2/src/powertrain_ros/setup.py`
- Modify: `ros2/src/powertrain_ros/package.xml`
- Modify: `docker/Dockerfile.ros`

**Interfaces:**
- Produces `/safety_verdict` reliable depth 1 at configured sample rate; `/wheel_states` at chassis tick rate; `~/reset_estop`.
- Consumes Task 3 `Verdict`, Task 5 `ChassisSnapshot`, Task 6 ROS messages.

- [ ] **Step 1: Write failing pure adapter tests**

`message_adapter.py` functions are `fill_safety_message(msg, verdict, stamp, frame_id="us100_link")` and `fill_wheel_states_message(msg, snapshot, stamp, tick_duration_ms, overrun_count, wheel_factory)`.

Test with lightweight message doubles:

```python
def test_fill_safety_message_uses_nan_for_missing_distance():
    msg = SimpleNamespace(header=SimpleNamespace())
    verdict = SimpleNamespace(
        status="CHECKING", distance_mm=None, estop_required=False,
        consecutive_failures=1, detail="waiting",
    )
    fill_safety_message(msg, verdict, stamp="stamp")
    assert msg.header.stamp == "stamp"
    assert msg.status == 0
    assert math.isnan(msg.distance_mm)
    assert msg.estop_required is False


def test_fill_wheel_states_uses_actual_snapshot_values():
    msg = SimpleNamespace(header=SimpleNamespace())
    wheel = SimpleNamespace(
        name="front_left", corner_mode="ARMED", drive_turns_per_s=1.2,
        steer_deg=3.0, drive_current_a=0.4, steer_current_a=0.2,
        drive_stale=False, steer_stale=False, drive_axis_error=0,
        steer_fault=0,
    )
    snapshot = SimpleNamespace(
        chassis_mode="ARMED", stop_state="RUN", healthy=True, wheels=(wheel,),
    )
    fill_wheel_states_message(
        msg, snapshot, "stamp", 4.5, 2, wheel_factory=SimpleNamespace,
    )
    assert msg.wheels[0].drive_turns_per_s == 1.2
    assert msg.tick_duration_ms == 4.5
    assert msg.overrun_count == 2
```

Expected RED: module missing.

- [ ] **Step 2: Implement adapter functions and make tests GREEN**

Implement the adapter without importing rclpy or hardware code:

```python
import math

_STATUS_CODES = {
    "CHECKING": 0,
    "VALID": 1,
    "INVALID_READING": 2,
    "NO_RESPONSE": 3,
}


def fill_safety_message(msg, verdict, stamp, frame_id="us100_link"):
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.status = _STATUS_CODES[verdict.status]
    msg.distance_mm = (
        float(verdict.distance_mm) if verdict.distance_mm is not None else math.nan
    )
    msg.estop_required = bool(verdict.estop_required)
    msg.consecutive_failures = int(verdict.consecutive_failures)
    msg.detail = verdict.detail
    return msg


def fill_wheel_states_message(
    msg, snapshot, stamp, tick_duration_ms, overrun_count, wheel_factory
):
    msg.header.stamp = stamp
    msg.header.frame_id = "base_link"
    msg.chassis_mode = snapshot.chassis_mode
    msg.stop_state = snapshot.stop_state
    msg.healthy = snapshot.healthy
    msg.tick_duration_ms = float(tick_duration_ms)
    msg.overrun_count = int(overrun_count)
    msg.wheels = []
    for source in snapshot.wheels:
        wheel = wheel_factory()
        for field in (
            "name", "corner_mode", "drive_turns_per_s", "steer_deg",
            "drive_current_a", "steer_current_a", "drive_stale",
            "steer_stale", "drive_axis_error", "steer_fault",
        ):
            setattr(wheel, field, getattr(source, field))
        msg.wheels.append(wheel)
    return msg
```

- [ ] **Step 3: Implement `Us100SafetyNode`**

Implement the node as follows; the only omitted code is the standard `main()` spin/finally shell shown after the class:

```python
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from powertrain_msgs.msg import SafetyVerdict as SafetyVerdictMsg
from powertrain_ros.message_adapter import fill_safety_message

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

from safety_us100.config import SafetyConfig
from safety_us100.safety_monitor import SafetyMonitor
from safety_us100.us100 import Us100Sensor


class Us100SafetyNode(Node):
    def __init__(self):
        super().__init__("us100_safety_node")
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("sample_hz", 5.0)
        self.declare_parameter("stop_mm", 200.0)
        self.declare_parameter("fail_stop_count", 3)
        cfg = SafetyConfig(
            stop_mm=float(self.get_parameter("stop_mm").value),
            fail_stop_count=int(self.get_parameter("fail_stop_count").value),
            port=str(self.get_parameter("port").value),
            baud=int(self.get_parameter("baud").value),
        )
        self.sensor = Us100Sensor(port=cfg.port, baud=cfg.baud)
        self.sensor.open()
        self.monitor = SafetyMonitor(self.sensor, cfg)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            SafetyVerdictMsg, "/safety_verdict", qos,
        )
        hz = max(float(self.get_parameter("sample_hz").value), 0.1)
        self.create_timer(1.0 / hz, self._sample)

    def _sample(self):
        self.monitor.tick()
        msg = SafetyVerdictMsg()
        fill_safety_message(msg, self.monitor.verdict(), self.get_clock().now().to_msg())
        self.publisher.publish(msg)

    def close(self):
        self.sensor.close()


def main(argv=None):
    rclpy.init(args=argv)
    node = Us100SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
```

Keep blocking UART exclusively in this node process. Change the Docker pip line to
`RUN pip3 install --no-cache-dir python-can pyserial` and add console entry point
`us100_safety = powertrain_ros.us100_safety_node:main`.

- [ ] **Step 4: Write chassis-node FAKE safety lifecycle smoke script**

Create no permanent test helper; use ROS CLI after build:

```bash
ros2 run powertrain_ros chassis --ros-args -p fake:=true -p safety_required:=true
ros2 topic pub -r 10 /safety_verdict powertrain_msgs/msg/SafetyVerdict \
  '{status: 1, distance_mm: 500.0, estop_required: false, consecutive_failures: 0, detail: far}'
ros2 service call /chassis_node/arm std_srvs/srv/Trigger '{}'
ros2 topic hz /wheel_states
```

Before implementation, expected failure is unknown message type/topic and missing safety subscription.

- [ ] **Step 5: Harden `ChassisNode`**

Declare and read:

```python
self.declare_parameter("safety_required", True)
self.declare_parameter("safety_topic_timeout", 0.75)
self.declare_parameter("safety_startup_timeout", 1.0)
self._safety_required = bool(self.get_parameter("safety_required").value)
self._safety_topic_timeout = validate_safety_topic_timeout(
    self.get_parameter("safety_topic_timeout").value
)
self._safety_startup_timeout = float(self.get_parameter("safety_startup_timeout").value)
self._started_ms = self._now_ms()
self._last_safety_ms = None
self._overrun_count = 0
```

`validate_safety_topic_timeout()`은 값이 유한하고 0.75초 이상인지 검사하고, 아니면
`ValueError`를 발생시킨다. 0.75초는 최악의 거리 요청+생존 확인 sample 0.4초에 타이머
스케줄링·DDS 전달 여유 0.35초를 더한 운영 최솟값이다. 0.4, 0.5, 0.749, NaN, infinity를
거부하고 0.75를 허용하는 순수 단위시험을 추가한다.

```python
def validate_safety_topic_timeout(value):
    timeout_s = float(value)
    if not math.isfinite(timeout_s) or timeout_s < 0.75:
        raise ValueError(
            "safety_topic_timeout must be finite and at least 0.75 s"
        )
    return timeout_s
```

Create a reliable depth-1 safety subscription, a `WheelStates` publisher, and
`~/reset_estop` service. Use `WheelState` as the adapter's `wheel_factory`.
Construct `ChassisConfig` with `watchdog_ms=self._cmd_timeout * 1000.0` and delete the
node's old `cm.set(0.0, 0.0)` timeout branch, so one ChassisManager watchdog owns the
automatic `MOTION_HOLD` transition.

Required callback mapping:

```python
    def _on_safety_verdict(self, msg):
        status_name = {
            SafetyVerdict.CHECKING: "CHECKING",
            SafetyVerdict.VALID: "VALID",
            SafetyVerdict.INVALID_READING: "INVALID_READING",
            SafetyVerdict.NO_RESPONSE: "NO_RESPONSE",
        }.get(msg.status, "CHECKING")
        self.cm.update_external_safety(status_name, msg.estop_required, msg.detail)
        self.cm.set_safety_link_stale(False)
        self._last_safety_ms = self._now_ms()
```

At every 50 Hz tick, set link stale after the startup/normal timeout, call `cm.tick()`, catch exceptions with `cm.estop("control_exception", str(exc))`, measure duration, increment overrun when duration exceeds 20 ms, and publish `WheelStates` after motor handling.

Use this exact timer body:

```python
    def _tick(self):
        now_ms = self._now_ms()
        if self._safety_required:
            if self._last_safety_ms is None:
                expired = now_ms - self._started_ms > self._safety_startup_timeout * 1000.0
                self.cm.set_safety_link_stale(expired, "safety_startup_timeout")
                if not expired:
                    self.cm.update_external_safety("CHECKING", False, "startup")
            else:
                stale = now_ms - self._last_safety_ms > self._safety_topic_timeout * 1000.0
                self.cm.set_safety_link_stale(stale, "safety_topic_stale")

        started = time.monotonic()
        try:
            self.cm.tick()
        except Exception as exc:
            self.cm.estop("control_exception", str(exc))
        duration_ms = (time.monotonic() - started) * 1000.0
        if duration_ms > 1000.0 / self.cm.cfg.loop_hz:
            self._overrun_count += 1
        msg = WheelStates()
        fill_wheel_states_message(
            msg, self.cm.snapshot(), self.get_clock().now().to_msg(),
            duration_ms, self._overrun_count, WheelState,
        )
        self.pub_wheels.publish(msg)
```

`_srv_reset_estop` calls `cm.reset_estop()` and never calls arm. `_srv_arm` returns failure while latched.

- [ ] **Step 6: Add launch file and package data**

Create:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "stop_mm",
            description="HIL-approved US-100 emergency-stop distance (mm)",
        ),
        Node(
            package="powertrain_ros",
            executable="us100_safety",
            output="screen",
            parameters=[{"stop_mm": LaunchConfiguration("stop_mm")}],
        ),
        Node(package="powertrain_ros", executable="chassis", output="screen"),
    ])
```

`DeclareLaunchArgument("stop_mm")`에는 default를 두지 않는다. `LaunchConfiguration`은
US-100 노드에만 전달하며 chassis 노드에는 전달하지 않는다. 인자를 생략한 결합 launch는
의도적으로 실패한다. 독립 `us100_safety_node`의 `200.0` 기본값은 진단과 통제된 저속
HIL용 임시 후보일 뿐 생산 승인값이 아니다.

Add `("share/" + package_name + "/launch", ["launch/wp5_control.launch.py"])` to
`setup.py` data_files, `<exec_depend>launch_ros</exec_depend>` to package.xml, and the
`us100_safety` console entry point.

- [ ] **Step 7: Build all three ROS packages locally and run FAKE smoke checks**

Rebuild and run on the development host with `fake:=true`:

```bash
docker compose -f docker/docker-compose.jetson.yml build powertrain_ros
docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros
docker exec powertrain_ros bash -lc \
  'cd /workspace/ros2 && source /opt/ros/humble/setup.bash && colcon build && \
   source install/setup.bash && colcon test --packages-select powertrain_ros && \
   colcon test-result --verbose'
```

Then run the FAKE publisher/service/topic-hz sequence. Expected: arm succeeds with fresh far verdict, `/wheel_states` averages 49~51 Hz, close verdict produces `ESTOP`, far verdict alone does not clear it, reset returns IDLE.

Observed evidence at commit `49831bb`: Jetson software-only FAKE PASS — 60초 count 3000,
mean/minimum 5초 window 50.000 Hz, tick p99 0.280 ms, overrun 0, maximum interval 21.453 ms,
publisher-death E-stop 0.753초. 이 결과는 실기 HIL이 아니며 raw FAKE log도 보존되지 않았다.

- [ ] **Step 8: Commit Task 7**

```bash
git add ros2/src/powertrain_ros docker/Dockerfile.ros
git commit -m "feat(ros2): bridge US100 safety and wheel telemetry"
```

---

### Task 8: Migrate non-ROS teleop entry points to the same E-stop semantics

**Files:**
- Create: `motor_control/safety_us100/background_monitor.py`
- Create: `motor_control/safety_us100/tests/test_background_monitor.py`
- Modify: `motor_control/chassis/teleop_dualsense.py`
- Modify: `motor_control/chassis/teleop_server.py`
- Modify: `motor_control/corner_module/teleop_dualsense.py`
- Modify: `motor_control/safety_us100/teleop_odrive_only.py`
- Modify: `motor_control/corner_module/README.md`
- Modify: `motor_control/safety_us100/README.md`

**Interfaces:**
- Produces `BackgroundSafetyMonitor.start()`, `.verdict()`, `.close()` so serial blocking never runs in a 50 Hz teleop loop.
- Consumes Task 3 `SafetyMonitor` and Task 5 `ChassisManager.update_external_safety`.

- [ ] **Step 1: Write failing background-monitor test**

```python
def test_background_monitor_samples_without_blocking_caller():
    sampled = threading.Event()

    class Monitor:
        def tick(self):
            sampled.set()

        def verdict(self):
            return Verdict(VALID, 500.0, False, 0, "far")

    worker = BackgroundSafetyMonitor(Monitor(), period_s=0.01)
    worker.start()
    assert sampled.wait(0.2)
    assert worker.verdict().status == VALID
    worker.close()
```

Expected RED: module missing.

- [ ] **Step 2: Implement a daemon worker with a lock and stop event**

Implement:

```python
import threading
import time


class BackgroundSafetyMonitor:
    def __init__(self, monitor, period_s=0.1):
        self._monitor = monitor
        self._period_s = period_s
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._latest = monitor.verdict()

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="us100-monitor", daemon=True,
        )
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            started = time.monotonic()
            self._monitor.tick()
            with self._lock:
                self._latest = self._monitor.verdict()
            remaining = self._period_s - (time.monotonic() - started)
            self._stop.wait(max(remaining, 0.0))

    def verdict(self):
        with self._lock:
            return self._latest

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
```

Only this thread calls `monitor.tick()`. It never closes the sensor it does not own.

- [ ] **Step 3: Update chassis teleop paths**

Replace `ChassisManager(..., monitor=monitor)` with a background worker. Each 50 Hz loop executes:

```python
verdict = background.verdict()
cm.update_external_safety(verdict.status, verdict.estop_required, verdict.detail)
if square_rising:
    if cm.mode == "ESTOP":
        cm.reset_estop()
    elif cm.mode == "ARMED":
        cm.disarm()
    else:
        cm.arm()
if circle_rising:
    cm.estop("manual", "dualsense")
```

The first square press after E-stop only resets to IDLE; the next press arms.

- [ ] **Step 4: Update single-corner and legacy ODrive demos**

For `corner_module.teleop_dualsense`, use `verdict.estop_required` to call
`cm.estop()` and require square to call `cm.reset_fault()` before a later arm press. For
`safety_us100.teleop_odrive_only`, keep a local `estop_latched` boolean: a hazardous
verdict calls `disarm()` and sets it true; a normal verdict never clears it; square first
clears the boolean while stopped and a later square arms. Remove all `.level` field
accesses.

- [ ] **Step 5: Run teleop mapping, chassis, corner and US-100 suites**

Run:

```bash
docker run --rm -v "$PWD:/workspace" \
  -w /workspace/motor_control powertrain-sw:dev \
  python3 -m pytest chassis/tests corner_module/tests safety_us100/tests -v
```

Expected: all PASS; no test or source outside historical docs accesses `verdict.level`.

- [ ] **Step 6: Commit Task 8**

```bash
git add motor_control/safety_us100 motor_control/chassis/teleop_dualsense.py \
  motor_control/chassis/teleop_server.py \
  motor_control/corner_module/teleop_dualsense.py \
  motor_control/corner_module/README.md
git commit -m "refactor(safety): unify teleop estop behavior"
```

---

### Task 9: Verify, document, deploy and perform HIL

Tasks 1~8에서는 실물 모터·US-100 HIL을 수행하지 않는다. 순수 Python, 로컬 ROS build,
FAKE ROS 검증과 전체 코드 리뷰를 먼저 끝낸 뒤, 본 Task에서 Jetson 배포와 실물 HIL을
한 번에 수행한다.

**Files:**
- Modify: `docs/specs/2026-07-10-wp5-control-safety-hardening-design.md`
- Modify: `docs/plans/2026-07-10-wp5-control-safety-hardening-plan.md`
- Modify: `docs/plans/2026-07-02-autonomous-driving-kickoff.md`
- Modify: `docs/reports/2026-07-10-project-and-jetson-state.md`
- Modify: `docs/specs/2026-05-25-us100-safety-module-design.md` (SUPERSEDED banner only; preserve body)
- Modify: `docs/plans/2026-05-25-us100-safety-module-plan.md` (SUPERSEDED banner only; preserve body)
- Modify: `ros2/README.md`
- Modify: `README.md`
- Modify: `AGENTS.md`
- Modify: `.claude/CLAUDE.md`
- Modify: `.claude/AGENTS.md`
- Create: `docs/reports/2026-07-10-wp5-control-safety-hil.md`

**Interfaces:**
- Consumes all Task 1~8 artifacts.
- Produces verified repository/Jetson documentation and the go/no-go decision for command authority, L515 and WP6.

- [ ] **Step 1: Run the supported local automatic regression suites**

```bash
docker run --rm -v "$PWD:/workspace" \
  -w /workspace/motor_control powertrain-sw:dev \
  python3 -m pytest chassis/tests corner_module/tests safety_us100/tests -v

docker run --rm -v "$PWD:/workspace" \
  -w /workspace powertrain-sw:dev \
  python3 -m pytest motor_gui/tests -v
```

Expected: both explicitly supported automatic suites PASS with zero failures and zero collection
errors. Do not run unbounded recursive collection under `motor_control/`; filenames such as
`odrive_*_test.py` and `realsense_test.py` are real-hardware scripts, not automatic pytest targets.

- [ ] **Step 2: Run full ROS build and tests on Jetson without motors**

Commit Tasks 1~8 on the current feature branch, record the immutable commit, and push that branch
without assuming `main`. Verify that the remote branch resolves to the same commit, then encode the
two values for safe transfer into the separate Jetson SSH shell:

```bash
set -eu
test -n "${JETSON_SSH_PASS:-}"
BRANCH=$(git branch --show-current)
test -n "$BRANCH"
git check-ref-format --branch "$BRANCH" >/dev/null
DEPLOY_COMMIT=$(git rev-parse HEAD)
git cat-file -e "${DEPLOY_COMMIT}^{commit}"
test "$(git rev-parse "${BRANCH}^{commit}")" = "$DEPLOY_COMMIT"
git push -u origin "$BRANCH"
REMOTE_COMMIT=$(
  git ls-remote --heads origin "refs/heads/$BRANCH" |
    awk 'NR == 1 {print $1}'
)
test "$REMOTE_COMMIT" = "$DEPLOY_COMMIT"
BRANCH_B64=$(printf '%s' "$BRANCH" | base64 -w0)
DEPLOY_COMMIT_B64=$(printf '%s' "$DEPLOY_COMMIT" | base64 -w0)
```

On Jetson, preserve existing untracked `motor_control/vision/tests/`, confirm there are no
overlapping tracked changes, fetch the same feature branch, and fast-forward to that exact commit.
The base64 values are single-quoted in the remote command, and the quoted heredoc prevents local
expansion; the remote shell decodes and validates both values before using them. Before rebuilding,
require `git rev-parse HEAD` to equal the transferred `$DEPLOY_COMMIT`; do not test an implicit
branch tip that may have moved. Rebuild `powertrain_ros` and run `colcon test-result --verbose`.

```bash
SSHPASS="$JETSON_SSH_PASS" sshpass -e ssh zetin@jetson-orin.local \
  "BRANCH_B64='$BRANCH_B64' DEPLOY_COMMIT_B64='$DEPLOY_COMMIT_B64' bash -s" \
  <<'JETSON'
set -eu
BRANCH=$(printf '%s' "$BRANCH_B64" | base64 -d)
DEPLOY_COMMIT=$(printf '%s' "$DEPLOY_COMMIT_B64" | base64 -d)
test -n "$BRANCH"
test -n "$DEPLOY_COMMIT"
git check-ref-format --branch "$BRANCH" >/dev/null
test "${#DEPLOY_COMMIT}" -eq 40
case "$DEPLOY_COMMIT" in
  *[!0-9a-f]*) exit 1 ;;
esac
cd ~/power-train-sw
test -z "$(git status --porcelain --untracked-files=no)"
git fetch --no-tags origin "refs/heads/$BRANCH"
test "$(git rev-parse 'FETCH_HEAD^{commit}')" = "$DEPLOY_COMMIT"
git cat-file -e "${DEPLOY_COMMIT}^{commit}"
git merge --ff-only "$DEPLOY_COMMIT"
test "$(git rev-parse HEAD)" = "$DEPLOY_COMMIT"
JETSON
```

Expected: `robot_arm_msgs`, `powertrain_msgs`, `powertrain_ros` build; all ROS tests PASS.

- [ ] **Step 3: Run FAKE ROS safety and 50 Hz acceptance test**

Start a fresh FAKE chassis with no competing process, publish far verdict at 10 Hz, arm, publish `/cmd_vel`, and measure `/wheel_states` for 60 seconds. Then publish close, far, reset and arm in order.

Expected:

```text
far + fresh verdict       → RUN
60 s wheel rate           → mean 49~51 Hz, no sustained <48 Hz
close verdict             → ESTOP
far verdict after trip    → remains ESTOP
reset                     → IDLE, wheels stopped
arm after reset           → ARMED
publisher killed          → after age >0.75 s, by next 50 Hz tick (nominal 0.75–0.77 s) ESTOP
```

- [ ] **Step 4: Request Phase A HIL setup from the user before touching hardware**

Ask for:

```text
1. 바퀴 6개 지면에서 부양
2. 48V 물리 E-stop 접근 가능
3. AK 1~4 + ODrive 11~16 전원/배선 완료
4. US-100 /dev/ttyTHS1 연결
5. 테스트 중 주변 인원에게 구동 고지
```

Do not proceed until the user confirms. This approval covers only Phase A scenarios 1~8 with all
six wheels lifted. It does not authorize lowering the chassis or ground motion.

- [ ] **Step 5: Capture pre-HIL state**

Record `ip -details -statistics link show can0`, zombie process list, node heartbeat presence, and `/wheel_states` baseline. Bring up can0 with the repository script only after confirming loopback is off.

- [ ] **Step 6: Execute Phase A scenarios 1~8 from spec §10.3 with wheels lifted**

For each scenario record timestamp, expected result, observed result, tick rate, tick p99, CAN counter deltas, safety status, E-stop source and motor visual behavior in `docs/reports/2026-07-10-wp5-control-safety-hil.md`.

The pre-HIL candidate command must pass an explicit provisional value and remain at controlled low
speed. Omitting `stop_mm` must fail:

```bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=200
```

`200` here is provisional only, never a production approval.

- [ ] **Step 7: Request separate Phase B approval, execute scenario 9 and determine production `stop_mm`**

After scenarios 1~8, stop and disarm. Before lowering the 50 kg chassis, obtain a new explicit user
confirmation for a controlled corridor, staged low speeds beginning at the minimum, a spotter,
an exclusion zone and an immediately accessible physical E-stop. Do not lower the wheels or start
scenario 9 without every condition. Record the new approval and lowering time separately from
Phase A.

At controlled low speeds, measure worst sensor interval, processing delay and physical stopping distance. Set:

```text
stop_mm ≥ 최고속도 × (최악 센서주기 + 처리지연) + 실측 제동거리 + 안전여유
```

Document the chosen value and test speed. Do not keep 200 mm merely because it was the historical
default. Revalidate the chosen value, then use only this production command form:

```bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=<HIL-approved-mm>
```

- [ ] **Step 8: Update authority documents**

Update the kickoff plan with WP5.1 completion evidence, measured rate, E-stop terminology,
`/wheel_states` and safety flow. Update the current design/plan status, ROS README commands, root
README feature table, current-state report, root AGENTS, `.claude/CLAUDE.md`, and
`.claude/AGENTS.md`. The authority update must explain all of the following:

- `RUN` / `MOTION_HOLD` / latched `ESTOP`, reset, and separate arm semantics
- the hybrid architecture: pure-Python policy and motor ownership, ROS transport/freshness, and
  the separate blocking US-100 process
- why that split exists: deterministic 50 Hz chassis work despite UART latency, one final E-stop
  authority, and reusable non-ROS teleop semantics
- why the 2026-05-25 `safe/warn/stop` + `Verdict.level` + startup/`None`→`stop` model is retired,
  with both old design/plan documents marked SUPERSEDED and linked to the current WP5 authority

Preserve historical completion banners but add a newer override rather than rewriting old HIL
history.

- [ ] **Step 9: Run final verification and commit documentation**

Run fresh pure tests, ROS build/test, `git diff --check`, and inspect `git status`. Stage only the
listed tracked Task 9 files; verify the index before committing. Push the current feature branch,
never a hard-coded `main`, and record the exact documentation commit:

```bash
git add -- README.md AGENTS.md .claude/CLAUDE.md .claude/AGENTS.md ros2/README.md \
  docs/specs/2026-07-10-wp5-control-safety-hardening-design.md \
  docs/plans/2026-07-10-wp5-control-safety-hardening-plan.md \
  docs/reports/2026-07-10-project-and-jetson-state.md \
  docs/specs/2026-05-25-us100-safety-module-design.md \
  docs/plans/2026-05-25-us100-safety-module-plan.md \
  docs/plans/2026-07-02-autonomous-driving-kickoff.md \
  docs/reports/2026-07-10-wp5-control-safety-hil.md
git diff --cached --name-only
git commit -m "docs: record WP5 control safety HIL"
DOC_COMMIT=$(git rev-parse HEAD)
BRANCH=$(git branch --show-current)
test -n "$BRANCH"
git push -u origin "$BRANCH"
test "$(git rev-parse HEAD)" = "$DOC_COMMIT"
```

- [ ] **Step 10: Move to the next approved architecture track**

After WP5.1 HIL passes, create the separate command-authority spec. Then create the L515 depth-image-default spec. WP6 starts only after both interfaces are fixed, so its odometry implementation does not build on ambiguous `/cmd_vel` ownership or an unnecessarily heavy point-cloud pipeline.

---

## Plan Self-Review Checklist

- Spec §5 non-blocking CAN: Task 1
- Spec §6 stop terminology/latch/reset: Tasks 2, 4, 5, 7, 8
- Spec §7 US-100 states/liveness/residual risk: Tasks 3, 7, 9
- Spec §8 ROS messages/services: Tasks 6, 7
- Spec §9 error handling and ordering: Tasks 4, 5, 7
- Spec §10 pure/ROS/HIL tests: Tasks 1~9
- Spec §11 completion criteria and docs: Task 9
- Excluded command authority/L515/WP6 tracks: Task 9 Step 10
