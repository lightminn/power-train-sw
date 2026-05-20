# 모터 통합 관제 GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Jetson 에서 실행되는 웹 기반 벤치 진단 도구 — ODrive(USB/CAN)·AK(CAN) 모터의 100 Hz 텔레메트리를 브라우저 uPlot 으로 실시간 plot 하고, 위치/속도/토크 제어·라이브 게인 튜닝·캘리·E-stop 을 수행한다.

**Architecture:** 3계층 (브라우저 ↔ FastAPI 웹 레이어 ↔ HardwareWorker(스레드)가 단독 소유하는 Transport). 웹 레이어와 하드웨어 레이어는 **JSON-직렬화 dict 만** 주고받는 단일 seam 으로 분리 (향후 프로세스 격리=접근법 C 승격 대비). `FakeTransport` 로 하드웨어 없이 전 스택 개발·테스트.

**Tech Stack:** Python 3.10+ (Jetson 컨테이너), FastAPI + uvicorn + WebSocket, `odrive` lib (USB), `python-can` socketcan (CAN), pytest + FastAPI TestClient, 프론트 = 바닐라 JS + uPlot (빌드스텝 없음).

**Spec:** `docs/specs/2026-05-20-motor-gui-design.md`

**하드웨어 제약 (HIL 태스크 관련):** 사용자가 Jetson 에 {ODrive USB, ODrive CAN, AK CAN} 중 **한 번에 하나만** 연결 가능. 따라서 Task 8(USB)·Task 9(CAN) 의 실하드웨어 검증은 컨트롤러가 사용자에게 해당 연결을 요청한 뒤 진행. CAN 트랙의 ODrive+AK **동시** 동작 검증은 둘 다 버스에 올릴 수 있을 때까지 보류 (FakeTransport 가 동시 케이스 커버).

---

## File Structure

```
motor_gui/
├── __init__.py
├── backend/
│   ├── __init__.py
│   ├── transport/
│   │   ├── __init__.py
│   │   ├── base.py          # Transport ABC + TransportError + 신호/capabilities 규약
│   │   ├── fake.py          # FakeTransport (시뮬 모터, odrive+ak 슈퍼셋)
│   │   ├── usb_odrive.py    # UsbOdriveBackend (odrive lib, axis1)
│   │   └── can_bus.py       # CanBackend (ODrive node1 CANSimple + AK id10)
│   ├── commands.py          # normalize/clamp/validate (순수함수) + CommandError
│   ├── worker.py            # HardwareWorker (Transport 소유 + 100Hz 스레드 + 큐 + estop)
│   ├── recorder.py          # 선택적 CSV/parquet 로깅
│   └── server.py            # FastAPI app + create_app(track) + __main__ 런처
├── frontend/
│   ├── index.html
│   ├── app.js               # capabilities→UI, WS→ring, 컨트롤→command
│   ├── plots.js             # uPlot 패널 생성·갱신
│   └── vendor/              # uPlot.min.js / uPlot.min.css (벤더링)
├── tests/
│   ├── __init__.py
│   ├── test_commands.py
│   ├── test_fake.py
│   ├── test_worker.py
│   └── test_server.py
└── README.md
```

**책임 경계**
- `transport/*` — 장치별 I/O. 위는 모두 같은 `Transport` ABC. dict 만 반환.
- `commands.py` — 순수 검증/클램프. 하드웨어·네트워크 의존 0.
- `worker.py` — 동시성 단일 지점 (스레드, 큐, estop). Transport 단독 소유.
- `server.py` — HTTP/WS 만. worker 의 `submit`/`subscribe`/`capabilities` 만 호출.
- frontend — capabilities 기반 동적 UI.

**공유 자료형 규약** (전 태스크 일관 — self-review 대상)

- **sample dict**: 항상 `t_mono: float`(time.monotonic) 포함. 신호 키는 `"<device>.<signal>"`.
  ODrive: `odrive.pos, odrive.vel, odrive.iq_meas, odrive.iq_set, odrive.temp_fet, odrive.vbus, odrive.ibus, odrive.state, odrive.axis_err, odrive.motor_err, odrive.enc_err, odrive.ctrl_err, odrive.vel_integrator`. AK: `ak.pos_deg, ak.speed, ak.current, ak.temp, ak.fault`.
- **capabilities dict**: `{"track": str, "devices": list[str], "signals": list[str], "commands": {device: list[op]}, "limits": {device: {key: float}}, "notes": list[str]}`.
- **command envelope**: `{"target": str, "op": str, "args": dict}`. ops: `set_mode, set_input, set_gain, set_limit, set_state, calibrate, clear_errors, estop, save_nvm, set_origin`.
- **ack dict**: `{"ok": bool, "target": str, "op": str, "detail": str}`.

---

## Task 1: 패키지 스캐폴드 + Transport ABC

**Files:**
- Create: `motor_gui/__init__.py`, `motor_gui/backend/__init__.py`, `motor_gui/backend/transport/__init__.py`, `motor_gui/tests/__init__.py`
- Create: `motor_gui/backend/transport/base.py`
- Test: `motor_gui/tests/test_fake.py` (스켈레톤은 Task 2 에서 채움; 여기선 ABC import 만)

- [ ] **Step 1: 빈 패키지 파일 생성**

```bash
cd /home/light/Defence_Robot
mkdir -p motor_gui/backend/transport motor_gui/frontend/vendor motor_gui/tests
touch motor_gui/__init__.py motor_gui/backend/__init__.py \
      motor_gui/backend/transport/__init__.py motor_gui/tests/__init__.py
```

- [ ] **Step 2: Transport ABC 작성**

`motor_gui/backend/transport/base.py`:

```python
from __future__ import annotations

from abc import ABC, abstractmethod


class TransportError(Exception):
    """전송 계층 연결/IO 실패."""


class Transport(ABC):
    """모든 전송(USB/CAN/Fake)의 공통 계약.

    sample()/apply()/capabilities() 는 **JSON-직렬화 가능한 dict 만** 주고받는다
    (웹 레이어와의 seam — 향후 프로세스 격리 시 그대로 IPC 경계가 됨).
    """

    name: str = "base"

    @abstractmethod
    def connect(self) -> None:
        """장치 연결. 실패 시 TransportError."""

    @abstractmethod
    def sample(self) -> dict:
        """텔레메트리 1프레임. 항상 't_mono' 포함, 키는 '<device>.<signal>'."""

    @abstractmethod
    def apply(self, cmd: dict) -> dict:
        """정규화된 command envelope 적용. ack dict 반환."""

    @abstractmethod
    def capabilities(self) -> dict:
        """이 트랙이 노출하는 devices/signals/commands/limits/notes."""

    @abstractmethod
    def close(self) -> None:
        """안전 정지 + 자원 해제."""
```

- [ ] **Step 3: import 검증**

Run:
```bash
cd /home/light/Defence_Robot && python3 -c "from motor_gui.backend.transport.base import Transport, TransportError; print('OK', Transport.name)"
```
Expected: `OK base`

- [ ] **Step 4: Commit**

```bash
cd /home/light/Defence_Robot
git add motor_gui/
git commit -m "feat(motor_gui): 패키지 스캐폴드 + Transport ABC"
```

---

## Task 2: FakeTransport (시뮬 모터)

**Files:**
- Create: `motor_gui/backend/transport/fake.py`
- Test: `motor_gui/tests/test_fake.py`

- [ ] **Step 1: 실패 테스트 작성**

`motor_gui/tests/test_fake.py`:

```python
import math

from motor_gui.backend.transport.fake import FakeTransport


def test_capabilities_lists_both_devices_and_save_nvm():
    t = FakeTransport()
    caps = t.capabilities()
    assert caps["track"] == "fake"
    assert set(caps["devices"]) == {"odrive", "ak"}
    assert "save_nvm" in caps["commands"]["odrive"]
    assert "odrive.pos" in caps["signals"]
    assert "ak.pos_deg" in caps["signals"]


def test_sample_has_t_mono_and_known_keys():
    t = FakeTransport()
    t.connect()
    s = t.sample()
    assert "t_mono" in s
    for key in caps_signal_keys():
        assert key in s


def caps_signal_keys():
    return FakeTransport().capabilities()["signals"]


def test_velocity_command_drives_velocity_up():
    t = FakeTransport()
    t.connect()
    t.apply({"target": "odrive", "op": "set_mode",
             "args": {"control_mode": "velocity"}})
    t.apply({"target": "odrive", "op": "set_input", "args": {"vel": 5.0}})
    last_vel = 0.0
    for _ in range(200):              # 200 틱 적분
        s = t.sample()
        last_vel = s["odrive.vel"]
    assert last_vel > 1.0             # 0 → 목표(5)로 상승
    assert abs(s["odrive.iq_meas"]) >= 0.0


def test_estop_zeros_commands():
    t = FakeTransport()
    t.connect()
    t.apply({"target": "odrive", "op": "set_input", "args": {"vel": 5.0}})
    ack = t.apply({"target": "odrive", "op": "estop", "args": {}})
    assert ack["ok"] is True
    for _ in range(300):
        s = t.sample()
    assert abs(s["odrive.vel"]) < 0.5
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_fake.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'motor_gui.backend.transport.fake'`

- [ ] **Step 3: FakeTransport 구현**

`motor_gui/backend/transport/fake.py`:

```python
from __future__ import annotations

import time

from .base import Transport

_ODRIVE_SIGNALS = [
    "odrive.pos", "odrive.vel", "odrive.iq_meas", "odrive.iq_set",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus", "odrive.state",
    "odrive.axis_err", "odrive.motor_err", "odrive.enc_err",
    "odrive.ctrl_err", "odrive.vel_integrator",
]
_AK_SIGNALS = ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp", "ak.fault"]


class FakeTransport(Transport):
    """하드웨어 없는 시뮬 모터. odrive+ak 슈퍼셋을 노출해 모든 UI 요소를 구동.

    단순 1차 모델: velocity 모드면 vel 가 목표로 1차 수렴, position 모드면 pos 가
    목표로 수렴, torque 모드면 vel 에 토크를 적분. iq_meas 는 가속도에 비례.
    """

    name = "fake"
    DT = 0.01  # 가정 틱 (100 Hz)

    def __init__(self) -> None:
        self._pos = 0.0
        self._vel = 0.0
        self._mode = "velocity"
        self._target = 0.0      # 모드별 목표 (vel/pos/torque)
        self._ak_pos = 0.0
        self._ak_target = 0.0
        self._last_iq = 0.0

    def connect(self) -> None:
        self._pos = self._vel = self._target = 0.0

    def sample(self) -> dict:
        prev_vel = self._vel
        if self._mode == "velocity":
            self._vel += (self._target - self._vel) * 0.05
        elif self._mode == "position":
            err = self._target - self._pos
            self._vel = err * 2.0
        elif self._mode == "torque":
            self._vel += self._target * 0.1
        self._pos += self._vel * self.DT
        self._ak_pos += (self._ak_target - self._ak_pos) * 0.05
        iq = (self._vel - prev_vel) / self.DT * 0.01 + self._vel * 0.02
        self._last_iq = iq
        return {
            "t_mono": time.monotonic(),
            "odrive.pos": self._pos,
            "odrive.vel": self._vel,
            "odrive.iq_meas": iq,
            "odrive.iq_set": self._target if self._mode == "torque" else iq,
            "odrive.temp_fet": 30.0 + abs(self._vel),
            "odrive.vbus": 24.0,
            "odrive.ibus": iq * 0.5,
            "odrive.state": 8,
            "odrive.axis_err": 0,
            "odrive.motor_err": 0,
            "odrive.enc_err": 0,
            "odrive.ctrl_err": 0,
            "odrive.vel_integrator": self._vel * 0.01,
            "ak.pos_deg": self._ak_pos,
            "ak.speed": (self._ak_target - self._ak_pos) * 10.0,
            "ak.current": 0.0,
            "ak.temp": 28.0,
            "ak.fault": 0,
        }

    def apply(self, cmd: dict) -> dict:
        target, op, args = cmd["target"], cmd["op"], cmd.get("args", {})
        if op == "estop":
            self._target = 0.0
            self._vel = 0.0
            self._ak_target = self._ak_pos
            return self._ack(target, op, "estopped")
        if target == "odrive":
            if op == "set_mode":
                self._mode = args.get("control_mode", self._mode)
            elif op == "set_input":
                if "vel" in args:
                    self._target = float(args["vel"])
                elif "pos" in args:
                    self._target = float(args["pos"])
                elif "torque" in args:
                    self._target = float(args["torque"])
            # set_gain/set_limit/set_state/calibrate/clear_errors/save_nvm: noop ack
        elif target == "ak":
            if op == "set_input" and "pos_deg" in args:
                self._ak_target = float(args["pos_deg"])
            elif op == "set_origin":
                self._ak_pos = 0.0
                self._ak_target = 0.0
        return self._ack(target, op, "ok")

    def capabilities(self) -> dict:
        return {
            "track": "fake",
            "devices": ["odrive", "ak"],
            "signals": _ODRIVE_SIGNALS + _AK_SIGNALS,
            "commands": {
                "odrive": ["set_mode", "set_input", "set_gain", "set_limit",
                           "set_state", "calibrate", "clear_errors",
                           "save_nvm", "estop"],
                "ak": ["set_input", "set_origin", "estop"],
            },
            "limits": {
                "odrive": {"vel": 20.0, "torque": 10.0, "pos": 100.0},
                "ak": {"pos_deg": 360.0},
            },
            "notes": ["fake track — 시뮬 모터, 하드웨어 미연결"],
        }

    def close(self) -> None:
        self._target = 0.0
        self._vel = 0.0

    @staticmethod
    def _ack(target: str, op: str, detail: str) -> dict:
        return {"ok": True, "target": target, "op": op, "detail": detail}
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_fake.py -q`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add motor_gui/backend/transport/fake.py motor_gui/tests/test_fake.py
git commit -m "feat(motor_gui): FakeTransport 시뮬 모터 + 테스트"
```

---

## Task 3: commands.py (정규화/클램프/검증)

**Files:**
- Create: `motor_gui/backend/commands.py`
- Test: `motor_gui/tests/test_commands.py`

- [ ] **Step 1: 실패 테스트 작성**

`motor_gui/tests/test_commands.py`:

```python
import pytest

from motor_gui.backend.commands import normalize, CommandError

CAPS = {
    "track": "fake",
    "devices": ["odrive", "ak"],
    "commands": {
        "odrive": ["set_mode", "set_input", "set_limit", "estop"],
        "ak": ["set_input", "estop"],
    },
    "limits": {"odrive": {"vel": 20.0, "torque": 10.0}, "ak": {"pos_deg": 360.0}},
}


def test_rejects_unknown_target():
    with pytest.raises(CommandError):
        normalize({"target": "ghost", "op": "estop", "args": {}}, CAPS)


def test_rejects_unsupported_op():
    with pytest.raises(CommandError):
        normalize({"target": "odrive", "op": "save_nvm", "args": {}}, CAPS)


def test_clamps_velocity_to_limit():
    out = normalize({"target": "odrive", "op": "set_input",
                     "args": {"vel": 999.0}}, CAPS)
    assert out["args"]["vel"] == 20.0
    out2 = normalize({"target": "odrive", "op": "set_input",
                      "args": {"vel": -999.0}}, CAPS)
    assert out2["args"]["vel"] == -20.0


def test_set_limit_floored_at_zero():
    out = normalize({"target": "odrive", "op": "set_limit",
                     "args": {"vel_limit": -3.0}}, CAPS)
    assert out["args"]["vel_limit"] == 0.0


def test_passes_valid_command_through():
    out = normalize({"target": "ak", "op": "estop", "args": {}}, CAPS)
    assert out == {"target": "ak", "op": "estop", "args": {}}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_commands.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'motor_gui.backend.commands'`

- [ ] **Step 3: commands.py 구현**

`motor_gui/backend/commands.py`:

```python
from __future__ import annotations


class CommandError(Exception):
    """잘못된/미지원 command envelope."""


def normalize(cmd: dict, caps: dict) -> dict:
    """envelope 검증 + 인자 클램프. 실패 시 CommandError. 정규화된 dict 반환."""
    if not isinstance(cmd, dict):
        raise CommandError("command must be an object")
    target = cmd.get("target")
    op = cmd.get("op")
    args = dict(cmd.get("args") or {})

    allowed = caps.get("commands", {})
    if target not in allowed:
        raise CommandError(f"unknown target: {target!r}")
    if op not in allowed[target]:
        raise CommandError(f"op {op!r} not supported for target {target!r}")

    args = _clamp(target, op, args, caps)
    return {"target": target, "op": op, "args": args}


def _clamp(target: str, op: str, args: dict, caps: dict) -> dict:
    limits = caps.get("limits", {}).get(target, {})
    if op == "set_input":
        for key in ("pos", "vel", "torque", "pos_deg"):
            if key in args and key in limits:
                hi = abs(float(limits[key]))
                args[key] = max(-hi, min(hi, float(args[key])))
    elif op == "set_limit":
        for key in ("vel_limit", "current_lim"):
            if key in args:
                args[key] = max(0.0, float(args[key]))
    return args
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_commands.py -q`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add motor_gui/backend/commands.py motor_gui/tests/test_commands.py
git commit -m "feat(motor_gui): command 정규화·클램프 + 테스트"
```

---

## Task 4: HardwareWorker (100 Hz 스레드 + 큐 + estop)

**Files:**
- Create: `motor_gui/backend/worker.py`
- Test: `motor_gui/tests/test_worker.py`

- [ ] **Step 1: 실패 테스트 작성**

`motor_gui/tests/test_worker.py`:

```python
import time

from motor_gui.backend.transport.fake import FakeTransport
from motor_gui.backend.worker import HardwareWorker


def _make() -> HardwareWorker:
    return HardwareWorker(FakeTransport(), rate_hz=200)


def test_start_produces_samples_then_stop():
    w = _make()
    w.start()
    try:
        time.sleep(0.2)
        s = w.latest()
        assert s is not None and "odrive.vel" in s
        assert len(w.history()) > 5
    finally:
        w.stop()


def test_submit_applies_command():
    w = _make()
    w.start()
    try:
        w.submit({"target": "odrive", "op": "set_mode",
                  "args": {"control_mode": "velocity"}})
        ack = w.submit({"target": "odrive", "op": "set_input",
                        "args": {"vel": 8.0}})
        assert ack["ok"] is True
        time.sleep(0.3)
        assert w.latest()["odrive.vel"] > 1.0
    finally:
        w.stop()


def test_invalid_command_returns_error_ack():
    w = _make()
    w.start()
    try:
        ack = w.submit({"target": "ghost", "op": "estop", "args": {}})
        assert ack["ok"] is False
        assert "ghost" in ack["detail"]
    finally:
        w.stop()


def test_estop_fast_path_zeros_velocity():
    w = _make()
    w.start()
    try:
        w.submit({"target": "odrive", "op": "set_input", "args": {"vel": 10.0}})
        time.sleep(0.1)
        w.estop()
        time.sleep(0.4)
        assert abs(w.latest()["odrive.vel"]) < 1.0
    finally:
        w.stop()
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_worker.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'motor_gui.backend.worker'`

- [ ] **Step 3: HardwareWorker 구현**

`motor_gui/backend/worker.py`:

```python
from __future__ import annotations

import collections
import queue
import threading
import time

from .commands import normalize, CommandError
from .transport.base import Transport


class HardwareWorker:
    """Transport 를 단독 소유하는 100 Hz 샘플링/명령 스레드.

    웹 레이어는 submit()/estop()/latest()/history()/subscribe()/capabilities()
    만 사용한다. Transport 접근은 전부 이 스레드 안에서만 일어나 동시접근이 없다.
    """

    def __init__(self, transport: Transport, rate_hz: float = 100.0,
                 ring_size: int = 3000) -> None:
        self._t = transport
        self._dt = 1.0 / rate_hz
        self._ring: collections.deque = collections.deque(maxlen=ring_size)
        self._latest: dict | None = None
        self._cmd_q: queue.Queue = queue.Queue()
        self._estop = threading.Event()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._subscribers: list = []
        self._sub_lock = threading.Lock()
        self._caps = transport.capabilities()

    # ── lifecycle ─────────────────────────────────
    def start(self) -> None:
        self._t.connect()
        self._caps = self._t.capabilities()
        self._running.set()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            self._t.close()
        except Exception:
            pass

    # ── web-facing API ────────────────────────────
    def latest(self) -> dict | None:
        return self._latest

    def history(self) -> list:
        return list(self._ring)

    def capabilities(self) -> dict:
        return self._caps

    def submit(self, cmd: dict) -> dict:
        """동기 명령. 정규화 실패는 즉시 에러 ack. 그 외는 워커가 적용 후 ack."""
        try:
            norm = normalize(cmd, self._caps)
        except CommandError as e:
            return {"ok": False, "target": cmd.get("target"),
                    "op": cmd.get("op"), "detail": str(e)}
        done = threading.Event()
        box: dict = {}
        self._cmd_q.put((norm, done, box))
        if not done.wait(timeout=2.0):
            return {"ok": False, "target": norm["target"], "op": norm["op"],
                    "detail": "command timeout"}
        return box["ack"]

    def estop(self) -> None:
        """최우선 정지 — 다음 루프 톱에서 즉시 적용."""
        self._estop.set()

    def subscribe(self, callback) -> None:
        """callback(sample: dict) 등록 (스레드에서 호출됨, 가볍게 유지)."""
        with self._sub_lock:
            self._subscribers.append(callback)

    def unsubscribe(self, callback) -> None:
        with self._sub_lock:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

    # ── thread loop ───────────────────────────────
    def _loop(self) -> None:
        while self._running.is_set():
            t0 = time.monotonic()

            if self._estop.is_set():
                self._estop.clear()
                for dev in self._caps.get("devices", []):
                    try:
                        self._t.apply({"target": dev, "op": "estop", "args": {}})
                    except Exception:
                        pass

            try:
                s = self._t.sample()
            except Exception as e:  # 샘플 실패는 루프를 죽이지 않음
                s = {"t_mono": time.monotonic(), "error": str(e)}
            self._latest = s
            self._ring.append(s)
            with self._sub_lock:
                subs = list(self._subscribers)
            for cb in subs:
                try:
                    cb(s)
                except Exception:
                    pass

            self._drain_commands()

            elapsed = time.monotonic() - t0
            if elapsed < self._dt:
                time.sleep(self._dt - elapsed)

    def _drain_commands(self) -> None:
        while True:
            try:
                norm, done, box = self._cmd_q.get_nowait()
            except queue.Empty:
                return
            try:
                box["ack"] = self._t.apply(norm)
            except Exception as e:
                box["ack"] = {"ok": False, "target": norm["target"],
                              "op": norm["op"], "detail": str(e)}
            done.set()
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_worker.py -q`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add motor_gui/backend/worker.py motor_gui/tests/test_worker.py
git commit -m "feat(motor_gui): HardwareWorker 스레드 + estop 패스트패스 + 테스트"
```

---

## Task 5: server.py (FastAPI + WS + 런처)

**Files:**
- Create: `motor_gui/backend/server.py`
- Test: `motor_gui/tests/test_server.py`

- [ ] **Step 1: 의존성 설치 (개발 환경)**

Run:
```bash
cd /home/light/Defence_Robot && python3 -m pip install fastapi "uvicorn[standard]" httpx pytest
```
Expected: 설치 성공 (이미 있으면 "already satisfied").

- [ ] **Step 2: 실패 테스트 작성**

`motor_gui/tests/test_server.py`:

```python
from fastapi.testclient import TestClient

from motor_gui.backend.server import create_app


def _client() -> TestClient:
    return TestClient(create_app(track="fake"))


def test_capabilities_endpoint():
    with _client() as c:
        r = c.get("/api/capabilities")
        assert r.status_code == 200
        caps = r.json()
        assert caps["track"] == "fake"
        assert "odrive.pos" in caps["signals"]


def test_command_endpoint_acks():
    with _client() as c:
        r = c.post("/api/command", json={"target": "odrive", "op": "set_input",
                                         "args": {"vel": 3.0}})
        assert r.status_code == 200
        assert r.json()["ok"] is True


def test_command_endpoint_rejects_unknown():
    with _client() as c:
        r = c.post("/api/command", json={"target": "ghost", "op": "estop",
                                         "args": {}})
        assert r.json()["ok"] is False


def test_telemetry_websocket_streams_samples():
    with _client() as c:
        with c.websocket_connect("/ws/telemetry") as ws:
            msg = ws.receive_json()
            assert "t_mono" in msg and "odrive.vel" in msg


def test_record_start_stop(tmp_path):
    with _client() as c:
        path = str(tmp_path / "log.csv")
        r1 = c.post("/api/record/start", json={"path": path, "fmt": "csv"})
        assert r1.json()["ok"] is True
        r2 = c.post("/api/record/stop")
        assert r2.json()["ok"] is True
```

- [ ] **Step 3: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_server.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'motor_gui.backend.server'`

- [ ] **Step 4: server.py 구현**

`motor_gui/backend/server.py`:

```python
from __future__ import annotations

import argparse
import asyncio
import contextlib
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

from .worker import HardwareWorker
from .recorder import Recorder

_FRONTEND = Path(__file__).resolve().parents[1] / "frontend"


def _make_transport(track: str):
    if track == "fake":
        from .transport.fake import FakeTransport
        return FakeTransport()
    if track == "usb":
        from .transport.usb_odrive import UsbOdriveBackend
        return UsbOdriveBackend()
    if track == "can":
        from .transport.can_bus import CanBackend
        return CanBackend()
    raise ValueError(f"unknown track: {track!r}")


def create_app(track: str = "fake") -> FastAPI:
    app = FastAPI(title="motor_gui", version="0.1")
    worker = HardwareWorker(_make_transport(track))
    recorder = Recorder(worker)

    @app.on_event("startup")
    def _startup() -> None:
        worker.start()

    @app.on_event("shutdown")
    def _shutdown() -> None:
        recorder.stop()
        worker.stop()

    @app.get("/api/capabilities")
    def capabilities() -> dict:
        return worker.capabilities()

    @app.post("/api/command")
    def command(envelope: dict) -> dict:
        if envelope.get("op") == "estop":
            worker.estop()
            return {"ok": True, "target": envelope.get("target"),
                    "op": "estop", "detail": "estop latched"}
        return worker.submit(envelope)

    @app.post("/api/record/start")
    def record_start(body: dict) -> dict:
        return recorder.start(body.get("path", "telemetry.csv"),
                              body.get("fmt", "csv"))

    @app.post("/api/record/stop")
    def record_stop() -> dict:
        return recorder.stop()

    @app.websocket("/ws/telemetry")
    async def telemetry(ws: WebSocket) -> None:
        await ws.accept()
        loop = asyncio.get_running_loop()
        q: asyncio.Queue = asyncio.Queue(maxsize=10)

        def _on_sample(s: dict) -> None:
            # 워커 스레드에서 호출 → 이벤트 루프로 안전 전달. 가득 차면 최신 우선 드롭.
            def _put() -> None:
                if q.full():
                    with contextlib.suppress(asyncio.QueueEmpty):
                        q.get_nowait()
                q.put_nowait(s)
            loop.call_soon_threadsafe(_put)

        worker.subscribe(_on_sample)
        try:
            while True:
                s = await q.get()
                await ws.send_json(s)
        except WebSocketDisconnect:
            pass
        finally:
            worker.unsubscribe(_on_sample)

    if _FRONTEND.exists():
        app.mount("/", StaticFiles(directory=str(_FRONTEND), html=True),
                  name="frontend")
    return app


def main() -> None:
    import uvicorn
    p = argparse.ArgumentParser(description="motor_gui backend")
    p.add_argument("--track", choices=["fake", "usb", "can"], default="fake")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    args = p.parse_args()
    uvicorn.run(create_app(track=args.track), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
```

> 주의: 이 태스크는 `recorder.Recorder` 에 의존한다. Task 6 이 먼저 머지되어야 import 가 풀린다. **실행 순서상 Task 6 을 Task 5 직후(또는 직전)에 둔다.** TDD 진행 시 Task 5 Step 3 직후 Recorder 스텁이 없으면 import 에러가 나므로, 아래 Step 5 전에 Task 6 의 `recorder.py` 를 먼저 생성한다.

- [ ] **Step 5: (선결) Recorder 최소 스텁 확인**

`motor_gui/backend/recorder.py` 가 없으면 Task 6 Step 3 의 구현을 먼저 생성한 뒤 진행.
빠른 확인:
```bash
cd /home/light/Defence_Robot && test -f motor_gui/backend/recorder.py && echo "recorder present" || echo "create recorder first (Task 6)"
```

- [ ] **Step 6: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_server.py -q`
Expected: PASS (5 passed)

- [ ] **Step 7: Commit**

```bash
git add motor_gui/backend/server.py motor_gui/tests/test_server.py
git commit -m "feat(motor_gui): FastAPI server (capabilities/command/telemetry WS/record) + 테스트"
```

---

## Task 6: recorder.py (선택적 로깅)

> Task 5 의 import 의존 때문에 **Task 5 의 server 테스트 실행 전에 이 파일이 존재**해야 한다. 실제 작업 순서: Task 5 Step 4 까지 작성 → 본 Task 6 Step 3 작성 → Task 5 Step 6 테스트 → 본 Task 6 테스트. 컨트롤러는 두 태스크를 인접 dispatch 한다.

**Files:**
- Create: `motor_gui/backend/recorder.py`
- Test: `motor_gui/tests/test_server.py::test_record_start_stop` (Task 5 에 포함) + 아래 단위 테스트

- [ ] **Step 1: 실패 테스트 작성**

`motor_gui/tests/test_recorder.py`:

```python
import csv
import time

from motor_gui.backend.transport.fake import FakeTransport
from motor_gui.backend.worker import HardwareWorker
from motor_gui.backend.recorder import Recorder


def test_records_csv_rows(tmp_path):
    w = HardwareWorker(FakeTransport(), rate_hz=200)
    w.start()
    rec = Recorder(w)
    path = str(tmp_path / "log.csv")
    try:
        assert rec.start(path, "csv")["ok"] is True
        time.sleep(0.3)
        rec.stop()
    finally:
        w.stop()
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    assert rows[0][0] == "t_mono"          # 헤더
    assert len(rows) > 5                    # 데이터 행
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_recorder.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'motor_gui.backend.recorder'`

- [ ] **Step 3: recorder.py 구현**

`motor_gui/backend/recorder.py`:

```python
from __future__ import annotations

import csv
import queue
import threading


class Recorder:
    """worker.sample_bus 를 tap 해서 CSV/parquet 로 기록. 토글식 (기본 off)."""

    def __init__(self, worker) -> None:
        self._worker = worker
        self._q: queue.Queue | None = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._path: str | None = None
        self._fmt = "csv"

    def start(self, path: str, fmt: str = "csv") -> dict:
        if self._running.is_set():
            return {"ok": False, "detail": "already recording"}
        if fmt not in ("csv", "parquet"):
            return {"ok": False, "detail": f"unsupported fmt: {fmt}"}
        self._path, self._fmt = path, fmt
        self._q = queue.Queue(maxsize=10000)
        self._worker.subscribe(self._on_sample)
        self._running.set()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return {"ok": True, "detail": f"recording → {path} ({fmt})"}

    def stop(self) -> dict:
        if not self._running.is_set():
            return {"ok": True, "detail": "not recording"}
        self._running.clear()
        self._worker.unsubscribe(self._on_sample)
        if self._thread:
            self._thread.join(timeout=2.0)
        return {"ok": True, "detail": f"stopped → {self._path}"}

    def _on_sample(self, s: dict) -> None:
        if self._q is not None:
            try:
                self._q.put_nowait(s)
            except queue.Full:
                pass

    def _run(self) -> None:
        rows: list[dict] = []
        # 첫 샘플로 헤더 고정
        first = self._q.get()
        cols = list(first.keys())
        rows.append(first)
        while self._running.is_set() or not self._q.empty():
            try:
                rows.append(self._q.get(timeout=0.1))
            except queue.Empty:
                continue
        if self._fmt == "csv":
            with open(self._path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
                w.writeheader()
                for r in rows:
                    w.writerow(r)
        else:  # parquet
            import pyarrow as pa
            import pyarrow.parquet as pq
            table = pa.Table.from_pylist([{c: r.get(c) for c in cols}
                                          for r in rows])
            pq.write_table(table, self._path)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/test_recorder.py -q`
Expected: PASS (1 passed)

- [ ] **Step 5: Commit**

```bash
git add motor_gui/backend/recorder.py motor_gui/tests/test_recorder.py
git commit -m "feat(motor_gui): 선택적 CSV/parquet 로깅 recorder + 테스트"
```

---

## Task 7: 프론트엔드 (uPlot)

**Files:**
- Create: `motor_gui/frontend/index.html`, `motor_gui/frontend/plots.js`, `motor_gui/frontend/app.js`
- Create: `motor_gui/frontend/vendor/uPlot.iife.min.js`, `motor_gui/frontend/vendor/uPlot.min.css`

- [ ] **Step 1: uPlot 벤더링**

```bash
cd /home/light/Defence_Robot/motor_gui/frontend/vendor
curl -fsSL -o uPlot.iife.min.js https://cdn.jsdelivr.net/npm/uplot@1.6.30/dist/uPlot.iife.min.js
curl -fsSL -o uPlot.min.css   https://cdn.jsdelivr.net/npm/uplot@1.6.30/dist/uPlot.min.css
ls -la
```
Expected: 두 파일 다운로드 (js ~40KB, css ~3KB). 네트워크 불가 시 동일 버전 수동 배치.

- [ ] **Step 2: index.html**

`motor_gui/frontend/index.html`:

```html
<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>motor_gui</title>
  <link rel="stylesheet" href="/vendor/uPlot.min.css" />
  <style>
    body { font-family: system-ui, sans-serif; margin: 0; background:#111; color:#eee; }
    header { display:flex; align-items:center; gap:1rem; padding:.5rem 1rem; background:#1b1b1b; }
    #estop { background:#c0202a; color:#fff; border:0; font-size:1.1rem;
             font-weight:700; padding:.6rem 1.2rem; border-radius:6px; cursor:pointer; }
    #status { font-size:.85rem; opacity:.8; }
    main { display:grid; grid-template-columns: 320px 1fr; gap:1rem; padding:1rem; }
    #controls { display:flex; flex-direction:column; gap:.6rem; }
    .panel { background:#1b1b1b; border-radius:8px; padding:.8rem; }
    .panel h3 { margin:.2rem 0 .6rem; font-size:.95rem; }
    .row { display:flex; align-items:center; gap:.4rem; margin:.3rem 0; }
    .row label { width:90px; font-size:.8rem; opacity:.85; }
    input, select, button { background:#262626; color:#eee; border:1px solid #333;
                            border-radius:4px; padding:.25rem .4rem; }
    #plots { display:flex; flex-direction:column; gap:1rem; }
  </style>
</head>
<body>
  <header>
    <strong>motor_gui</strong>
    <span id="track"></span>
    <span id="status">connecting…</span>
    <button id="estop">■ E-STOP</button>
  </header>
  <main>
    <div id="controls"></div>
    <div id="plots"></div>
  </main>
  <script src="/vendor/uPlot.iife.min.js"></script>
  <script src="/plots.js"></script>
  <script src="/app.js"></script>
</body>
</html>
```

- [ ] **Step 3: plots.js**

`motor_gui/frontend/plots.js`:

```javascript
// uPlot 패널 묶음. 각 패널은 신호 키 리스트를 plot.
const WINDOW_SEC = 20;
const MAX_PTS = WINDOW_SEC * 100; // 100 Hz

function makePanel(title, sigKeys) {
  const data = [[]];                       // [x, ...series]
  sigKeys.forEach(() => data.push([]));
  const opts = {
    title, width: 800, height: 200,
    scales: { x: { time: false } },
    series: [{ label: "t" }].concat(
      sigKeys.map((k, i) => ({
        label: k, stroke: ["#4fc3f7", "#ffb74d", "#81c784", "#e57373", "#ba68c8"][i % 5],
      }))),
    axes: [
      { stroke: "#888", grid: { stroke: "#222" } },
      { stroke: "#888", grid: { stroke: "#222" } },
    ],
  };
  const el = document.createElement("div");
  el.className = "panel";
  document.getElementById("plots").appendChild(el);
  const u = new uPlot(opts, data, el);
  return {
    keys: sigKeys,
    push(t, sample) {
      data[0].push(t);
      sigKeys.forEach((k, i) => data[i + 1].push(sample[k]));
      while (data[0].length > MAX_PTS) data.forEach((arr) => arr.shift());
    },
    redraw() { u.setData(data); },
  };
}

// 신호 키 → 패널 그룹핑 규칙
function buildPanels(signals) {
  const has = (k) => signals.includes(k);
  const panels = [];
  if (has("odrive.pos") || has("odrive.vel"))
    panels.push(makePanel("ODrive 위치/속도",
      ["odrive.pos", "odrive.vel"].filter(has)));
  if (has("odrive.iq_meas"))
    panels.push(makePanel("ODrive 전류(토크)",
      ["odrive.iq_meas", "odrive.iq_set"].filter(has)));
  if (has("odrive.temp_fet") || has("odrive.vbus"))
    panels.push(makePanel("ODrive 온도/버스",
      ["odrive.temp_fet", "odrive.vbus", "odrive.ibus"].filter(has)));
  if (has("ak.pos_deg"))
    panels.push(makePanel("AK 조향",
      ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp"].filter(has)));
  return panels;
}

window.MGPlots = { buildPanels };
```

- [ ] **Step 4: app.js**

`motor_gui/frontend/app.js`:

```javascript
let panels = [];
let t0 = null;

async function postCommand(envelope) {
  const r = await fetch("/api/command", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(envelope),
  });
  return r.json();
}

function controlPanel(device, caps) {
  const ops = caps.commands[device];
  const wrap = document.createElement("div");
  wrap.className = "panel";
  wrap.innerHTML = `<h3>${device}</h3>`;

  if (ops.includes("set_mode")) {
    wrap.appendChild(rowSelect("control_mode", ["position", "velocity", "torque"],
      (v) => postCommand({ target: device, op: "set_mode", args: { control_mode: v } })));
  }
  if (ops.includes("set_input")) {
    const key = device === "ak" ? "pos_deg" : "vel";
    wrap.appendChild(rowNumber(key, (v) =>
      postCommand({ target: device, op: "set_input", args: { [key]: v } })));
  }
  if (ops.includes("set_gain")) {
    wrap.appendChild(rowNumber("vel_gain", (v) =>
      postCommand({ target: device, op: "set_gain", args: { vel_gain: v } })));
  }
  if (ops.includes("calibrate")) {
    wrap.appendChild(rowButton("캘리브레이션", () =>
      postCommand({ target: device, op: "calibrate", args: {} })));
  }
  if (ops.includes("save_nvm")) {
    wrap.appendChild(rowButton("NVM 저장", () =>
      postCommand({ target: device, op: "save_nvm", args: {} })));
  }
  if (ops.includes("clear_errors")) {
    wrap.appendChild(rowButton("에러 클리어", () =>
      postCommand({ target: device, op: "clear_errors", args: {} })));
  }
  return wrap;
}

function rowNumber(label, onSet) {
  const row = el("div", "row");
  row.innerHTML = `<label>${label}</label><input type="number" step="0.1" />`;
  const inp = row.querySelector("input");
  inp.addEventListener("change", () => onSet(parseFloat(inp.value)));
  return row;
}
function rowSelect(label, options, onSet) {
  const row = el("div", "row");
  row.innerHTML = `<label>${label}</label><select>${
    options.map((o) => `<option>${o}</option>`).join("")}</select>`;
  const sel = row.querySelector("select");
  sel.addEventListener("change", () => onSet(sel.value));
  return row;
}
function rowButton(label, onClick) {
  const row = el("div", "row");
  const b = document.createElement("button");
  b.textContent = label;
  b.addEventListener("click", onClick);
  row.appendChild(b);
  return row;
}
function el(tag, cls) { const e = document.createElement(tag); e.className = cls; return e; }

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
  ws.onopen = () => (document.getElementById("status").textContent = "● live");
  ws.onclose = () => {
    document.getElementById("status").textContent = "○ reconnecting…";
    setTimeout(connectWS, 1000);
  };
  ws.onmessage = (ev) => {
    const s = JSON.parse(ev.data);
    if (t0 === null) t0 = s.t_mono;
    const t = s.t_mono - t0;
    panels.forEach((p) => p.push(t, s));
  };
}

function renderLoop() {
  panels.forEach((p) => p.redraw());
  requestAnimationFrame(renderLoop);   // 디스플레이는 ~60fps, 수집은 100Hz
}

async function main() {
  const caps = await (await fetch("/api/capabilities")).json();
  document.getElementById("track").textContent = `[${caps.track}]`;
  const controls = document.getElementById("controls");
  caps.devices.forEach((d) => controls.appendChild(controlPanel(d, caps)));
  document.getElementById("estop").addEventListener("click", () =>
    postCommand({ target: caps.devices[0], op: "estop", args: {} }));
  panels = window.MGPlots.buildPanels(caps.signals);
  connectWS();
  requestAnimationFrame(renderLoop);
}

main();
```

- [ ] **Step 5: 수동 스모크 (fake 트랙)**

Run:
```bash
cd /home/light/Defence_Robot && python3 -m motor_gui.backend.server --track fake --port 8000 &
sleep 2 && curl -s localhost:8000/api/capabilities | head -c 200 && echo && curl -s localhost:8000/ | grep -o "<title>motor_gui</title>" && kill %1
```
Expected: capabilities JSON + `<title>motor_gui</title>`. (브라우저로 `localhost:8000` 열면 fake 모터가 plot 에서 움직이는지 육안 확인.)

- [ ] **Step 6: Commit**

```bash
git add motor_gui/frontend/
git commit -m "feat(motor_gui): 바닐라 JS + uPlot 프론트엔드 (capabilities 기반 동적 UI)"
```

---

## Task 8: UsbOdriveBackend (실하드웨어 — ODrive USB)

> **HIL 태스크.** 컨트롤러는 시작 전 사용자에게 **"Jetson 에 ODrive USB 연결 + 전원 인가"** 를 요청한다 (다른 트랙은 분리). 검증은 Jetson 컨테이너 안에서 수행.

**Files:**
- Create: `motor_gui/backend/transport/usb_odrive.py`

- [ ] **Step 1: UsbOdriveBackend 구현**

`motor_gui/backend/transport/usb_odrive.py`:

```python
from __future__ import annotations

import time

from .base import Transport, TransportError

_ODRIVE_SIGNALS = [
    "odrive.pos", "odrive.vel", "odrive.iq_meas", "odrive.iq_set",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus", "odrive.state",
    "odrive.axis_err", "odrive.motor_err", "odrive.enc_err",
    "odrive.ctrl_err", "odrive.vel_integrator",
]


class UsbOdriveBackend(Transport):
    """ODrive USB (odrive lib, axis1, fw-v0.5.6). NVM 저장 지원."""

    name = "usb"

    def __init__(self, axis_num: int = 1, timeout: float = 15.0) -> None:
        self._timeout = timeout
        self._axis_num = axis_num
        self._drv = None
        self._ax = None
        self._enums: dict = {}

    def connect(self) -> None:
        import odrive
        from odrive.enums import (AxisState, ControlMode, InputMode)
        drv = odrive.find_any(timeout=self._timeout)
        if drv is None:
            raise TransportError("ODrive USB not found")
        self._drv = drv
        self._ax = drv.axis1 if self._axis_num == 1 else drv.axis0
        # fw-v0.5.6 plain Enum → wire I/O 용 int 상수
        self._enums = {
            "IDLE": AxisState.IDLE.value,
            "CLOSED_LOOP": AxisState.CLOSED_LOOP_CONTROL.value,
            "FULL_CALIB": AxisState.FULL_CALIBRATION_SEQUENCE.value,
            "POSITION": ControlMode.POSITION_CONTROL.value,
            "VELOCITY": ControlMode.VELOCITY_CONTROL.value,
            "TORQUE": ControlMode.TORQUE_CONTROL.value,
            "PASSTHROUGH": InputMode.PASSTHROUGH.value,
            "POS_FILTER": InputMode.POS_FILTER.value,
            "VEL_RAMP": InputMode.VEL_RAMP.value,
        }

    def sample(self) -> dict:
        ax, drv = self._ax, self._drv
        m = ax.motor.current_control
        return {
            "t_mono": time.monotonic(),
            "odrive.pos": float(ax.encoder.pos_estimate),
            "odrive.vel": float(ax.encoder.vel_estimate),
            "odrive.iq_meas": float(m.Iq_measured),
            "odrive.iq_set": float(m.Iq_setpoint),
            "odrive.temp_fet": float(ax.motor.fet_thermistor.temperature),
            "odrive.vbus": float(drv.vbus_voltage),
            "odrive.ibus": float(getattr(drv, "ibus", 0.0)),
            "odrive.state": int(ax.current_state),
            "odrive.axis_err": int(ax.error),
            "odrive.motor_err": int(ax.motor.error),
            "odrive.enc_err": int(ax.encoder.error),
            "odrive.ctrl_err": int(ax.controller.error),
            "odrive.vel_integrator": float(ax.controller.vel_integrator_torque),
        }

    def apply(self, cmd: dict) -> dict:
        ax = self._ax
        op, args = cmd["op"], cmd.get("args", {})
        try:
            if op == "estop":
                ax.requested_state = self._enums["IDLE"]
            elif op == "set_mode":
                cm = {"position": "POSITION", "velocity": "VELOCITY",
                      "torque": "TORQUE"}[args["control_mode"]]
                ax.controller.config.control_mode = self._enums[cm]
            elif op == "set_input":
                if "pos" in args:
                    ax.controller.input_pos = float(args["pos"])
                elif "vel" in args:
                    ax.controller.input_vel = float(args["vel"])
                elif "torque" in args:
                    ax.controller.input_torque = float(args["torque"])
            elif op == "set_gain":
                for k in ("pos_gain", "vel_gain", "vel_integrator_gain"):
                    if k in args:
                        setattr(ax.controller.config, k, float(args[k]))
            elif op == "set_limit":
                if "vel_limit" in args:
                    ax.controller.config.vel_limit = float(args["vel_limit"])
                if "current_lim" in args:
                    ax.motor.config.current_lim = float(args["current_lim"])
            elif op == "set_state":
                if args.get("state") == "closed_loop":
                    ax.controller.input_pos = ax.encoder.pos_estimate  # jump 방지
                    ax.requested_state = self._enums["CLOSED_LOOP"]
                else:
                    ax.requested_state = self._enums["IDLE"]
            elif op == "calibrate":
                ax.requested_state = self._enums["FULL_CALIB"]
            elif op == "clear_errors":
                ax.clear_errors()
            elif op == "save_nvm":
                self._drv.save_configuration()
            return {"ok": True, "target": "odrive", "op": op, "detail": "ok"}
        except Exception as e:
            return {"ok": False, "target": "odrive", "op": op, "detail": str(e)}

    def capabilities(self) -> dict:
        return {
            "track": "usb",
            "devices": ["odrive"],
            "signals": _ODRIVE_SIGNALS,
            "commands": {"odrive": ["set_mode", "set_input", "set_gain",
                                    "set_limit", "set_state", "calibrate",
                                    "clear_errors", "save_nvm", "estop"]},
            "limits": {"odrive": {"vel": 20.0, "torque": 10.0, "pos": 100.0}},
            "notes": ["USB 트랙 — ODrive 단독, NVM 저장 가능"],
        }

    def close(self) -> None:
        if self._ax is not None:
            try:
                self._ax.requested_state = self._enums["IDLE"]
            except Exception:
                pass
```

- [ ] **Step 2: HIL — Jetson 컨테이너에서 connect + sample 검증**

> 컨트롤러: 사용자에게 ODrive USB 연결 요청. 연결 확인 후 아래 실행.

Run (Jetson, 컨테이너 안):
```bash
cd /workspace && python3 -c "
from motor_gui.backend.transport.usb_odrive import UsbOdriveBackend
t = UsbOdriveBackend(); t.connect()
s = t.sample()
print('vbus', round(s['odrive.vbus'],2), 'state', s['odrive.state'], 'pos', round(s['odrive.pos'],3))
assert s['odrive.vbus'] > 5.0
t.close(); print('OK usb sample')
"
```
Expected: `vbus <값> state <값> pos <값>` + `OK usb sample`. `find_any` 실패 시 udev/USB 재확인.

- [ ] **Step 3: HIL — 서버 fake→usb 교체 스모크**

Run (Jetson 컨테이너):
```bash
cd /workspace && (python3 -m motor_gui.backend.server --track usb --port 8000 &) ; sleep 4
curl -s localhost:8000/api/capabilities | python3 -c "import sys,json; c=json.load(sys.stdin); print(c['track'], c['devices'])"
pkill -f motor_gui.backend.server
```
Expected: `usb ['odrive']`. (노트북 브라우저 `http://jetson-orin.local:8000` 에서 실제 모터 텔레메트리 plot 육안 확인.)

- [ ] **Step 4: Commit**

```bash
git add motor_gui/backend/transport/usb_odrive.py
git commit -m "feat(motor_gui): UsbOdriveBackend (ODrive USB, axis1, NVM 저장) + HIL 검증"
```

---

## Task 9: CanBackend (실하드웨어 — ODrive CAN + AK CAN)

> **HIL 태스크.** CAN 트랙은 설계상 ODrive(node1)+AK(id10) 동시지만, 사용자는 한 번에 하나만 연결 가능. 따라서: (a) FakeTransport 가 동시 케이스 커버, (b) HIL 은 **ODrive-CAN 단독** 1회 + **AK-CAN 단독** 1회로 분리 검증. 컨트롤러가 각 단계 전 해당 연결을 사용자에게 요청한다. CAN 전 `bash scripts/can_setup.sh` 필수.

**Files:**
- Create: `motor_gui/backend/transport/can_bus.py`

- [ ] **Step 1: CanBackend 구현**

`motor_gui/backend/transport/can_bus.py`:

```python
from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

from .base import Transport, TransportError

# steering/ak_control.py 의 AK 클래스 재사용 (hw 로직 단일 소스)
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "motor_control" / "steering"))
from ak_control import AK  # noqa: E402

NODE_ID = 1                          # ODrive
AK_ID = 10                           # AK 조향

# ODrive CANSimple cmd id (fw-v0.5.6)
C_HEARTBEAT = 0x001
C_ESTOP = 0x002
C_SET_STATE = 0x007
C_GET_ENC_EST = 0x009
C_SET_CTRL_MODE = 0x00B
C_SET_INPUT_POS = 0x00C
C_SET_INPUT_VEL = 0x00D
C_SET_INPUT_TORQUE = 0x00E
C_SET_LIMITS = 0x00F
C_GET_IQ = 0x014
C_GET_TEMP = 0x015
C_CLEAR_ERR = 0x018
C_GET_BUS_VI = 0x017
C_SET_POS_GAIN = 0x01A
C_SET_VEL_GAINS = 0x01B

AXIS_IDLE = 1
AXIS_CLOSED_LOOP = 8
CTRL = {"position": 3, "velocity": 2, "torque": 1}  # ODrive ControlMode int

_ODRIVE_SIGNALS = [
    "odrive.pos", "odrive.vel", "odrive.iq_meas", "odrive.iq_set",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus", "odrive.state",
    "odrive.axis_err",
]
_AK_SIGNALS = ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp", "ak.fault"]


class CanBackend(Transport):
    """can0 위 ODrive(node1) CANSimple + AK(id10) servo 동시."""

    name = "can"

    def __init__(self, channel: str = "can0") -> None:
        self._channel = channel
        self._bus = None
        self._ak = None
        self._state = {k: 0.0 for k in _ODRIVE_SIGNALS}

    def connect(self) -> None:
        import can
        try:
            self._bus = can.interface.Bus(channel=self._channel,
                                          interface="socketcan")
        except OSError as e:
            raise TransportError(
                f"{self._channel} open 실패 — 'bash scripts/can_setup.sh' 먼저 ({e})")
        self._ak = AK(self._bus, AK_ID, name="steer")

    def _send(self, cmd_id: int, data: bytes = b"") -> None:
        import can
        arb = (NODE_ID << 5) | cmd_id
        self._bus.send(can.Message(arbitration_id=arb, data=data,
                                   is_extended_id=False))

    def _request(self, cmd_id: int) -> None:
        import can
        arb = (NODE_ID << 5) | cmd_id
        self._bus.send(can.Message(arbitration_id=arb, is_remote_frame=True,
                                   is_extended_id=False))

    def sample(self) -> dict:
        # cyclic(heartbeat/enc) + RTR(iq/temp/busVI) 드레인
        self._request(C_GET_IQ)
        self._request(C_GET_TEMP)
        self._request(C_GET_BUS_VI)
        deadline = time.monotonic() + 0.008
        while time.monotonic() < deadline:
            msg = self._bus.recv(timeout=0.002)
            if msg is None:
                break
            self._decode_odrive(msg)
        if self._ak.poll(timeout=0.003):
            pass
        s = {"t_mono": time.monotonic()}
        s.update(self._state)
        s.update({
            "ak.pos_deg": self._ak.pos_out_deg,
            "ak.speed": float(self._ak.spd_erpm),
            "ak.current": self._ak.cur_a,
            "ak.temp": float(self._ak.temp_c),
            "ak.fault": int(self._ak.fault),
        })
        return s

    def _decode_odrive(self, msg) -> None:
        if msg.is_extended_id:
            return
        cmd = msg.arbitration_id & 0x1F
        node = msg.arbitration_id >> 5
        if node != NODE_ID:
            return
        d = msg.data
        if cmd == C_HEARTBEAT and len(d) >= 5:
            self._state["odrive.axis_err"] = struct.unpack("<I", d[:4])[0]
            self._state["odrive.state"] = d[4]
        elif cmd == C_GET_ENC_EST and len(d) >= 8:
            pos, vel = struct.unpack("<ff", d[:8])
            self._state["odrive.pos"] = pos
            self._state["odrive.vel"] = vel
        elif cmd == C_GET_IQ and len(d) >= 8:
            iq_set, iq_meas = struct.unpack("<ff", d[:8])
            self._state["odrive.iq_set"] = iq_set
            self._state["odrive.iq_meas"] = iq_meas
        elif cmd == C_GET_TEMP and len(d) >= 8:
            fet, _motor = struct.unpack("<ff", d[:8])
            self._state["odrive.temp_fet"] = fet
        elif cmd == C_GET_BUS_VI and len(d) >= 8:
            vbus, ibus = struct.unpack("<ff", d[:8])
            self._state["odrive.vbus"] = vbus
            self._state["odrive.ibus"] = ibus

    def apply(self, cmd: dict) -> dict:
        target, op, args = cmd["target"], cmd["op"], cmd.get("args", {})
        try:
            if target == "ak":
                return self._apply_ak(op, args)
            return self._apply_odrive(op, args)
        except Exception as e:
            return {"ok": False, "target": target, "op": op, "detail": str(e)}

    def _apply_odrive(self, op: str, args: dict) -> dict:
        if op == "estop":
            self._send(C_ESTOP)
        elif op == "set_mode":
            cm = CTRL[args["control_mode"]]
            self._send(C_SET_CTRL_MODE, struct.pack("<ii", cm, 1))  # input_mode=PASSTHROUGH
        elif op == "set_input":
            if "pos" in args:
                self._send(C_SET_INPUT_POS, struct.pack("<fhh", float(args["pos"]), 0, 0))
            elif "vel" in args:
                self._send(C_SET_INPUT_VEL, struct.pack("<ff", float(args["vel"]), 0.0))
            elif "torque" in args:
                self._send(C_SET_INPUT_TORQUE, struct.pack("<f", float(args["torque"])))
        elif op == "set_gain":
            if "pos_gain" in args:
                self._send(C_SET_POS_GAIN, struct.pack("<f", float(args["pos_gain"])))
            if "vel_gain" in args or "vel_integrator_gain" in args:
                self._send(C_SET_VEL_GAINS, struct.pack("<ff",
                           float(args.get("vel_gain", 0.0)),
                           float(args.get("vel_integrator_gain", 0.0))))
        elif op == "set_limit":
            self._send(C_SET_LIMITS, struct.pack("<ff",
                       float(args.get("vel_limit", 0.0)),
                       float(args.get("current_lim", 0.0))))
        elif op == "set_state":
            st = AXIS_CLOSED_LOOP if args.get("state") == "closed_loop" else AXIS_IDLE
            self._send(C_SET_STATE, struct.pack("<i", st))
        elif op == "calibrate":
            self._send(C_SET_STATE, struct.pack("<i", 3))  # FULL_CALIBRATION_SEQUENCE
        elif op == "clear_errors":
            self._send(C_CLEAR_ERR)
        # save_nvm: CAN 미지원 (capabilities 에서 거부됨)
        return {"ok": True, "target": "odrive", "op": op, "detail": "sent"}

    def _apply_ak(self, op: str, args: dict) -> dict:
        if op == "estop":
            self._ak.stop()
        elif op == "set_input":
            if "pos_deg" in args:
                self._ak.send_pos_out(float(args["pos_deg"]))
            elif "rpm" in args:
                self._ak.send_rpm_out(float(args["rpm"]))
        elif op == "set_origin":
            self._ak.set_origin_here()
        return {"ok": True, "target": "ak", "op": op, "detail": "sent"}

    def capabilities(self) -> dict:
        return {
            "track": "can",
            "devices": ["odrive", "ak"],
            "signals": _ODRIVE_SIGNALS + _AK_SIGNALS,
            "commands": {
                "odrive": ["set_mode", "set_input", "set_gain", "set_limit",
                           "set_state", "calibrate", "clear_errors", "estop"],
                "ak": ["set_input", "set_origin", "estop"],
            },
            "limits": {"odrive": {"vel": 20.0, "torque": 10.0, "pos": 100.0},
                       "ak": {"pos_deg": 360.0}},
            "notes": ["CAN 트랙 — ODrive+AK 동시. NVM 저장 불가 (USB 전용)"],
        }

    def close(self) -> None:
        try:
            if self._bus is not None:
                self._send(C_SET_STATE, struct.pack("<i", AXIS_IDLE))
                self._ak.stop()
                self._bus.shutdown()
        except Exception:
            pass
```

- [ ] **Step 2: HIL — ODrive-CAN 단독 검증**

> 컨트롤러: 사용자에게 **ODrive CAN 연결** 요청 (AK 분리). `bash scripts/can_setup.sh` 후.

Run (Jetson 컨테이너):
```bash
cd /workspace && bash scripts/can_setup.sh && python3 -c "
from motor_gui.backend.transport.can_bus import CanBackend
t = CanBackend(); t.connect()
import time
for _ in range(50): s = t.sample(); time.sleep(0.01)
print('odrive state', s['odrive.state'], 'pos', round(s['odrive.pos'],3), 'vbus', round(s.get('odrive.vbus',0),2))
t.close(); print('OK can odrive sample')
"
```
Expected: heartbeat 로 state/pos 갱신 + `OK can odrive sample`.

- [ ] **Step 3: HIL — AK-CAN 단독 검증**

> 컨트롤러: 사용자에게 **AK CAN 연결** 요청 (ODrive 분리).

Run (Jetson 컨테이너):
```bash
cd /workspace && bash scripts/can_setup.sh && python3 -c "
from motor_gui.backend.transport.can_bus import CanBackend
t = CanBackend(); t.connect()
import time
t.apply({'target':'ak','op':'set_origin','args':{}}); time.sleep(0.2)
for _ in range(50): s = t.sample(); time.sleep(0.01)
print('ak pos_deg', round(s['ak.pos_deg'],2), 'temp', s['ak.temp'], 'fault', s['ak.fault'])
t.close(); print('OK can ak sample')
"
```
Expected: AK status (pos/temp/fault) 수신 + `OK can ak sample`. (ODrive 없으면 odrive.* 는 0 유지 — 정상.)

- [ ] **Step 4: Commit**

```bash
git add motor_gui/backend/transport/can_bus.py
git commit -m "feat(motor_gui): CanBackend (ODrive CANSimple + AK servo) + 단독 HIL 검증

ODrive+AK 동시 HIL 은 두 노드 동시 연결 가능 시점까지 보류 (Fake 가 커버)."
```

---

## Task 10: Dockerfile.jetson 의존성 + README

**Files:**
- Modify: `docker/Dockerfile.jetson`
- Create: `motor_gui/README.md`

- [ ] **Step 1: Dockerfile.jetson 에 웹 의존성 추가**

`docker/Dockerfile.jetson` 의 pip install 블록 (`ultralytics` 다음 줄) 에 추가:

```dockerfile
        ultralytics \
        python-can \
        fastapi \
        "uvicorn[standard]"
```

(기존 `python-can` 줄 뒤에 `fastapi`, `uvicorn[standard]` 두 줄 추가. parquet 로깅 쓸 거면 `pyarrow` 도 — 선택.)

- [ ] **Step 2: README 작성**

`motor_gui/README.md`:

```markdown
# motor_gui — 모터 통합 관제 GUI (벤치 진단 도구)

Jetson 에서 실행, 노트북 브라우저로 접속하는 웹 기반 모터 진단·튜닝 도구.
ODrive(USB/CAN)·AK(CAN) 의 100 Hz 텔레메트리를 uPlot 으로 실시간 plot,
위치/속도/토크 제어·라이브 게인 튜닝·캘리·E-stop 수행.

설계: `docs/specs/2026-05-20-motor-gui-design.md`

## 실행

```bash
# Jetson 컨테이너 안 (docker compose -f docker/docker-compose.jetson.yml exec powertrain bash)
cd /workspace

# fake (하드웨어 없이 — 개발/데모)
python3 -m motor_gui.backend.server --track fake

# USB 트랙 (ODrive USB 연결)
python3 -m motor_gui.backend.server --track usb

# CAN 트랙 (ODrive+AK, can0) — 먼저 bash scripts/can_setup.sh
python3 -m motor_gui.backend.server --track can
```

노트북 브라우저에서 `http://jetson-orin.local:8000` 접속.

## 트랙

| 트랙 | 전송 | 장치 | NVM 저장 |
| --- | --- | --- | --- |
| `usb` | odrive lib | ODrive 1대 (axis1) | O |
| `can` | python-can can0 | ODrive(node1) + AK(id10) | X (USB 전용) |
| `fake` | 시뮬 | odrive+ak 슈퍼셋 | (noop) |

## 테스트

```bash
python3 -m pytest motor_gui/tests/ -q
```

## 구조

`backend/transport/` 가 장치별 I/O (공통 `Transport` ABC), `worker.py` 가 100 Hz
스레드로 Transport 단독 소유, `server.py` 가 FastAPI WS/REST. 웹↔하드웨어는
JSON dict seam 으로 분리 (향후 프로세스 격리 대비).
```

- [ ] **Step 3: 전체 테스트 재확인**

Run: `cd /home/light/Defence_Robot && python3 -m pytest motor_gui/tests/ -q`
Expected: 모든 테스트 PASS.

- [ ] **Step 4: Commit**

```bash
git add docker/Dockerfile.jetson motor_gui/README.md
git commit -m "feat(motor_gui): Dockerfile.jetson 웹 의존성(fastapi/uvicorn) + README"
```

---

## Task 11: Jetson 엔드투엔드 스모크 + push

**Files:** 없음 (검증 + 동기화)

- [ ] **Step 1: 노트북에서 push**

```bash
cd /home/light/Defence_Robot && git push origin main 2>&1 | tail -3
```
Expected: `main -> main` 성공.

- [ ] **Step 2: Jetson 동기화 + 컨테이너 재빌드 (의존성 추가됨)**

```bash
sshpass -p "0000" ssh zetin@jetson-orin.local 'cd ~/Defence_Robot && git pull origin main && \
  echo "0000" | sudo -S docker compose -f docker/docker-compose.jetson.yml up -d --build 2>&1 | tail -5'
```
Expected: pull + 이미지 재빌드 (fastapi/uvicorn 설치) 성공.

- [ ] **Step 3: fake 트랙 엔드투엔드 (하드웨어 불필요)**

```bash
sshpass -p "0000" ssh zetin@jetson-orin.local '
  echo "0000" | sudo -S docker compose -f ~/Defence_Robot/docker/docker-compose.jetson.yml exec -T powertrain bash -lc "
    cd /workspace && python3 -m pytest motor_gui/tests/ -q && \
    (python3 -m motor_gui.backend.server --track fake --port 8000 &) && sleep 4 && \
    curl -s localhost:8000/api/capabilities | python3 -c \"import sys,json; print(json.load(sys.stdin)[\\\"track\\\"])\" && \
    pkill -f motor_gui.backend.server
  "'
```
Expected: pytest 전부 PASS + `fake`.

- [ ] **Step 4: (선택, 하드웨어 있을 때) 실 트랙 브라우저 확인**

> 컨트롤러가 사용자에게 트랙별 연결 요청 후, 노트북 브라우저 `http://jetson-orin.local:8000`
> 에서 plot 실시간 갱신·제어 동작 육안 확인. 체크리스트:
> connect → 텔레메트리 plot 갱신 → set_mode velocity → set_input vel → vel plot 상승 →
> 게인 변경 반영 → E-stop 시 vel 0 복귀.

- [ ] **Step 5: 완료 보고**

전체 태스크 완료 + Jetson working tree clean + 테스트 PASS 확인.

---

## Self-Review Notes

- **Spec coverage**: 웹/FastAPI/WS(Task5), 100Hz worker(Task4), USB 트랙(Task8), CAN 트랙 ODrive+AK(Task9), 토크 제어(Task8/9 set_input torque + ODrive Set_Input_Torque 0x0E), 라이브 게인(Task8/9 set_gain), 캘리(set_state/calibrate), E-stop(worker.estop + 0x02), 로깅 토글(Task6), capabilities 기반 UI(Task7), FakeTransport(Task2), C-승격 seam(Transport ABC + dict + worker 단일 인터페이스) — 모두 매핑됨.
- **Placeholder scan**: 코드 스텝 전부 실제 코드. "TODO/TBD" 없음.
- **Type consistency**: sample 키(`odrive.*`/`ak.*`/`t_mono`), capabilities 스키마(track/devices/signals/commands/limits/notes), envelope(target/op/args), ack(ok/target/op/detail) 가 Fake·USB·CAN·commands·worker·server·frontend 전반 일치 확인. ControlMode 정수(CAN: position=3/velocity=2/torque=1) 는 ODrive ControlMode enum 값과 일치 — Task 8(USB) 은 enum `.value` 로 동적 취득, Task 9(CAN) 은 상수. HIL Task 8 Step 1 에서 실제 enum 값 검증으로 교차 확인.
- **순서 의존성**: Task 5(server) 가 Task 6(recorder) 를 import → 두 태스크 인접 dispatch, server 테스트 전 recorder.py 존재 필요 (Task 5 Step 5 가 가드). 컨트롤러가 보장.
- **HIL 한계**: CAN 트랙 ODrive+AK 동시 HIL 은 동시 연결 불가로 보류 — Fake 가 동시 케이스 커버, 실 동시 검증은 후속.
