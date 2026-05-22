# AK-CAN GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** AK40 조향 모터를 CAN으로 제어하는 웹 GUI 트랙(`--track ak`)을, ODrive-CAN 동시제어까지 재작성 없이 확장되는 컴포저블 디바이스 구조로 구현한다.

**Architecture:** `CanTransport`(버스1개+디바이스 리스트 집계자) + `CanDevice` 인터페이스. `AkDevice`가 AK40을 래핑(4모드·워치독 재전송·과전류 자동정지). 프론트는 capabilities 기반이라 모드선택/타깃입력/튜닝패널 자동 렌더, AK fault 디코드만 추가.

**Tech Stack:** Python 3.10, python-can 4.6.1 (socketcan), FastAPI, pytest 9.0.3, vanilla JS. 설계 근거: `docs/specs/2026-05-21-ak-can-gui-design.md`.

**테스트 실행 환경:** x86 dev 컨테이너.
```
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest <경로> -v"
```
**HIL 환경:** Jetson `powertrain_jetson` 컨테이너(network_mode host → can0 공유). 사전 `bash scripts/can_setup.sh`. 접속: `sshpass -p 0000 ssh zetin@jetson-orin.local`, sudo는 `echo 0000 | sudo -S`. 파일 이동은 마운트 `~/orin_mount/Defence_Robot/`.

---

## File Structure

- `motor_control/steering/ak_control.py` (수정) — `AK40.send_brake`, `AK40.send_duty` 추가.
- `motor_gui/backend/transport/can_device.py` (신규) — `CanDevice` ABC + `CanTransport`.
- `motor_gui/backend/transport/ak_device.py` (신규) — `AkDevice`.
- `motor_gui/backend/transport/base.py` (수정) — `ak.speed` 단위 ERPM→RPM.
- `motor_gui/backend/server.py` (수정) — `--track ak` → `CanTransport([AkDevice()], track="ak")`.
- `motor_gui/frontend/app.js` (수정) — `AK_FAULT_CODES` + `ak.fault` 디코드.
- `motor_gui/tests/test_ak_control.py` (신규) — send_brake/send_duty 프레임.
- `motor_gui/tests/test_can_transport.py` (신규) — 라우팅/병합/capabilities.
- `motor_gui/tests/test_ak_device.py` (신규) — 모드별 프레임/파싱/과전류.

기존 `motor_gui/backend/transport/can_bus.py`(CanBackend)는 **보존**(3·4단계 OdriveCanDevice 추출 원본).

테스트 공용 스텁(각 테스트 파일 상단에 정의):
```python
import struct
import can

class StubBus:
    """python-can Bus 흉내: send 캡처, recv 는 미리 넣은 rx 큐에서."""
    def __init__(self, rx=None):
        self.sent = []
        self._rx = list(rx or [])
    def send(self, msg, timeout=None):
        self.sent.append(msg)
    def recv(self, timeout=None):
        return self._rx.pop(0) if self._rx else None
    def shutdown(self):
        pass
```

---

## Task 1: ak_control 에 send_brake / send_duty 추가

**Files:**
- Modify: `motor_control/steering/ak_control.py` (PKT 상수 위 + AK40 메서드)
- Test: `motor_gui/tests/test_ak_control.py`

- [ ] **Step 1: 실패 테스트 작성**

`motor_gui/tests/test_ak_control.py`:
```python
import struct
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "motor_control" / "steering"))
from ak_control import AK40  # noqa: E402


class StubBus:
    def __init__(self):
        self.sent = []
    def send(self, msg, timeout=None):
        self.sent.append(msg)


def test_send_brake_frame():
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    assert m.send_brake(2.0) is True
    msg = bus.sent[-1]
    assert msg.is_extended_id is True
    assert msg.arbitration_id == (2 << 8) | 10        # PKT_SET_BRAKE=2, id=10
    assert msg.data == struct.pack(">i", 2000)        # 2.0A → 2000 mA


def test_send_duty_frame():
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    assert m.send_duty(0.5) is True
    msg = bus.sent[-1]
    assert msg.arbitration_id == (0 << 8) | 10        # PKT_SET_DUTY=0
    assert msg.data == struct.pack(">i", 50000)       # 0.5 × 100000


def test_send_duty_clamps():
    bus = StubBus()
    m = AK40(bus, 10, name="ak")
    m.send_duty(5.0)                                  # 과도 → 0.95 클램프
    assert bus.sent[-1].data == struct.pack(">i", 95000)
```

- [ ] **Step 2: 실패 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_ak_control.py -v"`
Expected: FAIL — `AttributeError: 'AK40' object has no attribute 'send_brake'`

- [ ] **Step 3: 구현**

`ak_control.py` 의 `AK40` 클래스에 (예: `send_rpm_out` 메서드 아래) 추가:
```python
    def send_brake(self, current_a):
        """전류 기반 브레이크 (VESC: mA). 0~20A 클램프."""
        cur = max(0.0, min(20.0, float(current_a)))
        return self._send(PKT_SET_BRAKE, struct.pack(">i", int(cur * 1000)))

    def send_duty(self, duty):
        """직접 듀티 (-0.95~0.95 클램프, VESC: ×100000)."""
        d = max(-0.95, min(0.95, float(duty)))
        return self._send(PKT_SET_DUTY, struct.pack(">i", int(d * 100000)))
```

- [ ] **Step 4: 통과 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_ak_control.py -v"`
Expected: PASS (3 passed)

- [ ] **Step 5: 커밋**

```bash
git add motor_control/steering/ak_control.py motor_gui/tests/test_ak_control.py
git commit -m "feat(steering): AK40 send_brake/send_duty 추가 (VESC 브레이크/듀티 패킷)"
```

---

## Task 2: CanDevice 인터페이스 + CanTransport 집계자

**Files:**
- Create: `motor_gui/backend/transport/can_device.py`
- Test: `motor_gui/tests/test_can_transport.py`

- [ ] **Step 1: 실패 테스트 작성**

`motor_gui/tests/test_can_transport.py`:
```python
from motor_gui.backend.transport.can_device import CanDevice, CanTransport


class StubBus:
    def __init__(self, rx=None):
        self.sent = []
        self._rx = list(rx or [])
    def send(self, msg, timeout=None):
        self.sent.append(msg)
    def recv(self, timeout=None):
        return self._rx.pop(0) if self._rx else None
    def shutdown(self):
        pass


class StubDevice(CanDevice):
    name = "stub"
    def __init__(self):
        self.attached = None
        self.rx_count = 0
    def attach(self, bus):
        self.attached = bus
    def capabilities_fragment(self):
        return {"devices": ["stub"], "signals": ["stub.x"],
                "commands": {"stub": ["ping"]}, "control_modes": {},
                "inputs": {}, "tunables": {}, "limits": {"stub": {}},
                "signal_meta": {"stub.x": {"label": "X", "unit": ""}}}
    def on_rx(self, msg):
        self.rx_count += 1
    def sample(self):
        return {"stub.x": 1.0}
    def apply(self, bus, op, args):
        return {"ok": True, "target": "stub", "op": op, "detail": "ok"}


def test_capabilities_merges_device_fragments():
    t = CanTransport([StubDevice()], track="ak", bus=StubBus())
    caps = t.capabilities()
    assert caps["track"] == "ak"
    assert caps["devices"] == ["stub"]
    assert "stub.x" in caps["signals"]
    assert caps["commands"]["stub"] == ["ping"]
    assert caps["signal_meta"]["stub.x"]["label"] == "X"


def test_sample_merges_and_has_t_mono():
    t = CanTransport([StubDevice()], bus=StubBus())
    t.connect()
    s = t.sample()
    assert "t_mono" in s and s["stub.x"] == 1.0


def test_apply_routes_by_target():
    dev = StubDevice()
    t = CanTransport([dev], bus=StubBus())
    t.connect()
    ack = t.apply({"target": "stub", "op": "ping", "args": {}})
    assert ack["ok"] is True
    bad = t.apply({"target": "nope", "op": "ping", "args": {}})
    assert bad["ok"] is False and bad["detail"] == "unknown target"


def test_attach_called_on_connect():
    dev = StubDevice()
    bus = StubBus()
    t = CanTransport([dev], bus=bus)
    t.connect()
    assert dev.attached is bus
```

- [ ] **Step 2: 실패 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_can_transport.py -v"`
Expected: FAIL — `ModuleNotFoundError: ... can_device`

- [ ] **Step 3: 구현**

`motor_gui/backend/transport/can_device.py`:
```python
from __future__ import annotations

import time
from abc import ABC, abstractmethod

from .base import Transport, TransportError


class CanDevice(ABC):
    """공유 CAN 버스 위 한 모터 유닛. CanTransport 가 집계한다."""

    name: str = "base"

    @abstractmethod
    def attach(self, bus) -> None:
        """공유 버스 주입 (connect 시 1회)."""

    @abstractmethod
    def capabilities_fragment(self) -> dict:
        """이 디바이스의 signals/commands/control_modes/inputs/tunables/limits/signal_meta 조각."""

    def request(self, bus) -> None:
        """폴링형 디바이스의 RTR 송신. 기본 no-op."""

    def on_rx(self, msg) -> None:
        """내 프레임이면 캐시 상태 갱신. 기본 no-op."""

    def tick(self, bus) -> None:
        """워치독 재전송 등 주기 동작. 기본 no-op."""

    @abstractmethod
    def sample(self) -> dict:
        """캐시 상태 → 텔레메트리 조각."""

    @abstractmethod
    def apply(self, bus, op: str, args: dict) -> dict:
        """이 디바이스 대상 명령 처리. ack dict."""

    def close(self, bus) -> None:
        """안전 정지. 기본 no-op."""


class CanTransport(Transport):
    """can0 버스 1개 + CanDevice 리스트 집계 (Transport 계약 구현)."""

    name = "can"

    def __init__(self, devices, channel: str = "can0",
                 track: str = "can", bus=None) -> None:
        self._devices = devices
        self._channel = channel
        self._track = track
        self._bus = bus              # 주입 시 테스트용 (socketcan open 생략)
        self._owns_bus = bus is None

    def connect(self) -> None:
        if self._bus is None:
            import can
            try:
                self._bus = can.interface.Bus(channel=self._channel,
                                              interface="socketcan")
            except OSError as e:
                raise TransportError(
                    f"{self._channel} open 실패 — 'bash scripts/can_setup.sh' 먼저 ({e})")
        for d in self._devices:
            d.attach(self._bus)

    def sample(self) -> dict:
        for d in self._devices:
            d.request(self._bus)
        deadline = time.monotonic() + 0.008
        while time.monotonic() < deadline:
            msg = self._bus.recv(timeout=0.002)
            if msg is None:
                break
            for d in self._devices:
                d.on_rx(msg)
        for d in self._devices:
            d.tick(self._bus)
        s = {"t_mono": time.monotonic()}
        for d in self._devices:
            s.update(d.sample())
        return s

    def apply(self, cmd: dict) -> dict:
        target = cmd.get("target")
        for d in self._devices:
            if d.name == target:
                return d.apply(self._bus, cmd["op"], cmd.get("args", {}))
        return {"ok": False, "target": target,
                "op": cmd.get("op"), "detail": "unknown target"}

    def capabilities(self) -> dict:
        caps = {"track": self._track, "devices": [], "signals": [],
                "commands": {}, "control_modes": {}, "inputs": {},
                "tunables": {}, "limits": {}, "signal_meta": {},
                "notes": ["CAN 트랙 — NVM 저장 불가 (USB 전용)"]}
        for d in self._devices:
            f = d.capabilities_fragment()
            caps["devices"] += f.get("devices", [])
            caps["signals"] += f.get("signals", [])
            for key in ("commands", "control_modes", "inputs", "tunables", "limits"):
                caps[key].update(f.get(key, {}))
            caps["signal_meta"].update(f.get("signal_meta", {}))
        return caps

    def close(self) -> None:
        for d in self._devices:
            try:
                d.close(self._bus)
            except Exception:
                pass
        if self._owns_bus and self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
```

- [ ] **Step 4: 통과 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_can_transport.py -v"`
Expected: PASS (4 passed)

- [ ] **Step 5: 커밋**

```bash
git add motor_gui/backend/transport/can_device.py motor_gui/tests/test_can_transport.py
git commit -m "feat(motor_gui): CanDevice 인터페이스 + CanTransport 집계자 (컴포저블 CAN 구조)"
```

---

## Task 3: base.py — ak.speed 단위 RPM 으로

**Files:**
- Modify: `motor_gui/backend/transport/base.py` (SIGNAL_META 의 `ak.speed`)

- [ ] **Step 1: 수정**

`base.py` 의 `SIGNAL_META` 에서:
```python
    "ak.speed": {"label": "AK 속도", "unit": "ERPM"},
```
를 다음으로 변경:
```python
    "ak.speed": {"label": "AK 속도(출력축)", "unit": "RPM"},
```

- [ ] **Step 2: 기존 테스트 깨지지 않음 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/ -v"`
Expected: 기존 전부 PASS (SIGNAL_META 키는 그대로, 라벨/단위만 변경).

- [ ] **Step 3: 커밋**

```bash
git add motor_gui/backend/transport/base.py
git commit -m "chore(motor_gui): ak.speed 단위 ERPM→출력축 RPM (명령 단위와 일치)"
```

---

## Task 4: AkDevice 구현

**Files:**
- Create: `motor_gui/backend/transport/ak_device.py`
- Test: `motor_gui/tests/test_ak_device.py`

- [ ] **Step 1: 실패 테스트 작성**

`motor_gui/tests/test_ak_device.py`:
```python
import struct
import can

from motor_gui.backend.transport.ak_device import AkDevice, PKT_STATUS_1, AK_ID


class StubBus:
    def __init__(self):
        self.sent = []
    def send(self, msg, timeout=None):
        self.sent.append(msg)


def _status_msg(pos_deg=12.0, spd_erpm=0, cur_a=0.0, temp=40, fault=0):
    # AK40._parse_status: ">hhhbb" = pos*10, spd/10, cur*100, temp, fault
    data = struct.pack(">hhhbb", int(pos_deg * 10), int(spd_erpm / 10),
                       int(cur_a * 100), temp, fault)
    return can.Message(arbitration_id=(PKT_STATUS_1 << 8) | AK_ID,
                       data=data, is_extended_id=True)


def _mk():
    d = AkDevice()
    bus = StubBus()
    d.attach(bus)
    return d, bus


def test_capabilities_fragment_modes_and_commands():
    f = AkDevice().capabilities_fragment()
    assert f["devices"] == ["ak"]
    assert f["control_modes"]["ak"] == ["position", "velocity", "brake", "duty"]
    assert "set_param" in f["commands"]["ak"]
    assert f["inputs"]["ak"]["velocity"]["key"] == "rpm"
    assert "ak.fault" in f["signals"]


def test_set_input_velocity_sends_rpm_frame():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "velocity"})
    d.apply(bus, "set_input", {"rpm": 30.0})
    # PKT_SET_RPM=3
    assert bus.sent[-1].arbitration_id == (3 << 8) | AK_ID


def test_set_input_brake_sends_brake_frame():
    d, bus = _mk()
    d.apply(bus, "set_input", {"brake_cur": 2.0})
    assert bus.sent[-1].arbitration_id == (2 << 8) | AK_ID
    assert bus.sent[-1].data == struct.pack(">i", 2000)


def test_on_rx_parses_status_and_sample_converts_speed():
    d, bus = _mk()
    # POLE_PAIRS=14, GEAR_RATIO=10 → out_rpm = spd_erpm / 140
    d.on_rx(_status_msg(pos_deg=12.0, spd_erpm=1400, cur_a=1.5, temp=42, fault=0))
    s = d.sample()
    assert abs(s["ak.pos_deg"] - 12.0) < 0.1
    assert abs(s["ak.speed"] - 10.0) < 0.2          # 1400 erpm / 140
    assert abs(s["ak.current"] - 1.5) < 0.05
    assert s["ak.temp"] == 42


def test_set_param_then_position_uses_spd():
    d, bus = _mk()
    d.apply(bus, "set_param", {"spd_erpm": 2222, "acc_erpm_s2": 3333, "max_cur_a": 7.0})
    d.apply(bus, "set_input", {"pos_deg": 90.0})
    # PKT_SET_POS_SPD=6, data=">ihh" (deg*1e4, spd, acc)
    msg = bus.sent[-1]
    assert msg.arbitration_id == (6 << 8) | AK_ID
    pos, spd, acc = struct.unpack(">ihh", msg.data)
    assert pos == 900000 and spd == 2222 and acc == 3333


def test_overcurrent_trips_in_tick():
    d, bus = _mk()
    d.apply(bus, "set_param", {"max_cur_a": 3.0})
    d.apply(bus, "set_input", {"rpm": 50.0})
    d.on_rx(_status_msg(cur_a=5.0))     # 한계 초과
    n_before = len(bus.sent)
    d.tick(bus)
    # rpm0 정지 프레임 송신 + active 해제
    assert len(bus.sent) > n_before
    assert d._active is None
```

- [ ] **Step 2: 실패 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_ak_device.py -v"`
Expected: FAIL — `ModuleNotFoundError: ... ak_device`

- [ ] **Step 3: 구현**

`motor_gui/backend/transport/ak_device.py`:
```python
from __future__ import annotations

import sys
import time
from pathlib import Path

from .base import SIGNAL_META
from .can_device import CanDevice

# steering/ak_control.py 의 AK40 재사용 (hw 로직 단일 소스)
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "motor_control" / "steering"))
from ak_control import AK40, POLE_PAIRS, GEAR_RATIO  # noqa: E402

AK_ID = 10
PKT_STATUS_1 = 41
_AK_SIGNALS = ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp", "ak.fault"]
_RESEND_HZ = 20.0


class AkDevice(CanDevice):
    """AK40 조향 모터 (확장 CAN ID). 4모드 + 워치독 재전송 + 과전류 자동정지."""

    name = "ak"

    def __init__(self, motor_id: int = AK_ID) -> None:
        self._mid = motor_id
        self._ak = None
        self._mode = "position"
        self._active = None          # 재전송할 무인자 호출 (워치독)
        self._spd = 1500.0
        self._acc = 6000.0
        self._maxcur = 5.0
        self._last_send = 0.0
        self._tripped = False

    def attach(self, bus) -> None:
        self._ak = AK40(bus, self._mid, name="ak")

    def capabilities_fragment(self) -> dict:
        meta = {k: SIGNAL_META[k] for k in _AK_SIGNALS if k in SIGNAL_META}
        return {
            "devices": ["ak"],
            "signals": list(_AK_SIGNALS),
            "commands": {"ak": ["set_mode", "set_input", "set_param",
                                 "set_origin", "estop"]},
            "control_modes": {"ak": ["position", "velocity", "brake", "duty"]},
            "inputs": {"ak": {
                "position": {"key": "pos_deg", "label": "목표 위치", "unit": "°"},
                "velocity": {"key": "rpm", "label": "목표 속도(출력축)", "unit": "RPM"},
                "brake": {"key": "brake_cur", "label": "브레이크 전류", "unit": "A"},
                "duty": {"key": "duty", "label": "듀티", "unit": ""},
            }},
            "tunables": {"ak": [
                {"op": "set_param", "key": "spd_erpm", "label": "속도제한 [ERPM]"},
                {"op": "set_param", "key": "acc_erpm_s2", "label": "가속 [ERPM/s²]"},
                {"op": "set_param", "key": "max_cur_a", "label": "최대전류(자동정지) [A]"},
            ]},
            "limits": {"ak": {"pos_deg": 100000.0, "rpm": 1000.0,
                              "brake_cur": 20.0, "duty": 1.0}},
            "signal_meta": meta,
        }

    def on_rx(self, msg) -> None:
        if not msg.is_extended_id:
            return
        pkt = (msg.arbitration_id >> 8) & 0xFF
        nid = msg.arbitration_id & 0xFF
        if pkt == PKT_STATUS_1 and nid == self._mid and len(msg.data) >= 8:
            self._ak._parse_status(msg.data)

    def tick(self, bus) -> None:
        if self._ak is None:
            return
        if abs(self._ak.cur_a) > self._maxcur:      # 과전류 자동정지
            self._ak.send_rpm_out(0)
            self._active = None
            self._tripped = True
            return
        now = time.monotonic()
        if self._active is not None and now - self._last_send >= 1.0 / _RESEND_HZ:
            self._active()
            self._last_send = now

    def sample(self) -> dict:
        a = self._ak
        if a is None:
            return {k: 0.0 for k in _AK_SIGNALS}
        out_rpm = a.spd_erpm / (POLE_PAIRS * GEAR_RATIO)
        return {
            "ak.pos_deg": float(a.pos_out_deg),
            "ak.speed": float(out_rpm),
            "ak.current": float(a.cur_a),
            "ak.temp": float(a.temp_c),
            "ak.fault": int(a.fault),
        }

    def apply(self, bus, op: str, args: dict) -> dict:
        a = self._ak
        try:
            if op == "estop":
                a.stop()
                self._active = None
            elif op == "set_mode":
                self._mode = args["control_mode"]
                self._tripped = False
                if self._mode == "position":
                    tgt = a.pos_out_deg
                    self._active = lambda: a.send_pos_out(tgt, self._spd, self._acc)
                elif self._mode == "velocity":
                    self._active = lambda: a.send_rpm_out(0.0)
                elif self._mode == "brake":
                    self._active = lambda: a.send_brake(0.0)
                else:  # duty
                    self._active = lambda: a.send_duty(0.0)
                self._active()
            elif op == "set_input":
                if "pos_deg" in args:
                    tgt = float(args["pos_deg"])
                    self._active = lambda: a.send_pos_out(tgt, self._spd, self._acc)
                elif "rpm" in args:
                    v = float(args["rpm"])
                    self._active = lambda: a.send_rpm_out(v)
                elif "brake_cur" in args:
                    v = float(args["brake_cur"])
                    self._active = lambda: a.send_brake(v)
                elif "duty" in args:
                    v = float(args["duty"])
                    self._active = lambda: a.send_duty(v)
                else:
                    return {"ok": False, "target": "ak", "op": op,
                            "detail": "no known input key"}
                self._active()
            elif op == "set_param":
                if "spd_erpm" in args:
                    self._spd = float(args["spd_erpm"])
                if "acc_erpm_s2" in args:
                    self._acc = float(args["acc_erpm_s2"])
                if "max_cur_a" in args:
                    self._maxcur = float(args["max_cur_a"])
            elif op == "set_origin":
                a.set_origin_here()
            else:
                return {"ok": False, "target": "ak", "op": op,
                        "detail": "unsupported op"}
            return {"ok": True, "target": "ak", "op": op, "detail": "sent"}
        except Exception as e:
            return {"ok": False, "target": "ak", "op": op, "detail": str(e)}

    def close(self, bus) -> None:
        if self._ak is not None:
            try:
                self._ak.stop()
            except Exception:
                pass
```

- [ ] **Step 4: 통과 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_ak_device.py -v"`
Expected: PASS (6 passed)

- [ ] **Step 5: 커밋**

```bash
git add motor_gui/backend/transport/ak_device.py motor_gui/tests/test_ak_device.py
git commit -m "feat(motor_gui): AkDevice — AK40 4모드/워치독/과전류정지 (CanDevice 구현)"
```

---

## Task 5: server.py 에 --track ak 추가

**Files:**
- Modify: `motor_gui/backend/server.py` (`_make_transport` + argparse choices)
- Test: `motor_gui/tests/test_server.py` (케이스 추가)

- [ ] **Step 1: 실패 테스트 작성** — `motor_gui/tests/test_server.py` 끝에 추가:

```python
def test_make_transport_ak_track_capabilities():
    from motor_gui.backend.server import _make_transport
    t = _make_transport("ak")
    caps = t.capabilities()                     # connect 없이 (정적 조각)
    assert caps["track"] == "ak"
    assert caps["devices"] == ["ak"]
    assert caps["control_modes"]["ak"] == ["position", "velocity", "brake", "duty"]
    assert "set_param" in caps["commands"]["ak"]
```

- [ ] **Step 2: 실패 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_server.py::test_make_transport_ak_track_capabilities -v"`
Expected: FAIL — `ValueError: unknown track: 'ak'`

- [ ] **Step 3: 구현** — `server.py` 의 `_make_transport` 에 분기 추가 (`track == "can"` 분기 위):

```python
    if track == "ak":
        from .transport.can_device import CanTransport
        from .transport.ak_device import AkDevice
        return CanTransport([AkDevice()], track="ak")
```

그리고 argparse choices 에 `"ak"` 추가:
```python
    p.add_argument("--track", choices=["fake", "usb", "can", "ak"], default="fake")
```

- [ ] **Step 4: 통과 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_server.py -v"`
Expected: 기존 + 신규 전부 PASS

- [ ] **Step 5: 커밋**

```bash
git add motor_gui/backend/server.py motor_gui/tests/test_server.py
git commit -m "feat(motor_gui): --track ak → CanTransport([AkDevice]) 런처"
```

---

## Task 6: 프론트 AK fault 디코드

**Files:**
- Modify: `motor_gui/frontend/app.js` (ERROR_BITS 블록 근처 + monitorSample)

> JS 단위 테스트 하네스 없음 → 코드 변경 후 검증은 Task 7(HIL). 변경은 가산적이며 기존 동작 불변.

- [ ] **Step 1: AK_FAULT_CODES + 디코더 추가** — `app.js` 의 `decodeErr` 함수 정의 바로 아래에 추가:

```javascript
// VESC/AK mc_fault_code (enum 값 → 이름; 비트필드 아님)
const AK_FAULT_CODES = {
  0: "NONE", 1: "OVER_VOLTAGE", 2: "UNDER_VOLTAGE", 3: "DRV", 4: "ABS_OVER_CURRENT",
  5: "OVER_TEMP_FET", 6: "OVER_TEMP_MOTOR", 7: "GATE_DRIVER_OVER_VOLTAGE",
  8: "GATE_DRIVER_UNDER_VOLTAGE", 9: "MCU_UNDER_VOLTAGE", 10: "BOOTING_FROM_WATCHDOG_RESET",
  11: "ENCODER_SPI", 12: "ENCODER_SINCOS_BELOW_MIN", 13: "ENCODER_SINCOS_ABOVE_MAX",
  14: "FLASH_CORRUPTION", 18: "UNBALANCED_CURRENTS",
};
function decodeAkFault(v) {
  return AK_FAULT_CODES[v] !== undefined ? `${v} (${AK_FAULT_CODES[v]})` : String(v);
}
```

- [ ] **Step 2: monitorSample 의 ak.fault 로깅 교체** — 기존:

```javascript
      if (v !== 0) logMsg(`ak.fault = ${v}`, "err");
```
를 다음으로:
```javascript
      if (v !== 0) logMsg(`ak.fault = ${decodeAkFault(v)}`, "err");
```

- [ ] **Step 3: 문법 확인 (node 파싱)**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && node --check motor_gui/frontend/app.js && echo OK"`
Expected: `OK` (node 있으면. 없으면 이 스텝 생략하고 Task 7 에서 브라우저 로드로 확인)

- [ ] **Step 4: 커밋**

```bash
git add motor_gui/frontend/app.js
git commit -m "feat(motor_gui): AK fault 코드 디코드 (VESC mc_fault_code 이름 표시)"
```

---

## Task 7: HIL 검증 (실 AK, Jetson)

**Files:** 없음 (검증 전용). 결과는 `docs/plans/2026-05-21-ak-can-gui-plan.md` 하단 또는 별도 verification 로그에 기록.

> 코드 task 아님 — 실하드웨어 동작 확인. 이미 AK 가 can0 에 연결돼 있음.

- [ ] **Step 1: 배포 + can0 기동** (마운트로 동기 또는 git pull)

```bash
# 로컬에서 push 후 Jetson pull, 또는 마운트로 직접:
# rsync 변경분 → ~/orin_mount/Defence_Robot/
sshpass -p "0000" ssh zetin@jetson-orin.local 'cd ~/Defence_Robot && git pull origin main'
sshpass -p "0000" ssh zetin@jetson-orin.local 'cd ~/Defence_Robot && echo "0000" | sudo -S bash scripts/can_setup.sh'
```

- [ ] **Step 2: ak 트랙 서버 기동**

```bash
sshpass -p "0000" ssh zetin@jetson-orin.local 'cd ~/Defence_Robot && echo "0000" | sudo -S docker compose -f docker/docker-compose.jetson.yml exec -d powertrain bash -lc "cd /workspace && python3 -m motor_gui.backend.server --track ak --port 8000 >/tmp/mg_ak.log 2>&1"'
```
검증: `curl http://jetson-orin.local:8000/api/capabilities` → track="ak", devices=["ak"], control_modes 4개.

- [ ] **Step 3: 브라우저 기능 확인** (각 항목 체크)

  - [ ] WS 텔레메트리: ak.pos_deg/speed/current/temp 그래프 갱신
  - [ ] 영점 설정 버튼 → pos 0 기준 재설정
  - [ ] position 모드 + 목표 90° → 이동, 그래프 추종
  - [ ] velocity 모드 + 30 RPM → 연속 회전 (워치독 재전송으로 유지)
  - [ ] brake 모드 + 2A → 제동/홀딩
  - [ ] duty 모드 + 0.1 → 저속 구동
  - [ ] 튜닝: spd_erpm/acc_erpm_s2 변경 후 position 이동 동특성 변화 확인
  - [ ] 과전류 자동정지: max_cur_a 낮춰(예 1A) 부하 시 자동 정지 + 로그
  - [ ] fault 발생 시 로그에 이름 디코드 표시
  - [ ] CSV 로깅 시작/종료 → `logs/motor_ak_*.csv` 생성, ak.* 컬럼 포함

- [ ] **Step 4: 안전 정지 + 결과 기록**

서버 종료, 결과를 커밋 메시지/메모리에 기록.

---

## 자기 검토 메모 (작성자 self-review)

- **Spec 커버리지**: 4모드(T1·T4), 워치독(T4 tick), 과전류정지(T4), 튜닝(T4 set_param + caps), 신호/속도변환(T3·T4), CanDevice/CanTransport(T2), 런처(T5), fault디코드(T6), HIL(T7). ODrive-CAN 은 범위 밖(다음 spec). ✅
- **타입 일관성**: `CanDevice` 메서드 시그니처가 T2 정의 ↔ T4 AkDevice 구현 일치(attach/capabilities_fragment/on_rx/tick/sample/apply/close). `_active` 무인자 closure 규약 일관. `set_param` 키(spd_erpm/acc_erpm_s2/max_cur_a) T4 caps ↔ apply ↔ 테스트 일치. ✅
- **플레이스홀더 없음**: 모든 코드 스텝에 실제 코드/명령/기대출력 포함. ✅
