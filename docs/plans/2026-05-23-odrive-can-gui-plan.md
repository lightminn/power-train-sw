# ODrive-CAN GUI 구현 계획 (motor_gui 3단계)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ODrive 를 CAN(socketcan can0)으로 제어하는 `--track odrive_can` 트랙을 추가한다 — AK-CAN 의 컴포저블 `CanDevice` 구조를 재사용한 `OdriveCanDevice`, RTR 폴링 텔레메트리, 편집 가능한 Kt 튜너블, 3제어모드(position/position_traj/velocity).

**Architecture:** 보존된 모놀리식 `transport/can_bus.py` 의 ODrive CANSimple 로직을 `OdriveCanDevice(CanDevice)` 로 추출. `CanTransport([OdriveCanDevice()])` 로 조립하고 `--track odrive_can` 으로 노출. 프론트엔드는 capabilities 데이터-드리븐이라 변경 없음. setpoint 는 readback 불가라 명령값 로컬 추적(AkDevice 방식), torque_est 는 편집 가능한 Kt × Iq.

**Tech Stack:** Python 3, python-can (socketcan), struct(CANSimple little-endian), pytest, FastAPI(server). 설계 문서: `docs/specs/2026-05-23-odrive-can-gui-design.md`.

**실행 환경:** 모든 pytest 는 x86 dev 컨테이너 안에서 실행한다:
```bash
docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && <cmd>"
```
(HIL 은 Task 6 에서 Jetson `powertrain_jetson` + 실 ODrive 로 별도 진행.)

**브랜치:** 이미 `feat/odrive-can-gui` 에서 작업 중(spec 커밋 완료). 이 브랜치에 이어서 커밋.

---

## 파일 구조

- **Create** `motor_gui/backend/transport/odrive_can_device.py` — `OdriveCanDevice(CanDevice)` 단일 책임: ODrive 한 축의 CANSimple 인코딩/디코딩 + 상태 캐시 + 명령. 모놀리식 `can_bus.py` 의 ODrive 부분을 추출·개선(네이티브 영점, TRAP 헤드룸, 편집 Kt).
- **Create** `motor_gui/tests/test_odrive_can_device.py` — 가짜 버스 주입 단위 테스트.
- **Modify** `motor_gui/backend/server.py` — `_make_transport` 에 `odrive_can` 분기 + argparse choices 추가.
- **Modify** `motor_gui/tests/test_server.py` — `odrive_can` 트랙 capabilities 테스트 추가.
- **변경 없음**: 프론트엔드(`app.js`/`plots.js`), `commands.py`(set_param·pos/vel 키 이미 처리), `base.py`(`ODRIVE_TUNABLES_CAN`/`DEFAULT_TUNABLES`/`ODRIVE_INPUTS`/`SIGNAL_META` 이미 존재), `can_bus.py`(4단계까지 보존).

---

## Task 1: OdriveCanDevice 골격 — 상수·생성자·capabilities·attach

**Files:**
- Create: `motor_gui/backend/transport/odrive_can_device.py`
- Test: `motor_gui/tests/test_odrive_can_device.py`

이 태스크는 정적 capabilities 조각(connect 불필요)과 연결 시 기본 게인/한계 push 를 담당하는 `attach` 까지 만든다. 텔레메트리/명령은 Task 2·3.

- [ ] **Step 1: 실패 테스트 작성** — `motor_gui/tests/test_odrive_can_device.py`

```python
import struct
import can

from motor_gui.backend.transport.odrive_can_device import (
    OdriveCanDevice, NODE_ID, C_HEARTBEAT, C_SET_POS_GAIN, C_SET_VEL_GAINS,
    C_GET_ENC_EST, C_GET_IQ, C_GET_TEMP, C_GET_BUS_VI,
    C_SET_CTRL_MODE, C_SET_INPUT_POS, C_SET_INPUT_VEL, C_SET_LIMITS,
    C_SET_STATE, C_SET_LINEAR_COUNT, C_CLEAR_ERR, C_ESTOP,
    AXIS_IDLE, AXIS_CLOSED_LOOP, AXIS_FULL_CALIB,
)


class StubBus:
    def __init__(self):
        self.sent = []
    def send(self, msg, timeout=None):
        self.sent.append(msg)


def _arb(cmd, node=NODE_ID):
    return (node << 5) | cmd


def _sent_cmds(bus):
    return [m.arbitration_id & 0x1F for m in bus.sent]


def _mk():
    d = OdriveCanDevice()
    bus = StubBus()
    d.attach(bus)
    return d, bus


def test_capabilities_three_modes_no_torque():
    f = OdriveCanDevice().capabilities_fragment()
    assert f["devices"] == ["odrive"]
    assert f["control_modes"]["odrive"] == ["position", "position_traj", "velocity"]
    assert "torque" not in f["control_modes"]["odrive"]
    assert set(f["inputs"]["odrive"]) == {"position", "position_traj", "velocity"}


def test_capabilities_commands_include_set_param_not_save_nvm():
    f = OdriveCanDevice().capabilities_fragment()
    cmds = f["commands"]["odrive"]
    assert "set_param" in cmds
    assert "set_origin" in cmds
    assert "save_nvm" not in cmds


def test_capabilities_tunables_prefill_values():
    f = OdriveCanDevice().capabilities_fragment()
    tk = {t["key"]: t for t in f["tunables"]["odrive"]}
    # 게인/한계는 DEFAULT_TUNABLES 값으로 prefill
    assert tk["pos_gain"]["value"] == 8.0
    assert tk["vel_limit"]["value"] == 5.0
    assert tk["current_lim"]["value"] == 10.0
    # trap_vel_limit 은 vel_limit 에 결합 → UI 미노출
    assert "trap_vel_limit" not in tk
    # input_filter_bandwidth 는 CAN 미지원
    assert "input_filter_bandwidth" not in tk
    # 편집 가능한 토크 상수 Kt prefill
    assert tk["torque_constant"]["op"] == "set_param"
    assert abs(tk["torque_constant"]["value"] - 0.0084) < 1e-9


def test_signals_exclude_id_and_suberrors():
    f = OdriveCanDevice().capabilities_fragment()
    sig = f["signals"]
    assert "odrive.pos" in sig and "odrive.torque_est" in sig
    assert "odrive.id_meas" not in sig
    assert "odrive.motor_err" not in sig


def test_attach_pushes_default_gains():
    d, bus = _mk()
    cmds = _sent_cmds(bus)
    assert C_SET_POS_GAIN in cmds          # pos_gain push
    assert C_SET_VEL_GAINS in cmds         # vel gains push
    assert C_SET_LIMITS in cmds            # vel_limit/current_lim push
```

- [ ] **Step 2: 실패 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_odrive_can_device.py -q"`
Expected: FAIL — `ModuleNotFoundError: No module named '...odrive_can_device'`

- [ ] **Step 3: 최소 구현** — `motor_gui/backend/transport/odrive_can_device.py`

```python
from __future__ import annotations

import struct
import time

from .base import (SIGNAL_META, ODRIVE_INPUTS, ODRIVE_TUNABLES_CAN,
                   DEFAULT_TUNABLES)
from .can_device import CanDevice

NODE_ID = 1                              # ODrive axis1 (스크립트 컨벤션)

# ── CANSimple cmd id (fw-v0.5.6), arb = (node_id << 5) | cmd ──
C_HEARTBEAT = 0x001
C_ESTOP = 0x002
C_SET_STATE = 0x007
C_GET_ENC_EST = 0x009
C_SET_CTRL_MODE = 0x00B
C_SET_INPUT_POS = 0x00C
C_SET_INPUT_VEL = 0x00D
C_SET_LIMITS = 0x00F
C_SET_TRAJ_VEL_LIMIT = 0x011
C_SET_TRAJ_ACCEL_LIMITS = 0x012
C_GET_IQ = 0x014
C_GET_TEMP = 0x015
C_GET_BUS_VI = 0x017
C_CLEAR_ERR = 0x018
C_SET_LINEAR_COUNT = 0x019
C_SET_POS_GAIN = 0x01A
C_SET_VEL_GAINS = 0x01B

AXIS_IDLE = 1
AXIS_CLOSED_LOOP = 8
AXIS_FULL_CALIB = 3

# control_mode → (ControlMode, 기본 InputMode) fw-v0.5.6 정수.
# torque 모드 제외(CAN 으로 enable_current_mode_vel_limit 설정 불가 → runaway 위험).
_CTRL = {"position": 3, "position_traj": 3, "velocity": 2}
_IN_MODE = {"position": 3, "position_traj": 5, "velocity": 2}  # POS_FILTER/TRAP_TRAJ/VEL_RAMP
_CONTROL_MODES = ["position", "position_traj", "velocity"]

# trap_vel_limit 은 vel_limit 에 결합(헤드룸) → UI 튜너블에서 제거(windup 방지).
_BASE_TUNABLES = [t for t in ODRIVE_TUNABLES_CAN if t["key"] != "trap_vel_limit"]

_DEFAULT_KT = 0.0084                     # X2212-13 추정(8.27/980KV). UI 에서 편집 가능.

_SIGNALS = [
    "odrive.pos", "odrive.pos_setpoint", "odrive.vel", "odrive.vel_setpoint",
    "odrive.iq_meas", "odrive.iq_set", "odrive.torque_est",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus",
    "odrive.state", "odrive.axis_err",
]


class OdriveCanDevice(CanDevice):
    """ODrive 한 축 CANSimple 제어 (node1=axis1, fw-v0.5.6). 3모드, NVM 저장 불가."""

    name = "odrive"

    def __init__(self, node_id: int = NODE_ID) -> None:
        self._node = node_id
        self._bus = None
        self._state = {k: 0.0 for k in _SIGNALS}
        self._mode = "position"
        self._torque_const = _DEFAULT_KT
        self._vel_limit = float(DEFAULT_TUNABLES["vel_limit"])
        self._cur_lim = float(DEFAULT_TUNABLES["current_lim"])
        self._pos_setpoint = 0.0
        self._vel_setpoint = 0.0
        # pair-frame 명령(두 값을 한 프레임에) 부분 업데이트 병합용 캐시.
        self._vel_gains = {"vel_gain": float(DEFAULT_TUNABLES["vel_gain"]),
                           "vel_integrator_gain": float(DEFAULT_TUNABLES["vel_integrator_gain"])}
        self._trap = {"trap_accel_limit": float(DEFAULT_TUNABLES["trap_accel_limit"]),
                      "trap_decel_limit": float(DEFAULT_TUNABLES["trap_decel_limit"])}

    # ── 프레임 송신 헬퍼 ──
    def _arb(self, cmd: int) -> int:
        return (self._node << 5) | cmd

    def _send(self, cmd: int, data: bytes = b"") -> None:
        import can
        self._bus.send(can.Message(arbitration_id=self._arb(cmd), data=data,
                                   is_extended_id=False))

    def _request(self, cmd: int) -> None:
        import can
        self._bus.send(can.Message(arbitration_id=self._arb(cmd),
                                   is_remote_frame=True, is_extended_id=False))

    def _send_limits(self) -> None:
        """Set_Limits(vel_cap, current_lim) 1프레임. TRAP 은 캡에 헤드룸."""
        if self._mode == "position_traj":
            cur = abs(float(self._state.get("odrive.vel", 0.0)))
            cap = max(self._vel_limit * 1.3, cur * 1.3)
        else:
            cap = self._vel_limit
        self._send(C_SET_LIMITS, struct.pack("<ff", cap, self._cur_lim))

    def _sync_vel_limit(self) -> None:
        """TRAP 순항=vel_limit + 컨트롤러 하드캡(헤드룸) 동기."""
        self._send(C_SET_TRAJ_VEL_LIMIT, struct.pack("<f", self._vel_limit))
        self._send_limits()

    def attach(self, bus) -> None:
        self._bus = bus
        self._state = {k: 0.0 for k in _SIGNALS}
        self._mode = "position"
        self._pos_setpoint = 0.0
        self._vel_setpoint = 0.0
        # 기본 게인/한계 push → UI prefill 값과 실제 장치 일치.
        self._send(C_SET_POS_GAIN, struct.pack("<f", float(DEFAULT_TUNABLES["pos_gain"])))
        self._send(C_SET_VEL_GAINS, struct.pack("<ff",
                   self._vel_gains["vel_gain"], self._vel_gains["vel_integrator_gain"]))
        self._send(C_SET_TRAJ_ACCEL_LIMITS, struct.pack("<ff",
                   self._trap["trap_accel_limit"], self._trap["trap_decel_limit"]))
        self._sync_vel_limit()

    def capabilities_fragment(self) -> dict:
        meta = {k: SIGNAL_META[k] for k in _SIGNALS if k in SIGNAL_META}
        tunables = []
        for t in _BASE_TUNABLES:
            item = dict(t)
            if t["key"] in DEFAULT_TUNABLES:
                item["value"] = float(DEFAULT_TUNABLES[t["key"]])  # prefill
            tunables.append(item)
        tunables.append({
            "op": "set_param", "key": "torque_constant",
            "label": "토크 상수 Kt [Nm/A]", "value": self._torque_const,
            "help": "Iq→토크 환산용. 기본값은 X2212-13 추정(8.27/KV). "
                    "USB 트랙 모터정보 readout 값으로 교체 가능.",
        })
        return {
            "devices": ["odrive"],
            "signals": list(_SIGNALS),
            "commands": {"odrive": ["set_mode", "set_input", "set_gain",
                                    "set_limit", "set_state", "calibrate",
                                    "clear_errors", "set_param", "set_origin",
                                    "estop"]},
            "control_modes": {"odrive": list(_CONTROL_MODES)},
            "inputs": {"odrive": {m: ODRIVE_INPUTS[m] for m in _CONTROL_MODES}},
            "tunables": {"odrive": tunables},
            "limits": {"odrive": {"vel": 200.0, "pos": 100000.0}},
            "signal_meta": meta,
        }

    def sample(self) -> dict:
        return dict(self._state)        # Task 2 에서 setpoint/torque_est 보강

    def apply(self, bus, op: str, args: dict) -> dict:
        return {"ok": False, "target": "odrive", "op": op,
                "detail": "not implemented"}     # Task 3 에서 구현
```

- [ ] **Step 4: 통과 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_odrive_can_device.py -q"`
Expected: PASS (5 passed)

- [ ] **Step 5: 커밋**

```bash
git add motor_gui/backend/transport/odrive_can_device.py motor_gui/tests/test_odrive_can_device.py
git commit -m "feat(motor_gui): OdriveCanDevice 골격 — capabilities/attach + 편집 Kt 튜너블

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 2: 텔레메트리 — request(RTR)·on_rx(디코드)·sample

**Files:**
- Modify: `motor_gui/backend/transport/odrive_can_device.py`
- Test: `motor_gui/tests/test_odrive_can_device.py`

- [ ] **Step 1: 실패 테스트 추가** (test_odrive_can_device.py 하단에 추가)

```python
def _enc_msg(pos, vel, node=NODE_ID):
    return can.Message(arbitration_id=_arb(C_GET_ENC_EST, node),
                       data=struct.pack("<ff", pos, vel), is_extended_id=False)


def _iq_msg(iq_set, iq_meas):
    return can.Message(arbitration_id=_arb(C_GET_IQ),
                       data=struct.pack("<ff", iq_set, iq_meas), is_extended_id=False)


def _heartbeat_msg(axis_err=0, state=AXIS_IDLE):
    return can.Message(arbitration_id=_arb(C_HEARTBEAT),
                       data=struct.pack("<IB3x", axis_err, state), is_extended_id=False)


def test_request_sends_four_rtr_polls():
    d, bus = _mk()
    bus.sent.clear()
    d.request(bus)
    rtr = [m.arbitration_id & 0x1F for m in bus.sent if m.is_remote_frame]
    assert set(rtr) == {C_GET_ENC_EST, C_GET_IQ, C_GET_TEMP, C_GET_BUS_VI}


def test_on_rx_decodes_encoder_and_heartbeat():
    d, bus = _mk()
    d.on_rx(_enc_msg(2.5, -1.25))
    d.on_rx(_heartbeat_msg(axis_err=0x20, state=AXIS_CLOSED_LOOP))
    s = d.sample()
    assert abs(s["odrive.pos"] - 2.5) < 1e-6
    assert abs(s["odrive.vel"] + 1.25) < 1e-6
    assert s["odrive.state"] == AXIS_CLOSED_LOOP
    assert s["odrive.axis_err"] == 0x20


def test_on_rx_ignores_other_node_and_extended():
    d, bus = _mk()
    d.on_rx(_enc_msg(9.9, 9.9, node=NODE_ID + 1))      # 다른 node
    ext = can.Message(arbitration_id=0x2901, data=struct.pack("<ff", 5.0, 5.0),
                      is_extended_id=True)
    d.on_rx(ext)                                        # 확장 ID(AK)
    s = d.sample()
    assert s["odrive.pos"] == 0.0 and s["odrive.vel"] == 0.0


def test_sample_torque_est_is_iq_times_kt():
    d, bus = _mk()
    d.on_rx(_iq_msg(1.0, 2.0))                          # iq_meas=2.0
    s = d.sample()
    assert abs(s["odrive.iq_meas"] - 2.0) < 1e-6
    assert abs(s["odrive.torque_est"] - 2.0 * 0.0084) < 1e-9


def test_sample_includes_tracked_setpoints():
    d, bus = _mk()
    d._pos_setpoint = 3.0
    d._vel_setpoint = 4.0
    s = d.sample()
    assert s["odrive.pos_setpoint"] == 3.0
    assert s["odrive.vel_setpoint"] == 4.0
```

- [ ] **Step 2: 실패 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_odrive_can_device.py -q"`
Expected: FAIL — `test_request_sends_four_rtr_polls` 등 (request no-op, sample 미보강)

- [ ] **Step 3: 구현** — `odrive_can_device.py` 의 `sample` 교체 + `request`/`on_rx` 추가

`sample` 메서드를 아래로 교체:
```python
    def request(self, bus) -> None:
        for c in (C_GET_ENC_EST, C_GET_IQ, C_GET_TEMP, C_GET_BUS_VI):
            self._request(c)

    def on_rx(self, msg) -> None:
        if msg.is_extended_id:
            return
        if (msg.arbitration_id >> 5) != self._node:
            return
        cmd = msg.arbitration_id & 0x1F
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

    def sample(self) -> dict:
        s = dict(self._state)
        s["odrive.pos_setpoint"] = self._pos_setpoint
        s["odrive.vel_setpoint"] = self._vel_setpoint
        s["odrive.torque_est"] = float(self._state.get("odrive.iq_meas", 0.0)) * self._torque_const
        return s
```
(기존 한 줄짜리 `sample` 은 삭제하고 위 4개 메서드로 대체. `apply` 자리는 그대로 둔다.)

- [ ] **Step 4: 통과 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_odrive_can_device.py -q"`
Expected: PASS (10 passed)

- [ ] **Step 5: 커밋**

```bash
git add motor_gui/backend/transport/odrive_can_device.py motor_gui/tests/test_odrive_can_device.py
git commit -m "feat(motor_gui): OdriveCanDevice 텔레메트리 — RTR 폴링 + 디코드 + torque_est/setpoint

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 3: 명령 — apply 전체 (모드/입력/게인/한계/상태/영점/Kt/estop)

**Files:**
- Modify: `motor_gui/backend/transport/odrive_can_device.py`
- Test: `motor_gui/tests/test_odrive_can_device.py`

`apply` 한 메서드를 전부 구현한다 + `close`. 설계의 HIL 픽스(폐루프 점프 방지, TRAP 헤드룸/재계획, 네이티브 영점) 포함.

- [ ] **Step 1: 실패 테스트 추가** (test_odrive_can_device.py 하단에 추가)

```python
def _last(bus, cmd):
    """해당 cmd 의 마지막 송신 메시지(없으면 None)."""
    hits = [m for m in bus.sent if (m.arbitration_id & 0x1F) == cmd and not m.is_remote_frame]
    return hits[-1] if hits else None


def test_set_mode_position_sets_ctrl_and_holds_pos():
    d, bus = _mk()
    d.on_rx(_enc_msg(1.5, 0.0))
    bus.sent.clear()
    ack = d.apply(bus, "set_mode", {"control_mode": "position"})
    assert ack["ok"] is True
    cm = _last(bus, C_SET_CTRL_MODE)
    assert struct.unpack("<ii", cm.data) == (3, 3)        # POSITION / POS_FILTER
    ip = _last(bus, C_SET_INPUT_POS)
    pos, _vff, _tff = struct.unpack("<fhh", ip.data)
    assert abs(pos - 1.5) < 1e-6                          # 현재 위치 hold(점프 방지)
    assert abs(d._pos_setpoint - 1.5) < 1e-6


def test_set_mode_torque_rejected():
    d, bus = _mk()
    ack = d.apply(bus, "set_mode", {"control_mode": "torque"})
    assert ack["ok"] is False


def test_set_input_pos_sends_frame_and_tracks_setpoint():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "position"})
    bus.sent.clear()
    d.apply(bus, "set_input", {"pos": 4.0})
    ip = _last(bus, C_SET_INPUT_POS)
    pos, _v, _t = struct.unpack("<fhh", ip.data)
    assert abs(pos - 4.0) < 1e-6
    assert abs(d._pos_setpoint - 4.0) < 1e-6


def test_set_input_vel_tracks_vel_setpoint():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "velocity"})
    d.apply(bus, "set_input", {"vel": 2.5})
    iv = _last(bus, C_SET_INPUT_VEL)
    vel, _tff = struct.unpack("<ff", iv.data)
    assert abs(vel - 2.5) < 1e-6
    assert abs(d._vel_setpoint - 2.5) < 1e-6


def test_set_input_no_known_key_rejected():
    d, bus = _mk()
    ack = d.apply(bus, "set_input", {"bogus": 1.0})
    assert ack["ok"] is False


def test_set_gain_partial_vel_merges_cached_pair():
    d, bus = _mk()
    bus.sent.clear()
    # vel_gain 만 변경 → attach 가 캐시한 vel_integrator_gain 과 병합되어 한 프레임
    d.apply(bus, "set_gain", {"vel_gain": 0.05})
    vg = _last(bus, C_SET_VEL_GAINS)
    g, ig = struct.unpack("<ff", vg.data)
    assert abs(g - 0.05) < 1e-6
    assert abs(ig - 0.0) < 1e-6          # DEFAULT_TUNABLES vel_integrator_gain


def test_set_limit_velocity_mode_no_headroom():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "velocity"})
    bus.sent.clear()
    d.apply(bus, "set_limit", {"vel_limit": 10.0})
    lim = _last(bus, C_SET_LIMITS)
    cap, cur_lim = struct.unpack("<ff", lim.data)
    assert abs(cap - 10.0) < 1e-6        # velocity 모드 = 정확한 캡(헤드룸 없음)


def test_set_limit_traj_mode_has_headroom():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "position_traj"})
    bus.sent.clear()
    d.apply(bus, "set_limit", {"vel_limit": 10.0})
    lim = _last(bus, C_SET_LIMITS)
    cap, _cur = struct.unpack("<ff", lim.data)
    assert abs(cap - 13.0) < 1e-6        # max(10*1.3, 0) = 13 (헤드룸)


def test_set_param_torque_constant_updates_torque_est():
    d, bus = _mk()
    d.on_rx(_iq_msg(0.0, 3.0))
    d.apply(bus, "set_param", {"torque_constant": 0.02})
    s = d.sample()
    assert abs(s["odrive.torque_est"] - 3.0 * 0.02) < 1e-9


def test_set_origin_native_zero_sequence():
    d, bus = _mk()
    d.on_rx(_heartbeat_msg(state=AXIS_CLOSED_LOOP))
    bus.sent.clear()
    d.apply(bus, "set_origin", {})
    cmds = [m.arbitration_id & 0x1F for m in bus.sent if not m.is_remote_frame]
    # IDLE → Set_Linear_Count(0) → Input_Pos(0) → CLOSED_LOOP 복귀
    assert C_SET_LINEAR_COUNT in cmds
    lc = _last(bus, C_SET_LINEAR_COUNT)
    assert struct.unpack("<i", lc.data)[0] == 0
    states = [struct.unpack("<I", m.data)[0] for m in bus.sent
              if (m.arbitration_id & 0x1F) == C_SET_STATE]
    assert states[0] == AXIS_IDLE and states[-1] == AXIS_CLOSED_LOOP
    assert d._pos_setpoint == 0.0


def test_set_state_closed_loop_holds_pos():
    d, bus = _mk()
    d.on_rx(_enc_msg(2.0, 0.0))
    bus.sent.clear()
    d.apply(bus, "set_state", {"state": "closed_loop"})
    ip = _last(bus, C_SET_INPUT_POS)
    pos, _v, _t = struct.unpack("<fhh", ip.data)
    assert abs(pos - 2.0) < 1e-6         # 폐루프 진입 전 현재 위치 hold
    st = _last(bus, C_SET_STATE)
    assert struct.unpack("<I", st.data)[0] == AXIS_CLOSED_LOOP


def test_calibrate_and_clear_and_estop_frames():
    d, bus = _mk()
    bus.sent.clear()
    d.apply(bus, "calibrate", {})
    assert struct.unpack("<I", _last(bus, C_SET_STATE).data)[0] == AXIS_FULL_CALIB
    d.apply(bus, "clear_errors", {})
    assert _last(bus, C_CLEAR_ERR) is not None
    d.apply(bus, "estop", {})
    assert _last(bus, C_ESTOP) is not None


def test_unknown_op_rejected():
    d, bus = _mk()
    ack = d.apply(bus, "frobnicate", {})
    assert ack["ok"] is False
```

- [ ] **Step 2: 실패 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_odrive_can_device.py -q"`
Expected: FAIL — apply 가 항상 `not implemented` 반환

- [ ] **Step 3: 구현** — `odrive_can_device.py` 의 `apply` 자리 교체 + `close` 추가

```python
    def apply(self, bus, op: str, args: dict) -> dict:
        try:
            if op == "estop":
                self._send(C_ESTOP)
            elif op == "set_mode":
                mode = args.get("control_mode")
                if mode not in _CTRL:
                    return {"ok": False, "target": "odrive", "op": op,
                            "detail": f"unsupported mode {mode!r}"}
                self._mode = mode
                self._send(C_SET_CTRL_MODE, struct.pack("<ii", _CTRL[mode], _IN_MODE[mode]))
                self._sync_vel_limit()
                if mode in ("position", "position_traj"):
                    cur = float(self._state.get("odrive.pos", 0.0))
                    self._pos_setpoint = cur
                    self._send(C_SET_INPUT_POS, struct.pack("<fhh", cur, 0, 0))  # hold
                else:
                    self._vel_setpoint = 0.0
            elif op == "set_input":
                if "pos" in args:
                    self._pos_setpoint = float(args["pos"])
                    self._send(C_SET_INPUT_POS, struct.pack("<fhh", self._pos_setpoint, 0, 0))
                elif "vel" in args:
                    self._vel_setpoint = float(args["vel"])
                    self._send(C_SET_INPUT_VEL, struct.pack("<ff", self._vel_setpoint, 0.0))
                else:
                    return {"ok": False, "target": "odrive", "op": op,
                            "detail": "no known input key (pos/vel)"}
            elif op == "set_gain":
                if "pos_gain" in args:
                    self._send(C_SET_POS_GAIN, struct.pack("<f", float(args["pos_gain"])))
                if "vel_gain" in args or "vel_integrator_gain" in args:
                    for k in ("vel_gain", "vel_integrator_gain"):
                        if k in args:
                            self._vel_gains[k] = float(args[k])
                    self._send(C_SET_VEL_GAINS, struct.pack("<ff",
                               self._vel_gains["vel_gain"],
                               self._vel_gains["vel_integrator_gain"]))
                if "trap_accel_limit" in args or "trap_decel_limit" in args:
                    for k in ("trap_accel_limit", "trap_decel_limit"):
                        if k in args:
                            self._trap[k] = float(args[k])
                    self._send(C_SET_TRAJ_ACCEL_LIMITS, struct.pack("<ff",
                               self._trap["trap_accel_limit"],
                               self._trap["trap_decel_limit"]))
                # trap_vel_limit 은 무시(vel_limit 결합) — set_limit 에서만.
            elif op == "set_limit":
                if "vel_limit" in args:
                    self._vel_limit = float(args["vel_limit"])
                    self._sync_vel_limit()
                    # TRAP 진행 중 캡 변경 시 setpoint 재발행 → 새 순항속도로 재계획.
                    if (self._mode == "position_traj"
                            and int(self._state.get("odrive.state", 0)) == AXIS_CLOSED_LOOP):
                        self._send(C_SET_INPUT_POS,
                                   struct.pack("<fhh", self._pos_setpoint, 0, 0))
                if "current_lim" in args:
                    self._cur_lim = float(args["current_lim"])
                    self._send_limits()
            elif op == "set_state":
                if args.get("state") == "closed_loop":
                    cur = float(self._state.get("odrive.pos", 0.0))
                    self._pos_setpoint = cur
                    self._send(C_SET_INPUT_POS, struct.pack("<fhh", cur, 0, 0))  # jump 방지
                    self._send(C_SET_STATE, struct.pack("<I", AXIS_CLOSED_LOOP))
                else:
                    self._send(C_SET_STATE, struct.pack("<I", AXIS_IDLE))
            elif op == "calibrate":
                self._send(C_SET_STATE, struct.pack("<I", AXIS_FULL_CALIB))
            elif op == "clear_errors":
                self._send(C_CLEAR_ERR)
            elif op == "set_param":
                if "torque_constant" in args:
                    self._torque_const = float(args["torque_constant"])
            elif op == "set_origin":
                # 순정 영점: IDLE 디스암 → Set_Linear_Count(0) → Input_Pos(0) → 재무장.
                was_closed = int(self._state.get("odrive.state", 0)) == AXIS_CLOSED_LOOP
                self._send(C_SET_STATE, struct.pack("<I", AXIS_IDLE))
                time.sleep(0.2)
                self._send(C_SET_LINEAR_COUNT, struct.pack("<i", 0))
                self._pos_setpoint = 0.0
                self._send(C_SET_INPUT_POS, struct.pack("<fhh", 0.0, 0, 0))
                if was_closed:
                    self._send(C_SET_STATE, struct.pack("<I", AXIS_CLOSED_LOOP))
            else:
                return {"ok": False, "target": "odrive", "op": op,
                        "detail": "unsupported op"}
            return {"ok": True, "target": "odrive", "op": op, "detail": "sent"}
        except Exception as e:
            return {"ok": False, "target": "odrive", "op": op, "detail": str(e)}

    def close(self, bus) -> None:
        try:
            self._send(C_SET_STATE, struct.pack("<I", AXIS_IDLE))
        except Exception:
            pass
```
(기존 `not implemented` 짜리 `apply` 는 삭제하고 위로 대체.)

- [ ] **Step 4: 통과 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_odrive_can_device.py -q"`
Expected: PASS (23 passed)

- [ ] **Step 5: 커밋**

```bash
git add motor_gui/backend/transport/odrive_can_device.py motor_gui/tests/test_odrive_can_device.py
git commit -m "feat(motor_gui): OdriveCanDevice apply — 3모드/게인/한계(헤드룸)/네이티브 영점/Kt/estop

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 4: 트랙 배선 — server.py --track odrive_can

**Files:**
- Modify: `motor_gui/backend/server.py` (`_make_transport` 24-38, argparse 127)
- Test: `motor_gui/tests/test_server.py`

- [ ] **Step 1: 실패 테스트 추가** (test_server.py 하단)

```python
def test_make_transport_odrive_can_track():
    from motor_gui.backend.server import _make_transport
    t = _make_transport("odrive_can")
    caps = t.capabilities()                     # connect 없이 (정적 조각)
    assert caps["track"] == "can"
    assert caps["devices"] == ["odrive"]
    assert caps["control_modes"]["odrive"] == ["position", "position_traj", "velocity"]
    assert "set_param" in caps["commands"]["odrive"]
    tk = {t["key"]: t for t in caps["tunables"]["odrive"]}
    assert "torque_constant" in tk
```

- [ ] **Step 2: 실패 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_server.py::test_make_transport_odrive_can_track -q"`
Expected: FAIL — `ValueError: unknown track: 'odrive_can'`

- [ ] **Step 3: 구현** — `server.py` 수정

`_make_transport` 의 `ak` 분기 바로 뒤에 추가:
```python
    if track == "odrive_can":
        from .transport.can_device import CanTransport
        from .transport.odrive_can_device import OdriveCanDevice
        return CanTransport([OdriveCanDevice()], track="can")
```

argparse choices 한 줄 교체:
```python
    p.add_argument("--track", choices=["fake", "usb", "can", "ak", "odrive_can"], default="fake")
```

- [ ] **Step 4: 통과 확인**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/test_server.py -q"`
Expected: PASS (전부)

- [ ] **Step 5: 커밋**

```bash
git add motor_gui/backend/server.py motor_gui/tests/test_server.py
git commit -m "feat(motor_gui): --track odrive_can = CanTransport([OdriveCanDevice])

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 5: 전체 스위트 green + 최종 점검

**Files:** (없음 — 검증·정리만)

- [ ] **Step 1: 전체 테스트 실행**

Run: `docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/ -q"`
Expected: PASS — 기존 56 + 신규(약 25) 전부 green. 실패 시 해당 태스크로 돌아가 수정.

- [ ] **Step 2: 프론트엔드 무변경 확인**

`OdriveCanDevice` 가 내보내는 신호/모드/튜너블이 기존 `plots.js`(odrive.* 패널 `has()` 게이트)·`app.js`(튜너블 `t.value` prefill, 모드 셀렉터) 와 호환되는지 코드로 재확인. `id_meas`/`torque_est` 패널은 `has()` 로 자동 적응(Id 없으면 Iq 만). 변경 불필요 확인.

- [ ] **Step 3: 커밋(필요 시)** — 코드 변경 없으면 생략.

---

## Task 6: HIL 검증 (Jetson + 실 ODrive) — 컨트롤러 수행

**환경:** ODrive CAN 연결됨(2026-05-23 검증: node1 IDLE, RTR 응답 OK). Jetson `powertrain_jetson` 컨테이너, host-network. 배포는 `~/orin_mount` 마운트 cp. (운영 메모 jetson-deploy-can 참조)

> 이 태스크는 실하드웨어가 필요해 자동화 불가 — 서브에이전트가 아닌 컨트롤러(메인 세션)가 수행한다.

- [ ] **Step 1: 배포 + 기동**
```bash
cp motor_gui/backend/transport/odrive_can_device.py ~/orin_mount/Defence_Robot/motor_gui/backend/transport/
cp motor_gui/backend/server.py ~/orin_mount/Defence_Robot/motor_gui/backend/
# CAN 준비 + 서버 기동
sshpass -p 0000 ssh zetin@jetson-orin.local "cd ~/Defence_Robot && echo 0000 | sudo -S bash scripts/can_setup.sh && docker exec -d powertrain_jetson bash -lc 'cd /workspace && python3 -m motor_gui.backend.server --track odrive_can --port 8000 >/tmp/mg_ocan.log 2>&1'"
```

- [ ] **Step 2: 텔레메트리 확인** — 브라우저 `http://jetson-orin.local:8000` 또는 `curl http://jetson-orin.local:8000/api/capabilities`. pos/vel/iq/온도/Vbus 가 RTR 폴링으로 갱신되는지(0 아닌 실값).

- [ ] **Step 3: 폐루프 + 각 모드** — set_state closed_loop(점프 없는지) → position(목표 위치 이동) → position_traj(사다리꼴) → velocity(목표 속도). 각 모드에서 setpoint 오버레이(점선)가 실제(실선)와 짝맞는지.

- [ ] **Step 4: 네이티브 영점** — 임의 위치에서 set_origin → pos 0 복귀, fling 없는지.

- [ ] **Step 5: TRAP 속도캡 변경** — position_traj 이동 중 vel_limit 상향 → 스파이크 없이 부드럽게 변속되는지.

- [ ] **Step 6: Kt 텍스트박스** — 토크 상수 변경 → 추정토크 그래프 스케일 즉시 반영.

- [ ] **Step 7: 재연결/estop** — `/api/reconnect` 후 텔레메트리 정상, estop 시 IDLE.

- [ ] **Step 8: 발견된 버그 수정** — HIL 에서 나온 문제는 systematic-debugging 으로 진단 후 해당 코드 수정·재배포·재검증. (AK 때 spd÷10, reconnect bus-null 같은 펌웨어/프로토콜 함정 주의.)

- [ ] **Step 9: 메모리 기록** — HIL 학습을 `ak-can-gui` 패턴처럼 메모리에 갱신/추가.

---

## 완료 후

모든 태스크 완료 시 **superpowers:finishing-a-development-branch** 로 `feat/odrive-can-gui` → main 병합 절차 진행(전체 테스트 green 확인 → 병합 → 푸시 → 브랜치 정리).
