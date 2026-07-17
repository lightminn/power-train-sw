# A2a 배치 구현 계획 — ops broker(:9001) + 역할 토큰 + 복구 chord

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **이 레포의 실행 관례:** Codex 위임(⚠️ git 명령 금지 — 커밋은 리뷰어) + 리뷰어 3환경 검증. A1 교훈: Codex 샌드박스 `.git` read-only, 호스트 conda에 python-can/rclpy 없음(스텁 /tmp/t3can 또는 리뷰어 컨테이너 검증).

**Goal:** 스펙 r6 §3.1·§3.2의 A2a — 현장 복구·운용 명령의 단일 게이트인 ops broker 노드(TCP :9001, 역할 토큰, 4상태 ACK, 비상 2단계 서버 검증)와 DualSense 복구 chord 클라이언트를 구현한다.

**Architecture:** 순수 코어 3층 — ①`ops_contract.py`(와이어 계약·action 표) ②`ops_broker_core.py`(인증·검증·멱등 캐시·비상 2단계·전이표 — ROS/소켓 무관, 시계 주입) ③`ops_broker_node.py`(TCP 스레드 + rclpy `call_async` 프록시 + 5 Hz ops-state push). 상태 소스로 teleop에 `/teleop/gateway_state`, chassis에 `/chassis/safety_state` 발행을 추가. 클라이언트는 `ops_channel_client.py`(재전송·ACK 상관) + `RecoveryChordDetector`(순수)를 `remote_operation_client`에 통합. 배포는 D5대로 신규 `control.launch.py`(teleop+broker)를 기존 `powertrain_control` compose 서비스가 실행.

**Tech Stack:** Python 3.10, rclpy(Humble), TCP newline-JSON, pytest.

**Spec:** `docs/superpowers/specs/2026-07-17-abc-program-design.md` (r6) — §3.1, §3.2, §7.

## Global Constraints

- 기준선(A1 후): 호스트 240 / dev 991+2skip / ros 418 / 젯슨 418. 실패 0 유지.
- 호스트 pytest: `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest`.
- dev 컨테이너: `docker run --rm -v "$PWD:/workspace" -w /workspace -e PYTHONPATH=/workspace/ros2/src/powertrain_ros:/workspace/motor_control:/workspace powertrain-sw:dev python3 -m pytest motor_control motor_gui powertrain_observability powertrain_autonomy powertrain_sim remote_video tests operator_console -q`.
- ros 컨테이너: 핸드오프 §4 레시피 (colcon /tmp + `src/powertrain_ros/test`). 테스트는 DDS domain 77 + **에페메랄 포트**(:9000/:9001 라이브 충돌 금지 — §9-0 TCP판).
- 테스트&&커밋 체인. `docs/defence_docs/`·`docs/creativeEngineering/` 접근 금지. 커밋 말미 Co-Authored-By.
- **토큰 실값 금지**: 커밋되는 파일에 실제 토큰 문자열 금지 — 테스트는 tmp_path 생성 토큰만.
- 모터 실기 없음. chord 실감·비상 chord 햅틱 확인은 A배치 벤치 스모크로 이월.
- 키매핑·chord 시간은 `recovery-v1-initial-candidate` — **전부 임시**(코드 주석 명시).

---

### Task 1: `ops_contract.py` — 와이어 계약·action 표 (순수)

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/ops_contract.py`
- Test: `ros2/src/powertrain_ros/test/test_ops_contract.py`

**Interfaces:**
- Produces (이후 태스크 전부가 소비):

```python
SCHEMA_VERSION = 1
DEFAULT_PORT = 9001
MAX_RECORD_BYTES = 4 * 1024
ROLE_CONSOLE = "console"
ROLE_CONTROLLER = "controller"
STATUS_PENDING = "PENDING"
STATUS_FINAL_SUCCESS = "FINAL_SUCCESS"
STATUS_FINAL_REJECTED = "FINAL_REJECTED"
STATUS_OUTCOME_UNKNOWN = "OUTCOME_UNKNOWN"
RETRANSMIT_INTERVAL_S = 0.25
REQUEST_DEADLINE_S = 2.0
SERVICE_CALL_TIMEOUT_S = 1.0
EMERGENCY_HOLD_S = {"estop_reset": 5.0, "arm": 3.0}   # ⚠️ 임시(초기 후보)
OPS_STATE_STALE_S = 0.5
decode_request(line: str) -> dict            # 위반 시 ValueError(사유)
encode_response(**fields) -> bytes           # newline-JSON
ACTIONS: dict[str, ActionSpec]               # ActionSpec(roles, emergency_roles, kind, target)
```

- `ACTIONS` 표 (스펙 §3.1 화이트리스트 그대로; kind ∈ {"composite","service","publish"}):

| action | roles | emergency_roles | kind | target |
|---|---|---|---|---|
| clear_transient_hold | console,controller | — | composite | `/teleop_command/clear_hold` + `/chassis_node/authority_clear_hold` |
| authority_manual | console,controller | — | service | `/chassis_node/authority_manual` |
| authority_auto | console,controller | — | service | `/chassis_node/authority_auto` |
| authority_idle | console | — | service | `/chassis_node/authority_idle` |
| estop_reset | console | controller | service | `/chassis_node/reset_estop` |
| arm | console | controller | service | `/chassis_node/arm` |
| disarm | console | — | service | `/chassis_node/disarm` |
| mission_arrive_pickup / arrive_drop / skip / retry / regrasp_confirmed / clear_grip_lost | console | — | service | `/chassis_node/mission_*` |
| operator_hold / operator_resume | console | — | publish | `/section_events` (`{"type","stamp_s","payload"}`) |
| status_query | console,controller | — | local | pending/final 캐시 조회 |

(`arm_lock_override`·`calibration_*`·`extraction_grant`는 각각 A2b/A2c/C0에서 표에 추가 — 이 태스크는 자리만 주석으로 표시.)

- [ ] **Step 1: 실패하는 테스트 작성** — `test_ops_contract.py`:

```python
"""A2a ops 채널 와이어 계약(스펙 r6 §3.1) — 디코드 엄격성·action 표."""
import json

import pytest

from powertrain_ros import ops_contract as oc


def _request(**overrides):
    payload = {
        "schema_version": 1,
        "token": "tok",
        "request_id": "r-1",
        "sequence": 0,
        "action": "authority_manual",
        "params": {},
        "stamp_s": 1.0,
    }
    payload.update(overrides)
    return json.dumps(payload)


def test_decode_accepts_exact_fields_and_optional_extras():
    decoded = oc.decode_request(_request())
    assert decoded["action"] == "authority_manual"
    decoded = oc.decode_request(
        _request(expected_state_revision=3, phase="begin")
    )
    assert decoded["expected_state_revision"] == 3
    assert decoded["phase"] == "begin"


@pytest.mark.parametrize(
    "bad",
    [
        _request(schema_version=2),
        _request(action="rm_rf"),
        _request(sequence=-1),
        _request(request_id=""),
        "not json",
        json.dumps({"schema_version": 1}),
        _request(phase="maybe"),
    ],
)
def test_decode_rejects_contract_violations(bad):
    with pytest.raises(ValueError):
        oc.decode_request(bad)


def test_decode_rejects_oversized_record():
    with pytest.raises(ValueError):
        oc.decode_request(_request(params={"x": "y" * oc.MAX_RECORD_BYTES}))


def test_action_table_role_bindings_match_spec():
    assert oc.ACTIONS["estop_reset"].roles == frozenset({oc.ROLE_CONSOLE})
    assert oc.ACTIONS["estop_reset"].emergency_roles == frozenset(
        {oc.ROLE_CONTROLLER}
    )
    assert oc.ACTIONS["arm"].emergency_roles == frozenset({oc.ROLE_CONTROLLER})
    assert oc.ACTIONS["authority_manual"].roles == frozenset(
        {oc.ROLE_CONSOLE, oc.ROLE_CONTROLLER}
    )
    assert oc.ACTIONS["mission_skip"].roles == frozenset({oc.ROLE_CONSOLE})
    assert oc.ACTIONS["operator_hold"].kind == "publish"
    assert oc.ACTIONS["clear_transient_hold"].kind == "composite"
    assert oc.ACTIONS["status_query"].kind == "local"


def test_encode_response_is_newline_json():
    raw = oc.encode_response(
        request_id="r-1", status=oc.STATUS_PENDING, state_revision=4,
        detail="",
    )
    assert raw.endswith(b"\n")
    assert json.loads(raw)["status"] == "PENDING"
```

- [ ] **Step 2: 실패 확인** — ros 컨테이너(또는 호스트: rclpy 불필요):
`PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest ros2/src/powertrain_ros/test/test_ops_contract.py -q` → `ModuleNotFoundError: ops_contract`

- [ ] **Step 3: 구현** — `ops_contract.py`:

```python
"""A2a ops 채널 와이어 계약 — 스펙 r6 §3.1의 권위 구현.

요청 = newline-JSON {schema_version, token, request_id, sequence, action,
params, stamp_s [, expected_state_revision, phase]}. 응답 = {request_id,
status(PENDING/FINAL_SUCCESS/FINAL_REJECTED/OUTCOME_UNKNOWN), state_revision,
detail}. 역할 인가는 서버의 토큰→역할 매핑이 유일 근거다(client_type 없음).
"""
from dataclasses import dataclass, field
import json

SCHEMA_VERSION = 1
DEFAULT_PORT = 9001
MAX_RECORD_BYTES = 4 * 1024
ROLE_CONSOLE = "console"
ROLE_CONTROLLER = "controller"
STATUS_PENDING = "PENDING"
STATUS_FINAL_SUCCESS = "FINAL_SUCCESS"
STATUS_FINAL_REJECTED = "FINAL_REJECTED"
STATUS_OUTCOME_UNKNOWN = "OUTCOME_UNKNOWN"
RETRANSMIT_INTERVAL_S = 0.25
REQUEST_DEADLINE_S = 2.0
SERVICE_CALL_TIMEOUT_S = 1.0
# ⚠️ recovery-v1-initial-candidate — HIL·운전자 피드백 후 변경 전제(임시).
EMERGENCY_HOLD_S = {"estop_reset": 5.0, "arm": 3.0}
OPS_STATE_STALE_S = 0.5
_PHASES = {"begin", "execute"}
_REQUIRED = (
    "schema_version", "token", "request_id", "sequence", "action",
    "params", "stamp_s",
)
_OPTIONAL = ("expected_state_revision", "phase")


@dataclass(frozen=True)
class ActionSpec:
    roles: frozenset
    kind: str                      # composite | service | publish | local
    target: tuple = ()
    emergency_roles: frozenset = field(default_factory=frozenset)


_BOTH = frozenset({ROLE_CONSOLE, ROLE_CONTROLLER})
_CONSOLE = frozenset({ROLE_CONSOLE})
_CTRL_EMERGENCY = frozenset({ROLE_CONTROLLER})
_MISSIONS = (
    "mission_arrive_pickup", "mission_arrive_drop", "mission_skip",
    "mission_retry", "mission_regrasp_confirmed", "mission_clear_grip_lost",
)

ACTIONS = {
    "clear_transient_hold": ActionSpec(_BOTH, "composite", (
        "/teleop_command/clear_hold", "/chassis_node/authority_clear_hold",
    )),
    "authority_manual": ActionSpec(
        _BOTH, "service", ("/chassis_node/authority_manual",)
    ),
    "authority_auto": ActionSpec(
        _BOTH, "service", ("/chassis_node/authority_auto",)
    ),
    "authority_idle": ActionSpec(
        _CONSOLE, "service", ("/chassis_node/authority_idle",)
    ),
    "estop_reset": ActionSpec(
        _CONSOLE, "service", ("/chassis_node/reset_estop",),
        emergency_roles=_CTRL_EMERGENCY,
    ),
    "arm": ActionSpec(
        _CONSOLE, "service", ("/chassis_node/arm",),
        emergency_roles=_CTRL_EMERGENCY,
    ),
    "disarm": ActionSpec(_CONSOLE, "service", ("/chassis_node/disarm",)),
    "operator_hold": ActionSpec(_CONSOLE, "publish", ("/section_events",)),
    "operator_resume": ActionSpec(_CONSOLE, "publish", ("/section_events",)),
    "status_query": ActionSpec(_BOTH, "local"),
    # A2b: arm_lock_override / A2c: calibration_* / C0: extraction_grant
}
for _name in _MISSIONS:
    ACTIONS[_name] = ActionSpec(
        _CONSOLE, "service", ("/chassis_node/%s" % _name,)
    )


def decode_request(line):
    if len(line.encode("utf-8", errors="replace")) > MAX_RECORD_BYTES:
        raise ValueError("record exceeds %d bytes" % MAX_RECORD_BYTES)
    try:
        payload = json.loads(line)
    except (TypeError, ValueError) as exc:
        raise ValueError("invalid JSON: %s" % exc) from exc
    if not isinstance(payload, dict):
        raise ValueError("request must be a JSON object")
    unknown = set(payload) - set(_REQUIRED) - set(_OPTIONAL)
    if unknown:
        raise ValueError("unknown fields: %s" % sorted(unknown))
    missing = [key for key in _REQUIRED if key not in payload]
    if missing:
        raise ValueError("missing fields: %s" % missing)
    if payload["schema_version"] != SCHEMA_VERSION:
        raise ValueError(
            "unrecognized schema_version: %r" % payload["schema_version"]
        )
    if payload["action"] not in ACTIONS:
        raise ValueError("unknown action: %r" % payload["action"])
    if not isinstance(payload["request_id"], str) or not payload["request_id"]:
        raise ValueError("request_id must be a non-empty string")
    sequence = payload["sequence"]
    if not isinstance(sequence, int) or isinstance(sequence, bool) \
            or sequence < 0:
        raise ValueError("sequence must be a non-negative integer")
    if not isinstance(payload["token"], str) or not payload["token"]:
        raise ValueError("token must be a non-empty string")
    if not isinstance(payload["params"], dict):
        raise ValueError("params must be an object")
    float(payload["stamp_s"])
    if "phase" in payload and payload["phase"] not in _PHASES:
        raise ValueError("phase must be 'begin' or 'execute'")
    if "expected_state_revision" in payload:
        revision = payload["expected_state_revision"]
        if not isinstance(revision, int) or isinstance(revision, bool) \
                or revision < 0:
            raise ValueError("expected_state_revision must be >= 0 int")
    return payload


def encode_response(*, request_id, status, state_revision, detail=""):
    return (
        json.dumps(
            {
                "request_id": str(request_id),
                "status": str(status),
                "state_revision": int(state_revision),
                "detail": str(detail),
            },
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")
```

- [ ] **Step 4: 통과 확인** — 같은 명령 → 신규 전부 PASS
- [ ] **Step 5: 커밋** — `feat: ops-channel wire contract and action table (spec r6 §3.1)`

---

### Task 2: `ops_broker_core.py` — 인증·검증·멱등 캐시·직렬화 (순수)

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/ops_broker_core.py`
- Test: `ros2/src/powertrain_ros/test/test_ops_broker_core.py`

**Interfaces:**
- Consumes: Task 1 전부.
- Produces:

```python
@dataclass(frozen=True)
class OpsState:            # 노드가 조립·코어가 소비하는 스냅샷
    revision: int
    authority_mode: str        # "IDLE"/"AUTONOMY"/.../"UNKNOWN"
    gateway_state: str         # "DRIVE"/.../"UNKNOWN"
    gateway_input_fresh: bool
    gateway_neutral: bool
    estop_latched: bool
    active_estop_sources: tuple
    wheels_stopped: bool
    field_age_s: dict          # 필드 그룹별 age (authority/gateway/safety/wheels)

class OpsBrokerCore:
    def __init__(self, token_roles: dict, *, clock, state_provider,
                 cache_size=64, rate_limit_per_s=10): ...
    def handshake(self, client_key: str, line: str) -> tuple[str|None, bytes]
        # 반환 (role|None, 응답 bytes). 토큰 불일치 → (None, FINAL_REJECTED)
    def handle_line(self, client_key: str, role: str, line: str) -> Decision
    def complete(self, pending_key, success: bool, detail: str) -> bytes
        # 실행 완료 보고 → final 응답 bytes(캐시 갱신 포함)

@dataclass(frozen=True)
class Decision:
    response: bytes | None         # 즉시 회신(거부·캐시 히트·PENDING)
    execute: ExecutionOrder | None # 노드가 수행할 실행 지시

@dataclass(frozen=True)
class ExecutionOrder:
    pending_key: tuple             # (client_key, request_id)
    action: str
    kind: str
    targets: tuple
    params: dict
```

- 핸드셰이크 = 첫 줄 `{"schema_version":1,"hello":true,"token":...}` → 응답
  `{"request_id":"hello","status":"FINAL_SUCCESS","state_revision":N,
  "detail":"role=<role> broker_boot_id=<id>"}`.
- 의미론: 세션별 sequence 단조(역행 즉시 FINAL_REJECTED), 동일 request_id 재전송
  → 캐시 응답 재송(PENDING 중이면 PENDING), rate limit 초과 → FINAL_REJECTED,
  mutation은 **한 번에 하나**(이전 ExecutionOrder 미완이면 새 mutation은
  FINAL_REJECTED "busy"), `expected_state_revision` 불일치 → FINAL_REJECTED,
  `status_query` → params.request_id 캐시 조회(없으면 OUTCOME_UNKNOWN).

- [ ] **Step 1: 실패하는 테스트 작성** — `test_ops_broker_core.py` (전 시나리오, 시계 주입):

```python
"""A2a broker 코어 — 역할 인가·멱등·직렬화·revision (스펙 r6 §3.1)."""
import json

from powertrain_ros import ops_contract as oc
from powertrain_ros.ops_broker_core import OpsBrokerCore, OpsState


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def _state(revision=1, **overrides):
    base = dict(
        revision=revision, authority_mode="IDLE", gateway_state="DRIVE",
        gateway_input_fresh=True, gateway_neutral=True, estop_latched=False,
        active_estop_sources=(), wheels_stopped=True,
        field_age_s={"authority": 0.0, "gateway": 0.0, "safety": 0.0,
                     "wheels": 0.0},
    )
    base.update(overrides)
    return OpsState(**base)


def _core(clock=None, state=None):
    clock = clock or Clock()
    holder = {"state": state or _state()}
    core = OpsBrokerCore(
        {"tok-console": oc.ROLE_CONSOLE, "tok-ctrl": oc.ROLE_CONTROLLER},
        clock=clock,
        state_provider=lambda: holder["state"],
    )
    return core, clock, holder


def _hello(token):
    return json.dumps(
        {"schema_version": 1, "hello": True, "token": token}
    )


def _req(token, action, request_id="r-1", sequence=0, **extra):
    payload = {
        "schema_version": 1, "token": token, "request_id": request_id,
        "sequence": sequence, "action": action, "params": {},
        "stamp_s": 1.0,
    }
    payload.update(extra)
    return json.dumps(payload)


def test_handshake_maps_token_to_role_and_rejects_unknown():
    core, _, _ = _core()
    role, response = core.handshake("c1", _hello("tok-console"))
    assert role == oc.ROLE_CONSOLE
    assert json.loads(response)["status"] == "FINAL_SUCCESS"
    assert "role=console" in json.loads(response)["detail"]

    role, response = core.handshake("c2", _hello("wrong"))
    assert role is None
    assert json.loads(response)["status"] == "FINAL_REJECTED"


def test_console_only_action_is_rejected_for_controller_role():
    core, _, _ = _core()
    decision = core.handle_line(
        "c1", oc.ROLE_CONTROLLER, _req("tok-ctrl", "mission_skip")
    )
    assert decision.execute is None
    assert json.loads(decision.response)["status"] == "FINAL_REJECTED"


def test_mutation_flow_pending_then_final_and_idempotent_cache():
    core, _, _ = _core()
    decision = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "authority_manual")
    )
    assert json.loads(decision.response)["status"] == "PENDING"
    assert decision.execute.action == "authority_manual"

    # 재전송(같은 request_id) → PENDING 재송, 실행 지시는 중복 발행 금지
    retry = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "authority_manual", sequence=1),
    )
    assert retry.execute is None
    assert json.loads(retry.response)["status"] == "PENDING"

    final = core.complete(decision.execute.pending_key, True, "ok")
    assert json.loads(final)["status"] == "FINAL_SUCCESS"

    cached = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "authority_manual", sequence=2),
    )
    assert cached.execute is None
    assert json.loads(cached.response)["status"] == "FINAL_SUCCESS"


def test_second_mutation_while_pending_is_busy_rejected():
    core, _, _ = _core()
    first = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "authority_manual")
    )
    busy = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "disarm", request_id="r-2", sequence=1),
    )
    assert busy.execute is None
    body = json.loads(busy.response)
    assert body["status"] == "FINAL_REJECTED"
    assert "busy" in body["detail"]
    core.complete(first.execute.pending_key, True, "ok")


def test_sequence_regression_is_rejected():
    core, _, _ = _core()
    core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "status_query", sequence=5,
             params={"request_id": "x"}),
    )
    stale = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "status_query", request_id="r-9", sequence=4,
             params={"request_id": "x"}),
    )
    assert json.loads(stale.response)["status"] == "FINAL_REJECTED"


def test_expected_state_revision_mismatch_is_rejected():
    core, _, holder = _core()
    holder["state"] = _state(revision=7)
    decision = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "authority_manual", expected_state_revision=6),
    )
    assert json.loads(decision.response)["status"] == "FINAL_REJECTED"
    assert decision.execute is None


def test_status_query_returns_cached_or_unknown():
    core, _, _ = _core()
    decision = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "authority_manual")
    )
    core.complete(decision.execute.pending_key, False, "denied")

    hit = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "status_query", request_id="q-1", sequence=1,
             params={"request_id": "r-1"}),
    )
    assert json.loads(hit.response)["status"] == "FINAL_REJECTED"

    miss = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "status_query", request_id="q-2", sequence=2,
             params={"request_id": "never"}),
    )
    assert json.loads(miss.response)["status"] == "OUTCOME_UNKNOWN"


def test_rate_limit_rejects_flood():
    core, clock, _ = _core()
    rejected = 0
    for index in range(30):
        decision = core.handle_line(
            "c1", oc.ROLE_CONSOLE,
            _req("tok-console", "status_query",
                 request_id="f-%d" % index, sequence=index,
                 params={"request_id": "x"}),
        )
        if json.loads(decision.response)["status"] == "FINAL_REJECTED":
            rejected += 1
    assert rejected > 0
```

- [ ] **Step 2: 실패 확인** — 호스트 동일 명령 → `ModuleNotFoundError: ops_broker_core`
- [ ] **Step 3: 구현** — `ops_broker_core.py` (아래 그대로; 비상 2단계는 Task 3에서 확장):

```python
"""ops broker 순수 코어 — 인증·인가·멱등·단일 mutation 직렬화 (§3.1).

소켓·ROS 무관: 시계와 상태 공급자를 주입받고, 실행은 ExecutionOrder 로
노드에 위임한다. 모든 결정(수락·거부·완료)은 응답 bytes 로 표현된다.
"""
from dataclasses import dataclass, field
import collections
import uuid

from powertrain_ros import ops_contract as oc


@dataclass(frozen=True)
class OpsState:
    revision: int
    authority_mode: str
    gateway_state: str
    gateway_input_fresh: bool
    gateway_neutral: bool
    estop_latched: bool
    active_estop_sources: tuple
    wheels_stopped: bool
    field_age_s: dict


@dataclass(frozen=True)
class ExecutionOrder:
    pending_key: tuple
    action: str
    kind: str
    targets: tuple
    params: dict


@dataclass(frozen=True)
class Decision:
    response: bytes = None
    execute: ExecutionOrder = None


class OpsBrokerCore:
    def __init__(self, token_roles, *, clock, state_provider,
                 cache_size=64, rate_limit_per_s=10):
        self._token_roles = dict(token_roles)
        self._clock = clock
        self._state = state_provider
        self._cache_size = int(cache_size)
        self._rate_limit = int(rate_limit_per_s)
        self.boot_id = uuid.uuid4().hex
        self._sequences = {}                 # client_key -> last sequence
        self._cache = collections.OrderedDict()   # (client,rid) -> bytes|None
        self._pending_order = None           # ExecutionOrder | None
        self._rate_window = {}               # client_key -> (window_s, count)
        self._emergency = {}                 # (client,action) -> begin_s

    # -- 응답 헬퍼 ------------------------------------------------------
    def _revision(self):
        return int(self._state().revision)

    def _reject(self, request_id, detail):
        return oc.encode_response(
            request_id=request_id, status=oc.STATUS_FINAL_REJECTED,
            state_revision=self._revision(), detail=detail,
        )

    # -- 핸드셰이크 -----------------------------------------------------
    def handshake(self, client_key, line):
        try:
            import json

            payload = json.loads(line)
            token = payload.get("token")
            is_hello = bool(payload.get("hello"))
        except (TypeError, ValueError):
            return None, self._reject("hello", "invalid handshake")
        if not is_hello:
            return None, self._reject("hello", "handshake required first")
        role = self._token_roles.get(token)
        if role is None:
            return None, self._reject("hello", "unauthorized token")
        self._sequences.setdefault(client_key, -1)
        return role, oc.encode_response(
            request_id="hello", status=oc.STATUS_FINAL_SUCCESS,
            state_revision=self._revision(),
            detail="role=%s broker_boot_id=%s" % (role, self.boot_id),
        )

    # -- 본 처리 --------------------------------------------------------
    def _rate_ok(self, client_key):
        now_s = float(self._clock())
        window, count = self._rate_window.get(client_key, (now_s, 0))
        if now_s - window >= 1.0:
            window, count = now_s, 0
        count += 1
        self._rate_window[client_key] = (window, count)
        return count <= self._rate_limit

    def _cache_put(self, key, response):
        self._cache[key] = response
        while len(self._cache) > self._cache_size:
            self._cache.popitem(last=False)

    def handle_line(self, client_key, role, line):
        try:
            request = oc.decode_request(line)
        except ValueError as exc:
            return Decision(response=self._reject("invalid", str(exc)))
        request_id = request["request_id"]
        key = (client_key, request_id)

        if not self._rate_ok(client_key):
            return Decision(
                response=self._reject(request_id, "rate limit exceeded")
            )
        if self._token_roles.get(request["token"]) != role:
            return Decision(
                response=self._reject(request_id, "token/role mismatch")
            )

        last = self._sequences.get(client_key, -1)
        if key in self._cache:                    # 멱등 재전송
            cached = self._cache[key]
            if cached is None:                    # 아직 PENDING
                return Decision(response=oc.encode_response(
                    request_id=request_id, status=oc.STATUS_PENDING,
                    state_revision=self._revision(), detail="in flight",
                ))
            return Decision(response=cached)
        if request["sequence"] <= last:
            return Decision(
                response=self._reject(request_id, "sequence regression")
            )
        self._sequences[client_key] = request["sequence"]

        spec = oc.ACTIONS[request["action"]]
        emergency = self._authorize(role, request, spec)
        if isinstance(emergency, bytes):
            return Decision(response=emergency)

        if request["action"] == "status_query":
            return Decision(response=self._status_query(request))

        if "expected_state_revision" in request and (
            request["expected_state_revision"] != self._revision()
        ):
            return Decision(response=self._reject(
                request_id, "state revision mismatch"
            ))

        gate = self._precondition_gate(role, request, spec,
                                       emergency=emergency)
        if gate is not None:
            return Decision(response=gate)

        if self._pending_order is not None:
            return Decision(
                response=self._reject(request_id, "busy: mutation in flight")
            )
        order = ExecutionOrder(
            pending_key=key, action=request["action"], kind=spec.kind,
            targets=tuple(spec.target), params=dict(request["params"]),
        )
        self._pending_order = order
        self._cache_put(key, None)
        return Decision(
            response=oc.encode_response(
                request_id=request_id, status=oc.STATUS_PENDING,
                state_revision=self._revision(), detail="accepted",
            ),
            execute=order,
        )

    def _authorize(self, role, request, spec):
        """일반 인가. 반환: False(일반)/True(비상 경로)/bytes(거부)."""
        if role in spec.roles:
            return False
        if role in spec.emergency_roles:
            return True
        return self._reject(request["request_id"], "role not authorized")

    def _precondition_gate(self, role, request, spec, *, emergency):
        """Task 3에서 비상 2단계·전이표로 확장. 기본 통과."""
        return None

    def _status_query(self, request):
        target = request["params"].get("request_id")
        for (client, rid), cached in reversed(self._cache.items()):
            if rid == target:
                if cached is None:
                    return oc.encode_response(
                        request_id=request["request_id"],
                        status=oc.STATUS_PENDING,
                        state_revision=self._revision(), detail="in flight",
                    )
                import json

                body = json.loads(cached)
                return oc.encode_response(
                    request_id=request["request_id"], status=body["status"],
                    state_revision=self._revision(), detail=body["detail"],
                )
        return oc.encode_response(
            request_id=request["request_id"],
            status=oc.STATUS_OUTCOME_UNKNOWN,
            state_revision=self._revision(), detail="no record",
        )

    def complete(self, pending_key, success, detail):
        status = (
            oc.STATUS_FINAL_SUCCESS if success else oc.STATUS_FINAL_REJECTED
        )
        response = oc.encode_response(
            request_id=pending_key[1], status=status,
            state_revision=self._revision(), detail=detail,
        )
        self._cache_put(pending_key, response)
        if self._pending_order is not None \
                and self._pending_order.pending_key == pending_key:
            self._pending_order = None
        return response
```

- [ ] **Step 4: 통과 확인** — 호스트 신규 전부 PASS
- [ ] **Step 5: 커밋** — `feat: ops broker pure core - auth, idempotent cache, single-mutation serialization`

---

### Task 3: 코어 확장 — 비상 2단계 서버 검증 + authority 전이표 + clear 의미론

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/ops_broker_core.py` (`_precondition_gate`, `_emergency`)
- Test: `ros2/src/powertrain_ros/test/test_ops_broker_emergency.py`

**Interfaces:**
- Consumes: Task 2 코어.
- Produces: `_precondition_gate` 완성 —
  ①**비상 경로**(emergency=True): `phase` 필수. `begin` → `(client,action)`에
  시각 기록, FINAL_SUCCESS("begin recorded; hold"). `execute` → 경과 ≥
  `EMERGENCY_HOLD_S[action]` 아니면 FINAL_REJECTED("hold not satisfied").
  arm execute는 추가로 `gateway_neutral ∧ gateway_input_fresh ∧
  wheels_stopped` 아니면 거부(스펙 §3.1). begin 없이 execute → 거부.
  ②**authority 전이표**(§3.1 표): manual → authority_mode ∈ {IDLE, AUTONOMY}
  ∧ gateway_state == "DRIVE" ∧ input_fresh ∧ ¬estop_latched; auto →
  mode ∈ {IDLE, TELEOP} ∧ ¬estop_latched; idle → mode != MOTION_HOLD;
  MOTION_HOLD 중 manual/auto → 거부("clear first"). gateway ARM/STOPPING_* 중
  → 거부. ③field_age_s의 해당 그룹 > `OPS_STATE_STALE_S` → 거부("stale state").
  ④`clear_transient_hold`: composite 실행은 노드 몫이나, **부분 성공 보고
  계약**을 위해 params에 하위 대상 이름을 채워 ExecutionOrder로 전달.

- [ ] **Step 1: 실패하는 테스트 작성** — `test_ops_broker_emergency.py`:

```python
"""비상 2단계 서버 검증·전이표 (스펙 r6 §3.1, 레드팀 26/Codex 6 해소)."""
import json

from powertrain_ros import ops_contract as oc
from powertrain_ros.ops_broker_core import OpsBrokerCore, OpsState

from test_ops_broker_core import Clock, _state, _req


def _core(state):
    clock = Clock()
    holder = {"state": state}
    core = OpsBrokerCore(
        {"tok-ctrl": oc.ROLE_CONTROLLER, "tok-console": oc.ROLE_CONSOLE},
        clock=clock, state_provider=lambda: holder["state"],
    )
    return core, clock, holder


def _status(decision):
    return json.loads(decision.response)["status"]


def test_emergency_execute_without_begin_is_rejected():
    core, _, _ = _core(_state())
    decision = core.handle_line(
        "c1", oc.ROLE_CONTROLLER,
        _req("tok-ctrl", "estop_reset", phase="execute"),
    )
    assert _status(decision) == "FINAL_REJECTED"


def test_emergency_execute_before_hold_elapsed_is_rejected():
    core, clock, _ = _core(_state())
    core.handle_line(
        "c1", oc.ROLE_CONTROLLER,
        _req("tok-ctrl", "estop_reset", phase="begin"),
    )
    clock.now += 4.0            # 5.0 미만
    early = core.handle_line(
        "c1", oc.ROLE_CONTROLLER,
        _req("tok-ctrl", "estop_reset", request_id="r-2", sequence=1,
             phase="execute"),
    )
    assert _status(early) == "FINAL_REJECTED"
    assert early.execute is None


def test_emergency_execute_after_hold_produces_execution_order():
    core, clock, _ = _core(_state())
    core.handle_line(
        "c1", oc.ROLE_CONTROLLER,
        _req("tok-ctrl", "estop_reset", phase="begin"),
    )
    clock.now += 5.1
    ready = core.handle_line(
        "c1", oc.ROLE_CONTROLLER,
        _req("tok-ctrl", "estop_reset", request_id="r-2", sequence=1,
             phase="execute"),
    )
    assert _status(ready) == "PENDING"
    assert ready.execute.targets == ("/chassis_node/reset_estop",)


def test_emergency_arm_requires_neutral_fresh_and_stopped():
    core, clock, holder = _core(
        _state(gateway_neutral=False, wheels_stopped=False)
    )
    core.handle_line(
        "c1", oc.ROLE_CONTROLLER, _req("tok-ctrl", "arm", phase="begin")
    )
    clock.now += 3.1
    blocked = core.handle_line(
        "c1", oc.ROLE_CONTROLLER,
        _req("tok-ctrl", "arm", request_id="r-2", sequence=1,
             phase="execute"),
    )
    assert _status(blocked) == "FINAL_REJECTED"

    holder["state"] = _state()
    core.handle_line(
        "c1", oc.ROLE_CONTROLLER,
        _req("tok-ctrl", "arm", request_id="r-3", sequence=2, phase="begin"),
    )
    clock.now += 3.1
    ready = core.handle_line(
        "c1", oc.ROLE_CONTROLLER,
        _req("tok-ctrl", "arm", request_id="r-4", sequence=3,
             phase="execute"),
    )
    assert _status(ready) == "PENDING"


def test_console_estop_reset_needs_no_phase():
    core, _, _ = _core(_state())
    decision = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "estop_reset")
    )
    assert _status(decision) == "PENDING"


def test_authority_manual_transition_table():
    core, _, _ = _core(_state(authority_mode="MOTION_HOLD"))
    held = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "authority_manual")
    )
    assert _status(held) == "FINAL_REJECTED"
    assert "clear" in json.loads(held.response)["detail"]

    core2, _, _ = _core(_state(gateway_state="STOPPING_FOR_ARM"))
    stopping = core2.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "authority_manual")
    )
    assert _status(stopping) == "FINAL_REJECTED"

    core3, _, _ = _core(_state(estop_latched=True))
    latched = core3.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "authority_manual")
    )
    assert _status(latched) == "FINAL_REJECTED"


def test_stale_state_group_blocks_gated_action():
    core, _, _ = _core(
        _state(field_age_s={"authority": 0.0, "gateway": 2.0,
                            "safety": 0.0, "wheels": 0.0})
    )
    stale = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "authority_manual")
    )
    assert _status(stale) == "FINAL_REJECTED"
    assert "stale" in json.loads(stale.response)["detail"]
```

- [ ] **Step 2: 실패 확인** — 호스트 → 다수 FAIL (`_precondition_gate` 기본 통과라 전이표 미작동)
- [ ] **Step 3: 구현** — `_precondition_gate` 교체 + `_AUTHORITY_RULES`:

```python
    _STOPPING_STATES = ("STOPPING_FOR_ARM", "STOPPING_FOR_DRIVE")

    def _precondition_gate(self, role, request, spec, *, emergency):
        request_id = request["request_id"]
        state = self._state()
        action = request["action"]

        if emergency:
            phase = request.get("phase")
            if phase is None:
                return self._reject(
                    request_id, "emergency action requires phase"
                )
            hold_s = oc.EMERGENCY_HOLD_S[action]
            key = ("emergency", role, action)
            if phase == "begin":
                self._emergency[key] = float(self._clock())
                return oc.encode_response(
                    request_id=request_id, status=oc.STATUS_FINAL_SUCCESS,
                    state_revision=self._revision(),
                    detail="begin recorded; hold %.1fs" % hold_s,
                )
            begin_s = self._emergency.get(key)
            if begin_s is None:
                return self._reject(request_id, "execute without begin")
            if float(self._clock()) - begin_s < hold_s:
                return self._reject(request_id, "hold not satisfied")
            del self._emergency[key]
            if action == "arm" and not (
                state.gateway_neutral and state.gateway_input_fresh
                and state.wheels_stopped
            ):
                return self._reject(
                    request_id,
                    "arm requires released neutral input and stopped wheels",
                )
            return None
        if request.get("phase") is not None:
            return self._reject(request_id, "phase is emergency-only")

        if action.startswith("authority_"):
            if state.field_age_s.get("authority", 9.9) > oc.OPS_STATE_STALE_S \
                    or state.field_age_s.get("gateway", 9.9) \
                    > oc.OPS_STATE_STALE_S:
                return self._reject(request_id, "stale state; retry")
            if state.authority_mode == "MOTION_HOLD" and action in (
                "authority_manual", "authority_auto",
            ):
                return self._reject(
                    request_id, "MOTION_HOLD: clear_transient_hold first"
                )
            if state.gateway_state in ("ARM",) + self._STOPPING_STATES:
                return self._reject(
                    request_id, "gateway busy: %s" % state.gateway_state
                )
            if state.estop_latched and action in (
                "authority_manual", "authority_auto",
            ):
                return self._reject(request_id, "E-stop latched")
            if action == "authority_manual" and not (
                state.gateway_state == "DRIVE" and state.gateway_input_fresh
                and state.authority_mode in ("IDLE", "AUTONOMY")
            ):
                return self._reject(
                    request_id, "manual preconditions not met"
                )
            if action == "authority_auto" and state.authority_mode not in (
                "IDLE", "TELEOP",
            ):
                return self._reject(request_id, "auto preconditions not met")
        return None
```

- [ ] **Step 4: 통과 확인** — 호스트: Task 2·3 테스트 전부 PASS
- [ ] **Step 5: 커밋** — `feat: server-verified emergency two-phase and authority transition table in broker core`

---

### Task 4: 상태 소스 — `/teleop/gateway_state` + `/chassis/safety_state`

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/remote_input_gateway.py` (`frame_is_neutral` 모듈 함수로 추출)
- Modify: `ros2/src/powertrain_ros/powertrain_ros/teleop_command_node.py` (`_tick`에서 발행)
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py` (`_tick`에서 5 Hz 발행)
- Test: `ros2/src/powertrain_ros/test/test_ops_state_sources.py`

**Interfaces:**
- Produces: `/teleop/gateway_state` = String JSON `{"state","input_fresh","neutral","stamp_s"}` 매 틱(30 Hz). `/chassis/safety_state` = String JSON `{"mode","estop_latched","active_estop_sources","stamp_s"}` ≤5 Hz. `frame_is_neutral(frame) -> bool` (기존 gateway `_neutral` 로직을 모듈 함수로 추출, gateway는 이를 호출).

- [ ] **Step 1: 실패하는 테스트 작성**:

```python
"""broker ops-state 소스 토픽 계약 (스펙 r6 §3.1)."""
import ast
import json
import re
from pathlib import Path

from powertrain_ros.remote_input_gateway import frame_is_neutral
from test_remote_input_gateway import _frame

PACKAGE = Path(__file__).resolve().parents[1]
TELEOP = (PACKAGE / "powertrain_ros/teleop_command_node.py").read_text(
    encoding="utf-8"
)
CHASSIS = (PACKAGE / "powertrain_ros/chassis_node.py").read_text(
    encoding="utf-8"
)


def test_frame_is_neutral_matches_gateway_semantics():
    assert frame_is_neutral(_frame())
    assert not frame_is_neutral(_frame(deadman=True))
    assert not frame_is_neutral(_frame(right_trigger=0.2))
    assert not frame_is_neutral(_frame(dpad_x=1))
    assert not frame_is_neutral(_frame(estop_edge=True))


def test_teleop_publishes_gateway_state_each_tick():
    assert '"/teleop/gateway_state"' in TELEOP
    assert '"neutral"' in TELEOP and '"input_fresh"' in TELEOP


def test_chassis_publishes_safety_state():
    assert '"/chassis/safety_state"' in CHASSIS
    assert '"estop_latched"' in CHASSIS
    assert '"active_estop_sources"' in CHASSIS
```

- [ ] **Step 2: 실패 확인** — `ImportError: frame_is_neutral`
- [ ] **Step 3: 구현**
  - `remote_input_gateway.py`: `_neutral(self, frame)` 본문을 모듈 함수
    `frame_is_neutral(frame)`로 추출하고 기존 메서드는 위임(동작 불변).
  - `teleop_command_node.py`: `pub_gateway_state = create_publisher(String,
    "/teleop/gateway_state", 10)`; `_drain_events` frame 분기에서
    `self._last_frame = payload` 저장; `_tick` 말미에

    ```python
        state_message = String()
        state_message.data = json.dumps(
            {
                "state": output.state,
                "input_fresh": bool(output.input_fresh),
                "neutral": bool(
                    self._last_frame is not None
                    and frame_is_neutral(self._last_frame)
                ),
                "stamp_s": time.monotonic(),
            },
            separators=(",", ":"),
        )
        self.pub_gateway_state.publish(state_message)
    ```

    (`self._last_frame = None` 초기화, disconnect 이벤트에서 None으로 리셋.)
  - `chassis_node.py`: `pub_safety_state` 생성(… `"/chassis/safety_state"`, 10),
    `_tick`에서 0.2 s 주기로

    ```python
        safety = self.cm.safety_snapshot()
        message = String()
        message.data = json.dumps(
            {
                "mode": self.cm.mode,
                "estop_latched": bool(safety.estop_latched),
                "active_estop_sources": list(safety.active_estop_sources),
                "stamp_s": time.monotonic(),
            },
            separators=(",", ":"),
        )
        self.pub_safety_state.publish(message)
    ```

    ⚠️ `cm.safety_snapshot()`이 없으면 `self.cm._interlock.snapshot()` 대신
    **ChassisManager에 공개 `safety_snapshot()` 위임 메서드를 추가**한다
    (private 접근 금지 규율).
- [ ] **Step 4: 통과 확인** — ros 컨테이너 전체(기존 418 유지 + 신규)
- [ ] **Step 5: 커밋** — `feat: gateway/safety state topics for the ops broker`

---

### Task 5: `ops_broker_node.py` — TCP 서버 + rclpy 프록시 + 5 Hz push

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/ops_broker_node.py`
- Modify: `ros2/src/powertrain_ros/setup.py` (entry point `ops_broker`)
- Test: `ros2/src/powertrain_ros/test/test_ops_broker_node.py`

**Interfaces:**
- Consumes: Task 1~4 전부.
- Produces: 노드 `ops_broker` — 파라미터 `port`(기본 `ops_contract.DEFAULT_PORT`),
  `token_dir`(기본 `/etc/powertrain`; `ops_console.token`·`ops_controller.token`
  각 1줄). TCP listen(4), 접속당 스레드(teleop 노드 패턴: settimeout 0.2·
  KEEPALIVE·NODELAY·idle 10 s), 첫 줄 핸드셰이크 → 이후 요청. 서비스 실행 =
  `call_async` + `SERVICE_CALL_TIMEOUT_S` future 대기(**타임아웃 시
  core.complete 하지 않고 PENDING 유지**, late completion 도착 시 final push —
  Codex 31 의미론). composite = 순차 두 서비스, 부분 성공은
  `detail="teleop=ok chassis=timeout"` 식 분리 보고 후 성공=전체 성공.
  publish kind = `/section_events`에 `{"type": action.upper(), "stamp_s",
  "payload": params}` 발행 후 즉시 complete(True). **ops-state push**: 5 Hz
  타이머(서비스 콜과 분리된 MutuallyExclusiveCallbackGroup)로 전 연결에
  `{"push":"ops_state", "revision", "authority_mode", "gateway_state",
  "gateway_input_fresh", "gateway_neutral", "estop_latched",
  "active_estop_sources", "wheels_stopped", "field_age_s", "stamp_s"}` —
  revision은 **의미 필드 튜플이 변할 때만 증가**. journal: 모든 최종 결과를
  EventClient로 `event_type="OPS_COMMAND"`(§3.1) — 토큰은 절대 미포함.
- 구독: `/command_authority/state`(String `"MODE|..."` → mode = split("|")[0]),
  `/chassis/safety_state`, `/teleop/gateway_state`, `/wheel_states`
  (`all(|drive_turns_per_s|<0.1)` → wheels_stopped, 그룹 stamp 갱신).

- [ ] **Step 1: 실패하는 테스트 작성** — `test_ops_broker_node.py`
  (에페메랄 포트·tmp 토큰·가짜 Trigger 서버 — 핵심 6개):

```python
"""ops broker 노드 E2E — 인증·프록시·push·부분 성공 (스펙 r6 §3.1)."""
import json
import socket
import time

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from powertrain_ros import ops_broker_node as broker_module
from powertrain_ros.ops_broker_node import OpsBrokerNode


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def token_dir(tmp_path):
    (tmp_path / "ops_console.token").write_text("tok-console-test\n")
    (tmp_path / "ops_controller.token").write_text("tok-ctrl-test\n")
    return tmp_path


class FakeServices(Node):
    """chassis/teleop 서비스 대역 — 호출 기록 + 지연 주입."""

    def __init__(self):
        super().__init__("fake_targets")
        self.calls = []
        self.fail = set()
        for name in (
            "/chassis_node/authority_manual", "/chassis_node/reset_estop",
            "/chassis_node/arm", "/teleop_command/clear_hold",
            "/chassis_node/authority_clear_hold",
        ):
            self.create_service(
                Trigger, name,
                lambda req, resp, n=name: self._serve(n, resp),
            )

    def _serve(self, name, response):
        self.calls.append(name)
        response.success = name not in self.fail
        response.message = "fake"
        return response


def _free_port():
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def _node(token_dir, port):
    return OpsBrokerNode(parameter_overrides=None, port_override=port,
                         token_dir_override=str(token_dir))


def _client(port, token):
    sock = socket.create_connection(("127.0.0.1", port), timeout=2.0)
    sock.settimeout(0.2)
    sock.sendall((json.dumps(
        {"schema_version": 1, "hello": True, "token": token}
    ) + "\n").encode())
    return sock


def _read_lines(sock, nodes, want=1, timeout=3.0):
    lines, buffer = [], b""
    deadline = time.monotonic() + timeout
    while len(lines) < want and time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.02)
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        buffer += chunk
        while b"\n" in buffer:
            line, _, buffer = buffer.partition(b"\n")
            lines.append(json.loads(line))
    return lines


def _request(token, action, request_id="r-1", sequence=0, **extra):
    payload = {"schema_version": 1, "token": token, "request_id": request_id,
               "sequence": sequence, "action": action, "params": {},
               "stamp_s": time.monotonic()}
    payload.update(extra)
    return (json.dumps(payload) + "\n").encode()


def test_handshake_and_role_binding(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    try:
        sock = _client(port, "tok-console-test")
        hello = _read_lines(sock, [node])[0]
        assert hello["status"] == "FINAL_SUCCESS"
        assert "role=console" in hello["detail"]
        sock.close()

        bad = _client(port, "nope")
        rejected = _read_lines(bad, [node])[0]
        assert rejected["status"] == "FINAL_REJECTED"
        bad.close()
    finally:
        node.close()
        node.destroy_node()


def test_authority_manual_round_trip_calls_target_service(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    targets = FakeServices()
    try:
        sock = _client(port, "tok-console-test")
        _read_lines(sock, [node, targets])
        sock.sendall(_request("tok-console-test", "authority_manual"))
        replies = _read_lines(sock, [node, targets], want=2)
        statuses = [item["status"] for item in replies
                    if item.get("request_id") == "r-1"]
        assert "PENDING" in statuses
        assert "FINAL_SUCCESS" in statuses
        assert "/chassis_node/authority_manual" in targets.calls
        sock.close()
    finally:
        node.close()
        node.destroy_node()
        targets.destroy_node()


def test_composite_clear_reports_partial_results(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    targets = FakeServices()
    targets.fail.add("/chassis_node/authority_clear_hold")
    try:
        sock = _client(port, "tok-console-test")
        _read_lines(sock, [node, targets])
        sock.sendall(_request("tok-console-test", "clear_transient_hold"))
        replies = _read_lines(sock, [node, targets], want=2)
        final = [item for item in replies
                 if item["status"] == "FINAL_REJECTED"]
        assert final and "authority_clear_hold" in final[0]["detail"]
        sock.close()
    finally:
        node.close()
        node.destroy_node()
        targets.destroy_node()


def test_ops_state_push_arrives_with_revision(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    publisher = Node("state_feeder")
    authority_pub = publisher.create_publisher(
        String, "/command_authority/state", 10
    )
    try:
        sock = _client(port, "tok-ctrl-test")
        _read_lines(sock, [node])
        authority_pub.publish(String(data="IDLE|ok"))
        pushes = [item for item in
                  _read_lines(sock, [node, publisher], want=4)
                  if item.get("push") == "ops_state"]
        assert pushes
        assert pushes[0]["authority_mode"] in ("IDLE", "UNKNOWN")
        assert isinstance(pushes[0]["revision"], int)
        sock.close()
    finally:
        node.close()
        node.destroy_node()
        publisher.destroy_node()


def test_controller_direct_estop_reset_is_rejected(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    try:
        sock = _client(port, "tok-ctrl-test")
        _read_lines(sock, [node])
        sock.sendall(_request("tok-ctrl-test", "estop_reset"))
        reply = [item for item in _read_lines(sock, [node], want=1)
                 if item.get("request_id") == "r-1"][0]
        assert reply["status"] == "FINAL_REJECTED"
        sock.close()
    finally:
        node.close()
        node.destroy_node()
```

- [ ] **Step 2: 실패 확인** — ros 컨테이너 → `ModuleNotFoundError: ops_broker_node`
- [ ] **Step 3: 구현** — `ops_broker_node.py`. 골격(테스트 계약에 맞춤):

```python
"""ops broker 노드 — TCP :9001 소유, 복구·운용 명령의 단일 게이트 (§3.1).

패턴은 teleop_command_node의 TCP 스레드를 따르되, 실행은 rclpy call_async
+ future 타임아웃으로 한다. ops-state push 타이머는 서비스 콜과 분리된
callback group — 콜 지연이 push를 절대 막지 못한다(레드팀 2b).
"""
import json
import os
import socket
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from powertrain_msgs.msg import WheelStates
from powertrain_ros import ops_contract as oc
from powertrain_ros.ops_broker_core import Decision, OpsBrokerCore, OpsState

CLIENT_IDLE_TIMEOUT_S = 10.0
WHEEL_STOP_TURNS = 0.1
_SEMANTIC_FIELDS = (
    "authority_mode", "gateway_state", "gateway_input_fresh",
    "gateway_neutral", "estop_latched", "active_estop_sources",
    "wheels_stopped",
)


def load_token_roles(token_dir):
    roles = {}
    for filename, role in (
        ("ops_console.token", oc.ROLE_CONSOLE),
        ("ops_controller.token", oc.ROLE_CONTROLLER),
    ):
        path = os.path.join(token_dir, filename)
        try:
            with open(path, "r", encoding="utf-8") as handle:
                token = handle.readline().strip()
        except OSError:
            continue
        if token:
            roles[token] = role
    return roles


class OpsBrokerNode(Node):
    def __init__(self, *, parameter_overrides=None, port_override=None,
                 token_dir_override=None):
        super().__init__("ops_broker",
                         parameter_overrides=parameter_overrides or [])
        self.declare_parameter("port", oc.DEFAULT_PORT)
        self.declare_parameter("token_dir", "/etc/powertrain")
        self._port = int(port_override
                         if port_override is not None
                         else self.get_parameter("port").value)
        token_dir = str(token_dir_override
                        if token_dir_override is not None
                        else self.get_parameter("token_dir").value)
        roles = load_token_roles(token_dir)
        if not roles:
            self.get_logger().error(
                "no ops tokens under %s — all clients will be rejected"
                % token_dir
            )
        self._state_lock = threading.Lock()
        self._fields = {name: None for name in _SEMANTIC_FIELDS}
        self._stamps = {"authority": None, "gateway": None,
                        "safety": None, "wheels": None}
        self._revision = 0
        self._last_semantic = None
        self._core = OpsBrokerCore(
            roles, clock=time.monotonic, state_provider=self._ops_state,
        )
        self._connections = []          # (socket, lock)
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._push_group = MutuallyExclusiveCallbackGroup()
        self._clients = {}
        self._section_pub = self.create_publisher(String, "/section_events", 10)
        self.create_subscription(
            String, "/command_authority/state", self._on_authority, 10)
        self.create_subscription(
            String, "/chassis/safety_state", self._on_safety, 10)
        self.create_subscription(
            String, "/teleop/gateway_state", self._on_gateway, 10)
        self.create_subscription(
            WheelStates, "/wheel_states", self._on_wheels, 10)
        self.create_timer(0.2, self._push_ops_state,
                          callback_group=self._push_group)
        self._stop_event = threading.Event()
        self._server_socket = None
        self._closed = False
        self._server_thread = threading.Thread(
            target=self._serve, name="ops-broker-tcp", daemon=True)
        self._server_thread.start()
        self.get_logger().info("ops broker TCP :%d" % self._port)
```

  이어서(전체를 파일에 완성): `_on_authority`(mode=data.split("|")[0], stamp),
  `_on_safety`/`_on_gateway`(JSON 파싱, 실패 무시+로그), `_on_wheels`
  (`wheels_stopped` 계산), `_ops_state()`(락 하에 필드→OpsState, revision은
  `_SEMANTIC_FIELDS` 튜플 변화 시에만 +1, `field_age_s`는 stamp 기반·None→9.9),
  `_serve`/`_serve_client`(teleop 패턴: 첫 줄 → `core.handshake`, 이후
  `core.handle_line` → `decision.response` 송신, `decision.execute` 있으면
  `self._execute(order, connection)`), `_execute`(kind별: service →
  `create_client(Trigger, target, callback_group=self._service_group)` +
  `call_async`; future를 `_pending[order.pending_key]=(future,...)`에 넣고
  0.05 s 타이머 폴링으로 완료/타임아웃 처리 — **타임아웃 시 PENDING 유지·
  journal만**, 완료 도착 시 `core.complete` 응답을 해당 연결로 push; composite
  → 순차 두 콜, 부분 결과 detail 병합; publish → `/section_events` 발행 후
  즉시 complete), `_push_ops_state`(전 연결에 push JSON — 죽은 소켓 정리),
  `_journal(event_type="OPS_COMMAND", payload)` (§ 이벤트 스키마, 토큰 필드
  없음), `close()`.
  `setup.py` entry_points에 `"ops_broker = powertrain_ros.ops_broker_node:main"`.
- [ ] **Step 4: 통과 확인** — ros 컨테이너 전체 스위트(418+신규, 실패 0)
- [ ] **Step 5: 커밋** — `feat: ops broker node - TCP gate, rclpy proxies, 5Hz ops-state push`

---

### Task 6: 배포 편입 — `control.launch.py` + compose command + healthcheck

**Files:**
- Create: `ros2/src/powertrain_ros/launch/control.launch.py`
- Modify: `docker/docker-compose.jetson.yml` (`powertrain_control` command·healthcheck)
- Test: `ros2/src/powertrain_ros/test/test_control_launch_contract.py`

**Interfaces:**
- Produces: `control.launch.py` = teleop_command + ops_broker 두 노드(개별
  프로세스), 인자 `ops_port`(기본 9001)·`ops_token_dir`(기본 /etc/powertrain).
  compose `powertrain_control`: command를 `ros2 launch powertrain_ros
  control.launch.py`로 교체, healthcheck에 :9000 **및** :9001 TCP-connect 추가
  (기존 bash 래퍼 cmdline-grep 금지 원칙 유지).

- [ ] **Step 1: 실패하는 테스트 작성** — `test_control_launch_contract.py`:

```python
"""D5 배포 계약 — control.launch가 teleop+broker를 함께 감독한다."""
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch/control.launch.py"
COMPOSE = PACKAGE.parents[2] / "docker/docker-compose.jetson.yml"


def test_control_launch_runs_teleop_and_broker_as_separate_nodes():
    source = LAUNCH.read_text(encoding="utf-8")
    assert 'executable="teleop_command"' in source
    assert 'executable="ops_broker"' in source
    assert '"token_dir"' in source


def test_compose_control_service_uses_launch_and_checks_both_ports():
    source = COMPOSE.read_text(encoding="utf-8")
    assert "control.launch.py" in source
    assert "9001" in source
```

- [ ] **Step 2: 실패 확인** — launch 파일 부재로 FAIL
- [ ] **Step 3: 구현** — `control.launch.py`:

```python
"""powertrain_control 서비스의 PID 1 — teleop_command + ops_broker (D5)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ops_port", default_value="9001"),
        DeclareLaunchArgument("ops_token_dir",
                              default_value="/etc/powertrain"),
        Node(package="powertrain_ros", executable="teleop_command",
             name="teleop_command", output="screen"),
        Node(package="powertrain_ros", executable="ops_broker",
             name="ops_broker", output="screen",
             parameters=[{
                 "port": LaunchConfiguration("ops_port"),
                 "token_dir": LaunchConfiguration("ops_token_dir"),
             }]),
    ])
```

  compose `powertrain_control` (기존 command의 `ros2 run powertrain_ros
  teleop_command` 부분만 `ros2 launch powertrain_ros control.launch.py`로,
  healthcheck 스크립트에 9001 connect 추가 — 기존 9000 체크 형식 재사용).
  ⚠️ compose는 젯슨 배포 파일 — 문법만 바꾸고 젯슨 검증은 배치 말 동기에서.
- [ ] **Step 4: 통과 확인** — ros 컨테이너 전체 + `python3 -c "import yaml,io;yaml.safe_load(open('docker/docker-compose.jetson.yml'))"` (dev 컨테이너)
- [ ] **Step 5: 커밋** — `feat: control launch supervises teleop+ops broker; compose healthcheck covers :9001`

---

### Task 7: 클라이언트 — `ops_channel_client.py` + `RecoveryChordDetector` + 통합

**Files:**
- Create: `motor_control/laptop/ops_channel_client.py`
- Modify: `motor_control/laptop/remote_operation_client.py` (chord·ops 통합)
- Test: `motor_control/laptop/tests/test_ops_channel_client.py`

**Interfaces:**
- Consumes: Task 1 와이어 계약(**미러 상수** — powertrain_ros import 금지,
  기존 SCHEMA_VERSION 미러 패턴).
- Produces:

```python
# ops_channel_client.py (pygame-free)
OPS_SCHEMA_VERSION = 1
OPS_DEFAULT_PORT = 9001
RETRANSMIT_INTERVAL_S = 0.25
REQUEST_DEADLINE_S = 2.0
EMERGENCY_HOLD_S = {"estop_reset": 5.0, "arm": 3.0}   # ⚠️ 임시
CHORD_CLEAR_HOLD_NS = 2_000_000_000        # □+CREATE
CHORD_AUTHORITY_HOLD_NS = 1_000_000_000    # D-pad ↓/↑
class RecoveryChordDetector:
    def update(self, sample, *, now_ns) -> list[dict]
        # ClientInput 유사 객체(buttons 콜백 주입) → 액션 이벤트
        # [{"action": "clear_transient_hold"}], [{"action": "authority_manual"}],
        # 비상: [{"action":"estop_reset","phase":"begin"}] → hold 충족 시
        #       [{"action":"estop_reset","phase":"execute"}] (클라 UX용 —
        #       서버가 재검증)
class OpsChannelClient:
    def __init__(self, host, port, token, *, clock=time.monotonic,
                 connector=..., sleep_fn=...)
    def submit(self, action, *, params=None, phase=None) -> str  # request_id
    def pump(self) -> list[dict]      # 수신 응답·push 처리 + 재전송(250ms/2s)
    def latest_ops_state(self) -> dict | None
    def close(self)
```

- chord 조합(전부 `recovery-v1-initial-candidate`, ⚠️ 임시): □(3)+CREATE(8)
  2 s = clear_transient_hold; D-pad y=-1 1 s = authority_manual, y=+1 1 s =
  authority_auto; L1(4)+R1(5)+□(3) 5 s = estop_reset begin→execute;
  L1+R1+△(2? — **△ 버튼 인덱스는 매핑에 없다: `triangle_button: 2`를
  v2 매핑에 추가**) 3 s = arm. 부분 해제 = 즉시 리셋. detector는 버튼 상태
  콜백(`button(name)->bool`)과 dpad 값을 받는 순수 클래스 — pygame 무관.
- `remote_operation_client.main()` 통합: `--ops-port`(기본 9001)·
  `--ops-token-file`(기본 `~/.config/powertrain/ops_controller.token`) 인자,
  토큰 파일 없으면 ops 채널 비활성(경고 1회) — 주행 경로는 영향 0. 루프에서
  `detector.update(...)` → `ops.submit(...)`, `ops.pump()` 응답을 stderr 표시.

- [ ] **Step 1: 실패하는 테스트 작성** — `test_ops_channel_client.py`
  (FakeSocket·FakeClock — 기존 laptop 테스트 패턴):

  핵심 케이스 7개: ①□+CREATE 2 s 미만 유지 → 액션 없음, 2 s 도달 →
  clear 1회(래치, 계속 눌러도 재발행 없음) ②부분 해제 → 타이머 리셋
  ③D-pad ↓ 1 s → authority_manual ④L1+R1+□ 5 s → begin은 즉시(눌린 순간),
  execute는 5 s 유지 후 ⑤submit → 첫 send 즉시, ACK 없으면 250 ms마다 재전송,
  2 s에 중단 ⑥동일 request_id 유지(재전송 멱등) ⑦push(`"push":"ops_state"`)
  수신 → latest_ops_state 갱신. (코드는 기존 `test_remote_operation_client.py`
  의 FakeSocket/입력 시퀀스 패턴을 그대로 따라 작성 — 각 케이스 assert는
  액션 리스트·소켓 송신 횟수·request_id 동일성.)

- [ ] **Step 2: 실패 확인** — 호스트 laptop 테스트 → ModuleNotFoundError
- [ ] **Step 3: 구현** — 위 인터페이스 그대로. detector 상태기계는 기존
  `DualSenseInputAdapter`의 chord 로직(:146-162)과 동일 구조(조합별 시작 ns·
  래치). 비상 begin은 조합 시작 시 1회 submit, execute는 hold 충족 시 1회.
  v2 매핑에 `"triangle_button": 2` 추가(주석 "measured triangle; emergency
  arm chord용, ⚠️ 임시").
- [ ] **Step 4: 통과 확인** — 호스트: `pytest motor_control/laptop/tests -q`
  (기존 + 신규 전부), dev 컨테이너 laptop 포함 회귀
- [ ] **Step 5: 커밋** — `feat: laptop ops channel client and recovery chords (initial candidates)`

---

### Task 8: 미러 계약 테스트 + 문서 + 3환경 + 배치 마감

**Files:**
- Test: `ros2/src/powertrain_ros/test/test_ops_mirror_contract.py`
- Modify: `.claude/CLAUDE.md`(원격운용 절에 ops 채널 한 줄), `docs/reports/2026-07-16-project-state-and-handoff.md`(§2 체인·기준선)

**Interfaces:** 없음(검증·문서).

- [ ] **Step 1: 미러 계약 테스트**:

```python
"""클라이언트 미러 상수 == 서버 계약 (기존 SCHEMA_VERSION 미러 패턴)."""
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parents[2] / "motor_control"))

from laptop import ops_channel_client as client  # noqa: E402
from powertrain_ros import ops_contract as server  # noqa: E402


def test_client_mirror_matches_server_contract():
    assert client.OPS_SCHEMA_VERSION == server.SCHEMA_VERSION
    assert client.OPS_DEFAULT_PORT == server.DEFAULT_PORT
    assert client.RETRANSMIT_INTERVAL_S == server.RETRANSMIT_INTERVAL_S
    assert client.REQUEST_DEADLINE_S == server.REQUEST_DEADLINE_S
    assert client.EMERGENCY_HOLD_S == server.EMERGENCY_HOLD_S
```

  (laptop 패키지 import 형태는 실제 구조에 맞춰 조정 — `laptop`이 패키지가
  아니면 `importlib` 파일 로드로.)
- [ ] **Step 2: 문서** — CLAUDE.md 원격운용 부분에 "ops 채널 :9001(복구·운용
  명령, 역할 토큰, chord = recovery-v1-initial-candidate ⚠️ 임시)" 요약 추가;
  핸드오프 §2에 A2a 행(+커밋 해시)·기준선 갱신.
- [ ] **Step 3: 3환경 전체 회귀** — 호스트 240+α / dev 991+α+2skip / ros 418+α,
  실패 0. 젯슨 동기+parity는 배치 마감 절차(리뷰어).
- [ ] **Step 4: 커밋** — `docs: A2a ops channel chain + mirror contract test`

---

## A2a 완료 기준 (스펙 §7 대조)

- 무토큰/오토큰/역할 위조 거부·sequence 역행·멱등 재전송·busy 직렬화·
  revision 불일치·rate limit — 코어 테스트로 전부 커버.
- 비상 2단계: begin 없는 execute 거부·hold 미충족 거부·arm의
  neutral/fresh/stopped 게이트 — 서버 시간 기준.
- 노드 E2E: 핸드셰이크·서비스 왕복·composite 부분 성공 분리 보고·push
  revision·controller 직접 reset 거부.
- 배포: control.launch 계약 테스트 + compose 문법 검증 (젯슨 실증은 배치 마감).
- **벤치 이월**: chord 실감·비상 chord 5 s/3 s 체감·라이브 :9001 스모크 —
  A배치 벤치 세션 목록에 추가.
