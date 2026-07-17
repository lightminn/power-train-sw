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
