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


def test_ops_state_estop_cause_defaults_are_empty_strings():
    state = _state()

    assert state.estop_source == ""
    assert state.estop_detail == ""


def _core(clock=None, state=None, **core_kwargs):
    clock = clock or Clock()
    holder = {"state": state or _state()}
    core = OpsBrokerCore(
        {"tok-console": oc.ROLE_CONSOLE, "tok-ctrl": oc.ROLE_CONTROLLER},
        clock=clock,
        state_provider=lambda: holder["state"],
        **core_kwargs,
    )
    core.handshake("c1", _hello("tok-console", stamp_s=100.0))
    core.handshake("c2", _hello("tok-ctrl", stamp_s=100.0))
    return core, clock, holder


def _hello(token, **extra):
    client_id = extra.pop("client_id", "pytest-%s" % token)
    payload = {"schema_version": 1, "hello": True, "token": token}
    if client_id is not None:
        payload["client_id"] = client_id
    payload.update(extra)
    return json.dumps(payload)


def _req(token, action, request_id="r-1", sequence=0, **extra):
    payload = {
        "schema_version": 1, "token": token, "request_id": request_id,
        "sequence": sequence, "action": action, "params": {},
        "stamp_s": 100.0,
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


def test_mutation_requires_clock_sync_handshake():
    core, _, _ = _core()
    role, _ = core.handshake("legacy", _hello("tok-console"))

    decision = core.handle_line(
        "legacy", role,
        _req("tok-console", "operator_hold", stamp_s=100.0),
    )

    assert decision.execute is None
    assert "clock sync" in json.loads(decision.response)["detail"]


def test_expired_request_is_rejected_before_sequence_or_cache_change():
    core, _, _ = _core()
    role, _ = core.handshake(
        "delayed", _hello("tok-console", stamp_s=10.0)
    )

    stale = core.handle_line(
        "delayed", role,
        _req(
            "tok-console", "operator_hold", request_id="stale",
            sequence=50, stamp_s=7.99,
        ),
    )

    assert stale.execute is None
    assert "deadline" in json.loads(stale.response)["detail"]
    fresh = core.handle_line(
        "delayed", role,
        _req(
            "tok-console", "operator_hold", request_id="fresh",
            sequence=0, stamp_s=10.0,
        ),
    )
    assert fresh.execute is not None


def test_too_far_future_request_is_rejected_before_sequence_change():
    core, _, _ = _core()
    role, _ = core.handshake(
        "future", _hello("tok-console", stamp_s=10.0)
    )

    future = core.handle_line(
        "future", role,
        _req(
            "tok-console", "operator_hold", request_id="future",
            sequence=50, stamp_s=10.251,
        ),
    )

    assert future.execute is None
    assert "future" in json.loads(future.response)["detail"]
    fresh = core.handle_line(
        "future", role,
        _req(
            "tok-console", "operator_hold", request_id="fresh",
            sequence=0, stamp_s=10.0,
        ),
    )
    assert fresh.execute is not None


def test_completed_mutation_is_idempotent_across_reconnect():
    core, clock, _ = _core()
    role, _ = core.handshake(
        "transport-1", _hello("tok-console", stamp_s=10.0)
    )
    first = core.handle_line(
        "transport-1", role,
        _req("tok-console", "operator_hold", stamp_s=10.0),
    )
    core.complete(first.execute.pending_key, True, "published")

    clock.now += 0.1
    role, _ = core.handshake(
        "transport-2", _hello("tok-console", stamp_s=20.0)
    )
    retry = core.handle_line(
        "transport-2", role,
        _req(
            "tok-console", "operator_hold", sequence=1, stamp_s=20.0,
        ),
    )

    assert retry.execute is None
    assert json.loads(retry.response)["status"] == "FINAL_SUCCESS"


def test_request_fingerprint_is_enforced_across_reconnect():
    core, clock, _ = _core()
    role, _ = core.handshake(
        "transport-1", _hello("tok-console", stamp_s=10.0)
    )
    first = core.handle_line(
        "transport-1", role,
        _req(
            "tok-console", "operator_hold", stamp_s=10.0,
            expected_state_revision=1,
        ),
    )
    core.complete(first.execute.pending_key, True, "published")

    clock.now += 0.1
    role, _ = core.handshake(
        "transport-2", _hello("tok-console", stamp_s=20.0)
    )
    conflict = core.handle_line(
        "transport-2", role,
        _req(
            "tok-console", "operator_hold", sequence=1, stamp_s=20.0,
            expected_state_revision=2,
        ),
    )

    assert conflict.execute is None
    assert "request_id reused" in json.loads(conflict.response)["detail"]


def test_status_query_is_scoped_to_authenticated_token_identity():
    clock = Clock()
    core = OpsBrokerCore(
        {
            "tok-console-a": oc.ROLE_CONSOLE,
            "tok-console-b": oc.ROLE_CONSOLE,
        },
        clock=clock,
        state_provider=lambda: _state(),
    )
    role_a, _ = core.handshake(
        "transport-a", _hello("tok-console-a", stamp_s=10.0)
    )
    first = core.handle_line(
        "transport-a", role_a,
        _req(
            "tok-console-a", "operator_hold", request_id="shared-id",
            stamp_s=10.0,
        ),
    )
    core.complete(first.execute.pending_key, False, "A denied")

    role_b, _ = core.handshake(
        "transport-b", _hello("tok-console-b", stamp_s=20.0)
    )
    query = core.handle_line(
        "transport-b", role_b,
        _req(
            "tok-console-b", "status_query", request_id="query-b",
            stamp_s=20.0, params={"request_id": "shared-id"},
        ),
    )

    body = json.loads(query.response)
    assert body["status"] == "OUTCOME_UNKNOWN"
    assert body["detail"] == "no record"


def test_status_query_is_scoped_to_client_id_when_token_is_shared():
    clock = Clock()
    core = OpsBrokerCore(
        {"shared-console-token": oc.ROLE_CONSOLE},
        clock=clock,
        state_provider=lambda: _state(),
    )
    role_a, _ = core.handshake(
        "transport-a",
        _hello(
            "shared-console-token", stamp_s=10.0, client_id="console-a"
        ),
    )
    first = core.handle_line(
        "transport-a", role_a,
        _req(
            "shared-console-token", "operator_hold",
            request_id="shared-id", stamp_s=10.0,
        ),
    )
    core.complete(first.execute.pending_key, False, "A denied")

    role_b, _ = core.handshake(
        "transport-b",
        _hello(
            "shared-console-token", stamp_s=20.0, client_id="console-b"
        ),
    )
    query = core.handle_line(
        "transport-b", role_b,
        _req(
            "shared-console-token", "status_query",
            request_id="query-b", stamp_s=20.0,
            params={"request_id": "shared-id"},
        ),
    )

    body = json.loads(query.response)
    assert body["status"] == "OUTCOME_UNKNOWN"
    assert body["detail"] == "no record"


def test_handshake_requires_stable_client_id():
    core, _, _ = _core()

    role, response = core.handshake(
        "missing-id",
        _hello("tok-console", stamp_s=100.0, client_id=None),
    )

    assert role is None
    assert "client_id" in json.loads(response)["detail"]


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


def test_outcome_unknown_is_cached_and_releases_mutation_slot():
    core, _, _ = _core()
    first = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "disarm")
    )

    final = core.complete(
        first.execute.pending_key,
        False,
        "no response from /chassis_node/disarm",
        status=oc.STATUS_OUTCOME_UNKNOWN,
    )

    assert json.loads(final)["status"] == "OUTCOME_UNKNOWN"
    cached = core.handle_line(
        "c1",
        oc.ROLE_CONSOLE,
        _req("tok-console", "disarm", sequence=1),
    )
    assert cached.execute is None
    assert json.loads(cached.response)["status"] == "OUTCOME_UNKNOWN"

    second = core.handle_line(
        "c1",
        oc.ROLE_CONSOLE,
        _req("tok-console", "operator_hold", request_id="r-2", sequence=1),
    )
    assert second.execute is not None
    assert "busy" not in json.loads(second.response)["detail"]


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


def test_estop_preempts_pending_mutation_and_produces_execution_order():
    core, _, _ = _core()
    arm = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "arm")
    )
    assert arm.execute.action == "arm"

    estop = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "estop", request_id="r-estop", sequence=1),
    )

    assert json.loads(estop.response)["status"] == "PENDING"
    assert estop.execute.action == "estop"
    assert estop.execute.targets == ("/chassis_node/estop",)


def test_estop_bypasses_rate_limit_after_authenticated_flood():
    core, _, _ = _core(rate_limit_per_s=1)
    core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "status_query", params={"request_id": "x"}),
    )

    estop = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "estop", request_id="r-estop", sequence=1),
    )

    assert json.loads(estop.response)["status"] == "PENDING"
    assert estop.execute.action == "estop"


def test_estop_same_request_id_as_completed_hold_is_rejected_as_reuse():
    core, _, _ = _core()
    hold = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "operator_hold")
    )
    core.complete(hold.execute.pending_key, True, "operator hold published")

    estop = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "estop", sequence=1),
    )

    assert json.loads(estop.response)["status"] == "FINAL_REJECTED"
    assert "request_id reused" in json.loads(estop.response)["detail"]
    assert estop.execute is None


def test_estop_completion_keeps_older_mutation_busy_until_it_completes():
    core, _, _ = _core()
    hold = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "operator_hold")
    )
    estop = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "estop", request_id="r-estop", sequence=1),
    )
    assert estop.execute.pending_key != hold.execute.pending_key

    core.complete(estop.execute.pending_key, True, "estop triggered")
    blocked = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "operator_resume", request_id="r-2", sequence=1),
    )

    assert blocked.execute is None
    assert "busy" in json.loads(blocked.response)["detail"]
    core.complete(hold.execute.pending_key, True, "operator hold published")

    accepted = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "operator_resume", request_id="r-3", sequence=2),
    )
    assert accepted.execute is not None
    assert accepted.execute.action == "operator_resume"


def test_retransmitted_pending_estop_is_idempotent_and_executes_once():
    core, _, _ = _core()
    first = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "estop")
    )
    second = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "estop", sequence=1)
    )

    assert json.loads(first.response)["status"] == "PENDING"
    assert first.execute.action == "estop"
    assert json.loads(second.response)["status"] == "PENDING"
    assert second.execute is None
    core.complete(first.execute.pending_key, True, "estop triggered")


def test_pending_fingerprint_survives_cache_eviction_until_completion():
    core, _, _ = _core(cache_size=1)
    hold = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "operator_hold")
    )
    core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "estop", request_id="r-estop", sequence=1),
    )
    core.complete(hold.execute.pending_key, True, "operator hold published")

    retry = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "operator_hold", sequence=1),
    )

    assert retry.execute is None
    body = json.loads(retry.response)
    assert body["status"] == "FINAL_SUCCESS"
    assert body["detail"] == "operator hold published"


def test_same_request_id_with_different_non_estop_action_is_rejected():
    core, _, _ = _core()
    hold = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-console", "operator_hold")
    )
    core.complete(hold.execute.pending_key, True, "operator hold published")

    conflict = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "operator_resume", sequence=1),
    )

    body = json.loads(conflict.response)
    assert body["status"] == "FINAL_REJECTED"
    assert "request_id reused" in body["detail"]
    assert conflict.execute is None


def test_same_request_id_with_different_params_is_rejected():
    core, _, _ = _core()
    hold = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req("tok-console", "operator_hold", params={"reason": "first"}),
    )
    core.complete(hold.execute.pending_key, True, "operator hold published")

    conflict = core.handle_line(
        "c1", oc.ROLE_CONSOLE,
        _req(
            "tok-console", "operator_hold", sequence=1,
            params={"reason": "different"},
        ),
    )

    body = json.loads(conflict.response)
    assert body["status"] == "FINAL_REJECTED"
    assert "request_id reused" in body["detail"]
    assert conflict.execute is None


def test_estop_priority_path_keeps_token_and_role_authorization():
    core, _, _ = _core()

    mismatch = core.handle_line(
        "c1", oc.ROLE_CONSOLE, _req("tok-ctrl", "estop")
    )
    forbidden = core.handle_line(
        "c2", oc.ROLE_CONTROLLER, _req("tok-ctrl", "estop")
    )

    assert json.loads(mismatch.response)["detail"] == "token/role mismatch"
    assert mismatch.execute is None
    assert json.loads(forbidden.response)["detail"] == "role not authorized"
    assert forbidden.execute is None


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
