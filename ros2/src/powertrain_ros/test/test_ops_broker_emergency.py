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
