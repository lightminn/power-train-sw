"""Demo auto-resume arm (docs/specs/2026-11-06-demo-mode.md D2).

The broker only forwards a compare-and-arm request when the robot itself is
in demo mode, stopped, unlatched and still at the disarm counter the console
saw. The chassis repeats the comparison; this gate keeps a stale request
from even reaching it.
"""
import json
from types import SimpleNamespace

import pytest

from powertrain_ros import ops_contract as oc
from powertrain_ros.ops_broker_node import OpsBrokerNode
from test_ops_broker_core import _core, _req, _state


def _demo_state(**overrides):
    base = dict(demo_mode=True, disarm_seq=4, disarm_cause="demo_fault",
                chassis_mode="IDLE", authority_mode="TELEOP")
    base.update(overrides)
    return _state(**base)


def _submit(state, params=None):
    core, _, _ = _core(state=state)
    decision = core.handle_line("c1", oc.ROLE_CONSOLE, _req(
        "tok-console", "demo_auto_arm",
        params={"expect_disarm_seq": 4} if params is None else params))
    return decision, json.loads(decision.response)


def test_matching_demo_state_produces_an_execution_order():
    decision, response = _submit(_demo_state())
    assert response["status"] == "PENDING"
    assert decision.execute.kind == "publish_demo_auto_arm"
    assert decision.execute.params == {"expect_disarm_seq": 4}


@pytest.mark.parametrize("overrides,params", [
    (dict(demo_mode=False), None),
    (dict(estop_latched=True), None),
    (dict(chassis_mode="ARMED"), None),
    (dict(wheels_stopped=False), None),
    (dict(gateway_neutral=False), None),
    (dict(gateway_input_fresh=False), None),
    (dict(disarm_seq=5), None),
    (dict(field_age_s={"authority": 0.0, "gateway": 0.0, "safety": 9.0,
                       "wheels": 0.0}), None),
    (dict(field_age_s={"authority": 9.0, "gateway": 0.0, "safety": 0.0,
                       "wheels": 0.0}), None),
    (dict(authority_mode="MOTION_HOLD"), None),
    (dict(authority_mode="IDLE"), None),
    (dict(gateway_state="MOTION_HOLD"), None),
    (dict(gateway_state="STALE"), None),
    ({}, {"expect_disarm_seq": True}),
    ({}, {"expect_disarm_seq": 4, "extra": 1}),
    ({}, {}),
])
def test_demo_auto_arm_is_rejected_unless_every_condition_holds(overrides, params):
    decision, response = _submit(_demo_state(**overrides), params)
    assert decision.execute is None
    assert response["status"] == "FINAL_REJECTED"


def _dispatch(params):
    published, outcomes = [], []
    broker = SimpleNamespace(
        _demo_auto_arm_pub=SimpleNamespace(publish=lambda m: published.append(json.loads(m.data))),
        _complete_order=lambda *args: outcomes.append(args[-2:]))
    order = SimpleNamespace(kind="publish_demo_auto_arm", params=params, deadline_s=None)
    OpsBrokerNode._execute(broker, order, None, oc.ROLE_CONSOLE)
    return published, outcomes


def test_node_publishes_only_the_expected_counter():
    published, outcomes = _dispatch({"expect_disarm_seq": 4})
    assert published == [{"expect_disarm_seq": 4}]
    assert outcomes[0][0] is True


@pytest.mark.parametrize("params", [
    {}, {"expect_disarm_seq": -1}, {"expect_disarm_seq": "4"},
    {"expect_disarm_seq": 4, "force": True},
])
def test_node_rejects_malformed_params(params):
    published, outcomes = _dispatch(params)
    assert not published and outcomes[0][0] is False
