"""Arm bench: GUI range calibration routing and the bridge restart action."""
import json
from types import SimpleNamespace

import pytest

from powertrain_ros.ops_broker_node import OpsBrokerNode
from powertrain_ros.ops_contract import ACTIONS, ROLE_CONSOLE


def dispatch(kind, params, **broker_fields):
    published, outcomes = [], []
    sink = SimpleNamespace(publish=lambda m: published.append(json.loads(m.data)))
    broker = SimpleNamespace(
        _arm_calibration_pub=sink, _tool_dual_calibration_pub=sink,
        _tool_single_calibration_pub=sink,
        _arm_bridge_restart_command='', _arm_bridge_restart_proc=None,
        _complete_order=lambda *args: outcomes.append(args[-2:]))
    broker.__dict__.update(broker_fields)
    order = SimpleNamespace(kind=kind, params=params, action='')
    OpsBrokerNode._execute(broker, order, None, ROLE_CONSOLE)
    return broker, published, outcomes


@pytest.mark.parametrize('action', ['measure', 'verify', 'apply_temporary', 'save'])
def test_arm_range_steps_go_to_the_arm_calibration_topic(action):
    _, published, outcomes = dispatch(
        'publish_tool_calibration', {'scope': 'arm', 'step': 'range', 'action': action})
    assert published == [{'step': 'range', 'action': action}]
    assert outcomes[0][0] is True


@pytest.mark.parametrize('params', [
    {'scope': 'arm', 'step': 'zero', 'action': 'measure'},
    {'scope': 'arm', 'step': 'gear_ratio', 'action': 'save'},
    {'scope': 'arm', 'step': 'range', 'action': 'start'},
])
def test_other_arm_steps_are_refused(params):
    _, published, outcomes = dispatch('publish_tool_calibration', params)
    assert not published and outcomes[0][0] is False


def test_bridge_restart_is_refused_until_configured():
    _, _, outcomes = dispatch('run_arm_bridge_restart', {})
    assert outcomes[0] == (False, 'bridge restart not configured on this broker')


def test_bridge_restart_runs_the_configured_command_once(tmp_path):
    marker = tmp_path / 'ran'
    broker, _, outcomes = dispatch(
        'run_arm_bridge_restart', {},
        _arm_bridge_restart_command=f'sh -c "sleep 1; touch {marker}"')
    assert outcomes[0][0] is True
    proc = broker._arm_bridge_restart_proc
    again = []
    broker._complete_order = lambda *args: again.append(args[-2:])
    OpsBrokerNode._execute(broker, SimpleNamespace(kind='run_arm_bridge_restart',
                                                   params={}, action=''), None, ROLE_CONSOLE)
    assert again[0] == (False, 'bridge restart already running')
    proc.wait(timeout=10)
    assert marker.exists()


def test_bridge_restart_takes_no_params():
    _, _, outcomes = dispatch('run_arm_bridge_restart', {'cmd': 'rm -rf /'},
                              _arm_bridge_restart_command='true')
    assert outcomes[0][0] is False


def test_bridge_restart_is_console_only():
    spec = ACTIONS['robot_arm_bridge_restart']
    assert spec.roles == frozenset({ROLE_CONSOLE})
    assert spec.kind == 'run_arm_bridge_restart'
