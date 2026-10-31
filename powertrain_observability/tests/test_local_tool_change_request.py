"""Broker translation of bench slot teach/exchange actions into bridge commands."""

import pytest

from powertrain_observability.tool_snapshot import local_tool_change_request


@pytest.fixture(autouse=True)
def bench(monkeypatch):
    monkeypatch.setenv('ARM_LOCAL_ALL_AXES', '1')


def test_actions_map_to_bridge_commands():
    assert local_tool_change_request(
        'robot_arm_slot_capture', {'slot': '2', 'tool_type': 'dual_motor_gripper',
                                   'pose': 'above'}) == {
        'slot': '2', 'tool_type': 'dual_motor_gripper', 'pose': 'above', 'command': 'CAPTURE'}
    assert local_tool_change_request(
        'robot_arm_exchange_start', {'tool_type': 'cleaner'}) == {
        'tool_type': 'cleaner', 'command': 'START'}
    assert local_tool_change_request('robot_arm_exchange_stop', {}) == {'command': 'STOP'}


@pytest.mark.parametrize('action,params', [
    ('robot_arm_exchange_start', {'tool_type': 'hammer'}),
    ('robot_arm_exchange_start', {}),
    ('robot_arm_slot_capture', {'slot': '9', 'tool_type': 'cleaner', 'pose': 'seat'}),
    ('robot_arm_slot_capture', {'slot': '1', 'tool_type': 'cleaner'}),
    ('robot_arm_slot_capture', {'slot': '1', 'tool_type': 'cleaner', 'pose': 'side'}),
    ('robot_arm_exchange_stop', {'command': 'START'}),
])
def test_malformed_requests_are_refused(action, params):
    with pytest.raises(ValueError):
        local_tool_change_request(action, params)


def test_outside_the_bench_nothing_is_translated(monkeypatch):
    monkeypatch.delenv('ARM_LOCAL_ALL_AXES')
    with pytest.raises(ValueError, match='bench-only'):
        local_tool_change_request('robot_arm_exchange_stop', {})


def _status(**exchange):
    return {'tool_type': 'spur_1motor_gripper', 'actuator_ids': [5],
            'local_tool_change': exchange}


def test_exchange_status_is_projected_for_the_console():
    from powertrain_observability.tool_snapshot import normalize_tool
    out = normalize_tool(_status(
        state='DESCEND', slot='1', step=2, steps=6, error='x' * 500,
        slots={'1': 'spur_1motor_gripper'}, incomplete=['1'], attached_tool=None,
        extra='dropped'))
    exchange = out['local_tool_change']
    assert exchange == {'state': 'DESCEND', 'slot': '1', 'step': 2, 'steps': 6,
                        'error': 'x' * 160, 'slots': {'1': 'spur_1motor_gripper'},
                        'incomplete': ['1'], 'attached_tool': None}


@pytest.mark.parametrize('bad', [
    dict(state='DESCEND', step='2'), dict(state=7), dict(slots={'1': 'hammer'}),
    dict(attached_tool='hammer'), dict(slots={}, incomplete=['1'])])
def test_malformed_exchange_status_is_rejected(bad):
    from powertrain_observability.tool_snapshot import normalize_tool
    with pytest.raises(ValueError):
        normalize_tool(_status(**bad))


def test_dry_run_is_an_optional_boolean_on_start_only():
    assert local_tool_change_request(
        'robot_arm_exchange_start', {'tool_type': 'cleaner', 'dry_run': True}) == {
        'tool_type': 'cleaner', 'dry_run': True, 'command': 'START'}
    for action, params in (
            ('robot_arm_exchange_start', {'tool_type': 'cleaner', 'dry_run': 'yes'}),
            ('robot_arm_slot_capture', {'slot': '1', 'tool_type': 'cleaner', 'pose': 'seat',
                                        'dry_run': True})):
        with pytest.raises(ValueError):
            local_tool_change_request(action, params)


def test_bench_limit_and_trip_switches_ride_along():
    assert local_tool_change_request(
        'robot_arm_slot_capture', {'slot': '1', 'tool_type': 'cleaner', 'pose': 'seat',
                                   'limits': False}) == {
        'slot': '1', 'tool_type': 'cleaner', 'pose': 'seat', 'limits': False, 'command': 'CAPTURE'}
    assert local_tool_change_request(
        'robot_arm_exchange_start',
        {'tool_type': 'cleaner', 'limits': False, 'trip': False})['trip'] is False
    with pytest.raises(ValueError):
        local_tool_change_request('robot_arm_slot_capture',
                                  {'slot': '1', 'tool_type': 'cleaner', 'pose': 'seat',
                                   'trip': False})
