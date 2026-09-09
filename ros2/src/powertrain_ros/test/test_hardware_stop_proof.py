"""The ops gate must require fresh proof from the actual chassis owner."""
import json
import time

import pytest

from test_ops_state_sources import _ON_SAFETY, _OPS_STATE, _broker_harness, _safety_message


def message(**changes):
    data = json.loads(_safety_message(include_mask=False).data)
    proof = dict(source='chassis_can', valid=True, stopped=True,
                 max_feedback_age_ms=100., node_ids=[11, 12, 13, 14, 15, 16])
    proof.update(changes)
    data['hardware_stop_proof'] = proof
    return type('Message', (), {'data': json.dumps(data)})()


@pytest.mark.parametrize('source', ['chassis_can', 'chassis_usb'])
def test_only_hardware_owner_proof_sets_wheels_stopped(source):
    node = _broker_harness()
    msg = message(source=source)
    _ON_SAFETY(node, msg)
    state = _OPS_STATE(node)
    assert state.wheels_stopped is True
    assert .1 <= state.field_age_s['wheels'] < .15


@pytest.mark.parametrize('change', [
    {'source': 'fake'}, {'source': 'wheel_states'}, {'valid': False}, {'valid': 'true'},
    {'stopped': 'true'}, {'stopped': False}, {'node_ids': []}, {'node_ids': [11,12,13,14,15]},
    {'node_ids': [11,12,13,14,15,15]}, {'node_ids': [1,2,3,4,5,6]},
    {'max_feedback_age_ms': None}, {'max_feedback_age_ms': -1},
    {'max_feedback_age_ms': 201}, {'max_feedback_age_ms': True},
    {'max_feedback_age_ms': float('nan')}, {'max_feedback_age_ms': float('inf')},
])
def test_invalid_or_moving_proof_revokes_previous_stopped_state(change):
    node = _broker_harness()
    _ON_SAFETY(node, message())
    assert _OPS_STATE(node).wheels_stopped
    _ON_SAFETY(node, message(**change))
    assert _OPS_STATE(node).wheels_stopped is False


@pytest.mark.parametrize('change', [
    {'hardware_stop_proof': None}, {'hardware_stop_proof': {}},
    {'mode': 'FAKE'}, {'component_mask': {'drive': False}},
    {'stamp_s': time.monotonic()+1000},
])
def test_missing_or_inapplicable_owner_proof_is_not_admission_evidence(change):
    node = _broker_harness()
    msg = message()
    data = json.loads(msg.data)
    data.update(change)
    msg.data = json.dumps(data)
    _ON_SAFETY(node, msg)
    assert _OPS_STATE(node).wheels_stopped is False


def test_future_timestamp_is_stale_instead_of_being_clamped_to_fresh():
    node = _broker_harness()
    node._stamps['gateway'] = time.monotonic()+1000
    assert _OPS_STATE(node).field_age_s['gateway'] > .5
