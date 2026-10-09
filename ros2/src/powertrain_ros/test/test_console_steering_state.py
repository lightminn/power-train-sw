"""Chassis safety state -> broker push -> existing console steering selector."""
import json
import threading
import time

import pytest

from operator_console.ops_panel import PANEL_ACTIONS, action_is_available
from test_ops_state_sources import (
    _broker_harness, _ON_SAFETY, _OPS_STATE, _PUSH_OPS_STATE, _safety_message,
)


def feed(node, **fields):
    message = _safety_message(include_mask=False)
    payload = json.loads(message.data)
    payload.update(fields)
    message.data = json.dumps(payload)
    _ON_SAFETY(node, message)


def pushed(node):
    sent = []
    node._closed = False
    node._ops_state = lambda: _OPS_STATE(node)
    node._connections_lock = threading.Lock()
    node._connections = [object()]
    node._send = lambda _, payload: sent.append(json.loads(payload))
    _PUSH_OPS_STATE(node)
    return sent[-1]


@pytest.mark.parametrize('mode,available,transport,label,enabled,target', [
    ('ackermann', True, 'can', '애커만', True, True),
    ('skid', True, 'can', '스키드', True, False),
    ('skid', False, 'usb', '스키드', False, False),
])
def test_real_broker_push_enables_existing_console_selector(
    mode, available, transport, label, enabled, target,
):
    node = _broker_harness()
    feed(node, steering_mode=mode, steering_available=available,
         drive_transport=transport)
    state = pushed(node)
    assert state['steering_mode'] == mode
    assert state['steering_available'] is available
    assert state['drive_transport'] == transport
    action = next(a for a in PANEL_ACTIONS if a.action == 'steer_mode_skid')
    assert action.state_text_from_state(state) == label
    assert action.bool_value_from_state(state) is target
    assert action_is_available(action.action, state)[0] is enabled


def test_steering_changes_invalidate_old_confirmation_revision():
    node = _broker_harness()
    fields = dict(steering_mode='ackermann', steering_available=True,
                  drive_transport='can')
    feed(node, **fields)
    previous = _OPS_STATE(node).revision
    feed(node, **fields)
    assert _OPS_STATE(node).revision == previous
    for key, value in (('steering_mode', 'skid'),
                       ('steering_available', False), ('drive_transport', 'usb')):
        fields[key] = value
        feed(node, **fields)
        current = _OPS_STATE(node).revision
        assert current == previous + 1
        previous = current


@pytest.mark.parametrize('invalid', [None, '', 'crab', True, {}, []])
def test_invalid_mode_disables_selection_without_losing_estop(invalid):
    node = _broker_harness()
    feed(node, steering_mode='ackermann', steering_available=True,
         drive_transport='can')
    feed(node, steering_mode=invalid, estop_latched=True,
         active_estop_sources=['corner_fault'], mode='ESTOP')
    state = pushed(node)
    assert state['steering_mode'] is None
    assert not action_is_available('steer_mode_skid', state)[0]
    assert state['estop_latched'] and state['chassis_mode'] == 'ESTOP'


def test_old_publisher_clears_previous_steering_state():
    node = _broker_harness()
    feed(node, steering_mode='skid', steering_available=True, drive_transport='can')
    feed(node)
    state = pushed(node)
    assert state['steering_mode'] is None
    assert state['steering_available'] is False
    assert state['drive_transport'] is None


@pytest.mark.parametrize('offset', [-1.0, 1.0])
def test_stale_or_future_safety_cannot_keep_selector_enabled(offset):
    node = _broker_harness()
    feed(node, steering_mode='ackermann', steering_available=True,
         drive_transport='can', stamp_s=time.monotonic() + offset)
    state = pushed(node)
    assert state['steering_mode'] is None
    assert state['steering_available'] is False
    assert state['drive_transport'] is None
    assert not action_is_available('steer_mode_skid', state)[0]


@pytest.mark.parametrize('invalid', [1, 'true', None])
def test_non_boolean_availability_does_not_unlock_usb_return(invalid):
    node = _broker_harness()
    feed(node, steering_mode='skid', steering_available=invalid, drive_transport='usb')
    state = pushed(node)
    assert state['steering_available'] is False
    assert not action_is_available('steer_mode_skid', state)[0]


def test_future_capability_never_becomes_valid_without_a_new_message(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    node = _broker_harness()
    fields = dict(steering_mode='ackermann', steering_available=True,
                  drive_transport='can')
    feed(node, **fields, stamp_s=101.0)
    assert pushed(node)['steering_mode'] is None
    clock[0] = 101.1
    assert pushed(node)['steering_mode'] is None
    feed(node, **fields)
    assert pushed(node)['steering_mode'] == 'ackermann'
