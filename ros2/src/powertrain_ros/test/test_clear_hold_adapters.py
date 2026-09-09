"""Integrated composite clear adapters are idempotent across one-sided HOLD."""
from types import SimpleNamespace

from chassis.authority import (
    CommandAuthority,
    IDLE,
    MOTION_HOLD as AUTHORITY_HOLD,
    TELEOP,
)
from powertrain_ros.chassis_node import ChassisNode
from powertrain_ros.remote_input_gateway import (
    DISCONNECTED,
    DRIVE,
    MOTION_HOLD as GATEWAY_HOLD,
    RemoteInputGateway,
)
from powertrain_ros.teleop_command_node import TeleopCommandNode


def _response():
    return SimpleNamespace(success=None, message="")


def _clear_gateway(gateway):
    node = SimpleNamespace(_gateway=gateway)
    return TeleopCommandNode._clear_hold(node, None, _response())


def _clear_authority(authority):
    node = SimpleNamespace(_authority=authority)
    return ChassisNode._clear_authority_hold(node, None, _response())


def test_only_gateway_hold_clears_and_authority_mode_is_unchanged():
    gateway = RemoteInputGateway()
    gateway.state = GATEWAY_HOLD
    authority = CommandAuthority()
    authority.mode = TELEOP

    gateway_result = _clear_gateway(gateway)
    authority_result = _clear_authority(authority)

    assert gateway_result.success is True
    assert gateway.state == DISCONNECTED
    assert authority_result.success is True
    assert authority.mode == TELEOP
    assert "already clear" in authority_result.message


def test_only_authority_hold_clears_and_gateway_mode_is_unchanged():
    gateway = RemoteInputGateway()
    gateway.state = DRIVE
    authority = CommandAuthority()
    authority.mode = AUTHORITY_HOLD

    gateway_result = _clear_gateway(gateway)
    authority_result = _clear_authority(authority)

    assert gateway_result.success is True
    assert gateway.state == DRIVE
    assert "already clear" in gateway_result.message
    assert authority_result.success is True
    assert authority.mode == IDLE


def test_no_hold_is_successful_noop_without_resuming_either_mode():
    gateway = RemoteInputGateway()
    gateway.state = DISCONNECTED
    authority = CommandAuthority()
    authority.mode = TELEOP

    gateway_result = _clear_gateway(gateway)
    authority_result = _clear_authority(authority)

    assert gateway_result.success is authority_result.success is True
    assert gateway.state == DISCONNECTED
    assert authority.mode == TELEOP
    assert "already clear" in gateway_result.message
    assert "already clear" in authority_result.message
