"""Execute ROS adapter methods on the host; middleware proof is separate."""
import ast
from pathlib import Path
import math
import time
from types import SimpleNamespace

import pytest

from powertrain_ros.remote_input_gateway import DriveOutput


PACKAGE = Path(__file__).resolve().parents[1] / "powertrain_ros"


@pytest.mark.parametrize("command_format,assist_enabled,valid", [
    ("steering", False, True), ("steering", True, False),
    ("twist", True, True), ("twist", False, True), ("steerng", False, False),
])
def test_startup_contract_rejects_unsupported_steering_assist(command_format, assist_enabled, valid):
    from powertrain_ros.remote_input_gateway import validate_manual_command_format
    if valid:
        validate_manual_command_format(command_format, assist_enabled=assist_enabled)
    else:
        with pytest.raises(ValueError, match="manual_command_format"):
            validate_manual_command_format(command_format, assist_enabled=assist_enabled)


class ManualMessage:
    __slots__ = ("speed_mps", "steering")


class TwistMessage:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0)
        self.angular = SimpleNamespace(z=0.0)


def _methods(module, class_name, *names):
    source = PACKAGE / module
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len(methods) == len(names), "manual ROS adapter entrypoint is missing"
    for method in methods:
        for arg in method.args.args:
            arg.annotation = None
        method.returns = None
    scope = {"math": math, "time": time, "String": SimpleNamespace,
             "ManualDriveCommand": ManualMessage, "Twist": TwistMessage}
    compiled = ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[]))
    exec(compile(compiled, str(source), "exec"), scope)
    return type("ExtractedAdapter", (), {name: scope[name] for name in names})


@pytest.mark.parametrize("speed", [0.0, 0.7, -0.7])
def test_default_drive_adapter_publishes_steering_independent_of_speed(speed):
    cls = _methods("teleop_command_node.py", "TeleopCommandNode", "_publish_drive")
    node = cls()
    node._manual_command_format = "steering"
    sent = []
    node.pub_drive = SimpleNamespace(publish=sent.append)
    node._publish_drive(SimpleNamespace(drive=DriveOutput(speed, 0.0, -0.5)))
    assert isinstance(sent[0], ManualMessage)
    assert (sent[0].speed_mps, sent[0].steering) == (speed, -0.5)


def test_explicit_legacy_drive_adapter_keeps_twist_payload():
    cls = _methods("teleop_command_node.py", "TeleopCommandNode", "_publish_drive")
    node = cls()
    node._manual_command_format = "twist"
    sent = []
    node.pub_drive = SimpleNamespace(publish=sent.append)
    node._publish_drive(SimpleNamespace(drive=DriveOutput(0.7, -0.5)))
    assert isinstance(sent[0], TwistMessage)
    assert (sent[0].linear.x, sent[0].angular.z) == (0.7, -0.5)


@pytest.mark.parametrize("age_ms,accepted", [(100, True), (400, False)])
def test_manual_adapter_preserves_steering_and_dds_receipt_age(age_ms, accepted):
    cls = _methods("chassis_node.py", "ChassisNode", "_on_manual_drive_command", "_command_received_s")
    node = cls()
    node._now_s = lambda: 10.0
    sent = []
    node._authority = SimpleNamespace(submit=lambda *args, **kwargs: sent.append((args, kwargs)))
    message = SimpleNamespace(speed_mps=-0.7, steering=0.5)
    info = SimpleNamespace(received_timestamp=time.time_ns() - age_ms * 1_000_000)
    node._on_manual_drive_command(message, info)
    if accepted:
        args, kwargs = sent[0]
        assert args[:3] == ("teleop", -0.7, 0.0)
        assert args[3] == pytest.approx(9.9, abs=0.01)
        assert kwargs == {"steering": 0.5}
    else:
        assert sent == []


@pytest.mark.parametrize("force_hold,v_cap,speed,steering", [(False, None, 0.7, 0.5),
                         (False, 0.3, 0.3, 0.5), (True, None, 0.0, 0.0)])
def test_authority_adapter_preserves_manual_metadata_until_explicit_hold(force_hold, v_cap, speed, steering):
    cls = _methods("chassis_node.py", "ChassisNode", "_tick_authority")
    node = cls()
    node._authority = SimpleNamespace(mode="TELEOP", select=lambda now: SimpleNamespace(
        v=0.7, omega=0.0, steering=0.5, ok=True, reason="teleop"))
    node._assist_enabled = False
    node._section_enforcer = SimpleNamespace(decide=lambda *args, **kwargs: SimpleNamespace(
        force_hold=force_hold, v_cap=v_cap))
    node._section_floor_v_m_s = 0.0
    node._emit_section_enforcement_event = lambda *args: None
    node.pub_authority_state = SimpleNamespace(publish=lambda message: None)
    sent = []
    node.cm = SimpleNamespace(set=lambda *args, **kwargs: sent.append((args, kwargs)))
    node._tick_authority(10.0)
    assert sent == [((speed, 0.0), {"steering": steering})]
    assert node._authority_final_steering == steering


def test_stationary_steering_is_not_reused_as_a_qualified_wheel_stop():
    from chassis.chassis_manager import ChassisManager, build_corners
    from corner_module.fake import FakeDrive, FakeSteer
    from powertrain_ros.wheel_stop import WheelStopConfig, WheelStopPredicate

    cls = _methods("chassis_node.py", "ChassisNode", "_update_local_wheel_stop")
    node = cls()
    cm = ChassisManager(build_corners(lambda cid: FakeSteer(), lambda nid: FakeDrive()))
    node._wheel_stop = WheelStopPredicate(WheelStopConfig(dict.fromkeys(cm.corners, 0.1), dwell_ms=0, qualified=True))
    node.cm = SimpleNamespace(hardware_stop_proof=lambda transport: {"valid": True, "max_feedback_age_ms": 0.0})
    node._drive_transport = "can"
    node._fake_chassis = False
    node._now_s = lambda: 10.0
    node._authority_final_v = node._authority_final_omega = 0.0
    node._authority_final_steering = 0.5
    node._update_local_wheel_stop(cm.snapshot())
    assert node._wheel_stop.confirmed is False
    assert node._wheel_stop.last_reject_reason == "authority_output_nonzero"
