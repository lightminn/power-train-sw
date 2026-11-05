"""chassis_node demo-mode switch (docs/specs/2026-11-06-demo-mode.md D1·D2·D3)."""
import json
import re
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from powertrain_ros.chassis_node import ChassisNode

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "wp5_control.launch.py"


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _node(tmp_path, demo):
    return ChassisNode(parameter_overrides=[
        Parameter("fake", value=True),
        Parameter("safety_required", value=False),
        Parameter("demo_mode", value=demo),
        Parameter("console_estop_latch_path", value=str(tmp_path / "estop.json")),
    ])


@pytest.mark.parametrize("demo,gate", [(True, "arm_absent_field"), (False, "production")])
def test_demo_parameter_sets_arm_gate_and_fault_stop(tmp_path, demo, gate):
    node = _node(tmp_path, demo)
    try:
        assert node._demo_mode is demo
        assert node._arm_gate_mode == gate
        assert node.cm.cfg.demo_fault_stop is demo
        assert node.describe_parameter("demo_mode").read_only
    finally:
        node.close()
        node.destroy_node()


def _handler(demo, result=True, authority="TELEOP", gateway_ok=True):
    calls, logs = [], []
    node = SimpleNamespace(
        _demo_mode=demo,
        _authority=SimpleNamespace(mode=authority),
        _gateway_input_ok=gateway_ok,
        cm=SimpleNamespace(arm_no_clear=lambda **kw: calls.append(kw) or result,
                           mode="ARMED" if result else "IDLE"),
        _refresh_safety_baseline=lambda: None,
        get_logger=lambda: SimpleNamespace(info=logs.append, warning=logs.append,
                                           error=logs.append))
    return node, calls, logs


def test_demo_auto_arm_calls_compare_and_arm_with_the_counter():
    node, calls, _ = _handler(True)
    ChassisNode._on_demo_auto_arm(node, String(data=json.dumps({"expect_disarm_seq": 3})))
    assert calls == [{"expect_disarm_seq": 3}]


@pytest.mark.parametrize("demo,data", [
    (False, json.dumps({"expect_disarm_seq": 3})),
    (True, "not json"),
    (True, json.dumps({"expect_disarm_seq": True})),
    (True, json.dumps({"expect_disarm_seq": 3, "clear": True})),
    (True, json.dumps([3])),
])
def test_demo_auto_arm_ignores_wrong_mode_or_payload(demo, data):
    node, calls, logs = _handler(demo)
    ChassisNode._on_demo_auto_arm(node, String(data=data))
    assert calls == [] and logs


@pytest.mark.parametrize("authority,gateway_ok", [
    ("MOTION_HOLD", True), ("IDLE", True), ("TELEOP", False),
])
def test_demo_auto_arm_rechecks_hold_and_gateway(authority, gateway_ok):
    node, calls, logs = _handler(True, authority=authority, gateway_ok=gateway_ok)
    ChassisNode._on_demo_auto_arm(node, String(data=json.dumps({"expect_disarm_seq": 3})))
    assert calls == [] and logs


def test_launch_exposes_demo_mode_default_off():
    launch = LAUNCH.read_text(encoding="utf-8")
    assert re.search(r'DeclareLaunchArgument\(\s*"demo_mode",\s*default_value="false"', launch)
    assert 'LaunchConfiguration("demo_mode")' in launch
