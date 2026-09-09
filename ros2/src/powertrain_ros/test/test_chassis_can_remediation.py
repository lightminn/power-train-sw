"""Execute extracted node methods without claiming ROS middleware proof."""
import ast
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

SOURCE = Path(__file__).parents[1] / 'powertrain_ros/chassis_node.py'


def node_class(*names):
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ChassisNode')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert len(methods) == len(names)
    for method in methods:
        for arg in method.args.args:
            arg.annotation = None
        method.returns = None
    scope = {'time': time, 'math': math, 'json': json, 'String': SimpleNamespace}
    module = ast.Module(body=methods, type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), 'exec'), scope)
    return type('NodeMethods', (), {name: scope[name] for name in names})


def test_old_twist_cannot_rejuvenate_manual_command():
    cls = node_class('_on_cmd_vel', '_command_received_s')
    node = cls()
    seen = []
    node.cm = SimpleNamespace(set=lambda *args: seen.append(args))
    node._now_s = lambda: 10.0
    message = SimpleNamespace(linear=SimpleNamespace(x=1.0), angular=SimpleNamespace(z=0))
    info = SimpleNamespace(received_timestamp=time.time_ns() - 400_000_000)
    node._on_cmd_vel(message, info)
    assert seen == []


def test_authority_uses_transport_receipt_time_not_callback_time():
    cls = node_class('_on_authority_cmd', '_command_received_s')
    node = cls()
    seen = []
    node._authority = SimpleNamespace(submit=lambda *args: seen.append(args))
    node._now_s = lambda: 10.0
    message = SimpleNamespace(linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0))
    info = SimpleNamespace(received_timestamp=time.time_ns() - 100_000_000)
    node._on_authority_cmd('manual', message, info)
    assert seen[0][-1] == pytest.approx(9.9, abs=.01)


def test_services_cannot_refresh_safety_without_a_new_verdict():
    cls = node_class('_refresh_safety_baseline')
    node = cls()
    node._last_safety_ms = 1000
    node._now_ms = lambda: 1200
    node._refresh_safety_baseline()
    assert node._last_safety_ms == 1000


def test_external_wheel_publisher_cannot_update_local_stop_predicate():
    cls = node_class('_on_wheel_states_for_stop')
    node = cls()
    updates = []
    node._wheel_stop = SimpleNamespace(update=lambda *args, **kwargs: updates.append(args))
    node._on_wheel_states_for_stop(SimpleNamespace())
    assert updates == []


def test_manual_subscription_keeps_only_latest_twist():
    tree = ast.parse(SOURCE.read_text())
    subscriptions = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Attribute) and n.func.attr == 'create_subscription']
    for topic in ['/cmd_vel', '/teleop/cmd_vel']:
        subscription = next(n for n in subscriptions if len(n.args) >= 4
                            and isinstance(n.args[1], ast.Constant) and n.args[1].value == topic)
        assert isinstance(subscription.args[3], ast.Constant) and subscription.args[3].value == 1


def test_watchdog_step_uses_existing_executor_state_callback():
    cls = node_class('_publish_state')
    calls = []
    node = cls()
    node._can_watchdog = SimpleNamespace(step=lambda: calls.append('step'))
    node.cm = SimpleNamespace(state=lambda: {'mode': 'IDLE', 'v': 0, 'omega': 0})
    node._v = node._omega = 0.0
    node._roll = node._pitch = 0.0
    node.get_logger = lambda: SimpleNamespace(info=lambda *args: None)
    node.pub_state = SimpleNamespace(publish=lambda message: None)
    node._publish_state()
    assert calls == ['step']


def test_changed_speed_cannot_reuse_confirmed_stop_with_same_oldest_stamp():
    from powertrain_ros.wheel_stop import WheelStopConfig, WheelStopPredicate
    from chassis.chassis_manager import ChassisManager, build_corners
    from corner_module.fake import FakeDrive, FakeSteer
    cls = node_class('_update_local_wheel_stop')
    node = cls()
    cm = ChassisManager(build_corners(lambda cid: FakeSteer(), lambda nid: FakeDrive()))
    names = list(cm.corners)
    node._wheel_stop = WheelStopPredicate(WheelStopConfig(dict.fromkeys(names, .1), dwell_ms=0, qualified=True))
    now = [10.0]
    age = [0.0]
    node.cm = SimpleNamespace(hardware_stop_proof=lambda transport: {'valid': True, 'max_feedback_age_ms': age[0]})
    node._drive_transport = 'can'
    node._fake_chassis = False
    node._now_s = lambda: now[0]
    node._authority_final_v = node._authority_final_omega = 0.0
    node._update_local_wheel_stop(cm.snapshot())
    assert node._wheel_stop.confirmed
    next(iter(cm.corners.values())).drive._actual = 1.0
    now[0] += .02
    age[0] = 20.0  # Heartbeat remains the oldest required feedback timestamp.
    node._update_local_wheel_stop(cm.snapshot())
    assert node._wheel_stop.confirmed is False
