"""ROS-free chassis adapter integration tests for remote assist."""

import ast
import json
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from chassis import remote_assist
from chassis.authority import (
    AUTONOMY,
    AUTO_SOURCE,
    TELEOP,
    MANUAL_SOURCE,
    CommandAuthority,
)


CHASSIS_NODE = (
    Path(__file__).resolve().parents[1]
    / "powertrain_ros"
    / "chassis_node.py"
)


class DataMessage:
    def __init__(self, data=None):
        self.data = data


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class EventClient:
    def __init__(self):
        self.events = []

    def emit(self, event):
        self.events.append(event)
        return True


class Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def _method(name):
    tree = ast.parse(CHASSIS_NODE.read_text(encoding="utf-8"))
    chassis = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "ChassisNode"
    )
    return next(
        node
        for node in chassis.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _node_class():
    names = (
        "_on_assist_correction",
        "_on_assist_bypass",
        "_emit_remote_assist_event",
        "_tick_authority",
    )
    namespace = {
        "json": json,
        "remote_assist": remote_assist,
        "String": DataMessage,
        "Bool": object,
        "time": time,
    }
    methods = []
    for name in names:
        methods.append(_method(name))
    module = ast.Module(body=methods, type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(CHASSIS_NODE), "exec"), namespace)
    return type(
        "ExtractedChassisNode",
        (),
        {name: namespace[name] for name in names},
    )


def _authority(mode, source, *, now_s=10.0, v=1.2, omega=0.1):
    authority = CommandAuthority()
    assert authority.set_mode(mode)
    authority.submit(source, 0.0, 0.0, now_s)
    assert authority.select(now_s).ok
    authority.submit(source, v, omega, now_s)
    return authority


def _node(mode=TELEOP, *, correction_stamp_s=10.0, bypass=False):
    Node = _node_class()
    node = Node()
    source = MANUAL_SOURCE if mode == TELEOP else AUTO_SOURCE
    node._authority = _authority(mode, source)
    node._assist_enabled = True
    node._assist_config = remote_assist.AssistConfig()
    node._assist_correction = remote_assist.AssistCorrection(
        stamp_s=correction_stamp_s,
        omega_correction_rad_s=0.2,
        speed_cap_m_s=0.5,
        confidence=0.8,
    )
    node._assist_bypass_active = bypass
    node._assist_bypass_stamp_s = 10.0
    node._profile_max_speed_m_s = 1.5
    node._observability_event_client = EventClient()
    node._last_remote_assist_event_ns = 0
    node._last_remote_assist_event_state = None
    node._remote_assist_event_period_ns = 0
    node.pub_authority_state = Publisher()
    node.cm = SimpleNamespace(
        commands=[],
        set=lambda v, w: node.cm.commands.append((v, w)),
    )
    node.get_logger = lambda: Logger()
    return node


def test_teleop_selection_composes_fresh_correction_after_authority():
    node = _node()

    node._tick_authority(10.0)

    assert node.cm.commands == [(0.5, pytest.approx(0.3))]
    assert (node._authority_final_v, node._authority_final_omega) == pytest.approx(
        (0.5, 0.3)
    )
    assert node.pub_authority_state.messages[-1].data == (
        "TELEOP|teleop|assist=on"
    )
    assert node._observability_event_client.events[-1]["event_type"] == (
        "REMOTE_ASSIST"
    )
    assert node._observability_event_client.events[-1]["payload"]["state"] == (
        "engaged"
    )


def test_neutral_teleop_selection_stays_exactly_zero_with_fresh_correction():
    node = _node()
    node._authority.submit(MANUAL_SOURCE, 0.0, 0.0, 10.0)

    node._tick_authority(10.0)

    assert node.cm.commands == [(0.0, 0.0)]
    assert (node._authority_final_v, node._authority_final_omega) == (0.0, 0.0)
    event = node._observability_event_client.events[-1]
    assert event["payload"]["reasons"] == ["operator_neutral"]


def test_bypass_true_returns_raw_teleop_on_the_next_authority_tick():
    node = _node(bypass=True)

    node._tick_authority(10.0)

    assert node.cm.commands == [(1.2, 0.1)]
    assert node.pub_authority_state.messages[-1].data.endswith("|assist=off")
    event = node._observability_event_client.events[-1]
    assert event["payload"]["state"] == "bypassed"
    assert event["payload"]["reasons"] == ["assist_bypass"]


def test_stale_correction_degrades_speed_without_changing_operator_omega():
    node = _node(correction_stamp_s=9.0)

    node._tick_authority(10.0)

    assert node.cm.commands == [(pytest.approx(0.9), 0.1)]
    event = node._observability_event_client.events[-1]
    assert event["payload"]["state"] == "degraded"
    assert event["payload"]["reasons"] == ["correction_stale"]


def test_autonomy_selection_never_calls_assist_compose():
    node = _node(mode=AUTONOMY)

    node._tick_authority(10.0)

    assert node.cm.commands == [(1.2, 0.1)]
    assert node.pub_authority_state.messages[-1].data == "AUTONOMY|auto"
    assert node._observability_event_client.events == []


def test_malformed_correction_is_ignored_and_warning_is_throttled():
    node = _node()
    original = node._assist_correction
    node._last_assist_parse_warning_ns = 0
    node._assist_parse_warning_period_ns = 1_000_000_000
    logger = Logger()
    node.get_logger = lambda: logger

    assert node._on_assist_correction(DataMessage("not-json")) is False
    assert node._on_assist_correction(DataMessage("not-json")) is False

    assert node._assist_correction is original
    assert len(logger.warnings) == 1


def test_overflowing_correction_number_is_ignored():
    node = _node()
    original = node._assist_correction
    logger = Logger()
    node.get_logger = lambda: logger
    payload = {
        "stamp_s": 10**1000,
        "omega_correction_rad_s": 0.0,
        "speed_cap_m_s": 1.0,
        "confidence": 1.0,
    }

    assert node._on_assist_correction(DataMessage(json.dumps(payload))) is False

    assert node._assist_correction is original
    assert len(logger.warnings) == 1


def test_bypass_callback_caches_value_and_local_receive_stamp():
    node = _node()
    node._now_s = lambda: 12.25

    node._on_assist_bypass(SimpleNamespace(data=True))

    assert node._assist_bypass_active is True
    assert node._assist_bypass_stamp_s == 12.25


def test_remote_assist_state_change_event_is_rate_limited(monkeypatch):
    clock = {"monotonic_ns": 1_000_000_000}
    monkeypatch.setattr(
        time,
        "monotonic_ns",
        lambda: clock["monotonic_ns"],
    )
    monkeypatch.setattr(time, "time_ns", lambda: 123_000_000_000)
    node = _node()
    node._remote_assist_event_period_ns = 1_000_000_000

    node._tick_authority(10.0)
    node._assist_bypass_active = True
    clock["monotonic_ns"] = 1_500_000_000
    node._tick_authority(10.01)

    assert len(node._observability_event_client.events) == 1
    assert node._observability_event_client.events[0]["payload"]["state"] == (
        "engaged"
    )

    clock["monotonic_ns"] = 2_100_000_000
    node._tick_authority(10.02)

    assert len(node._observability_event_client.events) == 2
    assert node._observability_event_client.events[1]["payload"]["state"] == (
        "bypassed"
    )


def test_assist_parameters_and_subscriptions_stay_inside_authority_boundary():
    source = CHASSIS_NODE.read_text(encoding="utf-8")
    initialize = _method("_initialize")
    assert any(
        isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "declare_parameter"
        and len(call.args) >= 2
        and ast.literal_eval(call.args[0]) == "assist_enabled"
        and ast.literal_eval(call.args[1]) is False
        for call in ast.walk(initialize)
    )
    authority_guard = next(
        node
        for node in ast.walk(initialize)
        if isinstance(node, ast.If)
        and ast.unparse(node.test) == "self._authority_enabled"
    )
    guarded = ast.unparse(authority_guard)
    assert "/autonomy/assist_correction" in guarded
    assert "/teleop/assist_bypass" in guarded
