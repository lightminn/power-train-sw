"""broker ops-state 소스 토픽 계약 (스펙 r6 §3.1)."""
import ast
import json
import re
import threading
import time
from pathlib import Path
from types import SimpleNamespace

from powertrain_ros.ops_broker_core import (
    DEFAULT_COMPONENT_MASK,
    OpsState,
)
from powertrain_ros.remote_input_gateway import frame_is_neutral
from test_remote_input_gateway import _frame

PACKAGE = Path(__file__).resolve().parents[1]
TELEOP = (PACKAGE / "powertrain_ros/teleop_command_node.py").read_text(
    encoding="utf-8"
)
CHASSIS = (PACKAGE / "powertrain_ros/chassis_node.py").read_text(
    encoding="utf-8"
)
OPS_BROKER = (PACKAGE / "powertrain_ros/ops_broker_node.py").read_text(
    encoding="utf-8"
)


def _semantic_fields():
    tree = ast.parse(OPS_BROKER)
    assignment = next(
        item
        for item in tree.body
        if isinstance(item, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "_SEMANTIC_FIELDS"
            for target in item.targets
        )
    )
    return ast.literal_eval(assignment.value)


_SEMANTIC_FIELDS = _semantic_fields()


def _extract_broker_method(name):
    tree = ast.parse(OPS_BROKER)
    cls = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "OpsBrokerNode"
    )
    method = next(
        item
        for item in cls.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "DEFAULT_COMPONENT_MASK": DEFAULT_COMPONENT_MASK,
        "OpsState": OpsState,
        "_SEMANTIC_FIELDS": _SEMANTIC_FIELDS,
        "json": json,
        "math": __import__("math"),
        "time": time,
    }
    exec(compile(module, str(PACKAGE / "powertrain_ros/ops_broker_node.py"), "exec"), namespace)
    return namespace[name]


_ON_SAFETY = _extract_broker_method("_on_safety")
_OPS_STATE = _extract_broker_method("_ops_state")
_PUSH_OPS_STATE = _extract_broker_method("_push_ops_state")


def test_frame_is_neutral_matches_gateway_semantics():
    assert frame_is_neutral(_frame())
    assert not frame_is_neutral(_frame(deadman=True))
    assert not frame_is_neutral(_frame(right_trigger=0.2))
    assert not frame_is_neutral(_frame(dpad_x=1))
    assert not frame_is_neutral(_frame(estop_edge=True))


def test_teleop_publishes_gateway_state_each_tick():
    assert '"/teleop/gateway_state"' in TELEOP
    assert '"neutral"' in TELEOP and '"input_fresh"' in TELEOP


def test_chassis_publishes_safety_state():
    assert '"/chassis/safety_state"' in CHASSIS
    assert '"estop_latched"' in CHASSIS
    assert '"active_estop_sources"' in CHASSIS
    assert '"component_mask"' in CHASSIS


class _Logger:
    def warning(self, *_args):
        pass


def _broker_harness():
    return SimpleNamespace(
        _state_lock=threading.Lock(),
        _fields={name: None for name in _SEMANTIC_FIELDS},
        _stamps={
            "authority": None,
            "gateway": None,
            "safety": None,
            "wheels": None,
        },
        _revision=0,
        _last_semantic=None,
        get_logger=lambda: _Logger(),
    )


def _safety_message(component_mask=None, *, include_mask=True):
    payload = {
        "stamp_s": time.monotonic(),
        "estop_latched": False,
        "active_estop_sources": [],
    }
    if include_mask:
        payload["component_mask"] = component_mask
    return SimpleNamespace(data=json.dumps(payload))


def test_safety_state_component_mask_updates_ops_state_and_revision():
    node = _broker_harness()
    all_on = {
        "drive": True,
        "steer": True,
        "us100": True,
        "robot_arm": True,
    }
    _ON_SAFETY(node, _safety_message(all_on))
    initial = _OPS_STATE(node)

    changed = dict(all_on, us100=False)
    _ON_SAFETY(node, _safety_message(changed))
    updated = _OPS_STATE(node)

    assert initial.component_mask == all_on
    assert updated.component_mask == changed
    assert updated.revision == initial.revision + 1


def test_safety_state_without_component_mask_defaults_to_all_enabled():
    node = _broker_harness()

    _ON_SAFETY(
        node,
        _safety_message(include_mask=False),
    )

    assert _OPS_STATE(node).component_mask == {
        "drive": True,
        "steer": True,
        "us100": True,
        "robot_arm": True,
    }


def test_ops_state_push_serializes_component_mask():
    node = _broker_harness()
    expected = {
        "drive": False,
        "steer": True,
        "us100": True,
        "robot_arm": True,
    }
    _ON_SAFETY(node, _safety_message(expected))
    state = _OPS_STATE(node)
    sent = []
    connection = object()
    node._closed = False
    node._ops_state = lambda: state
    node._connections_lock = threading.Lock()
    node._connections = [connection]
    node._send = lambda target, payload: sent.append((target, payload))

    _PUSH_OPS_STATE(node)

    assert sent[0][0] is connection
    assert json.loads(sent[0][1])["component_mask"] == expected
