"""○ E-stop 전역 latch(스펙 r6 §2.1) — chassis_node 구독 dedup·진입점 검증."""
import ast
import collections
import json
import re
from pathlib import Path
from types import SimpleNamespace

PACKAGE = Path(__file__).resolve().parents[1]
CHASSIS_NODE = PACKAGE / "powertrain_ros/chassis_node.py"
SOURCE = CHASSIS_NODE.read_text(encoding="utf-8")


def _extract_method(name):
    tree = ast.parse(SOURCE)
    cls = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "ChassisNode"
    )
    method = next(
        item
        for item in cls.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {"json": json, "collections": collections}
    exec(compile(module, str(CHASSIS_NODE), "exec"), namespace)
    return namespace[name]


class _RecordingCm:
    def __init__(self):
        self.calls = []

    def estop(self, source, detail=""):
        self.calls.append((source, detail))


class _Logger:
    def error(self, *_args):
        pass


def _node(cm):
    return SimpleNamespace(
        cm=cm,
        _teleop_estop_seen=collections.OrderedDict(),
        get_logger=lambda: _Logger(),
    )


def _msg(event_id="abc123", stamp_s=1.5):
    return SimpleNamespace(
        data=json.dumps({"event_id": event_id, "stamp_s": stamp_s})
    )


def test_first_event_trips_cm_estop_with_remote_operator_source():
    on_estop = _extract_method("_on_teleop_estop")
    cm = _RecordingCm()
    node = _node(cm)

    on_estop(node, _msg())

    assert len(cm.calls) == 1
    source, detail = cm.calls[0]
    assert source == "remote_operator"
    assert "abc123" in detail


def test_duplicate_event_id_is_idempotent():
    on_estop = _extract_method("_on_teleop_estop")
    cm = _RecordingCm()
    node = _node(cm)

    on_estop(node, _msg())
    on_estop(node, _msg())          # 재발행(같은 event_id) — 1회만 trip

    assert len(cm.calls) == 1


def test_new_event_id_trips_again_and_ledger_is_bounded():
    on_estop = _extract_method("_on_teleop_estop")
    cm = _RecordingCm()
    node = _node(cm)

    for index in range(40):
        on_estop(node, _msg(event_id="event-%d" % index))

    assert len(cm.calls) == 40
    assert len(node._teleop_estop_seen) <= 32


def test_invalid_payload_is_rejected_without_trip():
    on_estop = _extract_method("_on_teleop_estop")
    cm = _RecordingCm()
    node = _node(cm)

    on_estop(node, SimpleNamespace(data="not json"))
    on_estop(node, SimpleNamespace(data=json.dumps({"event_id": "x"})))

    assert cm.calls == []


def test_subscription_is_unconditional_and_transient_local():
    """구독은 authority_enabled 와 무관하게 항상 생성 + latched QoS 여야 한다."""
    assert re.search(
        r'create_subscription\(\s*String,\s*"/teleop/estop",\s*'
        r"self\._on_teleop_estop,",
        SOURCE,
    ), "chassis_node must subscribe /teleop/estop"
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in SOURCE
