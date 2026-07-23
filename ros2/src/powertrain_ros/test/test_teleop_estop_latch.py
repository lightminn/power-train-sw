"""○ E-stop 전역 latch(스펙 r6 §2.1) — chassis_node 구독 dedup·진입점 검증."""
import ast
import collections
import json
import re
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import uuid

from powertrain_ros.remote_input import (
    DPad,
    NormalizedAxes,
    ParseResult,
    RemoteInputFrame,
)
from powertrain_ros.remote_input_gateway import (
    DRIVE,
    MOTION_HOLD,
    RemoteInputGateway,
)

PACKAGE = Path(__file__).resolve().parents[1]
CHASSIS_NODE = PACKAGE / "powertrain_ros/chassis_node.py"
SOURCE = CHASSIS_NODE.read_text(encoding="utf-8")
TELEOP_NODE = PACKAGE / "powertrain_ros/teleop_command_node.py"
TELEOP_SOURCE = TELEOP_NODE.read_text(encoding="utf-8")


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


def _extract_teleop_methods(*names):
    tree = ast.parse(TELEOP_SOURCE)
    cls = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef)
        and item.name == "TeleopCommandNode"
    )
    methods = [
        item
        for item in cls.body
        if isinstance(item, ast.FunctionDef) and item.name in names
    ]
    module = ast.Module(body=methods, type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "deque": collections.deque,
        "ESTOP_REBROADCAST_S": 1.0,
        "json": json,
        "MAX_EVENTS_PER_TICK": 256,
        "MAX_LIFECYCLE_EVENTS": 8,
        "MAX_VIOLATION_EVENTS_PER_S": 50,
        "MAX_VIOLATION_KINDS": 64,
        "String": SimpleNamespace,
        "threading": threading,
        "time": time,
        "uuid": uuid,
    }
    exec(compile(module, str(TELEOP_NODE), "exec"), namespace)
    return {name: namespace[name] for name in names}


_TELEOP_METHODS = _extract_teleop_methods(
    "_begin_estop_event",
    "_drain_events",
    "_publish_estop_event",
    "_queue_decoder_results",
    "_queue_lifecycle_event",
    "_queue_motion_frame",
    "_queue_violation",
)


class _RecordingCm:
    def __init__(self):
        self.calls = []

    def estop(self, source, detail=""):
        self.calls.append((source, detail))


class _Logger:
    def error(self, *_args):
        pass


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _TeleopHarness:
    _begin_estop_event = _TELEOP_METHODS["_begin_estop_event"]
    _drain_events = _TELEOP_METHODS["_drain_events"]
    _publish_estop_event = _TELEOP_METHODS["_publish_estop_event"]
    _queue_decoder_results = _TELEOP_METHODS["_queue_decoder_results"]
    _queue_lifecycle_event = _TELEOP_METHODS["_queue_lifecycle_event"]
    _queue_motion_frame = _TELEOP_METHODS["_queue_motion_frame"]
    _queue_violation = _TELEOP_METHODS["_queue_violation"]

    def __init__(self):
        self._events_lock = threading.Lock()
        self._motion_frame = None
        self._motion_frames_dropped = 0
        self._lifecycle_events = collections.deque(maxlen=8)
        self._violation_events = collections.deque(maxlen=64)
        self._violation_rate_lock = threading.Lock()
        self._violation_window_start_s = 0.0
        self._violation_events_in_window = 0
        self._violation_events_suppressed = 0
        self._violation_events_reported = 0
        self._last_violation_log_s = None
        self._gateway = RemoteInputGateway()
        self._last_frame = None
        self._estop_lock = threading.Lock()
        self._estop_event = None
        self.pub_estop = _Publisher()

    def _log_violation_throttled(self, _message, _now_s):
        return True


def _remote_frame(
    session_id,
    sequence,
    received_s,
    *,
    estop_edge=False,
):
    return RemoteInputFrame(
        schema_version=2,
        session_id=session_id,
        sequence=sequence,
        client_monotonic_ns=0,
        mode="DRIVE",
        deadman=False,
        axes=NormalizedAxes(
            left_x=0.0,
            right_y=0.0,
            left_trigger=0.0,
            right_trigger=0.0,
        ),
        dpad=DPad(x=0, y=0),
        mode_chord=False,
        estop_edge=estop_edge,
        assist_bypass=False,
        received_monotonic_s=received_s,
    )


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


def test_coalesced_estop_then_normal_frame_latches_gateway_and_publishes():
    harness = _TeleopHarness()
    session_id = uuid.uuid4().hex
    harness._queue_lifecycle_event("connect", session_id)
    harness._queue_decoder_results(
        [
            ParseResult(
                frame=_remote_frame(session_id, 0, 0.0),
            )
        ],
        now_s=0.0,
    )
    harness._drain_events(now_s=0.0)
    assert harness._gateway.tick(0.0).state == DRIVE

    harness._queue_decoder_results(
        [
            ParseResult(
                frame=_remote_frame(
                    session_id,
                    1,
                    0.01,
                    estop_edge=True,
                )
            ),
            ParseResult(
                frame=_remote_frame(session_id, 2, 0.02),
            ),
        ],
        now_s=0.02,
    )
    harness._drain_events(now_s=0.02)

    output = harness._gateway.tick(0.02)
    assert output.state == MOTION_HOLD
    assert "E-stop" in output.reason
    assert harness._publish_estop_event() is True
    assert len(harness.pub_estop.messages) == 1
