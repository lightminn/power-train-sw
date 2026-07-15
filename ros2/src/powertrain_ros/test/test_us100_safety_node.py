import ast
from pathlib import Path
import threading
import time
from types import SimpleNamespace

from safety_us100.config import SafetyConfig
from safety_us100.fake_sensor import FakeUs100
from safety_us100.safety_monitor import SafetyMonitor
from safety_us100.verdict import (
    NO_RESPONSE,
    VALID,
    SensorReading,
    Verdict,
)


PACKAGE = Path(__file__).resolve().parents[1]
US100_NODE = PACKAGE / "powertrain_ros/us100_safety_node.py"


def _method_ast(name):
    tree = ast.parse(US100_NODE.read_text(encoding="utf-8"))
    cls = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "Us100SafetyNode"
    )
    return next(
        item
        for item in cls.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )


def _fill_message(message, verdict, stamp):
    message.verdict = verdict
    message.stamp = stamp


def _node_class(*method_names):
    namespace = {
        "NO_RESPONSE": NO_RESPONSE,
        "SafetyVerdictMsg": SimpleNamespace,
        "Verdict": Verdict,
        "fill_safety_message": _fill_message,
        "threading": threading,
        "time": time,
    }
    for name in method_names:
        method = _method_ast(name)
        module = ast.Module(body=[method], type_ignores=[])
        ast.fix_missing_locations(module)
        exec(compile(module, str(US100_NODE), "exec"), namespace)
    return type(
        "ExtractedUs100SafetyNode",
        (),
        {name: namespace[name] for name in method_names},
    )


class _ClosingSensor:
    def __init__(self):
        self.close_count = 0
        self.reader_alive = lambda: False
        self.reader_alive_at_close = []

    def close(self):
        self.close_count += 1
        self.reader_alive_at_close.append(self.reader_alive())


def _reader_node(monitor, sensor, latest=None, period_s=0.01):
    ReaderNode = _node_class(
        "_fail_safe_verdict",
        "_start_reader",
        "_reader_loop",
        "_snapshot_for_publish",
        "_sample",
        "close",
    )
    node = ReaderNode()
    node.monitor = monitor
    node.sensor = sensor
    node._reader_lock = threading.Lock()
    node._reader_stop = threading.Event()
    node._reader_thread = None
    node._reader_period_s = period_s
    node._reader_join_timeout_s = 0.25
    node._latest_verdict = latest or monitor.verdict()
    return node


def _install_publisher(node):
    messages = []
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=time.monotonic),
    )
    node.publisher = SimpleNamespace(publish=messages.append)
    return messages


def _wait_for(predicate, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.002)
    return predicate()


class _FirstValidThenTwoSecondBlockSensor(_ClosingSensor):
    def __init__(self):
        super().__init__()
        self.read_count = 0
        self.blocking_read_entered = threading.Event()

    def read(self):
        self.read_count += 1
        if self.read_count == 1:
            return SensorReading(VALID, 800.0, "distance")
        self.blocking_read_entered.set()
        time.sleep(2.0)
        return SensorReading(VALID, 700.0, "distance")


def test_sample_keeps_10hz_cadence_and_last_snapshot_during_two_second_read():
    sensor = _FirstValidThenTwoSecondBlockSensor()
    monitor = SafetyMonitor(sensor, SafetyConfig())
    node = _reader_node(monitor, sensor)
    messages = _install_publisher(node)

    node._start_reader()
    reader_thread = node._reader_thread
    assert sensor.blocking_read_entered.wait(0.5)

    period_s = 0.1
    started = time.monotonic()
    for index in range(8):
        deadline = started + index * period_s
        time.sleep(max(0.0, deadline - time.monotonic()))
        callback_started = time.monotonic()
        node._sample()
        assert time.monotonic() - callback_started < 0.03

    stamps = [message.stamp for message in messages]
    gaps = [later - earlier for earlier, later in zip(stamps, stamps[1:])]
    assert len(stamps) == 8
    assert min(gaps) >= 0.06
    assert max(gaps) <= 0.14
    assert all(message.verdict.status == VALID for message in messages)
    assert all(message.verdict.distance_mm == 800.0 for message in messages)

    node.close()
    reader_thread.join(2.0)
    assert not reader_thread.is_alive()


def test_sample_fails_safe_when_reader_is_not_started_or_has_died():
    sensor = _ClosingSensor()
    monitor = SafetyMonitor(FakeUs100([]), SafetyConfig())
    manipulated = Verdict(VALID, 999.0, False, 0, "manipulated")
    node = _reader_node(monitor, sensor, latest=manipulated)
    messages = _install_publisher(node)

    node._sample()
    node._reader_thread = SimpleNamespace(is_alive=lambda: False)
    node._sample()

    assert len(messages) == 2
    assert all(message.verdict.status == NO_RESPONSE for message in messages)
    assert all(message.verdict.estop_required for message in messages)
    assert all(message.verdict.distance_mm is None for message in messages)


def test_sample_keeps_legacy_partial_node_fixture_fail_safe():
    SampleNode = _node_class("_sample")
    node = SimpleNamespace()
    messages = _install_publisher(node)

    SampleNode._sample(node)

    assert len(messages) == 1
    assert messages[0].verdict.status == NO_RESPONSE
    assert messages[0].verdict.estop_required is True


def test_reader_preserves_three_miss_no_response_promotion():
    miss = SensorReading(NO_RESPONSE, None, "liveness_timeout")
    sensor = FakeUs100([miss, miss, miss])
    monitor = SafetyMonitor(sensor, SafetyConfig(fail_stop_count=3))
    node = _reader_node(monitor, _ClosingSensor(), period_s=0.005)

    node._start_reader()

    def promoted_snapshot():
        verdict = node._snapshot_for_publish()
        return verdict if verdict.status == NO_RESPONSE else None

    promoted = _wait_for(promoted_snapshot)
    node.close()

    assert promoted is not None
    assert promoted.estop_required is True
    assert promoted.consecutive_failures >= 3


def test_close_joins_reader_before_closing_sensor():
    sensor = _ClosingSensor()
    monitor = SafetyMonitor(
        FakeUs100([SensorReading(VALID, 500.0, "distance")]),
        SafetyConfig(),
    )
    node = _reader_node(monitor, sensor, period_s=5.0)

    node._start_reader()
    reader_thread = node._reader_thread
    sensor.reader_alive = reader_thread.is_alive
    assert _wait_for(lambda: reader_thread.is_alive())

    node.close()

    assert not reader_thread.is_alive()
    assert sensor.close_count == 1
    assert sensor.reader_alive_at_close == [False]
    assert node.sensor is None


def test_close_keeps_legacy_partial_node_fixture_compatible():
    sensor = _ClosingSensor()
    node = SimpleNamespace(sensor=sensor)
    CloseNode = _node_class("close")

    CloseNode.close(node)

    assert sensor.close_count == 1
    assert node.sensor is None
