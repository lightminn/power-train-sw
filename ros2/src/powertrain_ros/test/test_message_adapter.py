import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from powertrain_ros import chassis_node, us100_safety_node
from powertrain_ros.chassis_node import ChassisNode
from powertrain_ros.message_adapter import (
    fill_safety_message,
    fill_wheel_states_message,
)
from safety_us100.us100 import Us100Sensor
from safety_us100.verdict import NO_RESPONSE


def test_fill_safety_message_uses_nan_for_missing_distance():
    msg = SimpleNamespace(header=SimpleNamespace())
    verdict = SimpleNamespace(
        status="CHECKING",
        distance_mm=None,
        estop_required=False,
        consecutive_failures=1,
        detail="waiting",
    )

    fill_safety_message(msg, verdict, stamp="stamp")

    assert msg.header.stamp == "stamp"
    assert msg.header.frame_id == "us100_link"
    assert msg.status == 0
    assert math.isnan(msg.distance_mm)
    assert msg.estop_required is False
    assert msg.consecutive_failures == 1
    assert msg.detail == "waiting"


def test_fill_wheel_states_uses_actual_snapshot_values():
    msg = SimpleNamespace(header=SimpleNamespace())
    wheel = SimpleNamespace(
        name="front_left",
        corner_mode="ARMED",
        drive_turns_per_s=1.2,
        steer_deg=3.0,
        drive_current_a=0.4,
        steer_current_a=0.2,
        drive_stale=False,
        steer_stale=False,
        drive_axis_error=0,
        steer_fault=0,
    )
    snapshot = SimpleNamespace(
        chassis_mode="ARMED",
        stop_state="RUN",
        healthy=True,
        wheels=(wheel,),
    )

    fill_wheel_states_message(
        msg,
        snapshot,
        "stamp",
        4.5,
        2,
        wheel_factory=SimpleNamespace,
    )

    assert msg.header.stamp == "stamp"
    assert msg.header.frame_id == "base_link"
    assert msg.chassis_mode == "ARMED"
    assert msg.stop_state == "RUN"
    assert msg.healthy is True
    assert msg.wheels[0].name == "front_left"
    assert msg.wheels[0].drive_turns_per_s == 1.2
    assert msg.tick_duration_ms == 4.5
    assert msg.overrun_count == 2


class _RecordingLogger:
    def __init__(self):
        self.errors = []
        self.infos = []

    def error(self, message):
        self.errors.append(message)

    def info(self, message):
        self.infos.append(message)


class _ClosingCorner:
    def __init__(self, error=None):
        self.error = error
        self.close_count = 0

    def close(self):
        self.close_count += 1
        if self.error is not None:
            raise self.error


def test_chassis_close_estops_and_closes_later_corners_after_failure():
    first = _ClosingCorner(RuntimeError("first close failed"))
    second = _ClosingCorner()
    manager = SimpleNamespace(
        corners={"first": first, "second": second},
        estop_calls=[],
    )
    manager.estop = lambda source, detail: manager.estop_calls.append(
        (source, detail)
    )
    logger = _RecordingLogger()
    node = SimpleNamespace(
        cm=manager,
        get_logger=lambda: logger,
    )

    ChassisNode.close(node)

    assert manager.estop_calls == [
        ("node_shutdown", "chassis node cleanup"),
    ]
    assert first.close_count == 1
    assert second.close_count == 1
    assert node.cm is None
    assert any("first close failed" in message for message in logger.errors)


def _clock_double():
    return SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=Time),
    )


def _tick_node(manager, publisher=None):
    logger = _RecordingLogger()
    return SimpleNamespace(
        cm=manager,
        _safety_required=False,
        _overrun_count=0,
        _wheel_telemetry_failed=False,
        _now_ms=lambda: 0.0,
        get_clock=_clock_double,
        get_logger=lambda: logger,
        pub_wheels=publisher or SimpleNamespace(publish=lambda _msg: None),
    ), logger


def test_tick_contains_control_and_snapshot_failures_across_calls():
    valid_snapshot = SimpleNamespace(
        chassis_mode="ESTOP",
        stop_state="ESTOP",
        healthy=True,
        wheels=(),
    )
    manager = SimpleNamespace(
        cfg=SimpleNamespace(loop_hz=50.0),
        tick_count=0,
        snapshot_count=0,
        estop_calls=[],
        snapshot_results=[
            RuntimeError("snapshot failed"),
            RuntimeError("snapshot failed"),
            valid_snapshot,
            RuntimeError("snapshot failed again"),
        ],
    )

    def tick():
        manager.tick_count += 1
        if manager.tick_count == 1:
            raise RuntimeError("tick failed")

    def snapshot():
        manager.snapshot_count += 1
        result = manager.snapshot_results.pop(0)
        if isinstance(result, Exception):
            raise result
        return result

    manager.tick = tick
    manager.snapshot = snapshot
    manager.estop = lambda source, detail: manager.estop_calls.append(
        (source, detail)
    )
    node, logger = _tick_node(manager)

    ChassisNode._tick(node)
    ChassisNode._tick(node)
    ChassisNode._tick(node)
    ChassisNode._tick(node)

    assert manager.tick_count == 4
    assert manager.snapshot_count == 4
    assert manager.estop_calls == [("control_exception", "tick failed")]
    assert logger.errors == [
        "wheel telemetry failed: snapshot failed",
        "wheel telemetry failed: snapshot failed again",
    ]
    assert logger.infos == ["wheel telemetry recovered"]


def test_tick_contains_wheel_message_conversion_failure():
    manager = SimpleNamespace(
        cfg=SimpleNamespace(loop_hz=50.0),
        tick=lambda: None,
        snapshot=lambda: SimpleNamespace(),
        estop=lambda _source, _detail: None,
    )
    node, logger = _tick_node(manager)

    ChassisNode._tick(node)

    assert any("wheel telemetry" in message for message in logger.errors)


def test_tick_contains_wheel_publish_failure():
    manager = SimpleNamespace(
        cfg=SimpleNamespace(loop_hz=50.0),
        tick=lambda: None,
        snapshot=lambda: SimpleNamespace(
            chassis_mode="ARMED",
            stop_state="RUN",
            healthy=True,
            wheels=(),
        ),
        estop=lambda _source, _detail: None,
    )
    publisher = SimpleNamespace(
        publish=lambda _msg: (_ for _ in ()).throw(
            RuntimeError("publish failed")
        )
    )
    node, logger = _tick_node(manager, publisher)

    ChassisNode._tick(node)

    assert any("publish failed" in message for message in logger.errors)


def test_publish_state_contains_manager_state_failure():
    logger = _RecordingLogger()
    node = SimpleNamespace(
        cm=SimpleNamespace(
            state=lambda: (_ for _ in ()).throw(
                RuntimeError("state failed")
            )
        ),
        get_logger=lambda: logger,
    )

    ChassisNode._publish_state(node)

    assert any("state failed" in message for message in logger.errors)


@pytest.mark.parametrize("sample_hz", [5.0, 7.5, 10.0])
def test_us100_sample_rate_accepts_supported_range(sample_hz):
    assert us100_safety_node.validate_sample_hz(sample_hz) == sample_hz


@pytest.mark.parametrize(
    "sample_hz",
    [0.0, 4.99, 10.01, math.nan, math.inf],
)
def test_us100_sample_rate_rejects_incompatible_values(sample_hz):
    with pytest.raises(ValueError, match="5.0.*10.0"):
        us100_safety_node.validate_sample_hz(sample_hz)


class _VirtualClock:
    def __init__(self):
        self.elapsed_s = 0.0

    def advance(self, duration_s):
        self.elapsed_s += duration_s


class _SlowNoResponseSerial:
    def __init__(self, clock):
        self.clock = clock
        self.read_delay_s = None

    def reset_input_buffer(self):
        pass

    def write(self, _data):
        pass

    def flush(self):
        pass

    def read(self, _expected):
        self.clock.advance(self.read_delay_s)
        return b""


def test_safety_timeout_covers_measured_worst_read_and_margin():
    clock = _VirtualClock()
    serial_port = _SlowNoResponseSerial(clock)
    sensor = Us100Sensor(
        serial_port=serial_port,
        sleeper=clock.advance,
    )
    serial_port.read_delay_s = sensor._timeout
    expected_no_response_s = 2 * (
        sensor._response_wait + sensor._timeout
    )

    reading = sensor.read()

    assert reading.status == NO_RESPONSE
    assert clock.elapsed_s == pytest.approx(expected_no_response_s)
    assert (
        chassis_node.US100_NO_RESPONSE_WORST_CASE_S
        == clock.elapsed_s
    )
    assert chassis_node.SAFETY_TOPIC_SCHEDULING_MARGIN_S >= 0.35
    assert (
        chassis_node.MIN_SAFETY_TOPIC_TIMEOUT_S
        >= clock.elapsed_s
        + chassis_node.SAFETY_TOPIC_SCHEDULING_MARGIN_S
    )
    assert (
        chassis_node.DEFAULT_SAFETY_TOPIC_TIMEOUT_S
        >= chassis_node.MIN_SAFETY_TOPIC_TIMEOUT_S
    )


@pytest.mark.parametrize("timeout_s", [0.4, 0.5, 0.749, math.nan, math.inf])
def test_safety_timeout_rejects_values_without_required_margin(timeout_s):
    with pytest.raises(ValueError, match="0.75"):
        chassis_node.validate_safety_topic_timeout(timeout_s)


def test_safety_timeout_accepts_minimum_with_explicit_margin():
    assert chassis_node.validate_safety_topic_timeout(0.75) == 0.75


def test_initial_safety_state_is_checking_when_required():
    manager = SimpleNamespace(update_calls=[])
    manager.update_external_safety = (
        lambda status, estop_required, detail: manager.update_calls.append(
            (status, estop_required, detail)
        )
    )
    node = SimpleNamespace(cm=manager, _safety_required=True)

    ChassisNode._seed_initial_safety(node)

    assert manager.update_calls == [("CHECKING", False, "startup")]
