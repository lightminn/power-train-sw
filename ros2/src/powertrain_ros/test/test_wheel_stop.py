"""Qualified per-wheel stop predicate fixtures (WP5.2 Task 4a)."""

import ast
from dataclasses import replace
from pathlib import Path

import pytest

from powertrain_ros.wheel_stop import (
    WheelStopConfig,
    WheelStopPredicate,
    WheelStopSample,
    WheelStopWheel,
    load_wheel_stop_config,
)


WHEEL_NAMES = (
    "front_left",
    "front_right",
    "mid_left",
    "mid_right",
    "rear_left",
    "rear_right",
)
THRESHOLDS = {name: 0.05 for name in WHEEL_NAMES}
CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "wheel_stop.yaml"
PACKAGE = CONFIG_PATH.parent.parent
WHEEL_STOP_MODULE = PACKAGE / "powertrain_ros" / "wheel_stop.py"
CHASSIS_NODE = PACKAGE / "powertrain_ros" / "chassis_node.py"
SETUP = PACKAGE / "setup.py"


def _wheel(name, speed=0.01, **overrides):
    values = {
        "name": name,
        "drive_turns_per_s": speed,
        "drive_stale": False,
        "steer_stale": False,
        "drive_axis_error": 0,
        "steer_fault": 0,
    }
    values.update(overrides)
    return WheelStopWheel(**values)


def _sample(stamp_s, *, wheels=None, healthy=True, authority=(0.0, 0.0)):
    return WheelStopSample(
        stamp_s=stamp_s,
        healthy=healthy,
        wheels=tuple(wheels or (_wheel(name) for name in WHEEL_NAMES)),
        authority_v=authority[0],
        authority_omega=authority[1],
    )


def _predicate(*, qualified=True, thresholds=None, dwell_ms=300):
    return WheelStopPredicate(
        WheelStopConfig(
            thresholds_rev_s=THRESHOLDS if thresholds is None else thresholds,
            dwell_ms=dwell_ms,
            qualified=qualified,
        ),
        sample_timeout_s=0.1,
    )


def test_default_yaml_is_explicitly_unqualified_until_six_wheel_hil():
    config = load_wheel_stop_config(CONFIG_PATH)

    assert config.qualified is False
    assert config.thresholds_rev_s == {}
    assert config.dwell_ms == 300

    predicate = WheelStopPredicate(config)
    assert predicate.update(_sample(0.0), now_s=0.0) is False
    assert predicate.last_reject_reason == "unqualified"


def test_qualified_six_wheel_sample_requires_continuous_dwell():
    predicate = _predicate(dwell_ms=300)

    assert predicate.update(_sample(1.00), now_s=1.00) is False
    assert predicate.last_reject_reason == "dwell_not_met"
    assert predicate.update(_sample(1.29), now_s=1.29) is False
    assert predicate.update(_sample(1.30), now_s=1.30) is True
    assert predicate.last_reject_reason == ""


def test_one_rotating_wheel_rejects_instead_of_using_median_of_six():
    wheels = [_wheel(name) for name in WHEEL_NAMES]
    wheels[4] = replace(wheels[4], drive_turns_per_s=0.051)
    predicate = _predicate(dwell_ms=0)

    assert predicate.update(_sample(2.0, wheels=wheels), now_s=2.0) is False
    assert predicate.last_reject_reason == "wheel_above_threshold:rear_left"


def _invalid_sample(case, stamp_s):
    sample = _sample(stamp_s)
    wheels = list(sample.wheels)

    if case == "stale_header":
        return replace(sample, stamp_s=stamp_s - 0.2)
    if case == "future_header":
        return replace(sample, stamp_s=stamp_s + 0.01)
    if case == "duplicate_stamp":
        return replace(sample, stamp_s=0.15)
    if case == "backward_stamp":
        return replace(sample, stamp_s=0.14)
    if case == "five_wheels":
        return replace(sample, wheels=tuple(wheels[:-1]))
    if case == "duplicate_wheel":
        wheels[-1] = replace(wheels[-1], name=wheels[0].name)
        return replace(sample, wheels=tuple(wheels))
    if case == "unknown_and_missing_wheel":
        wheels[-1] = replace(wheels[-1], name="intruder")
        return replace(sample, wheels=tuple(wheels))
    if case == "nan_speed":
        wheels[0] = replace(wheels[0], drive_turns_per_s=float("nan"))
        return replace(sample, wheels=tuple(wheels))
    if case == "infinite_speed":
        wheels[0] = replace(wheels[0], drive_turns_per_s=float("inf"))
        return replace(sample, wheels=tuple(wheels))
    if case == "unhealthy":
        return replace(sample, healthy=False)
    if case == "drive_stale":
        wheels[0] = replace(wheels[0], drive_stale=True)
        return replace(sample, wheels=tuple(wheels))
    if case == "steer_stale":
        wheels[0] = replace(wheels[0], steer_stale=True)
        return replace(sample, wheels=tuple(wheels))
    if case == "axis_error":
        wheels[0] = replace(wheels[0], drive_axis_error=1)
        return replace(sample, wheels=tuple(wheels))
    if case == "steer_fault":
        wheels[0] = replace(wheels[0], steer_fault=1)
        return replace(sample, wheels=tuple(wheels))
    if case == "authority_v_nonzero":
        return replace(sample, authority_v=0.001)
    if case == "authority_omega_nonzero":
        return replace(sample, authority_omega=-0.001)
    if case == "wheel_above_threshold":
        wheels[0] = replace(wheels[0], drive_turns_per_s=0.051)
        return replace(sample, wheels=tuple(wheels))
    raise AssertionError("unknown fixture case: %s" % case)


@pytest.mark.parametrize(
    "case",
    (
        "stale_header",
        "future_header",
        "duplicate_stamp",
        "backward_stamp",
        "five_wheels",
        "duplicate_wheel",
        "unknown_and_missing_wheel",
        "nan_speed",
        "infinite_speed",
        "unhealthy",
        "drive_stale",
        "steer_stale",
        "axis_error",
        "steer_fault",
        "authority_v_nonzero",
        "authority_omega_nonzero",
        "wheel_above_threshold",
    ),
)
def test_every_bad_sample_type_resets_dwell(case):
    predicate = _predicate(dwell_ms=300)
    assert predicate.update(_sample(0.00), now_s=0.00) is False
    assert predicate.update(_sample(0.15), now_s=0.15) is False

    bad = _invalid_sample(case, stamp_s=0.20)
    assert predicate.update(bad, now_s=0.20) is False
    assert predicate.last_reject_reason not in ("", "dwell_not_met")

    assert predicate.update(_sample(0.21), now_s=0.21) is False
    assert predicate.update(_sample(0.50), now_s=0.50) is False
    assert predicate.update(_sample(0.51), now_s=0.51) is True


def test_clock_rollback_resets_stamp_history_and_dwell():
    predicate = _predicate(dwell_ms=300)
    assert predicate.update(_sample(10.00), now_s=10.00) is False
    assert predicate.update(_sample(10.15), now_s=10.15) is False

    assert predicate.update(_sample(1.00), now_s=1.00) is False
    assert predicate.last_reject_reason == "clock_not_monotonic"
    assert predicate.update(_sample(1.01), now_s=1.01) is False
    assert predicate.update(_sample(1.31), now_s=1.31) is True


@pytest.mark.parametrize(
    "thresholds",
    (
        {name: 0.05 for name in WHEEL_NAMES[:-1]},
        {**THRESHOLDS, "seventh": 0.05},
        {**THRESHOLDS, "front_left": 0.0},
    ),
)
def test_qualified_flag_cannot_override_invalid_threshold_map(thresholds):
    predicate = _predicate(qualified=True, thresholds=thresholds, dwell_ms=0)

    assert predicate.qualified is False
    assert predicate.update(_sample(0.0), now_s=0.0) is False
    assert predicate.last_reject_reason == "unqualified_threshold_map"


def test_wheel_stop_core_has_no_ros_or_message_imports():
    tree = ast.parse(WHEEL_STOP_MODULE.read_text(encoding="utf-8"))
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".")[0])

    assert "rclpy" not in imported_roots
    assert "powertrain_msgs" not in imported_roots


def test_chassis_node_loads_and_injects_wheel_stop_only_for_authority_path():
    source = CHASSIS_NODE.read_text(encoding="utf-8")

    assert '"wheel_stop_config"' in source
    assert "declare_parameter(" in source
    assert 'get_package_share_directory("powertrain_ros")' in source
    assert "load_wheel_stop_config" in source
    assert "WheelStopPredicate" in source
    assert '"/wheel_states"' in source
    assert "self._on_wheel_states_for_stop" in source
    assert "wheel_stopped=lambda: self._wheel_stop.confirmed" in source
    assert "wheel_stop_qualified=lambda: self._wheel_stop.qualified" in source

    initialize = ast.parse(source)
    cls = next(
        node
        for node in initialize.body
        if isinstance(node, ast.ClassDef) and node.name == "ChassisNode"
    )
    method = next(
        node
        for node in cls.body
        if isinstance(node, ast.FunctionDef) and node.name == "_initialize"
    )
    authority_guard = next(
        node
        for node in ast.walk(method)
        if isinstance(node, ast.If)
        and ast.unparse(node.test) == "self._authority_enabled"
    )
    enabled = ast.unparse(ast.Module(body=authority_guard.body, type_ignores=[]))
    disabled = ast.unparse(ast.Module(body=authority_guard.orelse, type_ignores=[]))
    assert "WheelStopPredicate" in enabled
    assert "_on_wheel_states_for_stop" in enabled
    assert "WheelStopPredicate" not in disabled
    assert "_on_wheel_states_for_stop" not in disabled


def test_chassis_adapter_passes_primitive_wheel_and_authority_fields():
    source = CHASSIS_NODE.read_text(encoding="utf-8")

    assert "WheelStopSample(" in source
    assert "WheelStopWheel(" in source
    for field in (
        "drive_turns_per_s",
        "drive_stale",
        "steer_stale",
        "drive_axis_error",
        "steer_fault",
        "healthy",
        "authority_v",
        "authority_omega",
    ):
        assert field in source


def test_setup_installs_wheel_stop_yaml():
    source = SETUP.read_text(encoding="utf-8")
    assert '"config/wheel_stop.yaml"' in source
