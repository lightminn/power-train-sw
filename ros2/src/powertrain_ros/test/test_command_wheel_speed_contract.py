"""ROS-free checks for the additive commanded wheel-speed contract."""

import ast
from pathlib import Path
from types import SimpleNamespace

from chassis.kinematics import default_geometry
from powertrain_ros.message_adapter import fill_wheel_states_message
from powertrain_ros.state_estimation import (
    StateEstimator,
    WheelSample,
    WheelValue,
)


REPO_ROOT = Path(__file__).resolve().parents[4]
WHEEL_STATE_MSG = REPO_ROOT / "ros2/src/powertrain_msgs/msg/WheelState.msg"
ODOMETRY_NODE = REPO_ROOT / "ros2/src/powertrain_ros/powertrain_ros/odometry_node.py"
WHEEL_NAMES = (
    "front_left",
    "front_right",
    "mid_left",
    "mid_right",
    "rear_left",
    "rear_right",
)


def _source_wheel(name, *, command=1.0, measured=0.0):
    return SimpleNamespace(
        name=name,
        corner_mode="ARMED",
        command_turns_per_s=command,
        drive_turns_per_s=measured,
        steer_deg=0.0,
        drive_current_a=0.0,
        steer_current_a=0.0,
        drive_stale=False,
        steer_stale=False,
        drive_axis_error=0,
        steer_fault=0,
    )


def _wheel_message(snapshot):
    message = SimpleNamespace(header=SimpleNamespace())
    return fill_wheel_states_message(
        message,
        snapshot,
        stamp="stamp",
        tick_duration_ms=1.0,
        overrun_count=0,
        wheel_factory=SimpleNamespace,
    )


def test_wheel_state_message_declares_additive_command_field():
    declarations = {
        line.strip()
        for line in WHEEL_STATE_MSG.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    }

    assert "float32 command_turns_per_s" in declarations


def test_message_adapter_maps_commanded_turns_per_second():
    snapshot = SimpleNamespace(
        chassis_mode="ARMED",
        stop_state="RUN",
        healthy=True,
        wheels=(_source_wheel("front_left", command=1.25, measured=0.5),),
    )

    message = _wheel_message(snapshot)

    assert message.wheels[0].command_turns_per_s == 1.25
    assert message.wheels[0].drive_turns_per_s == 0.5


def test_odometry_seam_turns_stalled_command_chain_into_slip_and_stuck():
    tree = ast.parse(ODOMETRY_NODE.read_text(encoding="utf-8"))
    getattr_fields = {
        node.args[1].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "getattr"
        and len(node.args) >= 2
        and isinstance(node.args[1], ast.Constant)
        and isinstance(node.args[1].value, str)
    }
    assert "command_turns_per_s" in getattr_fields

    snapshot = SimpleNamespace(
        chassis_mode="ARMED",
        stop_state="RUN",
        healthy=True,
        wheels=tuple(_source_wheel(name) for name in WHEEL_NAMES),
    )
    message = _wheel_message(snapshot)
    estimator = StateEstimator(default_geometry())
    decision = estimator.update_wheels(
        WheelSample(
            stamp_s=1.0,
            wheels=tuple(
                WheelValue(
                    name=wheel.name,
                    command_turns_per_s=wheel.command_turns_per_s,
                    measured_turns_per_s=wheel.drive_turns_per_s,
                    steer_deg=wheel.steer_deg,
                    stale=wheel.drive_stale or wheel.steer_stale,
                )
                for wheel in message.wheels
            ),
        ),
        now_s=1.0,
    )
    diagnostics = estimator.snapshot(now_s=1.0).diagnostics

    assert decision.accepted
    assert diagnostics.slip_candidate
    assert diagnostics.stuck_candidate
