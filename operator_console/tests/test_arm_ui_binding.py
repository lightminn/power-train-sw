import time

from operator_console.arm_telemetry import parse_arm_telemetry
from operator_console.arm_ui import contracts as C
from operator_console.arm_ui_binding import ArmUiTelemetryBinding
from powertrain_ros.arm_console_mirror import build_arm_telemetry_payload


def _snapshot(*, kind="spur_1motor_gripper", ids=(5,), sequence=1, age=0.0):
    return parse_arm_telemetry(build_arm_telemetry_payload(
        sequence=sequence, stamp_s=0, motors=None,
        joints={"names": ["joint_1"], "position_rad": [0.5], "velocity": []},
        source_age_s={"joints": age},
        tool_runtime={"tool": {"tool_type": kind, "actuator_ids": list(ids),
                      "actuators": [{"id": item, "online": True} for item in ids]},
                      "source_age_s": age},
        arm_runtime={"control_mode": "MANUAL", "fsm_state": "IDLE",
                     "arm_status": "READY", "source_age_s": {
                         "control_mode": age, "fsm_state": age, "arm_status": age}},
    ), received_monotonic_s=time.monotonic())


def test_projection_reuses_current_tool_only_and_exposes_no_commands():
    binding = ArmUiTelemetryBinding()
    state = binding.state(_snapshot())

    assert state.link.state == C.LINK_LIVE
    assert state.detected_tool.kind == C.TOOL_SINGLE_GRIPPER
    assert state.detected_tool.actuator_ids == (5,)
    assert state.teleop.axes[0].name == "joint_1"
    assert state.arm_fsm.raw == "IDLE"
    assert state.capabilities == frozenset()
    assert state.diagnostics.motors[0].actuator_id == 5


def test_tool_generation_changes_only_when_detected_tool_changes():
    binding = ArmUiTelemetryBinding()
    first = binding.state(_snapshot(sequence=1))
    repeated = binding.state(_snapshot(sequence=2))
    changed = binding.state(_snapshot(kind="dual_motor_gripper", ids=(3, 4), sequence=3))

    assert repeated.detected_tool.generation == first.detected_tool.generation
    assert changed.detected_tool.generation == first.detected_tool.generation + 1
    assert [motor.actuator_id for motor in changed.diagnostics.motors] == [3, 4]


def test_hardware_fault_is_exposed_as_the_operator_block_reason():
    snapshot = _snapshot()
    snapshot.tool_runtime["tool"].update({
        "hardware_error": 32,
        "motion_allowed": False,
    })

    state = ArmUiTelemetryBinding().state(snapshot, ops_link_ready=True)

    assert state.block_reasons[0].code == "tool_hardware_error"
    assert "32" in state.block_reasons[0].korean
