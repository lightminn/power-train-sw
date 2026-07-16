import uuid
from pathlib import Path

import pytest

from powertrain_ros.remote_input import DPad, NormalizedAxes, RemoteInputFrame
from powertrain_ros.remote_input_gateway import (
    ARM,
    DISCONNECTED,
    DRIVE,
    MOTION_HOLD,
    STOPPING_FOR_ARM,
    STOPPING_FOR_DRIVE,
    ArmOutput,
    GatewayConfig,
    RemoteInputGateway,
    gated_arm_output,
)


def _frame(
    *,
    sequence=0,
    received_s=0.0,
    mode="DRIVE",
    deadman=False,
    left_x=0.0,
    right_y=0.0,
    left_trigger=0.0,
    right_trigger=0.0,
    dpad_x=0,
    mode_chord=False,
    estop_edge=False,
    assist_bypass=False,
    session_id=None,
):
    return RemoteInputFrame(
        schema_version=2,
        session_id=session_id or str(uuid.uuid4()),
        sequence=sequence,
        client_monotonic_ns=0,
        mode=mode,
        deadman=deadman,
        axes=NormalizedAxes(
            left_x=left_x,
            right_y=right_y,
            left_trigger=left_trigger,
            right_trigger=right_trigger,
        ),
        dpad=DPad(x=dpad_x, y=0),
        mode_chord=mode_chord,
        estop_edge=estop_edge,
        assist_bypass=assist_bypass,
        received_monotonic_s=received_s,
        input_timeout_s=0.20,
    )


def _assert_zero(output):
    assert output.drive.linear == 0.0
    assert output.drive.angular == 0.0
    assert output.arm.joint_velocity == 0.0
    assert output.arm.gripper == 0.0


def _assert_exclusive(output):
    drive_nonzero = output.drive.linear != 0.0 or output.drive.angular != 0.0
    arm_nonzero = (
        output.arm.joint_velocity != 0.0 or output.arm.gripper != 0.0
    )
    assert not (drive_nonzero and arm_nonzero)


def _connect_drive(gateway, session_id=None):
    session_id = session_id or str(uuid.uuid4())
    gateway.begin_connection()
    gateway.submit(_frame(session_id=session_id))
    output = gateway.tick(0.0)
    assert output.state == DRIVE
    _assert_zero(output)
    return session_id


def test_30hz_drive_stops_on_stale_and_hold_clear_never_restores_command():
    gateway = RemoteInputGateway()
    session_id = _connect_drive(gateway)

    for sequence in range(1, 7):
        now_s = sequence / 30.0
        gateway.submit(
            _frame(
                sequence=sequence,
                received_s=now_s,
                deadman=True,
                right_trigger=0.7,
                left_x=-0.2,
                session_id=session_id,
            )
        )
        output = gateway.tick(now_s)
        assert output.drive.linear == pytest.approx(0.7)
        assert output.drive.angular == pytest.approx(-0.2)
        _assert_exclusive(output)

    output = gateway.tick(now_s + 0.200001)
    assert output.state == MOTION_HOLD
    _assert_zero(output)
    assert gateway.clear_hold()
    assert gateway.state == DISCONNECTED
    _assert_zero(gateway.tick(now_s + 0.21))


def test_assist_bypass_is_unmodified_only_while_input_is_fresh():
    gateway = RemoteInputGateway()
    session_id = _connect_drive(gateway)
    gateway.submit(
        _frame(
            sequence=1,
            received_s=0.01,
            deadman=True,
            right_trigger=0.4,
            assist_bypass=True,
            session_id=session_id,
        )
    )

    fresh = gateway.tick(0.01)
    assert fresh.input_fresh is True
    assert fresh.assist_bypass is True

    stale = gateway.tick(0.210001)
    assert stale.input_fresh is False
    assert stale.assist_bypass is False


def test_deadman_release_and_estop_edge_zero_on_the_next_tick():
    gateway = RemoteInputGateway()
    session_id = _connect_drive(gateway)
    gateway.submit(
        _frame(
            sequence=1,
            received_s=0.01,
            deadman=True,
            right_trigger=0.5,
            session_id=session_id,
        )
    )
    assert gateway.tick(0.01).drive.linear == 0.5

    gateway.submit(
        _frame(sequence=2, received_s=0.02, session_id=session_id)
    )
    released = gateway.tick(0.02)
    assert released.state == DRIVE
    _assert_zero(released)

    gateway.submit(
        _frame(
            sequence=3,
            received_s=0.03,
            estop_edge=True,
            session_id=session_id,
        )
    )
    estop = gateway.tick(0.03)
    assert estop.state == MOTION_HOLD
    _assert_zero(estop)


def test_reconnect_requires_neutral_and_never_restores_mode_or_nonzero():
    gateway = RemoteInputGateway()
    old_session = _connect_drive(gateway)
    gateway.submit(
        _frame(
            sequence=1,
            received_s=0.01,
            deadman=True,
            right_trigger=0.8,
            session_id=old_session,
        )
    )
    assert gateway.tick(0.01).drive.linear == 0.8
    gateway.end_connection()
    assert gateway.state == DISCONNECTED

    gateway.begin_connection()
    new_session = str(uuid.uuid4())
    gateway.submit(
        _frame(
            received_s=0.02,
            deadman=True,
            right_trigger=0.8,
            mode="ARM",
            mode_chord=True,
            session_id=new_session,
        )
    )
    output = gateway.tick(0.02)
    assert output.state == DISCONNECTED
    _assert_zero(output)

    gateway.submit(
        _frame(sequence=1, received_s=0.03, session_id=new_session)
    )
    output = gateway.tick(0.03)
    assert output.state == DRIVE
    _assert_zero(output)


def test_arm_request_is_rejected_by_default_enable_gate():
    gateway = RemoteInputGateway()
    session_id = _connect_drive(gateway)
    gateway.submit(
        _frame(
            sequence=1,
            received_s=0.01,
            mode="ARM",
            mode_chord=True,
            session_id=session_id,
        )
    )
    output = gateway.tick(0.01)
    assert output.state == DRIVE
    assert "disabled" in output.reason
    _assert_zero(output)


def test_drive_to_arm_waits_for_qualified_wheel_and_stationary_ack():
    stop = {"qualified": True, "confirmed": False, "arm_ack": False}
    gateway = RemoteInputGateway(
        arm_output_enabled=True,
        wheel_stop_qualified=lambda: stop["qualified"],
        wheel_stopped=lambda: stop["confirmed"],
        arm_stationary_ack=lambda: stop["arm_ack"],
    )
    session_id = _connect_drive(gateway)
    gateway.submit(
        _frame(
            sequence=1,
            received_s=0.01,
            mode="ARM",
            mode_chord=True,
            session_id=session_id,
        )
    )
    output = gateway.tick(0.01)
    assert output.state == STOPPING_FOR_ARM
    _assert_zero(output)

    stop["confirmed"] = True
    output = gateway.tick(0.02)
    assert output.state == STOPPING_FOR_ARM
    _assert_zero(output)

    stop["arm_ack"] = True
    output = gateway.tick(0.03)
    assert output.state == ARM
    _assert_zero(output)

    gateway.submit(
        _frame(
            sequence=2,
            received_s=0.04,
            mode="ARM",
            deadman=True,
            right_y=-0.4,
            session_id=session_id,
        )
    )
    output = gateway.tick(0.04)
    assert output.arm.joint_name == "joint_1"
    assert output.arm.joint_velocity == pytest.approx(-0.4)
    assert output.drive.linear == 0.0
    _assert_exclusive(output)


def test_unqualified_wheel_stop_rejects_transition_without_stopping_state():
    gateway = RemoteInputGateway(
        arm_output_enabled=True,
        wheel_stop_qualified=lambda: False,
    )
    session_id = _connect_drive(gateway)
    gateway.submit(
        _frame(
            sequence=1,
            received_s=0.01,
            mode="ARM",
            mode_chord=True,
            session_id=session_id,
        )
    )
    output = gateway.tick(0.01)
    assert output.state == DRIVE
    assert "unqualified" in output.reason
    _assert_zero(output)


def test_stopping_timeout_enters_hold_and_requires_explicit_clear():
    gateway = RemoteInputGateway(
        GatewayConfig(stopping_timeout_s=0.10),
        arm_output_enabled=True,
        wheel_stop_qualified=lambda: True,
        wheel_stopped=lambda: False,
    )
    session_id = _connect_drive(gateway)
    gateway.submit(
        _frame(
            sequence=1,
            received_s=0.01,
            mode="ARM",
            mode_chord=True,
            session_id=session_id,
        )
    )
    assert gateway.tick(0.01).state == STOPPING_FOR_ARM
    output = gateway.tick(0.11)
    assert output.state == MOTION_HOLD
    assert "timeout" in output.reason
    _assert_zero(output)

    gateway.submit(
        _frame(
            sequence=2,
            received_s=0.12,
            mode="ARM",
            mode_chord=True,
            session_id=session_id,
        )
    )
    assert gateway.tick(0.12).state == MOTION_HOLD
    assert gateway.clear_hold()


def test_arm_to_drive_zeros_arm_then_waits_for_stow_confirmation():
    gate = {"stopped": True, "arm_ack": True, "stowed": False}
    gateway = RemoteInputGateway(
        arm_output_enabled=True,
        wheel_stop_qualified=lambda: True,
        wheel_stopped=lambda: gate["stopped"],
        arm_stationary_ack=lambda: gate["arm_ack"],
        stow_confirmed=lambda: gate["stowed"],
    )
    session_id = _connect_drive(gateway)
    gateway.submit(
        _frame(
            sequence=1,
            received_s=0.01,
            mode="ARM",
            mode_chord=True,
            session_id=session_id,
        )
    )
    assert gateway.tick(0.01).state == STOPPING_FOR_ARM
    assert gateway.tick(0.02).state == ARM
    gateway.submit(
        _frame(
            sequence=2,
            received_s=0.03,
            mode="ARM",
            deadman=True,
            right_y=0.6,
            right_trigger=0.4,
            session_id=session_id,
        )
    )
    arm_output = gateway.tick(0.03)
    assert arm_output.arm.joint_velocity == 0.6
    assert arm_output.arm.gripper == 0.4

    gateway.submit(
        _frame(
            sequence=3,
            received_s=0.04,
            mode="DRIVE",
            mode_chord=True,
            session_id=session_id,
        )
    )
    output = gateway.tick(0.04)
    assert output.state == STOPPING_FOR_DRIVE
    _assert_zero(output)
    output = gateway.tick(0.05)
    assert output.state == STOPPING_FOR_DRIVE
    _assert_zero(output)

    gate["stowed"] = True
    output = gateway.tick(0.06)
    assert output.state == DRIVE
    _assert_zero(output)


def test_arm_trigger_conflict_holds_all_arm_outputs_and_dpad_selects_joint():
    gateway = RemoteInputGateway(
        arm_output_enabled=True,
        wheel_stop_qualified=lambda: True,
        wheel_stopped=lambda: True,
        arm_stationary_ack=lambda: True,
    )
    session_id = _connect_drive(gateway)
    gateway.submit(
        _frame(
            sequence=1,
            received_s=0.01,
            mode="ARM",
            mode_chord=True,
            session_id=session_id,
        )
    )
    assert gateway.tick(0.01).state == STOPPING_FOR_ARM
    assert gateway.tick(0.02).state == ARM

    gateway.submit(
        _frame(
            sequence=2,
            received_s=0.03,
            mode="ARM",
            deadman=True,
            right_y=0.5,
            left_trigger=0.5,
            right_trigger=0.5,
            dpad_x=1,
            session_id=session_id,
        )
    )
    output = gateway.tick(0.03)
    assert output.arm.joint_name == "joint_2"
    _assert_zero(output)
    _assert_exclusive(output)


def test_contract_violation_forces_hold_and_zero_outputs():
    gateway = RemoteInputGateway()
    _connect_drive(gateway)
    gateway.contract_violation("CONTRACT_VIOLATION: malformed JSON")
    output = gateway.tick(0.01)
    assert output.state == MOTION_HOLD
    assert "CONTRACT_VIOLATION" in output.reason
    _assert_zero(output)


def test_ros_arm_publish_gate_is_zero_by_default():
    requested = ArmOutput(
        joint_name="joint_4",
        joint_velocity=0.7,
        gripper=-0.3,
    )
    blocked = gated_arm_output(requested)
    assert blocked.joint_name == "joint_4"
    assert blocked.joint_velocity == 0.0
    assert blocked.gripper == 0.0
    assert gated_arm_output(requested, enabled=True) == requested


def test_ros_wrapper_exposes_explicit_hold_ack_and_never_imports_pygame():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "powertrain_ros"
        / "teleop_command_node.py"
    )
    source = source_path.read_text(encoding="utf-8")
    assert '"~/clear_hold"' in source
    assert '"/teleop/assist_bypass"' in source
    assert "ARM_OUTPUT_ENABLED = False" in source
    assert "import pygame" not in source
    tick_source = source[source.index("    def _tick(self):") : source.index(
        "    def close(self):"
    )]
    assert tick_source.index("if not output.input_fresh:") < tick_source.index(
        "self._publish_assist_bypass(output)"
    )
