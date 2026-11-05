import ast
from contextlib import nullcontext
import json
from types import SimpleNamespace
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
    input_epoch=None,
    estop_id=0,
):
    return RemoteInputFrame(
        schema_version=2 if input_epoch is None else 3,
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
        input_epoch=input_epoch,
        estop_id=estop_id,
    )


def _assert_zero(output):
    assert output.drive.linear == 0.0
    assert output.drive.angular == 0.0
    assert output.drive.steering == 0.0
    assert output.arm.joint_velocity == 0.0
    assert output.arm.gripper == 0.0


def _assert_exclusive(output):
    drive_nonzero = any((output.drive.linear, output.drive.angular, output.drive.steering))
    arm_nonzero = (
        output.arm.joint_velocity != 0.0 or output.arm.gripper != 0.0
    )
    assert not (drive_nonzero and arm_nonzero)


def _connect_drive(gateway, session_id=None, at_s=0.0):
    session_id = session_id or str(uuid.uuid4())
    gateway.begin_connection()
    gateway.submit(_frame(session_id=session_id, received_s=at_s))
    output = gateway.tick(at_s)
    assert output.state == DRIVE
    _assert_zero(output)
    return session_id


@pytest.mark.parametrize(
    "left_x,right_trigger,left_trigger,linear,angular,front_sign",
    [
        # 2026-11-06: RT moves toward the camera, which is software -v (see gateway).
        (0.25, 0.5, 0.0, -0.75, -0.3, 1),
        (-0.25, 0.5, 0.0, -0.75, 0.3, -1),
        (0.25, 0.0, 0.5, 0.75, -0.3, -1),
        (-0.25, 0.0, 0.5, 0.75, 0.3, 1),
    ],
)
def test_drive_stick_direction_uses_rep103_before_four_wheel_steering(
    left_x, right_trigger, left_trigger, linear, angular, front_sign,
):
    from chassis.kinematics import default_geometry, solve

    gateway = RemoteInputGateway(GatewayConfig(
        max_linear=1.5, max_angular=1.2, manual_command_format="twist"))
    session_id = _connect_drive(gateway)
    gateway.submit(_frame(
        sequence=1, received_s=0.01, session_id=session_id, deadman=True,
        left_x=left_x, right_trigger=right_trigger, left_trigger=left_trigger,
    ))
    output = gateway.tick(0.01)
    assert output.drive.linear == pytest.approx(linear)
    # SDL stick-right is positive; REP-103 yaw-right is negative.
    assert output.drive.angular == pytest.approx(angular)
    assert output.drive.steering == 0.0
    wheels = solve(default_geometry(), output.drive.linear, output.drive.angular).wheels
    for name in ("front_left", "front_right"):
        assert wheels[name].steer_deg * front_sign > 0
    for name in ("rear_left", "rear_right"):
        assert wheels[name].steer_deg * front_sign < 0


@pytest.mark.parametrize("rt,lt,speed", [(0.0, 0.0, 0.0), (0.5, 0.0, -0.75), (0.0, 0.5, 0.75)])
@pytest.mark.parametrize("left_x,steering", [(0.6, -0.6), (-0.6, 0.6)])
def test_default_manual_steering_is_unchanged_at_stop_forward_and_reverse(rt, lt, speed, left_x, steering):
    gateway = RemoteInputGateway(GatewayConfig(max_linear=1.5))
    session_id = _connect_drive(gateway)
    gateway.submit(_frame(
        sequence=1, received_s=0.01, session_id=session_id, deadman=True,
        left_x=left_x, right_trigger=rt, left_trigger=lt,
    ))
    output = gateway.tick(0.01)
    assert output.drive.linear == pytest.approx(speed)
    assert output.drive.steering == pytest.approx(steering)
    assert output.drive.angular == 0.0


def test_unknown_manual_format_fails_instead_of_falling_back_to_twist():
    with pytest.raises(ValueError, match="manual_command_format"):
        RemoteInputGateway(GatewayConfig(manual_command_format="steerng"))


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
                left_trigger=0.7,
                left_x=-0.2,
                session_id=session_id,
            )
        )
        output = gateway.tick(now_s)
        assert output.drive.linear == pytest.approx(0.7)
        assert output.drive.angular == 0.0
        assert output.drive.steering == pytest.approx(0.2)
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
            left_trigger=0.4,
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
            left_trigger=0.5,
            left_x=0.5,
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
            left_trigger=0.8,
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
            left_trigger=0.8,
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


def test_ros_wrapper_publishes_one_zero_on_each_fresh_to_stale_edge():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "powertrain_ros"
        / "teleop_command_node.py"
    )
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    node_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "TeleopCommandNode"
    )
    tick = next(
        node
        for node in node_class.body
        if isinstance(node, ast.FunctionDef) and node.name == "_tick"
    )
    namespace = {
        "time": SimpleNamespace(monotonic=lambda: 10.0),
        "make_status_line": lambda _output: "status",
        "String": SimpleNamespace,
        "json": json,
    }
    module = ast.Module(body=[tick], type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(source_path), "exec"), namespace)

    def output(input_fresh, value):
        return SimpleNamespace(
            state="DRIVE",
            input_fresh=input_fresh,
            drive=SimpleNamespace(linear=value, angular=value, source_received_s=9.9 if input_fresh else None, connection_session_id="server-1"),
            arm=SimpleNamespace(joint_velocity=value, gripper=value),
        )

    fresh_one = output(True, 0.4)
    stale_one = output(False, 0.0)
    stale_two = output(False, 0.0)
    fresh_two = output(True, 0.2)
    stale_three = output(False, 0.0)
    outputs = iter((fresh_one, stale_one, stale_two, fresh_two, stale_three))
    published = {"drive": [], "arm": [], "bypass": [], "gateway": []}
    node = SimpleNamespace(
        _drain_events=lambda: None,
        _gateway=SimpleNamespace(tick=lambda _now: next(outputs)),
        _status_lock=nullcontext(),
        _input_was_fresh=False,
        _estop_event=None,          # A1: ○ E-stop 재발행 상태 (없으면 _tick AttributeError)
        _last_frame=None,
        _publish_drive=published["drive"].append,
        _publish_arm=published["arm"].append,
        _publish_assist_bypass=published["bypass"].append,
        pub_gateway_state=SimpleNamespace(
            publish=published["gateway"].append,
        ),
    )

    for _ in range(5):
        namespace["_tick"](node)

    assert published["drive"] == [
        fresh_one,
        stale_one,
        fresh_two,
        stale_three,
    ]
    assert published["arm"] == published["drive"]
    assert published["bypass"] == [fresh_one, fresh_two]


def test_field_mode_stale_input_zeroes_without_latching_and_resumes():
    gateway = RemoteInputGateway(
        GatewayConfig(input_timeout_s=0.5, latch_stale_input=False))
    session_id = _connect_drive(gateway, at_s=0.98)
    gateway.submit(_frame(sequence=1, received_s=1.0, deadman=True,
                          left_trigger=0.7, session_id=session_id))
    assert gateway.tick(1.0).drive.linear == pytest.approx(0.7)

    stale = gateway.tick(1.500001)
    assert stale.state == DRIVE
    _assert_zero(stale)

    gateway.submit(_frame(sequence=2, received_s=1.6, deadman=True,
                          left_trigger=0.7, session_id=session_id,
                          input_epoch=gateway.input_epoch))
    assert gateway.tick(1.6).drive.linear == pytest.approx(0.7)


def test_field_mode_still_latches_the_remote_estop_edge():
    gateway = RemoteInputGateway(GatewayConfig(latch_stale_input=False))
    session_id = _connect_drive(gateway)
    gateway.submit(_frame(sequence=1, received_s=1.0, estop_edge=True,
                          session_id=session_id))
    assert gateway.tick(1.0).state == MOTION_HOLD
    _assert_zero(gateway.tick(1.01))


def test_ros_wrapper_field_mode_keeps_zero_flowing_while_stale_but_not_in_hold():
    source_path = (
        Path(__file__).resolve().parents[1]
        / "powertrain_ros"
        / "teleop_command_node.py"
    )
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    node_class = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef) and node.name == "TeleopCommandNode")
    tick = next(node for node in node_class.body
                if isinstance(node, ast.FunctionDef) and node.name == "_tick")
    namespace = {
        "time": SimpleNamespace(monotonic=lambda: 10.0),
        "make_status_line": lambda _output: "status",
        "String": SimpleNamespace,
        "json": json,
        "MOTION_HOLD": MOTION_HOLD,
    }
    module = ast.Module(body=[tick], type_ignores=[])
    ast.fix_missing_locations(module)
    exec(compile(module, str(source_path), "exec"), namespace)

    def output(state, input_fresh, value):
        return SimpleNamespace(
            state=state, input_fresh=input_fresh,
            drive=SimpleNamespace(linear=value, angular=value, source_received_s=None,
                                  connection_session_id="server-1"),
            arm=SimpleNamespace(joint_velocity=value, gripper=value))

    fresh = output(DRIVE, True, 0.4)
    stale_edge = output(DRIVE, False, 0.0)
    stale_gap = output(DRIVE, False, 0.0)
    disconnected = output(DISCONNECTED, False, 0.0)
    held = output(MOTION_HOLD, False, 0.0)
    outputs = iter((fresh, stale_edge, stale_gap, disconnected, held))
    published = {"drive": [], "arm": [], "bypass": [], "gateway": []}
    node = SimpleNamespace(
        _drain_events=lambda: None,
        _gateway=SimpleNamespace(tick=lambda _now: next(outputs)),
        _status_lock=nullcontext(), _input_was_fresh=False, _estop_event=None,
        _last_frame=None, _latch_stale_input=False,
        _publish_drive=published["drive"].append,
        _publish_arm=published["arm"].append,
        _publish_assist_bypass=published["bypass"].append,
        pub_gateway_state=SimpleNamespace(publish=published["gateway"].append),
    )
    for _ in range(5):
        namespace["_tick"](node)

    assert published["drive"] == [fresh, stale_edge, stale_gap, disconnected]
    assert published["arm"] == [fresh, stale_edge]


def _field_gateway():
    return RemoteInputGateway(
        GatewayConfig(input_timeout_s=0.5, latch_stale_input=False))


def _drive(sequence, received_s, session_id, **kw):
    return _frame(sequence=sequence, received_s=received_s, deadman=True,
                  left_trigger=0.7, session_id=session_id, **kw)


def test_field_mode_late_pre_stall_record_never_restarts_driving():
    gateway = _field_gateway()
    session_id = _connect_drive(gateway, at_s=0.98)
    gateway.submit(_drive(1, 1.0, session_id, input_epoch=0))
    assert gateway.tick(1.0).drive.linear == pytest.approx(0.7)
    _assert_zero(gateway.tick(1.500001))
    assert gateway.input_epoch == 1
    # A forward record created before the stall completes late: fresh by
    # receive time, but it carries the pre-stall epoch.
    gateway.submit(_drive(2, 1.6, session_id, input_epoch=0))
    _assert_zero(gateway.tick(1.6))
    # Schema 2 records cannot prove they were made after the stall either.
    gateway.submit(_drive(3, 1.62, session_id))
    _assert_zero(gateway.tick(1.62))
    gateway.submit(_drive(4, 1.64, session_id, input_epoch=1))
    assert gateway.tick(1.64).drive.linear == pytest.approx(0.7)


def test_field_mode_post_stall_epoch_expires_by_server_clock():
    gateway = _field_gateway()
    session_id = _connect_drive(gateway, at_s=0.98)
    gateway.submit(_drive(1, 1.0, session_id, input_epoch=0))
    gateway.tick(1.0)
    gateway.tick(1.500001)
    assert gateway.input_epoch == 1
    gateway.tick(1.76)  # still recovering: re-issue so the client gets a fresh one
    assert gateway.input_epoch == 2
    # Made right after epoch 1 was seen, then delayed again past the timeout.
    gateway.submit(_drive(2, 2.05, session_id, input_epoch=1))
    _assert_zero(gateway.tick(2.05))
    gateway.submit(_drive(3, 2.06, session_id, input_epoch=2))
    assert gateway.tick(2.06).drive.linear == pytest.approx(0.7)


def test_field_mode_receive_gap_enters_recovery_even_if_no_tick_saw_it():
    gateway = _field_gateway()
    session_id = _connect_drive(gateway, at_s=0.98)
    gateway.submit(_drive(1, 1.0, session_id, input_epoch=0))
    gateway.tick(1.0)
    gateway.tick(1.49)
    gateway.submit(_drive(2, 1.52, session_id, input_epoch=0))
    output = gateway.tick(1.52)
    _assert_zero(output)
    assert output.state == DRIVE
    assert gateway.input_epoch == 1


def test_latch_mode_receive_gap_latches_hold_even_if_no_tick_saw_it():
    gateway = RemoteInputGateway(GatewayConfig(input_timeout_s=0.5))
    session_id = _connect_drive(gateway, at_s=0.98)
    gateway.submit(_drive(1, 1.0, session_id))
    gateway.tick(1.0)
    gateway.tick(1.49)
    gateway.submit(_drive(2, 1.52, session_id))
    output = gateway.tick(1.52)
    assert output.state == MOTION_HOLD
    _assert_zero(output)


@pytest.mark.parametrize("latch", [True, False])
def test_hold_clear_requires_input_made_after_the_clear(latch):
    gateway = RemoteInputGateway(
        GatewayConfig(input_timeout_s=0.5, latch_stale_input=latch))
    session_id = _connect_drive(gateway, at_s=0.98)
    gateway.submit(_frame(sequence=1, received_s=1.0, estop_edge=True,
                          estop_id=7, input_epoch=0, session_id=session_id))
    assert gateway.tick(1.0).state == MOTION_HOLD
    assert gateway.clear_hold()
    gateway.tick(1.01)
    required = gateway.input_epoch
    assert required >= 1
    # Buffered pre-clear neutral then forward must not re-arm and drive.
    gateway.submit(_frame(sequence=2, received_s=1.02, input_epoch=0,
                          session_id=session_id))
    _assert_zero(gateway.tick(1.02))
    gateway.submit(_drive(3, 1.03, session_id, input_epoch=0))
    _assert_zero(gateway.tick(1.03))
    gateway.submit(_frame(sequence=4, received_s=1.04, input_epoch=required,
                          session_id=session_id))
    assert gateway.tick(1.04).state == DRIVE
    gateway.submit(_drive(5, 1.05, session_id, input_epoch=required))
    assert gateway.tick(1.05).drive.linear == pytest.approx(0.7)


def test_estop_edge_in_a_pre_stall_record_still_latches():
    gateway = _field_gateway()
    session_id = _connect_drive(gateway, at_s=0.98)
    gateway.submit(_drive(1, 1.0, session_id, input_epoch=0))
    gateway.tick(1.0)
    gateway.tick(1.500001)
    gateway.submit(_frame(sequence=2, received_s=1.6, estop_edge=True,
                          estop_id=5, input_epoch=0, session_id=session_id))
    assert gateway.tick(1.6).state == MOTION_HOLD


def test_new_connection_ends_recovery_and_needs_only_neutral():
    gateway = _field_gateway()
    session_id = _connect_drive(gateway, at_s=0.98)
    gateway.submit(_drive(1, 1.0, session_id, input_epoch=0))
    gateway.tick(1.0)
    gateway.tick(1.500001)
    new_session = str(uuid.uuid4())
    gateway.begin_connection()
    gateway.submit(_frame(sequence=0, received_s=2.0, session_id=new_session))
    assert gateway.tick(2.0).state == DRIVE
    gateway.submit(_drive(1, 2.02, new_session))
    assert gateway.tick(2.02).drive.linear == pytest.approx(0.7)


def test_latch_mode_handled_gap_does_not_relatch_after_hold_clear():
    gateway = RemoteInputGateway(GatewayConfig(input_timeout_s=0.5))
    session_id = _connect_drive(gateway, at_s=0.98)
    gateway.submit(_drive(1, 1.0, session_id))
    gateway.tick(1.0)
    assert gateway.tick(1.6).state == MOTION_HOLD
    assert gateway.clear_hold()
    gateway.tick(1.61)
    gateway.submit(_frame(sequence=2, received_s=1.65, session_id=session_id,
                          input_epoch=gateway.input_epoch))
    assert gateway.tick(1.65).state == DRIVE
