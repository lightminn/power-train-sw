from dataclasses import replace

from operator_console.arm_pad import ArmPadControl
from operator_console.arm_ops.session import ArmCommandSession
from operator_console.arm_ops.policy import ArmCommandContext
from operator_console.arm_ops.transport import RecordingArmTransport


def fixture():
    session = ArmCommandSession(clock=lambda: 100, transport=RecordingArmTransport(),
        context=ArmCommandContext(link_live=True, granted_mode='MANUAL', tool_id='dual_motor_gripper:3,4',
            tool_kind='dual_gripper', tool_generation=7, state_revision=1,
            capabilities=frozenset({'gripper_command', 'tool_enable', 'tool_jog'})))
    return ArmPadControl(session), session


def update(pad, *buttons, **changes):
    status = dict(pad_connected=True, arm_control=True, age_s=0, tool_mapping='known', tool_buttons=buttons)
    status.update(changes)
    pad.update(status, enabled=True, focused=True)


def test_neutral_press_release_uses_authenticated_session_and_relative_lease():
    pad, session = fixture()
    update(pad)
    update(pad, 'open')
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_tool_jog'
    assert session.submitted[-1].params['ttl_s'] == .6
    assert session.submitted[-1].params['target'] == 'both'
    update(pad)
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_tool_jog_stop'


def test_held_button_cannot_restart_after_swap_or_focus_loss():
    pad, session = fixture()
    update(pad, 'open')
    session.pump()
    assert not session.submitted
    update(pad)
    update(pad, 'open')
    session.pump()
    session.update_context(replace(session.context, tool_id='spur_1motor_gripper:5',
                                   tool_kind='single_gripper', tool_generation=8))
    update(pad, 'open')
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_tool_jog_stop'
    update(pad)
    update(pad, 'close')
    session.pump()
    assert session.submitted[-1].params['target'] == 'single'
    pad.update({}, enabled=True, focused=False)
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_tool_jog_stop'


def test_disable_edge_is_once_and_missing_mapping_never_moves():
    pad, session = fixture()
    update(pad, tool_mapping='')
    update(pad, 'open', tool_mapping='')
    assert not session.submitted
    update(pad)
    update(pad, 'disable')
    update(pad, 'disable')
    assert len(session.submitted) == 1
    assert session.submitted[0].params['enabled'] is False


def arm_fixture():
    # No tool attached: the arm joints must still jog from the pad.
    session = ArmCommandSession(clock=lambda: 100, transport=RecordingArmTransport(),
        context=ArmCommandContext(link_live=True, granted_mode='MANUAL', state_revision=1,
            capabilities=frozenset({'teleop_jog'})))
    return ArmPadControl(session), session


def test_dpad_jogs_the_selected_arm_axis_without_a_tool():
    pad, session = arm_fixture()
    update(pad)
    update(pad, 'jog_plus')
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_jog'
    assert session.submitted[-1].params['axis'] == 1
    assert session.submitted[-1].params['direction'] == 1
    update(pad)
    session.pump()
    assert session.submitted[-1].action == 'robot_arm_jog_stop'


def test_bumpers_cycle_axis_once_per_press_and_dpad_sides_change_speed():
    pad, session = arm_fixture()
    update(pad)
    update(pad, 'axis_prev')
    update(pad, 'axis_prev')
    update(pad)
    assert pad.axis == 5
    update(pad, 'axis_next')
    update(pad)
    update(pad, 'axis_next')
    update(pad)
    assert pad.axis == 2
    update(pad, 'speed_up')
    update(pad, 'speed_up')
    assert session.speed_level == 6
    update(pad)
    update(pad, 'jog_minus')
    session.pump()
    assert session.submitted[-1].params['axis'] == 2
    assert session.submitted[-1].params['direction'] == -1


def test_switching_axis_while_jogging_stops_the_previous_axis():
    pad, session = arm_fixture()
    update(pad)
    update(pad, 'jog_plus')
    session.pump()
    update(pad, 'jog_plus', 'axis_next')
    session.pump()
    stops = [r for r in session.submitted if r.action == 'robot_arm_jog_stop']
    assert stops and stops[-1].params['axis'] == 1


class AxisView:
    """Stands in for the keyboard-teleop panel's shared axis selection."""

    def __init__(self, selected_axis):
        self.selected_axis = selected_axis

    def select_axis(self, index):
        self.selected_axis = index


def test_pad_shares_the_keyboard_panel_selected_axis():
    pad, session = arm_fixture()
    view = AxisView(3)
    pad = ArmPadControl(session, axis_view=view)
    update(pad)
    update(pad, 'axis_next')
    update(pad)
    assert view.selected_axis == 4
    view.selected_axis = 2          # operator clicks axis 2 on the panel
    update(pad, 'jog_plus')
    session.pump()
    assert session.submitted[-1].params['axis'] == 2
