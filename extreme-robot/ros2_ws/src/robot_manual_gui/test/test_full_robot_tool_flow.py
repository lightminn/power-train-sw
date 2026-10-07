"""One persistent arm/GUI/bridge across every interchangeable tool context."""
import os
import json
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def test_full_robot_shared_buttons_preserve_arm_and_fsm(monkeypatch, tmp_path):
    import rclpy
    import yaml
    from rclpy.executors import SingleThreadedExecutor
    from PyQt5.QtWidgets import QApplication, QPushButton
    from dynamixel_sdk import PortHandler, GroupSyncWrite
    from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge, JOINT_CONFIG
    from dynamixel_control.arm_fsm_node import ArmFsmNode
    from robot_manual_gui.ros_interface import ManualGuiNode, GuiSignals
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.korean_text import ko
    from sensor_msgs.msg import JointState

    def forbidden(*args, **kwargs):
        raise AssertionError('mock integration attempted a physical packet')
    monkeypatch.setattr(PortHandler, 'openPort', forbidden)
    monkeypatch.setattr(GroupSyncWrite, 'txPacket', forbidden)
    profile = Path(__file__).parents[2] / 'dynamixel_control/config/tool_profiles.yaml'
    profiles = yaml.safe_load(profile.read_text())
    profiles['tool_profiles']['cleaner'].update(
        calibrated=True, actuator_ids=[6], joint_names=['cleaning_actuator_joint'],
        direction=-1, profile_velocity=30)
    fixture = tmp_path / 'mock_profiles.yaml'
    fixture.write_text(yaml.safe_dump(profiles))
    rclpy.init(args=['--ros-args', '-p', 'mock_mode:=true', '-p', 'read_only:=false',
        '-p', 'tool_type:=dual_motor_gripper', '-p', 'control_scope:=FULL_ROBOT',
        '-p', 'dry_run_mode:=true', '-p', 'sensor_mock_mode:=true',
        '-p', f'tool_profile_file:={fixture}'])
    app = QApplication.instance() or QApplication([])
    bridge = MoveItDynamixelBridge()
    arm = ArmFsmNode()
    signals = GuiSignals()
    gui = ManualGuiNode(signals)
    window = ManualMainWindow(gui, signals, bridge.tool_profile, mock_mode=True)
    window.show()
    executor = SingleThreadedExecutor()
    for node in (bridge, arm, gui):
        executor.add_node(node)
    samples = []
    gui.create_subscription(JointState, '/joint_states',
                            lambda msg: samples.append(msg), 10)
    arm_names = tuple(JOINT_CONFIG)
    expected = {name: (index + 1) * .01 for index, name in enumerate(arm_names)}
    bridge._mock_arm_positions.update(expected)
    clients = (arm._move, arm._gripper, arm._fk_client)
    arm_state = arm.state
    action_server = bridge.action_server if hasattr(bridge, 'action_server') else None
    common = (window.close_button, window.open_button)

    def wait_for(predicate):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            app.processEvents()
            if predicate():
                return
        raise AssertionError(f'timeout: {bridge.tool_type}, {window.fsm_state}, '
                             f'{window.control_mode}, {window.tool_status}')

    try:
        wait_for(lambda: window.arm_fsm_state == arm.state.name and len(samples) >= 3)
        window.mode_combo.setCurrentIndex(window.mode_combo.findData('MANUAL'))
        window._request_mode()
        wait_for(lambda: arm.control_mode == bridge.control_mode == window.control_mode == 'MANUAL')
        arm_state = arm.state
        for tool, ids, fsm in (
            ('dual_motor_gripper', [3, 4], 'DualMotorGripperFSM'),
            ('spur_1motor_gripper', [5], 'SingleMotorGripperFSM'),
            ('cleaner', [6], 'CleanerFSM'),
            ('dual_motor_gripper', [3, 4], 'DualMotorGripperFSM')):
            before = len(samples)
            window.tool_combo.setCurrentIndex(window.tool_combo.findData(tool))
            next(b for b in window.findChildren(QPushButton)
                 if b.text() == ko('REQUEST TOOL CHANGE')).click()
            wait_for(lambda: gui.selected_tool == arm.selected_tool_type == tool
                     and arm._tool_status is not None
                     and window.fsm_state == 'READY' and len(samples) > before + 2)
            assert (window.close_button, window.open_button) == common
            assert bridge.tool_ids == gui.actuator_ids == ids
            assert type(bridge.tool_fsm).__name__ == fsm
            assert bridge.active_ids == {11, 14, 13, 12, 16, *ids}
            assert arm.state == arm_state
            assert (arm._move, arm._gripper, arm._fk_client) == clients
            assert arm.tool_profile == json.loads(json.dumps(bridge.tool_profile))
            assert all(arm._joint_position[n] == expected[n] for n in arm_names)
            assert all(gui.positions[n] == expected[n] for n in arm_names)
            wait_for(lambda: window.common_enable.isEnabled())
            window.common_enable.click()
            wait_for(lambda: all(bridge._tool_samples[i]['torque_state'] == 'ON' for i in ids)
                     and window.close_button.isEnabled())
            if tool != 'cleaner':
                common[0].click()
                wait_for(lambda: window.fsm_state == 'CLOSED')
                endpoints = bridge.tool_profile.get('motor_endpoints')
                for i in ids:
                    assert bridge.read_position(i) == (endpoints[i]['close'] if endpoints
                                                       else bridge.tool_profile['close_tick'])
                common[1].click()
                wait_for(lambda: window.fsm_state == 'OPEN')
                for i in ids:
                    assert bridge.read_position(i) == (endpoints[i]['open'] if endpoints
                                                       else bridge.tool_profile['open_tick'])
            else:
                wait_for(lambda: common[0].isEnabled())
                common[0].click()
                wait_for(lambda: bridge._tool_samples[6].get('velocity') == -30)
                common[1].click()
                wait_for(lambda: bridge._tool_samples[6].get('velocity') == 30)
                assert bridge.cleaning_running
                window.tool_stop.click()
                wait_for(lambda: not bridge.cleaning_running)
            window.grab().save(str(tmp_path / f'{tool}.png'))
            assert all(bridge._mock_arm_positions[n] == expected[n] for n in arm_names)
            for msg in samples[before:]:
                actual = dict(zip(msg.name, msg.position))
                assert all(actual[n] == expected[n] for n in arm_names)
            visible = {b for b in window.tool_control_box.findChildren(QPushButton)
                       if b.isVisible()}
            assert common[0] in visible and common[1] in visible
            assert visible <= {*common, window.tool_stop, window.common_enable,
                               window.common_disable, window.read_diag}
        assert action_server is (bridge.action_server if hasattr(bridge, 'action_server') else None)
    finally:
        from conftest import shutdown_qt_ros_runtime
        shutdown_qt_ros_runtime(
            app, window, executor, (arm._fk_node, gui, arm, bridge), rclpy)
