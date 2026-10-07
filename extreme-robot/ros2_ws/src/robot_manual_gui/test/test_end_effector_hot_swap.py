"""Production ROS/Qt hot-swap flow with serial access forbidden."""

import json
import os
from pathlib import Path
import time

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


@pytest.fixture
def runtime(monkeypatch, tmp_path, request):
    import rclpy
    from dynamixel_sdk import PortHandler, GroupSyncWrite
    from PyQt5.QtWidgets import QApplication
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Int32MultiArray
    from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.ros_interface import GuiSignals, ManualGuiNode

    def forbidden(*_args, **_kwargs):
        raise AssertionError('mock must never access the serial bus')

    monkeypatch.setattr(PortHandler, 'openPort', forbidden)
    monkeypatch.setattr(GroupSyncWrite, 'txPacket', forbidden)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    profile = Path(__file__).parents[2] / 'dynamixel_control/config/tool_profiles.yaml'
    start_tool = getattr(request, 'param', 'dual_motor_gripper')
    # Honour ROS_DOMAIN_ID (174 in the isolated mock invocation).  A fixed
    # domain bypassed the test environment and could collide with a lingering
    # Fast DDS shared-memory participant during process teardown.
    rclpy.init(args=['--ros-args', '-p', 'mock_mode:=true', '-p',
                     'control_scope:=END_EFFECTOR_ONLY', '-p',
                     f'tool_type:={start_tool}', '-p',
                     f'tool_profile_file:={profile}'])
    app = QApplication.instance() or QApplication([])
    bridge = MoveItDynamixelBridge()
    signals = GuiSignals()
    gui = ManualGuiNode(signals)
    window = ManualMainWindow(gui, signals, bridge.tool_profile, mock_mode=True)
    # Offscreen tests drive widgets directly.  Do not create a platform window:
    # repeated hot-swap panel deletion otherwise leaves the offscreen plugin
    # with a native surface to tear down after QApplication has started its
    # final cleanup.
    executor = SingleThreadedExecutor()
    for node in (bridge, gui):
        executor.add_node(node)
    requests = []
    gui.create_subscription(Int32MultiArray, '/dynamixel/torque_request',
                            lambda msg: requests.append(msg), 10)

    def wait(predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            app.processEvents()
            if predicate():
                return
        raise AssertionError(f'timeout: {window.tool_status}')

    wait(lambda: bool(window.tool_status))
    gui.request_mode('MANUAL')
    wait(lambda: window.control_mode == bridge.control_mode == 'MANUAL')
    try:
        yield app, bridge, gui, window, requests, wait
    finally:
        from conftest import shutdown_qt_ros_runtime
        shutdown_qt_ros_runtime(app, window, executor, (gui, bridge), rclpy)


@pytest.mark.parametrize('automatic', [False, True])
def test_round_trip_replaces_context_and_preserves_held_torque_click(runtime, automatic):
    from PyQt5.QtCore import Qt
    from PyQt5.QtTest import QTest
    from dynamixel_control.tool_manager import BusToolIdentityProvider

    app, bridge, gui, window, requests, wait = runtime
    present = set(bridge.tool_ids)
    bridge._probe_tool_id = lambda i: i in present
    bridge._bus_tool_identity = BusToolIdentityProvider(
        bridge._tool_profiles, bridge._probe_tool_id)
    common = window._common_buttons()
    reports = []

    def torque_click(enabled):
        button = window.common_enable if enabled else window.common_disable
        wait(button.isEnabled)
        positions = {i: s['position'] for i, s in bridge._tool_samples.items()}
        before = len(requests)
        QTest.mousePress(button, Qt.LeftButton)
        # Regression: a periodic refresh between mouse-down and mouse-up
        # used to disable the spur alias and silently cancel the dual click.
        for _ in range(3):
            window._refresh_buttons()
            app.processEvents()
            assert button.isDown()
        QTest.mouseRelease(button, Qt.LeftButton)
        expected = [int(enabled), *bridge.tool_ids]
        wait(lambda: len(requests) == before + 1
             and list(requests[-1].data) == expected
             and window.tool_status['tool_torque_state'] == ('ON' if enabled else 'OFF'))
        assert {i: s['position'] for i, s in bridge._tool_samples.items()} == positions
        assert window.fsm_state == ('READY' if enabled else 'STOPPED')

    for tool, fsm in (
            ('dual_motor_gripper', 'DualMotorGripperFSM'),
            ('spur_1motor_gripper', 'SingleMotorGripperFSM'),
            ('cleaner', 'CleanerFSM'),
            ('dual_motor_gripper', 'DualMotorGripperFSM')):
        old_fsm = bridge.tool_fsm
        old_session = bridge.calibration_session or bridge.dual_calibration_session
        old_generation = bridge._tool_context_generation
        if old_session:
            old_session.active = True
            old_session.captures = {'open': {99: 123}}
        bridge._contact_grasp_event = {'old_context': True}
        ids = bridge._tool_profiles[tool]['actuator_ids']
        if automatic:
            present.clear()
            for _ in range(bridge.tool_detection_confirmations):
                bridge._poll_physical_tool()
            assert bridge._physical_tool_detached
            assert not bridge._tool_enable_allowed()
            present.update(ids)
            for _ in range(bridge.tool_detection_confirmations):
                bridge._poll_physical_tool()
        else:
            window.tool_combo.setCurrentIndex(window.tool_combo.findData(tool))
            window._request_tool_change()
        wait(lambda: bridge._tool_context_generation > old_generation
             and gui.runtime_tool_generation == bridge._tool_context_generation
             and gui.selected_tool == tool)
        assert bridge.tool_ids == gui.actuator_ids == ids
        assert type(bridge.tool_fsm).__name__ == fsm
        assert bridge.tool_fsm is not old_fsm
        assert old_fsm.state.name == 'STOPPED'
        if old_session:
            assert not old_session.active and not old_session.captures
        assert bool(bridge.calibration_session) == (tool == 'spur_1motor_gripper')
        assert bool(bridge.dual_calibration_session) == (tool == 'dual_motor_gripper')
        assert bridge._contact_grasp_event is None
        assert bridge.spur_manual_control.deadline is None
        assert bridge.torque_enabled_ids == set()
        assert set(bridge._tool_samples) == set(ids)
        assert bridge.active_ids == set(ids)
        assert window._common_buttons() == common
        assert window.tool_status['tool_enable_allowed']
        modes = bridge._required_tool_modes(bridge.tool_profile)
        assert all(bridge._tool_samples[i]['operating_mode'] == modes[i] for i in ids)
        assert gui.tool_profile == json.loads(json.dumps(bridge.tool_profile))
        torque_click(True)
        torque_click(False)
        torque_click(True)
        reports.append({'tool': tool, 'ids': ids, 'fsm': fsm,
                        'generation': bridge._tool_context_generation,
                        'torque': window.tool_status['tool_torque_state'],
                        'enable_allowed': window.tool_status['tool_enable_allowed']})
    # STOP recovery uses the same startup, without sending OPEN/CLOSE/JOG.
    window.tool_stop.click()
    wait(lambda: window.fsm_state == 'STOPPED')
    torque_click(True)
    assert window.open_button.isEnabled() and window.close_button.isEnabled()
    assert window.jog_open.isEnabled() and window.jog_close.isEnabled()
    print(json.dumps({'automatic': automatic, 'round_trip': reports}))


def test_detach_estop_and_stale_requests_cannot_enable_replacement(runtime):
    from std_msgs.msg import Bool, Int32MultiArray, MultiArrayDimension, String

    _app, bridge, _gui, window, _requests, wait = runtime
    old = bridge._tool_context_generation
    bridge._switch_tool_runtime('dual_motor_gripper')
    request = Int32MultiArray(data=[1, *bridge.tool_ids])
    request.layout.dim = [MultiArrayDimension(label=f'tool_context:{old}')]
    bridge.torque_request_callback(request)
    assert not bridge.torque_enabled_ids
    bridge.fsm_command_callback(String(data=json.dumps({
        'tool_type': bridge.tool_type, 'tool_context_generation': old, 'command': 'OPEN'})))
    assert bridge.tool_fsm.state.name == 'READY'
    bridge._on_emergency_stop(Bool(data=True))
    request.layout.dim = []
    bridge.torque_request_callback(request)
    assert not bridge._tool_enable_allowed() and not bridge.torque_enabled_ids
    with pytest.raises(RuntimeError, match='emergency stop'):
        bridge._switch_tool_runtime('spur_1motor_gripper')
    wait(lambda: window.tool_status['emergency_stop'])
    assert not window.common_enable.isEnabled()


@pytest.mark.parametrize('fault', ['offline', 'hardware', 'sync', 'calibration', 'detached'])
def test_enable_gate_fails_closed_in_mock_and_gui(runtime, fault):
    from std_msgs.msg import Int32MultiArray

    _app, bridge, _gui, window, _requests, wait = runtime
    if fault == 'offline':
        bridge._tool_samples[3]['online'] = False
    elif fault == 'hardware':
        bridge._tool_samples[3]['hardware_error'] = 32
    elif fault == 'sync':
        bridge._tool_samples[3]['position'] = bridge.tool_profile['motor_endpoints'][3]['open']
    elif fault == 'calibration':
        bridge.dual_calibration_session.profile['endpoint_calibration_verified'] = False
    else:
        bridge.tool_detached = True
    assert not bridge._tool_enable_allowed()
    bridge.torque_request_callback(Int32MultiArray(data=[1, *bridge.tool_ids]))
    assert not bridge.torque_enabled_ids
    wait(lambda: not window.tool_status['tool_enable_allowed'])
    assert not window.common_enable.isEnabled()
    assert not window.open_button.isEnabled()


def test_failed_fsm_startup_discards_old_sessions_and_permissions(runtime, monkeypatch):
    from std_msgs.msg import String

    _app, bridge, _gui, _window, _requests, _wait = runtime
    old_session = bridge.dual_calibration_session
    old_session.active = True
    old_session.captures = {'open': {3: 10, 4: 20}}
    monkeypatch.setattr(bridge, 'read_model', lambda _i: (_ for _ in ()).throw(
        RuntimeError('simulated missing replacement feedback')))
    bridge.tool_change_callback(String(data='spur_1motor_gripper'))
    assert bridge._tool_change_error
    assert bridge.tool_type == 'spur_1motor_gripper'
    assert not bridge._tool_enable_allowed()
    assert not bridge._fsm_allowlist and not bridge.active_ids
    assert bridge.calibration_session is bridge.dual_calibration_session is None
    assert not old_session.active and not old_session.captures
    assert not bridge.torque_enabled_ids


def test_auto_enable_setting_does_not_enable_torque_after_attach(runtime):
    _app, bridge, gui, window, _requests, wait = runtime
    bridge.auto_enable_on_attach = True
    for tool in ('dual_motor_gripper', 'spur_1motor_gripper',
                 'cleaner', 'dual_motor_gripper'):
        bridge._switch_tool_runtime(tool)
        assert bridge._current_tool_torque_state() == 'OFF'
        assert bridge.tool_fsm.state.name == 'READY'
        # Attachment preparation leaves torque OFF. Backend motion readiness
        # must remain false until the explicit GUI enable request succeeds.
        assert not bridge._tool_backend_ready()
        wait(lambda: gui.selected_tool == tool
             and window.tool_status.get('tool_context_generation')
             == bridge._tool_context_generation
             and window.tool_status['tool_torque_state'] == 'OFF')
        assert window.common_enable.isEnabled()
        assert window.fsm_state == 'READY'
        assert bridge._current_tool_torque_state() == 'OFF'


@pytest.mark.parametrize('auto_enable', [False, True])
def test_runtime_mode_configuration_and_enable_readback_use_existing_helpers(runtime, auto_enable):
    from std_msgs.msg import Int32MultiArray, MultiArrayDimension
    from dynamixel_control.moveit_dynamixel_bridge import (
        ADDR_GOAL_POSITION, ADDR_GOAL_VELOCITY, ADDR_HARDWARE_ERROR_STATUS,
        ADDR_OPERATING_MODE, ADDR_PRESENT_POSITION, ADDR_PROFILE_ACCELERATION,
        ADDR_PROFILE_VELOCITY, ADDR_TORQUE_ENABLE)

    _app, bridge, _gui, _window, _requests, _wait = runtime

    class MemoryPacket:
        def __init__(self):
            self.registers = {}
            self.writes = []
            self.reject_torque_id = None
            for profile in bridge._tool_profiles.values():
                for i in profile['actuator_ids']:
                    endpoint = (profile.get('motor_endpoints') or {}).get(i)
                    position = (round((endpoint['open'] + endpoint['close']) / 2)
                                if endpoint else profile.get('open_tick', 0))
                    self.registers[i] = {
                        ADDR_PRESENT_POSITION: position & 0xffffffff,
                        ADDR_GOAL_POSITION: 100,
                        ADDR_GOAL_VELOCITY: 0,
                        ADDR_OPERATING_MODE: 0,
                        ADDR_TORQUE_ENABLE: 0,
                        ADDR_HARDWARE_ERROR_STATUS: 0,
                        ADDR_PROFILE_ACCELERATION: 0,
                        ADDR_PROFILE_VELOCITY: 0}

        def ping(self, _port, i):
            return (1060, 0, 0) if i in self.registers else (0, 1, 0)

        def read(self, _port, i, address):
            return self.registers[i].get(address, 0), 0, 0

        def write(self, _port, i, address, value):
            self.writes.append((i, address, value))
            if not (i == self.reject_torque_id and address == ADDR_TORQUE_ENABLE and value):
                self.registers[i][address] = value
            return 0, 0

        read1ByteTxRx = read2ByteTxRx = read4ByteTxRx = read
        write1ByteTxRx = write2ByteTxRx = write4ByteTxRx = write

    packet = MemoryPacket()
    original = bridge.packet_handler
    bridge.packet_handler = packet
    bridge.mock_mode = False
    bridge.auto_enable_on_attach = auto_enable

    def torque_request(enabled):
        message = Int32MultiArray(data=[int(enabled), *bridge.tool_ids])
        message.layout.dim = [MultiArrayDimension(
            label=f'tool_context:{bridge._tool_context_generation}')]
        bridge.torque_request_callback(message)

    try:
        for tool in ('dual_motor_gripper', 'spur_1motor_gripper',
                     'cleaner', 'dual_motor_gripper'):
            bridge._switch_tool_runtime(tool)
            assert bridge._tool_enable_allowed()
            for i, required in bridge._required_tool_modes(bridge.tool_profile).items():
                assert packet.registers[i][ADDR_OPERATING_MODE] == required
                assert packet.registers[i][ADDR_TORQUE_ENABLE] == 0
            positions = {i: packet.registers[i][ADDR_PRESENT_POSITION] for i in bridge.tool_ids}
            torque_request(True)
            assert bridge._current_tool_torque_state() == 'ON'
            assert bridge.tool_fsm.state.name == 'READY'
            assert all(packet.registers[i][ADDR_PRESENT_POSITION] == positions[i]
                       for i in bridge.tool_ids)
            torque_request(False)
            assert bridge._current_tool_torque_state() == 'OFF'
            assert bridge.tool_fsm.state.name == 'STOPPED'
        # An ACK without a matching readback must roll back the entire pair.
        packet.reject_torque_id = bridge.tool_ids[-1]
        torque_request(True)
        assert bridge._current_tool_torque_state() == 'OFF'
        assert not bridge.torque_enabled_ids
        assert all(value == 0 for _i, address, value in packet.writes
                   if address == ADDR_GOAL_VELOCITY)
    finally:
        bridge.mock_mode = True
        bridge.packet_handler = original


@pytest.mark.parametrize('runtime', ['cleaner'], indirect=True)
def test_physical_cleaner_dual_cleaner_round_trip_retargets_torque(runtime):
    """An absent ID2 must leave SyncRead and never receive a dual command."""
    from PyQt5.QtCore import Qt
    from PyQt5.QtTest import QTest
    from dynamixel_control.tool_manager import BusToolIdentityProvider
    from dynamixel_control.moveit_dynamixel_bridge import (
        ADDR_GOAL_POSITION, ADDR_GOAL_VELOCITY, ADDR_HARDWARE_ERROR_STATUS,
        ADDR_OPERATING_MODE, ADDR_PRESENT_POSITION, ADDR_PROFILE_ACCELERATION,
        ADDR_PROFILE_VELOCITY, ADDR_TORQUE_ENABLE)

    _app, bridge, gui, window, requests, wait = runtime
    bridge.feedback_timer.cancel()  # no real SDK bus transaction in this test

    class MemoryPacket:
        def __init__(self):
            self.present = {2}
            self.writes = []
            self.registers = {}
            for profile in bridge._tool_profiles.values():
                for dxl_id in profile['actuator_ids']:
                    endpoint = (profile.get('motor_endpoints') or {}).get(dxl_id)
                    position = (round((endpoint['open'] + endpoint['close']) / 2)
                                if endpoint else profile.get('open_tick', 0))
                    self.registers[dxl_id] = {
                        ADDR_PRESENT_POSITION: position & 0xffffffff,
                        ADDR_GOAL_POSITION: position & 0xffffffff,
                        ADDR_GOAL_VELOCITY: 0,
                        ADDR_OPERATING_MODE: 0,
                        ADDR_TORQUE_ENABLE: 0,
                        ADDR_HARDWARE_ERROR_STATUS: 0,
                        ADDR_PROFILE_ACCELERATION: 0,
                        ADDR_PROFILE_VELOCITY: 0}

        def ping(self, _port, dxl_id):
            return (1060, 0, 0) if dxl_id in self.present else (0, 1, 0)

        def read(self, _port, dxl_id, address):
            assert dxl_id in self.present, f'read from detached ID{dxl_id}'
            return self.registers[dxl_id].get(address, 0), 0, 0

        def write(self, _port, dxl_id, address, value):
            assert dxl_id in self.present, f'write to detached ID{dxl_id}'
            self.writes.append((dxl_id, address, value))
            self.registers[dxl_id][address] = value
            return 0, 0

        read1ByteTxRx = read2ByteTxRx = read4ByteTxRx = read
        write1ByteTxRx = write2ByteTxRx = write4ByteTxRx = write

    packet = MemoryPacket()
    bridge.packet_handler = packet
    bridge.mock_mode = False
    bridge.port_connected = True
    bridge.auto_tool_detection = True
    bridge._probe_tool_id = lambda dxl_id: dxl_id in packet.present
    bridge._bus_tool_identity = BusToolIdentityProvider(
        bridge._tool_profiles, bridge._probe_tool_id)

    def ready(tool, ids, fsm):
        wait(lambda: gui.selected_tool == tool
             and gui.runtime_tool_generation == bridge._tool_context_generation
             and window.tool_status.get('tool_preparation_generation')
             == bridge._tool_context_generation)
        assert bridge.tool_type == tool
        assert bridge.tool_ids == gui.actuator_ids == ids
        assert set(bridge.group_sync_read.data_dict) == set(ids)
        assert set(bridge._tool_samples) == set(ids)
        assert type(bridge.tool_fsm).__name__ == fsm
        assert bridge.tool_fsm.state.name == 'READY'
        assert bridge._tool_preparation_state == 'READY'
        assert bridge._current_tool_torque_state() == 'OFF'
        assert window.tool_status['tool_enable_allowed']
        assert window.common_enable.isEnabled()

    def click_torque(enabled, ids):
        button = window.common_enable if enabled else window.common_disable
        wait(button.isEnabled)
        before = len(requests)
        QTest.mouseClick(button, Qt.LeftButton)
        wait(lambda: len(requests) == before + 1
             and list(requests[-1].data) == [int(enabled), *ids]
             and bridge._current_tool_torque_state()
             == ('ON' if enabled else 'OFF'))
        assert [int(item) for item in requests[-1].data] == [int(enabled), *ids]

    def attach(ids):
        packet.present.clear()
        for _ in range(bridge.tool_detection_confirmations):
            bridge._poll_physical_tool()
        assert bridge._physical_tool_detached
        packet.present.update(ids)
        for _ in range(bridge.tool_detection_confirmations):
            bridge._poll_physical_tool()

    try:
        # Bootstrap cleaner, then use the same physical detector as production.
        bridge._switch_tool_runtime('cleaner')
        ready('cleaner', [2], 'CleanerFSM')
        click_torque(True, [2])
        assert packet.registers[2][ADDR_TORQUE_ENABLE] == 1
        click_torque(False, [2])
        assert packet.registers[2][ADDR_TORQUE_ENABLE] == 0
        cleaner_generation = bridge._tool_context_generation

        attach((3, 4))
        ready('dual_motor_gripper', [3, 4], 'DualMotorGripperFSM')
        assert bridge._tool_context_generation > cleaner_generation
        assert bridge.cleaning_actuator_id == -1
        assert not bridge.cleaning_configured and not bridge.cleaning_running
        assert packet.registers[3][ADDR_TORQUE_ENABLE] == 0
        assert packet.registers[4][ADDR_TORQUE_ENABLE] == 0
        # A stale SDK feedback registration is an unsafe context even when
        # profile, direct ping and torque readback all look healthy.
        bridge.group_sync_read.addParam(2)
        assert 'feedback actuator IDs' in bridge._tool_enable_block_reason()
        assert not bridge._tool_enable_allowed()
        bridge.group_sync_read.removeParam(2)
        assert bridge._tool_enable_allowed()
        write_boundary = len(packet.writes)
        click_torque(True, [3, 4])
        assert packet.registers[3][ADDR_TORQUE_ENABLE] == 1
        assert packet.registers[4][ADDR_TORQUE_ENABLE] == 1
        assert all(dxl_id != 2 for dxl_id, _address, _value
                   in packet.writes[write_boundary:])
        click_torque(False, [3, 4])
        assert packet.registers[3][ADDR_TORQUE_ENABLE] == 0
        assert packet.registers[4][ADDR_TORQUE_ENABLE] == 0

        attach((2,))
        ready('cleaner', [2], 'CleanerFSM')
        write_boundary = len(packet.writes)
        click_torque(True, [2])
        assert packet.registers[2][ADDR_TORQUE_ENABLE] == 1
        assert all(dxl_id == 2 for dxl_id, _address, _value
                   in packet.writes[write_boundary:])
        click_torque(False, [2])
        assert packet.registers[2][ADDR_TORQUE_ENABLE] == 0
    finally:
        bridge.mock_mode = True
