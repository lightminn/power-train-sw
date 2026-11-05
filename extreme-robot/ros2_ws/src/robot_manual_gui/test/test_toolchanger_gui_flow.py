"""실제 mock 브리지에 GUI 잠금 버튼을 눌러 교체 시퀀스를 확인한다."""
import os
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def test_gui_lock_buttons_drive_the_real_change_sequence(monkeypatch, tmp_path):
    import rclpy
    import yaml
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from PyQt5.QtWidgets import QApplication, QPushButton
    from dynamixel_sdk import PortHandler
    from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge
    from robot_manual_gui.ros_interface import ManualGuiNode, GuiSignals
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.korean_text import ko

    def forbidden(*args, **kwargs):
        raise AssertionError('mock test attempted physical serial access')
    monkeypatch.setattr(PortHandler, 'openPort', forbidden)
    source_profile = Path(__file__).parents[2] / 'dynamixel_control/config/tool_profiles.yaml'
    profiles = yaml.safe_load(source_profile.read_text())
    # A mock-only calibrated fixture must not certify the physical tool.
    profiles['tool_profiles']['spur_1motor_gripper']['calibrated'] = True
    profile_file = tmp_path / 'mock_tool_profiles.yaml'
    profile_file.write_text(yaml.safe_dump(profiles))
    rclpy.init(args=['--ros-args', '-p', 'toolchanger_enabled:=true',
                     '-p', 'mock_mode:=true', '-p', 'read_only:=false',
                     '-p', 'tool_type:=spur_1motor_gripper',
                     '-p', 'control_scope:=END_EFFECTOR_ONLY',
                     '-p', 'toolchanger_coupling_load_threshold:=120',
                     '-p', 'toolchanger_mock_load:=0',
                     '-p', f'tool_profile_file:={profile_file}'])
    app = QApplication.instance() or QApplication([])
    bridge = MoveItDynamixelBridge()
    signals = GuiSignals()
    gui = ManualGuiNode(signals)
    window = ManualMainWindow(gui, signals, bridge.tool_profile, mock_mode=True)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(gui)

    def wait_for(predicate):
        until = time.monotonic() + 5
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.02)
            app.processEvents()
            if predicate():
                return
        raise AssertionError(f'timed out: lock={window.toolchanger_status}, '
                             f'mode={window.control_mode}, tool={window.tool_status}')

    try:
        wait_for(lambda: window.toolchanger_status.get('state') == 'UNCALIBRATED')
        assert not window.lock_enable.isEnabled()
        window.mode_combo.setCurrentIndex(window.mode_combo.findData('MANUAL'))
        next(b for b in window.findChildren(QPushButton)
             if b.text() == ko('REQUEST MODE')).click()
        wait_for(lambda: window.control_mode == 'MANUAL')

        wait_for(lambda: window.lock_zero_locked.isEnabled())
        window.lock_zero_locked.click()
        wait_for(lambda: bridge.toolchanger.zero_tick == 2048
                 and bridge.toolchanger.zero_state == 'LOCKED')
        wait_for(lambda: window.lock_enable.isEnabled())
        window.lock_enable.click()
        wait_for(lambda: bridge.toolchanger.ready and window.lock_detach.isEnabled())

        # 도구를 실제로 켠 뒤 분리해야 잠금 전처리의 토크OFF 확인이 의미를 갖는다.
        window._request_tool_change()
        wait_for(lambda: window.tool_status.get('tool_preparation_state') == 'READY')
        wait_for(lambda: window.common_enable.isEnabled())
        window.common_enable.click()
        wait_for(lambda: bridge.read_torque(5) == 1)

        window.lock_direction.setCurrentIndex(
            window.lock_direction.findData(-1))
        window.lock_detach.click()
        wait_for(lambda: window.toolchanger_status.get('sequence_state') == 'CONFIRMED')
        assert bridge.toolchanger.state == 'UNLOCKED'
        # 모터1은 잠금 소유 경로로만 읽는다. read_position(1)은 allowlist가 막는다.
        assert window.toolchanger_status['position'] == 1792
        assert bridge.read_torque(5) == 0
        assert window.toolchanger_status['coupling_proxy'] == 'RELEASED'
        assert window.toolchanger_status['mechanically_confirmed'] is False

        # 음성 대조: 결합 부하 프록시가 임계값 미달이면 결합을 확인하지 않는다.
        window.lock_direction.setCurrentIndex(window.lock_direction.findData(1))
        window.lock_attach.click()
        wait_for(lambda: window.toolchanger_status.get('sequence_state') == 'ABORTED')
        assert window.toolchanger_status['coupling_proxy'] == 'NOT_ENGAGED'
        wait_for(lambda: window.toolchanger_status.get('torque') == 0)
        assert not window.lock_attach.isEnabled()

        bridge.set_parameters([Parameter('toolchanger_mock_load',
                                         Parameter.Type.INTEGER, 150)])
        wait_for(lambda: window.lock_enable.isEnabled())
        window.lock_enable.click()
        # 중단된 이동은 이미 결합 인덱스에 멈춰 있다. 분리로 되돌린 뒤 다시 결합한다.
        wait_for(lambda: window.lock_detach.isEnabled())
        window.lock_direction.setCurrentIndex(window.lock_direction.findData(-1))
        window.lock_detach.click()
        wait_for(lambda: window.toolchanger_status.get('state') == 'UNLOCKED'
                 and window.toolchanger_status.get('sequence_state') == 'CONFIRMED')
        window.lock_direction.setCurrentIndex(window.lock_direction.findData(1))
        window.lock_attach.click()
        wait_for(lambda: window.toolchanger_status.get('sequence_state') == 'CONFIRMED'
                 and window.toolchanger_status.get('coupling_proxy') == 'ENGAGED')
        assert bridge.toolchanger.state == 'LOCKED'
        assert window.toolchanger_status['coupling_peak_load'] == 150
        assert window.toolchanger_status['mechanically_confirmed'] is False
    finally:
        from conftest import shutdown_qt_ros_runtime
        shutdown_qt_ros_runtime(app, window, executor, (gui, bridge), rclpy)
