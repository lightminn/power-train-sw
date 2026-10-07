"""Entrypoint integrating the Qt event loop with a ROS executor thread."""

import sys
import threading

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.executors import SingleThreadedExecutor

from dynamixel_control.tool_profiles import get_profile, load_profiles
from robot_manual_gui.main_window import ManualMainWindow
from robot_manual_gui.qt_lifecycle import (
    install_context_watch, install_signal_quit, shutdown)
from robot_manual_gui.ros_interface import GuiSignals, ManualGuiNode


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    signals = GuiSignals()
    node = ManualGuiNode(signals)
    profile_path = (
        get_package_share_directory('dynamixel_control')
        + '/config/tool_profiles.yaml')
    profile = get_profile(load_profiles(profile_path), node.selected_tool)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    window = ManualMainWindow(node, signals, profile, node.mock_mode)
    # Use the available desktop while retaining window-manager controls.  The
    # dashboard itself is scrollable on displays smaller than its size hint.
    window.showMaximized()
    # Installed after rclpy.init so they replace rclpy's own handlers, which
    # take the context down without ever ending this Qt loop.
    signal_timer = install_signal_quit(app)
    context_timer = install_context_watch(app, rclpy.ok)
    try:
        result = app.exec_()
    finally:
        signal_timer.stop()
        context_timer.stop()
        for error in shutdown(executor, [node], spin_thread,
                              shutdown_ros=rclpy.shutdown, ok=rclpy.ok):
            print(f'shutdown step failed: {error}', file=sys.stderr)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
