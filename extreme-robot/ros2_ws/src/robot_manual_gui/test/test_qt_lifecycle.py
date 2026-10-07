"""SIGINT/SIGTERM must end the Qt loop instead of leaving a ghost GUI node."""

import os
import signal
import subprocess
import sys
from pathlib import Path

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

ROOT = Path(__file__).parents[1] / 'robot_manual_gui'


def _app():
    from PyQt5.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def _restore(handlers):
    for signum, handler in handlers.items():
        signal.signal(signum, handler)


def test_signal_handler_quits_the_event_loop():
    from PyQt5.QtCore import QTimer
    from robot_manual_gui.qt_lifecycle import install_signal_quit

    app = _app()
    saved = {num: signal.getsignal(num)
             for num in (signal.SIGINT, signal.SIGTERM)}
    seen = []
    try:
        timer = install_signal_quit(app, on_signal=seen.append)
        QTimer.singleShot(0, lambda: os.kill(os.getpid(), signal.SIGTERM))
        # Bounds the test if the handler never runs; the assertion below then
        # fails on `seen` rather than hanging the suite.
        QTimer.singleShot(5000, app.quit)
        app.exec_()
        timer.stop()
    finally:
        _restore(saved)
    assert seen == [signal.SIGTERM]


def test_context_watch_quits_when_the_ros_context_is_down():
    from robot_manual_gui.qt_lifecycle import install_context_watch

    app = _app()
    states = [True, False]
    down = []
    timer = install_context_watch(
        app, lambda: states.pop(0) if states else False,
        on_down=lambda: down.append(True), interval_ms=1)
    app.exec_()
    timer.stop()
    assert down == [True]


def test_shutdown_runs_every_step_in_order_and_reports_failures():
    from robot_manual_gui.qt_lifecycle import shutdown

    class Executor:
        def shutdown(self):
            order.append('executor')
            raise RuntimeError('executor refused')

    class Node:
        def __init__(self, name):
            self.name = name

        def destroy_node(self):
            order.append(self.name)

    class Thread:
        def __init__(self):
            self.joined = None

        def is_alive(self):
            return self.joined is None

        def join(self, timeout=None):
            self.joined = timeout
            order.append('join')

    order = []
    thread = Thread()
    errors = shutdown(Executor(), [Node('node_a'), Node('node_b')], thread,
                      shutdown_ros=lambda: order.append('rclpy'),
                      ok=lambda: True, join_timeout=0.5)
    assert order == ['executor', 'node_a', 'node_b', 'rclpy', 'join']
    assert [str(error) for error in errors] == ['executor refused']
    assert thread.joined == 0.5


def test_shutdown_skips_context_teardown_when_already_down():
    from robot_manual_gui.qt_lifecycle import shutdown

    called = []
    assert shutdown(None, [], None,
                    shutdown_ros=lambda: called.append('rclpy'),
                    ok=lambda: False) == []
    assert called == []


def test_restart_parent_launch_replays_exact_command_after_a_delay():
    from robot_manual_gui.qt_lifecycle import restart_parent_launch

    started, terminated = [], []
    command = restart_parent_launch(
        parent_pid=123,
        read_cmdline=lambda _pid: (
            b'/usr/bin/python3\0/opt/ros/humble/bin/ros2\0launch\0'
            b'robot_manual_gui\0manual_gui.launch.py\0read_only:=false\0'),
        start_detached=lambda program, args: started.append((program, args)) or True,
        terminate=lambda pid, signum: terminated.append((pid, signum)))
    assert command[-1] == 'read_only:=false'
    assert started == [('/bin/bash', [
        '-lc', 'sleep 2; exec "$@"', 'gui-restart', *command])]
    assert terminated == [(123, signal.SIGTERM)]


def test_restart_parent_launch_rejects_an_unknown_parent():
    from robot_manual_gui.qt_lifecycle import restart_parent_launch

    with pytest.raises(RuntimeError, match='ros2 launch'):
        restart_parent_launch(
            parent_pid=123, read_cmdline=lambda _pid: b'/bin/bash\0')


def test_process_exits_on_sigterm_without_leaving_a_ghost():
    """End-to-end: the same helper the GUI entrypoint installs."""
    script = (
        'import os, sys, signal\n'
        'os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")\n'
        'from PyQt5.QtWidgets import QApplication, QMainWindow\n'
        'from robot_manual_gui.qt_lifecycle import install_signal_quit\n'
        'app = QApplication([])\n'
        'window = QMainWindow()\n'
        'window.show()\n'
        'timer = install_signal_quit(app)\n'
        'print("ready", flush=True)\n'
        'code = app.exec_()\n'
        'print("exited", code, flush=True)\n')
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(
        [str(ROOT.parent), *sys.path]), QT_QPA_PLATFORM='offscreen')
    process = subprocess.Popen(
        [sys.executable, '-c', script], stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, text=True, env=env)
    try:
        assert process.stdout.readline().strip() == 'ready'
        process.send_signal(signal.SIGTERM)
        # A ghost GUI is exactly the process that never returns from here.
        out, err = process.communicate(timeout=15)
    except Exception:
        process.kill()
        raise
    assert process.returncode == 0, err
    assert 'exited 0' in out


def test_entrypoint_wires_signal_and_context_shutdown():
    source = (ROOT / 'main.py').read_text(encoding='utf-8')
    assert 'install_signal_quit(app)' in source
    assert 'install_context_watch(app, rclpy.ok)' in source
    assert 'shutdown(executor, [node], spin_thread' in source
