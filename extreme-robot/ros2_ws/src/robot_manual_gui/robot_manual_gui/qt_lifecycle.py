"""Signal/shutdown bridge between the Qt event loop and the ROS executor.

`rclpy`'s own SIGINT handler only brings the context down; it never returns
control to a Qt `exec_()` loop, so `manual_gui` used to survive SIGINT/SIGTERM
and linger as a ghost node publishing on the shared DDS domain.  Python also
runs signal handlers only between bytecodes of the main thread, which never
happens while Qt blocks inside C++, so a periodic no-op timer is required for
the handler to run at all.

Nothing here touches hardware, motion, or the tool contract.
"""

import signal
import os


def restart_parent_launch(parent_pid=None, read_cmdline=None,
                          start_detached=None, terminate=None):
    """Replace the parent ``ros2 launch`` after an E-stop.

    The GUI is a child of launch, so restarting only this process would retain
    the bridge's E-stop latch.  A delayed detached copy of the exact parent
    command lets launch terminate every old child and release the serial bus
    before the replacement stack opens it.  This function never publishes an
    E-stop reset or a motor command.
    """
    parent_pid = os.getppid() if parent_pid is None else int(parent_pid)
    if read_cmdline is None:
        def read_cmdline(pid):
            with open(f'/proc/{pid}/cmdline', 'rb') as stream:
                return stream.read()
    raw = read_cmdline(parent_pid)
    if isinstance(raw, str):
        raw = raw.encode()
    command = [part.decode(errors='surrogateescape')
               for part in raw.split(b'\0') if part]
    if (not any(part == 'ros2' or part.endswith('/ros2') for part in command)
            or 'launch' not in command
            or 'robot_manual_gui' not in command
            or 'manual_gui.launch.py' not in command):
        raise RuntimeError('현재 GUI를 시작한 ros2 launch 명령을 찾지 못했습니다')
    if start_detached is None:
        from PyQt5.QtCore import QProcess
        start_detached = QProcess.startDetached
    # ``$@`` retains every original argument exactly; do not rebuild the launch
    # command from UI state, which could silently drop a safety argument.
    started = start_detached(
        '/bin/bash', ['-lc', 'sleep 2; exec "$@"', 'gui-restart', *command])
    if isinstance(started, tuple):
        started = started[0]
    if not started:
        raise RuntimeError('새 프로그램 시작에 실패했습니다')
    (os.kill if terminate is None else terminate)(parent_pid, signal.SIGTERM)
    return command


# Fast enough that Ctrl-C feels immediate, slow enough to stay free.
SIGNAL_POLL_MS = 100
# The ROS context can also be taken down from outside this process; noticing it
# lets the window close instead of showing values that will never update again.
CONTEXT_POLL_MS = 250


def install_signal_quit(app, signals=(signal.SIGINT, signal.SIGTERM),
                        on_signal=None):
    """Route process signals to `app.quit()` and return the keep-alive timer.

    The caller must keep the returned timer referenced for as long as the
    application runs, otherwise Python may collect it and the handlers stop
    being reachable.
    """
    from PyQt5.QtCore import QTimer

    def handler(signum, _frame):
        if on_signal is not None:
            on_signal(signum)
        app.quit()

    for signum in signals:
        signal.signal(signum, handler)
    timer = QTimer(app)
    timer.setInterval(SIGNAL_POLL_MS)
    # Returning to the interpreter is the entire job; the slot stays empty.
    timer.timeout.connect(lambda: None)
    timer.start()
    return timer


def install_context_watch(app, ok, on_down=None, interval_ms=CONTEXT_POLL_MS):
    """Quit Qt when `ok()` (normally `rclpy.ok`) reports the context is down."""
    from PyQt5.QtCore import QTimer

    def check():
        if not ok():
            if on_down is not None:
                on_down()
            app.quit()

    timer = QTimer(app)
    timer.setInterval(interval_ms)
    timer.timeout.connect(check)
    timer.start()
    return timer


def shutdown(executor=None, nodes=(), spin_thread=None, shutdown_ros=None,
             ok=None, join_timeout=2.0):
    """Tear down in the order that actually releases the DDS participants.

    Executor first (so no callback runs against a destroyed node), then the
    nodes, then the context, and only then join the spin thread.  Every step is
    independent: one failure must not strand the ones after it.
    """
    errors = []
    steps = []
    if executor is not None:
        steps.append(executor.shutdown)
    for node in nodes:
        steps.append(node.destroy_node)
    if shutdown_ros is not None and (ok is None or ok()):
        steps.append(shutdown_ros)
    for step in steps:
        try:
            step()
        except Exception as exc:  # pragma: no cover - teardown must continue
            errors.append(exc)
    if spin_thread is not None and spin_thread.is_alive():
        spin_thread.join(timeout=join_timeout)
        if spin_thread.is_alive():
            errors.append(RuntimeError('ROS spin thread did not exit'))
    return errors
