"""Keep one Qt application alive until all GUI objects have been disposed."""

import os
import gc

import pytest

# ROS integration tests create real DDS nodes.  Keep them away from the live
# robot's production domain (0), even when pytest is launched from that shell.
os.environ['ROS_DOMAIN_ID'] = '174'


def shutdown_qt_ros_runtime(app, window, executor, nodes, rclpy_module):
    """Dispose a manual-GUI mock runtime without cross-framework callbacks.

    The executor is stopped first, so no ROS callback can emit a Qt signal
    while the window is being destroyed.  The window then unregisters its
    QApplication hooks and is deleted while QApplication still exists.  Nodes
    are removed before their DDS entities and context are torn down, avoiding
    an executor destructor revisiting already-destroyed nodes at interpreter
    exit.
    """
    from PyQt5.QtCore import QCoreApplication, QEvent

    if executor is not None:
        executor.shutdown()
    if window is not None:
        window.close()
        window.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        app.processEvents()
    if executor is not None:
        for node in nodes:
            try:
                executor.remove_node(node)
            except Exception:
                # A partially-constructed test runtime may not have added it.
                pass
    for node in nodes:
        node.destroy_node()
    if rclpy_module.ok():
        rclpy_module.shutdown()
    QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
    app.processEvents()
    gc.collect()


@pytest.fixture(scope='session', autouse=True)
def qt_application():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PyQt5.QtCore import QCoreApplication, QEvent
    from PyQt5.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    yield app
    # QApplication outlives every test QObject.  Flush deferred widget
    # destruction before its C++ destructor starts the platform-plugin
    # teardown; otherwise the offscreen backend can receive a late delete.
    for widget in app.topLevelWidgets():
        widget.close()
        widget.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
    app.processEvents()
    app.quit()
    gc.collect()


@pytest.fixture(autouse=True)
def dispose_test_windows(qt_application):
    from PyQt5.QtCore import QCoreApplication, QEvent

    yield
    # close() only hides a QMainWindow. Dispose its C++ children while the
    # application still exists, before another test allocates new widgets.
    for widget in qt_application.topLevelWidgets():
        widget.close()
        widget.deleteLater()
    QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
    gc.collect()
