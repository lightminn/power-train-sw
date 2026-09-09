"""Installed teleop entrypoint must stop cleanly after ROS handles SIGINT."""
import os
import signal
import socket
import subprocess
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


def test_installed_teleop_sigint_has_no_double_shutdown(tmp_path):
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=77)
    observer = rclpy.create_node("teleop_shutdown_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    states = []
    observer.create_subscription(String, "/teleop/gateway_state", states.append, 10)
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    log_path = tmp_path / "teleop.log"
    with log_path.open("w") as log:
        child = subprocess.Popen(
            ["ros2", "run", "powertrain_ros", "teleop_command", "--ros-args",
             "-p", "host:=127.0.0.1", "-p", f"port:={port}"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            env=dict(os.environ, ROS_DOMAIN_ID="77", ROS_LOCALHOST_ONLY="1"))
        try:
            deadline = time.monotonic() + 10
            while True:
                assert child.poll() is None, log_path.read_text()
                try:
                    with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                        break
                except OSError:
                    assert time.monotonic() < deadline, log_path.read_text()
                    time.sleep(0.05)
            # A listener opens during __init__; wait for an executor tick so
            # SIGINT exercises the normal running-node cleanup path.
            while not states:
                assert child.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                executor.spin_once(timeout_sec=0.1)
            os.killpg(child.pid, signal.SIGINT)
            child.wait(timeout=5)
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
            executor.shutdown()
            observer.destroy_node()
            rclpy.shutdown(context=context)
    output = log_path.read_text()
    assert "Traceback" not in output, output
    assert child.returncode == 0, output
