import os
import pty
import signal
import subprocess
import sys
import time

from l515_dashboard.client import GatewayClient


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.01)
    return predicate()


def dashboard(socket_path):
    master, slave = pty.openpty()
    env = {**os.environ, "TERM": "xterm-256color"}
    process = subprocess.Popen(
        [sys.executable, "-m", "l515_dashboard", "--socket", str(socket_path)],
        stdin=slave, stdout=slave, stderr=slave, env=env, close_fds=True,
    )
    os.close(slave)
    return process, master


def test_real_dashboard_exit_signals_and_crash_do_not_own_gateway(tmp_path):
    socket_path = tmp_path / "gateway.sock"
    pid_path = tmp_path / "child.pid"
    gateway = subprocess.Popen([
        sys.executable, "-m", "l515_dashboard.tests.fake_gateway_process",
        str(socket_path), str(pid_path),
    ])
    try:
        assert wait_for(lambda: socket_path.exists() and pid_path.exists())
        child = int(pid_path.read_text())
        for action in ("q", signal.SIGHUP, signal.SIGTERM, signal.SIGKILL):
            process, master = dashboard(socket_path)
            time.sleep(0.3)
            if action == "q":
                os.write(master, b"q")
            else:
                os.kill(process.pid, action)
            process.wait(timeout=3)
            os.close(master)
            assert gateway.poll() is None and alive(child)

        # Pilot coverage exercises the literal Shift+Q/y UI path. Here the real
        # socket request proves its acknowledged process-level consequence.
        snapshot = GatewayClient(socket_path).request("stop_gateway")
        assert snapshot.acknowledged
        gateway.wait(timeout=3)
        assert wait_for(lambda: not alive(child))
        assert not socket_path.exists()
    finally:
        if gateway.poll() is None:
            gateway.terminate()
            gateway.wait(timeout=3)
