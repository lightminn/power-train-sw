import os
import signal
import subprocess
import sys
import time

from l515_dashboard.client import GatewayClient


def alive(pid):
    try: os.kill(pid,0); return True
    except ProcessLookupError: return False


def test_dashboard_loss_does_not_own_gateway_but_stop_reaps_child(tmp_path):
    socket_path=tmp_path/"gateway.sock"; pid_path=tmp_path/"child.pid"
    gateway=subprocess.Popen([sys.executable,"-m","l515_dashboard.tests.fake_gateway_process",str(socket_path),str(pid_path)])
    try:
        deadline=time.monotonic()+3
        while (not socket_path.exists() or not pid_path.exists()) and time.monotonic()<deadline: time.sleep(.01)
        child=int(pid_path.read_text()); assert gateway.poll() is None and alive(child)
        for sig in (signal.SIGHUP, signal.SIGKILL):
            dashboard=subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"])
            os.kill(dashboard.pid,sig); dashboard.wait(timeout=2)
            assert gateway.poll() is None and alive(child)
        snap=GatewayClient(socket_path).request("stop_gateway")
        assert snap.payload["accepted"] is True
        gateway.wait(timeout=3)
        deadline=time.monotonic()+2
        while alive(child) and time.monotonic()<deadline: time.sleep(.01)
        assert not alive(child) and not socket_path.exists()
    finally:
        if gateway.poll() is None: gateway.terminate(); gateway.wait(timeout=3)
