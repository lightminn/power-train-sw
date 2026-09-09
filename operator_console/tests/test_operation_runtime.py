import time

import pytest

from operator_console.operation_runtime import OperationRuntime, ConsoleInstanceLock


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate()


class Child:
    def __init__(self):
        self.connected = None
        self.neutral = True
        self.closed = False
    def set_connection(self, value):
        self.connected = value
    def set_ops_state(self, state):
        pass
    def snapshot(self):
        return dict(pad_connected=True, input_connected=bool(self.connected), neutral=self.neutral, detail="pad", age_s=0)
    def close(self):
        self.closed = True


class Session:
    def __init__(self, _config=None):
        self.host = "127.0.0.1"
        self.snapshot = dict(connected=True, lease_id="one", ticket="private", robot_id="test",
                             input_port=9000, ops_port=9001, start_blocker=None,
                             status="IDLE", action="none", ops_state={})
        self.operations = []
        self.available = True
    def connect(self):
        if not self.available:
            raise OSError("unavailable")
        return self.snapshot
    def heartbeat(self):
        if not self.available:
            raise OSError("lost")
        return self.snapshot
    def request(self, op):
        self.operations.append(op)
        return self.snapshot
    def close(self):
        self.operations.append("release")
        return dict(status="PENDING")


@pytest.fixture
def runtime():
    session, child = Session(), Child()
    runtime = OperationRuntime({}, session_factory=lambda _: session, supervisor=child)
    wait_for(lambda: runtime.snapshot()["connected"])
    yield runtime, session, child
    runtime.close()


def test_boot_connection_does_not_send_start_and_snapshot_hides_ticket(runtime):
    runner, session, child = runtime
    assert session.operations == []
    assert "ticket" not in runner.snapshot()
    assert child.connected["lease_id"] == "one"


def test_start_requires_live_video_and_neutral_pad(runtime):
    runner, session, child = runtime
    assert runner.begin_start() is False
    runner.set_front_live(True)
    child.neutral = False
    assert runner.begin_start() is False
    assert session.operations == []


def test_release_during_hold_cancels_without_start(runtime):
    runner, session, child = runtime
    runner.set_front_live(True)
    assert runner.begin_start()
    wait_for(lambda: "start_begin" in session.operations)
    runner.cancel_start()
    wait_for(lambda: "start_cancel" in session.operations)
    assert "start" not in session.operations


def test_held_start_once_and_loss_never_restores_intent(runtime):
    runner, session, child = runtime
    runner.set_front_live(True)
    assert runner.begin_start()
    until = time.monotonic() + 2.2
    while time.monotonic() < until:
        runner.set_front_live(True)
        time.sleep(.02)
    assert session.operations.count("start") == 1
    session.available = False
    wait_for(lambda: not runner.snapshot()["connected"])
    assert child.connected is None
    session.available = True
    wait_for(lambda: runner.snapshot()["connected"])
    assert session.operations.count("start") == 1


def test_window_stall_invalidates_readiness_and_cancels_hold(runtime):
    runner, session, child = runtime
    runner.set_front_live(True)
    assert runner.begin_start()
    time.sleep(.8)
    assert "start" not in session.operations
    assert not runner.snapshot()["ready"]


def test_duplicate_console_lock_rejected_then_released(tmp_path):
    with ConsoleInstanceLock(tmp_path / "console.lock"):
        with pytest.raises(RuntimeError):
            ConsoleInstanceLock(tmp_path / "console.lock")
    with ConsoleInstanceLock(tmp_path / "console.lock"):
        pass
