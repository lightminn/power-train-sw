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


# Demo mode D2 intent machine (docs/specs/2026-11-06-demo-mode.md §3).
class DemoChild(Child):
    def __init__(self):
        super().__init__()
        self.estop_events = 0
        self.generation = 1
    def snapshot(self):
        return dict(super().snapshot(), estop_events=self.estop_events, child_generation=self.generation)


class DemoSession(Session):
    def __init__(self, _config=None):
        super().__init__()
        self.fields = []
        self.snapshot.update(boot_id="boot-1", stop_seq=0, stop_cause="", ops_state=dict(
            demo_mode=True, chassis_mode="IDLE", disarm_seq=0, disarm_cause="",
            authority_mode="IDLE", gateway_state="DRIVE"))
    def request(self, op, **fields):
        self.fields.append((op, fields))
        if op == "start":
            self.snapshot.update(status="SUCCEEDED", action="start")
            self.snapshot["ops_state"] = dict(self.snapshot["ops_state"], chassis_mode="ARMED",
                                              authority_mode="TELEOP")
        return super().request(op)
    def demo_fault(self):
        ops = self.snapshot["ops_state"]
        self.snapshot["ops_state"] = dict(ops, chassis_mode="IDLE", disarm_seq=ops["disarm_seq"] + 1,
                                          disarm_cause="demo_fault")


def _keep_live(runner, predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        runner.set_front_live(True)
        time.sleep(.02)
    return predicate()


@pytest.fixture
def demo_runtime():
    session, child = DemoSession(), DemoChild()
    runner = OperationRuntime(dict(demo_mode=True), session_factory=lambda _: session, supervisor=child)
    wait_for(lambda: runner.snapshot()["connected"])
    assert _keep_live(runner, lambda: runner.snapshot()["ready"])
    assert runner.begin_start()
    assert _keep_live(runner, lambda: runner.snapshot()["auto_resume"]["intent"])
    yield runner, session, child
    runner.close()


def test_demo_fault_stop_auto_starts_with_compare_fields(demo_runtime):
    runner, session, child = demo_runtime
    session.demo_fault()
    assert _keep_live(runner, lambda: [f for op, f in session.fields if op == "start" and f])
    (auto,) = [f["auto"] for op, f in session.fields if op == "start" and f]
    assert auto == dict(expect_stop_seq=0, expect_disarm_seq=1)


@pytest.mark.parametrize("change", ["estop", "generation"])
def test_pad_estop_or_child_restart_cancels_the_intent(demo_runtime, change):
    runner, session, child = demo_runtime
    if change == "estop":
        child.estop_events += 1
    else:
        child.generation += 1
    assert _keep_live(runner, lambda: not runner.snapshot()["auto_resume"]["intent"])
    session.demo_fault()
    _keep_live(runner, lambda: False, timeout=2.5)
    assert not [f for op, f in session.fields if op == "start" and f]


def test_demo_auto_start_needs_live_video_even_though_manual_start_does_not(demo_runtime):
    runner, session, child = demo_runtime
    session.demo_fault()
    time.sleep(2.5)  # no front video
    assert not [f for op, f in session.fields if op == "start" and f]
    assert "영상" in runner.snapshot()["auto_resume"]["note"]


def test_supervisor_counts_child_generations(tmp_path):
    from operator_console.operation_runtime import ControllerSupervisor
    supervisor = ControllerSupervisor("/bin/false")
    try:
        wait_for(lambda: supervisor.snapshot().get("child_generation", 0) >= 1)
        first = supervisor.snapshot()["child_generation"]
        wait_for(lambda: supervisor.snapshot()["child_generation"] > first, timeout=5)
    finally:
        supervisor.close()


def _auto_starts(session):
    return [f for op, f in session.fields if op == "start" and f]


def _plain_starts_after(session, index):
    return [op for op, f in session.fields[index:] if op == "start" and not f]


def test_cancel_during_auto_hold_never_turns_into_a_plain_start(demo_runtime):
    runner, session, child = demo_runtime
    session.demo_fault()
    assert _keep_live(runner, lambda: runner.snapshot()["holding"])
    index = len(session.fields)
    runner.cancel_drive_intent("콘솔 E-stop")
    _keep_live(runner, lambda: False, timeout=2.5)
    assert not _auto_starts(session)
    assert not _plain_starts_after(session, index)
    assert "start_cancel" in [op for op, _ in session.fields[index:]]


def test_hold_seen_while_armed_cancels_the_intent(demo_runtime):
    runner, session, child = demo_runtime
    session.snapshot["ops_state"] = dict(session.snapshot["ops_state"], authority_mode="MOTION_HOLD")
    assert _keep_live(runner, lambda: not runner.snapshot()["auto_resume"]["intent"])
    session.snapshot["ops_state"] = dict(session.snapshot["ops_state"], authority_mode="TELEOP")
    session.demo_fault()
    _keep_live(runner, lambda: False, timeout=2.5)
    assert not _auto_starts(session)


def test_intent_expires_120s_after_a_connected_disarm(demo_runtime):
    runner, session, child = demo_runtime
    session.demo_fault()          # no front video from here on: the auto start waits
    wait_for(lambda: (runner._intent or {}).get("since") is not None)
    with runner._lock:
        runner._intent["since"] = time.monotonic() - 121
    wait_for(lambda: not runner.snapshot()["auto_resume"]["intent"])
    assert "120" in runner.snapshot()["auto_resume"]["note"]


def test_cancel_racing_the_auto_start_send_is_followed_by_a_stop(demo_runtime):
    runner, session, child = demo_runtime
    original = session.request

    def request(op, **fields):
        result = original(op, **fields)
        if op == "start" and fields:
            runner.cancel_drive_intent("콘솔 E-stop")   # lands while the send is in flight
        return result
    session.request = request
    session.demo_fault()
    assert _keep_live(runner, lambda: _auto_starts(session))
    index = [i for i, (op, f) in enumerate(session.fields) if op == "start" and f][-1]
    wait_for(lambda: "stop" in [op for op, _ in session.fields[index + 1:]])


def test_cancel_just_before_the_send_lock_is_released_still_stops(demo_runtime):
    runner, session, child = demo_runtime
    real = runner._send_lock
    fired = []

    class Lock:
        def acquire(self, blocking=True):
            return real.acquire(blocking)

        def release(self):
            if not fired and _auto_starts(session):
                fired.append(True)
                runner.cancel_drive_intent("콘솔 E-stop")   # after the compensate read
            real.release()

        __enter__ = lambda self: self.acquire()
        __exit__ = lambda self, *exc: self.release()
    runner._send_lock = Lock()
    session.demo_fault()
    assert _keep_live(runner, lambda: _auto_starts(session))
    index = [i for i, (op, f) in enumerate(session.fields) if op == "start" and f][-1]
    wait_for(lambda: "stop" in [op for op, _ in session.fields[index + 1:]])
