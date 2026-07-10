import threading
import time

import pytest

from chassis.teleop_dualsense import (
    cleanup_chassis_resources,
    handle_chassis_square,
)
from chassis.teleop_server import (
    WIRELESS_RX_TIMEOUT_MS,
    apply_wireless_command,
    control_thread_failure,
    fresh_wireless_input,
    reset_wireless_input,
    run_control_thread,
    shutdown_control_resources,
    update_wireless_input,
)
from corner_module.teleop_dualsense import handle_corner_square
from safety_us100.background_monitor import BackgroundSafetyMonitor
from safety_us100.teleop_odrive_only import (
    confirm_odrive_closed_loop,
    handle_odrive_square,
)
from safety_us100.verdict import VALID, Verdict


class StaticMonitor:
    def tick(self):
        pass

    def verdict(self):
        return Verdict(VALID, 500.0, False, 0, "far")


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.lock = threading.Lock()

    def __call__(self):
        with self.lock:
            return self.now

    def advance(self, seconds):
        with self.lock:
            self.now += seconds


def wait_until(predicate, timeout_s=0.3):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.001)
    return predicate()


def test_background_monitor_samples_without_blocking_caller():
    sampled = threading.Event()

    class Monitor:
        def tick(self):
            sampled.set()

        def verdict(self):
            return Verdict(VALID, 500.0, False, 0, "far")

    worker = BackgroundSafetyMonitor(Monitor(), period_s=0.01)
    worker.start()
    try:
        assert sampled.wait(0.2)
        assert worker.verdict().status == VALID
    finally:
        worker.close()


@pytest.mark.parametrize("period_s", [0.0, -0.01, float("nan"), float("inf")])
def test_background_monitor_rejects_nonpositive_or_nonfinite_period(period_s):
    with pytest.raises(ValueError, match="period_s"):
        BackgroundSafetyMonitor(StaticMonitor(), period_s=period_s)


@pytest.mark.parametrize(
    "stale_timeout_s",
    [0.0, -0.01, float("nan"), float("inf")],
)
def test_background_monitor_rejects_invalid_stale_timeout(stale_timeout_s):
    with pytest.raises(ValueError, match="stale_timeout_s"):
        BackgroundSafetyMonitor(
            StaticMonitor(),
            stale_timeout_s=stale_timeout_s,
        )


def test_monitor_exception_publishes_fail_safe_and_worker_recovers():
    failed = threading.Event()
    allow_recovery = threading.Event()

    class RecoveringMonitor:
        def __init__(self):
            self.calls = 0

        def tick(self):
            self.calls += 1
            if self.calls == 1:
                failed.set()
                raise RuntimeError("serial exploded")
            allow_recovery.wait(0.5)

        def verdict(self):
            return Verdict(VALID, 500.0, False, 0, "far")

    worker = BackgroundSafetyMonitor(RecoveringMonitor(), period_s=0.01)
    worker.start()
    try:
        assert failed.wait(0.2)
        assert wait_until(lambda: worker.verdict().estop_required)
        failed_verdict = worker.verdict()
        assert failed_verdict.status == "NO_RESPONSE"
        assert failed_verdict.distance_mm is None
        assert "RuntimeError" in failed_verdict.detail

        allow_recovery.set()
        assert wait_until(lambda: worker.verdict().status == VALID)
        assert worker.verdict().estop_required is False
    finally:
        allow_recovery.set()
        worker.close()


def test_initial_monitor_exception_is_fail_safe_and_recovers_after_start():
    class InitiallyFailingMonitor(StaticMonitor):
        def __init__(self):
            self.verdict_calls = 0

        def verdict(self):
            self.verdict_calls += 1
            if self.verdict_calls == 1:
                raise RuntimeError("initial verdict failed")
            return super().verdict()

    worker = BackgroundSafetyMonitor(
        InitiallyFailingMonitor(),
        period_s=0.01,
    )
    initial = worker.verdict()
    assert initial.status == "NO_RESPONSE"
    assert initial.estop_required is True
    assert "RuntimeError" in initial.detail

    worker.start()
    try:
        assert wait_until(lambda: worker.verdict().status == VALID)
    finally:
        worker.close()


def test_unprintable_monitor_exception_cannot_kill_worker():
    raised = threading.Event()
    allow_recovery = threading.Event()

    class UnprintableError(RuntimeError):
        def __str__(self):
            raise ValueError("cannot render error")

    class Monitor(StaticMonitor):
        def __init__(self):
            self.calls = 0

        def tick(self):
            self.calls += 1
            if self.calls == 1:
                raised.set()
                raise UnprintableError()
            allow_recovery.wait(0.5)

    worker = BackgroundSafetyMonitor(Monitor(), period_s=0.01)
    worker.start()
    try:
        assert raised.wait(0.2)
        assert wait_until(lambda: worker.verdict().estop_required)
        assert "UnprintableError" in worker.verdict().detail
        allow_recovery.set()
        assert wait_until(lambda: worker.verdict().status == VALID)
    finally:
        allow_recovery.set()
        worker.close()


def test_hung_worker_verdict_becomes_fail_safe_when_stale():
    sampled = threading.Event()
    hung = threading.Event()
    release = threading.Event()
    clock = FakeClock()

    class HungAfterSampleMonitor(StaticMonitor):
        def __init__(self):
            self.calls = 0

        def tick(self):
            self.calls += 1
            if self.calls == 1:
                sampled.set()
                return
            hung.set()
            release.wait(0.5)

    worker = BackgroundSafetyMonitor(
        HungAfterSampleMonitor(),
        period_s=0.01,
        stale_timeout_s=0.75,
        clock=clock,
    )
    worker.start()
    try:
        assert sampled.wait(0.2)
        assert worker.verdict().status == VALID
        assert hung.wait(0.2)

        clock.advance(0.751)
        started = time.monotonic()
        verdict = worker.verdict()
        assert time.monotonic() - started < 0.2
        assert verdict.status == "NO_RESPONSE"
        assert verdict.estop_required is True
        assert verdict.distance_mm is None
        assert verdict.detail == "background_stale"
    finally:
        release.set()
        worker.close()


def test_start_close_are_idempotent_nonblocking_and_preserve_ownership():
    entered = threading.Event()
    release = threading.Event()
    caller_thread = threading.get_ident()

    class OwnedSensor:
        def __init__(self):
            self.close_calls = 0

        def close(self):
            self.close_calls += 1

    class BlockingMonitor(StaticMonitor):
        def __init__(self):
            self.tick_calls = 0
            self.tick_threads = []
            self.close_calls = 0
            self.sensor = OwnedSensor()

        def tick(self):
            self.tick_calls += 1
            self.tick_threads.append(threading.get_ident())
            entered.set()
            release.wait(0.5)

        def close(self):
            self.close_calls += 1

    monitor = BlockingMonitor()
    worker = BackgroundSafetyMonitor(monitor, period_s=1.0)

    started = time.monotonic()
    worker.start()
    worker.start()
    assert time.monotonic() - started < 0.2
    assert entered.wait(0.2)
    assert monitor.tick_calls == 1
    assert monitor.tick_threads == [worker._thread.ident]
    assert monitor.tick_threads[0] != caller_thread

    started = time.monotonic()
    assert worker.verdict().status == VALID
    assert time.monotonic() - started < 0.2

    release.set()
    worker.close()
    worker.close()
    assert monitor.close_calls == 0
    assert monitor.sensor.close_calls == 0

    worker.start()
    time.sleep(0.02)
    assert monitor.tick_calls == 1
    assert worker._thread is None


def test_close_keeps_live_thread_visible_and_allows_retry(monkeypatch):
    entered = threading.Event()
    release = threading.Event()

    class HungMonitor(StaticMonitor):
        def tick(self):
            entered.set()
            release.wait()

    worker = BackgroundSafetyMonitor(HungMonitor(), period_s=1.0)
    monkeypatch.setattr(worker, "_close_timeout_s", 0.01)
    worker.start()
    assert entered.wait(0.2)

    assert worker.close() is False
    assert worker._thread is not None
    assert worker._thread.is_alive()

    release.set()
    assert wait_until(lambda: not worker._thread.is_alive())
    assert worker.close() is True
    assert worker._thread is None


class LifecycleDouble:
    def __init__(self, mode, arm_result=True):
        self.mode = mode
        self.arm_result = arm_result
        self.calls = []

    def reset_estop(self):
        self.calls.append("reset_estop")
        self.mode = "IDLE"
        return True

    def reset_fault(self):
        self.calls.append("reset_fault")
        self.mode = "IDLE"
        return True

    def arm(self):
        self.calls.append("arm")
        if self.arm_result:
            self.mode = "ARMED"
        return self.arm_result

    def disarm(self):
        self.calls.append("disarm")
        self.mode = "IDLE"


def test_chassis_square_resets_then_requires_a_separate_press_to_arm():
    manager = LifecycleDouble("ESTOP")

    assert handle_chassis_square(manager) is True
    assert manager.mode == "IDLE"
    assert manager.calls == ["reset_estop"]

    assert handle_chassis_square(manager) is True
    assert manager.mode == "ARMED"
    assert manager.calls == ["reset_estop", "arm"]


def test_chassis_square_honors_rejected_arm_result():
    manager = LifecycleDouble("IDLE", arm_result=False)

    assert handle_chassis_square(manager) is False
    assert manager.mode == "IDLE"
    assert manager.calls == ["arm"]


def test_corner_square_resets_fault_then_requires_separate_arm_press():
    corner = LifecycleDouble("FAULT")

    assert handle_corner_square(corner) is True
    assert corner.mode == "IDLE"
    assert corner.calls == ["reset_fault"]

    assert handle_corner_square(corner) is True
    assert corner.mode == "ARMED"
    assert corner.calls == ["reset_fault", "arm"]


def test_odrive_square_rejects_active_hazard_then_clears_only_latch():
    calls = []

    def arm():
        calls.append("arm")
        return True

    def disarm():
        calls.append("disarm")

    state = handle_odrive_square(
        armed=False,
        estop_latched=True,
        hazard_active=True,
        arm=arm,
        disarm=disarm,
    )
    assert state == (False, True)
    assert calls == ["disarm"]

    state = handle_odrive_square(
        armed=False,
        estop_latched=True,
        hazard_active=False,
        arm=arm,
        disarm=disarm,
    )
    assert state == (False, False)
    assert calls == ["disarm", "disarm"]

    state = handle_odrive_square(
        armed=False,
        estop_latched=False,
        hazard_active=False,
        arm=arm,
        disarm=disarm,
    )
    assert state == (True, False)
    assert calls == ["disarm", "disarm", "arm"]


def test_chassis_cleanup_estops_and_continues_in_required_resource_order():
    events = []

    class Closeable:
        def __init__(self, name, fails=False):
            self.name = name
            self.fails = fails

        def close(self):
            events.append(self.name)
            if self.fails:
                raise RuntimeError(f"{self.name} failed")

    class Manager:
        def __init__(self):
            self.corners = {
                "first": Closeable("corner_first", fails=True),
                "second": Closeable("corner_second"),
            }
            self.mode = "ARMED"

        def estop(self, source, detail):
            events.append((source, detail))
            raise RuntimeError("estop failed")

    errors = cleanup_chassis_resources(
        Manager(),
        Closeable("background", fails=True),
        Closeable("sensor"),
    )

    assert events == [
        ("teleop_shutdown", "teleop cleanup"),
        "background",
        "corner_first",
        "corner_second",
    ]
    assert [type(error) for error in errors] == [
        RuntimeError,
        RuntimeError,
        RuntimeError,
    ]


@pytest.mark.parametrize("close_result", [False, None])
def test_chassis_cleanup_requires_explicit_worker_stop_before_sensor_close(
    close_result,
):
    events = []

    class Background:
        def close(self):
            events.append("background")
            return close_result

    class Sensor:
        def close(self):
            events.append("sensor")

    errors = cleanup_chassis_resources(None, Background(), Sensor())

    assert events == ["background"]
    assert len(errors) == 1
    assert "still running" in str(errors[0])


def test_wireless_silence_does_not_apply_cached_command_or_refresh_watchdog():
    state = {}
    reset_wireless_input(state)
    assert update_wireless_input(
        state,
        (0.0, 0.0, 0.0, 0, 0),
        now_ms=100.0,
    ) is True
    assert update_wireless_input(
        state,
        (0.0, 1.0, 0.0, 0, 0),
        now_ms=110.0,
    ) is True

    cached = fresh_wireless_input(state, now_ms=110.0)
    assert cached == (0.0, 1.0, 0.0, 0, 0)

    class Manager:
        mode = "ARMED"

        def __init__(self):
            self.set_calls = []
            self.now_ms = 110.0
            self.last_set_ms = None

        def set(self, v, omega):
            self.set_calls.append((v, omega))
            self.last_set_ms = self.now_ms

    manager = Manager()
    assert apply_wireless_command(
        manager,
        cached,
        v_max=1.0,
        omega_max=1.0,
    )[2] is True
    assert manager.set_calls == [(1.0, 0.0)]

    replay = fresh_wireless_input(state, now_ms=120.0)
    assert apply_wireless_command(
        manager,
        replay,
        v_max=1.0,
        omega_max=1.0,
    )[2] is False
    assert replay is None
    assert manager.set_calls == [(1.0, 0.0)]
    assert manager.last_set_ms == 110.0

    stale = fresh_wireless_input(
        state,
        now_ms=110.0 + WIRELESS_RX_TIMEOUT_MS + 0.1,
    )
    assert apply_wireless_command(
        manager,
        stale,
        v_max=1.0,
        omega_max=1.0,
    )[2] is False
    assert stale is None
    assert manager.set_calls == [(1.0, 0.0)]
    manager.now_ms = 110.0 + WIRELESS_RX_TIMEOUT_MS + 0.1
    assert manager.now_ms - manager.last_set_ms > WIRELESS_RX_TIMEOUT_MS

    assert update_wireless_input(
        state,
        (0.0, 0.5, 0.0, 0, 0),
        now_ms=500.0,
    ) is True
    assert fresh_wireless_input(state, now_ms=500.0) is not None


def test_reconnect_held_square_waits_for_release_then_new_press():
    manager = LifecycleDouble("ESTOP")
    assert handle_chassis_square(manager) is True
    assert manager.mode == "IDLE"

    state = {}
    reset_wireless_input(state)
    assert update_wireless_input(
        state,
        (0.0, 0.0, 0.0, 1, 0),
        now_ms=10.0,
    ) is False
    assert fresh_wireless_input(state, now_ms=10.0) is None
    assert manager.calls == ["reset_estop"]

    assert update_wireless_input(
        state,
        (0.0, 0.0, 0.0, 0, 0),
        now_ms=20.0,
    ) is True
    assert fresh_wireless_input(state, now_ms=20.0)[3] == 0
    assert update_wireless_input(
        state,
        (0.0, 0.0, 0.0, 1, 0),
        now_ms=30.0,
    ) is True
    assert fresh_wireless_input(state, now_ms=30.0)[3] == 1
    assert handle_chassis_square(manager) is True
    assert manager.mode == "ARMED"


@pytest.mark.parametrize(
    "held_sample",
    [
        (0.0, 0.1, 0.0, 0, 0),
        (0.0, 0.0, 0.1, 0, 0),
        (0.1, 0.0, 0.0, 0, 0),
    ],
)
def test_reconnect_motion_input_cannot_open_neutral_gate(held_sample):
    state = {}
    reset_wireless_input(state)

    assert update_wireless_input(state, held_sample, now_ms=10.0) is False
    assert fresh_wireless_input(state, now_ms=10.0) is None

    assert update_wireless_input(
        state,
        (0.04, 0.04, 0.04, 0, 0),
        now_ms=20.0,
    ) is True


@pytest.mark.parametrize(
    "error",
    [RuntimeError("ordinary"), pytest.param(None, id="unprintable")],
)
def test_control_thread_latches_and_exposes_exception(error):
    if error is None:
        class UnprintableError(RuntimeError):
            def __str__(self):
                raise ValueError("cannot render")

        error = UnprintableError()
    stop = threading.Event()
    failed = threading.Event()
    failure = {}

    class Manager:
        def __init__(self):
            self.estops = []

        def estop(self, source, detail):
            self.estops.append((source, detail))

    manager = Manager()

    def step():
        raise error

    run_control_thread(step, stop, manager, failure, failed)

    assert failed.is_set()
    assert failure["exception"] is error
    assert "control_exception" == manager.estops[0][0]
    assert manager.estops[-1][0] == "control_thread_exit"
    assert stop.is_set()


def test_control_thread_catches_direct_baseexception_and_reports_dead_thread():
    class DirectFailure(BaseException):
        pass

    stop = threading.Event()
    failed = threading.Event()
    failure = {}

    class Manager:
        def __init__(self):
            self.estops = []

        def estop(self, source, detail):
            self.estops.append((source, detail))

    manager = Manager()
    thread = threading.Thread(
        target=run_control_thread,
        args=(lambda: (_ for _ in ()).throw(DirectFailure("direct")),
              stop, manager, failure, failed),
    )
    thread.start()
    thread.join(0.2)

    assert not thread.is_alive()
    assert isinstance(failure["exception"], DirectFailure)
    assert "DirectFailure" in control_thread_failure(thread, failure)


def test_control_failure_is_visible_even_while_thread_finishes_estop():
    class StillAlive:
        def is_alive(self):
            return True

    assert control_thread_failure(
        StillAlive(),
        {"detail": "RuntimeError: failed"},
    ) == "RuntimeError: failed"


def test_shutdown_keeps_resources_open_until_control_thread_exits():
    entered = threading.Event()
    release = threading.Event()
    stop = threading.Event()
    failure = {}
    failed = threading.Event()
    events = []

    class Corner:
        def close(self):
            events.append("corner_close")

    class Manager:
        def __init__(self):
            self.estops = []
            self.corners = {"corner": Corner()}
            self.mode = "ARMED"

        def estop(self, source, detail):
            self.estops.append((source, detail))

    class Background:
        def close(self):
            events.append("background_close")
            return True

    class Sensor:
        def close(self):
            events.append("sensor_close")

    manager = Manager()

    def delayed_step():
        entered.set()
        release.wait()

    thread = threading.Thread(
        target=run_control_thread,
        args=(delayed_step, stop, manager, failure, failed),
    )
    thread.start()
    assert entered.wait(0.2)

    stopped, errors = shutdown_control_resources(
        thread,
        stop,
        manager,
        Background(),
        Sensor(),
        join_timeout_s=0.01,
    )
    assert stopped is False
    assert errors
    assert events == []
    assert manager.estops[-1][0] == "shutdown_timeout"

    release.set()
    thread.join(0.2)
    assert not thread.is_alive()
    assert any(source == "control_thread_exit"
               for source, _ in manager.estops)

    stopped, errors = shutdown_control_resources(
        thread,
        stop,
        manager,
        Background(),
        Sensor(),
        join_timeout_s=0.01,
    )
    assert stopped is True
    assert errors == []
    assert events == ["background_close", "sensor_close", "corner_close"]


def test_unstarted_control_thread_shutdown_cleans_without_join_error():
    stop = threading.Event()
    events = []

    class Resource:
        def __init__(self, name):
            self.name = name

        def close(self):
            events.append(self.name)
            return True

    class Manager:
        def __init__(self):
            self.corners = {"corner": Resource("corner")}
            self.mode = "IDLE"

        def estop(self, source, detail):
            pass

    thread = threading.Thread(target=lambda: None)
    stopped, errors = shutdown_control_resources(
        thread,
        stop,
        Manager(),
        Resource("background"),
        Resource("sensor"),
        join_timeout_s=0.01,
    )
    assert stopped is True
    assert errors == []
    assert events == ["background", "sensor", "corner"]


def test_background_start_failure_does_not_store_unstarted_thread(monkeypatch):
    worker = BackgroundSafetyMonitor(StaticMonitor())
    sensor_closed = []

    class Sensor:
        def close(self):
            sensor_closed.append(True)

    def fail_start(_thread):
        raise RuntimeError("thread start failed")

    monkeypatch.setattr(threading.Thread, "start", fail_start)
    with pytest.raises(RuntimeError, match="thread start failed"):
        worker.start()

    assert worker._thread is None
    assert worker.close() is True
    errors = cleanup_chassis_resources(None, worker, Sensor())
    assert errors == []
    assert sensor_closed == [True]


def test_odrive_failed_arm_stays_stopped_and_latches_estop():
    calls = []

    state = handle_odrive_square(
        armed=False,
        estop_latched=False,
        hazard_active=False,
        arm=lambda: False,
        disarm=lambda: calls.append("disarm"),
    )

    assert state == (False, True)
    assert calls == ["disarm"]


def test_odrive_arm_confirmation_waits_for_closed_loop():
    class Axis:
        def __init__(self):
            self.requested_state = None
            self.states = iter([1, 1, 8])

        @property
        def current_state(self):
            return next(self.states)

    clock = FakeClock()
    axis = Axis()

    confirmed = confirm_odrive_closed_loop(
        axis,
        closed_loop_state=8,
        timeout_s=0.1,
        clock=clock,
        sleeper=clock.advance,
    )

    assert confirmed is True
    assert axis.requested_state == 8


def test_odrive_arm_confirmation_times_out_without_claiming_armed():
    class Axis:
        requested_state = None
        current_state = 1

    clock = FakeClock()
    axis = Axis()

    confirmed = confirm_odrive_closed_loop(
        axis,
        closed_loop_state=8,
        timeout_s=0.02,
        clock=clock,
        sleeper=clock.advance,
    )

    assert confirmed is False
    assert axis.requested_state == 8
