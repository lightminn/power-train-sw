import threading
import time

import pytest

from chassis.teleop_dualsense import (
    cleanup_chassis_resources,
    handle_chassis_square,
)
from corner_module.teleop_dualsense import handle_corner_square
from safety_us100.background_monitor import BackgroundSafetyMonitor
from safety_us100.teleop_odrive_only import handle_odrive_square
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


def test_chassis_cleanup_does_not_close_sensor_while_worker_is_alive():
    events = []

    class Background:
        def close(self):
            events.append("background")
            return False

    class Sensor:
        def close(self):
            events.append("sensor")

    errors = cleanup_chassis_resources(None, Background(), Sensor())

    assert events == ["background"]
    assert len(errors) == 1
    assert "still running" in str(errors[0])
