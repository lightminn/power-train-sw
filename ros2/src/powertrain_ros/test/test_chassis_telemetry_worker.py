from pathlib import Path
import threading
import time

from powertrain_ros import chassis_telemetry


def _worker_type():
    return getattr(chassis_telemetry, "LatestPollWorker", None)


def test_slow_poll_does_not_block_periodic_latest_cache_reads():
    worker_type = _worker_type()
    assert worker_type is not None, "latest-only poll worker is missing"
    poll_started = threading.Event()

    def slow_poll():
        poll_started.set()
        time.sleep(0.5)
        return {"state": "RUNNING"}

    worker = worker_type(slow_poll, period_s=1.0, name="slow-test")
    try:
        assert poll_started.wait(0.2)
        read_times = []
        for _ in range(6):
            worker.latest()
            read_times.append(time.monotonic())
            time.sleep(0.05)
        gaps = [right - left for left, right in zip(read_times, read_times[1:])]
        assert max(gaps) < 0.10
    finally:
        assert worker.close(join_timeout_s=1.0)


def test_poll_exception_is_cached_as_unavailable_and_worker_survives():
    worker_type = _worker_type()
    assert worker_type is not None, "latest-only poll worker is missing"
    attempts = 0

    def flaky_poll():
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise RuntimeError("poll exploded")
        return {"state": "RUNNING"}

    worker = worker_type(flaky_poll, period_s=0.01, name="flaky-test")
    try:
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            cached = worker.latest()
            if cached.value == {"state": "RUNNING"}:
                break
            time.sleep(0.005)
        assert attempts >= 2
        assert cached.value == {"state": "RUNNING"}
        assert cached.error is None
    finally:
        assert worker.close(join_timeout_s=1.0)


def test_sender_node_uses_workers_bounded_encoder_and_no_poll_timers():
    source = (
        Path(__file__).parents[1]
        / "powertrain_ros"
        / "chassis_telemetry_sender_node.py"
    ).read_text(encoding="utf-8")
    assert "LatestPollWorker" in source
    assert "encode_telemetry_payload" in source
    assert "create_timer(1.0, self._poll_l515)" not in source
    assert "create_timer(1.0, self._poll_observability)" not in source
    assert "join_timeout_s=1.0" in source
