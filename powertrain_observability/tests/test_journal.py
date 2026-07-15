import threading

from powertrain_observability.health import HealthState
from powertrain_observability.journal import (
    BoundedEventQueue,
    MissionJournal,
    recover_records,
)


def _pending_event(**overrides):
    event = {
        "schema_version": 1,
        "wall_time_ns": 1_750_000_000_000_000_000,
        "monotonic_ns": 123_456_789,
        "source": "mission_supervisor",
        "event_type": "MISSION",
        "severity": "INFO",
        "payload": {"state": "RUNNING"},
    }
    event.update(overrides)
    return event


def test_open_append_flush_assigns_run_and_sequence(tmp_path):
    journal = MissionJournal(tmp_path, run_id="run-a")

    first = journal.append(_pending_event(run_id="producer", sequence=99))
    second = journal.append(_pending_event(payload={"state": "DONE"}))

    assert journal.flush() is True
    path = journal.path
    journal.close()

    assert first["run_id"] == "run-a"
    assert second["run_id"] == "run-a"
    assert [first["sequence"], second["sequence"]] == [0, 1]
    assert recover_records(path) == [first, second]


def test_each_run_uses_a_separate_jsonl_file(tmp_path):
    first = MissionJournal(tmp_path, run_id="run-a")
    second = MissionJournal(tmp_path, run_id="run-b")

    first.close()
    second.close()

    assert first.path != second.path
    assert first.path.exists()
    assert second.path.exists()


def test_rotate_keeps_sequence_monotonic_across_segments(tmp_path):
    journal = MissionJournal(tmp_path, run_id="run-rotate", max_file_bytes=1)

    first = journal.append(_pending_event())
    second = journal.append(_pending_event(payload={"state": "NEXT"}))
    journal.close()

    paths = sorted(tmp_path.glob("run-rotate.*.jsonl"))
    assert len(paths) == 2
    assert recover_records(paths[0]) == [first]
    assert recover_records(paths[1]) == [second]
    assert [first["sequence"], second["sequence"]] == [0, 1]


def test_reopen_ignores_and_truncates_last_partial_line(tmp_path):
    journal = MissionJournal(tmp_path, run_id="run-recover")
    first = journal.append(_pending_event())
    journal.close()
    path = journal.path

    with path.open("ab") as stream:
        stream.write(b'{"schema_version":1,"run_id":"incomplete')

    reopened = MissionJournal(tmp_path, run_id="run-recover")
    second = reopened.append(_pending_event(payload={"state": "RECOVERED"}))
    reopened.close()

    assert [first["sequence"], second["sequence"]] == [0, 1]
    assert recover_records(path) == [first, second]
    assert path.read_bytes().endswith(b"\n")


def test_full_bounded_queue_never_blocks_producer_and_only_counts_drop():
    health = HealthState()
    queue = BoundedEventQueue(capacity=1, health=health)
    assert queue.offer({"event": 1}) is True

    result = []
    producer = threading.Thread(
        target=lambda: result.append(queue.offer({"event": 2})),
        daemon=True,
    )
    producer.start()
    producer.join(timeout=0.2)

    assert producer.is_alive() is False
    assert result == [False]
    assert queue.get_nowait() == {"event": 1}
    snapshot = health.snapshot()
    assert snapshot.status == "OK"
    assert snapshot.drop_count == 1


class _FlushFailingFile:
    def __init__(self, wrapped):
        self._wrapped = wrapped

    def flush(self):
        raise OSError("simulated disk failure")

    def __getattr__(self, name):
        return getattr(self._wrapped, name)


def test_flush_error_degrades_health_without_stopping_producer(tmp_path):
    health = HealthState()
    queue = BoundedEventQueue(capacity=1, health=health)
    journal = MissionJournal(tmp_path, run_id="run-flush", health=health)
    journal.append(_pending_event())
    journal._file = _FlushFailingFile(journal._file)

    assert journal.flush() is False
    assert queue.offer({"event": "still accepted"}) is True
    assert health.snapshot().status == "DEGRADED"
    assert "simulated disk failure" in health.snapshot().last_error
    journal.close()
