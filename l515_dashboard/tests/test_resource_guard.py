import json
import os
import threading

import pytest

from l515_dashboard.resource_guard import ResourceBusy, ResourceGuard


def _identity(pid):
    return ResourceGuard.process_start_identity(pid)


def test_live_owner_blocks_second_acquire(tmp_path):
    lock = tmp_path / "camera.lock"
    socket = tmp_path / "gateway.sock"
    first = ResourceGuard(lock, socket)
    first.acquire()
    try:
        with pytest.raises(ResourceBusy):
            ResourceGuard(lock, socket).acquire()
    finally:
        first.release()


def test_stale_lock_and_socket_are_reclaimed(tmp_path):
    lock = tmp_path / "camera.lock"
    socket = tmp_path / "gateway.sock"
    lock.write_text(json.dumps({"pid": 99999999, "start_identity": "old"}))
    socket.write_text("stale")
    guard = ResourceGuard(lock, socket)
    guard.acquire()
    assert guard.acquired and not socket.exists()
    guard.release()


def test_pid_reuse_identity_mismatch_is_stale(tmp_path):
    lock = tmp_path / "camera.lock"
    lock.write_text(json.dumps({"pid": os.getpid(), "start_identity": "not-us"}))
    guard = ResourceGuard(lock, tmp_path / "gateway.sock")
    guard.acquire()
    assert json.loads(lock.read_text())["start_identity"] == _identity(os.getpid())
    guard.release()


def test_concurrent_acquire_has_exactly_one_winner(tmp_path):
    barrier = threading.Barrier(2)
    outcomes = []
    guards = [ResourceGuard(tmp_path / "lock", tmp_path / "sock") for _ in range(2)]
    def run(guard):
        barrier.wait()
        try:
            guard.acquire(); outcomes.append("won")
        except ResourceBusy:
            outcomes.append("busy")
    threads = [threading.Thread(target=run, args=(g,)) for g in guards]
    [t.start() for t in threads]; [t.join() for t in threads]
    assert sorted(outcomes) == ["busy", "won"]
    [g.release() for g in guards]


def test_unknown_owner_is_never_signalled(tmp_path, monkeypatch):
    lock = tmp_path / "lock"
    lock.write_text("not-json")
    calls = []
    monkeypatch.setattr(os, "kill", lambda *args: calls.append(args))
    ResourceGuard(lock, tmp_path / "sock").acquire()
    assert calls == []
