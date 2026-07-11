import json
import os
import threading
import socket as socket_module

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
    socket.write_text("stale")
    stat = socket.stat()
    lock.write_text(json.dumps({"pid": 99999999, "start_identity": "old",
                                "socket_identity": [stat.st_dev, stat.st_ino,
                                                    stat.st_ctime_ns,
                                                    stat.st_mode & 0o170000]}))
    guard = ResourceGuard(lock, socket)
    guard.acquire()
    assert guard.acquired and not socket.exists()
    guard.release()


def test_legacy_two_field_socket_identity_is_not_safe_to_unlink(tmp_path):
    lock = tmp_path / "camera.lock"
    socket = tmp_path / "gateway.sock"
    socket.write_text("successor")
    stat = socket.stat()
    lock.write_text(json.dumps({"pid": 99999999, "start_identity": "old",
                                "socket_identity": [stat.st_dev, stat.st_ino]}))
    guard = ResourceGuard(lock, socket)
    guard.acquire()
    assert socket.read_text() == "successor"
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
    with pytest.raises(ResourceBusy):
        ResourceGuard(lock, tmp_path / "sock").acquire()
    assert calls == []


def test_fully_written_lock_is_published_atomically_under_interleaving(tmp_path):
    lock, socket = tmp_path / "lock", tmp_path / "sock"
    ready, release = threading.Event(), threading.Event()
    first = ResourceGuard(lock, socket)
    first._before_publish = lambda: (ready.set(), release.wait())
    outcomes = []
    t1 = threading.Thread(target=lambda: (first.acquire(), outcomes.append("first")))
    t1.start(); assert ready.wait(1)
    second = ResourceGuard(lock, socket)
    t2 = threading.Thread(target=lambda: _acquire_outcome(second, outcomes, "second"))
    t2.start()
    assert not lock.exists()
    release.set(); t1.join(1); t2.join(1)
    assert len([x for x in outcomes if x in ("first", "second")]) == 1
    json.loads(lock.read_text())
    first.release(); second.release()


def _acquire_outcome(guard, outcomes, winner):
    try:
        guard.acquire(); outcomes.append(winner)
    except ResourceBusy:
        outcomes.append("busy")


def test_release_does_not_remove_unknown_or_replaced_socket(tmp_path):
    lock, path = tmp_path / "lock", tmp_path / "sock"
    guard = ResourceGuard(lock, path); guard.acquire()
    path.write_text("unknown")
    guard.release()
    assert path.exists()

    path.unlink(); guard.acquire(); path.write_text("ours"); guard.claim_socket()
    path.unlink(); path.write_text("successor")
    guard.release()
    assert path.read_text() == "successor"


def test_claim_persists_full_filesystem_identity(tmp_path):
    lock, path = tmp_path / "lock", tmp_path / "sock"
    guard = ResourceGuard(lock, path); guard.acquire()
    path.write_text("ours"); guard.claim_socket()
    stat = path.stat()
    assert json.loads(lock.read_text())["socket_identity"] == [
        stat.st_dev, stat.st_ino, stat.st_ctime_ns, stat.st_mode & 0o170000]
    guard.release()


def test_release_preserves_successor_created_exactly_during_cleanup(
        tmp_path, monkeypatch):
    lock, path = tmp_path / "lock", tmp_path / "sock"
    guard = ResourceGuard(lock, path); guard.acquire()
    path.write_text("owned"); guard.claim_socket()
    real_rename = os.rename

    def replace_during_cleanup(source, destination):
        real_rename(source, destination)
        if str(source) == str(path):
            path.write_text("successor")

    monkeypatch.setattr(os, "rename", replace_during_cleanup)
    guard.release()
    assert path.read_text() == "successor"


def test_stale_reclaim_does_not_remove_replaced_socket(tmp_path):
    lock, path = tmp_path / "lock", tmp_path / "sock"
    path.write_text("old"); old = path.stat()
    lock.write_text(json.dumps({"pid": 99999999, "start_identity": "old",
                                "socket_identity": [old.st_dev, old.st_ino]}))
    path.unlink(); path.write_text("successor")
    guard = ResourceGuard(lock, path); guard.acquire()
    assert path.read_text() == "successor"
    guard.release()


def test_new_runtime_directory_is_secure(tmp_path):
    parent=tmp_path / "runtime"
    guard=ResourceGuard(parent / "lock", parent / "sock")
    guard.acquire()
    assert parent.stat().st_mode & 0o777 in (0o700, 0o750)
    guard.release()
