import json
import os

import pytest

from l515_dashboard.resource_guard import ResourceBusy, ResourceGuard


def test_two_contenders_have_exactly_one_owner(tmp_path):
    path = tmp_path / "gateway.lock"
    first, second = ResourceGuard(path), ResourceGuard(path)
    first.acquire()
    try:
        with pytest.raises(ResourceBusy):
            second.acquire()
    finally:
        first.release()


def test_release_keeps_file_and_allows_reacquire(tmp_path):
    path = tmp_path / "gateway.lock"
    first = ResourceGuard(path); first.acquire(); first.release()
    assert path.is_file()
    second = ResourceGuard(path); second.acquire(); second.release()
    assert path.is_file()


def test_stale_metadata_is_overwritten_only_while_locked(tmp_path):
    path = tmp_path / "gateway.lock"
    path.write_text('{"pid":99999999,"start_identity":"stale"}')
    guard = ResourceGuard(path); guard.acquire()
    payload = json.loads(path.read_text())
    assert payload["pid"] == os.getpid()
    guard.release()


def test_symlink_lock_is_rejected_without_touching_target(tmp_path):
    target = tmp_path / "unknown"
    target.write_text("preserve")
    link = tmp_path / "gateway.lock"
    link.symlink_to(target)
    with pytest.raises(ResourceBusy):
        ResourceGuard(link).acquire()
    assert target.read_text() == "preserve"


def test_release_never_unlinks_lock(tmp_path, monkeypatch):
    path = tmp_path / "gateway.lock"
    guard = ResourceGuard(path); guard.acquire()
    calls = []
    monkeypatch.setattr(os, "unlink", lambda value: calls.append(value))
    guard.release()
    assert calls == [] and path.exists()


def test_runtime_directory_accepts_0750_and_rejects_0755(tmp_path):
    secure = tmp_path / "secure"
    secure.mkdir(mode=0o750)
    guard = ResourceGuard(secure / "gateway.lock")
    guard.acquire(); guard.release()

    unsafe = tmp_path / "unsafe"
    unsafe.mkdir(mode=0o755)
    with pytest.raises(ResourceBusy, match="directory permissions"):
        ResourceGuard(unsafe / "gateway.lock").acquire()
