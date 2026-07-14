"""Real SocketCAN owner lock lifecycle tests (WP5.2 Task 3)."""

import dataclasses
import errno
import os
from pathlib import Path
import subprocess
import sys

import pytest

from chassis import runtime_lock
from chassis.runtime_lock import (
    CanOwnerLock,
    CanOwnershipError,
    RealCanSession,
)


_HOLDER = r"""
import sys
from chassis.runtime_lock import RealCanSession

with RealCanSession(channel="can0", owner="cross-process-holder", path=sys.argv[1]):
    print("READY", flush=True)
    sys.stdin.read()
"""


def _start_holder(lock_path: Path) -> subprocess.Popen:
    env = os.environ.copy()
    motor_control = str(Path(__file__).resolve().parents[2])
    env["PYTHONPATH"] = os.pathsep.join(
        part for part in (motor_control, env.get("PYTHONPATH")) if part
    )
    process = subprocess.Popen(
        [sys.executable, "-c", _HOLDER, str(lock_path)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )
    assert process.stdout.readline().strip() == "READY"
    return process


def _stop_holder(process: subprocess.Popen) -> None:
    assert process.stdin is not None
    process.stdin.close()
    try:
        assert process.wait(timeout=5.0) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5.0)


def test_second_process_fails_then_lock_can_be_reacquired(tmp_path):
    lock_path = tmp_path / "can0.lock"
    holder = _start_holder(lock_path)
    try:
        with pytest.raises(CanOwnershipError) as caught:
            with RealCanSession(
                channel="can0",
                owner="second-process",
                path=str(lock_path),
            ):
                pass
        message = str(caught.value)
        assert "can0" in message
        assert str(lock_path) in message
        assert "pgrep" in message
        assert "powertrain_jetson" in message
        assert "powertrain_ros" in message
    finally:
        _stop_holder(holder)

    with RealCanSession(
        channel="can0",
        owner="replacement-process",
        path=str(lock_path),
    ):
        pass


def test_owner_snapshot_is_immutable_and_contains_acquisition_details(tmp_path):
    lock_path = tmp_path / "can4.lock"

    with RealCanSession(
        channel="can4",
        owner="pytest-owner",
        path=str(lock_path),
    ) as session:
        snapshot = session.owner_snapshot
        assert snapshot is not None
        assert snapshot.pid == os.getpid()
        assert snapshot.process_name == "pytest-owner"
        assert snapshot.lock_path == str(lock_path)
        assert snapshot.acquired_at.tzinfo is not None
        with pytest.raises(dataclasses.FrozenInstanceError):
            snapshot.pid = 1

    assert session.owner_snapshot == snapshot
    assert lock_path.exists()


def test_context_releases_lock_after_exception_without_deleting_file(tmp_path):
    lock_path = tmp_path / "can0.lock"

    with pytest.raises(ValueError, match="boom"):
        with RealCanSession(path=str(lock_path)):
            raise ValueError("boom")

    assert lock_path.exists()
    with RealCanSession(path=str(lock_path)):
        pass


def test_default_lock_path_is_derived_from_channel():
    assert RealCanSession(channel="can7").path == "/run/powertrain/can7.lock"


def test_missing_runtime_directory_fails_without_creating_it(tmp_path):
    missing_dir = tmp_path / "not-provisioned"
    lock_path = missing_dir / "can0.lock"

    with pytest.raises(CanOwnershipError) as caught:
        with RealCanSession(path=str(lock_path)):
            pass

    assert not missing_dir.exists()
    message = str(caught.value)
    assert str(missing_dir) in message
    assert "install_powertrain_runtime_dir.sh" in message


def test_flock_oserror_closes_fd_and_clears_primitive_state(tmp_path, monkeypatch):
    lock_path = tmp_path / "can0.lock"
    opened_fds = []
    real_open = os.open

    def tracking_open(*args, **kwargs):
        fd = real_open(*args, **kwargs)
        opened_fds.append(fd)
        return fd

    def fail_flock(_fd, _operation):
        raise OSError(errno.ENOLCK, "no locks available")

    monkeypatch.setattr(runtime_lock.os, "open", tracking_open)
    monkeypatch.setattr(runtime_lock.fcntl, "flock", fail_flock)
    lock = CanOwnerLock(str(lock_path))

    with pytest.raises(CanOwnershipError, match="no locks available"):
        lock.acquire()

    assert len(opened_fds) == 1
    assert lock.fd is None
    with pytest.raises(OSError) as caught:
        os.fstat(opened_fds[0])
    assert caught.value.errno == errno.EBADF


def test_entrypoint_checker_rejects_direct_bus_opener_fixture(tmp_path):
    checker = Path(__file__).resolve().parents[3] / "scripts/check_real_can_entrypoints.py"
    production = tmp_path / "motor_control"
    production.mkdir()
    candidate = production / "unsafe_tool.py"
    candidate.write_text("def harmless():\n    return 0\n", encoding="utf-8")

    clean = subprocess.run(
        [sys.executable, str(checker), "--root", str(tmp_path)],
        capture_output=True,
        text=True,
    )
    assert clean.returncode == 0, clean.stderr

    candidate.write_text(
        "import can\n"
        "bus = can.interface.Bus(channel='can0', interface='socketcan')\n",
        encoding="utf-8",
    )
    violation = subprocess.run(
        [sys.executable, str(checker), "--root", str(tmp_path)],
        capture_output=True,
        text=True,
    )
    assert violation.returncode == 1
    assert "motor_control/unsafe_tool.py" in violation.stderr
    assert "can.interface.Bus(" in violation.stderr
