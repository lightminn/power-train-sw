"""Real flock coordination around fake CAN observations and reset effects."""
from pathlib import Path

import pytest
from chassis.runtime_lock import RealCanSession
from corner_module.can_watchdog import CanWatchdog


class ObservedWatchdog(CanWatchdog):
    def __init__(self, path, *, owner_session=None, before_reset=None):
        super().__init__()
        # Set the dependency fields to also reproduce the old implementation.
        self._lock_path = str(path)
        self._owner_session = owner_session
        self._before_reset = before_reset
        self.up = True
        self.tx = 7
        self.probe = False
        self.mutations = []
        self.on_reset = None
        self.on_tx = None

    def _interface_is_up(self):
        return self.up

    def _tx_packets(self):
        if self.on_tx:
            self.on_tx()
        return self.tx

    def _probe_ok(self):
        return self.probe

    def _reset_interface(self):
        self.mutations.append("reset")
        if self.on_reset:
            self.on_reset()

    def _reopen_probe_socket(self):
        pass


def test_standalone_cannot_reset_under_active_owner(tmp_path):
    path = tmp_path / "can0.lock"
    wd = ObservedWatchdog(path)
    with RealCanSession(path=str(path)):
        wd._step()
        wd._step()
    assert wd.mutations == []


def test_owner_recovery_latches_stop_before_reset_and_keeps_ownership(tmp_path):
    path = tmp_path / "can0.lock"
    events = []
    with RealCanSession(path=str(path)) as owner:
        wd = ObservedWatchdog(path, owner_session=owner, before_reset=lambda: events.append("estop"))
        wd.on_reset = lambda: events.append("reset")
        wd._step()
        assert wd._step() == "reset"
        assert events == ["estop", "reset"]
        assert owner._lock.fd is not None


def test_owner_recovery_requires_stop_callback(tmp_path):
    path = tmp_path / "can0.lock"
    with RealCanSession(path=str(path)) as owner:
        wd = ObservedWatchdog(path, owner_session=owner)
        wd._step()
        wd._step()
    assert wd.mutations == []


def test_stop_exception_prevents_reset(tmp_path):
    path = tmp_path / "can0.lock"
    def reject():
        raise RuntimeError("stop did not latch")
    with RealCanSession(path=str(path)) as owner:
        wd = ObservedWatchdog(path, owner_session=owner, before_reset=reject)
        wd._step()
        assert wd._step() == "reset_failed"
    assert wd.mutations == []


def test_duplicate_old_stall_only_resets_once_even_if_tx_count_stays_unchanged(tmp_path):
    path = tmp_path / "can0.lock"
    first, second = ObservedWatchdog(path), ObservedWatchdog(path)
    first._step()
    second._step()
    assert first._step() == "reset"
    second._step()
    assert len(first.mutations) + len(second.mutations) == 1


def test_tx_progress_after_stall_observation_is_rechecked_before_reset(tmp_path):
    wd = ObservedWatchdog(tmp_path / "can0.lock")
    wd._step()
    reads = [7, 8]
    def progress():
        wd.tx = reads.pop(0) if reads else 8
    wd.on_tx = progress
    wd._step()
    assert wd.mutations == []


def test_unknown_tx_counter_never_proves_stall(tmp_path):
    wd = ObservedWatchdog(tmp_path / "can0.lock")
    wd.tx = None
    for _ in range(4):
        wd._step()
    assert wd.mutations == []


def test_closed_owner_session_cannot_authorize_reset(tmp_path):
    path = tmp_path / "can0.lock"
    with RealCanSession(path=str(path)) as owner:
        pass
    wd = ObservedWatchdog(path, owner_session=owner, before_reset=lambda: None)
    wd._step()
    wd._step()
    assert wd.mutations == []


def test_public_step_lazily_opens_and_close_releases_probe(tmp_path):
    wd = ObservedWatchdog(tmp_path / "can0.lock")
    class Probe:
        closed = False
        def close(self):
            self.closed = True
    probe = Probe()
    opened = []
    def open_probe():
        opened.append(probe)
        return probe
    wd._open_probe_socket = open_probe
    wd.probe = True
    assert wd._sock is None
    assert wd.step() == "ok"
    assert wd.step() == "ok"
    assert opened == [probe]
    wd.close()
    assert probe.closed and wd._sock is None


def test_foreground_service_survives_recovery_failure_and_releases_probe(tmp_path, monkeypatch, capsys):
    import corner_module.can_watchdog as module
    wd = ObservedWatchdog(tmp_path / "can0.lock")
    class Probe:
        closed = False
        def close(self):
            self.closed = True
    probe = Probe()
    wd._open_probe_socket = lambda: probe
    def reset_failure():
        raise OSError("link up failed")
    wd.on_reset = reset_failure
    sleeps = []
    def until_third_poll(_):
        sleeps.append(1)
        if len(sleeps) == 3:
            raise KeyboardInterrupt
    monkeypatch.setattr(module.time, "sleep", until_third_poll)
    with pytest.raises(KeyboardInterrupt):
        wd._run()
    assert len(sleeps) == 3
    assert wd.resets == 0
    assert probe.closed
    assert "reset_failed: link up failed" in capsys.readouterr().out


def test_interrupted_reset_can_retry_link_up_while_owner_stays_stopped(tmp_path):
    path = tmp_path / "can0.lock"
    stopped = []
    with RealCanSession(path=str(path)) as owner:
        wd = ObservedWatchdog(path, owner_session=owner, before_reset=lambda: stopped.append(True))
        def fail_after_down():
            wd.up = False
            raise OSError("up failed after down")
        wd.on_reset = fail_after_down
        wd._step()
        assert wd._step() == "reset_failed"
        wd.on_reset = lambda: setattr(wd, "up", True)
        assert wd._step() == "reset"
    assert wd.up and wd.resets == 1
    assert stopped == [True, True]


def test_other_maintenance_cancels_interrupted_reset_retry(tmp_path):
    from chassis.runtime_lock import CanMaintenanceSession
    path = tmp_path / "can0.lock"
    wd = ObservedWatchdog(path)
    def fail_after_down():
        wd.up = False
        raise OSError("up failed after down")
    wd.on_reset = fail_after_down
    wd._step()
    assert wd._step() == "reset_failed"
    with CanMaintenanceSession(path=str(path)) as maintenance:
        maintenance.mark_reset()
    assert wd._step() == "superseded"
    assert wd.mutations == ["reset"]
    assert not wd.up
