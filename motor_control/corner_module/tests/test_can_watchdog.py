from corner_module.can_watchdog import CanWatchdog


class FakeWatchdog(CanWatchdog):
    def __init__(self, *, up=True, probes=(), tx=(), reset_error=None):
        super().__init__(period_s=0.0)
        self.up = up
        self.probes = iter(probes)
        self.tx = iter(tx)
        self.reset_error = reset_error
        self.probe_calls = 0
        self.reset_calls = 0
        self.reopen_calls = 0

    def _interface_is_up(self):
        return self.up

    def _probe_ok(self):
        self.probe_calls += 1
        return next(self.probes)

    def _tx_packets(self):
        return next(self.tx)

    def _reset_interface(self):
        self.reset_calls += 1
        if self.reset_error is not None:
            raise self.reset_error

    def _reopen_probe_socket(self):
        self.reopen_calls += 1


def test_down_interface_skips_probe_and_recovery_without_counting_reset():
    wd = FakeWatchdog(up=False)

    assert wd._step() == "down"
    assert wd._step() == "down"

    assert wd.probe_calls == 0
    assert wd.reset_calls == 0
    assert wd.resets == 0


def test_unknown_interface_state_is_treated_as_down():
    wd = FakeWatchdog(up=None)

    assert wd._step() == "down"
    assert wd.probe_calls == 0
    assert wd.resets == 0


def test_interface_query_exception_is_treated_as_down():
    wd = FakeWatchdog()

    def fail_query():
        raise OSError("file table exhausted")

    wd._interface_is_up = fail_query
    assert wd._step() == "down"
    assert wd.probe_calls == 0


def test_transition_to_up_resumes_normal_probe_logic():
    wd = FakeWatchdog(up=False, probes=(True,), tx=(10,))
    assert wd._step() == "down"

    wd.up = True
    assert wd._step() == "ok"
    assert wd.probe_calls == 1


def test_up_wedge_resets_after_two_stalled_failed_probes():
    wd = FakeWatchdog(up=True, probes=(False, False), tx=(7, 7))

    assert wd._step() == "failed"
    assert wd._step() == "reset"

    assert wd.reset_calls == 1
    assert wd.reopen_calls == 1
    assert wd.resets == 1


def test_failed_interface_reset_is_not_counted_or_reopened():
    wd = FakeWatchdog(
        up=True,
        probes=(False, False),
        tx=(7, 7),
        reset_error=OSError("not configured"),
    )

    assert wd._step() == "failed"
    assert wd._step() == "reset_failed"

    assert wd.reset_calls == 1
    assert wd.reopen_calls == 0
    assert wd.resets == 0


def test_probe_socket_reopen_failure_keeps_old_socket_installed():
    class OldSocket:
        closed = False

        def close(self):
            self.closed = True

    wd = CanWatchdog()
    old = OldSocket()
    wd._sock = old

    def fail_open():
        raise OSError("temporary bind failure")

    wd._open_probe_socket = fail_open

    try:
        wd._reopen_probe_socket()
    except OSError:
        pass
    else:
        raise AssertionError("reopen failure must be reported")

    assert wd._sock is old
    assert old.closed is False
