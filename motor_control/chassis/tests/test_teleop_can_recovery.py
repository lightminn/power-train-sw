"""A denied CAN owner must never start a reset-capable watchdog."""
import pytest

from chassis import teleop_server


def test_can_owner_denial_precedes_watchdog_creation(monkeypatch):
    from chassis import runtime_lock
    from corner_module import can_watchdog
    watchdogs = []

    class BusySession:
        def __init__(self, **kwargs):
            pass

        def __enter__(self):
            raise RuntimeError('another CAN owner')

        def close(self):
            pass

    class Watchdog:
        def __init__(self, *args, **kwargs):
            watchdogs.append(self)

        def start(self):
            pass

    monkeypatch.setattr(runtime_lock, 'RealCanSession', BusySession)
    monkeypatch.setattr(can_watchdog, 'CanWatchdog', Watchdog)
    with pytest.raises(RuntimeError, match='another CAN owner'):
        teleop_server.main(['--diagnostic-direct-can', '--confirm-arm-stowed', '--no-us100'])
    assert not watchdogs
