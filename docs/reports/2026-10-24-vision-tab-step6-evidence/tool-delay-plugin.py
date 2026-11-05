"""Reproduce a single datagram expiring before the arm UI consumes it."""
import time
import pytest

@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(items):
    from operator_console.arm_ui_binding import ArmUiTelemetryBinding
    original = ArmUiTelemetryBinding.state
    injected = False
    def delayed(self, snapshot, *args, **kwargs):
        nonlocal injected
        if not injected and snapshot is not None and snapshot.sequence==2:
            injected = True
            time.sleep(1.1)
        return original(self, snapshot, *args, **kwargs)
    ArmUiTelemetryBinding.state = delayed
