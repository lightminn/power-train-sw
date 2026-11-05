"""A fresh streaming fixture must still reject a broken tool UI projection."""
import pytest

@pytest.hookimpl(trylast=True)
def pytest_collection_modifyitems(items):
    from operator_console.arm_ui_binding import ArmUiTelemetryBinding, C
    ArmUiTelemetryBinding.state = lambda *args, **kwargs: C.default_state()
