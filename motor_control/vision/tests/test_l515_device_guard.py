import ast
import importlib
from pathlib import Path
from types import SimpleNamespace

import pytest


VISION_DIR = Path(__file__).resolve().parents[1]


class Config:
    def __init__(self):
        self.serial = None

    def enable_device(self, serial):
        self.serial = serial


class Profile:
    def __init__(self, serial):
        self._serial = serial

    def get_device(self):
        return SimpleNamespace(get_info=lambda _key: self._serial)


class Pipeline:
    def __init__(self, serial):
        self.serial = serial
        self.stop_calls = 0

    def start(self, _config):
        return Profile(self.serial)

    def stop(self):
        self.stop_calls += 1


RS = SimpleNamespace(camera_info=SimpleNamespace(serial_number="serial"))


def test_l515_start_helper_selects_only_expected_serial_and_accepts_sdk_short_form():
    module = importlib.import_module("realsense_l515")
    config = Config()
    pipeline = Pipeline("f0271544")

    profile = module.start_l515_pipeline(pipeline, config, RS)

    assert config.serial == "00000000F0271544"
    assert profile.get_device().get_info("serial") == "f0271544"
    assert pipeline.stop_calls == 0


def test_l515_start_helper_stops_and_rejects_wrong_sdk_device():
    module = importlib.import_module("realsense_l515")
    config = Config()
    pipeline = Pipeline("250222071245")

    with pytest.raises(RuntimeError, match="unexpected RealSense serial"):
        module.start_l515_pipeline(pipeline, config, RS)

    assert pipeline.stop_calls == 1


def test_l515_serial_can_be_overridden_for_commissioning(monkeypatch):
    module = importlib.import_module("realsense_l515")
    monkeypatch.setenv("POWERTRAIN_L515_SERIAL", "commissioned-l515")
    try:
        module = importlib.reload(module)
        assert module.EXPECTED_L515_SERIAL == "commissioned-l515"
    finally:
        monkeypatch.delenv("POWERTRAIN_L515_SERIAL", raising=False)
        importlib.reload(module)


@pytest.mark.parametrize(
    "filename",
    ["yolo_depth_3d.py", "realsense_stream.py", "realsense_test.py"],
)
def test_every_powertrain_realsense_script_uses_the_serial_locked_start(filename):
    tree = ast.parse((VISION_DIR / filename).read_text(encoding="utf-8"))
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]

    assert any(
        isinstance(call.func, ast.Name) and call.func.id == "start_l515_pipeline"
        for call in calls
    )
    assert not any(
        isinstance(call.func, ast.Attribute)
        and call.func.attr == "start"
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == "pipe"
        for call in calls
    )
