"""C0 extraction wiring contracts without importing ROS dependencies."""

import ast
from pathlib import Path
from types import SimpleNamespace


PACKAGE = Path(__file__).resolve().parents[1]
CHASSIS_NODE = PACKAGE / "powertrain_ros/chassis_node.py"
SOURCE = CHASSIS_NODE.read_text(encoding="utf-8")


def _extract_method(name):
    tree = ast.parse(SOURCE)
    chassis_node = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "ChassisNode"
    )
    method = next(
        item
        for item in chassis_node.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {}
    exec(compile(module, str(CHASSIS_NODE), "exec"), namespace)
    return namespace[name]


class _RecordingCm:
    def __init__(self, *, granted, mode="ESTOP", reject=""):
        self.granted = granted
        self.mode = mode
        self.reject = reject
        self.grant_calls = 0

    def extraction_grant(self):
        self.grant_calls += 1
        return self.granted

    def snapshot(self):
        return SimpleNamespace(
            chassis_mode=self.mode,
            last_extraction_reject=self.reject,
        )


def _response():
    return SimpleNamespace(success=None, message="")


def test_extraction_grant_callback_reports_success_state():
    callback = _extract_method("_srv_extraction_grant")
    cm = _RecordingCm(granted=True, mode="EXTRACTION")
    response = _response()

    returned = callback(SimpleNamespace(cm=cm), object(), response)

    assert returned is response
    assert response.success is True
    assert "EXTRACTION" in response.message
    assert cm.grant_calls == 1


def test_extraction_grant_callback_reports_snapshot_reject_reason():
    callback = _extract_method("_srv_extraction_grant")
    cm = _RecordingCm(
        granted=False,
        mode="ESTOP",
        reject="estop_not_latched",
    )
    response = _response()

    callback(SimpleNamespace(cm=cm), object(), response)

    assert response.success is False
    assert "estop_not_latched" in response.message
    assert cm.grant_calls == 1


def test_extraction_parameter_and_service_source_contract():
    tree = ast.parse(SOURCE)
    assert any(
        isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "declare_parameter"
        and len(call.args) >= 2
        and ast.literal_eval(call.args[0]) == "extraction_enabled"
        and ast.literal_eval(call.args[1]) is False
        for call in ast.walk(tree)
    )
    assert 'self.get_parameter("extraction_enabled").value' in SOURCE
    assert "extraction_enabled=extraction_enabled" in SOURCE
    assert '"~/extraction_grant"' in SOURCE
    assert "self._srv_extraction_grant" in SOURCE
