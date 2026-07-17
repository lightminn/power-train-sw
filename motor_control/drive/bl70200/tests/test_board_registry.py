import importlib
import json

import pytest


def _registry_module():
    return importlib.import_module("drive.bl70200.board_registry")


def _setup_module():
    return importlib.import_module("drive.bl70200.bl70200_setup")


def test_loads_serial_to_axis_node_pairs(tmp_path) -> None:
    path = tmp_path / "boards.json"
    path.write_text(
        json.dumps({"3352": [11, 12], "336a": [13, 14]}),
        encoding="utf-8",
    )

    assert _registry_module().load(path) == {
        "3352": (11, 12),
        "336a": (13, 14),
    }


def test_missing_registry_is_value_error(tmp_path) -> None:
    with pytest.raises(ValueError, match="registry"):
        _registry_module().load(tmp_path / "missing.json")


def test_duplicate_node_is_value_error(tmp_path) -> None:
    path = tmp_path / "boards.json"
    path.write_text(
        json.dumps({"3352": [11, 12], "336a": [12, 13]}),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="duplicate node"):
        _registry_module().load(path)


@pytest.mark.parametrize(
    "payload",
    (
        [],
        {"3352": [11]},
        {"": [11, 12]},
        {"3352": [11, "12"]},
        {"3352": [11, 11]},
    ),
)
def test_invalid_registry_format_is_value_error(tmp_path, payload) -> None:
    path = tmp_path / "boards.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="registry"):
        _registry_module().load(path)


def test_resolve_node_uses_serial_and_axis() -> None:
    registry = {"3352": (11, 12)}
    module = _registry_module()

    assert module.resolve_node(registry, "3352", 0) == 11
    assert module.resolve_node(registry, "3352", 1) == 12
    with pytest.raises(ValueError, match="serial"):
        module.resolve_node(registry, "missing", 0)
    with pytest.raises(ValueError, match="axis"):
        module.resolve_node(registry, "3352", 2)


def test_argparse_exposes_serial_axis_and_persist_calibration() -> None:
    args = _setup_module().parse_args(
        ["--serial", "3352", "--axis", "both", "--persist-calibration"]
    )

    assert args.serial == "3352"
    assert args.axis == "both"
    assert args.persist_calibration is True


def test_persist_calibration_requires_serial() -> None:
    with pytest.raises(SystemExit):
        _setup_module().parse_args(["--persist-calibration"])


def test_find_odrive_selects_exact_serial_without_importing_hardware_library() -> None:
    calls = []
    board = object()

    class FakeOdrive:
        @staticmethod
        def find_any(**kwargs):
            calls.append(kwargs)
            return board

    found = _setup_module().find_odrive(FakeOdrive, serial="3352", timeout=7)

    assert found is board
    assert calls == [{"serial_number": "3352", "timeout": 7}]
