"""Validate the installed ``powertrain_ros`` Python distribution.

These checks are meaningful only after a colcon install space containing
``powertrain_ros`` has been sourced.  Once that condition is met, falling back
to modules from the source tree is a failure rather than a reason to skip.
"""

import ast
import importlib
from importlib import metadata
import os
from pathlib import Path

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_PACKAGE = (PACKAGE_ROOT / "powertrain_ros").resolve()
AMENT_RESOURCE = Path(
    "share/ament_index/resource_index/packages/powertrain_ros"
)
REQUIRED_MODULES = {
    "powertrain_ros.wheel_stop",
    "powertrain_ros.remote_input",
    "powertrain_ros.remote_input_gateway",
    "powertrain_ros.teleop_command_node",
    "powertrain_ros.detection_adapter",
    "powertrain_ros.arm_interlock",
}


def _powertrain_ament_prefixes():
    prefixes = [
        Path(item).resolve()
        for item in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if item
    ]
    installed = [
        prefix for prefix in prefixes if (prefix / AMENT_RESOURCE).is_file()
    ]
    if not installed:
        pytest.skip(
            "AMENT_PREFIX_PATH contains no powertrain_ros install space"
        )
    return installed


def _declared_console_scripts():
    tree = ast.parse((PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8"))
    setup_call = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "setup"
    )
    entry_points = next(
        keyword.value
        for keyword in setup_call.keywords
        if keyword.arg == "entry_points"
    )
    console_scripts = next(
        value
        for key, value in zip(entry_points.keys, entry_points.values)
        if isinstance(key, ast.Constant) and key.value == "console_scripts"
    )
    declared = {}
    for item in console_scripts.elts:
        text = ast.literal_eval(item)
        name, target = (part.strip() for part in text.split("=", 1))
        declared[name] = target
    return declared


def _assert_loaded_from_install_space(module, prefixes):
    loaded = Path(module.__file__).resolve()
    assert not loaded.is_relative_to(SOURCE_PACKAGE), (
        f"{module.__name__} loaded from source tree: {loaded}"
    )
    assert any(loaded.is_relative_to(prefix) for prefix in prefixes), (
        f"{module.__name__} did not load from AMENT_PREFIX_PATH: {loaded}"
    )


def test_installed_distribution_exposes_every_declared_console_script():
    prefixes = _powertrain_ament_prefixes()
    declared = _declared_console_scripts()
    distribution = metadata.distribution("powertrain-ros")
    installed = {
        entry.name: entry.value
        for entry in distribution.entry_points
        if entry.group == "console_scripts"
    }

    for name, target in declared.items():
        assert installed.get(name) == target
        module_name = target.partition(":")[0]
        module = importlib.import_module(module_name)
        _assert_loaded_from_install_space(module, prefixes)


def test_required_modules_load_from_current_colcon_install_space():
    prefixes = _powertrain_ament_prefixes()

    for module_name in sorted(REQUIRED_MODULES):
        module = importlib.import_module(module_name)
        _assert_loaded_from_install_space(module, prefixes)
