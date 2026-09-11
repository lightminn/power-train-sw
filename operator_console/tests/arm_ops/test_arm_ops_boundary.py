"""The boundary this package promises, checked rather than trusted.

No transport, no ROS, no motion — and no edits to the ops core, the existing
console, or the arm repository.
"""
from __future__ import annotations

import ast
from pathlib import Path
import subprocess

import pytest


PACKAGE = Path(__file__).resolve().parents[2] / "arm_ops"
REPO_ROOT = Path(__file__).resolve().parents[3]
BASE_COMMIT = "2ffbad6673f54c2bd4724dc19acaea7d31f23f89"

# Opening any of these would mean this package owns a transport or a device,
# which is precisely what the console's charter forbids.
FORBIDDEN_IMPORTS = frozenset({
    "socket", "select", "selectors", "asyncio", "ssl",
    "rclpy", "rospy", "can", "serial", "usb", "odrive",
    "requests", "urllib", "http", "sqlite3", "gi",
})

# Files whose edit would mean the arm work leaked into the chassis path.
PROTECTED_FILES = (
    "operator_console/app.py",
    "operator_console/ops_client.py",
    "operator_console/ops_panel.py",
    "operator_console/arm_telemetry.py",
    "operator_console/status_view.py",
    "operator_console/telemetry.py",
    "operator_console/themes.py",
    "ros2/src/powertrain_ros/powertrain_ros/ops_contract.py",
    "ros2/src/powertrain_ros/powertrain_ros/ops_broker_core.py",
    "ros2/src/powertrain_ros/powertrain_ros/ops_broker_node.py",
    "motor_control/laptop/ops_channel_client.py",
)

# The surrounding branch already contains the separately approved read-only
# arm telemetry and tab integration.  This arm-ops package must not broaden
# that set, but its boundary test must not mistake those earlier changes for
# edits made by this package.
APPROVED_ARM_UI_INTEGRATION_FILES = frozenset({
    "operator_console/app.py",
    "operator_console/arm_telemetry.py",
})


def module_files() -> list[Path]:
    return sorted(PACKAGE.glob("*.py"))


def imported_names(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module.split(".")[0])
    return names


def test_the_package_has_the_expected_modules():
    assert {path.name for path in module_files()} == {
        "__init__.py", "adapter.py", "contract.py", "jog.py", "policy.py",
        "session.py", "transport.py",
    }


@pytest.mark.parametrize("path", module_files(), ids=lambda p: p.name)
def test_no_transport_device_or_ros_imports(path: Path):
    offenders = imported_names(path) & FORBIDDEN_IMPORTS

    assert offenders == set(), f"{path.name} imports {sorted(offenders)}"


FORBIDDEN_CONSOLE_MODULES = frozenset({
    "app", "status_view", "arm_telemetry", "telemetry", "ops_client",
    "ops_panel", "operation_runtime", "udp_source", "metadata", "pipelines",
})


def relative_imports(path: Path) -> set[str]:
    """Names imported from the enclosing ``operator_console`` package."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.level >= 2:
            if node.module:
                found.add(node.module.split(".")[0])
            found.update(alias.name for alias in node.names)
    return found


@pytest.mark.parametrize("path", module_files(), ids=lambda p: p.name)
def test_nothing_imports_the_existing_console_modules(path: Path):
    # Checked against real imports, not source text: the package docstring
    # legitimately *names* ops_client in the integration example.
    offenders = relative_imports(path) & FORBIDDEN_CONSOLE_MODULES

    assert offenders == set(), f"{path.name} imports {sorted(offenders)}"


def test_the_package_never_names_a_ros_topic_or_service():
    """Guessing an arm topic here would silently fix the arm side's contract."""
    for path in module_files():
        source = path.read_text(encoding="utf-8")
        for line in source.splitlines():
            stripped = line.strip()
            if stripped.startswith("#") or stripped.startswith("*"):
                continue          # prose about what the backend must decide
            assert '"/' not in stripped, f"{path.name}: {stripped}"
            assert "'/" not in stripped, f"{path.name}: {stripped}"


def test_only_the_arm_ui_contracts_module_is_reused_from_the_console():
    reused: set[str] = set()
    for path in module_files():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.level == 2:
                reused.add(node.module or "")
    # ``arm_ui.contracts`` only — the pure state model, no widgets.
    assert reused <= {"arm_ui.contracts"}


def test_no_module_can_execute_a_command_on_import():
    """Import must be inert: no clock read, no submit, no side effect."""
    for path in module_files():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in tree.body:
            assert not isinstance(node, ast.Expr) or isinstance(
                node.value, ast.Constant
            ), f"{path.name}: module-level expression"


def test_protected_files_are_untouched_since_the_base_commit():
    result = subprocess.run(
        ["git", "diff", "--name-only", BASE_COMMIT, "--"],
        cwd=REPO_ROOT, capture_output=True, text=True, check=True,
    )
    changed = {line for line in result.stdout.splitlines() if line}

    touched = sorted((set(PROTECTED_FILES) - APPROVED_ARM_UI_INTEGRATION_FILES) & changed)
    assert touched == [], f"보호 대상 파일이 변경됨: {touched}"


def test_changes_stay_inside_the_declared_new_paths():
    result = subprocess.run(
        ["git", "diff", "--name-only", BASE_COMMIT, "--"],
        cwd=REPO_ROOT, capture_output=True, text=True, check=True,
    )
    changed = {line for line in result.stdout.splitlines() if line}
    allowed_prefixes = (
        "operator_console/arm_ui/",
        "operator_console/arm_ops/",
        "operator_console/arm_ui_binding.py",
        "operator_console/arm_binding.py",
        "operator_console/tool_binding.py",
        "operator_console/app.py",
        "operator_console/arm_telemetry.py",
        "operator_console/runtime_smoke.py",
        "operator_console/tests/test_arm_telemetry.py",
        "operator_console/tests/test_arm_binding.py",
        "operator_console/tests/test_arm_ui_binding.py",
        "operator_console/tests/test_runtime_smoke.py",
        "operator_console/tests/test_tool_binding.py",
        "powertrain_observability/tool_snapshot.py",
        "ros2/src/powertrain_ros/powertrain_ros/arm_console_bridge_node.py",
        "ros2/src/powertrain_ros/powertrain_ros/arm_console_mirror.py",
        "ros2/src/powertrain_ros/test/test_arm_console_bridge.py",
        "operator_console/tests/arm_ui/",
        "operator_console/tests/arm_ops/",
        "docs/reports/2026-09-10-claude-arm-ui.md",
        "docs/reports/2026-09-10-existing-tool-ui-connection.md",
        "docs/reports/2026-09-10-existing-arm-runtime-ui-connection.md",
        "docs/reports/2026-09-10-existing-arm-ui-integration-validation.md",
        "docs/reports/2026-09-10-arm-ui-skeleton-integration.md",
        "docs/plans/2026-09-10-arm-console-ui.md",
        "docs/specs/2026-09-10-arm-existing-ui-contract.md",
        "docs/specs/2026-09-10-claude-arm-ops-contract.md",
    )

    unexpected = sorted(
        name for name in changed if not name.startswith(allowed_prefixes)
    )
    assert unexpected == [], f"범위 밖 파일이 변경됨: {unexpected}"


def test_the_arm_repository_is_not_referenced_as_a_write_target():
    for path in module_files():
        source = path.read_text(encoding="utf-8")
        assert "extreme-robot" not in source, path.name


def test_the_recording_transport_is_not_reachable_from_the_package_root():
    """A fake that *accepts* must never be a production default.

    ``NullArmTransport`` refuses and is exported; ``RecordingArmTransport``
    accepts and is not, so it cannot be picked up by mistake in an integration
    that meant "do not send".
    """
    from operator_console import arm_ops

    assert not hasattr(arm_ops, "RecordingArmTransport")
    assert "NullArmTransport" in arm_ops.__all__
    assert "RecordingArmTransport" not in arm_ops.__all__


def test_the_default_session_transport_refuses():
    from operator_console.arm_ops import NullArmTransport
    from operator_console.arm_ops.session import ArmCommandSession

    session = ArmCommandSession(clock=lambda: 0.0)

    assert isinstance(session._transport, NullArmTransport)
