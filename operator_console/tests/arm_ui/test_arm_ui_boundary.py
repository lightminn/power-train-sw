"""The package boundary, asserted rather than merely documented.

The arm UI is a skeleton: no transport, no ROS, no motion, no persistence, and
no edits to the existing console files.  Those are review promises that rot
quietly, so they are checked here — statically for imports, and by actually
launching the preview for the execution path.
"""
from __future__ import annotations

import ast
from pathlib import Path
import subprocess
import sys

import pytest


PACKAGE = Path(__file__).resolve().parents[2] / "arm_ui"
REPO_ROOT = Path(__file__).resolve().parents[3]

# Modules this package must never reach for.  ``socket`` and friends would mean
# it owns a transport; ``rclpy`` would mean it owns a ROS node.
FORBIDDEN_IMPORTS = frozenset({
    "socket", "select", "asyncio", "rclpy", "rospy", "can", "serial",
    "requests", "urllib", "http", "subprocess", "sqlite3",
})

# Existing console modules the arm UI may read from but must never import in a
# way that drags the whole app (and GStreamer) into a preview.
FORBIDDEN_CONSOLE_MODULES = frozenset({"app", "status_view", "arm_telemetry"})


def module_files() -> list[Path]:
    return sorted(path for path in PACKAGE.glob("*.py"))


def imported_names(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.module and node.level == 0:
                names.add(node.module.split(".")[0])
            elif node.module:
                names.add(node.module.split(".")[0])
    return names


def test_package_has_the_expected_modules():
    assert {path.name for path in module_files()} == {
        "__init__.py", "__main__.py", "calibration_tab.py", "contracts.py",
        "fixtures.py", "keyboard_panel.py", "manual_tab.py", "preview.py",
        "styling.py", "tool_panels.py", "widgets.py",
    }


@pytest.mark.parametrize("path", module_files(), ids=lambda p: p.name)
def test_no_transport_or_ros_imports(path: Path):
    offenders = imported_names(path) & FORBIDDEN_IMPORTS
    # The preview entry point legitimately parses argv and prints; it still
    # must not open a socket or start a process.
    assert offenders == set(), f"{path.name} imports {sorted(offenders)}"


@pytest.mark.parametrize("path", module_files(), ids=lambda p: p.name)
def test_does_not_pull_in_the_existing_console_app(path: Path):
    offenders = imported_names(path) & FORBIDDEN_CONSOLE_MODULES
    assert offenders == set(), f"{path.name} imports {sorted(offenders)}"


def test_only_labels_is_reused_from_the_existing_console():
    # Reusing the shared Korean freshness labels is deliberate; anything else
    # would be a new coupling to an existing file we are not allowed to touch.
    reused: set[str] = set()
    for path in module_files():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.level == 2:
                reused.update(alias.name for alias in node.names)
    assert reused <= {"labels"}


def test_tab_package_does_not_expand_beyond_the_approved_console_integration():
    """Keep the skeleton isolated except for the explicitly approved adapters."""
    result = subprocess.run(
        ["git", "diff", "--name-only",
         "2ffbad6673f54c2bd4724dc19acaea7d31f23f89", "--"],
        cwd=REPO_ROOT, capture_output=True, text=True, check=True,
    )
    changed = {line for line in result.stdout.splitlines() if line}
    # The declared scope of this branch's work.  The arm-ops contract work
    # extended it with its own new paths; both tasks are listed here so this
    # assertion keeps meaning "nothing outside what was declared".
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
        "docs/plans/2026-09-10-arm-console-ui.md",
        "docs/specs/2026-09-10-arm-existing-ui-contract.md",
        "docs/specs/2026-09-10-claude-arm-ops-contract.md",
        "docs/reports/2026-09-10-existing-tool-ui-connection.md",
        "docs/reports/2026-09-10-existing-arm-runtime-ui-connection.md",
        "docs/reports/2026-09-10-existing-arm-ui-integration-validation.md",
        "docs/reports/2026-09-10-arm-ui-skeleton-integration.md",
        "operator_console/tests/arm_ui/",
        "operator_console/tests/arm_ops/",
        "docs/reports/2026-09-10-claude-arm-ui.md",
    )
    unexpected = sorted(
        name for name in changed
        if not name.startswith(allowed_prefixes)
    )
    assert unexpected == [], f"범위 밖 파일이 변경됨: {unexpected}"


def test_preview_launches_and_exits_cleanly():
    """Execution gate: a unit-green GTK panel can still crash on first paint."""
    result = subprocess.run(
        [sys.executable, "-m", "operator_console.arm_ui",
         "--seconds", "2", "--cycle"],
        cwd=REPO_ROOT, capture_output=True, text=True, timeout=90,
    )

    assert result.returncode == 0, result.stderr
    assert "Traceback" not in result.stderr, result.stderr
    for marker in ("CRITICAL", "Gtk-WARNING", "AttributeError"):
        assert marker not in result.stderr, result.stderr


def test_preview_scenarios_all_render():
    for scenario in ("disconnected", "waiting", "single", "dual", "cleaner", "stale"):
        result = subprocess.run(
            [sys.executable, "-m", "operator_console.arm_ui",
             "--scenario", scenario, "--seconds", "1"],
            cwd=REPO_ROOT, capture_output=True, text=True, timeout=60,
        )
        assert result.returncode == 0, f"{scenario}: {result.stderr}"
        assert "Traceback" not in result.stderr, f"{scenario}: {result.stderr}"
