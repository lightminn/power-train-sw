"""Execution gate: the real console must survive LIVE data on every panel.

Wraps operator_console.runtime_smoke so the host suite launches the actual
Gtk app under Xvfb (system python) even though pytest itself runs under a
gi-less interpreter. Skips where the desktop toolchain is absent (dev
container, CI without Xvfb) — those environments rely on this gate having
run on the operator PC.
"""
import inspect
import json
from pathlib import Path
import shutil
import subprocess

import pytest

import operator_console.runtime_smoke as runtime_smoke
from operator_console.runtime_smoke import SYSTEM_PYTHON, run_smoke


def _toolchain_ready() -> bool:
    if shutil.which("xvfb-run") is None or not Path(SYSTEM_PYTHON).exists():
        return False
    return subprocess.run(
        [SYSTEM_PYTHON, "-c", "import gi"], capture_output=True,
    ).returncode == 0


@pytest.mark.skipif(
    not _toolchain_ready(),
    reason="needs xvfb-run + system python with gi (operator PC)",
)
def test_console_survives_live_data_on_every_channel():
    passed, report = run_smoke()
    assert passed, report
    assert "invalid steering mode held disabled" in report


def test_ops_probe_requires_the_received_state_identity(tmp_path):
    probe_file = tmp_path / "panels.json"
    probe_file.write_text(json.dumps({
        "ops_steering_label": "조향 방식 [상태 미확인]",
        "ops_steering_sensitive": False,
    }), encoding="utf-8")

    assert runtime_smoke._probe_ops_steering(probe_file) is None

    probe_file.write_text(json.dumps({
        "ops_steering_label": "조향 방식 [상태 미확인]",
        "ops_steering_sensitive": False,
        "ops_state_revision": 7,
        "ops_state_steering_mode": "invalid-smoke-mode",
    }), encoding="utf-8")

    assert runtime_smoke._probe_ops_steering(probe_file) == (
        "조향 방식 [상태 미확인]",
        False,
        7,
        "invalid-smoke-mode",
    )


def test_runtime_smoke_constructs_the_token_gated_ops_controls():
    source = (
        Path(__file__).resolve().parents[1] / "runtime_smoke.py"
    ).read_text(encoding="utf-8")

    assert "/nonexistent/ops.token" not in source
    assert "NamedTemporaryFile" in source
    assert '"--ops-token-file", token_file' in source


def test_smoke_env_forces_x11_backend():
    smoke_child_env = getattr(runtime_smoke, "smoke_child_env", None)
    assert smoke_child_env is not None

    env = smoke_child_env({
        "WAYLAND_DISPLAY": "wayland-0",
        "GDK_BACKEND": "wayland",
        "PATH": "/usr/bin",
    })

    assert "WAYLAND_DISPLAY" not in env
    assert env["GDK_BACKEND"] == "x11"
    assert env["PATH"] == "/usr/bin"


def test_runtime_smoke_passes_isolated_environment_to_console_child():
    source = inspect.getsource(runtime_smoke.run_smoke)

    assert "env=smoke_child_env()" in source
