"""Execution gate: the real console must survive LIVE data on every panel.

Wraps operator_console.runtime_smoke so the host suite launches the actual
Gtk app under Xvfb (system python) even though pytest itself runs under a
gi-less interpreter. Skips where the desktop toolchain is absent (dev
container, CI without Xvfb) — those environments rely on this gate having
run on the operator PC.
"""
from pathlib import Path
import shutil
import subprocess

import pytest

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


def test_runtime_smoke_constructs_the_token_gated_ops_controls():
    source = (
        Path(__file__).resolve().parents[1] / "runtime_smoke.py"
    ).read_text(encoding="utf-8")

    assert "/nonexistent/ops.token" not in source
    assert "NamedTemporaryFile" in source
    assert '"--ops-token-file", token_file' in source
