"""Isolate real GTK/SRT widgets from the unit suite's global GTK context."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.parametrize("case", [
    "test_vision_roundtrip_keeps_single_decoder_and_restores_layout[False]",
    "test_vision_roundtrip_keeps_single_decoder_and_restores_layout[True]",
    "test_vision_overlay_toggle_shares_mission_state",
    "test_vision_maps_live_stale_recovery_hidden_and_controls",
    "test_vision_attitude_sources_missing_values_static_history_and_recovery",
])
def test_vision_srt_in_isolated_gtk_process(case):
    if shutil.which("xvfb-run") is None:
        pytest.skip("needs Xvfb")
    result = subprocess.run(
        ["xvfb-run", "-a", "/usr/bin/python3", "-m", "pytest", "-q",
         "operator_console/tests/_vision_gtk_e2e.py::" + case],
        cwd=Path(__file__).resolve().parents[2],
        env=dict(os.environ, GDK_BACKEND="x11"),
        capture_output=True, text=True, timeout=45,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Traceback" not in result.stderr, result.stderr
