"""Execution gate: the real console must survive LIVE data on every panel.

Wraps operator_console.runtime_smoke so the host suite launches the actual
Gtk app under Xvfb (system python) even though pytest itself runs under a
gi-less interpreter. Skips where the desktop toolchain is absent (dev
container, CI without Xvfb) — those environments rely on this gate having
run on the operator PC.
"""
import inspect
import json
import os
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


def test_arm_tabs_probe_requires_observed_tool_and_disabled_actions(tmp_path):
    probe_file = tmp_path / "panels.json"
    probe_file.write_text(json.dumps({
        "arm_ui_detected_tool": "dual_gripper",
        "arm_ui_actions_disabled": True,
    }), encoding="utf-8")

    assert runtime_smoke._probe_arm_tabs(probe_file) == ("dual_gripper", True)


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


def test_terrain_map_stage_passes_and_is_reported_before_the_arm_tabs():
    _, report = run_smoke()
    assert "terrain_map ok" in report, report[-2000:]


def test_terrain_map_negative_controls_name_their_reason():
    passed, report = run_smoke(omit_channel_for_test="terrain_map")
    assert not passed and report.startswith("panels never reached LIVE: terrain_map\n"), report[:300]
    passed, report = run_smoke(omit_channel_for_test="terrain_map_tiles")
    assert not passed and report.startswith("terrain map tiles never arrived"), report[:300]


@pytest.mark.skipif(not _toolchain_ready(), reason='needs Xvfb/system GTK/GStreamer')
def test_vision_runtime_smoke_connections_values_roundtrip_and_shutdown(tmp_path):
    metrics = tmp_path/'console-events.json'
    passed, report = runtime_smoke.run_vision_smoke(metrics_output=metrics)
    _save_vision_evidence('vision-positive.json' if passed else 'vision-positive-failed.log', report)
    assert passed, report
    data = json.loads(report)
    phases = {row['phase']:row for row in data['phases']}
    assert phases['missing']['raw_pitch'] is None
    assert phases['live']['raw_pitch']==pytest.approx(19.5)
    assert phases['live']['joint_values']==[None]*4
    assert phases['aged']['link']=='LIVE'
    assert phases['aged']['joint_values']==[None]*4
    assert phases['disconnected']['link']=='STALE'
    assert phases['recovered']['map_state']=='LIVE'
    assert phases['returned']['slots_mapped']==[True]*4
    assert data['shutdown']=='worker/timers/receivers/pipelines/ports closed'
    assert data['receiver_pipelines']==2
    observed = json.loads(metrics.read_text())
    assert observed['status']=='closed' and not observed['hardware_verified']
    vision = max((s for s in observed['segments'] if s['page']=='vision'), key=lambda s:s['duration_s'])
    assert 12 < vision['events']['l515_sink']['hz'] < 18
    assert vision['events']['top_images']['count'] >= 4
    assert vision['events']['oblique_images']['count'] >= 4


@pytest.mark.skipif(not _toolchain_ready(), reason='needs Xvfb/system GTK/GStreamer')
@pytest.mark.parametrize('fault,reason', [
    ('map', 'vision map/telemetry LIVE: timed out'),
    ('telemetry', 'vision map/telemetry LIVE: timed out'),
    ('missing-zero', 'vision missing values fabricated'),
    ('stale', 'vision disconnect/stale: timed out'),
    ('recovery', 'vision recovery: timed out'),
    ('hidden-render', 'vision hidden render continues'),
    ('duplicate-camera', 'vision duplicate camera/decoder creation'),
    ('shutdown', 'vision worker shutdown: timed out'),
    ('drawing-stall', 'vision camera draw stalled'),
])
def test_vision_runtime_negative_controls_reject_the_specific_fault(fault, reason, tmp_path):
    metrics = tmp_path/'fault-events.json' if fault=='drawing-stall' else None
    passed, report = runtime_smoke.run_vision_smoke(fault_for_test=fault, metrics_output=metrics)
    _save_vision_evidence(f'negative-{fault}.log', report)
    assert not passed, report
    assert reason in report, report
    if metrics is not None:
        observed = json.loads(metrics.read_text())
        assert observed['status']=='closed'
        assert max(s['events']['heartbeat']['max_gap_s'] or 0 for s in observed['segments']) >= .6


def _save_vision_evidence(name, report):
    destination = os.environ.get('VISION_SMOKE_EVIDENCE_DIR')
    if destination:
        out = Path(destination)
        out.mkdir(parents=True, exist_ok=True)
        (out/name).write_text(report+'\n', encoding='utf-8')


@pytest.mark.parametrize('mode,expected', [
    ('native-stdout', True), ('missing-file', False), ('malformed', False),
    ('missing-phase', False), ('nonzero-exit', False), ('traceback', False),
])
def test_vision_report_file_separates_native_output_and_requires_evidence(monkeypatch, mode, expected):
    class Child:
        returncode = 1 if mode=='nonzero-exit' else 0
        def __init__(self, command, **kwargs):
            self.report = Path(command[command.index('--report-file')+1])
        def communicate(self, **kwargs):
            data = dict(result='PASS', phases=[dict(phase=p) for p in (
                'missing','live','encoders','aged','disconnected','recovered','returned')],
                shutdown='worker/timers/receivers/pipelines/ports closed', receiver_pipelines=2)
            if mode=='missing-phase':
                data['phases'].pop()
            if mode!='missing-file':
                self.report.write_text('not JSON' if mode=='malformed' else json.dumps(data))
            return b'Xlib: native diagnostic on stdout\n', (b'Traceback: callback error\n' if mode=='traceback' else b'')
    monkeypatch.setattr(runtime_smoke.subprocess, 'Popen', Child)
    passed, report = runtime_smoke.run_vision_smoke()
    assert passed is expected, report
