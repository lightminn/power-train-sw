from __future__ import annotations

import os
from pathlib import Path
import subprocess

from remote_video.contract import (
    D435I_UNAVAILABLE_VERDICT,
    L515_UNAVAILABLE_VERDICT,
)
from powertrain_ros.console_can_status import can_status_text
from scripts.fault_injection.channel_matrix import plan


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/fault_injection/run_channel_matrix.sh"
EXPECTED_CHANNELS = {
    "ros_dds",
    "l515_srt_receiver",
    "d435i_srt_receiver",
    "d435i_metadata",
    "remote_input",
    "arm_heartbeat",
    "can_telemetry",
    "l515_camera_owner",
    "d435i_camera_owner",
}


def _by_channel():
    steps = plan()
    assert {step.channel for step in steps} == EXPECTED_CHANNELS
    assert len(steps) == len(EXPECTED_CHANNELS)
    assert len({step.channel for step in steps}) == len(steps)
    return {step.channel: step for step in steps}


def test_plan_covers_every_task7_channel_once_with_supervised_replacement():
    steps = _by_channel()

    assert all(step.recovery == "supervised-replacement" for step in steps.values())
    assert all(step.expected_orphan_processes == 0 for step in steps.values())
    assert all(step.expected_journal_event_type for step in steps.values())
    assert all(step.kill_target.kind in {"compose_service", "process_pattern"} for step in steps.values())
    assert all(step.kill_target.value for step in steps.values())


def test_production_channel_targets_are_replaced_only_through_compose():
    steps = _by_channel()
    production_channels = {
        "ros_dds",
        "d435i_metadata",
        "arm_heartbeat",
        "can_telemetry",
        "l515_camera_owner",
        "d435i_camera_owner",
    }

    assert {
        channel: steps[channel].kill_target.kind
        for channel in sorted(production_channels)
    } == {channel: "compose_service" for channel in sorted(production_channels)}


def test_video_metadata_remote_arm_and_can_operation_contracts_are_exact():
    steps = _by_channel()

    l515 = steps["l515_srt_receiver"]
    assert l515.expected_holds == ("remote_drive",)
    assert l515.expected_effect == L515_UNAVAILABLE_VERDICT

    d435i = steps["d435i_srt_receiver"]
    assert d435i.expected_holds == ("remote_arm",)
    assert d435i.expected_effect == D435I_UNAVAILABLE_VERDICT

    metadata = steps["d435i_metadata"]
    assert metadata.expected_holds == ()
    assert metadata.expected_effect == "OVERLAY_STALE"

    remote = steps["remote_input"]
    assert remote.expected_holds == ("remote_drive", "remote_arm")
    assert remote.expected_effect == "REMOTE_INPUT_STALE"

    arm = steps["arm_heartbeat"]
    assert arm.expected_holds == ("drive", "arm_work")
    assert arm.expected_effect == "arm_status_stale"
    assert arm.expected_gate_value == 0

    can = steps["can_telemetry"]
    assert can.expected_holds == ()
    assert can.expected_effect == can_status_text(None, 0)


def test_camera_owner_and_ros_faults_hold_the_dependent_operations_only():
    steps = _by_channel()

    assert steps["ros_dds"].expected_holds == (
        "autonomy_drive",
        "mission_arm",
    )
    assert steps["l515_camera_owner"].expected_holds == (
        "autonomy_drive",
        "remote_drive",
    )
    assert steps["l515_camera_owner"].expected_effect == L515_UNAVAILABLE_VERDICT
    assert steps["d435i_camera_owner"].expected_holds == ("remote_arm", "arm_work")
    assert steps["d435i_camera_owner"].expected_effect == D435I_UNAVAILABLE_VERDICT


def test_shell_wrapper_is_valid_bash_and_names_gateway_replacement_rule():
    completed = subprocess.run(
        ["bash", "-n", str(SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    source = SCRIPT.read_text(encoding="utf-8")
    assert "docker compose" in source
    assert "supervised replacement" in source
    assert "RSUSB" in source
    assert "in-process" in source


def test_shell_wrapper_resolves_repo_python_imports_without_caller_pythonpath():
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)

    completed = subprocess.run(
        [str(SCRIPT), "--list"],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode == 0, completed.stderr
    assert set(completed.stdout.splitlines()) == EXPECTED_CHANNELS
