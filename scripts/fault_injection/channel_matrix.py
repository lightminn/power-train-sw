#!/usr/bin/env python3
"""Pure channel-fault plan for the WP5.3 Task 7 Jetson bench."""
from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import asdict, dataclass
import json

from powertrain_ros.console_can_status import can_status_text
from remote_video.contract import (
    D435I_UNAVAILABLE_VERDICT,
    L515_UNAVAILABLE_VERDICT,
)


@dataclass(frozen=True)
class KillTarget:
    kind: str
    value: str


@dataclass(frozen=True)
class FaultInjectionStep:
    channel: str
    kill_target: KillTarget
    expected_holds: tuple[str, ...]
    expected_effect: str
    expected_journal_event_type: str
    recovery: str = "supervised-replacement"
    expected_orphan_processes: int = 0
    expected_gate_value: int | None = None


_STEPS = (
    FaultInjectionStep(
        channel="ros_dds",
        kill_target=KillTarget("compose_service", "dds_router"),
        expected_holds=("autonomy_drive", "mission_arm"),
        expected_effect="ROS_DDS_UNAVAILABLE",
        expected_journal_event_type="CHANNEL_HEALTH",
    ),
    FaultInjectionStep(
        channel="l515_srt_receiver",
        kill_target=KillTarget("process_pattern", "gst-launch-1.0.*:5000"),
        expected_holds=("remote_drive",),
        expected_effect=L515_UNAVAILABLE_VERDICT,
        expected_journal_event_type="CHANNEL_HEALTH",
    ),
    FaultInjectionStep(
        channel="d435i_srt_receiver",
        kill_target=KillTarget("process_pattern", "gst-launch-1.0.*:5002"),
        expected_holds=("remote_arm",),
        expected_effect=D435I_UNAVAILABLE_VERDICT,
        expected_journal_event_type="CHANNEL_HEALTH",
    ),
    FaultInjectionStep(
        channel="d435i_metadata",
        kill_target=KillTarget("compose_service", "d435i_metadata_sender"),
        expected_holds=(),
        expected_effect="OVERLAY_STALE",
        expected_journal_event_type="CHANNEL_HEALTH",
    ),
    FaultInjectionStep(
        channel="remote_input",
        kill_target=KillTarget(
            "process_pattern", "motor_control.laptop.remote_operation_client"
        ),
        expected_holds=("remote_drive", "remote_arm"),
        expected_effect="REMOTE_INPUT_STALE",
        expected_journal_event_type="MOTION_HOLD",
    ),
    FaultInjectionStep(
        channel="arm_heartbeat",
        kill_target=KillTarget("compose_service", "robot_arm_control"),
        expected_holds=("drive", "arm_work"),
        expected_effect="arm_status_stale",
        expected_journal_event_type="MOTION_HOLD",
        expected_gate_value=0,
    ),
    FaultInjectionStep(
        channel="can_telemetry",
        kill_target=KillTarget("compose_service", "powertrain_observability"),
        expected_holds=(),
        expected_effect=can_status_text(None, 0),
        expected_journal_event_type="CAN_HEALTH",
    ),
    FaultInjectionStep(
        channel="l515_camera_owner",
        kill_target=KillTarget("compose_service", "powertrain_ros"),
        expected_holds=("autonomy_drive", "remote_drive"),
        expected_effect=L515_UNAVAILABLE_VERDICT,
        expected_journal_event_type="CHANNEL_HEALTH",
    ),
    FaultInjectionStep(
        channel="d435i_camera_owner",
        kill_target=KillTarget("compose_service", "ros2_humble"),
        expected_holds=("remote_arm", "arm_work"),
        expected_effect=D435I_UNAVAILABLE_VERDICT,
        expected_journal_event_type="CHANNEL_HEALTH",
    ),
)


def plan() -> tuple[FaultInjectionStep, ...]:
    """Return the immutable, ordered one-channel-at-a-time bench steps."""
    return _STEPS


def _step(channel: str) -> FaultInjectionStep:
    for step in _STEPS:
        if step.channel == channel:
            return step
    raise ValueError(f"unknown channel: {channel}")


def _document(step: FaultInjectionStep) -> dict:
    document = asdict(step)
    document["expected_holds"] = list(step.expected_holds)
    return document


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--channel")
    parser.add_argument(
        "--field",
        choices=(
            "kill_kind",
            "kill_value",
            "expected_holds",
            "expected_effect",
            "expected_journal_event_type",
        ),
    )
    arguments = parser.parse_args(argv)
    if arguments.list:
        print("\n".join(step.channel for step in _STEPS))
        return 0
    if not arguments.channel:
        parser.error("--channel is required unless --list is used")
    step = _step(arguments.channel)
    if arguments.field is None:
        print(json.dumps(_document(step), sort_keys=True, allow_nan=False))
        return 0
    values = {
        "kill_kind": step.kill_target.kind,
        "kill_value": step.kill_target.value,
        "expected_holds": ",".join(step.expected_holds),
        "expected_effect": step.expected_effect,
        "expected_journal_event_type": step.expected_journal_event_type,
    }
    print(values[arguments.field])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
