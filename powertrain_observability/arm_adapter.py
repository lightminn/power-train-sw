"""Pure adapter from immutable WP5.2 arm outcomes to journal events."""
from __future__ import annotations

from dataclasses import dataclass
import time


POSTURE_STATUSES = frozenset({"STOWED_LOCKED", "CARRYING_LOCKED"})
FAILURE_STATUSES = frozenset({
    "FAILED",
    "GRIP_LOST",
    "IK_FAILURE",
    "TRAJECTORY_FAILURE",
    "SELF_COLLISION",
    "BASE_COLLISION",
    "JOINT_OVERCURRENT",
    "GRIP_UNCERTAIN",
    "STOW_FAILURE",
    "ACTION_TIMEOUT",
})
ARM_RESULT_STATUSES = FAILURE_STATUSES | POSTURE_STATUSES


@dataclass(frozen=True)
class ArmObservation:
    """Already-decided WP5.2 result and its original wire fields."""

    raw_status: str
    source_mission_id: int
    stamp_sec: int
    stamp_nanosec: int
    accepted: bool
    contract_violation: bool
    current_mission_id: int | None
    arm_posture: str
    hold_reason: str
    source_detail: str


def _payload(observation: ArmObservation) -> dict:
    mission_id = (
        observation.source_mission_id
        if observation.current_mission_id is None
        else observation.current_mission_id
    )
    hold_reason = observation.hold_reason or "-"
    raw_stamp = (
        f"{observation.stamp_sec}.{observation.stamp_nanosec:09d}"
    )
    return {
        "result": (
            f"{observation.raw_status} stamp={raw_stamp} "
            f"mission_id={mission_id} "
            f"hold_reason={hold_reason}"
        ),
        "raw_status": observation.raw_status,
        "stamp": {
            "sec": observation.stamp_sec,
            "nanosec": observation.stamp_nanosec,
        },
        "source_mission_id": observation.source_mission_id,
        "mission_id": mission_id,
        "arm_posture": observation.arm_posture,
        "hold_reason": observation.hold_reason,
        "source_detail": observation.source_detail,
    }


def _event(
    event_type: str,
    severity: str,
    payload: dict,
    *,
    source: str,
    wall_time_ns: int,
    monotonic_ns: int,
) -> dict:
    return {
        "schema_version": 1,
        "wall_time_ns": int(wall_time_ns),
        "monotonic_ns": int(monotonic_ns),
        "source": str(source),
        "event_type": event_type,
        "severity": severity,
        "payload": payload,
    }


def build_arm_events(
    observation: ArmObservation,
    *,
    source: str = "chassis_node",
    wall_time_ns: int | None = None,
    monotonic_ns: int | None = None,
) -> tuple[dict, ...]:
    """Convert a prior WP5.2 decision without deriving drive permission."""
    wall_ns = time.time_ns() if wall_time_ns is None else int(wall_time_ns)
    mono_ns = (
        time.monotonic_ns()
        if monotonic_ns is None
        else int(monotonic_ns)
    )
    payload = _payload(observation)

    if observation.contract_violation:
        violation = _event(
            "CONTRACT_VIOLATION",
            "ERROR",
            payload,
            source=source,
            wall_time_ns=wall_ns,
            monotonic_ns=mono_ns,
        )
        # Task 2's five-file TUI contract reads the ARM_RESULT row. Keep the
        # violation as the authoritative journal event and add a presentation
        # projection carrying the identical raw fields.
        projection_payload = dict(payload)
        projection_payload["projection"] = "CONTRACT_VIOLATION"
        projection = _event(
            "ARM_RESULT",
            "ERROR",
            projection_payload,
            source=source,
            wall_time_ns=wall_ns,
            monotonic_ns=mono_ns,
        )
        return violation, projection

    if not observation.accepted:
        return ()

    if observation.raw_status not in ARM_RESULT_STATUSES:
        return ()

    severity = (
        "ERROR"
        if observation.raw_status in FAILURE_STATUSES
        else "INFO"
    )
    return (
        _event(
            "ARM_RESULT",
            severity,
            payload,
            source=source,
            wall_time_ns=wall_ns,
            monotonic_ns=mono_ns,
        ),
    )
