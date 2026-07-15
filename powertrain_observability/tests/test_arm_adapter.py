from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest


FAILURE_STATUSES = (
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
)
POSTURE_STATUSES = ("STOWED_LOCKED", "CARRYING_LOCKED")


def observation(
    status,
    *,
    accepted=True,
    contract_violation=None,
    hold_reason="",
):
    from powertrain_observability.arm_adapter import ArmObservation

    return ArmObservation(
        raw_status=status,
        source_mission_id=41,
        stamp_sec=123,
        stamp_nanosec=456_789_012,
        accepted=accepted,
        contract_violation=(
            not accepted
            if contract_violation is None
            else contract_violation
        ),
        current_mission_id=41,
        arm_posture="STOWED_LOCKED",
        hold_reason=hold_reason,
        source_detail="state=FAILED_HOLD;operation=PICKUP",
    )


@pytest.mark.parametrize("status", FAILURE_STATUSES + POSTURE_STATUSES)
def test_wp52_required_and_optional_results_become_arm_result(status):
    from powertrain_observability.arm_adapter import build_arm_events

    events = build_arm_events(
        observation(status, hold_reason=f"arm_failure:{status}"),
        wall_time_ns=10,
        monotonic_ns=20,
    )

    assert len(events) == 1
    event = events[0]
    assert event["event_type"] == "ARM_RESULT"
    assert event["severity"] == (
        "ERROR" if status in FAILURE_STATUSES else "INFO"
    )
    assert event["wall_time_ns"] == 10
    assert event["monotonic_ns"] == 20
    assert event["payload"]["raw_status"] == status


def test_failure_preserves_current_mission_posture_and_source_detail():
    from powertrain_observability.arm_adapter import build_arm_events

    event, = build_arm_events(
        observation("FAILED", hold_reason="arm_failure:FAILED"),
        wall_time_ns=10,
        monotonic_ns=20,
    )

    assert event["payload"] == {
        "result": (
            "FAILED stamp=123.456789012 mission_id=41 "
            "hold_reason=arm_failure:FAILED"
        ),
        "raw_status": "FAILED",
        "stamp": {"sec": 123, "nanosec": 456_789_012},
        "source_mission_id": 41,
        "mission_id": 41,
        "arm_posture": "STOWED_LOCKED",
        "hold_reason": "arm_failure:FAILED",
        "source_detail": "state=FAILED_HOLD;operation=PICKUP",
    }


def test_failed_alone_is_a_complete_supported_arm_result():
    from powertrain_observability.arm_adapter import build_arm_events

    event, = build_arm_events(
        observation("FAILED", hold_reason="arm_failure:FAILED")
    )

    assert event["event_type"] == "ARM_RESULT"
    assert event["payload"]["raw_status"] == "FAILED"


def test_unknown_status_records_contract_violation_and_tui_projection():
    from powertrain_observability.arm_adapter import build_arm_events

    events = build_arm_events(
        observation(
            "FUTURE_ARM_STATUS",
            accepted=False,
            hold_reason="arm_contract_violation:FUTURE_ARM_STATUS",
        ),
        wall_time_ns=10,
        monotonic_ns=20,
    )

    assert [event["event_type"] for event in events] == [
        "CONTRACT_VIOLATION",
        "ARM_RESULT",
    ]
    assert events[0]["severity"] == "ERROR"
    assert events[0]["payload"]["raw_status"] == "FUTURE_ARM_STATUS"
    assert events[0]["payload"]["stamp"] == {
        "sec": 123,
        "nanosec": 456_789_012,
    }
    assert events[1]["payload"]["projection"] == "CONTRACT_VIOLATION"
    assert events[1]["payload"]["result"] == (
        "FUTURE_ARM_STATUS stamp=123.456789012 mission_id=41 "
        "hold_reason=arm_contract_violation:FUTURE_ARM_STATUS"
    )


def test_observation_is_immutable_and_adapter_copies_hold_result_verbatim():
    from powertrain_observability.arm_adapter import build_arm_events

    item = observation("GRIP_LOST", hold_reason="grip_lost_latched")

    with pytest.raises(FrozenInstanceError):
        item.hold_reason = "adapter_decided_something_else"

    event, = build_arm_events(item)
    assert event["payload"]["hold_reason"] == "grip_lost_latched"
    assert "allow_drive" not in event["payload"]


def test_non_result_accepted_heartbeat_is_not_journaled():
    from powertrain_observability.arm_adapter import build_arm_events

    assert build_arm_events(observation("PERCEIVING")) == ()


@pytest.mark.parametrize("status", ("STOWED_LOCKED", "FAILED"))
def test_rejected_known_status_without_contract_violation_is_not_journaled(
    status,
):
    from powertrain_observability.arm_adapter import build_arm_events

    item = observation(
        status,
        accepted=False,
        contract_violation=False,
        hold_reason="",
    )

    assert build_arm_events(item) == ()


def test_chassis_callback_wires_adapter_through_simplenamespace_safe_guard():
    chassis_node = (
        Path(__file__).parents[2]
        / "ros2/src/powertrain_ros/powertrain_ros/chassis_node.py"
    )
    source = chassis_node.read_text(encoding="utf-8")

    assert "def _emit_arm_result_event(self, msg, accepted, result=None):" in source
    assert 'getattr(self, "_emit_arm_result_event", None)' in source
