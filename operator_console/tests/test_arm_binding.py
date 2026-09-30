from types import SimpleNamespace
import time

from operator_console.arm_binding import (
    contract_text,
    control_mode_text,
    fsm_text,
    refresh_arm_summary,
)


class Label:
    def __init__(self):
        self.text = None

    def set_text(self, value):
        self.text = value


def test_existing_arm_labels_use_fresh_read_only_runtime_and_raw_feedback():
    snapshot = SimpleNamespace(
        received_monotonic_s=time.monotonic(),
        joints_age_s=0.1,
        joint_names=("joint_1", "joint_2"),
        joint_effort_raw=(-120.0, 40.0),
        arm_runtime={
            "control_mode": "MANUAL",
            "fsm_state": "IDLE",
            "arm_status": "READY",
            "source_age_s": {
                "control_mode": 0.1,
                "fsm_state": 0.1,
                "arm_status": 0.1,
            },
        },
    )
    window = SimpleNamespace(_mission_arm_mode=Label(), _mission_arm_load=Label())

    refresh_arm_summary(window, snapshot)

    assert window._mission_arm_mode.text == "MANUAL"
    assert window._mission_arm_load.text == "최고 120 raw · 관절 2개"
    assert fsm_text(snapshot) == "IDLE"
    assert contract_text(snapshot) == "READY"


def test_missing_runtime_is_honest_waiting_state():
    snapshot = SimpleNamespace(
        received_monotonic_s=time.monotonic(),
        joints_age_s=None,
        joint_names=(),
        joint_effort_raw=(),
        arm_runtime=None,
    )
    window = SimpleNamespace(_mission_arm_mode=Label(), _mission_arm_load=Label())

    refresh_arm_summary(window, snapshot)

    assert control_mode_text(snapshot) == "수신 대기"
    assert window._mission_arm_load.text == "정보 없음"
