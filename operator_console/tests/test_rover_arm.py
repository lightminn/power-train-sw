import json
import math

from operator_console.arm_telemetry import parse_arm_telemetry
from operator_console.rover_3d import ARM_JOINT_LIMITS, arm_state
from operator_console.rover_3d_demo import dummy_arm_payload


def sample(**overrides):
    payload = dummy_arm_payload(1, 0)
    payload.update(overrides)
    return parse_arm_telemetry(json.dumps(payload).encode(), received_monotonic_s=100)


def test_arm_uses_source_age_and_exact_cad_names():
    measured = sample()
    assert len(arm_state(measured, now_s=100.1)["arm"]) == 5
    assert arm_state(measured, now_s=101.1) == dict(arm_live=False, arm={})
    assert not arm_state(sample(source_age_s={}), now_s=100.1)["arm_live"]
    assert not arm_state(sample(source_age_s={"joints": .95}), now_s=100.1)["arm_live"]
    assert not arm_state(sample(source_age_s={"joints": -1}), now_s=100.1)["arm_live"]
    assert not arm_state(measured, now_s=float("nan"))["arm_live"]
    unknown = sample(joints=dict(names=["arm_joint_1"], position_rad=[.2], velocity=[]))
    assert not arm_state(unknown, now_s=100.1)["arm_live"]


def test_arm_rejects_out_of_limit_display_angles():
    invalid = sample(joints=dict(names=["arm_J2_shoulder"], position_rad=[.2], velocity=[]))
    assert not arm_state(invalid, now_s=100.1)["arm_live"]


def test_dummy_arm_stays_inside_cad_limits_and_changes_pose():
    first = dummy_arm_payload(1, 0)["joints"]["position_rad"]
    second = dummy_arm_payload(2, 2)["joints"]["position_rad"]
    assert math.dist(first, second) > .1
    for i in range(500):
        payload = dummy_arm_payload(i, i / 10)
        for name, angle in zip(payload["joints"]["names"], payload["joints"]["position_rad"]):
            low, high = ARM_JOINT_LIMITS[name]
            assert low <= angle <= high
