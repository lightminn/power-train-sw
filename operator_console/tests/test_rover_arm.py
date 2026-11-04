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
    assert arm_state(measured, now_s=101.1) == dict(arm_live=False, arm={}, tool_attached=False)
    assert not arm_state(sample(source_age_s={}), now_s=100.1)["arm_live"]
    assert not arm_state(sample(source_age_s={"joints": .95}), now_s=100.1)["arm_live"]
    assert not arm_state(sample(source_age_s={"joints": -1}), now_s=100.1)["arm_live"]
    assert not arm_state(measured, now_s=float("nan"))["arm_live"]
    unknown = sample(joints=dict(names=["arm_joint_9"], position_rad=[.2], velocity=[]))
    assert not arm_state(unknown, now_s=100.1)["arm_live"]


def test_bridge_joints_map_onto_cad_joints_from_the_matched_zero_pose():
    from operator_console.rover_3d import BRIDGE_TO_CAD
    names = list(BRIDGE_TO_CAD)
    zeros = [BRIDGE_TO_CAD[name][2] for name in names]
    at_zero = arm_state(sample(joints=dict(names=names, position_rad=zeros, velocity=[])),
                        now_s=100.1)
    assert at_zero["arm"] == {cad: 0.0 for cad, _sign, _zero in BRIDGE_TO_CAD.values()}
    # Raising the shoulder (bridge +) lifts the CAD shoulder toward negative.
    raised = sample(joints=dict(names=["arm_joint_2"], position_rad=[zeros[1] + .3], velocity=[]))
    assert arm_state(raised, now_s=100.1)["arm"] == {"arm_J2_shoulder": -.3}
    # Just below the matched zero, the shoulder is drawn at its CAD limit.
    lowered = sample(joints=dict(names=["arm_joint_2"], position_rad=[zeros[1] - .1], velocity=[]))
    assert arm_state(lowered, now_s=100.1)["arm"] == {"arm_J2_shoulder": 0.0}


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


def test_gripper_is_drawn_only_while_its_actuators_are_seen():
    def with_tool(age=0.0, **tool):
        tool = dict(tool_type="spur_1motor_gripper", actuator_ids=[5], **tool)
        return sample(tool_runtime={"source_age_s": age, "tool": tool})
    assert arm_state(with_tool(actuators_discovered=True), now_s=100.1)["tool_attached"]
    assert not arm_state(with_tool(actuators_discovered=False), now_s=100.1)["tool_attached"]
    assert not arm_state(with_tool(actuators_discovered=True, physical_tool_detached=True),
                         now_s=100.1)["tool_attached"]
    assert not arm_state(with_tool(age=5.0, actuators_discovered=True), now_s=100.1)["tool_attached"]
    assert not arm_state(sample(), now_s=100.1)["tool_attached"]
    assert not arm_state(None)["tool_attached"]
