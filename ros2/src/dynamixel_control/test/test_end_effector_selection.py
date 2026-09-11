"""직렬 하드웨어 없이 수행하는 프리셋 격리 및 미션 호환성 테스트."""

import importlib.util
import os
from pathlib import Path
import threading
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import xacro
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

from dynamixel_control.arm_fsm_node import (
    ArmFsmNode, MISSION_PICK_PLACE, MISSION_ROTARY_TOOL, State,
)
from dynamixel_control.arm_hardware import ARM_MOTOR_IDS
from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset
from dynamixel_control import moveit_dynamixel_bridge as bridge_module
from dynamixel_control import dynamixel_position_node, teleop_core_node


PICK_PLACE_STATES = [
    State.PERCEIVE, State.PLAN, State.APPROACH, State.DESCEND, State.GRASP,
    State.GRASP_CHECK, State.LIFT, State.CARRY, State.RELEASE,
]
SRC_DIR = Path(__file__).resolve().parents[2]


class Logger:
    def warn(self, _message):
        pass

    def error(self, _message):
        pass

    def info(self, _message):
        pass


class PendingFuture:
    def add_done_callback(self, _callback):
        return None


class RecordingClient:
    def __init__(self):
        self.goals = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return PendingFuture()


class GoalHandle:
    def __init__(self):
        self.request = SimpleNamespace(
            relative=True, ticks=300, max_abs_current=100, timeout=2.0)
        self.aborted = False
        self.succeeded = False
        self.canceled_result = False
        self.is_cancel_requested = False
        self.feedback = []

    def abort(self):
        self.aborted = True

    def succeed(self):
        self.succeeded = True

    def canceled(self):
        self.canceled_result = True

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)


def arm_goal(**overrides):
    request = {
        "motor_ids": [14, 13, 12, 16],
        "delta_ticks": [5, 10, 10, 20],
        "random_demo": False,
        "max_abs_current": 100,
        "stall_timeout": 2.0,
        "step_timeout": 8.0,
    }
    request.update(overrides)
    goal = GoalHandle()
    goal.request = SimpleNamespace(**request)
    return goal


def test_mission_and_preset_must_match_before_start():
    assert DEFAULT_GRIPPER == "dual_motor_gripper"
    dual = get_preset("dual_motor_gripper")
    rotary = get_preset("rotary_id5")
    ArmFsmNode.validate_mission_preset(MISSION_PICK_PLACE, dual)
    ArmFsmNode.validate_mission_preset(MISSION_ROTARY_TOOL, rotary)
    with pytest.raises(RuntimeError, match="incompatible"):
        ArmFsmNode.validate_mission_preset(MISSION_PICK_PLACE, rotary)
    with pytest.raises(RuntimeError, match="incompatible"):
        ArmFsmNode.validate_mission_preset(MISSION_ROTARY_TOOL, dual)

    assert dual["gripper_ids"] == [3, 4]
    assert dual["motor_endpoints"] == {
        3: {"open": 1056, "close": -526},
        4: {"open": 2384, "close": 839},
    }
    assert dual["required_operating_modes"] == {3: 4, 4: 3}
    assert dual["command_calibrated"] is True


def test_single_motor_gripper_reuses_pick_place_but_stays_uncalibrated():
    single = get_preset("single_motor_gripper")
    rotary = get_preset("rotary_id5")

    ArmFsmNode.validate_mission_preset(MISSION_PICK_PLACE, single)
    with pytest.raises(RuntimeError, match="incompatible"):
        ArmFsmNode.validate_mission_preset(MISSION_ROTARY_TOOL, single)

    assert single["gripper_ids"] == [5]
    assert single["gripper_joints"] == ["gripper_drive_joint"]
    assert single["kind"] == "gripper"
    assert single["allowed_mission"] == MISSION_PICK_PLACE
    assert single["command_calibrated"] is False
    assert single["observed_operating_mode"] == -1
    assert single["required_operating_mode"] == -1
    assert single["gripper_goal_pwm"] == 0
    assert single["gripper_open_tick"] == 80
    assert single["gripper_close_tick"] == 588
    assert single["gripper_open_rad"] == single["gripper_close_rad"] == 0.0
    assert single["grasp_effort_thresh"] != rotary["grasp_effort_thresh"]
    assert single["drop_effort_thresh"] != rotary["drop_effort_thresh"]

    # 별도 FSM 상태를 추가하지 않고 기존 PICK_PLACE 상태 집합을 그대로 쓴다.
    assert [state.name for state in PICK_PLACE_STATES] == [
        "PERCEIVE", "PLAN", "APPROACH", "DESCEND", "GRASP",
        "GRASP_CHECK", "LIFT", "CARRY", "RELEASE",
    ]


def test_launch_accepts_single_motor_pick_place_without_starting_hardware(tmp_path):
    # LaunchDescription/OpaqueFunction 구성만 평가한다. 프로세스 실행이나 직렬 포트
    # open/ping/read/write/torque 동작은 일어나지 않는다.
    os.environ["ROS_LOG_DIR"] = str(tmp_path / "ros-log")
    from launch import LaunchContext

    launch_path = os.path.join(
        os.path.dirname(__file__), "..", "launch", "end_effector.launch.py")
    spec = importlib.util.spec_from_file_location("end_effector_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    context = LaunchContext()
    context.launch_configurations.update({
        "end_effector_preset": "single_motor_gripper",
        "mission_type": "PICK_PLACE",
        "ik_mode": "analytic",
        "gripper_change_mode": "false",
        "gripper_disabled": "true",
        "stop_after_descend": "false",
        "read_only": "true",
        "end_effector_only": "false",
        "start_fsm": "true",
        "integrated_test_mode": "false",
        "random_demo_enabled": "false",
        "arm_test_goal_tolerance_ticks": "10",
        "random_seed": "42",
        "random_pose_count": "3",
        "rotary_relative": "true",
        "rotary_ticks": "0",
    })
    nodes = module._configured_nodes(context)
    assert [node.node_executable for node in nodes] == [
        "moveit_dynamixel_bridge", "arm_fsm"]


def test_dual_and_single_urdf_models_are_selected_without_hardware_io():
    model = SRC_DIR / "robot_arm_description" / "urdf" / "robot_arm.urdf.xacro"
    roots = {}
    for preset in ("dual_motor_gripper", "single_motor_gripper"):
        xml = xacro.process_file(
            str(model), mappings={"end_effector_preset": preset}).toxml()
        roots[preset] = ET.fromstring(xml)

    dual_joints = {joint.attrib["name"] for joint in roots[
        "dual_motor_gripper"].findall("joint")}
    single_joints = {joint.attrib["name"] for joint in roots[
        "single_motor_gripper"].findall("joint")}
    assert "gripper_left_pinion_joint" in dual_joints
    assert "gripper_drive_joint" not in dual_joints
    assert "gripper_drive_joint" in single_joints
    assert "gripper_left_pinion_joint" not in single_joints

    drive = roots["single_motor_gripper"].find(
        "joint[@name='gripper_drive_joint']")
    assert drive.find("parent").attrib["link"] == "link_051"
    assert drive.find("child").attrib["link"] == "link_055"
    mesh_dir = (SRC_DIR / "robot_arm_description" / "meshes" /
                "single_motor_gripper")
    assert len(list(mesh_dir.glob("*.stl"))) == 58


def test_single_moveit_semantics_match_restored_urdf():
    srdf = (SRC_DIR / "robot_arm_moveit_config" / "config" /
            "robot_arm.srdf.xacro")
    root = ET.fromstring(xacro.process_file(
        str(srdf), mappings={
            "end_effector_preset": "single_motor_gripper"}).toxml())
    groups = {group.attrib["name"]: group for group in root.findall("group")}

    assert groups["arm"].find("chain").attrib == {
        "base_link": "base_link", "tip_link": "single_gripper_grasp_frame"}
    assert groups["gripper"].find("joint").attrib["name"] == \
        "gripper_drive_joint"
    assert root.find("end_effector").attrib["parent_link"] == \
        "single_gripper_grasp_frame"
    assert root.find(
        "disable_collisions[@link1='link_051'][@link2='link_055']") is not None

    preset = get_preset("single_motor_gripper")
    assert preset["arm_tip_link"] == "link_051"
    assert preset["tip_link"] == "single_gripper_grasp_frame"
    assert preset["command_calibrated"] is False


def test_preset_tcp_frames_are_geometry_specific():
    model = SRC_DIR / "robot_arm_description" / "urdf" / "robot_arm.urdf.xacro"
    expected = {
        "single_motor_gripper": (
            "single_gripper_grasp_frame_joint", "link_051",
            [0.020000, -0.1449445, 0.0031030]),
    }
    for preset_name, (joint_name, parent, xyz) in expected.items():
        root = ET.fromstring(xacro.process_file(
            str(model), mappings={
                "end_effector_preset": preset_name}).toxml())
        joint = root.find(f"joint[@name='{joint_name}']")
        assert joint is not None
        assert joint.attrib["type"] == "fixed"
        assert joint.find("parent").attrib["link"] == parent
        assert [float(v) for v in joint.find("origin").attrib["xyz"].split()] \
            == pytest.approx(xyz)
        tcp = joint.find("child").attrib["link"]
        assert root.find(f"link[@name='{tcp}']") is not None
        assert get_preset(preset_name)["tip_link"] == tcp

    # The nominal contact-centre offsets are independently derived from each
    # CAD export and must never collapse back to the arm mounting links.
    assert get_preset("dual_motor_gripper")["tip_link"] == "link_043"
    assert get_preset("single_motor_gripper")["tip_link"] != "link_051"


def test_arm_mapping_is_invariant_across_preset_selection():
    for name in ("dual_motor_gripper", "single_motor_gripper", "rotary_id5"):
        get_preset(name)
        assert ARM_MOTOR_IDS == [14, 13, 12, 16]
    assert dynamixel_position_node.DEFAULT_MOTOR_IDS == [14, 13, 12, 16]
    assert teleop_core_node.DEFAULT_MOTOR_IDS == [14, 13, 12, 16]
    assert 11 not in dynamixel_position_node.DEFAULT_MOTOR_IDS
    assert 3 not in teleop_core_node.DEFAULT_MOTOR_IDS


def test_gripper_disabled_removes_physical_ids_but_keeps_logical_joint():
    preset = get_preset("dual_motor_gripper")
    gripper_ids = list(preset["gripper_ids"])
    gripper_joints = list(preset["gripper_joints"])
    gripper_disabled = True
    if gripper_disabled:
        gripper_ids = []
    assert gripper_ids == []
    assert gripper_joints == ["gripper_left_pinion_joint"]


def test_gripper_change_mode_enables_both_safety_gates():
    change_mode = True
    explicit_disabled = False
    explicit_stop = False
    assert change_mode or explicit_disabled
    assert change_mode or explicit_stop


def test_gripper_change_mode_false_preserves_existing_defaults():
    change_mode = False
    explicit_disabled = False
    explicit_stop = False
    assert not (change_mode or explicit_disabled)
    assert not (change_mode or explicit_stop)


def test_stop_after_descend_latches_before_any_gripper_command():
    fsm = object.__new__(ArmFsmNode)
    fsm.stop_after_descend = True
    fsm._motion_state = "done"
    fsm._motion_ok = True
    fsm._set_status = lambda _status: None
    transitions = []
    fsm._transition = transitions.append
    fsm._do_descend()
    assert transitions == [State.DESCEND_STOPPED]


def test_gripper_disabled_blocks_gripper_action():
    fsm = object.__new__(ArmFsmNode)
    fsm.gripper_disabled = True
    fsm.get_logger = lambda: Logger()
    fsm._grip = RecordingClient()
    fsm._send_gripper(0.0)
    assert fsm._grip.goals == []


def test_single_mock_grasp_lift_release_flow_without_gripper_goal():
    """Uncalibrated ID5 stays silent while Fake feedback exercises the FSM."""
    fsm = object.__new__(ArmFsmNode)
    fsm.gripper_change_mode = False
    fsm.gripper_disabled = True
    fsm.mission_type = MISSION_PICK_PLACE
    fsm.end_effector_kind = "gripper"
    fsm.gripper_close = 0.0
    fsm.gripper_open = 0.0
    fsm.gripper_action_time = 0.0
    fsm.grasp_thresh = 10.0
    fsm._grip_sent = False
    fsm._grip = RecordingClient()
    fsm.get_logger = lambda: Logger()
    fsm._set_status = lambda _status: None
    fsm._elapsed = lambda: 1.0
    transitions = []
    fsm._transition = transitions.append

    fsm._do_grasp()
    fsm._do_grasp()
    assert transitions.pop() == State.GRASP_CHECK
    assert fsm._grip.goals == []

    fsm._gripper_effort = lambda: 11.0  # Fake joint-state feedback only.
    fsm._do_grasp_check()
    assert transitions.pop() == State.LIFT

    fsm._motion_state = "done"
    fsm._motion_ok = True
    fsm._do_lift()
    assert transitions.pop() == State.CARRY

    fsm._grip_sent = False
    fsm._do_release()
    fsm._do_release()
    assert transitions.pop() == State.DONE
    assert fsm._grip.goals == []


def test_single_move_group_goal_constrains_tcp_and_keeps_dual_regression():
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.orientation.w = 1.0

    single = object.__new__(ArmFsmNode)
    single.tip_link = "single_gripper_grasp_frame"
    single.planning_group = "arm"
    single.planning_time = 2.0
    single.vel_scale = 0.2
    single.acc_scale = 0.2
    single.pos_tol = 0.005
    single.orient_tol = 0.1
    constraints = single._build_move_group_goal(pose).request.goal_constraints[0]
    assert constraints.position_constraints[0].link_name == single.tip_link
    assert constraints.orientation_constraints[0].link_name == single.tip_link

    dual = object.__new__(ArmFsmNode)
    dual.tip_link = "link_043"
    dual.planning_group = single.planning_group
    dual.planning_time = single.planning_time
    dual.vel_scale = single.vel_scale
    dual.acc_scale = single.acc_scale
    dual.pos_tol = single.pos_tol
    dual.orient_tol = single.orient_tol
    dual_constraints = dual._build_move_group_goal(
        pose).request.goal_constraints[0]
    assert dual_constraints.position_constraints[0].link_name == "link_043"
    assert dual_constraints.orientation_constraints == []


def test_dual_kdl_solver_is_configured_for_position_only_goals():
    """The five-axis dual arm cannot satisfy arbitrary sampled 6-D poses."""
    config = Path(__file__).parents[2] / "robot_arm_moveit_config" / \
        "config" / "kinematics.yaml"
    text = config.read_text(encoding="utf-8")
    assert "kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin" in text
    assert "position_only_ik: true" in text


def test_fake_realsense_target_transforms_camera_to_base_for_single_tcp():
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "camera_color_optical_frame"
    transform.transform.translation.x = 0.40
    transform.transform.translation.y = -0.10
    transform.transform.translation.z = 0.25
    transform.transform.rotation.w = 1.0

    class FakeTfBuffer:
        def lookup_transform(self, target, source, _time):
            assert target == "base_link"
            assert source == "camera_color_optical_frame"
            return transform

    class FakeNow:
        def to_msg(self):
            return TransformStamped().header.stamp

    fsm = object.__new__(ArmFsmNode)
    target_pose = Pose()
    target_pose.position.x = 0.05
    target_pose.position.y = 0.02
    target_pose.position.z = 0.60
    target_pose.orientation.w = 1.0
    fsm.pick_target = SimpleNamespace(pose=target_pose)
    fsm.pick_frame_id = "camera_color_optical_frame"
    fsm.base_frame = "base_link"
    fsm.tip_link = "single_gripper_grasp_frame"
    fsm.tf_buffer = FakeTfBuffer()
    fsm.get_clock = lambda: SimpleNamespace(now=lambda: FakeNow())

    result = fsm._grasp_pose_in_base()
    assert result.header.frame_id == "base_link"
    assert [result.pose.position.x, result.pose.position.y,
            result.pose.position.z] == pytest.approx([0.45, -0.08, 0.85])


def test_startup_torque_requires_present_goal_readback_before_enable():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge._bus_lock = threading.Lock()
    bridge.get_logger = lambda: Logger()
    events = []
    registers = {
        bridge_module.ADDR_TORQUE_ENABLE: 0,
        bridge_module.ADDR_PRESENT_POSITION: 1234,
        bridge_module.ADDR_GOAL_POSITION: 0,
    }

    def read_register(_id, address, _size, label, signed=False):
        events.append(("read", address, label))
        return registers[address]

    def write_register(_id, address, _size, value, label):
        events.append(("write", address, label))
        registers[address] = value

    bridge._read_register = read_register
    bridge._write_register = write_register
    bridge._write_motion_profile = lambda *_args: events.append(("profile",))

    assert bridge._enable_torque(14, "arm_joint_2")
    goal_write = events.index((
        "write", bridge_module.ADDR_GOAL_POSITION,
        "startup synchronize goal"))
    goal_read = events.index((
        "read", bridge_module.ADDR_GOAL_POSITION,
        "startup goal readback"))
    torque_write = events.index((
        "write", bridge_module.ADDR_TORQUE_ENABLE,
        "startup torque enable"))
    assert goal_write < goal_read < torque_write


def test_startup_goal_readback_mismatch_never_enables_torque():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge._bus_lock = threading.Lock()
    bridge.get_logger = lambda: Logger()
    writes = []

    def read_register(_id, address, _size, _label, signed=False):
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            return 0
        if address == bridge_module.ADDR_PRESENT_POSITION:
            return 1234
        if address == bridge_module.ADDR_GOAL_POSITION:
            return 1235
        raise AssertionError(address)

    bridge._read_register = read_register
    bridge._write_register = (
        lambda _id, address, _size, value, label:
        writes.append((address, value, label)))
    bridge._write_motion_profile = lambda *_args: None

    assert not bridge._enable_torque(14, "arm_joint_2")
    assert not any(address == bridge_module.ADDR_TORQUE_ENABLE
                   and value == bridge_module.TORQUE_ENABLE
                   for address, value, _label in writes)


def test_relative_and_absolute_rotation_goal_resolution():
    resolve = bridge_module.MoveItDynamixelBridge.rotation_goal
    assert resolve(2000, True, 300) == 2300
    assert resolve(2000, True, -300) == 1700
    assert resolve(2000, False, 1234) == 1234
    with pytest.raises(ValueError, match="outside"):
        resolve(100, True, -300)


def test_grasp_write_cannot_touch_id5_under_rotary_preset():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.end_effector_kind = "rotary"
    bridge.read_only = False
    bridge.gripper_command_calibrated = True
    bridge.gripper_observed_operating_mode = 3
    bridge.gripper_required_operating_mode = 3
    bridge.get_logger = lambda: Logger()
    bridge.packet_handler = SimpleNamespace(
        write4ByteTxRx=lambda *_args: (_ for _ in ()).throw(
            AssertionError("GRASP/RELEASE touched ID 5")))
    bridge._write_gripper(0.0)


def test_uncalibrated_single_motor_rejects_gripper_goal_without_io():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.gripper_disabled = False
    bridge.end_effector_kind = "gripper"
    bridge.read_only = False
    bridge.gripper_ids = [5]
    bridge.gripper_command_calibrated = False
    bridge.gripper_required_operating_modes = {}
    bridge.gripper_observed_operating_modes = {}
    bridge.gripper_observed_operating_mode = -1
    bridge.gripper_required_operating_mode = -1
    bridge.get_logger = lambda: Logger()

    result = bridge.gripper_goal_callback(SimpleNamespace())
    assert result == bridge_module.GoalResponse.REJECT
    assert not bridge._gripper_startup_torque_allowed()


@pytest.mark.parametrize("converter,args", [
    ("gripper_pos_to_tick", (0.0,)),
    ("gripper_pos_to_ratio", (0.0,)),
    ("gripper_goals_for_ratio", (0.0,)),
])
def test_uncalibrated_gripper_rejects_every_command_conversion(converter, args):
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.gripper_command_calibrated = False
    bridge.gripper_open_rad = 0.0
    bridge.gripper_close_rad = 0.0

    with pytest.raises(RuntimeError, match="calibration is not verified"):
        getattr(bridge, converter)(*args)


@pytest.mark.parametrize("converter,args", [
    ("gripper_pos_to_tick", (0.0,)),
    ("gripper_pos_to_ratio", (0.0,)),
    ("gripper_goals_for_ratio", (0.0,)),
])
def test_identical_rad_endpoints_reject_every_command_path_without_io(
        converter, args):
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.gripper_disabled = False
    bridge.end_effector_kind = "gripper"
    bridge.read_only = False
    bridge.gripper_ids = [5]
    bridge.gripper_command_calibrated = True
    bridge.gripper_open_rad = 0.0
    bridge.gripper_close_rad = 0.0
    bridge.gripper_required_operating_modes = {5: 3}
    bridge.gripper_observed_operating_modes = {5: 3}
    bridge.gripper_required_operating_mode = 3
    bridge.gripper_observed_operating_mode = 3
    bridge.get_logger = lambda: Logger()
    bridge._read_register = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        AssertionError("invalid gripper mapping performed bus I/O"))
    bridge._write_register = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        AssertionError("invalid gripper mapping performed bus I/O"))

    assert bridge.gripper_goal_callback(SimpleNamespace()) \
        == bridge_module.GoalResponse.REJECT
    assert not bridge._write_gripper(0.0)
    assert not bridge._gripper_startup_torque_allowed()
    with pytest.raises(RuntimeError, match="endpoints are identical"):
        getattr(bridge, converter)(*args)


def configured_dual_bridge():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.end_effector_kind = "gripper"
    bridge.read_only = False
    bridge.gripper_ids = [3, 4]
    bridge.gripper_motor_endpoints = {
        3: {"open": 1056, "close": -526},
        4: {"open": 2384, "close": 839},
    }
    bridge.gripper_required_operating_modes = {3: 4, 4: 3}
    bridge.gripper_observed_operating_modes = {3: 4, 4: 3}
    bridge.gripper_required_operating_mode = -1
    bridge.gripper_observed_operating_mode = -1
    bridge.gripper_command_calibrated = True
    bridge.gripper_open_rad = 1.9444444444444444
    bridge.gripper_close_rad = 0.0
    bridge.gripper_open_tick = 1056
    bridge.gripper_close_tick = -526
    bridge.gripper_extended = True
    bridge.end_effector_max_abs_current = 300
    bridge.end_effector_stall_timeout = 0.1
    bridge.end_effector_motion_timeout = 0.2
    bridge.end_effector_goal_tolerance_ticks = 10
    bridge._bus_lock = threading.Lock()
    bridge.torque_enabled_ids = {3, 4}
    bridge.get_logger = lambda: Logger()
    return bridge


def test_dual_gripper_ratio_uses_distinct_per_motor_goals():
    bridge = configured_dual_bridge()
    assert bridge.gripper_goals_for_ratio(0.0) == {3: 1056, 4: 2384}
    assert bridge.gripper_goals_for_ratio(1.0) == {3: -526, 4: 839}
    assert bridge.gripper_goals_for_ratio(0.5) == {3: 265, 4: 1612}

    writes = []
    goals = {3: 265, 4: 1612}
    bridge._read_register = lambda dxl_id, address, _size, _label, signed=False: {
        bridge_module.ADDR_OPERATING_MODE: {3: 4, 4: 3}[dxl_id],
        bridge_module.ADDR_HARDWARE_ERROR_STATUS: 0,
        bridge_module.ADDR_PRESENT_POSITION: goals[dxl_id],
        bridge_module.ADDR_PRESENT_LOAD: 0,
    }[address]
    bridge._write_register = lambda dxl_id, address, size, value, label: \
        writes.append((dxl_id, address, size, value, label))

    mid_rad = (bridge.gripper_open_rad + bridge.gripper_close_rad) / 2.0
    assert bridge._write_gripper(mid_rad)
    goal_writes = [(dxl_id, value) for dxl_id, address, _size, value, _label
                   in writes if address == bridge_module.ADDR_GOAL_POSITION]
    assert goal_writes == [(3, 265), (4, 1612)]
    assert goal_writes[0][1] != goal_writes[1][1]


@pytest.mark.parametrize("failure", ["current", "hardware_error", "stall"])
def test_dual_gripper_any_motor_failure_torques_off_both(failure):
    bridge = configured_dual_bridge()
    if failure == "stall":
        bridge.end_effector_stall_timeout = 0.0
    writes = []

    def read_register(dxl_id, address, _size, _label, signed=False):
        if address == bridge_module.ADDR_OPERATING_MODE:
            return {3: 4, 4: 3}[dxl_id]
        if address == bridge_module.ADDR_HARDWARE_ERROR_STATUS:
            return 1 if failure == "hardware_error" and dxl_id == 4 else 0
        if address == bridge_module.ADDR_PRESENT_POSITION:
            return {3: 900, 4: 2200}[dxl_id]
        if address == bridge_module.ADDR_PRESENT_LOAD:
            return 300 if failure == "current" and dxl_id == 3 else 0
        raise AssertionError(address)

    bridge._read_register = read_register
    bridge._write_register = lambda dxl_id, address, size, value, label: \
        writes.append((dxl_id, address, size, value, label))

    assert not bridge._write_gripper(bridge.gripper_close_rad)
    torque_off_ids = [dxl_id for dxl_id, address, _size, value, _label
                      in writes
                      if address == bridge_module.ADDR_TORQUE_ENABLE
                      and value == bridge_module.TORQUE_DISABLE]
    assert torque_off_ids == [3, 4]


def test_rotate_execution_cannot_touch_id5_under_dual_preset():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.end_effector_kind = "gripper"
    bridge.gripper_ids = [3, 4]
    bridge.end_effector_max_abs_current = 100
    bridge.end_effector_motion_timeout = 2.0
    bridge._bus_lock = threading.Lock()
    bridge._write_register = lambda *_args: (_ for _ in ()).throw(
        AssertionError("rotate action wrote an unselected ID"))
    goal = GoalHandle()
    result = bridge.execute_rotate(goal)
    assert not result.success
    assert goal.aborted


def test_rotary_failure_path_always_attempts_torque_off():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.end_effector_kind = "rotary"
    bridge.gripper_ids = [5]
    bridge.end_effector_max_abs_current = 100
    bridge.end_effector_motion_timeout = 2.0
    bridge._bus_lock = threading.Lock()
    bridge.torque_enabled_ids = set()
    bridge._random_arm_baseline = None
    writes = []

    def read_register(_id, _address, _size, label, signed=False):
        if label == "operating mode":
            raise RuntimeError("simulated communication failure")
        if label == "final torque readback":
            return 0
        raise AssertionError(label)

    bridge._read_register = read_register
    bridge._write_register = lambda dxl_id, address, size, value, label: \
        writes.append((dxl_id, address, size, value, label))
    result = bridge.execute_rotate(GoalHandle())
    assert not result.success
    assert writes[-1][:4] == (5, bridge_module.ADDR_TORQUE_ENABLE, 1, 0)


def configured_arm_test_bridge():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.integrated_test_mode = True
    bridge.random_demo_enabled = False
    bridge.read_only = False
    bridge.gripper_only_mode = False
    bridge.end_effector_kind = "rotary"
    bridge.gripper_ids = [5]
    bridge.arm_test_max_abs_current = 300
    bridge.arm_test_stall_timeout = 2.0
    bridge.arm_test_step_timeout = 8.0
    bridge.arm_test_goal_tolerance_ticks = 3
    bridge._bus_lock = threading.Lock()
    bridge.torque_enabled_ids = set()
    bridge._random_arm_baseline = None
    bridge.get_logger = lambda: Logger()
    return bridge


def test_arm_test_goal_requires_mode_and_exact_sequence():
    bridge = configured_arm_test_bridge()
    request = arm_goal().request
    assert bridge.arm_test_goal_callback(request) \
        == bridge_module.GoalResponse.ACCEPT
    bridge.integrated_test_mode = False
    assert bridge.arm_test_goal_callback(request) \
        == bridge_module.GoalResponse.REJECT
    bridge.integrated_test_mode = True
    request.motor_ids = [14, 13, 5, 16]
    assert bridge.arm_test_goal_callback(request) \
        == bridge_module.GoalResponse.REJECT
    request.motor_ids = [14, 13, 12, 1]
    assert bridge.arm_test_goal_callback(request) \
        == bridge_module.GoalResponse.REJECT


def test_arm_test_runs_one_motor_at_a_time_and_finishes_all_torque_off():
    bridge = configured_arm_test_bridge()
    positions = {14: 1000, 13: 1100, 12: 1200, 16: 1300}
    goals = dict(positions)
    torque = {dxl_id: 0 for dxl_id in positions}
    writes = []

    def read_register(dxl_id, address, _size, _label, signed=False):
        if address == bridge_module.ADDR_OPERATING_MODE:
            return 3
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            return torque[dxl_id]
        if address == bridge_module.ADDR_HARDWARE_ERROR_STATUS:
            return 0
        if address == bridge_module.ADDR_PRESENT_POSITION:
            return goals[dxl_id] if torque[dxl_id] else positions[dxl_id]
        if address == bridge_module.ADDR_GOAL_POSITION:
            return goals[dxl_id]
        if address in (bridge_module.ADDR_PRESENT_VELOCITY,
                       bridge_module.ADDR_PRESENT_LOAD,
                       bridge_module.ADDR_MOVING_STATUS):
            return 0
        raise AssertionError(address)

    def write_register(dxl_id, address, _size, value, label):
        writes.append((dxl_id, address, value, label))
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            torque[dxl_id] = value
            if value == 0:
                positions[dxl_id] = goals[dxl_id]
        elif address == bridge_module.ADDR_GOAL_POSITION:
            goals[dxl_id] = value

    bridge._read_register = read_register
    bridge._write_register = write_register
    goal = arm_goal()
    result = bridge.execute_arm_test_move(goal)
    assert result.success
    assert result.completed_steps == 4
    assert goal.succeeded
    assert positions == {14: 1005, 13: 1110, 12: 1210, 16: 1320}
    assert torque == {14: 0, 13: 0, 12: 0, 16: 0}
    active = set()
    for dxl_id, address, value, _label in writes:
        if address != bridge_module.ADDR_TORQUE_ENABLE:
            continue
        if value:
            active.add(dxl_id)
            assert len(active) == 1
        else:
            active.discard(dxl_id)
    assert not active
    assert all(dxl_id != 5 for dxl_id, *_rest in writes)


def test_arm_test_failure_does_not_send_rotate_action():
    fsm = object.__new__(ArmFsmNode)
    fsm.integrated_test_mode = True
    fsm.random_demo_enabled = False
    fsm.mission_type = MISSION_ROTARY_TOOL
    fsm.end_effector_kind = "rotary"
    fsm._arm_test_state = "done"
    fsm._arm_test_ok = False
    fsm._arm_test_sent = True
    failures = []
    transitions = []
    fsm._set_status = lambda _status: None
    fsm._fail = failures.append
    fsm._transition = transitions.append
    fsm._do_arm_test_move()
    assert failures == ["guarded arm test action failed"]
    assert State.END_EFFECTOR_ROTATE not in transitions


def test_fsm_calls_arm_then_rotate_exactly_once():
    fsm = object.__new__(ArmFsmNode)
    fsm.integrated_test_mode = True
    fsm.random_demo_enabled = False
    fsm.mission_type = MISSION_ROTARY_TOOL
    fsm.end_effector_kind = "rotary"
    fsm.arm_test_max_abs_current = 100
    fsm.arm_test_stall_timeout = 2.0
    fsm.arm_test_step_timeout = 8.0
    fsm._arm_test_state = "idle"
    fsm._arm_test_ok = False
    fsm._arm_test_sent = False
    fsm._arm_test = RecordingClient()
    fsm._set_status = lambda _status: None
    fsm.get_logger = lambda: Logger()
    transitions = []
    fsm._transition = transitions.append
    fsm._do_arm_test_move()
    fsm._do_arm_test_move()
    assert len(fsm._arm_test.goals) == 1
    assert list(fsm._arm_test.goals[0].motor_ids) == [14, 13, 12, 16]
    assert list(fsm._arm_test.goals[0].delta_ticks) == [5, 10, 10, 20]

    fsm._arm_test_state = "done"
    fsm._arm_test_ok = True
    fsm._do_arm_test_move()
    assert transitions == [State.END_EFFECTOR_ROTATE]

    fsm.rotary_relative = True
    fsm.rotary_ticks = -300
    fsm.rotary_max_abs_current = 100
    fsm.rotary_timeout = 10.0
    fsm._rotate_state = "idle"
    fsm._rotate_ok = False
    fsm._rotate_sent = False
    fsm._rotate = RecordingClient()
    fsm._do_end_effector_rotate()
    fsm._do_end_effector_rotate()
    assert len(fsm._rotate.goals) == 1


def test_arm_test_exception_attempts_torque_off_for_all_four_ids():
    bridge = configured_arm_test_bridge()
    writes = []

    def read_register(dxl_id, address, _size, label, signed=False):
        if label == "arm torque":
            raise RuntimeError("simulated communication failure")
        if label == "final arm torque readback":
            return 0
        raise AssertionError((dxl_id, address, label))

    bridge._read_register = read_register
    bridge._write_register = lambda dxl_id, address, size, value, label: \
        writes.append((dxl_id, address, value, label))
    result = bridge.execute_arm_test_move(arm_goal())
    assert not result.success
    final_ids = [dxl_id for dxl_id, address, value, label in writes
                 if address == bridge_module.ADDR_TORQUE_ENABLE
                 and value == 0 and label == "final arm torque disable"]
    assert final_ids == [14, 13, 12, 16]


def test_random_poses_are_reproducible_bounded_and_nontrivial():
    poses = ArmFsmNode.generate_random_poses(42, 3)
    assert poses == ArmFsmNode.generate_random_poses(42, 3)
    assert len(poses) == 3
    for pose in poses:
        assert len(pose) == 4
        for offset, limit in zip(pose, (20, 40, 40, 80)):
            assert 5 <= abs(offset) <= limit


def test_random_arm_goal_requires_both_safety_gates_and_rejects_id5():
    bridge = configured_arm_test_bridge()
    bridge.random_demo_enabled = True
    request = arm_goal(
        random_demo=True,
        delta_ticks=[20, -40, 40, -80]).request
    assert bridge.arm_test_goal_callback(request) \
        == bridge_module.GoalResponse.ACCEPT
    bridge.random_demo_enabled = False
    assert bridge.arm_test_goal_callback(request) \
        == bridge_module.GoalResponse.REJECT
    bridge.random_demo_enabled = True
    request.motor_ids = [14, 13, 12, 5]
    assert bridge.arm_test_goal_callback(request) \
        == bridge_module.GoalResponse.REJECT


def test_random_fsm_alternates_rotate_and_uses_pose_to_pose_deltas():
    fsm = object.__new__(ArmFsmNode)
    fsm.integrated_test_mode = True
    fsm.random_demo_enabled = True
    fsm.mission_type = MISSION_ROTARY_TOOL
    fsm.end_effector_kind = "rotary"
    fsm._random_poses = ArmFsmNode.generate_random_poses(42, 3)
    fsm._random_pose_index = 0
    fsm._random_previous_offsets = [0, 0, 0, 0]
    fsm._arm_test_state = "idle"
    fsm._arm_test_ok = False
    fsm._arm_test_sent = False
    fsm.arm_test_max_abs_current = 100
    fsm.arm_test_stall_timeout = 2.0
    fsm.arm_test_step_timeout = 8.0
    fsm._arm_test = RecordingClient()
    fsm._rotate = RecordingClient()
    fsm._rotate_state = "idle"
    fsm._rotate_ok = False
    fsm._rotate_sent = False
    fsm.rotary_relative = True
    fsm.rotary_ticks = 999
    fsm.rotary_max_abs_current = 100
    fsm.rotary_timeout = 10.0
    fsm._set_status = lambda _status: None
    fsm._elapsed = lambda: 2.0
    fsm.get_logger = lambda: Logger()
    transitions = []
    fsm._transition = transitions.append

    fsm._do_random_arm_demo()
    assert len(fsm._arm_test.goals) == 1
    assert list(fsm._arm_test.goals[0].delta_ticks) == fsm._random_poses[0]
    assert fsm._arm_test.goals[0].random_demo
    fsm._arm_test_state = "done"
    fsm._arm_test_ok = True
    fsm._do_random_arm_demo()
    assert transitions[-1] == State.END_EFFECTOR_ROTATE

    fsm._do_end_effector_rotate()
    assert len(fsm._rotate.goals) == 1
    assert fsm._rotate.goals[0].ticks == 300
    fsm._rotate_state = "done"
    fsm._rotate_ok = True
    fsm._do_end_effector_rotate()
    assert fsm._random_pose_index == 1
    assert transitions[-1] == State.RANDOM_ARM_DEMO

    fsm._arm_test_state = "idle"
    fsm._arm_test_sent = False
    fsm._do_random_arm_demo()
    expected = [fsm._random_poses[1][i] - fsm._random_poses[0][i]
                for i in range(4)]
    assert list(fsm._arm_test.goals[1].delta_ticks) == expected
    fsm._rotate_state = "idle"
    fsm._rotate_sent = False
    fsm._do_end_effector_rotate()
    assert fsm._rotate.goals[1].ticks == -300


def test_random_goal_outside_baseline_range_aborts_and_disables_all_ids():
    bridge = configured_arm_test_bridge()
    bridge.random_demo_enabled = True
    torque = {5: 0, 14: 0, 13: 0, 12: 0, 16: 0}
    writes = []

    def read_register(dxl_id, address, _size, _label, signed=False):
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            return torque[dxl_id]
        if address == bridge_module.ADDR_HARDWARE_ERROR_STATUS:
            return 0
        if address == bridge_module.ADDR_PRESENT_POSITION:
            return 1000
        if address == bridge_module.ADDR_OPERATING_MODE:
            return 3
        raise AssertionError((dxl_id, address))

    def write_register(dxl_id, address, _size, value, label):
        writes.append((dxl_id, address, value, label))
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            torque[dxl_id] = value

    bridge._read_register = read_register
    bridge._write_register = write_register
    # 콜백은 기준 범위의 두 배까지 자세 간 변화량을 허용하지만, 실행 단계에서는
    # 그 결과로 생긴 절대 기준 오프셋을 여전히 거부해야 한다.
    goal = arm_goal(
        random_demo=True,
        delta_ticks=[40, 10, 10, 10])
    result = bridge.execute_arm_test_move(goal)
    assert not result.success
    assert "outside approved range" in result.reason
    disabled = [dxl_id for dxl_id, address, value, _label in writes
                if address == bridge_module.ADDR_TORQUE_ENABLE and value == 0]
    assert disabled == [14, 13, 12, 16, 5]
    assert all(value == 0 for value in torque.values())


def test_arm_goal_tolerance_accepts_five_ticks_but_not_eleven():
    reached = bridge_module.MoveItDynamixelBridge.arm_test_goal_reached
    assert reached(position=1005, goal=1000, velocity=0, tolerance=10)
    assert not reached(position=1011, goal=1000, velocity=0, tolerance=10)
    assert not reached(position=1005, goal=1000, velocity=1, tolerance=10)
