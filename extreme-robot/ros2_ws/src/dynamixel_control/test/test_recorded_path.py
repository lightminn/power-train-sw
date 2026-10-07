"""하드웨어 없이 수행하는 signed 기록 경로 검증 및 브리지 안전 테스트."""

import threading
from types import SimpleNamespace

import pytest

from dynamixel_control import moveit_dynamixel_bridge as bridge_module
from dynamixel_control.recorded_path_replay import (
    arm_result_allows_rotation, build_reverse_paths, flatten_paths,
)


class Logger:
    def warn(self, _message):
        pass


class GoalHandle:
    def __init__(self, request):
        self.request = request
        self.is_cancel_requested = False
        self.feedback = []
        self.state = None

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self):
        self.state = "succeeded"

    def abort(self):
        self.state = "aborted"

    def canceled(self):
        self.state = "canceled"


def request(**overrides):
    values = {
        "motor_ids": [14, 13, 12],
        "waypoint_counts": [2, 2, 2],
        "signed_waypoints": [1000, 950, 200, 250, 3500, 3450],
        "max_abs_current": 300,
        "stall_timeout": 2.0,
        "step_timeout": 10.0,
        "goal_tolerance": 10,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def configured_bridge():
    bridge = object.__new__(bridge_module.MoveItDynamixelBridge)
    bridge.integrated_test_mode = True
    bridge.read_only = False
    bridge.gripper_only_mode = False
    bridge.end_effector_kind = "rotary"
    bridge.gripper_ids = [5]
    bridge._bus_lock = threading.Lock()
    bridge.torque_enabled_ids = set()
    bridge.get_logger = lambda: Logger()
    return bridge


def test_request_requires_exact_ids_counts_direction_and_limits():
    split = bridge_module.MoveItDynamixelBridge.split_recorded_path_request
    assert [item[0] for item in split(request())] == [14, 13, 12]
    with pytest.raises(ValueError, match="exactly"):
        split(request(motor_ids=[14, 13, 12, 16]))
    with pytest.raises(ValueError, match="match"):
        split(request(waypoint_counts=[2, 2, 3]))
    with pytest.raises(ValueError, match=r"\[1, 50\]"):
        split(request(signed_waypoints=[1000, 949, 200, 250, 3500, 3450]))
    with pytest.raises(ValueError, match="reverses"):
        split(request(
            waypoint_counts=[3, 2, 2],
            signed_waypoints=[1000, 950, 975, 200, 250, 3500, 3450]))
    with pytest.raises(ValueError, match="Mode 3"):
        split(request(signed_waypoints=[1000, 950, 200, 250, 4090, 4100]))


def test_fold_builder_truncates_id13_rebound_and_preserves_signed_mode4():
    samples = [
        {"id14": -100, "id13": 200, "id12": 3000},
        {"id14": -50, "id13": 150, "id12": 3025},
        {"id14": 0, "id13": 100, "id12": 3050},
        {"id14": 0, "id13": 121, "id12": 3050},
    ]
    paths, metadata = build_reverse_paths({"samples": samples, "error": None})
    assert paths[14][0] == 0 and paths[14][-1] == -100
    assert paths[13][0] == 100 and paths[13][-1] == 200
    assert paths[12][0] == 3050 and paths[12][-1] == 3000
    assert metadata["id13_minimum"] == 100
    assert metadata["id13_excluded_rebound"] == 21
    assert all(abs(b - a) <= 50 for path in paths.values()
               for a, b in zip(path, path[1:]))
    counts, flat = flatten_paths(paths)
    assert sum(counts) == len(flat)


def test_recorded_path_writes_only_14_13_12_and_finishes_torque_off():
    bridge = configured_bridge()
    positions = {14: 1000, 13: 200, 12: 3500, 16: 800, 5: 1800}
    goals = dict(positions)
    torque = {dxl_id: 0 for dxl_id in positions}
    writes = []
    maximum_enabled = [0]

    def read_register(dxl_id, address, _size, _label, signed=False):
        if address == bridge_module.ADDR_OPERATING_MODE:
            return 4 if dxl_id in (14, 13) else 3
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
        assert dxl_id not in (16, 5)
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            torque[dxl_id] = value
            maximum_enabled[0] = max(
                maximum_enabled[0], sum(torque.values()))
            if value == 0:
                positions[dxl_id] = goals[dxl_id]
        elif address == bridge_module.ADDR_GOAL_POSITION:
            goals[dxl_id] = value if value < 2**31 else value - 2**32

    bridge._read_register = read_register
    bridge._write_register = write_register
    goal = GoalHandle(request())
    result = bridge.execute_arm_recorded_path(goal)
    assert result.success
    assert goal.state == "succeeded"
    assert result.completed_waypoints == 6
    assert maximum_enabled[0] == 1
    assert all(value == 0 for value in torque.values())
    assert {item[0] for item in writes} == {14, 13, 12}
    for dxl_id in (14, 13, 12):
        torque_values = [value for item_id, address, value, _label in writes
                         if item_id == dxl_id
                         and address == bridge_module.ADDR_TORQUE_ENABLE]
        assert torque_values == [1, 0]
        off_indices = [index for index, item in enumerate(writes)
                       if item[0] == dxl_id
                       and item[1] == bridge_module.ADDR_TORQUE_ENABLE
                       and item[2] == 0]
        goal_indices = [index for index, item in enumerate(writes)
                        if item[0] == dxl_id
                        and item[1] == bridge_module.ADDR_GOAL_POSITION]
        assert off_indices[0] > max(goal_indices)


def test_recorded_path_start_failure_writes_no_id16_or_id5_and_turns_arm_off():
    bridge = configured_bridge()
    torque = {14: 0, 13: 0, 12: 0, 16: 0, 5: 0}
    positions = {14: 1100, 13: 200, 12: 3500, 16: 800, 5: 1800}
    writes = []

    def read_register(dxl_id, address, _size, _label, signed=False):
        if address == bridge_module.ADDR_OPERATING_MODE:
            return 4 if dxl_id in (14, 13) else 3
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            return torque[dxl_id]
        if address == bridge_module.ADDR_HARDWARE_ERROR_STATUS:
            return 0
        if address == bridge_module.ADDR_PRESENT_POSITION:
            return positions[dxl_id]
        raise AssertionError(address)

    def write_register(dxl_id, address, _size, value, label):
        writes.append((dxl_id, address, value, label))
        assert dxl_id not in (16, 5)
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            torque[dxl_id] = value

    bridge._read_register = read_register
    bridge._write_register = write_register
    goal = GoalHandle(request())
    result = bridge.execute_arm_recorded_path(goal)
    assert not result.success
    assert "start error" in result.reason
    assert goal.state == "aborted"
    assert all(value == 0 for value in torque.values())
    assert writes == []


def test_direction_guard_allows_short_rebound_and_rejects_sustained_motion():
    check = bridge_module.MoveItDynamixelBridge.recorded_direction_violation
    state = {"samples": 0, "ticks": 0}
    assert not check(-3, 1, state)
    assert not check(4, 1, state)
    assert state == {"samples": 0, "ticks": 0}

    state = {"samples": 0, "ticks": 0}
    assert not check(-3, 1, state)
    assert not check(-3, 1, state)
    assert check(-3, 1, state)

    state = {"samples": 0, "ticks": 0}
    assert not check(-6, 1, state)
    assert check(-5, 1, state)


def test_runtime_failure_turns_current_axis_off_without_writing_id16_or_id5():
    bridge = configured_bridge()
    torque = {14: 0, 13: 0, 12: 0, 16: 0, 5: 0}
    positions = {14: 1000, 13: 200, 12: 3500, 16: 800, 5: 1800}
    goals = dict(positions)
    writes = []

    def read_register(dxl_id, address, _size, _label, signed=False):
        if address == bridge_module.ADDR_OPERATING_MODE:
            return 4 if dxl_id in (14, 13) else 3
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            return torque[dxl_id]
        if address == bridge_module.ADDR_HARDWARE_ERROR_STATUS:
            return 0
        if address == bridge_module.ADDR_PRESENT_POSITION:
            return positions[dxl_id]
        if address == bridge_module.ADDR_GOAL_POSITION:
            return goals[dxl_id]
        if address == bridge_module.ADDR_PRESENT_LOAD:
            return 300
        if address in (bridge_module.ADDR_PRESENT_VELOCITY,
                       bridge_module.ADDR_MOVING_STATUS):
            return 0
        raise AssertionError(address)

    def write_register(dxl_id, address, _size, value, label):
        writes.append((dxl_id, address, value, label))
        assert dxl_id not in (16, 5)
        if address == bridge_module.ADDR_TORQUE_ENABLE:
            torque[dxl_id] = value
        elif address == bridge_module.ADDR_GOAL_POSITION:
            goals[dxl_id] = value

    bridge._read_register = read_register
    bridge._write_register = write_register
    goal = GoalHandle(request())
    result = bridge.execute_arm_recorded_path(goal)
    assert not result.success
    assert "current" in result.reason
    assert torque[14] == 0
    assert all(item[0] not in (16, 5) for item in writes)
    torque_values = [value for dxl_id, address, value, _label in writes
                     if dxl_id == 14
                     and address == bridge_module.ADDR_TORQUE_ENABLE]
    assert torque_values == [1, 0]


def test_arm_failure_does_not_allow_end_effector_rotation():
    assert not arm_result_allows_rotation(SimpleNamespace(success=False))
    assert arm_result_allows_rotation(SimpleNamespace(success=True))
