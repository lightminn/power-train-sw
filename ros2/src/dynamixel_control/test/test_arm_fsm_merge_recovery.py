"""팔 실기 FSM 흐름을 무효화했던 병합 손상의 회귀 테스트."""

import ast
from pathlib import Path


SOURCE = Path(__file__).parents[1] / 'dynamixel_control/arm_fsm_node.py'


def _class_methods():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    cls = next(node for node in tree.body
               if isinstance(node, ast.ClassDef) and node.name == 'ArmFsmNode')
    return [node for node in cls.body if isinstance(node, ast.FunctionDef)]


def _method(name):
    methods = [node for node in _class_methods() if node.name == name]
    assert len(methods) == 1, f'{name} must have exactly one definition'
    return methods[0]


def _calls(method):
    return {
        node.func.attr
        for node in ast.walk(method)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }


def test_pick_plan_has_one_definition_and_keeps_safety_and_frozen_target_flow():
    plan = _method('_do_plan')
    calls = _calls(plan)
    assert {'_tool_ready', '_has_frozen_target', '_grasp_pose_in_base',
            '_offset_pose_z', '_grasp_target_xyz'} <= calls
    # PLAN은 목표만 얼리고, 실제 이동은 APPROACH가 담당한다.
    assert '_begin_arm_move' not in calls
    assert '_move_to_xyz' not in calls


def test_approach_and_descend_keep_two_stage_august_19_motion():
    approach_calls = _calls(_method('_do_approach'))
    descend_calls = _calls(_method('_do_descend'))
    assert {'_begin_arm_move', '_move_to_xyz'} <= approach_calls
    assert {'_begin_arm_move', '_move_to_xyz', '_is_settled'} <= descend_calls


def test_tool_action_dispatch_and_grasp_command_are_reachable():
    dispatch = ast.unparse(_method('_do_tool_action'))
    grasp_calls = _calls(_method('_do_grasp'))
    assert "task_command == 'PICK'" in dispatch
    assert "backend == 'gripper'" in dispatch
    assert "task_command == 'CLEAN'" in dispatch
    assert "backend == 'cleaner'" in dispatch
    assert '_send_gripper' in grasp_calls
    assert '_tool_ready' in grasp_calls


def test_frozen_target_storage_is_initialized_and_reset_for_new_commands():
    init = ast.unparse(_method('__init__'))
    command = ast.unparse(_method('_on_task_command'))
    for field in ('_planned_grasp_pose', '_planned_approach_pose',
                  '_planned_grasp_xyz', '_planned_approach_xyz'):
        assert f'self.{field} = None' in init
    assert 'self._clear_frozen_target()' in command
