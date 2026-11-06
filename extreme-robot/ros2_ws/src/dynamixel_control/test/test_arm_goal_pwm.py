"""Arm Goal PWM parameter: only known joints and the servo's valid range are accepted."""

import ast
from pathlib import Path

SOURCE = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'


def parser():
    tree = ast.parse(SOURCE.read_text())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
              and n.name == '_parse_goal_pwms')
    config = next(n for n in tree.body if isinstance(n, ast.Assign)
                  and any(isinstance(t, ast.Name) and t.id == 'JOINT_CONFIG'
                          for t in n.targets))
    namespace = {'JOINT_CONFIG': ast.literal_eval(config.value)}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace['_parse_goal_pwms']


def test_valid_entry_is_kept_and_blank_default_is_ignored():
    assert parser()(['']) == ({}, [])
    assert parser()(['arm_joint_1:750']) == ({'arm_joint_1': 750}, [])


def test_bad_entries_are_reported_not_applied():
    values, errors = parser()(['arm_joint_1:0', 'arm_joint_1:886', 'arm_joint_9:500',
                               'arm_joint_2:abc'])
    assert values == {} and len(errors) == 4


def test_both_torque_startup_paths_write_the_limit():
    source = SOURCE.read_text()
    assert source.count('self._write_arm_goal_pwm(dxl_id)') == 2
