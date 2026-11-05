"""GUI range calibration: measure -> verify -> apply -> save, and the bridge entry."""

import ast
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from dynamixel_control import arm_range_calibration as R
from dynamixel_control import joint_limits

RAD = math.radians
SOURCE = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'


def swept(cal, axis, low_deg, high_deg):
    for deg in (0.0, low_deg, high_deg, 0.0):
        cal.observe([axis], [RAD(deg)])


def test_measure_records_running_min_max_only_while_recording(tmp_path):
    cal = R.ArmRangeCalibration(str(tmp_path / 'limits.json'))
    cal.observe(['arm_joint_2'], [RAD(-50)])        # before start: ignored
    cal.measure()
    assert cal.status == R.RUNNING and cal.recording
    swept(cal, 'arm_joint_2', -10, 95)
    cal.observe(['arm_joint_3'], [RAD(1)])          # barely moved: not swept
    cal.observe(['arm_joint_5'], [RAD(80)])         # joint 5 is not calibrated here
    cal.measure()
    assert cal.status == R.DONE and not cal.recording
    assert set(cal.measured) == {'arm_joint_2'}
    lo, hi = cal.measured['arm_joint_2']
    assert math.degrees(lo) == pytest.approx(-10) and math.degrees(hi) == pytest.approx(95)
    assert '기존 범위를 유지' in cal.detail


def test_measure_with_nothing_swept_fails(tmp_path):
    cal = R.ArmRangeCalibration(str(tmp_path / 'limits.json'))
    cal.measure()
    cal.observe(['arm_joint_1'], [0.01])
    cal.measure()
    assert cal.status == R.FAILED and not cal.measured


def test_verify_takes_margin_and_rejects_a_saved_pose_outside(tmp_path):
    cal = R.ArmRangeCalibration(str(tmp_path / 'limits.json'))
    cal.measure()
    swept(cal, 'arm_joint_4', -60, 80)
    cal.measure()
    cal.verify({'home': {'arm_joint_4': RAD(79)}})   # inside the stop, outside the margin
    assert cal.status == R.FAILED and not cal.verified
    assert 'home' in cal.detail
    cal.verify({'home': {'arm_joint_4': RAD(10)}})
    assert cal.status == R.DONE
    lo, hi = cal.verified['arm_joint_4']
    assert math.degrees(lo) == pytest.approx(-58) and math.degrees(hi) == pytest.approx(78)


def test_verify_and_save_need_their_previous_step(tmp_path):
    cal = R.ArmRangeCalibration(str(tmp_path / 'limits.json'))
    with pytest.raises(ValueError):
        cal.verify({})
    with pytest.raises(ValueError):
        cal.apply_to({})
    with pytest.raises(ValueError):
        cal.save()


def test_apply_changes_the_live_table_and_save_reloads_at_startup(tmp_path):
    path = tmp_path / 'limits.json'
    cal = R.ArmRangeCalibration(str(path))
    cal.measure()
    swept(cal, 'arm_joint_1', -16, 16)
    cal.measure()
    cal.verify({})
    table = copy.deepcopy(joint_limits.JOINT_LIMITS)
    cal.apply_to(table)
    assert table['arm_joint_1']['lower'] == pytest.approx(RAD(-14))
    assert table['arm_joint_1']['confidence'] == 'measured'
    assert table['arm_joint_2'] == joint_limits.JOINT_LIMITS['arm_joint_2']
    cal.save()
    assert cal.saved
    fresh = copy.deepcopy(joint_limits.JOINT_LIMITS)
    assert R.apply_saved(str(path), fresh) == ['arm_joint_1']
    assert fresh['arm_joint_1']['upper'] == pytest.approx(RAD(14))


def test_save_keeps_other_axes_already_in_the_file(tmp_path):
    path = tmp_path / 'limits.json'
    path.write_text(json.dumps({'arm_joint_3': {'lower': -0.1, 'upper': 2.8}}))
    cal = R.ArmRangeCalibration(str(path))
    cal.measure()
    swept(cal, 'arm_joint_1', -16, 16)
    cal.measure()
    cal.verify({})
    cal.save()
    assert set(json.loads(path.read_text())) == {'arm_joint_1', 'arm_joint_3'}


def test_a_new_measurement_clears_previous_results(tmp_path):
    cal = R.ArmRangeCalibration(str(tmp_path / 'limits.json'))
    cal.measure()
    swept(cal, 'arm_joint_1', -16, 16)
    cal.measure()
    cal.verify({})
    cal.apply_to({})
    cal.measure()
    assert cal.recording and not cal.measured and not cal.verified
    assert not cal.applied and not cal.saved


def test_bad_saved_file_is_rejected(tmp_path):
    path = tmp_path / 'limits.json'
    path.write_text(json.dumps({'arm_joint_1': {'lower': 1.0, 'upper': -1.0}}))
    with pytest.raises(ValueError):
        R.apply_saved(str(path), {})


def bridge_command():
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'MoveItDynamixelBridge')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == '_arm_calibration_command')
    namespace = dict(json=json, joint_limits=SimpleNamespace(JOINT_LIMITS={}))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace['_arm_calibration_command'], namespace['joint_limits'].JOINT_LIMITS


def fake_bridge(tmp_path, mode='MANUAL'):
    logs = []
    logger = SimpleNamespace(info=logs.append, warn=logs.append)
    return SimpleNamespace(
        control_mode=mode, _local_poses={},
        _range_cal=R.ArmRangeCalibration(str(tmp_path / 'limits.json')),
        get_logger=lambda: logger), logs


def test_bridge_command_needs_manual_and_the_range_step(tmp_path):
    command, _ = bridge_command()
    node, _ = fake_bridge(tmp_path, mode='FSM')
    command(node, SimpleNamespace(data=json.dumps({'step': 'range', 'action': 'measure'})))
    assert not node._range_cal.recording and 'MANUAL' in node._range_cal.detail
    node.control_mode = 'MANUAL'
    command(node, SimpleNamespace(data=json.dumps({'step': 'zero', 'action': 'measure'})))
    assert not node._range_cal.recording and '거부' in node._range_cal.detail
    command(node, SimpleNamespace(data=json.dumps({'step': 'range', 'action': 'measure'})))
    assert node._range_cal.recording


def test_bridge_command_reports_a_refused_step_instead_of_raising(tmp_path):
    command, _ = bridge_command()
    node, logs = fake_bridge(tmp_path)
    command(node, SimpleNamespace(data=json.dumps({'step': 'range', 'action': 'save'})))
    assert node._range_cal.detail.startswith('거부') and logs
