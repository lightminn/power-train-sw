"""Safety and selection tests for interchangeable tool profiles."""

import pytest

from dynamixel_control.spur_gripper_calibration import build_profile
from dynamixel_control.tool_manager import (
    ParameterToolIdentityProvider, ToolManager)
from dynamixel_control.tool_profiles import (
    ToolProfileError, get_profile, load_profiles, validate_control_scope,
    validate_profile)


def profile_file(tmp_path):
    path = tmp_path / 'profiles.yaml'
    path.write_text('''tool_profiles:
  spur_1motor_gripper: {backend: gripper, calibrated: false}
  dual_motor_gripper: {backend: gripper, calibrated: false}
  cleaner: {backend: cleaner, calibrated: false}
''', encoding='utf-8')
    return path


def test_unknown_tool_never_falls_back(tmp_path):
    profiles = load_profiles(profile_file(tmp_path))
    with pytest.raises(ToolProfileError):
        get_profile(profiles, 'typo_gripper')


def test_control_scope_defaults_are_explicit_and_unknown_scope_fails_closed():
    assert validate_control_scope('FULL_ROBOT') == 'FULL_ROBOT'
    assert validate_control_scope('end_effector_only') == 'END_EFFECTOR_ONLY'
    with pytest.raises(ToolProfileError, match='unsupported control_scope'):
        validate_control_scope('tool_only_typo')


def test_uncalibrated_real_profile_is_fail_closed(tmp_path):
    profiles = load_profiles(profile_file(tmp_path))
    selection = ToolManager(
        profiles, ParameterToolIdentityProvider('spur_1motor_gripper')) \
        .refresh('IDLE')
    assert not selection.valid
    assert 'calibrated must be true' in selection.reason


def test_mock_validates_dispatch_without_hardware_values(tmp_path):
    profiles = load_profiles(profile_file(tmp_path))
    selection = ToolManager(
        profiles, ParameterToolIdentityProvider('cleaner'), mock_mode=True) \
        .refresh('IDLE')
    assert selection.valid


def test_tool_change_is_allowed_only_while_idle(tmp_path):
    profiles = load_profiles(profile_file(tmp_path))
    provider = ParameterToolIdentityProvider('cleaner')
    manager = ToolManager(profiles, provider, mock_mode=True)
    manager.refresh('IDLE')
    provider._tool_type = 'dual_motor_gripper'
    with pytest.raises(ToolProfileError, match='denied'):
        manager.refresh('GRASP')
    assert manager.refresh('STOWED_LOCKED').tool_type == 'dual_motor_gripper'


def test_spur_calibration_computes_direction_thresholds_and_safe_range():
    profile = build_profile(
        actuator_id=5, open_tick=100, close_tick=600, safe_margin=10,
        profile_velocity=20, profile_acceleration=5,
        samples={
            'no_load': [30, 32, 34],
            'grasp': [200, 220, 240],
            'release': [35, 40, 45],
        })
    assert profile['direction'] == 1
    assert (profile['safe_min_tick'], profile['safe_max_tick']) == (100, 600)
    assert (profile['open_tick'], profile['close_tick']) == (110, 590)
    assert profile['no_load_effort'] < profile['grasp_threshold'] \
        <= profile['grasp_effort']
    assert not validate_profile('spur_1motor_gripper', profile)
