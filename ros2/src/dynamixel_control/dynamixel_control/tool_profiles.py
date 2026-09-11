"""Strict, data-driven profiles for interchangeable end effectors."""

from copy import deepcopy
from pathlib import Path

import yaml


SUPPORTED_TOOL_TYPES = (
    'spur_1motor_gripper', 'dual_motor_gripper', 'cleaner')
CONTROL_SCOPES = ('FULL_ROBOT', 'END_EFFECTOR_ONLY')


def validate_control_scope(value):
    scope = str(value).strip().upper()
    if scope not in CONTROL_SCOPES:
        raise ToolProfileError(
            f'unsupported control_scope {value!r}; expected '
            + ', '.join(CONTROL_SCOPES))
    return scope


class ToolProfileError(ValueError):
    """A profile is absent, malformed, or unsafe for real motion."""


def load_profiles(path):
    """Load the profile map from YAML without silently selecting a fallback."""
    profile_path = Path(path)
    if not profile_path.is_file():
        raise ToolProfileError(f'tool profile file not found: {profile_path}')
    with profile_path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    profiles = document.get('tool_profiles')
    if not isinstance(profiles, dict):
        raise ToolProfileError("YAML must contain a 'tool_profiles' mapping")
    return profiles


def get_profile(profiles, tool_type):
    """Return a copy so ROS parameter overrides cannot mutate shared data."""
    if tool_type not in SUPPORTED_TOOL_TYPES:
        raise ToolProfileError(
            f'unsupported tool_type {tool_type!r}; expected '
            + ', '.join(SUPPORTED_TOOL_TYPES))
    profile = profiles.get(tool_type)
    if not isinstance(profile, dict):
        raise ToolProfileError(f'missing profile for tool_type {tool_type!r}')
    return deepcopy(profile)


def validate_profile(tool_type, profile, mock_mode=False):
    """
    Return a list of reasons that prohibit actuator motion.

    Mock mode validates the dispatch contract but deliberately does not require
    unfinished physical actuator data.
    """
    errors = []
    expected_backend = 'cleaner' if tool_type == 'cleaner' else 'gripper'
    if profile.get('backend') != expected_backend:
        errors.append(f'backend must be {expected_backend!r}')
    if mock_mode:
        return errors
    if profile.get('mock_only'):
        errors.append('mock_only profile cannot control physical hardware')
    if not profile.get('calibrated', False):
        errors.append('calibrated must be true')
    ids = profile.get('actuator_ids')
    if not isinstance(ids, list) or not ids:
        errors.append('actuator_ids must be a non-empty list')
    elif len(ids) != len(set(ids)) or any(
            not isinstance(item, int) or not 0 <= item <= 252 for item in ids):
        errors.append('actuator_ids must contain unique IDs in [0,252]')
    if expected_backend == 'gripper':
        # The single-motor FSM uses endpoint motion only.  Contact/load
        # thresholds belong to a separate grasp calibration and must not make
        # an operator invent values just to save a witnessed endpoint pair.
        required = (
            'open_tick', 'close_tick', 'safe_min_tick', 'safe_max_tick',
            'direction')
        if tool_type != 'spur_1motor_gripper':
            required += (
                'profile_velocity', 'profile_acceleration', 'no_load_effort',
                'grasp_effort', 'grasp_threshold', 'release_drop_threshold',
                'action_time')
        for key in required:
            if profile.get(key) is None:
                errors.append(f'{key} is required')
        if not errors:
            low, high = profile['safe_min_tick'], profile['safe_max_tick']
            if low >= high:
                errors.append('safe_min_tick must be less than safe_max_tick')
            endpoints = profile.get('motor_endpoints') or {
                ids[0]: {'open': profile['open_tick'], 'close': profile['close_tick']}}
            for dxl_id in ids:
                endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
                if not endpoint:
                    errors.append(f'motor endpoint missing for actuator {dxl_id}')
                    continue
                if not (low <= endpoint['open'] <= high
                        and low <= endpoint['close'] <= high):
                    errors.append(f'actuator {dxl_id} endpoint outside safe range')
            if profile['direction'] not in (-1, 1):
                errors.append('direction must be -1 or 1')
            if (tool_type != 'spur_1motor_gripper'
                    and profile['profile_velocity'] <= 0):
                errors.append('profile_velocity must be positive')
            if (tool_type != 'spur_1motor_gripper'
                    and profile['profile_acceleration'] <= 0):
                errors.append('profile_acceleration must be positive')
            if (tool_type != 'spur_1motor_gripper' and not (
                    profile['no_load_effort'] < profile['grasp_threshold']
                    <= profile['grasp_effort'])):
                errors.append('grasp threshold must separate no-load and grasp')
            if (tool_type != 'spur_1motor_gripper' and not (
                    profile['no_load_effort'] <
                    profile['release_drop_threshold'] < profile['grasp_effort'])):
                errors.append('drop threshold must separate no-load and grasp')
    else:
        if profile.get('direction') not in (-1, 1):
            errors.append('direction must be -1 or 1')
        if not isinstance(profile.get('profile_velocity'), int) \
                or profile['profile_velocity'] <= 0:
            errors.append('profile_velocity must be a positive integer')
    return errors
