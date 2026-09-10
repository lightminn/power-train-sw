"""Bounded, observation-only projection of the arm team's tool status."""
import math

TOOL_LABELS = {
    'spur_1motor_gripper': '단일 그리퍼',
    'dual_motor_gripper': '듀얼 그리퍼',
    'cleaner': '청소 모듈',
}


def normalize_tool(value):
    if not isinstance(value, dict) or value.get('tool_type') not in TOOL_LABELS:
        raise ValueError('invalid tool type')
    profile = value.get('tool_profile', {})
    ids = value.get('actuator_ids', profile.get('actuator_ids') if isinstance(profile, dict) else None)
    if (not isinstance(ids, list) or not 1 <= len(ids) <= 8
            or any(type(i) is not int or not 0 <= i <= 252 for i in ids)
            or len(set(ids)) != len(ids)):
        raise ValueError('invalid tool IDs')
    out = {'tool_type': value['tool_type'], 'actuator_ids': ids, 'actuators': []}
    for key in ('actuators_discovered', 'tool_detached', 'physical_tool_detached',
                'mock_mode', 'motion_allowed', 'tool_enable_allowed', 'read_only',
                'emergency_stop', 'calibration_jog_enabled'):
        item = value.get(key)
        if item is not None and type(item) is not bool:
            raise ValueError('invalid tool flag')
        out[key] = item
    for key in ('control_mode', 'fsm_state', 'reason'):
        item = value.get(key)
        if item is not None and (not isinstance(item, str) or len(item) > 160):
            raise ValueError('invalid tool status text')
        out[key] = item.strip() if item else None
    dual_calibration = value.get('dual_calibration')
    if isinstance(dual_calibration, dict):
        out['dual_calibration'] = {
            'active': bool(dual_calibration.get('active')),
            'validated': bool(dual_calibration.get('validated')),
            'candidate_valid': bool(dual_calibration.get('candidate_valid')),
            'state': str(dual_calibration.get('state') or ''),
            'captures': dict(dual_calibration.get('captures') or {}),
        }
    samples = value.get('actuators', [])
    if not isinstance(samples, list) or len(samples) > 8:
        raise ValueError('invalid actuators')
    seen = set()
    for sample in samples:
        if not isinstance(sample, dict) or type(sample.get('id')) is not int:
            raise ValueError('invalid actuator')
        if sample['id'] not in ids:
            continue
        if sample['id'] in seen:
            raise ValueError('duplicate actuator')
        seen.add(sample['id'])
        row = {'id': sample['id']}
        for key in ('position', 'effort', 'hardware_error', 'operating_mode', 'model',
                    'temperature_c'):
            item = sample.get(key)
            if item is not None and (type(item) not in (int, float) or not math.isfinite(item)):
                raise ValueError('invalid numeric feedback')
            row[key] = item
        online = sample.get('online')
        if online is not None and type(online) is not bool:
            raise ValueError('invalid online flag')
        row['online'] = online
        row['torque_state'] = sample.get('torque_state', 'UNKNOWN')
        if row['torque_state'] not in ('ON', 'OFF', 'UNKNOWN'):
            raise ValueError('invalid torque state')
        out['actuators'].append(row)
    return out


def parse_runtime(value):
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError('invalid tool runtime')
    age = value.get('source_age_s')
    if age is not None and (type(age) not in (int, float) or not math.isfinite(age) or age < 0):
        raise ValueError('invalid tool age')
    tool = value.get('tool')
    return {'source_age_s': age, 'tool': None if tool is None else normalize_tool(tool)}


def parse_arm_runtime(value):
    """Validate small, observation-only arm runtime state."""
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError('invalid arm runtime')
    result = {'source_age_s': {}}
    ages = value.get('source_age_s', {})
    if not isinstance(ages, dict):
        raise ValueError('invalid arm runtime ages')
    for key in ('control_mode', 'fsm_state', 'arm_status'):
        item = value.get(key)
        if item is not None and (not isinstance(item, str) or not item.strip() or len(item) > 80):
            raise ValueError('invalid arm runtime value')
        result[key] = item.strip() if item else None
        age = ages.get(key)
        if age is not None and (type(age) not in (int, float) or not math.isfinite(age) or age < 0):
            raise ValueError('invalid arm runtime age')
        result['source_age_s'][key] = age
    return result
