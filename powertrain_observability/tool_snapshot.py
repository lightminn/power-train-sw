"""Bounded, observation-only projection of the arm team's tool status."""
import math
import os

TOOL_LABELS = {
    'spur_1motor_gripper': '단일 그리퍼',
    'dual_motor_gripper': '듀얼 그리퍼',
    'cleaner': '청소 모듈',
}


def _exchange_status(value):
    """Bench slot exchange progress, bounded like every other forwarded field."""
    out = {}
    for key in ('state', 'slot', 'error'):
        item = value.get(key)
        if item is not None and not isinstance(item, str):
            raise ValueError('invalid exchange status text')
        out[key] = item[:160] if item is not None else None
    for key in ('step', 'steps'):
        item = value.get(key, 0)
        if type(item) is not int or not 0 <= item <= 64:
            raise ValueError('invalid exchange step')
        out[key] = item
    slots = value.get('slots') or {}
    if (not isinstance(slots, dict) or len(slots) > 8
            or any(not isinstance(k, str) or v not in TOOL_LABELS for k, v in slots.items())):
        raise ValueError('invalid exchange slots')
    out['slots'] = dict(slots)
    incomplete = value.get('incomplete') or []
    if (not isinstance(incomplete, list) or len(incomplete) > 8
            or any(key not in slots for key in incomplete)):
        raise ValueError('invalid incomplete slots')
    out['incomplete'] = list(incomplete)
    attached = value.get('attached_tool')
    if attached is not None and attached not in TOOL_LABELS:
        raise ValueError('invalid attached tool')
    out['attached_tool'] = attached
    return out


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
    speed = value.get('local_jog_speed_level')
    if speed is not None:
        if type(speed) is not int or not 0 <= speed <= 10:
            raise ValueError('invalid local jog speed')
        out['local_jog_speed_level'] = speed
    generation = value.get('tool_context_generation')
    if generation is not None:
        if type(generation) is not int or generation < 0:
            raise ValueError('invalid tool generation')
        out['tool_context_generation'] = generation
    capabilities = value.get('manual_tool_capabilities', [])
    if (not isinstance(capabilities, list)
            or any(c not in ('tool_jog', 'dual_sync', 'individual_targets') for c in capabilities)):
        raise ValueError('invalid manual tool capabilities')
    out['manual_tool_capabilities'] = capabilities
    synchronization = value.get('synchronization')
    if synchronization is not None:
        if (not isinstance(synchronization, dict) or synchronization.get('state') not in (
                'SYNCHRONIZED', 'FAULT', 'UNKNOWN', 'NOT_APPLICABLE')):
            raise ValueError('invalid synchronization state')
        out['synchronization'] = {'state': synchronization['state']}
    for key in ('actuators_discovered', 'tool_detached', 'physical_tool_detached',
                'mock_mode', 'motion_allowed', 'tool_enable_allowed', 'read_only',
                'emergency_stop', 'calibration_jog_enabled', 'manual_coupling_lock'):
        item = value.get(key)
        if item is not None and type(item) is not bool:
            raise ValueError('invalid tool flag')
        out[key] = item
    for key in ('control_mode', 'fsm_state', 'reason', 'control_scope'):
        item = value.get(key)
        if item is not None and (not isinstance(item, str) or len(item) > 160):
            raise ValueError('invalid tool status text')
        out[key] = item.strip() if item else None
    poses = value.get('poses')
    if poses is not None:
        if (not isinstance(poses, list) or len(poses) > 64
                or any(not isinstance(p, str) or not 1 <= len(p) <= 64 for p in poses)):
            raise ValueError('invalid pose list')
        out['poses'] = poses
    exchange = value.get('local_tool_change')
    if isinstance(exchange, dict):
        out['local_tool_change'] = _exchange_status(exchange)
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
    result = {'source_age_s': age, 'tool': None if tool is None else normalize_tool(tool)}
    if 'coupling_lock' in value:
        result['coupling_lock'] = normalize_lock(value['coupling_lock'])
        lock_age = value.get('coupling_lock_age_s')
        if lock_age is not None and (type(lock_age) not in (int, float)
                or not math.isfinite(lock_age) or lock_age < 0):
            raise ValueError('invalid lock age')
        result['coupling_lock_age_s'] = lock_age
    return result


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


def validate_lock_command(value):
    """Only the existing manual lock vocabulary may cross the ops channel."""
    if not isinstance(value, dict):
        raise ValueError('invalid lock command')
    command = value.get('command')
    if not isinstance(command, str):
        raise ValueError('invalid lock command')
    keys = {'STOP': {'command'}, 'ENABLE': {'command'},
            'SET_ZERO': {'command', 'state'},
            'CHANGE': {'command', 'to', 'direction'}}.get(command)
    if keys is None or set(value) != keys:
        raise ValueError('invalid lock command fields')
    if command == 'SET_ZERO' and value['state'] not in ('LOCKED', 'UNLOCKED'):
        raise ValueError('invalid lock zero state')
    if command == 'CHANGE' and (
            value['to'] not in ('LOCKED', 'UNLOCKED')
            or type(value['direction']) is not int or value['direction'] not in (-1, 1)):
        raise ValueError('invalid lock transition')
    return dict(value)


def normalize_lock(value):
    """Bound status independently from the tool, which may be detached."""
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError('invalid lock status')
    result = {}
    for key in ('enabled', 'mechanically_confirmed', 'two_point'):
        item = value.get(key)
        if item is not None and type(item) is not bool:
            raise ValueError('invalid lock flag')
        result[key] = item
    for key in ('zero_tick', 'position', 'torque', 'coupling_peak_load',
                'locked_tick', 'unlocked_tick', 'state_tolerance'):
        item = value.get(key)
        if item is not None and (type(item) not in (int, float) or not math.isfinite(item)):
            raise ValueError('invalid lock feedback')
        result[key] = item
    for key in ('state', 'zero_state', 'sequence_state', 'coupling_proxy',
                'error', 'sequence_error', 'command_error'):
        item = value.get(key)
        if item is not None and (not isinstance(item, str) or len(item) > 160):
            raise ValueError('invalid lock status text')
        result[key] = item
    return result


def validate_local_pose_request(action, params):
    """Bench-only pose actions: a short name (except home) and optional switches."""
    if os.environ.get('ARM_LOCAL_ALL_AXES') != '1':
        raise ValueError('local pose actions are bench-only')
    named = action not in ('robot_arm_home', 'robot_arm_freedrive',
                           'robot_arm_freedrive_release')
    allowed = ({'name'} if named else set()) | (
        {'limits', 'trip'} if action in ('robot_arm_pose_move', 'robot_arm_home') else set())
    if not params.keys() <= allowed or (named and 'name' not in params):
        raise ValueError('invalid local pose parameters')
    if named and (not isinstance(params['name'], str) or not 1 <= len(params['name'].strip()) <= 64):
        raise ValueError('invalid pose name')
    for flag in ('limits', 'trip'):
        if flag in params and type(params[flag]) is not bool:
            raise ValueError(f'invalid {flag} flag')


EXCHANGE_TOOL_TYPES = ('spur_1motor_gripper', 'dual_motor_gripper', 'cleaner')


def local_tool_change_request(action, params):
    """Bench slot teach/exchange actions -> the bridge's /arm/local_tool_change JSON."""
    if os.environ.get('ARM_LOCAL_ALL_AXES') != '1':
        raise ValueError('local tool change actions are bench-only')
    flags = {'dry_run', 'limits', 'trip'}
    allowed = {'robot_arm_slot_capture': {'slot', 'tool_type', 'pose', 'limits'},
               'robot_arm_exchange_start': {'tool_type', 'dry_run', 'limits', 'trip'},
               'robot_arm_exchange_stop': set()}[action]
    if not allowed - flags <= set(params) <= allowed:
        raise ValueError('invalid local tool change parameters')
    for flag in flags & set(params):
        if type(params[flag]) is not bool:
            raise ValueError(f'invalid {flag} flag')
    if 'tool_type' in params and params['tool_type'] not in EXCHANGE_TOOL_TYPES:
        raise ValueError('unknown tool type')
    if 'slot' in params and params['slot'] not in ('1', '2', '3'):
        raise ValueError('unknown slot')
    if 'pose' in params and params['pose'] not in ('seat', 'above'):
        raise ValueError('unknown slot pose')
    command = {'robot_arm_slot_capture': 'CAPTURE', 'robot_arm_exchange_start': 'START',
               'robot_arm_exchange_stop': 'STOP'}[action]
    return dict(params, command=command)


def tool_manual_request(action, params, sent_at_s):
    """Translate authenticated GUI intent without accepting raw actuator IDs."""
    types = {'spur_1motor_gripper': ('5', ('single',)),
             'dual_motor_gripper': ('3,4', ('left', 'right', 'both')),
             'cleaner': ('2', ('single',))}
    identity = params.get('tool_id')
    if not isinstance(identity, str) or ':' not in identity:
        raise ValueError('missing active tool identity')
    kind, ids = identity.split(':', 1)
    if kind not in types or ids != types[kind][0] or params.get('target') not in types[kind][1]:
        raise ValueError('invalid active tool target')
    generation = params.get('tool_generation')
    if type(generation) is not int or generation < 0:
        raise ValueError('invalid tool generation')
    payload = dict(tool_type=kind, target=params['target'],
                   tool_context_generation=generation, sent_at_s=sent_at_s)
    if action == 'robot_arm_tool_command':
        allowed = {'target', 'command', 'tool_id', 'tool_generation'}
        command = params.get('command')
        if command not in (('left', 'right', 'stop') if kind == 'cleaner' else ('open', 'close', 'stop')):
            raise ValueError('invalid tool command')
        payload['command'] = command.upper()
    elif action in ('robot_arm_tool_jog', 'robot_arm_tool_jog_stop'):
        allowed = {'target', 'tool_id', 'tool_generation', 'sequence'}
        sequence = params.get('sequence')
        if type(sequence) is not int or sequence < 0:
            raise ValueError('invalid tool sequence')
        payload['sequence'] = sequence
        if action.endswith('_stop'):
            allowed.add('reason')
            payload['command'] = 'HOLD'
        else:
            allowed.update(('direction', 'ttl_s', 'expires_at_s', 'speed_level'))
            direction, ttl = params.get('direction'), params.get('ttl_s')
            if (type(direction) is not int or direction not in (-1, 1)
                    or type(ttl) not in (int, float) or not math.isfinite(ttl) or not 0 < ttl <= 0.6):
                raise ValueError('invalid tool lease')
            payload.update(command='JOG', direction=direction, ttl_s=ttl)
    else:
        raise ValueError('unsupported tool action')
    if params.keys() - allowed:
        raise ValueError('unexpected tool parameters')
    return payload


def validate_local_wrist_request(action, params):
    """The local bench accepts only axis 5, with bounded jog lifetime metadata."""
    stop = action == 'robot_arm_jog_stop'
    required = ({'axis', 'sequence', 'reason'} if stop else
                {'axis', 'sequence', 'direction', 'expires_at_s'})
    allowed = required if stop else required | {'speed_level', 'limits', 'trip'}
    if not required <= params.keys() or not params.keys() <= allowed:
        raise ValueError('invalid local wrist parameters')
    axes = (1, 2, 3, 4, 5) if os.environ.get('ARM_LOCAL_ALL_AXES') == '1' else (5,)
    if type(params['axis']) is not int or params['axis'] not in axes:
        raise ValueError(f'local bench supports only axes {axes}')
    for flag in ('limits', 'trip'):
        if flag in params and type(params[flag]) is not bool:
            raise ValueError(f'invalid {flag} flag')
    if type(params['sequence']) is not int or params['sequence'] < 0:
        raise ValueError('invalid wrist sequence')
    if stop:
        if not isinstance(params['reason'], str) or not 1 <= len(params['reason']) <= 64:
            raise ValueError('invalid wrist stop reason')
    else:
        if type(params['direction']) is not int or params['direction'] not in (-1, 1):
            raise ValueError('invalid wrist direction')
        if type(params['expires_at_s']) not in (int, float) or not math.isfinite(params['expires_at_s']):
            raise ValueError('invalid wrist deadline')
        if 'speed_level' in params and (type(params['speed_level']) is not int
                                       or not 0 <= params['speed_level'] <= 10):
            raise ValueError('invalid wrist speed level')
