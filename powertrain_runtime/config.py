"""Configuration for the prepared, opt-in integrated deployment."""
import json
import math
from pathlib import Path


def load_config(path, role='operator'):
    if role not in ('operator', 'robot'):
        raise ValueError('unknown runtime role')
    with Path(path).expanduser().open() as stream:
        config = json.load(stream)
    if not isinstance(config, dict):
        raise ValueError('configuration must be an object')
    for key in ('robot_id', 'token_file'):
        if not isinstance(config.get(key), str) or not config[key].strip():
            raise ValueError(f'{key} is required')
    defaults = dict(session_port=9002)
    if role == 'operator':
        if not isinstance(config.get('hosts'), list) or not config['hosts'] or any(
                not isinstance(host, str) or not host.strip() for host in config['hosts']):
            raise ValueError('hosts must contain paired IPv4 host candidates')
        defaults.update(controller_python='python3', profile='can-4ws')
    else:
        defaults.update(host='0.0.0.0', input_port=9000, ops_port=9001,
                        input_target_port=19000, ops_target_port=19001,
                        destination_file='/run/powertrain/operator-session.json', lease_timeout_s=2.0)
    result = {**defaults, **config}
    for key in ('session_port', 'input_port', 'ops_port', 'input_target_port', 'ops_target_port'):
        if key in result and (type(result[key]) is not int or not 1 <= result[key] <= 65535):
            raise ValueError(f'invalid {key}')
    if 'lease_timeout_s' in result:
        duration = float(result['lease_timeout_s'])
        if not math.isfinite(duration) or not .2 <= duration <= 30:
            raise ValueError('lease_timeout_s must be between 0.2 and 30')
    result['token_file'] = str(Path(result['token_file']).expanduser())
    if 'destination_file' in result:
        result['destination_file'] = str(Path(result['destination_file']).expanduser())
    return result
