"""Persistent ID3-only gripper endpoint calibration."""

import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def default_path():
    # In a symlink-install workspace, write the version-controlled source file
    # rather than replacing the installed symlink with a regular file.
    source_candidate = (
        Path(__file__).resolve().parent.parent
        / 'config' / 'dual_id3_endpoints.json')
    if source_candidate.exists():
        return source_candidate
    return Path(get_package_share_directory(
        'dynamixel_control')) / 'config' / 'dual_id3_endpoints.json'


def load(path=None):
    path = Path(path or default_path())
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except (FileNotFoundError, ValueError, OSError):
        return {'open_tick': None, 'close_tick': None}
    return {
        'open_tick': _optional_int(data.get('open_tick')),
        'close_tick': _optional_int(data.get('close_tick')),
    }


def save(kind, tick, path=None):
    if kind not in ('open', 'close'):
        raise ValueError('endpoint kind must be open or close')
    path = Path(path or default_path())
    data = load(path)
    data[f'{kind}_tick'] = int(tick)
    if (data['open_tick'] is not None and data['close_tick'] is not None
            and data['open_tick'] == data['close_tick']):
        raise ValueError('open and close endpoints must be different')
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    os.replace(temporary, path)
    return data


def jog_limits(data):
    """Return (low, high, complete); legacy dual-profile limits are ignored."""
    opened = data.get('open_tick')
    closed = data.get('close_tick')
    if opened is None or closed is None:
        return -4096, 4095, False
    return min(int(opened), int(closed)), max(int(opened), int(closed)), True


def _optional_int(value):
    return None if value is None else int(value)
