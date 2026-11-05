"""도크 teach 표의 저장·복구와 잘못된 캡처를 검증한다."""

import json
import pytest

from dynamixel_control.dock_poses import DockPoseStore, load_docks
from auto_tool_change_fixtures import dock_fixture


def test_atomic_capture_roundtrip_and_partial_reload(tmp_path):
    path = tmp_path / 'docks.json'
    names = dock_fixture()['joint_names']
    store = DockPoseStore(str(path), names)
    samples = {name: dict(position=0.0, torque=0, velocity=0) for name in names}
    for slot in ('1', '2', '3'):
        for pose in ('approach', 'seat', 'retreat'):
            store.capture(slot, pose, dock_fixture()['slots'][slot]['tool_type'], samples, 0.01, 2.0, 100)
            restored = DockPoseStore(str(path), names)
            assert restored.load() == store.data
    assert store.calibrated and restored.calibrated
    assert load_docks(path, names) == store.data
    assert sorted(p.name for p in tmp_path.iterdir()) == ['docks.json']


@pytest.mark.parametrize('kind', ['torque', 'moving', 'missing', 'limit', 'nan'])
def test_bad_capture_makes_no_file_change(tmp_path, kind):
    path = tmp_path / 'docks.json'
    names = dock_fixture()['joint_names']
    store = DockPoseStore(str(path), names)
    samples = {name: dict(position=0.0, torque=0, velocity=0) for name in names}
    store.capture('1', 'approach', 'spur_1motor_gripper', samples, 0.01, 2.0, 100)
    before = path.read_bytes()
    if kind == 'torque':
        samples[names[0]]['torque'] = 1
    elif kind == 'moving':
        samples[names[0]]['velocity'] = 1
    elif kind == 'missing':
        del samples[names[0]]
    elif kind == 'limit':
        samples[names[0]]['position'] = 999
    else:
        samples[names[0]]['position'] = float('nan')
    with pytest.raises(ValueError):
        store.capture('1', 'seat', 'spur_1motor_gripper', samples, 0.01, 2.0, 100)
    assert path.read_bytes() == before
    assert 'seat' not in store.data['slots']['1']['poses']


@pytest.mark.parametrize('kind', ['names', 'limit', 'broken', 'missing_pose'])
def test_bad_load_is_uncalibrated(tmp_path, kind):
    data = dock_fixture()
    if kind == 'names':
        data['joint_names'][0] = 'other_joint'
    elif kind == 'limit':
        data['slots']['1']['poses']['seat']['positions'][0] = 999
    elif kind == 'missing_pose':
        del data['slots']['1']['poses']['seat']
    path = tmp_path / 'docks.json'
    path.write_text('broken json' if kind == 'broken' else json.dumps(data))
    store = DockPoseStore(str(path), dock_fixture()['joint_names'])
    try:
        store.load()
    except ValueError:
        pass
    assert not store.calibrated


def test_failed_atomic_save_preserves_prior_table(tmp_path, monkeypatch):
    import os
    path = tmp_path / 'docks.json'
    store = DockPoseStore(str(path), dock_fixture()['joint_names'])
    samples = {name: dict(position=0.0, torque=0, velocity=0) for name in store.joint_names}
    store.capture('1', 'approach', 'spur_1motor_gripper', samples, 0.01, 2.0, 100)
    before = path.read_bytes()
    def fail(*_args):
        raise OSError('injected atomic replace failure')
    monkeypatch.setattr(os, 'replace', fail)
    with pytest.raises(OSError):
        store.capture('1', 'seat', 'spur_1motor_gripper', samples, 0.01, 2.0, 100)
    assert path.read_bytes() == before
    assert list(tmp_path.iterdir()) == [path]


def test_tool_names_resolve_to_their_fixed_slots():
    from dynamixel_control.dock_poses import tool_slots
    data = dock_fixture()
    assert tool_slots(data, 'spur_1motor_gripper', 'dual_motor_gripper') == ('1', '2')
    assert tool_slots(data, 'dual_motor_gripper', 'cleaner') == ('2', '3')


@pytest.mark.parametrize('current,desired', [(None, 'cleaner'), ('cleaner', 'cleaner'),
                                             ('missing', 'cleaner'), ('cleaner', 'missing')])
def test_unresolved_or_same_tool_is_rejected(current, desired):
    from dynamixel_control.dock_poses import tool_slots
    with pytest.raises(ValueError):
        tool_slots(dock_fixture(), current, desired)


def test_duplicate_fixed_tool_slots_are_rejected():
    from dynamixel_control.dock_poses import validate_docks
    data = dock_fixture()
    data['slots']['2']['tool_type'] = data['slots']['1']['tool_type']
    with pytest.raises(ValueError, match='one fixed dock'):
        validate_docks(data, data['joint_names'])


def test_measured_exchange_settings_roundtrip_and_no_defaults(tmp_path):
    from dynamixel_control.dock_poses import validate_exchange
    path = tmp_path / 'docks.json'
    store = DockPoseStore(str(path), dock_fixture()['joint_names'])
    with pytest.raises(ValueError):
        validate_exchange(None)
    settings = dock_fixture()['exchange']
    store.configure_exchange(settings)
    restored = DockPoseStore(str(path), store.joint_names)
    restored.load()
    assert restored.data['exchange'] == settings
    before = path.read_bytes()
    with pytest.raises(ValueError):
        store.configure_exchange(dict(settings, feedback_timeout_s=0))
    assert path.read_bytes() == before
