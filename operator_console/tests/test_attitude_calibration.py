from dataclasses import replace
import json
import math

import pytest

from operator_console.attitude_calibration import resolve_calibration
from operator_console.vision_attitude import AttitudeTracker
from operator_console.tests.test_vision_attitude import packet, map_status
from terrain_map import contract as C


def profile(tmp_path, **changes):
    angle=math.radians(19.5)
    data=dict(schema_version=1,confirmed=True,confirmed_by='synthetic test fixture',reference='test only',
              source_basis='udp5005_imu_filtered_euler_xyz',target_frame='base_link',
              sensor_to_body_xyzw=[0,math.sin(angle/2),0,math.cos(angle/2)],
              raw_roll_sign=1,raw_pitch_sign=1,map_frame_confirmed=False)
    data.update(changes);p=tmp_path/'calibration.json';p.write_text(json.dumps(data));return p


def test_default_and_pending_profile_never_correct_raw_mount_pitch():
    for selected in (resolve_calibration(environ={}),resolve_calibration('operator_console/config/attitude-calibration.pending.json')):
        view=AttitudeTracker(selected).update(packet(roll_rad=0,pitch_rad=math.radians(19.5)),None,100.01)
        assert view.body_state=='UNVERIFIED' and view.body_pitch is None
        assert view.raw_pitch.value==pytest.approx(19.5)


def test_reviewed_rotation_removes_synthetic_mount_and_handles_simultaneous_axes(tmp_path):
    cal=resolve_calibration(profile(tmp_path),environ={})
    tracker=AttitudeTracker(cal)
    view=tracker.update(packet(roll_rad=0,pitch_rad=math.radians(19.5)),None,100.01)
    assert view.body_state=='LIVE' and view.body_pitch.value==pytest.approx(0,abs=1e-10)
    assert view.body_roll.value==pytest.approx(0,abs=1e-10)
    # Compound rotation must use the gravity vector, not independently subtract a pitch offset.
    up=cal.body_up(math.radians(30),math.radians(25))
    assert sum(v*v for v in up)==pytest.approx(1)
    tilted=tracker.update(packet(roll_rad=math.radians(30),pitch_rad=math.radians(25)),None,100.01)
    assert tilted.body_roll.value!=pytest.approx(30,abs=.1)
    assert tracker.update(packet(roll_rad=0,pitch_rad=0),None,101.2).body_state=='STALE'


@pytest.mark.parametrize('changes', [dict(confirmed='yes'),dict(sensor_to_body_xyzw=[0,0,0,0]),
    dict(sensor_to_body_xyzw=[0,0,0,float('nan')]),dict(sensor_to_body_xyzw=[0,0,0,10**400]),dict(raw_roll_sign=True),dict(reference=''),
    dict(target_frame='unknown'),dict(map_frame_confirmed=True)])
def test_unreviewed_or_invalid_calibration_cannot_be_enabled(tmp_path,changes):
    with pytest.raises(ValueError):resolve_calibration(profile(tmp_path,**changes))


def test_relative_json_env_and_cli_calibration_precedence(tmp_path):
    p=profile(tmp_path)
    assert resolve_calibration(config={'attitude_calibration':p.name},config_dir=tmp_path,environ={}).confirmed
    assert resolve_calibration(config={'attitude_calibration':None},environ={'L515_ATTITUDE_CALIBRATION':str(p)}).confirmed
    assert resolve_calibration(p,config={'attitude_calibration':'/missing'},environ={}).confirmed


def test_still_comparison_is_only_a_receive_time_reference_and_disappears_while_driving(tmp_path):
    cal=resolve_calibration(profile(tmp_path,map_frame_confirmed=True,map_reference='synthetic matched base axes'))
    tracker=AttitudeTracker(cal)
    snap=packet(roll_rad=0,pitch_rad=math.radians(19.5))
    status=map_status(plane_tilt=C.PlaneTilt(0,0,20),gravity=C.GravityTilt(0,0,0,0,0))
    view=tracker.update(snap,status,100.02)
    assert '정지 참고 차이' in view.comparison and '센서 시각 미확인' in view.comparison
    driving=tracker.update(snap,replace(status,gravity=None),100.03)
    assert driving.still is not None and driving.comparison.startswith('비교 보류')
    assert tracker.update(snap,status,100.4).comparison.startswith('비교 보류')


def test_reviewed_calibration_does_not_wrap_invalid_raw_euler_into_body_reading(tmp_path):
    cal=resolve_calibration(profile(tmp_path))
    view=AttitudeTracker(cal).update(packet(roll_rad=0,pitch_rad=1000),None,100.01)
    assert view.body_state=='INVALID' and view.body_pitch is None
