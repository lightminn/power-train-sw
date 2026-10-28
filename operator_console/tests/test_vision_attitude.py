"""Physical-value provenance, age and calibration boundaries for the vision panel."""
from dataclasses import replace
import json
import math

import pytest

from operator_console.telemetry import parse_telemetry
from operator_console.runtime_smoke import _terrain_map_packets
from operator_console.vision_attitude import AttitudeTracker
from terrain_map import contract as C


def packet(**fields):
    return parse_telemetry(json.dumps(dict(schema_version=1,sequence=1,**fields)).encode(),
                           received_monotonic_s=100)


def map_status(**changes):
    st=C.parse_packet(_terrain_map_packets(1)[0],received_s=100)
    return replace(st,**changes)


def test_missing_imu_and_encoders_never_become_zero_or_calibrated_attitude():
    tracker=AttitudeTracker()
    for snapshot in (None,packet()):
        view=tracker.update(snapshot,None,100.1)
        assert view.raw_roll.value is None and view.raw_pitch.value is None
        assert view.raw_pitch.text()=='미수신'
        assert view.body_state=='UNVERIFIED'
        assert all(reading.value is None and reading.text()=='미수신' for _,reading in view.suspension)


def test_mount_pitch_is_shown_verbatim_not_subtracted_and_measurement_age_is_not_invented():
    view=AttitudeTracker().update(packet(roll_rad=0,pitch_rad=math.radians(19.5)),None,100.2)
    assert view.raw_roll.value==0
    assert view.raw_pitch.value==pytest.approx(19.5)
    assert view.raw_pitch.age_basis=='패킷 수신 후'
    assert view.raw_pitch.age_s==pytest.approx(.2)
    assert view.body_state=='UNVERIFIED'
    assert view.comparison.startswith('비교 보류')


@pytest.mark.parametrize('name',['roll_rad','pitch_rad'])
@pytest.mark.parametrize('invalid',[True,'0.1',float('nan'),float('inf')])
def test_invalid_imu_values_are_not_received_measurements(name,invalid):
    with pytest.raises(ValueError):
        packet(**{name:invalid})


@pytest.mark.parametrize('name',['steer_deg','drive_turns_per_s'])
@pytest.mark.parametrize('invalid',[True,'0.1',float('nan'),float('inf')])
def test_invalid_wheel_readings_reject_the_packet(name,invalid):
    with pytest.raises(ValueError):
        packet(wheel_statuses=[dict(name='front_left',mode='IDLE',**{name:invalid})])


def test_fresh_packet_with_missing_fields_does_not_reuse_old_imu():
    tracker=AttitudeTracker()
    assert tracker.update(packet(roll_rad=.1,pitch_rad=.2),None,100.1).raw_roll.state=='LIVE'
    view=tracker.update(packet(),None,100.2)
    assert view.link_state=='LIVE' and view.raw_roll.state=='WAITING'
    assert view.raw_roll.value is None
    stale=tracker.update(packet(roll_rad=.1),None,101.1)
    assert stale.link_state=='STALE' and stale.raw_roll.value is None
    recovered=tracker.update(replace(packet(roll_rad=0),received_monotonic_s=102),None,102.1)
    assert recovered.raw_roll.value==0 and recovered.raw_roll.state=='LIVE'


def test_null_encoder_group_does_not_drop_other_received_telemetry():
    view=AttitudeTracker().update(packet(roll_rad=.1,pitch_rad=.2,
        suspension_joints_rad=None,suspension_age_ms=None),None,100.1)
    assert view.link_state=='LIVE' and view.raw_pitch.value==pytest.approx(math.degrees(.2))
    assert all(r.state=='WAITING' and r.value is None for _,r in view.suspension)


def test_wheels_use_actual_feedback_and_stale_per_wheel_not_command_or_hall_assumptions():
    good=dict(name='front_left',mode='ARMED',steer_deg=-12.5,drive_turns_per_s=.12,
              command_turns_per_s=2.0,stale=False)
    bad=dict(name='front_right',mode='ARMED',steer_deg=7,drive_turns_per_s=.5,stale=True)
    view=AttitudeTracker().update(packet(wheel_statuses=[good,bad]),None,100.1)
    assert view.wheels[0].steer.value==-12.5
    assert view.wheels[0].speed.value==.12  # never replace a Hall reading with command/zero
    assert view.wheels[1].speed.state=='STALE' and view.wheels[1].speed.value is None
    assert view.wheels[2].speed.state=='WAITING'
    duplicate=AttitudeTracker().update(packet(wheel_statuses=[good,good]),None,100.1)
    assert duplicate.wheels[0].steer.state=='INVALID'


def test_encoder_group_requires_measurement_age_and_keeps_missing_channels_absent():
    joints=dict(ml_rocker_joint=0,br_bogie_joint=-.1,bl_bogie_joint=None)
    tracker=AttitudeTracker()
    unknown=tracker.update(packet(suspension_joints_rad=joints),None,100.1)
    assert unknown.suspension[0][1].state=='AGE_UNKNOWN'
    assert unknown.suspension[0][1].value is None
    measured=packet(suspension_joints_rad=joints,suspension_age_ms=100,
        suspension_source='as5048b@fixture',suspension_frame='urdf_joint')
    view=tracker.update(measured,None,100.2)
    assert view.suspension[0][1].value==0
    assert view.suspension[1][1].value is None
    assert view.suspension[2][1].value is None
    assert view.suspension[3][1].value==pytest.approx(math.degrees(-.1))
    assert view.suspension_age_s==pytest.approx(.3)
    assert 'as5048b@fixture' in view.suspension_source
    assert view.suspension_frame=='urdf_joint'
    assert tracker.update(measured,None,100.95).suspension[0][1].state=='STALE'


@pytest.mark.parametrize('field,value',[
    ('suspension_source',True),('suspension_source',''),('suspension_frame',3),
    ('suspension_frame','bad\nframe'),('suspension_age_ms',-1),
    ('suspension_age_ms',float('nan')),('suspension_age_ms',True),
])
def test_optional_encoder_metadata_must_be_valid(field,value):
    with pytest.raises(ValueError):
        packet(**{field:value})


def test_stationary_map_body_angles_and_ground_plane_are_not_a_live_euler_pose():
    tracker=AttitudeTracker()
    status=map_status(plane_tilt=C.PlaneTilt(4,-1,200),gravity=C.GravityTilt(6,-2,6.3,3,1))
    view=tracker.update(packet(pitch_rad=math.radians(19.5)),status,100.1)
    assert view.raw_pitch.value==pytest.approx(19.5)
    assert view.plane_pitch.value==4
    assert view.still.body_pitch_deg==3
    assert view.still_age_s==pytest.approx(.3)
    assert view.body_state=='UNVERIFIED' and '보정' in view.comparison
    # A later moving packet has no gravity. Keep only the explicitly aged static record.
    moving=replace(status,received_s=101,sequence=2,plane_tilt=None,gravity=None)
    view=tracker.update(packet(),moving,101.1)
    assert view.plane_pitch.value is None
    assert view.still.body_pitch_deg==3 and view.still_age_s==pytest.approx(1.3)
    reset=tracker.update(None,replace(moving,map_epoch=1),101.2)
    assert reset.still is None


def test_map_link_and_plane_measurement_age_are_independent():
    tracker=AttitudeTracker()
    status=map_status(plane_tilt=C.PlaneTilt(4,-1,900),gravity=C.GravityTilt(6,-2,6.3,3,1))
    view=tracker.update(None,status,100.2)
    assert view.map_link_state=='LIVE'
    assert view.plane_pitch.state=='STALE' and view.plane_pitch.value is None
    assert view.still is None
    missing_clock=map_status(gravity=C.GravityTilt(6,-2,6.3,3,1),plane_tilt=None)
    assert tracker.update(None,missing_clock,100.1).still is None
    future=tracker.update(replace(packet(roll_rad=.1),received_monotonic_s=101),None,100)
    assert future.raw_roll.state=='AGE_UNKNOWN' and future.raw_roll.value is None
