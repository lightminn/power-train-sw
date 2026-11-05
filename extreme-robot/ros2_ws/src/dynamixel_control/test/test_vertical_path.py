"""Seat->above joint path: head pitch held, joints arrive together, URDF-anchored signs."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from dynamixel_control import vertical_path as V

URDF = (Path(__file__).parents[2] / 'robot_arm_description/urdf/robot_arm.urdf')
# 2026-10-23 bench: head held at the floor, B measured 5 cm straight above A.
A = {'arm_joint_1': -0.3666, 'arm_joint_2': 2.3297, 'arm_joint_3': 0.5574,
     'arm_joint_4': -0.4341, 'arm_joint_5': -0.2393}
B = {'arm_joint_1': -0.3789, 'arm_joint_2': 1.9936, 'arm_joint_3': 0.6288,
     'arm_joint_4': -0.7961, 'arm_joint_5': -0.2393}


def test_geometry_and_signs_match_the_urdf():
    joints = {j.get('name'): j for j in ET.parse(URDF).getroot().findall('joint')}

    def xz(name):
        x, _, z = (float(v) for v in joints[name].find('origin').get('xyz').split())
        return x, z

    assert xz('arm_joint_2') == pytest.approx(V.ORIGIN_J2)
    assert xz('arm_joint_3') == pytest.approx(V.LINK_2)
    assert xz('arm_joint_4') == pytest.approx(V.LINK_3)
    assert xz('arm_joint_5') == pytest.approx(V.LINK_4)
    for name, sign in V.SIGNS.items():
        assert joints[name].find('axis').get('xyz') == f'0 {sign} 0'


def test_the_measured_head_down_pair_keeps_the_head_direction():
    assert math.degrees(V.pitch_difference(A, B)) < 5
    assert V.fk(*(A[n] for n in V.PLANAR))[2] == pytest.approx(V.head_pitch(A))


def test_all_positive_signs_would_have_turned_the_head():
    # The sign guess that rotated the head on the bench must stay rejected.
    signs = dict(V.SIGNS)
    try:
        V.SIGNS.update({'arm_joint_2': 1, 'arm_joint_3': 1, 'arm_joint_4': 1})
        assert math.degrees(V.pitch_difference(A, B)) > 30
    finally:
        V.SIGNS.update(signs)


def test_joint_path_moves_every_joint_linearly_and_holds_the_head():
    path = V.joint_path(A, B, 10, limits=False)
    assert len(path) == 11
    assert path[0] == pytest.approx(A) and path[-1] == pytest.approx(B)
    for index, pose in enumerate(path):
        for name in A:
            assert pose[name] == pytest.approx(A[name] + (B[name] - A[name]) * index / 10)
    pitches = [V.head_pitch(pose) for pose in path]
    assert max(pitches) - min(pitches) <= V.pitch_difference(A, B) + 1e-12


def test_joint_limits_are_checked_unless_switched_off():
    with pytest.raises(ValueError, match='joint limit'):
        V.joint_path(A, B, 10)        # arm_joint_2 2.33 rad is past its 1.78 limit
    assert V.joint_path(A, B, 10, limits=False)


@pytest.mark.parametrize('count', [0, -1, 2.5])
def test_invalid_waypoint_counts_are_refused(count):
    with pytest.raises(ValueError):
        V.joint_path(A, B, count, limits=False)
