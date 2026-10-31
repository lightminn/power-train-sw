"""PoseTrack (spec §4.4): interpolation, gaps, invalid samples, epoch clearing."""
import math

import pytest

from terrain_map import odometry as O
from terrain_map.config import MapConfig
from terrain_map.pose import PoseTrack

CFG = MapConfig()


def sample(t, x=0.0, valid=True, reason=None, epoch=0, yaw=0.0):
    return O.PoseSample(t, x, 0.0, yaw, valid, reason, epoch)


def test_pose_track_interpolates_and_reports_reasons():
    track = PoseTrack(CFG)
    for s in (sample(10.00, 0.0), sample(10.02, 0.02, yaw=math.pi - 0.01), sample(10.04, 0.04, yaw=-math.pi + 0.01)):
        track.add(s)
    pose, reason = track.pose_at(10.03)
    assert reason is None and pose.x == pytest.approx(0.03)
    assert abs(O.wrap(pose.yaw - math.pi)) < 1e-6
    assert track.pose_at(10.05) == (None, "WAITING")
    assert track.pose_at(9.0) == (None, "ODOM_MISSING")


def test_pose_track_rejects_gaps_invalid_samples_and_clears_on_epoch():
    track = PoseTrack(CFG)
    track.add(sample(10.0))
    track.add(sample(10.2))
    assert track.pose_at(10.1) == (None, "POSE_GAP")
    track.add(sample(10.22, valid=False, reason="WHEELS_STALE"))
    assert track.pose_at(10.21) == (None, "WHEELS_STALE")
    track.add(sample(11.0, epoch=1))
    assert track.pose_at(10.1) == (None, "ODOM_MISSING")
