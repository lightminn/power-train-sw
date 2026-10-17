"""Correction hook (spec §4.7): identity, delta algebra, and the B fixtures' shape."""
import math

import numpy as np
import pytest

from terrain_map.config import MapConfig
from terrain_map.correction import IdentityCorrector, apply_delta, make_corrector, update_delta
from terrain_map.geometry import IDENTITY, Pose2D, compose, wrap_angle
from terrain_map.tests import b_fixtures as B

CFG = MapConfig()


def close(a, b, pos=1e-9, ang=1e-9):
    return math.hypot(a.x - b.x, a.y - b.y) < pos and abs(wrap_angle(a.yaw - b.yaw)) < ang


def test_identity_returns_the_seed_and_leaves_the_map_alone():
    m = B.synthetic_map(CFG, B.SCATTERED)
    seed = Pose2D(0.3, -0.2, 0.4)
    before = [m.encode_tile(tx, ty, 5.0) for tx, ty in m.tiles()]
    c = IdentityCorrector().correct(B.scan_from(CFG, B.SCATTERED, seed), seed, m)
    assert (c.pose, c.applied, c.score, c.reason) == (seed, False, None, "identity")
    assert [m.encode_tile(tx, ty, 5.0) for tx, ty in m.tiles()] == before
    assert make_corrector("identity").name == "identity"
    with pytest.raises(ValueError):
        make_corrector("height_match")


def test_delta_is_composed_not_overwritten():
    truth = [Pose2D(0.1 * k, 0.02 * k, 0.05 * k) for k in range(30)]
    drift = Pose2D(0.004, -0.002, 0.003)                         # odometry error per step
    odom, delta = [], IDENTITY
    o = truth[0]
    for k in range(1, 30):
        step = compose(compose(_inv(truth[k - 1]), truth[k]), drift)
        o = compose(o, step)
        odom.append(o)
        seed = apply_delta(delta, o)
        delta = update_delta(truth[k], o)                        # a perfect corrector
        assert close(apply_delta(delta, o), truth[k])
        if k > 1:
            assert math.hypot(seed.x - truth[k].x, seed.y - truth[k].y) < 0.01   # one step of drift only
    assert not close(odom[-1], truth[-1], pos=0.05)              # uncorrected drift is large


def _inv(p):
    from terrain_map.geometry import inverse
    return inverse(p)


# --- B fixtures -------------------------------------------------------------------
def test_true_pose_matches_and_a_known_offset_does_not():
    m = B.synthetic_map(CFG, B.SCATTERED)
    truth = Pose2D(0.2, 0.1, math.radians(5.0))
    scan = B.scan_from(CFG, B.SCATTERED, truth)
    ratio, overlap, diff = B.agreement(CFG, scan, truth, m)
    assert ratio > 0.97 and overlap > 500 and np.median(np.abs(diff)) < 1e-6
    off_ratio, _, _ = B.agreement(CFG, scan, compose(truth, Pose2D(0.10, 0.0, 0.0)), m)
    assert off_ratio < ratio - 0.01


def test_ambiguous_patterns_match_equally_after_a_shift():
    wall_map = B.synthetic_map(CFG, B.wall())
    truth = Pose2D(0.0, 0.0, 0.0)
    scan = B.scan_from(CFG, B.wall(), truth)
    along = B.agreement(CFG, scan, Pose2D(0.0, 0.10, 0.0), wall_map)[0]
    assert along == pytest.approx(B.agreement(CFG, scan, truth, wall_map)[0])
    stripe_map = B.synthetic_map(CFG, B.stripes())
    scan = B.scan_from(CFG, B.stripes(), truth)
    one_period = B.agreement(CFG, scan, Pose2D(0.30, 0.0, 0.0), stripe_map)[0]
    assert one_period == pytest.approx(B.agreement(CFG, scan, truth, stripe_map)[0], abs=0.01)


def test_a_constant_height_reference_offset_vanishes_after_the_median():
    """Codex r4: a 1.2 cm reference difference alone must not move the answer."""
    m = B.synthetic_map(CFG, B.SCATTERED)
    truth = Pose2D(0.2, 0.1, 0.0)
    _, _, diff = B.agreement(CFG, B.scan_from(CFG, B.SCATTERED, truth, height_bias_m=0.012), truth, m)
    assert np.median(diff) == pytest.approx(-0.012, abs=1e-6)
    assert np.max(np.abs(diff - np.median(diff))) < 1e-6
