"""Mount rotation, base-horizontal frame and SE(2) helpers (spec §4.3, §4.8)."""
import math

import numpy as np
import pytest

from terrain_map import geometry as G
from terrain_map.tests.world_render import BasePose, Mount, camera_in_world, rot_x, rot_y, rot_z

D = math.radians


def test_zero_mount_is_the_legacy_optical_to_body_permutation():
    r = G.base_to_optical(0.0, 0.0, 0.0)
    v = np.array([0.3, -0.2, 0.9])                      # optical (x right, y down, z forward)
    assert np.allclose(r.T @ v, [0.9, -0.3, 0.2])       # imu_tilt opt_to_body: (z, -x, -y)


def test_pitch_down_maps_base_forward_into_the_optical_frame():
    assert np.allclose(G.base_to_optical(20.0, 0.0, 0.0) @ [1, 0, 0],
                       [0.0, -math.sin(D(20)), math.cos(D(20))], atol=1e-12)


@pytest.mark.parametrize("pitch, roll, yaw", [(20, 0, 0), (0, 20, 0), (0, 0, 30), (20, 20, 30), (35, -15, -25)])
def test_optical_axes_match_hand_expanded_rpy(pitch, roll, yaw):
    """Independent expectation: columns of Rz(y)·Ry(p)·Rx(r) written out by hand."""
    p, r, y = D(pitch), D(roll), D(yaw)
    cp, sp, cr, sr, cy, sy = math.cos(p), math.sin(p), math.cos(r), math.sin(r), math.cos(y), math.sin(y)
    mount_x = np.array([cy * cp, sy * cp, -sp])                                   # optical z (forward)
    mount_y = np.array([cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr])  # optical -x (left)
    mount_z = np.array([cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr])  # optical -y (up)
    r_ob = G.base_to_optical(pitch, roll, yaw)
    assert np.allclose(r_ob.T @ [0, 0, 1], mount_x, atol=1e-12)
    assert np.allclose(r_ob.T @ [1, 0, 0], -mount_y, atol=1e-12)
    assert np.allclose(r_ob.T @ [0, 1, 0], -mount_z, atol=1e-12)


def test_wrong_multiplication_order_is_detected_by_the_hand_expansion():
    p, r, y = D(20), D(20), D(30)
    wrong = (rot_x(r) @ rot_y(p) @ rot_z(y))            # X·Y·Z instead of Z·Y·X
    expected_forward = np.array([math.cos(y) * math.cos(p), math.sin(y) * math.cos(p), -math.sin(p)])
    assert not np.allclose(wrong @ [1, 0, 0], expected_forward, atol=1e-3)


def _plane_in_optical(base: BasePose, mount: Mount, floor_z: float = 0.0):
    origin, r_wo = camera_in_world(base, mount)
    normal = r_wo.T @ np.array([0.0, 0.0, 1.0])
    return normal, origin[2] - floor_z, origin, r_wo


def _true_base_horizontal(base: BasePose, world_xy):
    """Ground point relative to the base origin's vertical foot, along the base heading."""
    heading = base.rotation() @ [1.0, 0.0, 0.0]
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    left = np.cross([0.0, 0.0, 1.0], heading)
    rel = np.array([world_xy[0] - base.x, world_xy[1] - base.y, 0.0])
    return np.array([rel @ heading, rel @ left])


def _measured(base: BasePose, mount: Mount, target_base_xy):
    normal, offset, origin, r_wo = _plane_in_optical(base, mount)
    heading = base.rotation() @ [1.0, 0.0, 0.0]
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    left = np.cross([0.0, 0.0, 1.0], heading)
    world = np.array([base.x, base.y, 0.0]) + target_base_xy[0] * heading + target_base_xy[1] * left
    p_opt = r_wo.T @ (world - origin)
    r_ob = mount.base_to_optical()
    frame = G.base_horizontal_frame(normal, offset, r_ob, np.asarray(mount.t, float))
    return G.to_base_xy(frame, p_opt[None, :])[0], world


@pytest.mark.parametrize("base, mount", [
    (BasePose(z=0.15, roll_deg=20.0), Mount(t=(0.0, 0.0, 0.35), pitch_down_deg=20.0)),   # camera height fixed roll case
    (BasePose(z=0.0, roll_deg=20.0), Mount(t=(0.348, 0.0, 0.5), pitch_down_deg=20.0)),   # mount translation 0.5 m, roll 20
    (BasePose(), Mount()),                                                                # default CAD mount (-R^T t case)
    (BasePose(roll_deg=-12.0, pitch_deg=8.0, yaw_deg=40.0), Mount(roll_deg=5.0, yaw_deg=10.0)),
])
def test_base_horizontal_coordinates_match_world_truth(base, mount):
    measured, world = _measured(base, mount, (2.0, 0.0))
    truth = _true_base_horizontal(base, world[:2])
    assert np.linalg.norm(measured - truth) < 0.01


def test_optical_axis_projection_negative_control_is_off_by_decimetres():
    base, mount = BasePose(z=0.15, roll_deg=20.0), Mount(t=(0.0, 0.0, 0.35), pitch_down_deg=20.0)
    normal, offset, origin, r_wo = _plane_in_optical(base, mount)
    f = np.array([0.0, 0.0, 1.0]) - normal[2] * normal
    f /= np.linalg.norm(f)
    l = np.cross(normal, f)
    heading = base.rotation() @ [1.0, 0.0, 0.0]
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    foot = np.array([origin[0], origin[1], 0.0])
    world = foot + 2.0 * heading
    p = r_wo.T @ (world - origin)
    assert abs((p @ f) - 2.0) + abs(p @ l) > 0.2


def test_wrong_origin_formula_negative_control():
    base, mount = BasePose(), Mount()
    normal, offset, origin, r_wo = _plane_in_optical(base, mount)
    r_ob = mount.base_to_optical()
    t = np.asarray(mount.t, float)
    good = G.base_horizontal_frame(normal, offset, r_ob, t)
    bad = good._replace(origin_f=float((-r_ob.T @ t) @ good.f), origin_l=float((-r_ob.T @ t) @ good.l))
    p = r_wo.T @ (np.array([2.0 + base.x, base.y, 0.0]) - origin)
    assert np.linalg.norm(G.to_base_xy(bad, p[None])[0] - G.to_base_xy(good, p[None])[0]) > 0.2


@pytest.mark.parametrize("pitch", [0.0, 10.0])
def test_base_origin_height_is_independent_of_body_pitch(pitch):
    base, mount = BasePose(pitch_deg=pitch), Mount()
    normal, offset, _, _ = _plane_in_optical(base, mount)
    height = G.base_origin_height(normal, offset, mount.base_to_optical(), np.asarray(mount.t, float))
    assert height == pytest.approx(base.z, abs=1e-9)


def test_pose2d_compose_inverse_and_transform():
    a = G.Pose2D(1.0, 2.0, math.pi / 2)
    b = G.Pose2D(0.5, 0.0, math.pi / 4)
    ab = G.compose(a, b)
    assert (ab.x, ab.y) == pytest.approx((1.0, 2.5))
    assert ab.yaw == pytest.approx(3 * math.pi / 4)
    ident = G.compose(a, G.inverse(a))
    assert (ident.x, ident.y, ident.yaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
    assert G.transform_points(a, np.array([[1.0, 0.0]])) == pytest.approx(np.array([[1.0, 3.0]]))
    assert G.wrap_angle(3 * math.pi) == pytest.approx(math.pi) or G.wrap_angle(3 * math.pi) == pytest.approx(-math.pi)


def test_renderer_flat_floor_matches_the_detector_plane():
    from terrain_overlay.camera import CameraModel
    from terrain_overlay.detector import TerrainDetector
    from terrain_map.tests.world_render import flat, render_depth
    camera = CameraModel(fx=230.0, fy=230.0, cx=160.0, cy=120.0, width=320, height=240)
    base, mount = BasePose(pitch_deg=5.0), Mount()
    depth = render_depth(camera, base, mount, flat())
    detector = TerrainDetector(camera)
    result = detector.process(depth, 0.0)
    origin, _ = camera_in_world(base, mount)
    assert result.status == "OK"
    assert result.camera_height_m == pytest.approx(origin[2], abs=0.001)
