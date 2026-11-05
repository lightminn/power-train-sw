"""Joint waypoints between a taught seat pose and the taught pose above it.

Both poses are taught with the tool head pointing at the floor.  The head
pitch is a linear sum of arm_joint_2/3/4, so interpolating the joints linearly
keeps the head at the taught orientation the whole way; the path stays within a
few millimetres of the straight seat->above line for the 5 cm bench rise.

The URDF model is used only for that pitch sum.  Its absolute positions do not
match this arm (2026-10-23 bench: a measured straight 5 cm rise needed about
-23/-35 deg arm_joint_2/3 zero offsets to fit), so no path is computed from it.
Pure functions: no ROS, no motors.
"""

import math

from dynamixel_control.joint_limits import JOINT_LIMITS

# Joint origins (x, z) in the parent link frame, from robot_arm.urdf.
ORIGIN_J2 = (0.01021, 0.0679)
LINK_2 = (-0.293, 0.06006)          # arm_joint_2 -> arm_joint_3
LINK_3 = (0.258002, 0.0600043)      # arm_joint_3 -> arm_joint_4
LINK_4 = (0.103998, 4.28408e-06)    # arm_joint_4 -> arm_joint_5 (via arm_tool_fixed)
# Rotation sign about +Y for each joint, from the URDF axes 0 -1 0 / 0 1 0 / 0 1 0.
# Hardware-checked 2026-10-23: two hand-held head-down poses differ by 2.6 deg
# under these signs; the all-positive guess rotated the head about 90 deg.
SIGNS = {'arm_joint_2': -1, 'arm_joint_3': 1, 'arm_joint_4': 1}
PLANAR = ('arm_joint_2', 'arm_joint_3', 'arm_joint_4')


def _rot(theta, vector):
    x, z = vector
    c, s = math.cos(theta), math.sin(theta)
    return x * c + z * s, -x * s + z * c


def fk(q2, q3, q4):
    """URDF wrist (x, z) in the arm_link1 frame and summed head pitch."""
    t2 = SIGNS['arm_joint_2'] * q2
    t3 = t2 + SIGNS['arm_joint_3'] * q3
    t4 = t3 + SIGNS['arm_joint_4'] * q4
    x, z = ORIGIN_J2
    for theta, link in ((t2, LINK_2), (t3, LINK_3), (t4, LINK_4)):
        dx, dz = _rot(theta, link)
        x, z = x + dx, z + dz
    return x, z, t4


def head_pitch(pose):
    return sum(SIGNS[name] * pose[name] for name in PLANAR)


def pitch_difference(a, b):
    """Radians between the head orientations of two poses."""
    difference = head_pitch(b) - head_pitch(a)
    return abs(math.atan2(math.sin(difference), math.cos(difference)))


def joint_path(seat, above, count, limits=True):
    """``count`` + 1 poses from ``seat`` to ``above`` (both included).

    Every joint moves linearly, so all of them arrive together.  Raises
    ValueError when a pose leaves the joint limits (unless ``limits`` is False:
    the bench limit switch OFF).
    """
    if type(count) is not int or count < 1:
        raise ValueError('waypoint count must be a positive integer')
    if set(seat) != set(above):
        raise ValueError('seat and above poses must hold the same joints')
    path = []
    for index in range(count + 1):
        t = index / count
        pose = {name: seat[name] + (above[name] - seat[name]) * t for name in seat}
        if limits:
            for name, value in pose.items():
                bounds = JOINT_LIMITS[name]
                if not bounds['lower'] <= value <= bounds['upper']:
                    raise ValueError(f'path leaves joint limit: {name}={value:.3f}')
        path.append(pose)
    return path
