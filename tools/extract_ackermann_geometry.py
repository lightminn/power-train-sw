#!/usr/bin/env python3
"""Audit zero-pose Ackermann dimensions in the ZETIN integrated CAD URDF.

Usage: python tools/extract_ackermann_geometry.py /path/to/2026_07_24_URDF.urdf

Read-only, standard-library-only extractor for the semantic wheel/steer joint
names assigned by postprocess_20260724.py. Keep the relative binary STL meshes
beside the URDF. This is an offline source audit, not a runtime URDF dependency.
The exporter uses CAD +Y forward, +X right, +Z up; REP-103 is (Y, -X, Z).
All suspension and steering joint coordinates are zero. A loaded tire's rolling
radius and a safe steering limit cannot be certified from this mesh/URDF.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


IDENTITY = [[float(i == j) for j in range(4)] for i in range(4)]
ROLES = ("front_left", "front_right", "mid_left", "mid_right", "rear_left", "rear_right")


def _numbers(value, count=3):
    values = [float(x) for x in value.split()]
    if len(values) != count or not all(math.isfinite(x) for x in values):
        raise ValueError(f"Expected {count} finite numbers: {value!r}")
    return values


def _origin(element):
    if element is None:
        return IDENTITY
    x, y, z = _numbers(element.get("xyz", "0 0 0"))
    r, p, yaw = _numbers(element.get("rpy", "0 0 0"))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yaw), math.sin(yaw)
    # URDF fixed-axis roll/pitch/yaw: Rz(yaw) Ry(pitch) Rx(roll).
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
            [-sp, cp * sr, cp * cr, z], [0, 0, 0, 1]]


def _multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _point(transform, point, vector=False):
    return [sum(transform[i][j] * point[j] for j in range(3))
            + (0 if vector else transform[i][3]) for i in range(3)]


def _axis(joint, transform):
    element = joint.find("axis")
    axis = _point(transform, _numbers(element.get("xyz", "1 0 0")
                                    if element is not None else "1 0 0"), vector=True)
    norm = math.sqrt(sum(x * x for x in axis))
    if norm == 0:
        raise ValueError(f"Zero joint axis: {joint.get('name')}")
    return [x / norm for x in axis]


def _index(elements):
    result = {}
    for element in elements:
        name = element.get("name")
        if not name or name in result:
            raise ValueError(f"Missing or duplicate name: {name}")
        result[name] = element
    return result


def _tire_mesh(link, transform, spindle_axis, urdf):
    visual = link.find("visual")
    mesh = visual.find("geometry/mesh") if visual is not None else None
    if mesh is None:
        raise ValueError(f"Expected tire visual mesh in {link.get('name')}")
    filename = mesh.get("filename", "")
    if "://" in filename or not filename.lower().endswith(".stl"):
        raise ValueError("Expected a local, relative or absolute binary STL path")
    path = (urdf.parent / filename).resolve()
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if count == 0 or len(data) != 84 + count * 50:
        raise ValueError(f"Unsupported or malformed binary STL: {path}")
    scale = _numbers(mesh.get("scale", "1 1 1"))
    if min(scale) <= 0:
        raise ValueError("Mesh scale must be positive")
    visual_transform = _multiply(transform, _origin(visual.find("origin")))
    spindle = [transform[i][3] for i in range(3)]
    low, high = [math.inf] * 3, [-math.inf] * 3
    axial_low, axial_high, radial = math.inf, -math.inf, 0.0
    for i in range(count):
        vertices = struct.unpack_from("<9f", data, 84 + 50 * i + 12)
        for start in (0, 3, 6):
            point = _point(visual_transform, [vertices[start + j] * scale[j] for j in range(3)])
            if not all(math.isfinite(x) for x in point):
                raise ValueError(f"Non-finite STL vertex: {path}")
            offset = [point[j] - spindle[j] for j in range(3)]
            axial = sum(offset[j] * spindle_axis[j] for j in range(3))
            radius = math.sqrt(sum((offset[j] - axial * spindle_axis[j]) ** 2 for j in range(3)))
            radial, axial_low, axial_high = max(radial, radius), min(axial_low, axial), max(axial_high, axial)
            low = [min(low[j], point[j]) for j in range(3)]
            high = [max(high[j], point[j]) for j in range(3)]
    extents = [high[j] - low[j] for j in range(3)]
    return {"file": str(path), "sha256": hashlib.sha256(data).hexdigest(),
            "bbox_min_cad_m": low, "bbox_max_cad_m": high,
            "bbox_extents_cad_m": extents, "vertical_extent_radius_m": extents[2] / 2,
            "radial_envelope_m": radial, "axial_width_m": axial_high - axial_low}


def extract_geometry(urdf_path):
    """Return source evidence and symmetric wheel coordinates in metres.

    Wheel centers are drive-joint child origins, not visual mesh bbox centers.
    Report raw and centered coordinates separately from the symmetric runtime
    approximation. Center the planar frame at the front/rear axle midpoint;
    leave CAD Z unchanged (it is not the ground-frame height).
    """
    urdf = Path(urdf_path).resolve()
    data = urdf.read_bytes()
    robot = ET.fromstring(data)
    links, joints = _index(robot.findall("link")), _index(robot.findall("joint"))
    parents = {}
    for joint in joints.values():
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        if parent not in links or child not in links or child in parents:
            raise ValueError(f"Invalid link tree at {joint.get('name')}")
        parents[child] = joint
    roots = set(links) - set(parents)
    if roots != {"base_link"}:
        raise ValueError(f"Expected one CAD base_link root, got {sorted(roots)}")
    transforms = {"base_link": IDENTITY}

    def pose(link, visiting=()):
        if link in transforms:
            return transforms[link]
        if link in visiting or link not in parents:
            raise ValueError(f"Disconnected or cyclic link tree at {link}")
        joint = parents[link]
        mimic = joint.find("mimic")
        if mimic is not None and _numbers(mimic.get("offset", "0"), 1)[0] != 0:
            raise ValueError(f"Nonzero mimic offset requires explicit pose: {joint.get('name')}")
        transforms[link] = _multiply(pose(joint.find("parent").get("link"), visiting + (link,)),
                                     _origin(joint.find("origin")))
        return transforms[link]

    wheels = {}
    for role in ROLES:
        semantic_role = role.replace("mid_", "center_")
        name = f"wheel_{semantic_role}"
        if name not in joints:
            raise ValueError(f"Required joint is absent: {name}")
        joint = joints[name]
        link = joint.find("child").get("link")
        transform = pose(link)
        xyz = [transform[i][3] for i in range(3)]
        axis = _axis(joint, transform)
        chain, current = [], link
        while current in parents:
            chain.append(parents[current].get("name"))
            current = parents[current].find("parent").get("link")
        steering = None
        if not role.startswith("mid_"):
            steer_name = f"steer_{semantic_role}"
            if steer_name not in chain:
                raise ValueError(f"Required steering ancestor is absent: {steer_name}")
            steer = joints[steer_name]
            steer_pose = pose(steer.find("child").get("link"))
            limit = steer.find("limit")
            steering = {"joint": steer_name, "type": steer.get("type"),
                        "cad_xyz_m": [steer_pose[i][3] for i in range(3)],
                        "axis_cad": _axis(steer, steer_pose),
                        "limits_rad": None if steer.get("type") == "continuous" or limit is None
                        else {k: _numbers(limit.get(k), 1)[0] for k in ("lower", "upper")},
                        "wheel_center_offset_cad_xy_m": [xyz[i] - steer_pose[i][3] for i in range(2)]}
        wheels[role] = {"wheel_joint": name, "wheel_link": link, "chain": list(reversed(chain)),
                        "cad_xyz_m": xyz, "rep103_xyz_m": [xyz[1], -xyz[0], xyz[2]],
                        "axis_cad": axis, "steering": steering,
                        "tire_mesh": _tire_mesh(links[link], transform, axis, urdf)}
    axle_centers, tracks = {}, {}
    for axle in ("front", "mid", "rear"):
        left, right = (wheels[f"{axle}_{side}"]["cad_xyz_m"] for side in ("left", "right"))
        tracks[axle] = right[0] - left[0]
        if tracks[axle] <= 0:
            raise ValueError(f"CAD +X must point right: invalid {axle} track")
        axle_centers[axle] = [(left[i] + right[i]) / 2 for i in range(2)]
    if not axle_centers["front"][1] > axle_centers["mid"][1] > axle_centers["rear"][1]:
        raise ValueError("CAD +Y must point forward: axle order is invalid")
    center_x = sum(xy[0] for xy in axle_centers.values()) / 3
    center_y = (axle_centers["front"][1] + axle_centers["rear"][1]) / 2
    for role, wheel in wheels.items():
        axle, side = role.split("_")
        x, y, _ = wheel["cad_xyz_m"]
        wheel["centered_rep103_xy_m"] = [y - center_y, center_x - x]
        wheel["symmetric_xy_m"] = [axle_centers[axle][1] - center_y,
                                    tracks[axle] / 2 * (1 if side == "left" else -1)]
    return {"source": {"file": str(urdf), "robot": robot.get("name"),
                        "sha256": hashlib.sha256(data).hexdigest()},
            "pose": "all joint coordinates zero; no loaded-suspension or ground-height correction",
            "cad_to_rep103": "(x, y, z) -> (y, -x, z)",
            "origin_cad_xy_m": [center_x, center_y], "axle_centers_cad_xy_m": axle_centers,
            "wheelbase_m": axle_centers["front"][1] - axle_centers["rear"][1],
            "tracks_m": tracks, "wheels": wheels,
            "limits": "URDF joint limits are source metadata, not certified controller settings",
            "tire_radius_note": "Mesh vertical half-extent and radial envelope are distinct; loaded rolling radius is unverified"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    args = parser.parse_args()
    try:
        result = extract_geometry(args.urdf)
    except (OSError, ValueError, ET.ParseError) as exc:
        parser.exit(1, f"URDF geometry extraction failed: {exc}\n")
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
