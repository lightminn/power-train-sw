"""Independent URDF fixtures catch omitted rotations, frame signs and mesh scale."""
import importlib.util
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "tools/extract_ackermann_geometry.py"


def extractor():
    assert SCRIPT.is_file(), "URDF geometry extractor is not implemented"
    spec = importlib.util.spec_from_file_location("geometry_extractor", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def rover(tmp_path):
    # Millimetre mesh: spindle along local Y, radius 200 mm, width 70 mm.
    vertices = [(x, y, z) for x, z in [(200, 0), (0, 200), (-200, 0), (0, -200)]
                for y in [-35, 35]]
    data = bytes(80) + struct.pack("<I", len(vertices))
    for vertex in vertices:
        data += struct.pack("<12fH", 0, 0, 0, *(vertex * 3), 0)
    (tmp_path / "tire.stl").write_bytes(data)
    robot = ET.Element("robot", name="fixture")
    ET.SubElement(robot, "link", name="base_link")
    ET.SubElement(robot, "link", name="chassis")

    def joint(name, parent, child, xyz, kind="fixed", rpy="0 0 0", axis="0 0 -1"):
        j = ET.SubElement(robot, "joint", name=name, type=kind)
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
        ET.SubElement(j, "origin", xyz=xyz, rpy=rpy)
        ET.SubElement(j, "axis", xyz=axis)

    joint("mount", "base_link", "chassis", "10 20 1", rpy=f"0 0 {math.pi / 2}")
    # After the mount rotation: CAD forward=Y; left=-X.
    for axle, x, left_y, right_y in [("front", 2, .4, -.6),
                                     ("center", .6, .7, -.9),
                                     ("rear", -2, .2, -.4)]:
        for side, y in [("left", left_y), ("right", right_y)]:
            role = f"{axle}_{side}"
            parent = "chassis"
            xyz = f"{x} {y} -1"
            if axle != "center":
                parent = f"knuckle_{role}"
                ET.SubElement(robot, "link", name=parent)
                joint(f"steer_{role}", "chassis", parent, f"{x} {y} -.5", "continuous")
                xyz = "0 0 -.5"
            link = ET.SubElement(robot, "link", name=f"tire_{role}")
            visual = ET.SubElement(link, "visual")
            ET.SubElement(visual, "origin", xyz=".001 0 0", rpy="0 0 0")
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(geometry, "mesh", filename="tire.stl", scale=".001 .001 .001")
            joint(f"wheel_{role}", parent, f"tire_{role}", xyz, "continuous", axis="0 1 0")
    urdf = tmp_path / "rover.urdf"
    ET.ElementTree(robot).write(urdf)
    return urdf


def test_rotated_chain_and_asymmetric_cad_origin_become_rep103(rover):
    result = extractor().extract_geometry(rover)
    assert result["wheelbase_m"] == pytest.approx(4)
    assert result["origin_cad_xy_m"] == pytest.approx([10.1, 20])
    assert result["tracks_m"] == pytest.approx({"front": 1, "mid": 1.6, "rear": .6})
    front = result["wheels"]["front_left"]
    assert front["cad_xyz_m"] == pytest.approx([9.6, 22, 0])
    assert front["rep103_xyz_m"] == pytest.approx([22, -9.6, 0])
    assert front["centered_rep103_xy_m"] == pytest.approx([2, .5])
    assert front["symmetric_xy_m"] == pytest.approx([2, .5])
    assert result["wheels"]["mid_right"]["symmetric_xy_m"] == pytest.approx([.6, -.8])
    assert result["wheels"]["rear_left"]["symmetric_xy_m"] == pytest.approx([-2, .3])


def test_mesh_scale_and_visual_offset_do_not_replace_spindle_center(rover):
    front = extractor().extract_geometry(rover)["wheels"]["front_left"]
    assert front["cad_xyz_m"] == pytest.approx([9.6, 22, 0])
    assert front["tire_mesh"]["bbox_extents_cad_m"] == pytest.approx([.07, .4, .4])
    assert front["tire_mesh"]["vertical_extent_radius_m"] == pytest.approx(.2)
    assert front["tire_mesh"]["radial_envelope_m"] == pytest.approx(.201)
    assert front["tire_mesh"]["axial_width_m"] == pytest.approx(.07)


def test_continuous_steering_has_no_invented_limit(rover):
    result = extractor().extract_geometry(rover)
    steer = result["wheels"]["front_left"]["steering"]
    assert steer["axis_cad"] == pytest.approx([0, 0, -1])
    assert steer["type"] == "continuous"
    assert steer["limits_rad"] is None
    assert steer["wheel_center_offset_cad_xy_m"] == pytest.approx([0, 0])
    assert result["wheels"]["mid_left"]["steering"] is None


@pytest.mark.parametrize("breakage", ["missing_wheel", "nonfinite_origin", "bad_left_right",
                                     "missing_parent", "duplicate_joint", "nonzero_mimic"])
def test_invalid_geometry_fails_closed(rover, breakage):
    tree = ET.parse(rover)
    robot = tree.getroot()
    j = robot.find("joint[@name='wheel_center_left']")
    if breakage == "missing_wheel":
        j.set("name", "unknown_wheel")
    elif breakage == "nonfinite_origin":
        j.find("origin").set("xyz", "nan 0 0")
    elif breakage == "bad_left_right":
        j.find("origin").set("xyz", ".6 -10 -1")
    elif breakage == "missing_parent":
        j.find("parent").set("link", "missing")
    elif breakage == "duplicate_joint":
        robot.append(ET.fromstring(ET.tostring(j)))
    else:
        ET.SubElement(j, "mimic", joint="wheel_center_right", offset=".1")
    tree.write(rover)
    with pytest.raises(ValueError):
        extractor().extract_geometry(rover)


def test_cli_reads_file_and_emits_json(rover):
    assert SCRIPT.is_file(), "URDF geometry extractor is not implemented"
    completed = subprocess.run([sys.executable, str(SCRIPT), str(rover)],
                               capture_output=True, text=True, check=True)
    result = json.loads(completed.stdout)
    assert result["wheelbase_m"] == pytest.approx(4)
    assert len(result["source"]["sha256"]) == 64
