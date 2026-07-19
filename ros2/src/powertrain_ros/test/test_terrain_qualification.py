"""Host-side fail-closed L515 terrain qualification contracts."""
from __future__ import annotations

import ast
import copy
import math
from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE / "config/l515_terrain.yaml"


def _approved_document():
    return {
        "schema_version": 1,
        "qualification": {
            "status": "approved",
            "production_enabled": True,
        },
        "mount": {"pitch_deg": 25.0},
        "terrain": {
            "backend": "numpy",
            "roi": {
                "x": 80,
                "y": 160,
                "width": 480,
                "height": 240,
            },
            "depth_thresholds": {
                "min_depth_m": 0.2,
                "max_depth_m": 4.5,
                "min_valid_ratio": 0.8,
            },
        },
        "tf": {
            "base_link_to_l515_link": {
                "translation_m": [0.42, 0.0, 0.61],
                "rotation_xyzw": [
                    0.0,
                    math.sin(math.radians(12.5)),
                    0.0,
                    math.cos(math.radians(12.5)),
                ],
            },
        },
    }


def _write_yaml(tmp_path, document, name="qualification.yaml"):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def _qualification_module():
    from powertrain_ros import terrain_qualification

    return terrain_qualification


def test_setup_installs_fail_closed_terrain_qualification():
    tree = ast.parse((PACKAGE / "setup.py").read_text(encoding="utf-8"))
    source = ast.unparse(tree)

    assert "config/l515_terrain.yaml" in source


def test_package_declares_yaml_runtime_dependency():
    package_xml = (PACKAGE / "package.xml").read_text(encoding="utf-8")

    assert "<exec_depend>python3-yaml</exec_depend>" in package_xml
    assert "<exec_depend>ament_index_python</exec_depend>" in package_xml


def test_default_unapproved_template_cannot_enable_command_guidance():
    module = _qualification_module()

    with pytest.raises(ValueError, match="production_enabled"):
        module.require_command_guidance_qualified(
            guidance="terrain",
            propose=True,
            qualification_path=DEFAULT_CONFIG,
        )


@pytest.mark.parametrize("guidance", ("lane", "wall", "follow", "terrain"))
def test_every_command_producing_guidance_requires_qualification(
    tmp_path,
    guidance,
):
    module = _qualification_module()
    unapproved = _write_yaml(
        tmp_path,
        yaml.safe_load(DEFAULT_CONFIG.read_text()),
    )

    with pytest.raises(ValueError, match="production_enabled"):
        module.require_command_guidance_qualified(
            guidance=guidance,
            propose=True,
            qualification_path=unapproved,
        )


def test_diagnostics_only_guidance_does_not_require_production_approval():
    module = _qualification_module()

    assert module.require_command_guidance_qualified(
        guidance="terrain",
        propose=False,
        qualification_path=DEFAULT_CONFIG,
    ) is None


@pytest.mark.parametrize(
    ("mutation", "reason"),
    (
        (lambda data: data["mount"].update(pitch_deg=None), "mount.pitch_deg"),
        (lambda data: data["terrain"].update(roi=None), "terrain.roi"),
        (
            lambda data: data["terrain"].update(depth_thresholds=None),
            "terrain.depth_thresholds",
        ),
        (
            lambda data: data["tf"].update(base_link_to_l515_link=None),
            "tf.base_link_to_l515_link",
        ),
    ),
)
def test_incomplete_approved_qualification_is_rejected(
    tmp_path,
    mutation,
    reason,
):
    module = _qualification_module()
    document = copy.deepcopy(_approved_document())
    mutation(document)

    with pytest.raises(ValueError, match=reason.replace(".", r"\.")):
        module.load_approved_terrain_qualification(
            _write_yaml(tmp_path, document)
        )


def test_approved_tf_is_the_only_camera_extrinsic_source(tmp_path):
    module = _qualification_module()
    qualification = module.load_approved_terrain_qualification(
        _write_yaml(tmp_path, _approved_document())
    )

    assert qualification.translation_m == pytest.approx((0.42, 0.0, 0.61))
    assert qualification.roll_rad == pytest.approx(0.0, abs=1e-12)
    assert qualification.pitch_rad == pytest.approx(math.radians(25.0))
    assert qualification.yaw_rad == pytest.approx(0.0, abs=1e-12)
    assert qualification.roi == (80, 160, 480, 240)
    assert qualification.min_depth_m == pytest.approx(0.2)
    assert qualification.max_depth_m == pytest.approx(4.5)
    assert qualification.min_valid_ratio == pytest.approx(0.8)


def test_approved_mount_pitch_must_match_the_qualified_tf(tmp_path):
    module = _qualification_module()
    document = _approved_document()
    document["mount"]["pitch_deg"] = 20.0

    with pytest.raises(ValueError, match="mount.pitch_deg must match"):
        module.load_approved_terrain_qualification(
            _write_yaml(tmp_path, document)
        )


def test_launch_and_node_share_the_installed_qualification_source():
    launch = (PACKAGE / "launch/autonomy.launch.py").read_text(
        encoding="utf-8"
    )
    node = (
        PACKAGE / "powertrain_ros/autonomy_controller_node.py"
    ).read_text(encoding="utf-8")

    assert launch.count("require_command_guidance_qualified(") == 1
    assert '"terrain_qualification_file"' in launch
    assert "+ [qualification_gate]" in launch
    node_tree = ast.parse(node)
    declared = {
        call.args[0].value
        for call in ast.walk(node_tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "declare_parameter"
        and call.args
        and isinstance(call.args[0], ast.Constant)
    }
    assert "terrain_qualification_file" in declared
    assert 'declare_parameter("camera_height_m"' not in node
    assert 'declare_parameter("camera_pitch_down_deg"' not in node
    assert 'declare_parameter("camera_x_m"' not in node
    assert 'declare_parameter("camera_yaw_deg"' not in node
    assert "mount_pitch_rad=qualification.pitch_rad" in node
    assert "pitch_down_rad=0.0" in node


@pytest.mark.parametrize(
    ("filename", "guidance"),
    (
        # L515 기하를 실제로 소비하는 노드만 자격 게이트를 요구한다.
        ("lane_follower_node.py", "lane"),   # /l515/color/*
        ("wall_follower_node.py", "wall"),   # /l515/points
    ),
)
def test_direct_command_guidance_entrypoints_require_qualification(
    filename,
    guidance,
):
    source = (PACKAGE / "powertrain_ros" / filename).read_text(
        encoding="utf-8"
    )

    assert "enforce_node_command_guidance_qualification(" in source
    assert 'guidance="%s"' % guidance in source


def test_lead_follower_is_not_gated_on_l515_qualification():
    """lead_follower 는 L515 를 쓰지 않으므로 지형 자격에 묶으면 안 된다.

    이 노드는 /detected_objects(팔 D435i)와 TF 만 소비한다. 잠정 L515 mount/ROI
    값이 출력에 영향을 줄 수 없는데도 게이트를 걸면, 미승인 상태에서 추종이
    통째로 죽는다(2026-07-19 실기 컨테이너 검증에서 실제로 6건 실패로 드러남).
    """
    source = (PACKAGE / "powertrain_ros" / "lead_follower_node.py").read_text(
        encoding="utf-8"
    )
    assert "enforce_node_command_guidance_qualification(" not in source
    # 주석이 아니라 **실제 구독**을 본다.
    tree = ast.parse(source)
    subscribed = {
        call.args[1].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == "create_subscription"
        and len(call.args) > 1
        and isinstance(call.args[1], ast.Constant)
        and isinstance(call.args[1].value, str)
    }
    assert not [t for t in subscribed if t.startswith("/l515")], subscribed
