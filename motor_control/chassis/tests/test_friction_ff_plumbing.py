"""friction_ff 노브가 build_real_corners→DriveOdriveCan까지 배선되는지 검증."""
import re
from pathlib import Path

import chassis.chassis_manager as chassis_manager

CHASSIS_DIR = Path(chassis_manager.__file__).resolve().parent
TELEOP_SERVER = (CHASSIS_DIR / "teleop_server.py").read_text(encoding="utf-8")
TELEOP_DUALSENSE = (CHASSIS_DIR / "teleop_dualsense.py").read_text(encoding="utf-8")
CHASSIS_NODE = (
    CHASSIS_DIR.parents[1]
    / "ros2/src/powertrain_ros/powertrain_ros/chassis_node.py"
).read_text(encoding="utf-8")


class _RecordingDrive:
    instances = []

    def __init__(self, node_id, channel="can0", **kwargs):
        self.node_id = node_id
        self.kwargs = kwargs
        _RecordingDrive.instances.append(self)


class _RecordingSteer:
    def __init__(self, motor_id, channel="can0"):
        self.motor_id = motor_id


def test_build_real_corners_passes_friction_kwargs(monkeypatch):
    import corner_module.drive_odrive_can as drive_mod
    import corner_module.steer_ak40 as steer_mod

    _RecordingDrive.instances = []
    monkeypatch.setattr(drive_mod, "DriveOdriveCan", _RecordingDrive)
    monkeypatch.setattr(steer_mod, "SteerAk40", _RecordingSteer)

    chassis_manager.build_real_corners(
        "can0", friction_ff=0.25, v_knee_turns_s=0.4
    )

    assert len(_RecordingDrive.instances) == 6
    for drive in _RecordingDrive.instances:
        assert drive.kwargs["friction_ff"] == 0.25
        assert drive.kwargs["v_knee"] == 0.4


def test_build_real_corners_defaults_are_off():
    import inspect

    signature = inspect.signature(chassis_manager.build_real_corners)
    assert signature.parameters["friction_ff"].default == 0.0
    assert signature.parameters["v_knee_turns_s"].default == 0.5


def test_teleop_clis_expose_friction_knobs_with_off_defaults():
    for source in (TELEOP_SERVER, TELEOP_DUALSENSE):
        assert re.search(
            r'"--friction-ff",\s*type=float,\s*default=0\.0', source
        ), "teleop CLI must expose --friction-ff default 0.0"
        assert re.search(
            r'"--v-knee",\s*type=float,\s*default=0\.5', source
        ), "teleop CLI must expose --v-knee default 0.5"
        assert "friction_ff=args.friction_ff" in source
        assert "v_knee_turns_s=args.v_knee" in source


def test_chassis_node_declares_friction_parameters():
    assert '"friction_ff", 0.0' in CHASSIS_NODE
    assert '"friction_v_knee", 0.5' in CHASSIS_NODE
    assert "friction_ff=friction_ff" in CHASSIS_NODE
    assert "v_knee_turns_s=friction_v_knee" in CHASSIS_NODE
