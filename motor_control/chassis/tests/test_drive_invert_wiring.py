"""실기 코너 빌더가 wheel_map 기준으로 우측 구동축만 반전하는지 검증."""
import pytest

import chassis.chassis_manager as chassis_manager


class RecordingDrive:
    instances = []

    def __init__(self, node_id, channel="can0", **kwargs):
        self.node_id = node_id
        self.kwargs = kwargs
        RecordingDrive.instances.append(self)


class RecordingSteer:
    def __init__(self, motor_id, channel="can0"):
        self.motor_id = motor_id


def _capture_drive_inversion(monkeypatch, wheel_map):
    import corner_module.drive_odrive_can as drive_mod
    import corner_module.steer_ak40 as steer_mod

    RecordingDrive.instances = []
    monkeypatch.setattr(drive_mod, "DriveOdriveCan", RecordingDrive)
    monkeypatch.setattr(steer_mod, "SteerAk40", RecordingSteer)

    chassis_manager.build_real_corners(wheel_map=wheel_map)

    return {
        drive.node_id: drive.kwargs.get("invert")
        for drive in RecordingDrive.instances
    }


@pytest.mark.parametrize(
    ("wheel_map", "expected"),
    [
        (
            chassis_manager.DEFAULT_WHEEL_MAP,
            {
                11: False,
                12: True,
                13: False,
                14: True,
                15: False,
                16: True,
            },
        ),
        (
            chassis_manager.FOUR_WHEEL_MAP,
            {
                11: False,
                12: True,
                15: False,
                16: True,
            },
        ),
    ],
)
def test_build_real_corners_inverts_only_right_wheels(
    monkeypatch, wheel_map, expected
):
    assert _capture_drive_inversion(monkeypatch, wheel_map) == expected


def test_build_real_corners_derives_inversion_from_remapped_node_ids(monkeypatch):
    remapped = [
        chassis_manager.WheelMap("front_left", 1, 101),
        chassis_manager.WheelMap("front_right", 2, 102),
    ]

    assert _capture_drive_inversion(monkeypatch, remapped) == {
        101: False,
        102: True,
    }
