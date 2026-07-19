import ast
import runpy
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "yolo_openvino_detection.py"


def test_openvino_demo_contains_no_fake_intrinsics_or_xyz_publication():
    source = SCRIPT.read_text(encoding="utf-8")
    tree = ast.parse(source)
    assigned_names = {
        target.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        for target in node.targets
        if isinstance(target, ast.Name)
    }

    assert not {"fx", "fy", "cx", "cy", "X_cam", "Y_cam", "Z"} & assigned_names
    assert "3D Pos" not in source
    assert "2D detection" in source


def test_camera_open_failure_exits_nonzero(monkeypatch):
    class ClosedCamera:
        def set(self, *_args):
            return True

        def isOpened(self):
            return False

    cv2 = SimpleNamespace(
        VideoCapture=lambda *_args: ClosedCamera(),
        CAP_V4L2=1,
        CAP_PROP_FOURCC=2,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        VideoWriter_fourcc=lambda *_args: 0,
    )
    ultralytics = SimpleNamespace(YOLO=lambda *_args, **_kwargs: object())
    monkeypatch.setitem(sys.modules, "cv2", cv2)
    monkeypatch.setitem(sys.modules, "ultralytics", ultralytics)

    with pytest.raises(SystemExit) as caught:
        runpy.run_path(str(SCRIPT), run_name="__main__")

    assert caught.value.code == 1
