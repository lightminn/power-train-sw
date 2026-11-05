"""lidar_dev offline tools on a synthetic recording (conda: numpy + cv2)."""
import json
import sys

import cv2
import numpy as np
import pytest

from lidar_dev import ground, overlay, record, scan_terrain
from lidar_dev.recording import load_recording
from terrain_overlay.tests.scenes import DEPTH, default_scene


def write_recording(folder, frames=3, legacy=False):
    (folder / "depth").mkdir(parents=True)
    intrinsics = ([DEPTH.fx, DEPTH.fy, DEPTH.cx, DEPTH.cy] if legacy else
                  {"fx": DEPTH.fx, "fy": DEPTH.fy, "ppx": DEPTH.cx, "ppy": DEPTH.cy,
                   "width": DEPTH.width, "height": DEPTH.height})
    (folder / "meta.json").write_text(json.dumps(
        {"depth_scale_m": 0.00025, "frame_period_s": 0.1, "depth_intrinsics": intrinsics}))
    for index in range(frames):
        cv2.imwrite(str(folder / "depth" / f"{index:06d}.png"), default_scene())
    return folder


@pytest.mark.parametrize("legacy", [False, True])
def test_recording_loads_both_meta_formats(tmp_path, legacy):
    recording = load_recording(write_recording(tmp_path / "scene", legacy=legacy))
    assert recording.depth_camera == DEPTH
    assert len(recording.depth_paths) == 3


def test_scan_summary_sees_the_box(tmp_path):
    summary = scan_terrain.summarize(write_recording(tmp_path / "scene"))
    assert summary["frames"] == 3
    assert summary["statuses"] == {"OK/FRESH": 3}
    assert summary["zones_median_m"]["center"] == pytest.approx(1.2, abs=0.03)


def test_overlay_writes_images(tmp_path, monkeypatch):
    folder = write_recording(tmp_path / "scene")
    monkeypatch.chdir(tmp_path)
    assert overlay.main([str(folder)]) == 0
    assert len(list((tmp_path / "out_overlay" / "scene").glob("*.png"))) == 3


def test_ground_writes_a_2d_result_without_open3d(tmp_path, monkeypatch, capsys):
    folder = write_recording(tmp_path / "flat", frames=1)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("NO3D", "1")
    assert ground.main([str(folder), "0"]) == 0
    assert (tmp_path / "out_flat_0.png").exists()
    assert "status: OK FRESH" in capsys.readouterr().out


class _FakeRealSense:
    """Just enough of pyrealsense2 for record.main() without a camera."""

    class stream:
        depth, color = "depth", "color"

    class format:
        z16, bgr8 = "z16", "bgr8"

    class _Intrinsics:
        fx = fy = 600.0
        ppx, ppy, width, height, model, coeffs = 320.0, 240.0, 640, 480, "none", [0.0] * 5

    class _Stream:
        def as_video_stream_profile(self):
            return self

        def get_intrinsics(self):
            return _FakeRealSense._Intrinsics()

        def get_extrinsics_to(self, other):
            return type("E", (), {"rotation": [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
                                  "translation": [0.0, -0.024, 0.0]})()

    class _Frame:
        def __init__(self, shape, dtype):
            self._data = np.zeros(shape, dtype)

        def get_data(self):
            return self._data

    class _Frames:
        def get_depth_frame(self):
            return _FakeRealSense._Frame((480, 640), np.uint16)

        def get_color_frame(self):
            return _FakeRealSense._Frame((720, 1280, 3), np.uint8)

    class _Profile:
        def get_stream(self, _):
            return _FakeRealSense._Stream()

        def get_device(self):
            sensor = type("S", (), {"get_depth_scale": lambda self: 0.00025})()
            return type("D", (), {"first_depth_sensor": lambda self: sensor})()

    class pipeline:
        def start(self, _):
            return _FakeRealSense._Profile()

        def wait_for_frames(self):
            return _FakeRealSense._Frames()

        def stop(self):
            pass

    class config:
        def enable_stream(self, *_):
            pass


@pytest.fixture
def fake_camera(monkeypatch):
    monkeypatch.setitem(sys.modules, "pyrealsense2", _FakeRealSense)
    monkeypatch.setattr(record.time, "sleep", lambda _: None)


def test_record_writes_every_frame_pair(tmp_path, fake_camera):
    assert record.main([str(tmp_path / "scene")]) == 0
    assert len(list((tmp_path / "scene" / "depth").glob("*.png"))) == record.N_FRAMES
    assert len(list((tmp_path / "scene" / "color").glob("*.jpg"))) == record.N_FRAMES


def test_record_fails_when_an_image_cannot_be_written(tmp_path, fake_camera, monkeypatch, capsys):
    # Codex branch review P2: a False from cv2.imwrite (disk full) used to count as saved.
    monkeypatch.setattr(record.cv2, "imwrite", lambda path, image: False)
    assert record.main([str(tmp_path / "scene")]) != 0
    out = capsys.readouterr().out
    assert "저장 실패" in out and "000000.png" in out
    assert "저장 완료" not in out
