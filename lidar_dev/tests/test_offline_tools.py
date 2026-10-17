"""lidar_dev offline tools on a synthetic recording (conda: numpy + cv2)."""
import json

import cv2
import pytest

from lidar_dev import ground, overlay, scan_terrain
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
