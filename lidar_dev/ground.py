"""One-frame ground/obstacle analysis with an optional open3d 3D view.

    python -m lidar_dev.ground data/flat 5        (NO3D=1 skips the 3D window)
"""
from __future__ import annotations

import os
from pathlib import Path
import sys

import cv2
import numpy as np

from lidar_dev.overlay import render
from lidar_dev.recording import load_recording, make_detector, read_depth


def analyse(folder, index: int = 0):
    recording = load_recording(folder)
    depth = read_depth(recording.depth_paths[index])
    result = make_detector(recording).process(depth, 0.0)
    return recording, depth, result


def main(argv=None) -> int:
    args = argv or sys.argv[1:]
    folder, index = Path(args[0]), int(args[1]) if len(args) > 1 else 0
    recording, depth, result = analyse(folder, index)
    valid = depth > 0
    print(f"status: {result.status} {result.reason or result.ground_state}")
    if result.status == "OK":
        print(f"카메라 높이(바닥까지): {result.camera_height_m * 100:.1f} cm, "
              f"기울기: {result.camera_tilt_deg:.1f}도")
        print(f"유효 픽셀 중 바닥 {100 * result.floor_mask.sum() / valid.sum():.1f}%  "
              f"장애물 {100 * result.obstacle_mask.sum() / valid.sum():.1f}%")
        print("zone:", result.zones)
    out = f"out_{folder.name}_{index}.png"
    cv2.imwrite(out, render(depth, result, recording.depth_scale_m))
    print("저장:", out)
    if os.environ.get("NO3D") == "1" or result.status != "OK":
        return 0
    try:
        import open3d as o3d
    except ImportError:
        print("open3d 없음 — 3D 보기는 건너뜀(선택 의존)")
        return 0
    camera = recording.depth_camera
    v, u = np.nonzero(valid)
    z = depth[valid].astype(np.float64) * recording.depth_scale_m
    points = np.stack(((u - camera.cx) * z / camera.fx, (v - camera.cy) * z / camera.fy, z), axis=1)
    colors = np.tile([0.2, 0.5, 1.0], (len(points), 1))
    colors[result.floor_mask[valid]] = [0.6, 0.6, 0.6]
    colors[result.obstacle_mask[valid]] = [1.0, 0.0, 0.0]
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points * np.array([1, -1, -1]))
    cloud.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries(
        [cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(0.3)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
