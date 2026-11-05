"""3D point-cloud view of one recorded depth frame (needs optional open3d).

    python -m lidar_dev.view_depth ~/Downloads/L515/20260930_212517 5
"""
from __future__ import annotations

from pathlib import Path
import sys

import cv2
import numpy as np

from lidar_dev.recording import load_recording, read_depth


def main(argv=None) -> int:
    args = argv or sys.argv[1:]
    recording = load_recording(Path(args[0]))
    index = int(args[1]) if len(args) > 1 else 0
    try:
        import open3d as o3d
    except ImportError:
        print("open3d 가 없습니다 — `pip install open3d` 후 다시 실행하세요(선택 의존)")
        return 1
    depth = read_depth(recording.depth_paths[index])
    camera = recording.depth_camera
    valid = depth > 0
    v, u = np.nonzero(valid)
    z = depth[valid].astype(np.float64) * recording.depth_scale_m
    points = np.stack(((u - camera.cx) * z / camera.fx, (v - camera.cy) * z / camera.fy, z), axis=1)
    print(f"유효 픽셀 {100 * valid.mean():.1f}%  거리 {z.min():.2f} ~ {z.max():.2f} m  "
          f"중앙값 {np.median(z):.2f} m")
    scaled = ((z - z.min()) / (np.ptp(z) + 1e-6) * 255).astype(np.uint8)
    colors = cv2.applyColorMap(scaled.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)[:, ::-1] / 255.0
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points * np.array([1, -1, -1]))
    cloud.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries(
        [cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
