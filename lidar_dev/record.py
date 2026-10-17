"""Record raw (unaligned) L515 depth + colour for the offline tools.

⚠️ The production L515 is owned by the Gateway (single owner).  Stop it first
(`docker compose ... stop powertrain_ros`) or record on a bench PC.

    python -m lidar_dev.record data/obj_50cm      → meta.json, depth/*.png, color/*.jpg
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

N_FRAMES, SKIP, FPS = 40, 3, 30  # keep every 3rd frame → 0.1 s, like the Gateway


def _intrinsics(profile) -> dict:
    i = profile.as_video_stream_profile().get_intrinsics()
    return {"fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
            "width": i.width, "height": i.height,
            "model": str(i.model), "coeffs": list(i.coeffs)}


def main(argv=None) -> int:
    import pyrealsense2 as rs

    folder = Path((argv or sys.argv[1:] or ["data/scene"])[0])
    (folder / "depth").mkdir(parents=True, exist_ok=True)
    (folder / "color").mkdir(parents=True, exist_ok=True)
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, FPS)
    profile = pipeline.start(config)
    try:
        depth_profile = profile.get_stream(rs.stream.depth)
        color_profile = profile.get_stream(rs.stream.color)
        extrinsics = depth_profile.get_extrinsics_to(color_profile)
        meta = {
            "depth_scale_m": profile.get_device().first_depth_sensor().get_depth_scale(),
            "frame_period_s": SKIP / FPS,
            "depth_intrinsics": _intrinsics(depth_profile),
            "color_intrinsics": _intrinsics(color_profile),
            "depth_to_color": {"rotation": list(extrinsics.rotation),
                               "translation_m": list(extrinsics.translation)},
        }
        (folder / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        for seconds in (3, 2, 1):
            print(f"{seconds}초 후 녹화 시작", end="\r", flush=True)
            time.sleep(1)
        saved = count = 0
        while saved < N_FRAMES:
            frames = pipeline.wait_for_frames()
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue
            count += 1
            if count % SKIP:
                continue
            cv2.imwrite(str(folder / "depth" / f"{saved:06d}.png"),
                        np.asanyarray(depth.get_data()).copy())
            cv2.imwrite(str(folder / "color" / f"{saved:06d}.jpg"),
                        np.asanyarray(color.get_data()).copy())
            saved += 1
            print(f"saved {saved}/{N_FRAMES}   ", end="\r", flush=True)
    finally:
        pipeline.stop()
    print(f"\n{folder} 저장 완료")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
