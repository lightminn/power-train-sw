import sys, time
import numpy as np
import pyrealsense2 as rs

name = sys.argv[1] if len(sys.argv) > 1 else "scene"
N_FRAMES, SKIP = 40, 5


def start(with_imu):
    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    if with_imu:
        cfg.enable_stream(rs.stream.accel)
    return pipe, pipe.start(cfg)


try:
    pipe, profile = start(True)
    imu = True
except RuntimeError:
    pipe, profile = start(False)
    imu = False

depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
align = rs.align(rs.stream.color)

for s in (3, 2, 1):
    print(f"{s}초 후 녹화 시작", end="\r")
    time.sleep(1)

depths, colors, accels = [], [], []
accel, count, intr = (0, 0, 0), 0, None
try:
    while len(depths) < N_FRAMES:
        frames = pipe.wait_for_frames()
        if imu:
            for f in frames:
                if (
                    f.is_motion_frame()
                    and f.get_profile().stream_type() == rs.stream.accel
                ):
                    d = f.as_motion_frame().get_motion_data()
                    accel = (d.x, d.y, d.z)
        frames = align.process(frames)
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            continue
        count += 1
        if count % SKIP:
            continue
        if intr is None:
            i = color.profile.as_video_stream_profile().intrinsics
            intr = (i.fx, i.fy, i.ppx, i.ppy)
        depths.append(np.asanyarray(depth.get_data()).copy())
        colors.append(np.asanyarray(color.get_data()).copy())
        accels.append(accel)
        print(f"saved {len(depths)}/{N_FRAMES}   ", end="\r")
finally:
    pipe.stop()

np.savez_compressed(
    f"{name}.npz",
    depth=np.array(depths),
    color=np.array(colors),
    accel=np.array(accels),
    depth_scale=depth_scale,
    intrinsics=np.array(intr),
)
print(f"\n{name}.npz 저장 완료")
