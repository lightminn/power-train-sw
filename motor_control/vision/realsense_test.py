"""RealSense D435i 작동 점검 — depth + color 프레임 수신 확인.

Jetson 컨테이너에서 RealSense SDK(librealsense + pyrealsense2, Dockerfile.jetson 에
소스 빌드)가 정상인지 빠르게 확인하는 일회성 스크립트.

실행 (Jetson 컨테이너 안):
    python3 motor_control/vision/realsense_test.py

장치/펌웨어, depth·color 해상도, 화면 중앙 거리(m), 유효 depth 픽셀 비율을 출력한다.
"""
import pyrealsense2 as rs
import numpy as np


def main():
    ctx = rs.context()
    devs = ctx.query_devices()
    print(f"감지된 장치 수: {len(devs)}")
    if len(devs) == 0:
        print("장치 0개 — USB3 연결/권한(privileged, /dev) 확인")
        return
    for d in devs:
        print(f"  {d.get_info(rs.camera_info.name)} "
              f"SN={d.get_info(rs.camera_info.serial_number)} "
              f"FW={d.get_info(rs.camera_info.firmware_version)}")

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe.start(cfg)
    try:
        frames = None
        for _ in range(30):          # auto-exposure 안정용 워밍업
            frames = pipe.wait_for_frames()
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        w, h = depth.get_width(), depth.get_height()
        print(f"[depth] {w}x{h}  중앙 거리 = {depth.get_distance(w // 2, h // 2):.3f} m")
        print(f"[color] {color.get_width()}x{color.get_height()} 프레임 수신 OK")
        dimg = np.asanyarray(depth.get_data())
        valid = dimg[dimg > 0]
        if valid.size:
            print(f"[depth] 유효픽셀 {100 * valid.size / dimg.size:.1f}%  "
                  f"min={valid.min()}mm max={valid.max()}mm")
        print("=== RealSense depth+color 작동 OK ===")
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
