#!/usr/bin/env python3
"""RealSense D435i color + depth → GStreamer UDP H.264 송신 (Jetson → 노트북).

color(좌) | depth 컬러맵(우) 를 가로로 이어 붙여 한 프레임으로 송신한다.
depth 패널에는 화면 중앙 거리(m)를 오버레이한다.

Jetson 컨테이너 안에서 실행하고, 노트북에서는 다음으로 수신:
    scripts/recv_stream.sh [PORT]    (기본 5000)

송신 파이프라인은 yolo_cuda_stream.py 와 동일 (NVENC 불가 → openh264enc 소프트웨어).
"""
import argparse
import subprocess
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def build_gst_command(host: str, port: int, width: int, height: int,
                      fps: int) -> list:
    """fdsrc 로 raw BGR 프레임 받아 H.264 RTP 로 UDP 송신 (yolo_cuda_stream 동일)."""
    return [
        "gst-launch-1.0",
        "fdsrc", "fd=0", "do-timestamp=true",
        "!", "rawvideoparse",
             "format=bgr",
             f"width={width}", f"height={height}",
             f"framerate={fps}/1",
        "!", "videoconvert", "!", "video/x-raw,format=I420",
        "!", "openh264enc", "bitrate=4000000",
        "!", "h264parse", "config-interval=1",
        "!", "rtph264pay", "pt=96", "config-interval=1",
        "!", "udpsink", f"host={host}", f"port={port}",
             "sync=false", "async=false",
    ]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", required=True, help="수신 노트북 IP")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--width", type=int, default=640,
                   help="스트림당 가로 (합성 프레임은 2배)")
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--mode", choices=["sidebyside", "overlay"],
                   default="sidebyside",
                   help="sidebyside=color|depth 가로배치, overlay=depth 를 color 위에 반투명")
    p.add_argument("--align", action="store_true",
                   help="depth 를 color 시점에 정렬 (overlay 는 자동 정렬)")
    p.add_argument("--alpha", type=float, default=0.5,
                   help="overlay 투명도 (0=color만, 1=depth만)")
    p.add_argument("--bench-frames", type=int, default=0,
                   help="0=무한, >0=N프레임 후 종료")
    return p.parse_args()


def main() -> None:
    a = parse_args()

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, a.width, a.height, rs.format.z16, a.fps)
    cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, a.fps)
    pipe.start(cfg)
    overlay = a.mode == "overlay"
    align = rs.align(rs.stream.color) if (a.align or overlay) else None

    comp_w = a.width if overlay else a.width * 2
    cmd = build_gst_command(a.host, a.port, comp_w, a.height, a.fps)
    print("[gst-launch]", " ".join(cmd), file=sys.stderr)
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, bufsize=0)
    time.sleep(0.3)
    if proc.poll() is not None:
        sys.exit(f"ERROR: gst-launch 즉시 종료 (rc={proc.returncode})")

    cx, cy = a.width // 2, a.height // 2
    fps_win: list = []
    idx = 0
    t_start = time.time()
    try:
        while True:
            t0 = time.time()
            frames = pipe.wait_for_frames()
            if align:
                frames = align.process(frames)
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()
            if not depth or not color:
                continue

            color_img = np.asanyarray(color.get_data())
            depth_raw = np.asanyarray(depth.get_data())
            depth_cm = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_raw, alpha=0.03), cv2.COLORMAP_JET)
            dist = depth.get_distance(cx, cy)

            if overlay:
                # depth 를 color 위에 반투명 합성 (유효 depth 픽셀만 — 0 은 color 그대로)
                composite = color_img.copy()
                mask = depth_raw > 0
                blended = cv2.addWeighted(color_img, 1.0 - a.alpha,
                                          depth_cm, a.alpha, 0.0)
                composite[mask] = blended[mask]
                cv2.circle(composite, (cx, cy), 4, (255, 255, 255), 2)
                cv2.putText(composite, f"center {dist:.2f} m", (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                cv2.circle(depth_cm, (cx, cy), 4, (255, 255, 255), 1)
                cv2.putText(depth_cm, f"center {dist:.2f} m", (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(color_img, "COLOR", (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(depth_cm, "DEPTH", (a.width - 110, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                composite = np.hstack((color_img, depth_cm))
            try:
                proc.stdin.write(composite.tobytes())
            except BrokenPipeError:
                print("gst-launch 파이프 끊김", file=sys.stderr)
                break

            dt = max(time.time() - t0, 1e-6)
            fps_win.append(1.0 / dt)
            idx += 1
            if idx % 30 == 0:
                print(f"[{idx:5d}] fps={sum(fps_win)/len(fps_win):4.1f}  "
                      f"center={dist:.2f}m")
                fps_win.clear()
            if a.bench_frames and idx >= a.bench_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        try:
            proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.terminate()
        print(f"\n[summary] frames={idx} elapsed={time.time()-t_start:.1f}s")


if __name__ == "__main__":
    main()
