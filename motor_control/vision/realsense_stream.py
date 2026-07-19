#!/usr/bin/env python3
"""Powertrain L515 color + depth → GStreamer H.264/SRT 송신 (Jetson → 노트북).

color(좌) | depth 컬러맵(우) 를 가로로 이어 붙여 한 프레임으로 송신한다.
depth 패널에는 화면 중앙 거리(m)를 오버레이한다.

depth 점검 전용 진단 도구다 — depth JET 컬러맵은 고주파라 H.264 압축 효율이
최악이고 sidebyside 는 픽셀도 2배여서, 원격주행/상시 모니터링 영상으로는
쓰지 말 것 (그 용도는 yolo_depth_3d.py 의 color 단독 + 좌표 분리 채널).

Jetson 컨테이너 안에서 실행하고, 노트북에서는 다음으로 수신:
    scripts/recv_stream.sh [PORT] [JETSON_HOST]    (기본 5000 jetson-orin.local)

송신은 SRT listener — 노트북이 접속해 오므로 수신측 IP 가 필요 없다.
파이프라인 상세·인코더 선택은 gst_stream.py 참고.
"""
import argparse
import subprocess
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

# 같은 폴더의 공용 송신 파이프라인 (스크립트 직접 실행 시 sys.path[0] = vision/)
from gst_stream import ENCODERS, build_gst_command
from realsense_l515 import start_l515_pipeline


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--encoder", choices=ENCODERS, default="x264",
                   help="x264 권장 (plugins-ugly 필요). 구이미지는 openh264")
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
    start_l515_pipeline(pipe, cfg, rs)
    overlay = a.mode == "overlay"
    align = rs.align(rs.stream.color) if (a.align or overlay) else None

    comp_w = a.width if overlay else a.width * 2
    cmd = build_gst_command(a.port, comp_w, a.height, a.fps, encoder=a.encoder)
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
