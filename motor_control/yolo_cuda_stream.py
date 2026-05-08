#!/usr/bin/env python3
"""USB 카메라 → YOLOv8 (CUDA / TensorRT) → GStreamer UDP H.264 송신.

Jetson 컨테이너 안에서 실행. 호스트(노트북)에서는 scripts/recv_stream.sh 로 수신.
"""
import argparse
import sys
import time

import cv2
from ultralytics import YOLO


def build_gst_pipeline(host: str, port: int, width: int, height: int, fps: int) -> str:
    """H.264 인코딩 + UDP RTP 송신 pipeline.

    Jetson NVENC(`nvv4l2h264enc`)는 dustynv 컨테이너의 GStreamer 1.20+ 와 NVIDIA
    L4T plugin (1.14 ABI) 사이 element registration 호환 이슈로 동작 불가. 일단
    소프트웨어 인코더 `openh264enc` 사용. 720p/30fps 정도는 ARM A78AE 6코어에서 OK.
    """
    return (
        f"appsrc ! "
        f"video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
        f"videoconvert ! video/x-raw,format=I420 ! "
        f"openh264enc bitrate=4000000 ! "
        f"h264parse config-interval=1 ! "
        f"rtph264pay pt=96 config-interval=1 ! "
        f"udpsink host={host} port={port} sync=false async=false"
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--camera", default="/dev/video0",
                   help="V4L2 device (default: /dev/video0)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--model", default="yolov8n.pt",
                   help="ultralytics model: .pt or .engine path")
    p.add_argument("--backend", choices=["pt", "trt"], default="pt",
                   help="pt = PyTorch CUDA, trt = TensorRT FP16")
    p.add_argument("--host", required=True, help="receiver IP (노트북)")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--conf", type=float, default=0.4)
    p.add_argument("--bench-frames", type=int, default=0,
                   help="0=무한, >0=N프레임 후 종료 (벤치 모드)")
    return p.parse_args()


def resolve_model(path: str, backend: str, imgsz: tuple[int, int]) -> str:
    """backend=trt 인 경우 .engine 이 없으면 export 수행. .engine 경로 반환."""
    if backend == "pt":
        return path
    if path.endswith(".engine"):
        return path
    print(f"[info] exporting TensorRT engine from {path} (FP16, imgsz={imgsz})...")
    base = YOLO(path)
    engine = base.export(format="engine", half=True, imgsz=imgsz)
    print(f"[info] engine: {engine}")
    return engine


def open_camera(dev: str, w: int, h: int, fps: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if not cap.isOpened():
        sys.exit(f"ERROR: cannot open {dev}")
    return cap


def open_writer(host: str, port: int, w: int, h: int, fps: int) -> cv2.VideoWriter:
    pipeline = build_gst_pipeline(host, port, w, h, fps)
    out = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, fps, (w, h))
    if not out.isOpened():
        sys.exit("ERROR: GStreamer pipeline 열기 실패")
    return out


def main() -> None:
    args = parse_args()
    model_path = resolve_model(
        args.model, args.backend, imgsz=(args.height, args.width))
    model = YOLO(model_path)

    cap = open_camera(args.camera, args.width, args.height, args.fps)
    out = open_writer(args.host, args.port, args.width, args.height, args.fps)

    inf_window: list[float] = []
    e2e_window: list[float] = []
    fps_window: list[float] = []
    frame_idx = 0
    t_start = time.time()
    try:
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if not ret:
                print("camera read failed", file=sys.stderr)
                break
            results = model.predict(frame, conf=args.conf, verbose=False)
            annotated = results[0].plot()
            out.write(annotated)
            t1 = time.time()
            dt = max(t1 - t0, 1e-6)
            inf_window.append(float(results[0].speed.get("inference", 0.0)))
            e2e_window.append(dt * 1000.0)
            fps_window.append(1.0 / dt)
            frame_idx += 1
            if frame_idx % 30 == 0:
                n = len(fps_window)
                print(f"[{frame_idx:5d}] "
                      f"fps={sum(fps_window)/n:5.1f}  "
                      f"infer={sum(inf_window)/n:5.1f}ms  "
                      f"e2e={sum(e2e_window)/n:5.1f}ms")
                fps_window.clear(); inf_window.clear(); e2e_window.clear()
            if args.bench_frames and frame_idx >= args.bench_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - t_start
        avg = frame_idx / elapsed if elapsed > 0 else 0.0
        print(f"\n[summary] backend={args.backend} model={args.model} "
              f"size={args.width}x{args.height} "
              f"frames={frame_idx} elapsed={elapsed:.1f}s avg_fps={avg:.1f}")
        cap.release()
        out.release()


if __name__ == "__main__":
    main()
