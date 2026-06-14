#!/usr/bin/env python3
"""USB 카메라 → YOLOv8 (CUDA / TensorRT) → GStreamer H.264/SRT 송신.

Jetson 컨테이너 안에서 실행. 호스트(노트북)에서는 scripts/recv_stream.sh 로 수신.
송신은 SRT listener 라 수신측 IP 불필요. 인코더는 SW 전용 — Orin Nano 에는
NVENC 하드웨어가 없다 (상세: gst_stream.py).
"""
import argparse
import os
import subprocess
import sys
import time

import cv2
from ultralytics import YOLO

# 같은 폴더의 공용 송신 파이프라인 (스크립트 직접 실행 시 sys.path[0] = vision/)
from gst_stream import ENCODERS, build_gst_command


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--camera", default="/dev/video0",
                   help="V4L2 device (default: /dev/video0)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--model", default="yolo26n.pt",
                   help="ultralytics model: .pt or .engine path (팀 결정: YOLO26)")
    p.add_argument("--backend", choices=["pt", "trt"], default="pt",
                   help="pt = PyTorch CUDA, trt = TensorRT FP16")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--encoder", choices=ENCODERS, default="x264",
                   help="x264 권장 (plugins-ugly 필요). 구이미지는 openh264")
    p.add_argument("--conf", type=float, default=0.4)
    p.add_argument("--bench-frames", type=int, default=0,
                   help="0=무한, >0=N프레임 후 종료 (벤치 모드)")
    return p.parse_args()


def resolve_model(path: str, backend: str, imgsz: tuple[int, int]) -> str:
    """backend=trt 인 경우 입력 사이즈별 .engine 캐시 (없으면 export). 경로 반환.

    ultralytics 의 model.export() 는 자체 캐싱 안 함 — 매번 호출하면 매번 빌드 (5-10분).
    여기서 입력 사이즈를 파일명에 박아 캐시.
    """
    if backend == "pt":
        return path
    if path.endswith(".engine"):
        return path
    # imgsz=(H, W) 를 파일명에 반영. ultralytics 가 32 multiple 로 올림하니
    # 표시도 그 기준으로.
    h32 = ((imgsz[0] + 31) // 32) * 32
    w32 = ((imgsz[1] + 31) // 32) * 32
    base_name = os.path.splitext(path)[0]
    cached = f"{base_name}_{h32}x{w32}_fp16.engine"
    if os.path.exists(cached):
        print(f"[info] reusing cached engine: {cached}")
        return cached
    print(f"[info] exporting TensorRT engine from {path} (FP16, imgsz={imgsz})...")
    base = YOLO(path)
    engine = base.export(format="engine", half=True, imgsz=imgsz)
    # ultralytics 가 만든 기본 yolov8n.engine 을 사이즈별 이름으로 rename.
    if engine != cached and os.path.exists(engine):
        os.rename(engine, cached)
        engine = cached
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


def open_writer(port: int, w: int, h: int, fps: int,
                encoder: str) -> subprocess.Popen:
    cmd = build_gst_command(port, w, h, fps, encoder=encoder)
    print("[gst-launch]", " ".join(cmd), file=sys.stderr)
    # stderr 는 부모 콘솔로 그대로 노출 (디버깅), bufsize=0 으로 즉시 flush.
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, bufsize=0)
    time.sleep(0.3)
    if proc.poll() is not None:
        sys.exit(f"ERROR: gst-launch 즉시 종료 (rc={proc.returncode})")
    return proc


def main() -> None:
    args = parse_args()
    model_path = resolve_model(
        args.model, args.backend, imgsz=(args.height, args.width))
    model = YOLO(model_path)

    cap = open_camera(args.camera, args.width, args.height, args.fps)
    out_proc = open_writer(args.port, args.width, args.height, args.fps,
                           args.encoder)

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
            try:
                out_proc.stdin.write(annotated.tobytes())
            except BrokenPipeError:
                print("gst-launch 파이프 끊김", file=sys.stderr)
                break
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
        try:
            out_proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        try:
            out_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            out_proc.terminate()
            out_proc.wait(timeout=5)


if __name__ == "__main__":
    main()
