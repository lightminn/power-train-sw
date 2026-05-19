#!/usr/bin/env python3
"""USB 카메라 → YOLOv8 (CUDA / TensorRT) → axis1 추종 + (옵션) UDP RTP H.264 송신.

5/8 의 yolo_cuda_stream.py 검출 흐름 + 기존 odrive_yolo_object_tracking.py 의
추종 컨트롤러 결합. Jetson 컨테이너 안에서 단일 프로세스 30Hz 폐루프.

비전: 검출 박스 중심 (u,v) → 카메라 X_cam (intrinsics) → motor turns →
      axis1.input_pos = origin + target_turns. ODrive 측 POS_FILTER 가 명령 점프
      smoothing.

영상 송신은 --stream 옵션. 끄면 노트북 의존 없이 단독 동작.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
from ultralytics import YOLO

import odrive
from odrive.enums import AxisState, ControlMode, InputMode

# vision/ 의 helper 재사용 — 폴더 분리 후 sys.path 동적 추가.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "vision"))
from yolo_cuda_stream import (
    build_gst_command,
    open_camera,
    open_writer,
    resolve_model,
)

# fw-v0.5.6 의 odrive.enums 는 plain Enum (IntEnum 아님). fibre serializer 가
# `int(value)` 로 wire serialize 시 plain Enum 은 coerce 실패 →
# 미리 `.value` 로 int 만 뽑아두고 wire I/O 에 사용.
AXIS_IDLE = AxisState.IDLE.value
AXIS_CLOSED_LOOP = AxisState.CLOSED_LOOP_CONTROL.value
AXIS_FULL_CALIB = AxisState.FULL_CALIBRATION_SEQUENCE.value
CTRL_POSITION = ControlMode.POSITION_CONTROL.value
INPUT_POS_FILTER = InputMode.POS_FILTER.value


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    # vision
    p.add_argument("--camera", default="/dev/video0",
                   help="V4L2 device (default: /dev/video0)")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--model", default="yolov8n.pt",
                   help="ultralytics model: .pt or .engine")
    p.add_argument("--backend", choices=["pt", "trt"], default="trt")
    p.add_argument("--target", default="bottle",
                   help="COCO class name to track (default: bottle)")
    p.add_argument("--conf", type=float, default=0.4)
    # control
    p.add_argument("--scale", type=float, default=5.0,
                   help="X_cam(m) → motor turns 변환 계수")
    p.add_argument("--max-turns", type=float, default=2.0,
                   help="motor 측 안전 한계 (출력단 ÷5)")
    p.add_argument("--deadzone", type=float, default=0.05,
                   help="명령 변화 이 미만이면 무시 (turns)")
    p.add_argument("--input-filter", type=float, default=2.0,
                   help="ODrive POS_FILTER input_filter_bandwidth (Hz)")
    p.add_argument("--no-motor", action="store_true",
                   help="ODrive 명령 송신 안 함 (코드 경로만 검증)")
    # streaming (optional)
    p.add_argument("--stream", action="store_true",
                   help="annotated frame 을 GStreamer UDP RTP H.264 로 송신")
    p.add_argument("--host", default="127.0.0.1",
                   help="--stream 켜진 경우 receiver IP")
    p.add_argument("--port", type=int, default=5000)
    # bench
    p.add_argument("--bench-frames", type=int, default=0,
                   help="0=무한, >0=N 프레임 후 종료")
    return p.parse_args()


def coco_class_id(model: YOLO, name: str) -> int:
    """ultralytics 모델의 names dict 에서 클래스명 → ID 역검색."""
    for cid, cname in model.names.items():
        if cname == name:
            return int(cid)
    sys.exit(f"ERROR: class '{name}' not in model.names")


def setup_axis(ax, input_filter_hz: float) -> float:
    """폐루프 진입 + 게인 세팅. 원점 turns 반환."""
    if ax.motor.is_calibrated and ax.encoder.is_ready:
        print("[odrive] axis1 already calibrated — skipping")
    else:
        print("[odrive] running FULL_CALIBRATION_SEQUENCE...")
        ax.clear_errors()
        ax.requested_state = AXIS_FULL_CALIB
        while ax.current_state != AXIS_IDLE:
            time.sleep(0.1)
        if not (ax.motor.is_calibrated and ax.encoder.is_ready):
            sys.exit(f"[odrive] calibration failed. axis={ax.error:#x} "
                     f"motor={ax.motor.error:#x} enc={ax.encoder.error:#x}")
        print("[odrive] calibration OK")

    # 기존 에러 초기화 (v0.5.6 권장 방식)
    ax.clear_errors()

    # 🚨 하드코딩된 쓰레기 게인 설정 코드들 전부 삭제됨! (보드 내부 저장값 사용)
    ax.controller.config.control_mode = CTRL_POSITION
    ax.controller.config.input_mode = INPUT_POS_FILTER
    ax.controller.config.input_filter_bandwidth = input_filter_hz

    # 🚨 [가장 중요한 수정] 모터에 힘을 넣기 '전'에, 현재 위치를 목표 위치로 일치시킴!
    origin = ax.encoder.pos_estimate
    ax.controller.input_pos = origin

    print("[odrive] entering CLOSED_LOOP_CONTROL...")
    ax.requested_state = AXIS_CLOSED_LOOP
    time.sleep(0.5)

    if ax.current_state != AXIS_CLOSED_LOOP:
        sys.exit(f"[odrive] closed-loop entry failed. motor={ax.motor.error:#x}")

    print(f"[odrive] closed-loop OK. origin={origin:.3f} turns")
    return origin


def main() -> None:
    args = parse_args()

    # vision setup
    model_path = resolve_model(
        args.model, args.backend, imgsz=(args.height, args.width))
    # task="detect" 명시 — .engine 파일명에서 ultralytics 가 task 추정 못 해서 경고.
    model = YOLO(model_path, task="detect")
    target_id = coco_class_id(model, args.target)
    print(f"[vision] target='{args.target}' (class id {target_id})")

    cap = open_camera(args.camera, args.width, args.height, args.fps)

    # 카메라 intrinsics (odrive_yolo_object_tracking.py 와 동일 가정)
    fx, fy = 500.0, 500.0
    cx0, cy0 = args.width / 2.0, args.height / 2.0

    # streaming (optional)
    out_proc = None
    if args.stream:
        out_proc = open_writer(args.host, args.port,
                               args.width, args.height, args.fps)

    # ODrive
    ax = None
    origin = 0.0
    if not args.no_motor:
        print("[odrive] searching...")
        drv = odrive.find_any(timeout=10)
        ax = drv.axis1
        print(f"[odrive] sn={drv.serial_number} vbus={drv.vbus_voltage:.2f}V")
        origin = setup_axis(ax, args.input_filter)

    last_target_pos = origin
    frame_idx = 0
    inf_window: list[float] = []
    fps_window: list[float] = []
    t_start = time.time()

    try:
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if not ret:
                print("[camera] read failed", file=sys.stderr)
                break

            results = model.predict(
                frame, classes=[target_id], conf=args.conf, verbose=False)
            r = results[0]
            annotated = r.plot()

            target_turns = None
            if len(r.boxes) > 0:
                box = r.boxes[0]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0
                Z = 1.0
                X_cam = (u - cx0) * Z / fx

                target_turns = max(-args.max_turns,
                                   min(args.max_turns, X_cam * args.scale))
                target_pos = origin + target_turns

                if ax is not None and abs(target_pos - last_target_pos) > args.deadzone:
                    ax.controller.input_pos = target_pos
                    last_target_pos = target_pos

            # ODrive 트립 모니터
            if ax is not None and ax.error != 0:
                print(f"[odrive] tripped: axis={ax.error:#x} "
                      f"motor={ax.motor.error:#x}", file=sys.stderr)
                break
            if ax is not None and ax.current_state != AXIS_CLOSED_LOOP:
                print(f"[odrive] left closed-loop: state={ax.current_state}",
                      file=sys.stderr)
                break

            # 영상 송신
            if out_proc is not None:
                try:
                    out_proc.stdin.write(annotated.tobytes())
                except BrokenPipeError:
                    print("[gst] pipe broken", file=sys.stderr)
                    break

            t1 = time.time()
            dt = max(t1 - t0, 1e-6)
            inf_window.append(float(r.speed.get("inference", 0.0)))
            fps_window.append(1.0 / dt)
            frame_idx += 1
            if frame_idx % 30 == 0:
                n = len(fps_window)
                cur = (ax.encoder.pos_estimate - origin) if ax is not None else 0.0
                tt = f"{target_turns:+.2f}" if target_turns is not None else "  -- "
                print(f"[{frame_idx:5d}] fps={sum(fps_window)/n:5.1f}  "
                      f"infer={sum(inf_window)/n:5.1f}ms  "
                      f"tgt={tt}  cur={cur:+.2f}")
                fps_window.clear()
                inf_window.clear()

            if args.bench_frames and frame_idx >= args.bench_frames:
                break
    except KeyboardInterrupt:
        print("\n[main] Ctrl-C")
    finally:
        elapsed = time.time() - t_start
        avg = frame_idx / elapsed if elapsed > 0 else 0.0
        print(f"\n[summary] backend={args.backend} target={args.target} "
              f"size={args.width}x{args.height} frames={frame_idx} "
              f"elapsed={elapsed:.1f}s avg_fps={avg:.1f}")

        if ax is not None:
            print("[odrive] returning to origin + IDLE...")
            try:
                ax.controller.input_pos = origin
                time.sleep(2)
            except Exception as e:
                print(f"[odrive] origin return error: {e}", file=sys.stderr)
            ax.requested_state = AXIS_IDLE

        cap.release()
        if out_proc is not None:
            try:
                out_proc.stdin.close()
            except (BrokenPipeError, OSError):
                pass
            try:
                out_proc.wait(timeout=5)
            except Exception:
                out_proc.terminate()
                out_proc.wait(timeout=5)


if __name__ == "__main__":
    main()
