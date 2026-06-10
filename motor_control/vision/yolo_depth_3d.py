#!/usr/bin/env python3
"""RealSense D435i + YOLOv8 → 검출 객체의 3D 좌표(카메라 기준) 추출.

YOLO 2D 박스 중심의 depth 를 읽어 카메라 좌표계 (X, Y, Z)[m] 와
거리·방위각(az)·고도각(el)으로 변환한다. 모터 명령 없음 — 측정·검증 전용.
정밀 접근(코너 모듈/FSM 목표 거리)과 로봇팔 좌표 핸드오프의 비전 프론트엔드.

실행 (Jetson 컨테이너 안, /workspace 에서):
    python3 motor_control/vision/yolo_depth_3d.py                 # 헤드리스 — 콘솔 출력만
    python3 motor_control/vision/yolo_depth_3d.py --host <노트북IP>  # + 어노테이트 영상 송신
                                                                  # (수신: scripts/recv_stream.sh)

좌표계 (RealSense 카메라 기준): X=오른쪽, Y=아래, Z=전방 [m].
방위각 az = atan2(X, Z) (우측 +), 고도각 el = atan2(-Y, Z) (위쪽 +) [deg].

depth 는 단일 픽셀이 아니라 박스 중앙 패치(가로세로 1/3)의 유효값 중앙값을 사용
— 단일 픽셀은 0(측정불가)이거나 튀는 경우가 많다.
"""
import argparse
import math
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# 같은 폴더의 헬퍼 재사용 (스크립트 직접 실행 시 sys.path[0] = vision/)
from yolo_cuda_stream import resolve_model
from realsense_stream import build_gst_command  # gop-size=30 포함 송신 파이프라인


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", default="yolov8n.pt",
                   help="ultralytics 모델: .pt 또는 .engine")
    p.add_argument("--backend", choices=["pt", "trt"], default="pt",
                   help="pt = PyTorch CUDA, trt = TensorRT FP16")
    p.add_argument("--conf", type=float, default=0.4)
    p.add_argument("--classes", default="",
                   help="검출 클래스 이름 필터, 쉼표구분 (예: bottle,person). 빈값=전체")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--host", default=None,
                   help="지정 시 어노테이트 영상을 UDP H.264 로 송신")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--print-every", type=int, default=15,
                   help="N 프레임마다 검출 좌표 콘솔 출력 (30fps 기준 15=0.5초)")
    p.add_argument("--bench-frames", type=int, default=0,
                   help="0=무한, >0=N프레임 후 종료")
    p.add_argument("--tx-stamp", action="store_true",
                   help="송신 시각 워터마크 표시 — 수신 화면과 비교해 종단 지연 측정용")
    return p.parse_args()


def latest_frames(pipe: rs.pipeline) -> rs.composite_frame:
    """큐에 밀린 프레임을 버리고 가장 최신 프레임셋만 반환.

    처리 루프(YOLO+align)가 카메라 fps 보다 느리면 librealsense 큐에 프레임이
    쌓여 화면이 항상 과거가 된다(고정 랙). 매 루프 최신만 취해 랙을 1프레임
    이내로 유지한다.
    """
    frames = pipe.wait_for_frames()
    while True:
        nxt = pipe.poll_for_frames()
        if nxt.size() == 0:
            return frames
        frames = nxt


def robust_depth_m(depth_img: np.ndarray, depth_scale: float,
                   box: tuple[int, int, int, int]) -> float | None:
    """박스 중앙 1/3 패치의 유효(>0) depth 중앙값 [m]. 유효픽셀 부족 시 None."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    px1, px2 = x1 + w // 3, x2 - w // 3
    py1, py2 = y1 + h // 3, y2 - h // 3
    patch = depth_img[max(py1, 0):py2, max(px1, 0):px2]
    valid = patch[patch > 0]
    if valid.size < 5:
        return None
    return float(np.median(valid)) * depth_scale


def class_ids(model: YOLO, names_csv: str) -> list[int] | None:
    """클래스 이름 CSV → ultralytics class id 리스트. 빈 입력이면 None(전체)."""
    if not names_csv.strip():
        return None
    name_to_id = {v: k for k, v in model.names.items()}
    ids = []
    for n in names_csv.split(","):
        n = n.strip()
        if n not in name_to_id:
            sys.exit(f"ERROR: 모델에 없는 클래스 '{n}' — 가능: {sorted(name_to_id)}")
        ids.append(name_to_id[n])
    return ids


def open_writer(host: str, port: int, w: int, h: int,
                fps: int) -> subprocess.Popen:
    cmd = build_gst_command(host, port, w, h, fps)
    print("[gst-launch]", " ".join(cmd), file=sys.stderr)
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, bufsize=0)
    time.sleep(0.3)
    if proc.poll() is not None:
        sys.exit(f"ERROR: gst-launch 즉시 종료 (rc={proc.returncode})")
    return proc


class AsyncWriter(threading.Thread):
    """인코더 파이프 쓰기를 별도 스레드로 분리 — 검출 루프를 막지 않는다.

    파이프 write 는 gst 가 인코딩을 마칠 때까지 블록되므로 메인 루프에 두면
    캡처+추론과 인코딩이 직렬화돼 fps 가 절반 이하로 떨어진다. 최신 프레임
    1장만 들고 있다가 쓰고, 새 프레임이 오면 못 보낸 이전 프레임은 버린다.
    """

    def __init__(self, proc: subprocess.Popen):
        super().__init__(daemon=True)
        self._proc = proc
        self._cv = threading.Condition()
        self._buf: bytes | None = None
        self.alive = True
        self.start()

    def submit(self, frame_bytes: bytes) -> None:
        with self._cv:
            self._buf = frame_bytes      # 미전송분 교체 (drop)
            self._cv.notify()

    def run(self) -> None:
        while True:
            with self._cv:
                while self._buf is None:
                    self._cv.wait()
                buf, self._buf = self._buf, None
            try:
                self._proc.stdin.write(buf)
            except (BrokenPipeError, OSError):
                self.alive = False
                return

    def close(self) -> None:
        try:
            self._proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        try:
            self._proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._proc.terminate()


def main() -> None:
    a = parse_args()

    model = YOLO(resolve_model(a.model, a.backend, imgsz=(a.height, a.width)))
    cls_filter = class_ids(model, a.classes)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, a.width, a.height, rs.format.z16, a.fps)
    cfg.enable_stream(rs.stream.color, a.width, a.height, rs.format.bgr8, a.fps)
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)  # depth 를 color 시점으로 정렬
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    intr = (profile.get_stream(rs.stream.color)
            .as_video_stream_profile().get_intrinsics())

    writer = (AsyncWriter(open_writer(a.host, a.port, a.width, a.height, a.fps))
              if a.host else None)

    idx = 0
    t_start = time.time()
    fps_win: list[float] = []
    try:
        while True:
            t0 = time.time()
            frames = align.process(latest_frames(pipe))
            depth = frames.get_depth_frame()
            color = frames.get_color_frame()
            if not depth or not color:
                continue
            color_img = np.asanyarray(color.get_data())
            depth_img = np.asanyarray(depth.get_data())

            results = model.predict(color_img, conf=a.conf,
                                    classes=cls_filter, verbose=False)
            dets = []
            for b in results[0].boxes:
                x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
                name = model.names[int(b.cls[0])]
                conf = float(b.conf[0])
                cu, cv_ = (x1 + x2) // 2, (y1 + y2) // 2
                z = robust_depth_m(depth_img, depth_scale, (x1, y1, x2, y2))
                if z is not None:
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(
                        intr, [float(cu), float(cv_)], z)
                    dist = math.sqrt(X * X + Y * Y + Z * Z)
                    az = math.degrees(math.atan2(X, Z))
                    el = math.degrees(math.atan2(-Y, Z))
                    dets.append((name, conf, (X, Y, Z), dist, az, el))
                    lines = [f"{name} d={dist:.2f}m",
                             f"X{X:+.2f} Y{Y:+.2f} Z{Z:+.2f}m",
                             f"az{az:+.1f} el{el:+.1f}"]
                else:
                    dets.append((name, conf, None, None, None, None))
                    lines = [f"{name} no-depth"]
                if writer:
                    cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(color_img, (cu, cv_), 3, (0, 0, 255), -1)
                    for li, txt in enumerate(lines):
                        ty = y1 - 8 - 18 * (len(lines) - 1 - li)
                        if ty < 14:  # 박스가 화면 상단이면 박스 안쪽에
                            ty = y1 + 18 * (li + 1)
                        cv2.putText(color_img, txt, (x1, ty),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                                    (0, 0, 0), 3)
                        cv2.putText(color_img, txt, (x1, ty),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                                    (0, 255, 0), 1)

            if writer:
                if not writer.alive:
                    print("gst-launch 파이프 끊김", file=sys.stderr)
                    break
                if a.tx_stamp:
                    cv2.putText(color_img, f"tx {time.time() % 100:06.2f}",
                                (10, a.height - 12), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 0, 0), 3)
                    cv2.putText(color_img, f"tx {time.time() % 100:06.2f}",
                                (10, a.height - 12), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 255), 1)
                writer.submit(color_img.tobytes())

            fps_win.append(1.0 / max(time.time() - t0, 1e-6))
            idx += 1
            if idx % a.print_every == 0:
                fps = sum(fps_win) / len(fps_win)
                fps_win.clear()
                # 프레임 나이 = 현재시각 - 캡처시각(rs 글로벌 타임스탬프, ms).
                # 수백 ms 면 카메라 큐에 옛 프레임이 쌓여 있다는 뜻 (랙 진단용).
                age_ms = time.time() * 1000.0 - frames.get_timestamp()
                print(f"[{idx:5d}] frame_age={age_ms:6.0f}ms")
                if dets:
                    for name, conf, xyz, dist, az, el in dets:
                        if xyz is not None:
                            print(f"[{idx:5d}] {name:<12} {conf:.2f}  "
                                  f"d={dist:5.2f}m az={az:+6.1f}° el={el:+6.1f}°  "
                                  f"XYZ=({xyz[0]:+.2f},{xyz[1]:+.2f},{xyz[2]:+.2f})m"
                                  f"  fps={fps:.1f}")
                        else:
                            print(f"[{idx:5d}] {name:<12} {conf:.2f}  "
                                  f"depth 측정불가  fps={fps:.1f}")
                else:
                    print(f"[{idx:5d}] (검출 없음)  fps={fps:.1f}")
            if a.bench_frames and idx >= a.bench_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        if writer:
            writer.close()
        elapsed = time.time() - t_start
        print(f"\n[summary] frames={idx} elapsed={elapsed:.1f}s "
              f"avg_fps={idx / elapsed if elapsed > 0 else 0:.1f}")


if __name__ == "__main__":
    main()
