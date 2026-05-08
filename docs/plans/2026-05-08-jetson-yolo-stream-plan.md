# Jetson Orin Nano YOLO 검출 + 실시간 영상 스트리밍 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Jetson Orin Nano에서 USB 카메라 영상을 받아 CUDA YOLOv8n으로 검출, 검출 박스가 오버레이된 H.264 영상을 노트북에 UDP RTP로 실시간 송신. PyTorch FP32 vs TensorRT FP16 성능 비교.

**Architecture:** dustynv/ultralytics 컨테이너를 베이스로 ODrive/pygame만 추가. 신규 스크립트 `motor_control/yolo_cuda_stream.py`가 V4L2 캡처 → ultralytics 추론 → cv2.VideoWriter(GStreamer pipeline) 송신 + FPS 계측. 노트북 측은 `gst-launch-1.0` 한 줄 헬퍼 스크립트로 수신.

**Tech Stack:** Python 3.10, PyTorch 2.x (CUDA), Ultralytics YOLOv8, TensorRT 10.3, OpenCV(CUDA build), GStreamer 1.20 + nvv4l2h264enc (NVENC).

**환경 분리:** Task 1-3, 6-8, 12는 **호스트(Arch)**에서, Task 4-5, 9-11은 **Jetson 컨테이너 안**에서 진행.

---

## File Structure

| 파일 | 역할 | 작업 위치 |
|---|---|---|
| `docker/Dockerfile.jetson` | 베이스 이미지 변경, 의존성 정리 (수정) | 호스트 git |
| `motor_control/yolo_cuda_stream.py` | 비전 스크립트 본체 (신규) | 호스트 git |
| `scripts/recv_stream.sh` | 노트북 측 GStreamer 수신 헬퍼 (신규) | 호스트 git |
| `README.md` | motor_control 표에 1줄 추가 (수정) | 호스트 git |
| `docs/plans/2026-05-08-jetson-yolo-stream-plan.md` | 본 문서 | 호스트 git |

---

## Task 1: Dockerfile.jetson 베이스 이미지 변경

**Files:**
- Modify: `docker/Dockerfile.jetson` (전면 교체)

- [ ] **Step 1: 새 Dockerfile.jetson 작성**

`docker/Dockerfile.jetson`을 다음으로 교체:

```dockerfile
# Power Train SW — Jetson Orin Nano 배포용 (ARM64, JetPack 6 / L4T R36.x)
#
# 베이스: dustynv/ultralytics — l4t-pytorch + ultralytics + OpenCV(CUDA) + TensorRT
#   * https://github.com/dusty-nv/jetson-containers
# OpenVINO 는 Intel 전용이므로 제외. 젯슨에선 PyTorch+CUDA / TensorRT 경로 사용.

FROM dustynv/ultralytics:r36.4.0

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

# ODrive USB, DualSense (SDL2/joystick), 카메라(V4L2), GStreamer 도구만 추가.
# 베이스에 이미 설치된 것: PyTorch, ultralytics, opencv(CUDA), TensorRT, GStreamer 코어.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libusb-1.0-0 \
        libsdl2-2.0-0 \
        v4l-utils \
        joystick \
        usbutils \
        udev \
    && rm -rf /var/lib/apt/lists/*

# 모터 트랙용 — 비전과 무관하지만 컨테이너 일관성을 위해 보존.
RUN pip3 install --no-cache-dir \
        odrive \
        pygame

WORKDIR /workspace
CMD ["bash"]
```

- [ ] **Step 2: Commit**

```bash
cd ~/Defence_Robot
git add docker/Dockerfile.jetson
git commit -m "feat(docker): Jetson 이미지 베이스를 dustynv/ultralytics:r36.4.0로 변경

OpenCV CUDA + TensorRT + ultralytics 사전 통합으로 빌드 시간 단축 + 추론 성능 이득.
ODrive/pygame만 추가, 나머지는 베이스 의존."
```

---

## Task 2: 신규 스크립트 motor_control/yolo_cuda_stream.py

**Files:**
- Create: `motor_control/yolo_cuda_stream.py`

- [ ] **Step 1: 파일 생성 (전체 코드)**

```python
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
    """Jetson NVENC H.264 인코딩 + UDP RTP 송신 pipeline."""
    return (
        f"appsrc ! "
        f"video/x-raw,format=BGR,width={width},height={height},framerate={fps}/1 ! "
        f"videoconvert ! video/x-raw,format=I420 ! "
        f"nvvidconv ! "
        f"nvv4l2h264enc maxperf-enable=1 bitrate=4000000 "
        f"insert-sps-pps=1 idrinterval=15 ! "
        f"h264parse ! rtph264pay pt=96 config-interval=1 ! "
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
```

- [ ] **Step 2: 실행 권한 + commit**

```bash
chmod +x motor_control/yolo_cuda_stream.py
git add motor_control/yolo_cuda_stream.py
git commit -m "feat(motor_control): yolo_cuda_stream.py — YOLOv8 + GStreamer NVENC 송신

CUDA YOLOv8n 추론 후 검출 박스 오버레이된 영상을 UDP RTP H.264로 노트북에 송신.
PyTorch FP32 / TensorRT FP16 백엔드 선택, FPS·inference·end-to-end latency 30프레임 평균 출력."
```

---

## Task 3: 노트북 수신 헬퍼 + README 업데이트

**Files:**
- Create: `scripts/recv_stream.sh`
- Modify: `README.md` (motor_control 표에 1줄)

- [ ] **Step 1: scripts/recv_stream.sh 작성**

```bash
mkdir -p ~/Defence_Robot/scripts
```

```bash
#!/usr/bin/env bash
# 노트북 측: Jetson 에서 보낸 UDP RTP H.264 영상을 수신해 화면에 표시.
# 사용: ./recv_stream.sh [PORT]   (default 5000)

PORT="${1:-5000}"

exec gst-launch-1.0 -v \
    udpsrc port="$PORT" \
        caps='application/x-rtp,encoding-name=H264,payload=96' \
    ! rtph264depay ! avdec_h264 ! videoconvert \
    ! autovideosink sync=false
```

저장 후:
```bash
chmod +x scripts/recv_stream.sh
```

- [ ] **Step 2: README.md motor_control 트랙 B 표에 1줄 추가**

기존 표(`#### 환경 / 검출 단독`)에 다음 줄 추가:

```markdown
| `yolo_cuda_stream.py` | YOLOv8(CUDA/TensorRT) 검출 + GStreamer NVENC H.264 송신. 모터 명령 없음. Jetson 전용 |
```

- [ ] **Step 3: Commit**

```bash
git add scripts/recv_stream.sh README.md
git commit -m "feat(scripts): recv_stream.sh + README — 노트북 측 영상 수신 헬퍼"
```

---

## Task 4: spec + plan 문서 commit + 호스트 push

**Files:**
- Stage: `docs/specs/2026-05-08-jetson-yolo-stream-design.md` (이미 작성됨)
- Stage: `docs/plans/2026-05-08-jetson-yolo-stream-plan.md` (본 문서)

- [ ] **Step 1: 문서 commit**

```bash
cd ~/Defence_Robot
git add docs/specs/2026-05-08-jetson-yolo-stream-design.md \
        docs/plans/2026-05-08-jetson-yolo-stream-plan.md
git commit -m "docs: Jetson YOLO 스트리밍 spec + implementation plan"
```

- [ ] **Step 2: GitHub origin에 push**

```bash
git push origin <현재-브랜치-이름>
```

(브랜치 이름 확인: `git rev-parse --abbrev-ref HEAD`)

- [ ] **Step 3: push 완료 확인**

```bash
git log origin/<브랜치> --oneline -3
```

기대 출력: 최근 3개 commit이 origin에도 보임.

---

## Task 5: Jetson에 코드 가져오기

**환경: Jetson 안 (호스트(노트북)에서 SSH로 접속)**

```bash
ssh zetin@jetson-orin.local   # mDNS, 또는 ssh zetin@<IP>
```

- [ ] **Step 1: git clone (최초 1회)**

```bash
cd ~
git clone https://github.com/lightminn/power-train-sw.git Defence_Robot
cd Defence_Robot
```

(이미 클론 돼있다면 `cd ~/Defence_Robot && git pull` 만)

- [ ] **Step 2: 가져온 파일 확인**

```bash
ls docker/Dockerfile.jetson motor_control/yolo_cuda_stream.py scripts/recv_stream.sh
```

기대 출력: 세 파일 모두 존재.

---

## Task 6: Jetson 컨테이너 빌드

**환경: Jetson**

- [ ] **Step 1: 빌드**

```bash
cd ~/Defence_Robot
docker compose -f docker/docker-compose.jetson.yml build 2>&1 | tail -20
```

기대 동작: `dustynv/ultralytics:r36.4.0` 베이스 이미지 약 5GB 다운, 추가 패키지 설치 후 `powertrain-sw:jetson` 이미지 생성. 첫 실행 5-15분.

- [ ] **Step 2: 이미지 확인**

```bash
docker images | grep powertrain-sw
```

기대 출력: `powertrain-sw:jetson` 항목 1개.

---

## Task 7: 컨테이너 진입 + 의존성 검증

**환경: Jetson → 컨테이너 안**

- [ ] **Step 1: 컨테이너 띄우고 진입**

```bash
cd ~/Defence_Robot
docker compose -f docker/docker-compose.jetson.yml up -d
docker compose -f docker/docker-compose.jetson.yml exec powertrain bash
```

이후 step은 모두 컨테이너 내부 프롬프트에서.

- [ ] **Step 2: Python 의존성 import 검증**

```bash
python3 - <<'PY'
import torch
print("torch", torch.__version__, "cuda:", torch.cuda.is_available(),
      "device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else None)
import cv2
print("cv2", cv2.__version__, "cuda_devices:", cv2.cuda.getCudaEnabledDeviceCount())
import ultralytics
print("ultralytics", ultralytics.__version__)
import odrive, pygame
print("odrive", odrive.__version__, "pygame", pygame.version.ver)
PY
```

기대: 모두 성공, `cuda: True`, `cuda_devices: > 0`.

- [ ] **Step 3: NVENC + 카메라 인식**

```bash
gst-inspect-1.0 nvv4l2h264enc | head -3
v4l2-ctl --list-devices
```

기대: `nvv4l2h264enc` 플러그인 정보 출력, `/dev/video0` (또는 다른 USB 카메라) 등장.

---

## Task 8: PyTorch 백엔드로 첫 동작 검증

**환경: 두 터미널 — 노트북(수신) + Jetson 컨테이너(송신)**

- [ ] **Step 1: 노트북 수신측 시작**

호스트 노트북에서:

```bash
cd ~/Defence_Robot
./scripts/recv_stream.sh 5000
```

(GStreamer 미설치면: `sudo pacman -S --needed gstreamer gst-plugins-{base,good,bad,ugly} gst-libav` 후 재시도)

기대: 빈 GStreamer 창 또는 "waiting for buffer" 상태.

- [ ] **Step 2: 노트북 IP 확인**

호스트(노트북) 별도 터미널:

```bash
ip -4 addr show | grep -E 'inet ' | grep -v 127
```

송신 대상 IP를 메모. (예: `192.168.1.X`)

- [ ] **Step 3: Jetson에서 송신 시작 (PyTorch)**

Jetson 컨테이너 안:

```bash
cd /workspace
python3 motor_control/yolo_cuda_stream.py \
    --backend pt --model yolov8n.pt \
    --host <노트북-IP> --port 5000 \
    --width 1280 --height 720 --fps 30 \
    --bench-frames 300
```

(yolov8n.pt 자동 다운로드, 모델 로드 후 첫 추론까지 5-10초 정도 소요)

기대 출력 (30프레임마다):
```
[   30] fps= XX.X  infer= XX.Xms  e2e= XX.Xms
[   60] fps= XX.X  infer= XX.Xms  e2e= XX.Xms
...
[summary] backend=pt model=yolov8n.pt size=1280x720 frames=300 elapsed=...s avg_fps=...
```

노트북 화면에 영상 + 검출 박스 표시되어야 함.

- [ ] **Step 4: 수치 기록**

`avg_fps`, `infer`(평균), `e2e`(평균) 값을 plan 문서 끝의 결과 표 (Task 11)에 기록.

---

## Task 9: TensorRT 백엔드 추가 검증

**환경: Jetson 컨테이너**

- [ ] **Step 1: TensorRT engine export + 송신**

```bash
python3 motor_control/yolo_cuda_stream.py \
    --backend trt --model yolov8n.pt \
    --host <노트북-IP> --port 5000 \
    --width 1280 --height 720 --fps 30 \
    --bench-frames 300
```

기대: 첫 실행 시 `[info] exporting TensorRT engine ...` 5-10분 소요 후 `.engine` 파일 생성. 그 다음 추론. 두 번째 실행부터는 export 단계 스킵 (캐시된 .engine 자동 사용 — 단 같은 입력 사이즈여야 함).

- [ ] **Step 2: 결과 비교 + 기록**

PyTorch 결과(Task 8)와 비교. TensorRT 쪽 fps 가 1.5-3배 정도 높아야 정상.

`avg_fps`, `infer`, `e2e` 를 결과 표(Task 11)에 기록.

---

## Task 10: 해상도별 측정

**환경: Jetson 컨테이너**

- [ ] **Step 1: 640x480 측정**

```bash
python3 motor_control/yolo_cuda_stream.py \
    --backend pt --host <노트북-IP> --width 640 --height 480 --fps 30 --bench-frames 300

python3 motor_control/yolo_cuda_stream.py \
    --backend trt --host <노트북-IP> --width 640 --height 480 --fps 30 --bench-frames 300
```

(TRT는 입력 사이즈 바뀌면 engine 재export — 다시 5-10분)

- [ ] **Step 2: 1920x1080 측정**

```bash
python3 motor_control/yolo_cuda_stream.py \
    --backend pt --host <노트북-IP> --width 1920 --height 1080 --fps 30 --bench-frames 300

python3 motor_control/yolo_cuda_stream.py \
    --backend trt --host <노트북-IP> --width 1920 --height 1080 --fps 30 --bench-frames 300
```

- [ ] **Step 3: 카메라가 해당 해상도 못 받으면 표에 N/A로 기록**

USB 카메라가 1920x1080 미지원이면 `cap.read()`가 실패하거나 다른 해상도로 fallback. 그 경우 그대로 N/A.

---

## Task 11: 결과 표 작성 + 문서 commit

**Files:**
- Modify: `docs/plans/2026-05-08-jetson-yolo-stream-plan.md` (본 문서 끝에 결과 표 추가)

- [ ] **Step 1: 결과 표를 본 문서 맨 끝에 append**

다음 표를 본 문서 끝에 추가:

```markdown
## 측정 결과 (실측)

측정 환경: Jetson Orin Nano 8GB Super 모드 (25W), JetPack 6.2.2, dustynv/ultralytics:r36.4.0, YOLOv8n, conf=0.4, USB 카메라 <모델명> 실측.

| 백엔드 | 해상도 | avg_fps | infer (ms) | end-to-end (ms) |
|---|---|---|---|---|
| PyTorch FP32 | 640x480 | XX.X | XX.X | XX.X |
| PyTorch FP32 | 1280x720 | XX.X | XX.X | XX.X |
| PyTorch FP32 | 1920x1080 | XX.X | XX.X | XX.X |
| TensorRT FP16 | 640x480 | XX.X | XX.X | XX.X |
| TensorRT FP16 | 1280x720 | XX.X | XX.X | XX.X |
| TensorRT FP16 | 1920x1080 | XX.X | XX.X | XX.X |

비고:
- TensorRT FP16 vs PyTorch FP32: 추론 시간 X.X배 단축
- 1920x1080 카메라 미지원/실패 시 위 표에서 해당 행 제거
```

XX.X 자리에 Task 8/9/10에서 측정한 실제 값 기입.

- [ ] **Step 2: 문서 commit + push**

호스트에서:
```bash
cd ~/Defence_Robot
git add docs/plans/2026-05-08-jetson-yolo-stream-plan.md
git commit -m "docs(plan): Jetson YOLO 스트리밍 측정 결과 추가"
git push
```

---

## Task 12: 정리

- [ ] **Step 1: 컨테이너 종료 (선택)**

Jetson 측, 측정 끝났으면:
```bash
exit   # 컨테이너 bash 종료
docker compose -f docker/docker-compose.jetson.yml down
```

- [ ] **Step 2: 추후 재실행 안내 (README 또는 문서에 작성된 그대로)**

다음 세션:
```bash
ssh zetin@jetson-orin.local
cd ~/Defence_Robot && git pull
docker compose -f docker/docker-compose.jetson.yml up -d
docker compose -f docker/docker-compose.jetson.yml exec powertrain bash
python3 motor_control/yolo_cuda_stream.py --backend trt --host <노트북-IP>
```

---

## 검증 기준 (spec 대조)

- [x] (Task 6-7) 컨테이너 빌드 + import 검증 통과
- [x] (Task 8) Jetson 카메라 영상이 노트북에 표시됨 (지연 < 100ms 체감)
- [x] (Task 8) 검출 박스가 영상 위에 정확히 오버레이됨
- [x] (Task 11) 성능 표 작성 (PyTorch vs TensorRT, 해상도별 3가지)

## 잠재 이슈 + 해결책

| 증상 | 원인 후보 | 해결 |
|---|---|---|
| `docker compose build` 타임아웃 | 5GB 베이스 다운로드 중 네트워크 끊김 | 학교 유선랜에서 진행, 또는 `docker pull dustynv/ultralytics:r36.4.0` 따로 미리 |
| 노트북 화면이 검정 | 방화벽 UDP 5000 차단 | 노트북에서 `sudo iptables -L`, 또는 다른 포트로 변경 |
| `nvv4l2h264enc` 없음 | dustynv 베이스에 GStreamer 일부 누락 | Dockerfile에 `gstreamer1.0-plugins-bad gstreamer1.0-tools` 추가 후 rebuild |
| `cv2.cuda.getCudaEnabledDeviceCount()` 가 0 | 베이스 이미지의 OpenCV가 의외로 CPU build | 무시 가능 (ultralytics는 자체적으로 CUDA 사용). 결과 표에서 OpenCV CUDA 가속은 별도 항목 X |
| TRT export 실패 | 입력 사이즈가 너무 큼 | imgsz를 32 배수로 (640, 1280 등 OK; 1280x720 OK) |
| 카메라 못 열림 | `/dev/video0` 외 디바이스 (예: video2) | `v4l2-ctl --list-devices` 확인 후 `--camera /dev/videoN` |

---

## 측정 결과 (실측, 2026-05-09)

측정 환경: Jetson Orin Nano 8GB Super 모드 (25W), JetPack 6.2.2 / L4T R36.5.0,
컨테이너 베이스 `dustynv/l4t-pytorch:r36.4.0` (※ spec 의 dustynv/ultralytics 는
실제 미존재), YOLOv8n conf=0.4, USB 카메라 Microsoft LifeCam Studio,
인코더 OpenH264 (소프트웨어).

| 백엔드 | 해상도 | avg_fps | infer (ms) | end-to-end (ms) |
|---|---|---|---|---|
| PyTorch FP32 | 640×480 | 23.6 | 23.4 | 36.9 |
| PyTorch FP32 | 1280×720 | 12.7 | 25–30 | 51–106 |
| TensorRT FP16 | 640×480 | **27.6** | **9.9** | 33.3 |
| TensorRT FP16 | 1280×720 | 15.9 | 22.5 | 50–76 |

핵심 관찰:
- TensorRT FP16 가 PyTorch FP32 대비 **추론 약 2.4× 단축** (640×480, 23.4ms → 9.9ms).
- 640×480 + TRT 에서 **avg_fps 27.6 (≈ 30fps cap)** — 인코딩 병목 도달.
- e2e 33ms ≈ 1/30s — OpenH264 (소프트웨어) 인코딩이 30fps cap 의 주된 병목.
  추론은 더 빠를 수 있지만 인코딩이 못 따라감.
- 1280×720 PT 가 12.7fps 로 최저 — frame size + 추론 + 인코딩 다 부담.
- 1920×1080 측정 생략 (Microsoft LifeCam Studio 720p 가 최대).

## 진행 중 발견된 이슈 + 영구 fix (commit 이력 참조)

| # | 이슈 | 원인 | 해결 |
|---|---|---|---|
| 1 | Jetson 에 docker engine 없음 | JetPack 6 기본은 nvidia-container-toolkit 만 포함 | `apt install docker.io docker-compose-v2` + `nvidia-ctk runtime configure --runtime=docker` |
| 2 | `dustynv/ultralytics:r36.4.0` 이미지 없음 | spec 추측 잘못 (Docker Hub 미존재) | `dustynv/l4t-pytorch:r36.4.0` 베이스 + ultralytics pip 설치 |
| 3 | `pip install odrive` 실패 | Python 3.11+ 에 wheel 미배포 | 일단 odrive 제거 (이번 plan vision 중심). 모터 트랙 작업 시 별도 처리 |
| 4 | `pip` 인덱스 DNS 미해상 | dustynv 베이스가 jetson.webredirect.org 강제 | `--index-url https://pypi.org/simple` |
| 5 | NumPy 1.x ↔ 2.x 호환 경고 | torch 가 numpy 1.x 로 컴파일됐는데 deps 가 2.x 끌어옴 | `numpy<2` 핀 |
| 6 | `/dev/video0: Operation not permitted` | cgroup device 차단 | compose 에 `privileged: true` |
| 7 | `nvv4l2h264enc` (NVENC) 미인식 | L4T plugin (1.14 ABI) ↔ 컨테이너 GStreamer (1.20+) element 호환 X | 소프트웨어 `openh264enc` 으로 우회 |
| 8 | `cv2.VideoWriter` GStreamer 백엔드 X | dustynv 의 opencv-python wheel 빌드에 GStreamer 미포함 | `subprocess.Popen(gst-launch ...)` + raw BGR `frame.tobytes()` 를 stdin 으로 |
| 9 | `videoconvert: invalid video buffer` | fdsrc 출력은 byte stream — caps 만으론 video buffer 인정 X | `rawvideoparse format=bgr width=W height=H framerate=F/1` 추가 |
| 10 | TRT engine 매 실행 재 빌드 (5–10분) | ultralytics `model.export()` 자체 캐시 X | 입력 사이즈를 파일명 (`yolov8n_HxW_fp16.engine`) 에 인코딩, 존재 시 reuse |

## 향후 작업

- **NVENC 활성화**: jetson-containers 빌드 시스템으로 ultralytics 컨테이너 직접 빌드, 또는 GStreamer 1.14 베이스 이미지 사용. 인코딩 병목 해소 시 1280×720 도 30fps 기대.
- **ODrive 통합**: Python 3.10 컨테이너 별도 또는 odrive git source 설치. spec 의 비목표 였으니 별도 plan.
- **객체 추종 통합**: 본 plan 의 `yolo_cuda_stream.py` + 기존 `odrive_yolo_object_tracking.py` 구조 결합.
- **양자화 (INT8)**: TRT export 시 `int8=True` + 캘리브레이션 데이터 — 추론 추가 1.5–2× 가속 기대.
