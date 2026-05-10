# Jetson Orin Nano YOLO 검출 + 실시간 영상 스트리밍

날짜: 2026-05-08
**상태: 완료 (2026-05-09 검증)** — 측정 결과 + 이슈 fix 이력은
[`../plans/2026-05-08-jetson-yolo-stream-plan.md`](../plans/2026-05-08-jetson-yolo-stream-plan.md) 끝부분.

후속 단계 (vision + 모터제어 통합) 는 별도 spec/plan 으로 분리:
- [`./2026-05-10-vision-motor-integration-design.md`](./2026-05-10-vision-motor-integration-design.md)
- [`../plans/2026-05-10-vision-motor-integration-plan.md`](../plans/2026-05-10-vision-motor-integration-plan.md)

대상 기기: Jetson Orin Nano 8GB Developer Kit (Super 모드, JetPack 6.2.2 / L4T R36.5.0)
호스트: x86_64 Arch (개발/노트북, 영상 수신)

## 목표

1. Jetson에서 USB 카메라 영상을 받아 **CUDA 가속 YOLOv8** 추론으로 객체 검출
2. 검출 결과(박스 + 라벨)가 오버레이된 영상을 **노트북에 실시간 송신**
3. **PyTorch FP32 vs TensorRT FP16** 성능 비교 (FPS, inference latency)
4. ODrive 컨테이너 환경 셋업 (이번엔 import 검증까지만, 실모터 미사용)

## 비목표 (이번에 안 하는 것)

- ODrive 실모터 회전 (캘리, 폐루프 진입 등)
- DualSense 입력 처리
- YOLO 결과로 모터 명령 송출 (객체 추종)
- 노트북 → Jetson 명령 송신 (TCP 서버)

## 시스템 아키텍처

```
┌──────────────────── Jetson Orin Nano ─────────────────────┐
│ /dev/video0  (USB UVC, 1280x720 @ 30fps)                  │
│      │                                                     │
│      ▼ V4L2 capture                                        │
│ [ultralytics YOLOv8n]                                      │
│   ├── PyTorch FP32 (.pt)                                   │
│   └── TensorRT FP16 (.engine, model.export로 생성)         │
│      │                                                     │
│      ▼ annotated frame (BGR)                               │
│ [GStreamer appsrc → nvvidconv → nvv4l2h264enc              │
│   → h264parse → rtph264pay → udpsink]                      │
└──────────────────────┬─────────────────────────────────────┘
                       │ UDP/RTP H.264, port 5000
                       ▼
┌──────────────────── 노트북 (Arch) ────────────────────────┐
│ gst-launch-1.0 udpsrc port=5000                           │
│   ! rtph264depay ! avdec_h264 ! videoconvert              │
│   ! autovideosink sync=false                              │
└────────────────────────────────────────────────────────────┘
```

## 핵심 결정 사항

| 항목 | 선택 | 이유 |
|---|---|---|
| 베이스 이미지 | `dustynv/ultralytics:r36.4.0` | OpenCV CUDA + TensorRT + Ultralytics 사전 통합. 빌드 시간 단축. |
| YOLO 모델 | YOLOv8n | 가장 가볍고 빠름. Jetson에서 30-60 FPS 예상. |
| 추론 백엔드 | PyTorch + TensorRT (둘 다, 비교용) | TensorRT 변환 후 2-3배 속도 향상 측정 |
| 영상 전송 | UDP RTP H.264 (NVENC) | Jetson 하드웨어 인코더 활용, 저지연(<50ms), CPU 부담 미미 |
| 노트북 측 수신 | `gst-launch-1.0` 한 줄 | 별도 프로그램 불필요 |

## 컴포넌트

### 1. `docker/Dockerfile.jetson` (수정)

베이스 이미지를 `nvcr.io/nvidia/l4t-pytorch:r36.2.0-pth2.2-py3` → `dustynv/ultralytics:r36.4.0`로 변경.

핵심 변경:
- `FROM dustynv/ultralytics:r36.4.0`
- `ultralytics`, `opencv-python`, `numpy`는 베이스에 이미 포함 — 중복 설치 제거
- 추가 시스템 패키지(libusb, sdl2, joystick, GStreamer 일부)는 그대로 유지
- 추가 Python 패키지: `odrive`, `pygame`만 (모터 트랙용으로 보존)

### 2. `motor_control/yolo_cuda_stream.py` (신규)

CLI:
```
python yolo_cuda_stream.py \
  --camera /dev/video0 \
  --width 1280 --height 720 --fps 30 \
  --model yolov8n.pt        # 또는 yolov8n.engine
  --backend pt              # 또는 trt
  --host 192.168.1.X        # 노트북 IP
  --port 5000
  --classes bottle person   # 옵션, 미지정 시 COCO 80 전체
  --bench-frames 300        # 측정 후 자동 종료 (옵션)
```

내부 동작:
1. `cv2.VideoCapture(device, cv2.CAP_V4L2)` 카메라 열기
2. `YOLO(model_path)` 로드. backend가 trt면 `.engine` 사용, 없으면 자동 export
3. `cv2.VideoWriter(gst_pipeline, cv2.CAP_GSTREAMER, ...)` GStreamer 송신
4. 매 프레임:
   - `cap.read()`
   - `model.predict(frame, conf=0.4, verbose=False)`
   - `results[0].plot()` 박스 오버레이
   - `out.write(annotated)`
5. 30 프레임마다 stdout: `fps=XX.X inference_ms=XX.X end2end_ms=XX.X`
6. SIGINT 또는 `--bench-frames` 도달 시 종료, 평균 통계 출력

### 3. 노트북 측 수신 (1줄)

```bash
gst-launch-1.0 -v udpsrc port=5000 \
  caps='application/x-rtp,encoding-name=H264,payload=96' \
  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

호스트에 GStreamer 미설치 시: `sudo pacman -S --needed gstreamer gst-plugins-{base,good,bad,ugly} gst-libav`

### 4. (옵션) 헬퍼 스크립트 `scripts/recv_stream.sh`

위 gst-launch 명령을 한 줄로 실행. 노트북에서 매번 길게 입력 안 하게.

## Phase 분해

### Phase 1 — 환경 (Jetson)
1. 호스트에서 design doc commit + push
2. Jetson에서 `git clone https://github.com/lightminn/power-train-sw.git ~/Defence_Robot`
3. `docker compose -f docker/docker-compose.jetson.yml build` (베이스 다운 ~5GB + 추가 1GB, 약 5-15분)
4. `docker compose -f docker/docker-compose.jetson.yml up -d`
5. `docker compose -f docker/docker-compose.jetson.yml exec powertrain bash`
6. 검증:
   - `python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"`
   - `python3 -c "import cv2; print(cv2.__version__, cv2.cuda.getCudaEnabledDeviceCount())"`
   - `python3 -c "import ultralytics; print(ultralytics.__version__)"`
   - `python3 -c "import odrive; import pygame"`
   - `gst-inspect-1.0 nvv4l2h264enc | head -5`
   - `v4l2-ctl --list-devices`

### Phase 2 — 신규 스크립트 작성 (`yolo_cuda_stream.py`)
- 위 인터페이스대로 구현
- 우선 `--backend pt`로 동작 확인
- `--backend trt` 추가 (YOLO export 호출, 첫 실행 시 .engine 캐시)

### Phase 3 — 노트북 수신
1. 노트북에 GStreamer 패키지 설치 (없으면)
2. `recv_stream.sh` 작성, 실행
3. Jetson 측 스크립트 시작 → 노트북 화면에 실시간 영상 + 검출 박스 확인

### Phase 4 — 성능 측정
| 변수 | 값 |
|---|---|
| 백엔드 | PyTorch FP32 / TensorRT FP16 |
| 해상도 | 640x480 / 1280x720 / 1920x1080 |
| 측정 | FPS (전체), inference_ms (모델), end2end_ms (cap+infer+enc+send) |

각 조합 300프레임씩, `--bench-frames 300` 사용. 결과 표 작성.

## 잠재 이슈 / 사전 처리

1. **dustynv 컨테이너 풀 시간**: 5GB 다운로드, 폰 핫스팟이면 오래 걸림. 학교 유선랜 권장 (이미 연결돼있음).
2. **NVENC 권한**: `runtime: nvidia` + `/dev:/dev` 마운트로 컨테이너에서 사용 가능 (이미 compose에 설정).
3. **카메라 권한**: `/dev/video0` 노출 (`/dev:/dev`). `cap_add: SYS_RAWIO` 충분.
4. **Pi IP 192.168.1.91 하드코딩** (`robot_*.py`): 이번 plan에서 안 씀 — 무관.
5. **노트북 IP 변동**: 노트북 IP는 학교망 DHCP에 따라 변할 수 있음. 스크립트 인자로 받게 했으니 OK.
6. **TensorRT 첫 export**: 5-10분 소요. 캐시(.engine)되므로 두 번째부터 즉시.
7. **8GB 메모리 한계**: YOLOv8n + GStreamer + Python 합쳐 1-2GB. 여유 있음.

## 검증 기준

- ✅ 컨테이너 빌드 + import 검증 통과
- ✅ Jetson 카메라 영상이 노트북에 표시됨 (지연 < 100ms 체감)
- ✅ 검출 박스가 영상 위에 정확히 오버레이됨
- ✅ 성능 표 작성 (PyTorch vs TensorRT, 해상도별 3가지)

## 후속 작업 (이번 plan 외)

- **객체 추종 통합** — 별도 spec: `docs/specs/2026-05-10-vision-motor-integration-design.md` 에서 다룸. 본 spec 의 `yolo_cuda_stream.py` 흐름을 기존 `odrive_yolo_object_tracking.py` 와 결합 + Jetson 단일 노드로 운영.
- **DualSense 텔레옵 통합** — 위 통합 plan 의 후순위 phase.
- 노트북에서 검출 박스만 받기 (메타데이터 별도 채널, 영상은 raw로) — 미정.
- 다른 모델 비교: YOLOv8s/m, RT-DETR — 미정.

## 변경 파일 목록

| 파일 | 변경 |
|---|---|
| `docker/Dockerfile.jetson` | 베이스 이미지 + RUN 라인 정리 |
| `motor_control/yolo_cuda_stream.py` | 신규 |
| `scripts/recv_stream.sh` | 신규 (옵션) |
| `README.md` | motor_control 표에 새 스크립트 1줄 추가 |
| `docs/specs/2026-05-08-jetson-yolo-stream-design.md` | 본 문서 |
