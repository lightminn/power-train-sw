# Vision + 모터제어 통합 (Jetson 단일 노드)

날짜: 2026-05-10
선행: [`./2026-05-08-jetson-yolo-stream-design.md`](./2026-05-08-jetson-yolo-stream-design.md) (완료)
대상 기기: Jetson Orin Nano 8GB Super 모드, JetPack 6.2.2 / L4T R36.5.0
컨테이너: `docker/Dockerfile.jetson` (베이스 `dustynv/l4t-pytorch:r36.4.0`)
모터: D6374 150 KV + 내장 HALL, NVM 캘리 완료 가정 (HALL 트랙)

## 목표

1. Jetson 컨테이너 안에서 **ODrive Python lib** 동작 — 5/8 plan 에서 wheel 부재로
   미뤘던 부분을 해결, `import odrive` + `odrive.find_any()` 성공.
2. 5/8 의 `yolo_cuda_stream.py` (검출) + 기존 `odrive_yolo_object_tracking.py`
   (제어) 의 흐름을 결합한 **`yolo_odrive_jetson.py`** 신규 작성 — 검출 박스 중심
   → axis1 위치 명령.
3. **영상 스트리밍은 옵션** (`--stream`) 으로 유지: 켜면 5/8 의 GStreamer UDP RTP
   H.264 송신 동시 동작, 끄면 노트북 의존 없이 단독 동작.
4. **무부하** ODrive 회전 검증 — 실차 차체 미부착, axis1 만 회전. 객체가 좌/우로
   움직이면 모터가 같은 방향으로 추종.

## 비목표 (이번에 안 하는 것)

- 실차 운영 (차체 + 노면 위 주행)
- DualSense 텔레옵 결합 (스틱 + 자동추종 스위치)
- 노트북 → Jetson 명령 송신 (TCP 제어 채널)
- 다축 (axis0 + axis1 동시) 동작
- HALL 외 트랙 (외장 엔코더) 호환

## 시스템 아키텍처

```
┌──────────────────── Jetson Orin Nano ─────────────────────┐
│ /dev/video0  (USB UVC, 640×480 @ 30fps — TRT 실측 기준)    │
│ /dev/ttyACM* or /dev/bus/usb/...   (ODrive USB)            │
│      │                                                      │
│      ▼ V4L2 capture                                         │
│ [ultralytics YOLOv8n .engine, FP16]                         │
│      │                                                      │
│      ├──▶ 검출 박스 중심 (cx)                               │
│      │     │                                                │
│      │     ▼                                                │
│      │   [추종 컨트롤러: SCALE_FACTOR=5.0, MAX_TURNS,       │
│      │    POS_DEADZONE, POS_FILTER]                         │
│      │     │                                                │
│      │     ▼                                                │
│      │   ODrive axis1.controller.input_pos                  │
│      │                                                      │
│      └──▶ (옵션 --stream) annotated frame                   │
│            │                                                │
│            ▼ GStreamer (rawvideoparse → openh264enc         │
│              → rtph264pay → udpsink)                        │
└──────────────────────┬─────────────────────────────────────┘
                       │ UDP/RTP H.264, port 5000 (옵션)
                       ▼
                 노트북 (recv_stream.sh)
```

## 핵심 결정 사항

| 항목 | 선택 | 이유 |
|---|---|---|
| 운영 위치 | Jetson 단일 | Pi 거치면 USB→TCP→Pi→USB 추가 hop. 통합 검증엔 단일 노드가 단순 |
| ODrive 트랙 | HALL (D6374, NVM 캘리됨) | 5/8 plan 시점 이미 캘리되어 있음. 매 실행 풀 캘리하는 인코더 트랙은 통합 검증에 부담 |
| ODrive 환경 | **odrive git source 설치 in 현재 컨테이너** (1차) | 5/8 plan 의 PyPI wheel 부재 우회. 별도 Python 3.10 컨테이너 분리는 IPC 추가 필요해 후순위 |
| 추론 백엔드 | TensorRT FP16 | 5/8 plan 측정: 640×480 에서 추론 9.9ms, FP32 대비 2.4× 단축 |
| 입력 해상도 | 640×480 (기본) | 5/8 측정 27.6 fps — 추종 루프 30 Hz 충분. 720p 는 옵션 |
| 통합 방식 | 단일 프로세스 (vision + control) | 별도 프로세스 + IPC 는 latency 추가. 30 Hz 단일 루프로 충분 |
| 영상 스트리밍 | `--stream` 옵션 (기본 OFF) | 송신 켜면 인코더 부담 추가. 통합 검증 시엔 OFF 가 깔끔 |
| 안전 한계 | `MAX_TURNS = 2.0` (기존 5.0 → 축소) | 무부하 검증 단계 — 폭주 방지 우선 |

## 신규/수정 컴포넌트

### 1. `docker/Dockerfile.jetson` (수정)

ODrive 의존성 추가. PyPI wheel 미존재 → git source 빌드.

추가 layer:
```dockerfile
# odrive — PyPI 에 Python 3.11+ wheel 미배포, git source 에서 설치.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential \
    && pip3 install --no-cache-dir --index-url https://pypi.org/simple \
        "git+https://github.com/odriverobotics/ODrive@fw-v0.5.6#subdirectory=tools" \
    && rm -rf /var/lib/apt/lists/*
```

ODrive USB udev: `docker-compose.jetson.yml` 의 `privileged: true` + `/dev:/dev`
마운트로 충분 (5/8 plan 검증된 구성).

### 2. `motor_control/yolo_odrive_jetson.py` (신규)

CLI:
```
python3 yolo_odrive_jetson.py \
    --camera /dev/video0 \
    --width 640 --height 480 --fps 30 \
    --model yolov8n.pt --backend trt \
    --target bottle \
    --scale 5.0 --max-turns 2.0 --deadzone 0.05 --pos-filter 0.7 \
    --stream                        # 옵션: GStreamer UDP 송신
    --host 192.168.1.X --port 5000  # --stream 켜진 경우만
    --bench-frames 0                # 0=무한
```

내부 모듈 구조:
- `vision.py` 의 인터페이스 차용 (`yolo_cuda_stream.py` 의 `resolve_model`,
  `open_camera`, `build_gst_command`, `open_writer` 헬퍼 재사용 — duplicate 대신
  import 또는 utils 분리)
- `control.py` 부분: `odrive_yolo_object_tracking.py` 의 추종 루프 차용
  (SCALE_FACTOR, MAX_TURNS, POS_DEADZONE, POS_FILTER, axis1 input_pos 송신)
- 메인 루프: capture → predict → 검출 박스 중심 (`cx, cy`) → 위치 명령 →
  (옵션) annotated frame 을 GStreamer 송신

### 3. `motor_control/yolo_cuda_stream.py` (재사용)

기존 헬퍼 (`build_gst_command`, `resolve_model`, `open_camera`, `open_writer`)
를 import 가능하게 하려면 **로직을 함수로 노출만 유지** — `if __name__`
블록은 그대로 (CLI 단독 실행도 계속 가능). 큰 리팩토링 없음.

## Phase 분해

### Phase A — ODrive 환경 (컨테이너)
1. `Dockerfile.jetson` 에 odrive git 설치 layer 추가, rebuild
2. 컨테이너 안에서 `python3 -c "import odrive; odrive.find_any(timeout=5)"` 성공
3. axis1 상태 (`error`, `current_state`) 정상 확인 (NVM 캘리값 그대로)

### Phase B — 통합 스크립트 작성
1. `motor_control/yolo_odrive_jetson.py` 신규 (vision + control 단일 프로세스)
2. `--stream` OFF 로 단독 실행 — stdout 에 `cx`, `input_pos` 로그
3. 안전 정지 핸들러 (Ctrl-C 시 `axis1.requested_state = IDLE`)

### Phase C — 무부하 회전 검증
1. ODrive USB + 카메라 + Jetson 전원 켜고 컨테이너 진입
2. axis1 폐루프 진입 → 객체 (예: 페트병) 화면 좌/우 이동 → 모터 회전 방향 일치 확인
3. `MAX_TURNS = 2.0` 한계 도달 시 정지 확인
4. 객체 사라지면 모터 정지 (마지막 명령 유지) 확인

### Phase D — 영상 스트리밍 동시 동작 (옵션)
1. `--stream --host <노트북IP>` 추가, `recv_stream.sh` 노트북 측 시작
2. 추적 동작 + 영상 송신 동시 — fps / 추종 latency 영향 측정
3. 인코더 부담으로 추종 30Hz 깨지면 phase C 결과와 비교 기록

## Open Questions

해결 안 된 채 plan 진입 시 위험. plan Task 1 직전에 brainstorm 필요.

1. **odrive git source vs Python 3.10 별도 컨테이너** — git source 가 안 빌드되거나
   런타임 깨지면 plan B 로 즉시 전환. (별도 stage Dockerfile 또는 sidecar 컨테이너 +
   ZMQ/TCP IPC).
2. **CUDA + libusb 동시 로드 충돌** — TensorRT 엔진 + ODrive USB 같은 프로세스에서
   동작한 사례 검색 결과 없음. Phase A 에서 import 동시 검증.
3. **추종 루프 vs 추론 fps** — 추론 30Hz 라면 모터 명령도 30Hz. 기존
   `odrive_yolo_object_tracking.py` 는 화면 노이즈 완화 위해 `POS_FILTER=0.7` 1차
   필터. Jetson + TRT 환경에서 동일 게인이 적절한지 phase C 에서 튜닝.
4. **카메라 좌표 → 모터 회전** — 기존 스크립트 는 노트북 카메라 (1280×720) 기준
   `SCALE_FACTOR=5.0`. 입력 해상도 640×480 으로 줄면 동일 픽셀 변위 → 동일
   `cx_norm` 이라 그대로 써도 OK. 단, 좌우 반전 카메라 인지 확인 필요 (USB UVC 마운트
   방향).
5. **NVM 캘리 유효성** — 5/8 작업 이후 ODrive 전원 분리/재인가 됐다면 캘리는 유지되나
   재확인. `axis1.motor.is_calibrated == True` + `axis1.encoder.is_ready == True`.

## 검증 기준

- ✅ Phase A: `import odrive` + `odrive.find_any()` 성공, axis1 status 정상
- ✅ Phase B: `yolo_odrive_jetson.py --stream OFF` 가 카메라 → 검출 → 모터 명령
  파이프라인 끝까지 30Hz 로 돌고 stdout 에 일관된 로그
- ✅ Phase C: 객체 화면 좌측 → axis1 음(또는 양) 방향 회전, 우측 → 반대 방향.
  `MAX_TURNS` 도달 시 정지. Ctrl-C 시 IDLE 안전 종료
- ✅ Phase D (옵션): `--stream` 켜면 노트북 영상 + 검출 박스 + 추종 동작 모두 정상

## 후속 작업 (이번 spec 외)

- DualSense 텔레옵 + 비전 결합 (스틱 입력 우선, 객체 lock 시 자동 추종 토글)
- 다축 동작 (axis0 + axis1 차동 구동)
- 실차 운영 (차체 + 노면, 토크/전류 한계 재튜닝)
- 노트북 → Jetson 제어 채널 (TCP) — 원격 비상정지 / 모드 전환
- 검출 메타데이터 별도 채널 (영상은 그대로, 박스 좌표는 텍스트 stream)

## 변경 파일 목록

| 파일 | 변경 |
|---|---|
| `docker/Dockerfile.jetson` | odrive git source 설치 layer 추가 |
| `motor_control/yolo_odrive_jetson.py` | 신규 |
| `motor_control/yolo_cuda_stream.py` | 헬퍼 함수 import 가능하게 유지 (큰 변경 없음) |
| `docs/specs/2026-05-10-vision-motor-integration-design.md` | 본 문서 |
| `docs/plans/2026-05-10-vision-motor-integration-plan.md` | 구현 plan |
| `README.md` | motor_control 트랙 B 표에 새 스크립트 1줄 추가 |
| `.claude/CLAUDE.md` | Vision-only 항목에 통합 스크립트 추가 |
