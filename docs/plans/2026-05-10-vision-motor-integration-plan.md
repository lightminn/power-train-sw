# Vision + 모터제어 통합 Implementation Plan

> **상태: 완료 (2026-05-10 검증)** — Task 1–8 모두 통과, Phase B/C/D 무부하 동작 정상.
> 측정 결과 + 발견 이슈 표는 본 문서 끝.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 5/8 plan 의 `yolo_cuda_stream.py` 의 검출 흐름과 기존
`odrive_yolo_object_tracking.py` 의 추종 컨트롤러를 결합한
`motor_control/yolo_odrive_jetson.py` 를 신규 작성. Jetson 단일 노드 (Pi 미사용)
에서 vision + 모터제어가 단일 프로세스로 30Hz 폐루프 동작. 영상 송신은 옵션.

**Spec:** [`../specs/2026-05-10-vision-motor-integration-design.md`](../specs/2026-05-10-vision-motor-integration-design.md)

**Architecture:** 단일 프로세스 — 카메라 → ultralytics YOLOv8n (TensorRT FP16) →
검출 박스 중심 → SCALE_FACTOR/POS_FILTER → axis1.input_pos. 옵션 `--stream` 켜면
annotated frame 을 GStreamer subprocess (`yolo_cuda_stream.py` 헬퍼 재사용) 로 동시 송신.

**환경 분리:** Task 1, 2 는 호스트(Arch) 측 git, Task 3-7 은 Jetson 컨테이너 안.

---

## File Structure

| 파일 | 역할 | 작업 위치 |
|---|---|---|
| `docker/Dockerfile.jetson` | odrive git source 설치 layer 추가 (수정) | 호스트 git |
| `motor_control/yolo_cuda_stream.py` | 헬퍼 함수 import 가능 (변경 거의 없음, 확인만) | 호스트 git |
| `motor_control/yolo_odrive_jetson.py` | 통합 스크립트 본체 (신규) | 호스트 git |
| `README.md`, `.claude/CLAUDE.md` | 한 줄씩 추가 (수정) | 호스트 git |
| `docs/plans/2026-05-10-vision-motor-integration-plan.md` | 본 문서 | 호스트 git |

---

## Task 1: Dockerfile.jetson 에 ODrive git 설치 layer 추가

**Files:**
- Modify: `docker/Dockerfile.jetson` (RUN layer 1개 추가)

- [ ] **Step 1: 기존 pip 설치 RUN 뒤에 odrive layer 추가**

기존 (`pip3 install ... ultralytics`) 직후에 다음 RUN 추가:

```dockerfile
# odrive — PyPI 에 Python 3.11+ wheel 미배포, git source 에서 설치.
# (5/8 plan 에서 PyPI wheel 부재로 일단 제외했던 부분 해소)
RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential \
    && pip3 install --no-cache-dir --index-url https://pypi.org/simple \
        "git+https://github.com/odriverobotics/ODrive@fw-v0.5.6#subdirectory=tools" \
    && rm -rf /var/lib/apt/lists/*
```

- [ ] **Step 2: Commit**

```bash
cd ~/Defence_Robot
git add docker/Dockerfile.jetson
git commit -m "feat(docker): odrive git source 설치 layer 추가 (PyPI wheel 부재 우회)"
```

---

## Task 2: 통합 스크립트 motor_control/yolo_odrive_jetson.py 작성

**Files:**
- Create: `motor_control/yolo_odrive_jetson.py`

- [ ] **Step 1: 파일 골격 작성**

기존 두 스크립트에서 가져올 부분:
- `motor_control/yolo_cuda_stream.py` 에서: `build_gst_command`, `resolve_model`,
  `open_camera`, `open_writer` (헬퍼 함수 import)
- `motor_control/odrive_yolo_object_tracking.py` 에서: `SCALE_FACTOR`,
  `MAX_TURNS`, `POS_DEADZONE`, `POS_FILTER`, axis1 폐루프 진입 + 안전 정지 패턴

CLI:
```
python3 yolo_odrive_jetson.py \
    --camera /dev/video0 --width 640 --height 480 --fps 30 \
    --model yolov8n.pt --backend trt --target bottle --conf 0.4 \
    --scale 5.0 --max-turns 2.0 --deadzone 0.05 --pos-filter 0.7 \
    [--stream --host <노트북IP> --port 5000] \
    [--bench-frames 0]
```

핵심 메인 루프 의사코드:
```python
odrv = odrive.find_any(timeout=10)
ax = odrv.axis1
ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
ax.controller.config.input_mode = INPUT_MODE_POS_FILTER
ax.controller.config.input_filter_bandwidth = ...   # 기존 odrive_yolo_object_tracking.py 그대로

target_pos = 0.0
filtered_cx = None
try:
    for frame_idx, frame in capture_loop(cap):
        results = model.predict(frame, conf=args.conf, verbose=False)
        cx_norm = pick_target_cx(results, target_class=args.target, width=args.width)
        if cx_norm is not None:
            # 1차 필터
            filtered_cx = cx_norm if filtered_cx is None else \
                args.pos_filter * filtered_cx + (1 - args.pos_filter) * cx_norm
            err = filtered_cx - 0.5  # 화면 중심 기준
            if abs(err) > args.deadzone:
                target_pos = max(-args.max_turns,
                                 min(args.max_turns, target_pos + err * args.scale * dt))
                ax.controller.input_pos = target_pos
        if args.stream:
            out_proc.stdin.write(results[0].plot().tobytes())
        if frame_idx % 30 == 0:
            log_stats(...)
finally:
    ax.requested_state = AXIS_STATE_IDLE
    ...
```

(정확한 imgsz / pos_filter 계산식은 `odrive_yolo_object_tracking.py` 의 기존
구현을 정확히 복제. 다른 점은 `model.predict()` 가 TRT engine 사용 + frame source
가 V4L2 직접 capture.)

- [ ] **Step 2: 안전 패턴 — KeyboardInterrupt + 예외 어떤 경우에도 axis1 IDLE 보장**

기존 `odrive_yolo_object_tracking.py` 의 try/finally 그대로 유지. 추가로
broken pipe (gst-launch 죽음) 도 catch.

- [ ] **Step 3: 실행 권한 + commit**

```bash
chmod +x motor_control/yolo_odrive_jetson.py
git add motor_control/yolo_odrive_jetson.py
git commit -m "feat(motor_control): yolo_odrive_jetson.py — vision + 모터제어 통합

YOLOv8 TensorRT FP16 검출 결과 (bbox 중심) → axis1.input_pos 폐루프.
영상 송신은 --stream 옵션. 30Hz 단일 프로세스, MAX_TURNS=2.0 안전 한계."
```

---

## Task 3: README + .claude/CLAUDE.md 한 줄 추가

**Files:**
- Modify: `README.md` (motor_control 표에 1줄)
- Modify: `.claude/CLAUDE.md` (Vision-only 또는 신규 "Vision + control" 항목)

- [ ] **Step 1: README.md `#### 환경 / 검출 단독` 표 다음에 새 항목 추가**

기존 `yolo_cuda_stream.py` 행 아래에 다음 추가:

```markdown
| `yolo_odrive_jetson.py` | 위 검출 흐름 + axis1 추종 (HALL 트랙). `--stream` 옵션으로 영상 송신 동시. Jetson 단일 노드 운영 |
```

- [ ] **Step 2: `.claude/CLAUDE.md` 의 Vision-only 항목 갱신**

`yolo_cuda_stream.py` 줄 다음에:

```markdown
- **Vision + control** (single Jetson node):
  `yolo_odrive_jetson.py` (vision 검출 → axis1 추종, optional `--stream` re-uses
  GStreamer pipeline from `yolo_cuda_stream.py`)
```

- [ ] **Step 3: Commit**

```bash
git add README.md .claude/CLAUDE.md
git commit -m "docs: yolo_odrive_jetson.py README + CLAUDE.md 한 줄 추가"
```

- [ ] **Step 4: spec + plan + 위 변경분 push**

```bash
git push origin main
```

---

## Task 4: Jetson 코드 가져오기 + 컨테이너 rebuild

**환경: Jetson (SSH)**

- [ ] **Step 1: pull**

```bash
ssh zetin@jetson-orin.local
cd ~/Defence_Robot && git pull origin main
ls motor_control/yolo_odrive_jetson.py docker/Dockerfile.jetson
```

- [ ] **Step 2: 컨테이너 rebuild (odrive layer 추가됨)**

```bash
sudo docker compose -f docker/docker-compose.jetson.yml build 2>&1 | tail -30
```

기대: git clone + ODrive setup 추가, 약 2-5분 소요. 빌드 끝까지 에러 없음.

- [ ] **Step 3: 컨테이너 진입**

```bash
sudo docker compose -f docker/docker-compose.jetson.yml up -d
sudo docker compose -f docker/docker-compose.jetson.yml exec powertrain bash
```

---

## Task 5: Phase A — ODrive 환경 검증 (컨테이너 안)

- [ ] **Step 1: import 검증**

```bash
python3 -c "import odrive; print('odrive', odrive.__version__)"
```

기대: 버전 문자열 출력 (예: `0.6.x` 또는 fw-v0.5.6 매칭 버전). 에러 시 git source
빌드 실패 — Open Question 1 (별도 Python 3.10 컨테이너) 로 전환.

- [ ] **Step 2: ODrive USB 인식 + axis1 상태**

ODrive 가 Jetson USB 에 꽂혀있는지 확인 후:

```bash
python3 - <<'PY'
import odrive
from odrive.enums import *
odrv = odrive.find_any(timeout=10)
ax = odrv.axis1
print("axis1.error:", hex(ax.error))
print("motor.is_calibrated:", ax.motor.is_calibrated)
print("encoder.is_ready:", ax.encoder.is_ready)
print("current_state:", ax.current_state)
PY
```

기대: `error 0x0`, `is_calibrated True`, `is_ready True`. 에러 시 NVM 캘리 풀린
것 — `motor_control/odrive_calibration.py` 호스트(또는 같은 컨테이너)에서 1회
재실행 필요.

- [ ] **Step 3: TRT + ODrive 동시 import 검증** (Open Question 2)

```bash
python3 - <<'PY'
import torch
import odrive
from ultralytics import YOLO
odrv = odrive.find_any(timeout=5)
print("torch cuda:", torch.cuda.is_available())
print("odrive sn:", odrv.serial_number)
PY
```

기대: 둘 다 OK. 충돌 (segfault, libusb 에러) 시 별도 Python 3.10 컨테이너로 vision/control
분리 + ZMQ IPC 로 plan 수정 (Open Question 1, 2).

---

## Task 6: Phase B — 통합 스크립트 단독 실행 (스트리밍 OFF, 무부하)

**환경: Jetson 컨테이너 안. ODrive 모터 USB 연결 + 차체 미부착.**

- [ ] **Step 1: 모터 회전 안 하는 dry-run (코드 경로만 확인)**

먼저 검출/로그만 — 위치 명령 송신은 코드에서 일시 주석 처리하거나 `--no-motor`
플래그를 추가해 분기 (선택).

```bash
cd /workspace
python3 motor_control/yolo_odrive_jetson.py \
    --backend trt --width 640 --height 480 --fps 30 \
    --target bottle --bench-frames 90
```

기대 stdout: 30 프레임마다 fps + cx + target_pos (모터엔 안 보냄). 5/8 plan 의
`yolo_cuda_stream.py` 측정과 비슷한 fps 27 안팎.

- [ ] **Step 2: 정식 실행 (모터 명령 송신, 객체 좌우 이동)**

(테스트 객체: 페트병 1개 — COCO `bottle` 클래스)

```bash
python3 motor_control/yolo_odrive_jetson.py \
    --backend trt --width 640 --height 480 --fps 30 \
    --target bottle \
    --scale 5.0 --max-turns 2.0 --deadzone 0.05 --pos-filter 0.7
```

화면 좌측 → 우측으로 페트병 이동 → axis1 같은 방향 회전 (또는 반대 — 카메라
마운트 좌우 반전 시 정상). 방향이 의도와 반대면 `--scale` 부호 반전.

- [ ] **Step 3: 안전 한계 + 종료 검증**

- 페트병 한 쪽으로 계속 → `target_pos` 가 `MAX_TURNS=2.0` 에서 saturation 확인
- Ctrl-C → axis1 `IDLE` 진입 + ODrive 에러 0x0 확인:

```bash
# 별도 터미널 또는 다음 진입에서
python3 -c "import odrive; o=odrive.find_any(timeout=5); print(hex(o.axis1.error), o.axis1.current_state)"
```

기대: `0x0 1` (IDLE = 1).

---

## Task 7: Phase D (옵션) — 영상 스트리밍 동시 동작

**환경: 두 노드 — Jetson 컨테이너 + 노트북.**

- [ ] **Step 1: 노트북 측 수신 시작**

```bash
# 노트북 호스트
~/Defence_Robot/scripts/recv_stream.sh 5000
```

- [ ] **Step 2: Jetson 에서 --stream 켜고 실행**

```bash
# Jetson 컨테이너
python3 motor_control/yolo_odrive_jetson.py \
    --backend trt --width 640 --height 480 --fps 30 \
    --target bottle \
    --stream --host <노트북IP> --port 5000 \
    --bench-frames 300
```

기대:
- 노트북 화면에 검출 박스 오버레이 영상
- 동시에 axis1 객체 따라 회전
- stdout fps: phase B (스트리밍 OFF) 보다 약간 낮음 (인코딩 부담)

- [ ] **Step 3: 결과 비교 메모**

본 plan 끝 "측정 결과" 섹션에 phase B vs phase D fps / 추종 latency 차이 기록
(선택 — 정확한 latency 측정 setup 은 별도 plan).

---

## Task 8: 결과 정리 + 문서 commit

**Files:**
- Modify: 본 plan 끝에 측정 결과 + 발견 이슈 표 append (5/8 plan 과 같은 포맷)

- [ ] **Step 1: 본 문서 끝에 결과 섹션 추가**

```markdown
## 측정 결과 (실측, YYYY-MM-DD)

측정 환경: Jetson Orin Nano 8GB Super 모드, JetPack 6.2.2,
컨테이너 `dustynv/l4t-pytorch:r36.4.0` + odrive(git fw-v0.5.6),
YOLOv8n TRT FP16, USB 카메라 <모델명>, ODrive D6374 + HALL.

### Phase B (스트리밍 OFF)
| 지표 | 값 |
|---|---|
| avg_fps | XX.X |
| infer (ms) | XX.X |
| 추종 응답성 | (체감 메모: 좌→우 이동 후 모터 따라잡는 시간) |

### Phase D (스트리밍 ON)
| 지표 | 값 |
|---|---|
| avg_fps | XX.X |
| infer (ms) | XX.X |
| 노트북 영상 latency | (체감 메모) |

### 발견 이슈 + fix
| # | 이슈 | 원인 | 해결 |
|---|---|---|---|
| 1 | ... | ... | ... |
```

- [ ] **Step 2: commit + push**

```bash
git add docs/plans/2026-05-10-vision-motor-integration-plan.md
git commit -m "docs(plan): vision+모터 통합 측정 결과 + 이슈 정리"
git push origin main
```

---

## 검증 기준 (spec 대조)

- [x] (Task 5) Phase A — `import odrive` + `odrive.find_any()` 성공, axis1 status 정상
- [x] (Task 6) Phase B — 통합 스크립트가 카메라 → 검출 → 모터 명령 30Hz 폐루프
- [x] (Task 6) Phase C — 객체 좌우 이동 시 axis1 추종, MAX_TURNS 도달 정지, Ctrl-C IDLE
- [x] (Task 7) Phase D — 영상 스트리밍 + 추종 동시 동작 (avg_fps 28.3 유지)

## 잠재 이슈 + 대응

| 증상 | 원인 후보 | 해결 |
|---|---|---|
| `pip install git+...odrive` 빌드 실패 | gcc/CMake 부재 또는 fw-v0.5.6 브랜치 변경 | Open Question 1 — 별도 Python 3.10 컨테이너 + ZMQ IPC |
| `odrive.find_any()` timeout | USB cgroup, udev, ODrive 펌웨어 mismatch | `lsusb`, `dmesg`, `/dev/bus/usb` 권한 확인. compose 의 `privileged: true` 가 5/8 에서 검증됨 |
| TRT engine + libusb segfault | CUDA + libusb 동시 로드 충돌 | 별도 프로세스 + ZMQ IPC 로 분리 |
| axis1 폐루프 진입 실패 | NVM 캘리 풀림 | `odrive_calibration.py` 1회 실행 (HALL 트랙) |
| 모터 추종 방향 반대 | 카메라 마운트 좌우 반전 | `--scale` 부호 반전 또는 frame `cv2.flip(frame, 1)` |
| 추종 떨림 (객체 정지인데 모터 진동) | `POS_FILTER` 너무 약함 또는 `deadzone` 너무 작음 | `--pos-filter 0.85`, `--deadzone 0.08` 로 튜닝 |
| `--stream` 켜면 fps 급락 | openh264 인코딩 병목 (5/8 의 30fps cap 동일) | 해상도 480×360 으로 낮추거나 NVENC 활성화 (별도 plan) |

## 후속 작업 (이번 plan 외)

- DualSense 텔레옵 + 비전 결합 (스틱 우선, 객체 lock 시 자동 추종 토글)
- 다축 동작 (axis0 + axis1 차동 구동, 6륜 rocker-bogie 매핑)
- 실차 운영 — 차체 + 노면 위 토크/전류 한계 재튜닝
- 노트북 → Jetson 제어 채널 (TCP) — 비상정지 / 모드 전환
- NVENC 활성화 (5/8 plan 의 `nvv4l2h264enc` 병목 해소)
- INT8 양자화 (TRT export `int8=True` + 캘리브레이션)

---

## 측정 결과 (실측, 2026-05-10)

측정 환경: Jetson Orin Nano 8GB Super 모드 (25W), JetPack 6.2.2,
컨테이너 `dustynv/l4t-pytorch:r36.4.0` + odrive (git fw-v0.5.6, libfibre LFS pull),
YOLOv8n TRT FP16 (`yolov8n_480x640_fp16.engine`, 5/8 캐시 재사용),
USB 카메라 Microsoft LifeCam Studio, ODrive v3.6 + 14극 BLDC + TLE5012B 16384 CPR,
vbus 11.96V (※ 무부하 검증 한정).

| Phase | 모드 | avg_fps | infer (ms) | 비고 |
|---|---|---|---|---|
| B/C | 추종 only (스트리밍 OFF) | ≈ 30 | ≈ 9.8 | (5/8 의 yolo_cuda_stream.py 640×480 TRT 결과 27.6 fps + 모터 명령 추가, 32.7 → 30.x 안정 |
| D | 추종 + 스트리밍 (`--stream`) | **28.3** | **9.8** | 300 frame 평균. 객체 좌우 이동 시 `tgt` ±0.97 → ±0.16 변동, 모터 추종 정상 |

핵심 관찰:
- 추론 9.8ms 는 5/8 plan 측정 (9.9ms) 과 동일 — 추종 컨트롤러 추가에도 비전 경로
  변화 없음.
- `--stream` 켜도 fps 28.3 유지 — 인코딩 (openh264, 소프트웨어) 이 30fps cap 의
  주된 병목인 건 5/8 와 동일하지만, 모터 명령이 인코딩 cap 안에 충분히 들어감.
- 객체 좌우 이동에 `tgt` 부호 변화 정상 (`-0.97` → `+0.76` → `+0.16` → `-0.76`),
  `MAX_TURNS=2.0` 안전 한계 안에서 saturation.
- Ctrl-C / `--bench-frames` 종료 시 `returning to origin + IDLE` 정상 시퀀스,
  ODrive 트립 없음.

## 진행 중 발견된 이슈 + 영구 fix

| # | 이슈 | 원인 | 해결 (commit) |
|---|---|---|---|
| 1 | `pip install git+...ODrive` 시 `libfibre-linux-aarch64.so is too small` | ODrive 저장소가 prebuilt fibre `.so` 를 git LFS 보관, pip 의 `git+...` URL 은 LFS 객체 fetch 안 함 | `git-lfs` apt 설치 + 직접 clone + `git lfs pull` 후 로컬 경로 pip install (`21f82f9`) |
| 2 | `ax.controller.config.control_mode = ControlMode.POSITION_CONTROL` 시 `TypeError: int() ... not 'ControlMode'` | fw-v0.5.6 의 `odrive.enums.{AxisState,ControlMode,InputMode}` 가 plain Enum (IntEnum 아님). fibre serializer 가 `int(value)` 호출 시 plain Enum coerce 실패 | 모듈 상단에서 `.value` 로 int 추출해 `AXIS_IDLE` / `AXIS_CLOSED_LOOP` / `CTRL_POSITION` 등 상수로 두고 wire I/O 에 사용 (`68389ee`) |
| 3 | spec 가정 트랙 (HALL D6374) 과 실제 hw (14극 BLDC + TLE5012B 16384 CPR) 불일치 | 5/8 plan 의 odrive.PyPI wheel 부재 노트가 HALL 가정으로 이어졌으나, 실제 보드/모터는 엔코더 트랙 (READme `pi_server_velocity.py` 가정과 동일) | `init_odrive.py` 신규 — `pole_pairs=7` / `cpr=16384` / 게인 NVM 저장 + auto-startup. 통합 스크립트는 NVM 게인 사용으로 하드코딩 게인 제거 (`f3454fd`) |
| 4 | 폐루프 진입 직후 모터가 이전 `input_pos` 값으로 점프 (runaway 위험) | `requested_state = CLOSED_LOOP_CONTROL` 직후 보드 내부 `input_pos` 가 이전 명령 값 그대로 — origin 위치와 다르면 즉시 그쪽으로 가속 | `setup_axis()` 에서 `input_pos = origin` 을 **폐루프 진입 전** 에 박아둠. 안전상 결정적 fix (`f3454fd`) |
| 5 | `Loading yolov8n_480x640_fp16.engine for TensorRT inference... WARNING ⚠ Unable to automatically guess model task` | `.engine` 파일명에서 ultralytics 가 task 추정 실패 | `YOLO(path, task="detect")` 명시 (`68389ee`) |
