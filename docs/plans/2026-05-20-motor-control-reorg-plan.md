# motor_control 재구성 + Jetson 통합 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Jetson 측 독자 작업분 (CAN, AK 조향, US100, Docker 수정) 을 노트북 repo 로 통합하고
`motor_control/` 을 모터 hw 기준 (`drive/{x2212_test,bl70200}`, `steering/`) + 보조 카테고리
(`vision/`, `sensors/`) 구조로 재배치한다. 동시에 D6374 → BL70200 인벤토리 오기를 전수 교정.

**Architecture:** 5 단계 (회수 → 이동·rename → 문서 → commit·push → Jetson sync). 코드 로직
수정 없음, 순수 file system reorg + doc edit. TDD 의 "test" 대신 `git ls-files`, `git grep`,
`import` dry-run 검증을 각 단계 끝에 둠.

**Tech Stack:** git mv, rsync, sshpass + SSH, Markdown edit. Spec:
`docs/specs/2026-05-20-motor-control-reorg-design.md`.

---

## File Structure

**신규 폴더 (Tracked via .gitkeep 불요 — 파일 이동으로 자동 생성):**
- `motor_control/drive/x2212_test/` — SunnySky X2212-13 + TLE5012B 테스트 모터
- `motor_control/drive/bl70200/` — BL70200 + 내장 HALL ×3 실전 모터
- `motor_control/steering/` — AK40/AK45 조향
- `motor_control/vision/` — 비전 단독 (모터 명령 없음)
- `motor_control/sensors/` — UART 센서

**기존 폴더 — 손대지 않음:**
- `motor_control/laptop/`, `motor_control/pi/`, `parameter_calc/`, `docker/`, `docs/`

**신규 파일 (Jetson → 노트북 으로 회수):**
| 회수 경로 (Jetson) | 최종 경로 (노트북) |
| --- | --- |
| `motor_control/odrive_can_setup.py` | `motor_control/drive/x2212_test/odrive_can_setup.py` |
| `motor_control/odrive_can_drive.py` | `motor_control/drive/x2212_test/odrive_can_drive.py` |
| `motor_control/ak40_control.py` | `motor_control/steering/ak_control.py` |
| `motor_control/calibrate_ak40.py` | `motor_control/steering/calibrate_ak.py` |
| `motor_control/status_ak40.py` | `motor_control/steering/status_ak.py` |
| `motor_control/run_ak40.py` | `motor_control/steering/run_ak.py` |
| `us100_test.py` | `motor_control/sensors/us100_basic.py` |
| `uart_test.py` | `motor_control/sensors/us100_robust.py` |
| `~/can_setup.sh` | `scripts/can_setup.sh` |
| `docker/Dockerfile.jetson` (modified) | `docker/Dockerfile.jetson` |
| `docker/docker-compose.jetson.yml` (modified) | `docker/docker-compose.jetson.yml` |

**기존 추적 파일 이동 (git mv) — 15 개:**

```
motor_control/init_odrive.py                     → drive/x2212_test/
motor_control/odrive_dualsense_test.py           → drive/x2212_test/
motor_control/odrive_dualsense_vel_test.py       → drive/x2212_test/
motor_control/yolo_odrive_jetson.py              → drive/x2212_test/
motor_control/yolo_odrive_motor_test.py          → drive/x2212_test/
motor_control/odrive_yolo_object_tracking.py     → drive/x2212_test/
motor_control/odrive_calibration.py              → drive/bl70200/
motor_control/odrive_basic_test.py               → drive/bl70200/
motor_control/odrive_closed_loop_test.py         → drive/bl70200/
motor_control/odrive_diff_drive_test.py          → drive/bl70200/
motor_control/odrive_position_hold_test.py       → drive/bl70200/
motor_control/odrive_velocity_hold_test.py       → drive/bl70200/
motor_control/yolo_openvino_detection.py         → vision/
motor_control/yolo_cuda_stream.py                → vision/
motor_control/setup_yolo_env.sh                  → vision/
```

**수정 파일 (in-place edit):**
- `README.md` — 트랙 분류표 재작성, 트리 갱신, D6374 → BL70200 교정
- `.claude/CLAUDE.md` — Directory Layout, motor_control 인벤토리, Robot Specification
- `HANDOFF.md` — 모터 인벤토리 정정 + 구조 갱신
- `.gitignore` — 모델 산출물 + Kate swp

---

## Task 1: Jetson 측 파일 회수 (rsync)

**Files:**
- Create: `motor_control/odrive_can_setup.py`, `motor_control/odrive_can_drive.py`,
  `motor_control/ak40_control.py`, `motor_control/calibrate_ak40.py`, `motor_control/status_ak40.py`,
  `motor_control/run_ak40.py`, `us100_test.py`, `uart_test.py`, `can_setup.sh` (임시 위치)
- Modify: `docker/Dockerfile.jetson`, `docker/docker-compose.jetson.yml`

- [ ] **Step 1: Jetson 의 새 motor_control 파일 6개 rsync**

```bash
sshpass -p "0000" rsync -av \
  zetin@jetson-orin.local:'~/Defence_Robot/motor_control/odrive_can_setup.py
                          ~/Defence_Robot/motor_control/odrive_can_drive.py
                          ~/Defence_Robot/motor_control/ak40_control.py
                          ~/Defence_Robot/motor_control/calibrate_ak40.py
                          ~/Defence_Robot/motor_control/status_ak40.py
                          ~/Defence_Robot/motor_control/run_ak40.py' \
  /home/light/Defence_Robot/motor_control/
```

- [ ] **Step 2: 센서 테스트 파일 + can_setup.sh 회수**

```bash
sshpass -p "0000" rsync -av \
  zetin@jetson-orin.local:'~/Defence_Robot/uart_test.py
                          ~/Defence_Robot/us100_test.py' \
  /home/light/Defence_Robot/

sshpass -p "0000" rsync -av \
  zetin@jetson-orin.local:'~/can_setup.sh' \
  /home/light/Defence_Robot/
```

- [ ] **Step 3: 수정된 Docker 파일 회수 (덮어쓰기)**

```bash
sshpass -p "0000" rsync -av \
  zetin@jetson-orin.local:'~/Defence_Robot/docker/Dockerfile.jetson
                          ~/Defence_Robot/docker/docker-compose.jetson.yml' \
  /home/light/Defence_Robot/docker/
```

- [ ] **Step 4: 회수 결과 검증**

```bash
cd /home/light/Defence_Robot && git status --short
```

Expected: 11 줄 — modified `docker/Dockerfile.jetson`, `docker/docker-compose.jetson.yml`,
untracked `can_setup.sh`, `uart_test.py`, `us100_test.py`, `motor_control/{ak40_control,calibrate_ak40,run_ak40,status_ak40,odrive_can_drive,odrive_can_setup}.py`,
`HANDOFF.md` (preexisting untracked).

**No commit yet — files will be staged after reorg.**

---

## Task 2: 신규 폴더 생성 + 기존 추적 파일 이동 (git mv)

**Files:**
- Create directories: `motor_control/drive/x2212_test/`, `motor_control/drive/bl70200/`,
  `motor_control/steering/`, `motor_control/vision/`, `motor_control/sensors/`
- Move 15 tracked files via `git mv`

- [ ] **Step 1: 폴더 skeleton 생성**

```bash
cd /home/light/Defence_Robot
mkdir -p motor_control/drive/x2212_test motor_control/drive/bl70200 \
         motor_control/steering motor_control/vision motor_control/sensors
```

- [ ] **Step 2: drive/x2212_test/ 로 git mv (6 개 추적 파일)**

```bash
git mv motor_control/init_odrive.py                 motor_control/drive/x2212_test/
git mv motor_control/odrive_dualsense_test.py       motor_control/drive/x2212_test/
git mv motor_control/odrive_dualsense_vel_test.py   motor_control/drive/x2212_test/
git mv motor_control/yolo_odrive_jetson.py          motor_control/drive/x2212_test/
git mv motor_control/yolo_odrive_motor_test.py      motor_control/drive/x2212_test/
git mv motor_control/odrive_yolo_object_tracking.py motor_control/drive/x2212_test/
```

- [ ] **Step 3: drive/bl70200/ 로 git mv (6 개 추적 파일)**

```bash
git mv motor_control/odrive_calibration.py        motor_control/drive/bl70200/
git mv motor_control/odrive_basic_test.py         motor_control/drive/bl70200/
git mv motor_control/odrive_closed_loop_test.py   motor_control/drive/bl70200/
git mv motor_control/odrive_diff_drive_test.py    motor_control/drive/bl70200/
git mv motor_control/odrive_position_hold_test.py motor_control/drive/bl70200/
git mv motor_control/odrive_velocity_hold_test.py motor_control/drive/bl70200/
```

- [ ] **Step 4: vision/ 로 git mv (3 개 추적 파일)**

```bash
git mv motor_control/yolo_openvino_detection.py motor_control/vision/
git mv motor_control/yolo_cuda_stream.py        motor_control/vision/
git mv motor_control/setup_yolo_env.sh          motor_control/vision/
```

- [ ] **Step 5: 이동 결과 검증**

```bash
git status --short | grep -E "renamed:" | wc -l
```

Expected: `15` (정확히 15개 rename).

```bash
ls motor_control/   # 평면 잔류 파일은 untracked 회수분만 보여야 함
```

Expected: `ak40_control.py`, `calibrate_ak40.py`, `odrive_can_drive.py`,
`odrive_can_setup.py`, `run_ak40.py`, `status_ak40.py`, plus `drive/`, `steering/`,
`vision/`, `sensors/`, `laptop/`, `pi/`.

**No commit yet.**

---

## Task 3: 회수한 Jetson 파일 신규 위치로 이동 + rename

**Files:**
- Move 6 untracked Python files + rename 4
- Move `can_setup.sh` → `scripts/can_setup.sh`

- [ ] **Step 1: drive/x2212_test/ — ODrive CAN 2 개 (rename 없음)**

```bash
cd /home/light/Defence_Robot
mv motor_control/odrive_can_setup.py motor_control/drive/x2212_test/
mv motor_control/odrive_can_drive.py motor_control/drive/x2212_test/
```

- [ ] **Step 2: steering/ — AK 4 개 (`ak40_` → `ak_` rename)**

```bash
mv motor_control/ak40_control.py    motor_control/steering/ak_control.py
mv motor_control/calibrate_ak40.py  motor_control/steering/calibrate_ak.py
mv motor_control/status_ak40.py     motor_control/steering/status_ak.py
mv motor_control/run_ak40.py        motor_control/steering/run_ak.py
```

- [ ] **Step 3: sensors/ — US100 2 개 (rename 적용)**

```bash
mv us100_test.py motor_control/sensors/us100_basic.py
mv uart_test.py  motor_control/sensors/us100_robust.py
```

- [ ] **Step 4: scripts/can_setup.sh 배치 + 실행 권한**

```bash
mv can_setup.sh scripts/can_setup.sh
chmod +x scripts/can_setup.sh
```

- [ ] **Step 5: 이동 결과 검증**

```bash
ls motor_control/drive/x2212_test/ motor_control/drive/bl70200/ \
   motor_control/steering/ motor_control/vision/ motor_control/sensors/ \
   scripts/
```

Expected — 각각:
- `drive/x2212_test/`: 8 개 (`init_odrive.py`, `odrive_can_drive.py`,
  `odrive_can_setup.py`, `odrive_dualsense_test.py`, `odrive_dualsense_vel_test.py`,
  `odrive_yolo_object_tracking.py`, `yolo_odrive_jetson.py`, `yolo_odrive_motor_test.py`)
- `drive/bl70200/`: 6 개 (`odrive_basic_test.py`, `odrive_calibration.py`,
  `odrive_closed_loop_test.py`, `odrive_diff_drive_test.py`,
  `odrive_position_hold_test.py`, `odrive_velocity_hold_test.py`)
- `steering/`: 4 개 (`ak_control.py`, `calibrate_ak.py`, `run_ak.py`, `status_ak.py`)
- `vision/`: 3 개 (`setup_yolo_env.sh`, `yolo_cuda_stream.py`, `yolo_openvino_detection.py`)
- `sensors/`: 2 개 (`us100_basic.py`, `us100_robust.py`)
- `scripts/`: 2 개 (`can_setup.sh`, `recv_stream.sh`)

**No commit yet.**

---

## Task 4: .gitignore 갱신

**Files:**
- Modify: `.gitignore`

- [ ] **Step 1: 모델 산출물 + Kate swp 패턴 추가**

`.gitignore` 끝에 다음 블록 추가:

```gitignore

# YOLO 모델 산출물 (재export 가능 — 운영 머신에 보관)
yolov8n.pt
yolov8n*.onnx
yolov8n*_*.engine

# Kate editor swap
.*.kate-swp
```

- [ ] **Step 2: 동작 검증**

```bash
cd /home/light/Defence_Robot
# 가상 파일로 무시 패턴 확인
touch motor_control/yolov8n.pt
git check-ignore -v motor_control/yolov8n.pt
rm motor_control/yolov8n.pt
```

Expected: `.gitignore:<line>:yolov8n.pt        motor_control/yolov8n.pt`

---

## Task 5: README.md 갱신

**Files:**
- Modify: `README.md`

`README.md` 의 3 개 섹션을 갱신. 다른 부분은 손대지 않음.

- [ ] **Step 1: 저장소 구조 트리 (line 17~65 부근) 교체**

`README.md` 의 ` ``` ` 블록 (`.` 로 시작, ` ``` ` 으로 끝) 전체를 아래로 교체:

```
.
├── parameter_calc/
│   ├── CLAUDE.md                    # 이 트랙의 상세 문서 (필독)
│   ├── matlab/                      # MATLAB 원본 레퍼런스
│   ├── python/                      # CPU 포팅 (NumPy / SciPy)
│   ├── python_gpu/                  # GPU 포팅 (JAX / CUDA 12.x) — v3
│   ├── python_gpu_triangle/         # GPU 변형 — triangle 모드 한정 (v4)
│   └── scripts/                     # SLURM/서버 실행 래퍼
├── motor_control/
│   ├── drive/                       # 구동 모터
│   │   ├── x2212_test/              # SunnySky X2212-13 + TLE5012B (테스트)
│   │   │   ├── init_odrive.py          # USB NVM 셋업 (pp=7, cpr=16384)
│   │   │   ├── odrive_can_setup.py     # CAN NVM 셋업 (안전 IDLE 부팅)
│   │   │   ├── odrive_can_drive.py     # CAN 폐루프 데모
│   │   │   ├── odrive_dualsense_test.py
│   │   │   ├── odrive_dualsense_vel_test.py
│   │   │   ├── yolo_odrive_jetson.py        # Jetson 비전 추종 (USB)
│   │   │   ├── yolo_odrive_motor_test.py    # x86 OpenVINO 추종 (참조)
│   │   │   └── odrive_yolo_object_tracking.py
│   │   └── bl70200/                 # BL70200 + 내장 HALL ×3 (실전)
│   │       ├── odrive_calibration.py        # HALL 모드 (pp=5, cpr=30)
│   │       ├── odrive_diff_drive_test.py
│   │       ├── odrive_basic_test.py
│   │       ├── odrive_closed_loop_test.py
│   │       ├── odrive_position_hold_test.py
│   │       └── odrive_velocity_hold_test.py
│   ├── steering/                    # 조향 (AK40/AK45 동일 API, CAN)
│   │   ├── ak_control.py            # 메인 — AK 클래스 + CANSession
│   │   ├── calibrate_ak.py          # 기어비 결정 1회성
│   │   ├── status_ak.py             # CAN RX 디버깅
│   │   └── run_ak.py                # TMotorCANControl 데모
│   ├── vision/                      # 비전 단독 (모터 명령 없음)
│   │   ├── yolo_openvino_detection.py  # x86 OpenVINO
│   │   ├── yolo_cuda_stream.py         # Jetson CUDA/TRT + UDP H.264
│   │   └── setup_yolo_env.sh           # x86 conda (Docker 권장)
│   ├── sensors/                     # 센서 (UART /dev/ttyTHS1)
│   │   ├── us100_basic.py           # 0x55 기본
│   │   └── us100_robust.py          # Jetson UART 버그 우회
│   ├── laptop/                      # 노트북 측 텔레옵 클라이언트
│   │   ├── laptop_client_basic.py
│   │   ├── laptop_client_velocity.py
│   │   └── laptop_client_video.py
│   └── pi/                          # 라즈베리파이 측 서버
│       ├── pi_server_basic.py
│       ├── pi_server_video.py
│       ├── pi_server_velocity.py
│       ├── pi_server_position.py
│       └── camera/
├── docker/                          # 컨테이너 정의
│   ├── Dockerfile                   # x86_64 개발 (Ubuntu 22.04)
│   ├── Dockerfile.jetson            # Jetson (dustynv/l4t-pytorch:r36.4.0 + python-can)
│   ├── docker-compose.yml           # x86 기본
│   ├── docker-compose.gpu.yml       # x86 NVIDIA GPU
│   └── docker-compose.jetson.yml    # Jetson (runtime: nvidia, /dev/ttyTHS1)
├── scripts/                         # 호스트 측 헬퍼
│   ├── recv_stream.sh               # 노트북에서 Jetson UDP H.264 수신
│   └── can_setup.sh                 # Jetson can0 1 Mbps 셋업 (mttcan + devmem)
└── docs/                            # 트랙별 설계·계획 문서
    ├── specs/                       # 설계 (요구사항/인터페이스)
    └── plans/                       # 구현 계획 + 검증 기록
```

- [ ] **Step 2: 트랙 분류표 (line 119~124 부근) 재작성**

기존:
```
| 트랙 | 센서 | 모터 모델 | 캘리 방식 |
| --- | --- | --- | --- |
| **HALL** | 모터 내장 홀센서 | D6374 150 KV (...) | ... |
| **엔코더** | 외장 증분형 | ODrive 기본값 | ... |
```

교체:
```
| 트랙 | 모터 (테스트 → 실전) | 센서 | 통신 | 폴더 |
| --- | --- | --- | --- | --- |
| **구동 (실전)** | BL70200 | 내장 HALL ×3 (pp=5, cpr=30) | ODrive USB | `drive/bl70200/` |
| **구동 (테스트)** | SunnySky X2212-13 | TLE5012B 16384 CPR (외장) | ODrive USB · CAN | `drive/x2212_test/` |
| **조향** | AK40-10 → AK45 (동일 API) | 내장 | CAN (socketcan can0) | `steering/` |

ODrive 펌웨어 v0.5.x (CAN 트랙은 fw-v0.5.6 검증), 테스트 axis = `axis1`.
HALL 트랙의 캘리값 `pp=5, cpr=30` 은 BL70200 hw 와 일치 — 실차 부착 시 vbus 24 V
환경에서 `current_lim` / `vel_limit` 만 재확인.
```

- [ ] **Step 3: 트랙 본문 (line 126 ~ 213) 갱신**

`### 트랙 A — HALL (실차 운영용)` 섹션의 헤더와 도입부를 다음과 같이 교체:

```markdown
### 트랙: 구동 모터 (실전 — BL70200, `drive/bl70200/`)

BL70200 + 내장 HALL ×3. ODrive HALL 모드 사용, NVM 캘리 저장 후 재사용.
cpr=30 으로 낮아 게인 보수적 + vel-estimate 필터 bandwidth 낮춰 노이즈 억제.
```

표의 파일명은 `odrive_calibration.py` → `drive/bl70200/odrive_calibration.py` 처럼
경로 prefix 추가. `odrive_calibration.py` 의 "HALL 모드·`pole_pairs=5`·`cpr=30`·`HIGH_CURRENT`
강제" 설명은 그대로.

`### 트랙 B — 엔코더 (벤치 / 텔레옵 / 비전)` 섹션 헤더와 본문 도입부를 다음과 같이 교체:

```markdown
### 트랙: 구동 모터 (테스트 — SunnySky X2212-13, `drive/x2212_test/`)

외장 TLE5012B 증분 엔코더 (16384 CPR) + SunnySky X2212-13 (14극 BLDC, pp=7).
**ODrive 검증·게인 튜닝·비전 통합 PoC 용도**. 실차 운영은 BL70200 트랙으로 이전.
ODrive 통신은 USB · CAN 둘 다 지원 — 같은 모터·캘리값에 통신 매체만 다름.
```

표 안의 파일들은 새 경로로 prefix (`drive/x2212_test/init_odrive.py` 등). `init_odrive.py`
설명은 그대로 유지하되 위치는 `drive/x2212_test/` 명시.

신규 섹션 추가 — `### 트랙: 조향 (AK40/AK45, `steering/`)`:

```markdown
### 트랙: 조향 (AK40 / AK45, `steering/`)

CubeMars AK 시리즈 (AK40-10 테스트 → AK45 실전, 동일 API). CAN bus
(socketcan `can0`, motor_id=10) 로 위치·속도·브레이크 제어. 짐벌 운동학 함수도
포함하지만 본 프로젝트에서는 **조향 서보** 로 사용.

| 파일 | 역할 |
| --- | --- |
| `ak_control.py` | `AK` 클래스 + `CANSession` 컨텍스트 매니저. `send_pos_out(out_deg)` 출력축 직결, `move_rel_out()` 상대 위치, `send_rpm_out()` 속도, `stop()` 안전정지 |
| `calibrate_ak.py` | 기어비 (10/1) 결정용 1 회성 — `send_pos_raw(36°)` 결과로 출력축 기준 판별 |
| `status_ak.py` | CAN RX 디버깅 — 모든 응답 패킷 hex dump |
| `run_ak.py` | 서드파티 `TMotorCANControl` 라이브러리 사용 데모 (참조) |

CAN 사전 준비: 호스트에서 `bash scripts/can_setup.sh` 로 can0 1 Mbps 셋업
(mttcan + devmem mux + bitrate). Dockerfile.jetson 에 `python-can` 포함.
```

신규 섹션 추가 — `### 보조: 비전 단독 (vision/)` 와 `### 보조: 센서 (sensors/)`:

```markdown
### 보조: 비전 단독 (`vision/`)

모터 명령 없는 검출·스트리밍 전용. 본 트랙에서 만든 결과 (검출 박스 좌표) 가
구동 트랙의 추종 입력이 될 수 있음.

| 파일 | 역할 |
| --- | --- |
| `yolo_openvino_detection.py` | Intel OpenVINO YOLOv8n (V4L2, 1280×720) + 임의 카메라 파라미터 3D 변환 (x86 한정) |
| `yolo_cuda_stream.py` | YOLOv8 (PyTorch CUDA / TensorRT FP16) + GStreamer H.264 RTP UDP 송신. Jetson 컨테이너 안에서 실행, 수신은 `scripts/recv_stream.sh` |
| `setup_yolo_env.sh` | x86 yolo_env 콘다 환경 (Docker 권장으로 deprecated) |

### 보조: 센서 (`sensors/`)

| 파일 | 센서 | 비고 |
| --- | --- | --- |
| `us100_basic.py` | US100 초음파 거리 | UART `/dev/ttyTHS1`, 9600 bps, `0x55` 트리거 + 2 byte 응답 |
| `us100_robust.py` | 동일 | Jetson UART TX 전압 떨림 버그를 `0xFF`×8 prefix 로 우회 |
```

기존 `### 트랙 A — HALL (실차 운영용)` 및 `### 트랙 B — 엔코더` 본문을 삭제하고
위 신규 섹션들로 교체.

- [ ] **Step 4: 실행 절차 (line 206~213 부근) 교체**

기존 `#### 절차` 블록을 다음과 같이 교체:

```markdown
#### 실행 흐름

1. **환경 셋업**: Docker 권장 (아래 「Docker 환경」). Jetson 은 `bash scripts/can_setup.sh`
   로 can0 1 Mbps 활성화 (CAN 트랙 사용 시).
2. **모터 캘리** (트랙별 1 회):
   - BL70200: `python drive/bl70200/odrive_calibration.py` (HALL 모드, NVM 저장)
   - X2212 USB: `python drive/x2212_test/init_odrive.py` (NVM 게인 + auto closed-loop)
   - X2212 CAN: `python drive/x2212_test/odrive_can_setup.py` (안전 IDLE 부팅)
   - AK 조향: `python steering/calibrate_ak.py` (기어비 결정 후 `ak_control.py` 값 반영)
3. **단축 검증**:
   - BL70200: `python drive/bl70200/odrive_closed_loop_test.py` (2 회전 + axis.error 0x0)
   - X2212 USB: `python drive/x2212_test/odrive_dualsense_vel_test.py` (트리거 텔레옵)
   - X2212 CAN: `python drive/x2212_test/odrive_can_drive.py` (5.0 → 0.0 시퀀스)
   - AK: `python steering/ak_control.py` (데모 모드)
4. **통합 운영** (Jetson):
   - 비전 추종: `python drive/x2212_test/yolo_odrive_jetson.py --backend trt`
   - 원격 텔레옵: 노트북 `laptop/laptop_client_video.py` + Pi `pi/pi_server_video.py`
```

- [ ] **Step 5: D6374 잔존 검사 + 교정**

```bash
cd /home/light/Defence_Robot
git grep -ni "D6374" README.md
```

Expected: 결과 없음. 남아 있으면 BL70200 으로 교체 (HALL 모드, pp=5, cpr=30 의 hw 임을 명시).

```bash
git grep -ni "150 KV\|4\.95 Nm" README.md
```

D6374 specific 스펙 (150 KV, 4.95 Nm peak) 표현이 있으면 삭제 또는 BL70200 hw 값으로 교체
(BL70200 hw spec 미상이면 표현 자체를 삭제).

---

## Task 6: .claude/CLAUDE.md 갱신

**Files:**
- Modify: `.claude/CLAUDE.md`

- [ ] **Step 1: Directory Layout 섹션 교체 (line 12~35)**

기존 ` ``` ` 블록 안의 `motor_control/` 부분을 다음으로 교체:

```
├── motor_control/        ODrive · AK 조향 · YOLO · US100 센서 · 텔레옵
│   ├── drive/            구동 모터
│   │   ├── x2212_test/   SunnySky X2212-13 + TLE5012B (ODrive USB · CAN)
│   │   └── bl70200/      BL70200 + 내장 HALL ×3 (실전, ODrive USB)
│   ├── steering/         AK40/AK45 조향 (CAN, 동일 API)
│   ├── vision/           모터 명령 없는 검출·스트리밍
│   ├── sensors/          US100 거리 (UART /dev/ttyTHS1)
│   ├── laptop/           Laptop-side TCP teleop clients (DualSense → robot)
│   └── pi/               Raspberry-Pi-side servers (paired 1:1 with laptop/)
```

- [ ] **Step 2: Working in motor_control 섹션 (line 41~71) 재작성**

`## Working in motor_control/` 섹션 전체를 다음으로 교체:

```markdown
## Working in `motor_control/`

Self-contained Python scripts; no shared package structure. Three motor hardware
lines, isolated by subfolder. **Never mix tracks on the same ODrive** (calibration
/ gain / current limits diverge).

- **drive/bl70200/** (BL70200 + 내장 HALL ×3, 실전 구동): `odrive_calibration.py`,
  `odrive_diff_drive_test.py`, `odrive_basic_test.py`, `odrive_closed_loop_test.py`,
  `odrive_position_hold_test.py`, `odrive_velocity_hold_test.py` — HALL 모드,
  `pp=5, cpr=30, HIGH_CURRENT`, NVM 저장 후 재사용.
- **drive/x2212_test/** (SunnySky X2212-13 + TLE5012B, 테스트·PoC):
  `init_odrive.py` (USB 1회 NVM 셋업, pp=7 cpr=16384), `odrive_can_setup.py` /
  `odrive_can_drive.py` (CAN), `odrive_dualsense_*.py` (텔레옵), `yolo_odrive_jetson.py`
  (Jetson 비전 추종, USB), `yolo_odrive_motor_test.py` · `odrive_yolo_object_tracking.py`
  (x86 OpenVINO 추종, 참조).
- **steering/** (AK40-10 → AK45, CAN socketcan can0): `ak_control.py` (메인 라이브러리),
  `calibrate_ak.py` (기어비 1회성), `status_ak.py` (CAN RX 디버깅), `run_ak.py`
  (TMotorCANControl 데모). 사전 준비: `bash scripts/can_setup.sh`.
- **vision/** (모터 명령 없음): `yolo_openvino_detection.py` (x86 OpenVINO),
  `yolo_cuda_stream.py` (Jetson CUDA/TRT + GStreamer UDP H.264 송신 — 수신은
  `scripts/recv_stream.sh`), `setup_yolo_env.sh` (x86 conda, Docker 권장).
- **sensors/** (UART `/dev/ttyTHS1`): `us100_basic.py` (US100 0x55 기본),
  `us100_robust.py` (Jetson UART TX 떨림 버그 우회 — 0xFF prefix).
- **Networked teleop** (1:1 pairs): `laptop/laptop_client_*.py` ↔ `pi/pi_server_*.py`
  (TCP `:9000`, newline-delimited `%.4f\n` velocity); `laptop_client_video.py` adds
  GStreamer JPEG video at `:5000`.

ODrive 펌웨어 v0.5.x (CAN 트랙 fw-v0.5.6 검증), all scripts use `axis1`. Jetson CAN
트랙 입력 전 `input_pos = origin` 설정으로 폐루프 진입 시 모터 점프 방지.
```

- [ ] **Step 3: Robot Specification 섹션 (line 84~89) 갱신**

기존:
```
- Drive motor: D6374 150 KV via 5:1 gearbox (≈ 21 Nm at wheel, peak)
```

교체:
```
- Drive motor (test): SunnySky X2212-13 + TLE5012B 16384 CPR encoder
- Drive motor (real): BL70200 + internal HALL ×3 (pp=5, cpr=30)
- Steering: CubeMars AK40-10 (test) / AK45 (real), CAN bus, identical API
```

- [ ] **Step 4: Jetson Orin Nano deployment 섹션 (line 73~82) 의 entry point 경로 갱신**

`motor_control/yolo_cuda_stream.py` → `motor_control/vision/yolo_cuda_stream.py` 로 수정.

- [ ] **Step 5: D6374 잔존 검사**

```bash
cd /home/light/Defence_Robot
git grep -ni "D6374" .claude/CLAUDE.md
```

Expected: 결과 없음.

---

## Task 7: HANDOFF.md 갱신

**Files:**
- Modify: `HANDOFF.md`

HANDOFF 는 historical 문서지만 모터 인벤토리 오기는 명시적 정정. 새 구조 반영.

- [ ] **Step 1: 하드웨어 표 (line 30~43) 갱신**

`## 하드웨어 실측 구성` 표의 "모터 / 엔코더" 행을 다음으로 교체:

```
| 모터 / 엔코더 | **실전 구동: BL70200 + 내장 HALL ×3 (pp=5, cpr=30, HALL 모드)** / **테스트 구동: SunnySky X2212-13 + TLE5012B 16384 CPR (pp=7, 외장 엔코더)** / **조향: AK40-10 (테스트) → AK45 (실전), CAN 동일 API**. 5/10 무부하 검증은 SunnySky 트랙에서 수행 |
```

표 아래 "중요" 단락의 D6374 / "spec 작성 시엔 HALL 트랙 (D6374) 가정했으나" 문장을 다음으로
교체:

```
**중요**: spec 작성 시 모터 인벤토리 오기 존재 — 실제 hw 는 위 표대로 세 가지. 5/20
재구성 commit (`docs/specs/2026-05-20-motor-control-reorg-design.md`) 에서 폴더
구조 + 문서 전반 교정 완료.
```

- [ ] **Step 2: 저장소 구조 (line 90~127) 갱신**

`## 저장소 구조 (working tree)` 블록의 `motor_control/` 부분을 새 디자인 트리로 교체
(Task 5 Step 1 의 트리 동일). HANDOFF 는 한국어 + 짧은 주석 스타일 유지.

- [ ] **Step 3: 빠른 실행 (line 144~170) 의 경로 갱신**

`python3 motor_control/yolo_cuda_stream.py` → `python3 motor_control/vision/yolo_cuda_stream.py`
`python3 motor_control/init_odrive.py` → `python3 motor_control/drive/x2212_test/init_odrive.py`
`python3 motor_control/yolo_odrive_jetson.py` → `python3 motor_control/drive/x2212_test/yolo_odrive_jetson.py`

- [ ] **Step 4: 미해결 4건 (line 232~239) 의 항목 4 (HALL 트랙 보드 행방) 갱신**

기존: `HALL 트랙 보드의 행방: README 에 D6374 + HALL 트랙 스크립트가 다수 ...`

교체:
```
4. **BL70200 실차 부착 후 캘리 검증**: HALL 트랙 코드 (`drive/bl70200/`) 의 캘리값
   pp=5/cpr=30 은 BL70200 과 일치하나, vbus 24 V 환경에서 `current_lim` /
   `vel_limit` 재튜닝 필요할 가능성. NVM 캘리 유효성 매부착 시 재확인.
```

- [ ] **Step 5: D6374 잔존 검사**

```bash
cd /home/light/Defence_Robot
git grep -ni "D6374" HANDOFF.md
```

Expected: historical commit message 외에 없음.

---

## Task 8: 변경 전체 검증

**Files:** 없음 (read-only 검증)

- [ ] **Step 1: 파일 매핑 검증**

```bash
cd /home/light/Defence_Robot
git status --short
```

Expected: 회수 11 개 (rename 15 + new 8 + can_setup.sh 1 + Dockerfile.jetson modified + docker-compose.jetson.yml modified) + 문서 modified 4 (README, CLAUDE.md, HANDOFF.md, .gitignore) ≈ 28 줄. 정확한 수치는 git status 로 확인.

```bash
git diff --stat --staged 2>/dev/null; git diff --stat
```

대략 25 ~ 30 개 파일 변경, ~100 ~ 300 줄 변화 예상.

- [ ] **Step 2: 신규 경로의 파일 존재 검증**

```bash
find motor_control/drive motor_control/steering motor_control/vision motor_control/sensors scripts -type f | sort
```

Expected (정확히 24 개):
```
motor_control/drive/bl70200/odrive_basic_test.py
motor_control/drive/bl70200/odrive_calibration.py
motor_control/drive/bl70200/odrive_closed_loop_test.py
motor_control/drive/bl70200/odrive_diff_drive_test.py
motor_control/drive/bl70200/odrive_position_hold_test.py
motor_control/drive/bl70200/odrive_velocity_hold_test.py
motor_control/drive/x2212_test/init_odrive.py
motor_control/drive/x2212_test/odrive_can_drive.py
motor_control/drive/x2212_test/odrive_can_setup.py
motor_control/drive/x2212_test/odrive_dualsense_test.py
motor_control/drive/x2212_test/odrive_dualsense_vel_test.py
motor_control/drive/x2212_test/odrive_yolo_object_tracking.py
motor_control/drive/x2212_test/yolo_odrive_jetson.py
motor_control/drive/x2212_test/yolo_odrive_motor_test.py
motor_control/sensors/us100_basic.py
motor_control/sensors/us100_robust.py
motor_control/steering/ak_control.py
motor_control/steering/calibrate_ak.py
motor_control/steering/run_ak.py
motor_control/steering/status_ak.py
motor_control/vision/setup_yolo_env.sh
motor_control/vision/yolo_cuda_stream.py
motor_control/vision/yolo_openvino_detection.py
scripts/can_setup.sh
scripts/recv_stream.sh
```

(25 개. 위 카운트 24 는 scripts/recv_stream.sh 가 기존이라 신규 24 + 기존 1.)

- [ ] **Step 3: motor_control/ 루트 평면 잔류물 검사**

```bash
ls motor_control/*.py 2>/dev/null; ls motor_control/*.sh 2>/dev/null
```

Expected: 빈 결과 (루트에 .py / .sh 파일 없음).

- [ ] **Step 4: D6374 전체 sweep**

```bash
git grep -ni "D6374"
```

Expected: 없음. 남아 있으면 그 파일 다시 수정.

- [ ] **Step 5: yolo_odrive_jetson 의 import 경로 점검**

`drive/x2212_test/yolo_odrive_jetson.py` 안의 `from yolo_cuda_stream import ...` 가
새 구조에서 깨짐 — `yolo_cuda_stream.py` 가 `vision/` 으로 이동했기 때문.

```bash
grep -n "from yolo_cuda_stream" motor_control/drive/x2212_test/yolo_odrive_jetson.py
```

발견되면 다음 중 하나로 수정:
- 옵션 A (권장 — 상대 import 회피): 코드에서 `sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "vision"))` 추가 후 기존 `from yolo_cuda_stream import ...` 유지.
- 옵션 B: `vision/yolo_cuda_stream.py` 의 helper 들을 import 경로 무관한 위치로 추출 (이번 scope 밖, 추후 plan).

옵션 A 적용 예 — `motor_control/drive/x2212_test/yolo_odrive_jetson.py` 의 import 블록을 다음으로 교체:

```python
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
```

검증:

```bash
cd motor_control/drive/x2212_test && python3 -c "
import sys; from pathlib import Path
sys.path.insert(0, str(Path('.').resolve().parents[1] / 'vision'))
import yolo_cuda_stream
print('import OK:', yolo_cuda_stream.__file__)
"
```

Expected: `import OK: /home/light/Defence_Robot/motor_control/vision/yolo_cuda_stream.py`.
(`ultralytics` / `cv2` 미설치라면 cv2 import 단계에서 실패해도 OK — 경로 해결만 확인.)

---

## Task 9: 커밋 + 푸시

**Files:** Git operations

- [ ] **Step 1: 모든 변경분 stage**

```bash
cd /home/light/Defence_Robot
git add -A
git status --short | head -40
```

Expected: 모든 변경분이 `A`, `M`, `R` 상태로 stage 됨. 잔여 `??` 없음 (HANDOFF.md 는 이번에
같이 commit — Task 7 에서 modified).

- [ ] **Step 2: commit**

```bash
git commit -m "$(cat <<'EOF'
refactor(motor_control): hw 기준 폴더 재구성 + Jetson 독자 작업분 통합 + 모터 인벤토리 정정

폴더 구조 (모터 hw 1차 분기 + 보조 평면):
- drive/x2212_test/  SunnySky X2212-13 + TLE5012B (테스트 — USB·CAN 양쪽)
- drive/bl70200/     BL70200 + 내장 HALL ×3 (실전)
- steering/          AK40-10 → AK45 (동일 API, CAN)
- vision/            모터 명령 없는 검출·스트리밍
- sensors/           US100 거리 (UART)

Jetson 측 독자 작업분 회수:
- ODrive CAN 트랙 (odrive_can_setup.py, odrive_can_drive.py)
- AK 조향 (ak_control.py + calibrate/status/run)
- US100 (us100_basic.py, us100_robust.py)
- can_setup.sh → scripts/
- Dockerfile.jetson python-can 추가, docker-compose.jetson.yml /dev/ttyTHS1 마운트

인벤토리 정정:
- D6374 라는 모터는 존재하지 않음 — 모든 문서·주석에서 제거
- 실전 구동: BL70200, 테스트: SunnySky X2212-13, 조향: AK40/AK45

문서:
- README, .claude/CLAUDE.md, HANDOFF, .gitignore 갱신
- spec: docs/specs/2026-05-20-motor-control-reorg-design.md
- plan: docs/plans/2026-05-20-motor-control-reorg-plan.md

EOF
)"
```

- [ ] **Step 3: push**

```bash
git push origin main
```

Expected: `main -> main` 성공 메시지.

- [ ] **Step 4: 원격 반영 확인**

```bash
git log origin/main --oneline -3
```

Expected: 새 commit 이 origin 의 HEAD.

---

## Task 10: Jetson 동기화

**Files:** Jetson 측 working tree

- [ ] **Step 1: Jetson 측 변경분 stash**

```bash
sshpass -p "0000" ssh zetin@jetson-orin.local 'cd ~/Defence_Robot && git stash -u -m "pre-reorg-2026-05-20"'
```

Expected: `Saved working directory and index state On main: pre-reorg-2026-05-20`.

- [ ] **Step 2: 원격 fetch + fast-forward pull**

```bash
sshpass -p "0000" ssh zetin@jetson-orin.local 'cd ~/Defence_Robot && git pull origin main'
```

Expected: Fast-forward, 새 폴더 구조 반영.

- [ ] **Step 3: stash 폐기 + working tree clean 확인**

```bash
sshpass -p "0000" ssh zetin@jetson-orin.local 'cd ~/Defence_Robot && git stash drop && git status'
```

Expected: `On branch main`, `Your branch is up to date`, `nothing to commit, working tree clean`
(모델 산출물 yolov8n.*.engine 등은 .gitignore 로 제외됨).

- [ ] **Step 4: Jetson 측 ~/can_setup.sh 정리**

```bash
sshpass -p "0000" ssh zetin@jetson-orin.local '
  if [ -f ~/can_setup.sh ] && diff -q ~/can_setup.sh ~/Defence_Robot/scripts/can_setup.sh > /dev/null 2>&1; then
    echo "homedir 의 can_setup.sh 가 repo 의 사본과 동일 — 제거"
    rm ~/can_setup.sh
  else
    echo "homedir 의 can_setup.sh 가 repo 사본과 다르거나 없음 — 수동 확인"
    ls -la ~/can_setup.sh 2>&1
  fi
'
```

Expected: "homedir 의 can_setup.sh ... 제거".

- [ ] **Step 5: Jetson 측 import dry-run**

```bash
sshpass -p "0000" ssh zetin@jetson-orin.local '
  cd ~/Defence_Robot &&
  sudo docker compose -f docker/docker-compose.jetson.yml exec -T powertrain bash -lc "
    cd /workspace &&
    python3 -c \"import sys; sys.path.insert(0, \\\"motor_control/vision\\\"); from yolo_cuda_stream import resolve_model; print(\\\"OK vision\\\")\" &&
    python3 -c \"import sys; sys.path.insert(0, \\\"motor_control/steering\\\"); import ak_control; print(\\\"OK steering\\\")\" &&
    python3 -c \"import can; print(\\\"OK python-can\\\", can.__version__)\"
  "
'
```

Expected:
```
OK vision
OK steering
OK python-can <version>
```

컨테이너가 안 떠있으면 (`exec` 실패) 컨테이너 기동 후 재시도:

```bash
sshpass -p "0000" ssh zetin@jetson-orin.local '
  cd ~/Defence_Robot &&
  sudo docker compose -f docker/docker-compose.jetson.yml up -d
'
```

- [ ] **Step 6: 노트북 측에서도 동일 import 검증 (Docker)**

```bash
cd /home/light/Defence_Robot
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "
  cd /workspace &&
  python3 -c 'import sys; sys.path.insert(0, \"motor_control/vision\"); import yolo_cuda_stream; print(\"OK vision\")' &&
  python3 -c 'import sys; sys.path.insert(0, \"motor_control/steering\"); import ak_control; print(\"OK steering\")'
"
```

Expected: 2 줄 OK. `ak_control` 은 `python-can` 필요 — x86 Dockerfile 에는 미설치이므로
실패 가능. 그러면 Step 6 은 vision/ 만 검증하고 steering/ 은 skip.

---

## Self-Review Notes

- **Spec coverage**: 모든 spec 의 결정 사항 → task 매핑 됨. 검증 기준 (4 항목) → Task 8 에 모두 포함.
- **Placeholder scan**: 없음.
- **Type consistency**: 파일명 / 경로 / rename 매핑이 spec 와 plan 사이 일치 확인. AK rename
  은 `ak40_<X>` → `<X>_ak` (`status_ak40.py` → `status_ak.py`) 패턴 일관.
- **숨은 dependency**: `yolo_odrive_jetson.py` 의 `from yolo_cuda_stream import ...` 가 폴더
  분리로 깨짐 — Task 8 Step 5 에서 명시적으로 수정. spec 에는 누락된 부분이라 plan 에서 추가.
- **Jetson stash drop 안전성**: Task 10 Step 3 의 `git stash drop` 은 Step 2 의 fast-forward
  pull 이 성공한 이후라 안전. 만약 conflict 가 났다면 Step 3 전에 멈춰서 수동 처리.
