# Power Train SW

ZETIN 6륜 로커-보기(rocker-bogie) 방위 로봇의 파워트레인 SW 저장소. 두 트랙은
서로 독립적으로 동작한다.

| 트랙 | 폴더 | 역할 |
| --- | --- | --- |
| 파라미터 최적화 | `parameter_calc/` | 형상 파라미터 최적화 (v4: 15차원·7지형, MATLAB / NumPy / JAX-CUDA) |
| 모터 제어 | `motor_control/` | ODrive · AK 조향 · DualSense · YOLO · US100 · 노트북-Pi 네트워킹 |

> `parameter_calc/`는 개발 서버 검증본을 그대로 옮긴 것으로 결과물(`*.pkl`)을 신뢰할 수 있는 기준 코드.

---

## 저장소 구조

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
│   │   └── status_ak.py             # CAN RX 디버깅
│   ├── vision/                      # 비전 단독 (모터 명령 없음)
│   │   ├── yolo_openvino_detection.py  # x86 OpenVINO
│   │   ├── yolo_cuda_stream.py         # Jetson CUDA/TRT + UDP H.264
│   │   └── setup_yolo_env.sh           # x86 conda (Docker 권장)
│   ├── sensors/                     # 센서 (UART /dev/ttyTHS1)
│   │   ├── us100_basic.py           # 0x55 기본
│   │   └── us100_robust.py          # Jetson UART 버그 우회
│   ├── corner_module/               # 코너 모듈 (조향+구동 협조 제어 + DualSense 텔레옵)
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
├── motor_gui/                       # 웹 진단 GUI (FastAPI + 트랜스포트 추상화, CAN/USB 모터)
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

> 모든 모터 제어 스크립트는 `axis1` 사용으로 통일.

---

## parameter_calc — 형상 파라미터 최적화

형상 파라미터 벡터 `x`를 여러 지형에 대해 평가, 모터 토크·안정성(ZMP, TOI)·미끄럼·전류·승차감(S/N) 가중합 비용을 최소화. MATLAB 원본을 NumPy/SciPy(CPU)와 JAX/CUDA(GPU)로 포팅, 동일한 물리식 공유.

**현재 권위본은 v4** (`python_gpu_triangle/`, 면-기준 물리 수정본): **15차원 · 7종 지형**(계단 / 나무 블록 / 거친 노면 / 단차 / 곡면 경사 / 15° / 30° 경사), 삼각형·프레임 모드 동시 탐색. v3(`python_gpu/`)는 14차원 · 4종 지형.

### 빠른 실행

```bash
# v4 (현재 권위본, GPU) — 15차원·7지형, 삼각형+프레임 동시 탐색
cd parameter_calc/python_gpu_triangle/
JAX_PLATFORM_NAME=gpu python ZETIN_JointOptSearch_v4_gpu.py
python ZETIN_Animation_v3.py    # 결과 .pkl → 지형별 주행 mp4

# v3 (GPU, 참조 — RTX 3090 기준 약 3–10분, 첫 실행 JIT 30–60초)
cd parameter_calc/python_gpu/
pip install -r requirements_gpu.txt
JAX_PLATFORM_NAME=gpu python ZETIN_JointOptSearch_v3_gpu.py

# CPU (베이스라인, 약 1시간)
cd parameter_calc/python/
pip install -r requirements.txt
python ZETIN_JointOptSearch_v3.py
python ZETIN_Animation_v3.py    # 사전 계산된 .pkl 시각화

# MATLAB 원본 (R2020b+, Optimization Toolbox)
matlab -batch "run('parameter_calc/matlab/ZETIN_JointOptSearch_v3.m')"
```

### 모듈 책임

| 모듈 (CPU / GPU) | 역할 |
| --- | --- |
| `wpos.py` / `wpos_jax.py` | 순기구학: 조인트 각도 → 5점 좌표 (Wf, Wm, Wr, Pb, CG) |
| `ceq.py` / `ceq_jax.py` | 휠-지형 접촉 구속 방정식 |
| `kin_sim.py` / `newton_solver.py` | 역기구학 (CPU `fsolve` / GPU 배치 Newton + `vmap`) |
| `calc_envelope.py` / `_jax.py` | 휠 반경에 대한 지형 Minkowski 합 |
| `calc_dynamics.py` / `_jax.py` | 모터 토크 / 부하 불균형 |
| `calc_stability.py` / `_jax.py` | ZMP, Tip-Over Index, 들림 비율, 충돌 |
| `calc_metrics.py` / `_jax.py` | 토크 신호 S/N 비 (dB) |
| `gen_terrain.py` | 지형 프로파일 (v4 7종 / v3 4종, CPU/GPU 공유) |

상세 파이프라인, 파라미터 정의(v4 15차원 / v3 14차원), 목적함수 가중치, GPU 버그 히스토리는
[`parameter_calc/CLAUDE.md`](parameter_calc/CLAUDE.md). 수정 전 필독.

---

## motor_control — 실차 모터 제어

스크립트는 가정하는 **센서·모터 모델**에 따라 세 hw 라인으로 나뉘며, **반드시 같은
트랙 안의 스크립트만 함께 사용**해야 한다. 한 트랙으로 캘리한 ODrive를 다른
트랙 스크립트로 구동하면 게인·전류 한계·인코더 해석이 달라 폭주/과전류 위험.

| 트랙 | 모터 (테스트 → 실전) | 센서 | 통신 | 폴더 |
| --- | --- | --- | --- | --- |
| **구동 (실전)** | BL70200 | 내장 HALL ×3 (pp=5, cpr=30) | ODrive USB | `drive/bl70200/` |
| **구동 (테스트)** | SunnySky X2212-13 | TLE5012B 16384 CPR (외장) | ODrive USB · CAN | `drive/x2212_test/` |
| **조향** | AK40-10 → AK45-36 (동일 API) | 내장 | CAN (socketcan can0) | `steering/` |

ODrive 펌웨어 v0.5.x (CAN 트랙은 fw-v0.5.6 검증), 테스트 axis = `axis1`.
BL70200 트랙의 캘리값 `pp=5, cpr=30` 은 BL70200 hw 와 일치 — 실차 부착 시 vbus 24 V
환경에서 `current_lim` / `vel_limit` 만 재확인.

---

### 트랙: 구동 모터 (실전 — BL70200, `drive/bl70200/`)

BL70200 + 내장 HALL ×3. ODrive HALL 모드 사용, NVM 캘리 저장 후 재사용.
cpr=30 으로 낮아 게인 보수적 + vel-estimate 필터 bandwidth 낮춰 노이즈 억제.

#### 캘리브레이션

| 파일 | 역할 |
| --- | --- |
| `drive/bl70200/odrive_calibration.py` | `axis1`에 HALL 모드·`pole_pairs=5`·`cpr=30`·`HIGH_CURRENT` 강제 후 풀 캘리(모터 → 폴라리티 → 오프셋), 게인 세팅, NVM 저장. **HALL 트랙 모든 작업 전 1회 필수** |
| `drive/bl70200/odrive_diff_drive_test.py` | 동일 HALL 설정 + `ENCODER_HALL_POLARITY_CALIBRATION` + 10회전 자체 검증 |

#### 단축 동작 검증 (사전 캘리 필요)

| 파일 | 제어 모드 | 검증 내용 |
| --- | --- | --- |
| `drive/bl70200/odrive_closed_loop_test.py` | 위치 / 기본 | 폐루프 진입 + 2회전 |
| `drive/bl70200/odrive_basic_test.py` | 위치 / `TRAP_TRAJ` | 게인 + 사다리꼴 궤적 (bandwidth 50, pos 3.0, vel 0.04) |
| `drive/bl70200/odrive_position_hold_test.py` | 위치 / `PASSTHROUGH` | 외력 인가 시 자동 복귀 (30 초 모니터) |
| `drive/bl70200/odrive_velocity_hold_test.py` | 속도 / `VEL_RAMP` | 8.0 rev/s 유지. **HALL 노이즈 억제 게인** (bandwidth 20, vel 0.05, integrator 0.1) |

#### 절차

1. `python drive/bl70200/odrive_calibration.py`
2. `python drive/bl70200/odrive_closed_loop_test.py` — 2회전 + axis.error 0x0 확인
3. 용도별 단축 검증

---

### 트랙: 구동 모터 (테스트 — SunnySky X2212-13, `drive/x2212_test/`)

외장 TLE5012B 증분 엔코더 (16384 CPR) + SunnySky X2212-13 (14극 BLDC, pp=7).
**ODrive 검증·게인 튜닝·비전 통합 PoC 용도**. 실차 운영은 BL70200 트랙으로 이전.
ODrive 통신은 USB · CAN 둘 다 지원 — 같은 모터·캘리값에 통신 매체만 다름.

#### 환경 / 셋업

| 파일 | 역할 |
| --- | --- |
| `drive/x2212_test/init_odrive.py` | **1회 실행** ODrive NVM 셋업 (USB): pp=7, cpr=16384, 게인, `pre_calibrated=True`, `startup_closed_loop_control=True` 저장 + reboot |
| `drive/x2212_test/odrive_can_setup.py` | CAN NVM 셋업 — `init_odrive.py` 의 CAN 버전, **안전: 부팅 시 IDLE 모드** (startup_closed_loop=False). baudrate 250 kbps, node_id=1 |
| `drive/x2212_test/odrive_can_drive.py` | CAN 폐루프 진입 + 위치 5.0 → 0.0 데모. python-can socketcan |

#### DualSense 직결 텔레옵

| 파일 | 제어 / 입력 | 매핑 |
| --- | --- | --- |
| `drive/x2212_test/odrive_dualsense_test.py` | `POSITION_CONTROL` | 좌스틱 → 누적 목표 위치 |
| `drive/x2212_test/odrive_dualsense_vel_test.py` | `VELOCITY_CONTROL` + `PASSTHROUGH` | 트리거 → 속도 명령 (위치 필터 우회) |

#### YOLO 비전 통합

| 파일 | 역할 |
| --- | --- |
| `drive/x2212_test/yolo_odrive_motor_test.py` | YOLO 박스 중심 → 위치 명령 (`SCALE_FACTOR=10.0`) |
| `drive/x2212_test/odrive_yolo_object_tracking.py` | 운영 버전. 대상 = COCO `bottle`, `SCALE_FACTOR=5.0`, `MAX_TURNS=20.0`, `POS_DEADZONE=0.05`, **POS_FILTER**로 카메라 노이즈 완화. 캘리 상태 분기(이미 캘리되어 있으면 스킵) |
| `drive/x2212_test/yolo_odrive_jetson.py` | Jetson CUDA/TRT 비전 + USB ODrive 추종 (실차 통합 PoC). `--stream` 으로 영상 송신 동시. 폐루프 진입 전 `input_pos=origin` 박아 jump 방지, `MAX_TURNS=2.0` 안전 한계 |

### 트랙: 조향 (AK40 / AK45, `steering/`)

CubeMars AK 시리즈 (AK40-10 테스트 → AK45-36 실전, 동일 API). CAN bus
(socketcan `can0`, motor_id=10) 로 위치·속도·브레이크 제어. 짐벌 운동학 함수도
포함하지만 본 프로젝트에서는 **조향 서보** 로 사용.

| 파일 | 역할 |
| --- | --- |
| `steering/ak_control.py` | `AK` 클래스 + `CANSession` 컨텍스트 매니저. `send_pos_out(out_deg)` 출력축 직결, `move_rel_out()` 상대 위치, `send_rpm_out()` 속도, `stop()` 안전정지 |
| `steering/calibrate_ak.py` | 기어비 (10/1) 결정용 1 회성 — `send_pos_raw(36°)` 결과로 출력축 기준 판별 |
| `steering/status_ak.py` | CAN RX 디버깅 — 모든 응답 패킷 hex dump |

CAN 사전 준비: 호스트에서 `bash scripts/can_setup.sh` 로 can0 1 Mbps 셋업
(mttcan + devmem mux + bitrate). Dockerfile.jetson 에 `python-can` 포함.

### 보조: 비전 단독 (`vision/`)

모터 명령 없는 검출·스트리밍 전용. 본 트랙에서 만든 결과 (검출 박스 좌표) 가
구동 트랙의 추종 입력이 될 수 있음.

| 파일 | 역할 |
| --- | --- |
| `vision/yolo_openvino_detection.py` | Intel OpenVINO YOLOv8n (V4L2, 1280×720) + 임의 카메라 파라미터 3D 변환 (x86 한정) |
| `vision/yolo_cuda_stream.py` | YOLOv8 (PyTorch CUDA / TensorRT FP16) + GStreamer H.264 RTP UDP 송신. Jetson 컨테이너 안에서 실행, 수신은 `scripts/recv_stream.sh` |
| `vision/setup_yolo_env.sh` | x86 yolo_env 콘다 환경 (Docker 권장으로 deprecated) |

### 보조: 센서 (`sensors/`)

| 파일 | 센서 | 비고 |
| --- | --- | --- |
| `sensors/us100_basic.py` | US100 초음파 거리 | UART `/dev/ttyTHS1`, 9600 bps, `0x55` 트리거 + 2 byte 응답 |
| `sensors/us100_robust.py` | 동일 | Jetson UART TX 전압 떨림 버그를 `0xFF`×8 prefix 로 우회 |

### 코너 모듈 (`corner_module/`)

로커보기 코너 1개(조향 AK + 구동 ODrive)를 묶어 `(조향각°, 구동속도 turns/s)` 로 협조 제어하는 재사용 라이브러리 + DualSense 텔레옵. 트랜스포트 무관 액추에이터 추상화(USB→CAN 전환 시 드라이버만 교체) → **미래 4WS 애커만 키네마틱스 레이어의 빌딩블록**. 안전(워치독·estop·과전류/fault 트립·폐루프 점프방지) 내장.

| 파일 | 역할 |
| --- | --- |
| `corner_module/corner_module.py` | `CornerModule` — 상태머신·안전·협조 제어 (핵심) |
| `corner_module/actuator.py` | `Actuator` / `SteerActuator` / `DriveActuator` 인터페이스 (ABC) |
| `corner_module/steer_ak40.py` · `drive_odrive_usb.py` · `drive_odrive_can.py` | 조향(AK, CAN) · 구동(ODrive USB 현재 / CAN 추후) 드라이버 |
| `corner_module/fake.py` | 무하드웨어 단위테스트용 더블 |
| `corner_module/teleop_dualsense.py` | DualSense 텔레옵 — `python3 -m corner_module.teleop_dualsense` |

단위테스트 24개 + 실하드웨어 HIL(조향·구동·통합·텔레옵) 검증 완료. 설계/계획: `docs/specs/2026-05-25-corner-module-controller-design.md`, `docs/plans/2026-05-25-corner-module-controller-plan.md`.

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

#### 노트북 ↔ Pi 네트워크 텔레옵

`laptop/` (노트북) ↔ `pi/` (라즈베리파이) 1:1 짝. 프로토콜은 모든 서버 공통 —
줄바꿈 단위 텍스트 속도값(`%.4f\n`)을 TCP `:9000` 으로. 클라이언트는 어느
서버와도 호환되지만 `MAX_VEL` / 트리거 축이 같은 짝꿍을 사용해야 캘리·게인이
일치.

| 노트북 측 (`laptop/`) | 라즈베리파이 측 (`pi/`) | 용도 |
| --- | --- | --- |
| `laptop_client_basic.py` | `pi_server_basic.py` | 단순 텔레옵 (영상 X), MAX 5.0 rev/s, 매 실행 풀 캘리 |
| `laptop_client_velocity.py` | `pi_server_velocity.py` | 운영 — Python 50 Hz 소프트 램프 + 자동 재부팅/재캘리. `--detect` 로 트리거 축 인덱스 탐색 |
| `laptop_client_velocity.py` | `pi_server_position.py` | 위치 제어 — Pi 측에서 속도값을 적분해 input_pos 로, 트리거 떼면 위치 홀딩 |
| `laptop_client_video.py` | `pi_server_video.py` | 텔레옵 + GStreamer JPEG 영상 수신 (`:5000`), Pi 캘리 종료까지 자동 재연결 |

> `pi_server_velocity.py` / `pi_server_position.py` 는 **14극 BLDC + TLE5012B
> 16384 CPR + 2.0 Ω 브레이크 저항** 가정.
> Pi IP `192.168.1.91` 하드코딩 — `laptop_client_*.py` 의 `PI_HOST` 수정.
> Pi 측 카메라 PoC: `pi/camera/cam_snapshot.py`, `cam_udp_stream.py`.

---

## motor_gui — 웹 진단 GUI

브라우저 기반 모터 진단·튜닝 GUI. FastAPI 백엔드(100 Hz 워커 + 텔레메트리 WebSocket + CSV/Parquet 레코더) + 트랜스포트 추상화로 AK·ODrive 를 USB/CAN 으로 제어. `motor_control/` 을 재사용(예: `ak_control`)하되 역의존은 금지.

| 트랙 (`--track`) | 대상 |
| --- | --- |
| `fake` | 시뮬 (하드웨어 없이) |
| `usb` | ODrive (USB) |
| `ak` | AK 조향 (CAN can0, id 10) |
| `odrive_can` | ODrive (CAN node 1) |
| `can` | 범용 CAN 버스 |

```bash
# powertrain 컨테이너 안에서
python3 -m motor_gui.backend.server --track ak    # 또는 usb / odrive_can / fake
# 브라우저: http://<host>:8000   (network_mode host → 포트 매핑 불필요)
```

구조: `backend/server.py`(FastAPI) · `backend/worker.py`(100 Hz 샘플) · `backend/recorder.py` · `backend/transport/`(`base` ABC + `ak_device`·`odrive_can_device`·`usb_odrive`·`can_device`·`fake`) · `frontend/`(uPlot 실시간 플롯). 단위·통합 테스트 `motor_gui/tests/`. 자세한 건 `motor_gui/README.md`.

---

## Docker 환경

팀원이 동일한 환경에서 `motor_control/` 스크립트를 실행하고, 그대로 Jetson Orin
Nano 에 배포할 수 있도록 컨테이너 정의를 제공한다. 코드는 호스트에 두고
`/workspace` 로 bind mount — 이미지를 다시 빌드하지 않고 수정/실행 가능.

| 파일 | 대상 |
| --- | --- |
| `docker/Dockerfile` | x86_64 개발 이미지 (Ubuntu 22.04 + Python 3.10 + ODrive · pygame · OpenCV · ultralytics · OpenVINO) |
| `docker/Dockerfile.jetson` | Jetson Orin Nano 배포 (`dustynv/l4t-pytorch:r36.4.0` 기반, OpenVINO 제외) |
| `docker/docker-compose.yml` | 기본 (CPU / Intel iGPU). 호스트 네트워크 + `/dev` 마운트 + X11 패스 |
| `docker/docker-compose.gpu.yml` | x86 NVIDIA GPU 오버레이. `nvidia-container-toolkit` 호스트 설치 필요 |
| `docker/docker-compose.jetson.yml` | 젯슨 전용. `runtime: nvidia` (JetPack 기본 포함) |

### x86 (개발용 노트북)

```bash
# 1) X11 권한 (cv2.imshow 창을 호스트로 띄우려면)
xhost +local:docker

# 2) 빌드 + 백그라운드 기동
docker compose -f docker/docker-compose.yml up -d --build

# 3) 컨테이너 진입 (이후 motor_control 스크립트 실행)
docker compose -f docker/docker-compose.yml exec powertrain bash
# 컨테이너 안에서:
#   cd /workspace
#   python3 motor_control/drive/bl70200/odrive_calibration.py
```

NVIDIA GPU 가 있는 노트북:

```bash
docker compose -f docker/docker-compose.yml -f docker/docker-compose.gpu.yml up -d --build
```

### Jetson Orin Nano (배포)

JetPack 6.x (L4T r36.x) 기준. JetPack 에 `nvidia-container-runtime` 이 기본 포함
되어 있으므로 추가 설정 없이 동작한다. 베이스 이미지는 `dustynv/l4t-pytorch:r36.4.0`
(CUDA + cuDNN + TensorRT + ARM PyTorch 포함) — Docker Hub 에서 바로 pull.

```bash
# 젯슨 위에서
git clone https://github.com/lightminn/power-train-sw.git
cd power-train-sw
sudo docker compose -f docker/docker-compose.jetson.yml up -d --build
sudo docker compose -f docker/docker-compose.jetson.yml exec powertrain bash
```

비전 단독 + 영상 스트리밍 동작 확인:

```bash
# (Jetson 컨테이너 안)
python3 motor_control/vision/yolo_cuda_stream.py \
        --backend trt --host <노트북IP> --width 640 --height 480

# (노트북, 호스트 OS)
scripts/recv_stream.sh 5000   # GStreamer 창에 검출 박스 오버레이된 영상 표시
```

설계·검증 기록은 `docs/specs/2026-05-08-jetson-yolo-stream-design.md` 와
`docs/plans/2026-05-08-jetson-yolo-stream-plan.md` 참고.

> **주의**: `yolo_openvino_detection.py` 등 OpenVINO 사용 스크립트는 젯슨에서
> 그대로 동작하지 않는다 — 모델을 TensorRT 엔진으로 재변환하거나 PyTorch+CUDA
> 경로(ultralytics 기본)로 바꿔야 한다. ODrive · DualSense · 네트워크 텔레옵
> 부분은 그대로 동작.
> NVENC GStreamer 플러그인(`nvv4l2h264enc`) 은 dustynv 컨테이너 GStreamer 1.20+
> 와 L4T plugin (1.14 ABI) 불일치로 현재 미사용 — `yolo_cuda_stream.py` 는
> 소프트웨어 인코더(`openh264enc`) 로 송신. 720p/30fps 까지 ARM A78AE 6코어로
> 충분.

### 호스트 측 사전 준비

| 항목 | 내용 |
| --- | --- |
| ODrive udev 규칙 | 호스트에 `/etc/udev/rules.d/91-odrive.rules` 가 있어야 일반 사용자 권한으로 인식. 컨테이너는 `/dev` 마운트로 호스트 디바이스를 공유 |
| Wayland 환경 | Arch/Ubuntu 가 Wayland 기본이면 XWayland 가 떠 있어야 cv2 창 표시 가능. `echo $XDG_SESSION_TYPE` 으로 확인 |
| ODrive · DualSense · 카메라 | 호스트에 USB 로 꽂으면 `/dev/bus/usb`, `/dev/input/js*`, `/dev/video*` 가 자동으로 컨테이너에 노출됨 |

### 컨테이너 안에서 안 되는 것

- `setup_yolo_env.sh` (이미 컨테이너에 환경이 구성되어 있어 불필요)
- 호스트 GUI 도구(예: `odrivetool` 의 일부 GUI 기능) — CLI 는 동작

---

## 기여 가이드

- `parameter_calc/` 수정 전 `parameter_calc/CLAUDE.md`의 GPU 버그 히스토리 섹션 필독.
- `motor_control/` 스크립트는 독립 실행형 원칙 유지 — 공용 모듈 분리는 사전 합의.
- **BL70200 트랙(HALL 모드) / X2212 트랙(엔코더 모드) 을 한 ODrive 에서 번갈아 쓰지 말 것.** NVM에 남은 캘리 설정이 의도치 않게 적용된다. 교차 사용 시 `drive/bl70200/odrive_calibration.py` 에서 모드 강제 재설정 후 시작.
- 모든 테스트는 `axis1`. axis0 사용 금지.
- 결과 파일(`*.pkl`, `*.mat`, `*.mp4`, `fig*.png`)은 서버 검증본 — 의도 없이 덮어쓰지 말 것.

## 라이선스 / 연락

내부 프로젝트. 외부 공개·재배포 전 ZETIN 측 확인.
