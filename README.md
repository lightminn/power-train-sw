# Power Train SW

ZETIN 6륜 로커-보기(rocker-bogie) 서스펜션 방위 로봇의 파워트레인 소프트웨어 저장소.
설계 측 코드와 실차 제어 측 코드가 한 저장소에 함께 들어 있으며, 두 트랙은 서로 독립적으로 동작한다.

| 트랙 | 폴더 | 역할 |
| --- | --- | --- |
| 파라미터 최적화 | `parameter_calc/` | 14차원 로커-보기 형상 파라미터 최적화 (MATLAB / NumPy / JAX-CUDA) |
| 모터 제어 | `motor_control/` | ODrive 기반 실차 구동, DualSense 텔레옵, YOLO 비전 트래킹, 노트북-Pi 네트워킹 |

> 본 저장소의 `parameter_calc/` 트리는 개발 서버에서 검증된 빌드를 그대로 옮긴 것으로,
> 결과물(`*.pkl`)을 신뢰할 수 있는 기준 코드입니다.

---

## 저장소 구조

```
.
├── parameter_calc/                  # 형상 파라미터 최적화 (오프라인 시뮬레이션)
│   ├── CLAUDE.md                    # 이 트랙의 상세 문서 (필독)
│   ├── matlab/                      # MATLAB 원본 레퍼런스
│   │   ├── functions/               # 물리/기하 함수 (calc_*.m, kin_sim.m 등)
│   │   ├── ZETIN_JointOptSearch_v3.m   # 최적화 메인
│   │   ├── ZETIN_Animation_v3.m        # 결과 애니메이션
│   │   └── zetin_optimal_params_v3.mat # 사전 계산 결과
│   ├── python/                      # CPU 포팅 (NumPy / SciPy)
│   │   ├── functions/               # CPU용 모듈 (wpos, ceq, kin_sim ...)
│   │   ├── ZETIN_JointOptSearch_v3.py
│   │   ├── ZETIN_Animation_v3.py
│   │   ├── requirements.txt
│   │   ├── zetin_optimal_params_v3.pkl  # 최적 파라미터 (pickle)
│   │   ├── fig*.png                 # 결과 그래프
│   │   └── ZETIN_animation_*.mp4    # 지형별 애니메이션
│   ├── python_gpu/                  # GPU 포팅 (JAX / CUDA 12.x) — v3
│   │   ├── functions/               # JAX 모듈 (newton_solver, *_jax.py)
│   │   ├── ZETIN_JointOptSearch_v3_gpu.py
│   │   ├── requirements_gpu.txt
│   │   ├── README_GPU.md
│   │   └── zetin_optimal_params_v3.pkl
│   ├── python_gpu_triangle/         # GPU 변형 — triangle 모드 한정 (v4)
│   │   ├── ZETIN_JointOptSearch_v4_gpu.py
│   │   ├── test_v4.py
│   │   └── zetin_optimal_params_v4.pkl
│   └── scripts/
│       ├── run_gpu.sh               # SLURM/서버 실행 래퍼 (v3)
│       └── run_gpu_triangle.sh      # 동일 (v4)
└── motor_control/                   # ODrive · DualSense · YOLO 실차 제어
    │   ─── [HALL 센서 트랙: D6374 + 내장 HALL, 5 pole pairs, HIGH_CURRENT] ───
    ├── odrive_calibration.py        # HALL 모드 캘리브레이션 (필수 1회, NVM 저장)
    ├── odrive_diff_drive_test.py    # HALL 폴라리티 캘리 + 차동 구동 사전 점검
    ├── odrive_basic_test.py         # 사전 캘리 전제, 위치 제어 + TRAP_TRAJ 동작 확인
    ├── odrive_closed_loop_test.py   # 사전 캘리 전제, 폐루프 진입 + 2회전 테스트
    ├── odrive_position_hold_test.py # 사전 캘리 전제, 현재 위치 홀딩
    ├── odrive_velocity_hold_test.py # 사전 캘리 전제, HALL 노이즈 억제 게인으로 속도 유지
    │   ─── [엔코더 트랙: 외장 증분형 엔코더 + ODrive 기본 모터 모델] ───
    ├── setup_yolo_env.sh            # yolo_env 콘다 환경 자동 설치 (PyTorch / OpenVINO / ultralytics 등)
    ├── yolo_openvino_detection.py   # YOLO 검출 단독 (3D 좌표 변환, 모터 명령 없음)
    ├── odrive_dualsense_test.py     # FULL_CALIBRATION_SEQUENCE → DualSense 위치 제어
    ├── odrive_dualsense_vel_test.py # FULL_CALIBRATION_SEQUENCE → DualSense 속도 제어
    ├── yolo_odrive_motor_test.py    # FULL_CALIBRATION_SEQUENCE → YOLO 추종 (테스트 변형)
    ├── odrive_yolo_object_tracking.py # FULL_CALIBRATION_SEQUENCE → YOLO 추종 (운영 버전)
    ├── robot_client.py              # 노트북: DualSense → TCP → Pi
    ├── robot_client2.py             # 위와 동일, 트리거→속도 단순 매핑 변형
    └── robot_laptop.py              # 노트북: 명령 송신 + GStreamer 영상 수신
```

> **테스트 axis는 axis1로 통일.** 모든 단축 테스트·캘리·운영 스크립트는
> `axis1`만 대상으로 한다.

---

## parameter_calc — 형상 파라미터 최적화

### 무엇을 푸는가

로커-보기 서스펜션의 14차원 형상 파라미터 벡터 `x`를 4종 지형(계단 / 나무 블록 /
거친 노면 / 단차)에 대해 평가하여, 모터 토크·안정성(ZMP, TOI)·승차감(S/N)을
가중합한 비용을 최소화. 원본 알고리즘은 MATLAB이며, NumPy/SciPy CPU 포트와
JAX/CUDA GPU 포트가 동일한 물리식을 공유한다.

### 빠른 실행

```bash
# GPU (권장, RTX 3090 기준 약 3–10분)
cd parameter_calc/python_gpu/
pip install -r requirements_gpu.txt
JAX_PLATFORM_NAME=gpu python ZETIN_JointOptSearch_v3_gpu.py
# 첫 실행은 JIT 컴파일로 30–60초 워밍업 발생

# CPU (베이스라인, 약 1시간)
cd parameter_calc/python/
pip install -r requirements.txt
python ZETIN_JointOptSearch_v3.py

# 시각화 (사전 계산된 .pkl 사용)
python ZETIN_Animation_v3.py

# MATLAB 원본 (R2020b+, Optimization Toolbox 필요)
matlab -batch "run('parameter_calc/matlab/ZETIN_JointOptSearch_v3.m')"
```

### 모듈 책임 (요약)

| 모듈 (CPU / GPU) | 역할 |
| --- | --- |
| `wpos.py` / `wpos_jax.py` | 순기구학: 조인트 각도 → 5점 좌표 (Wf, Wm, Wr, Pb, CG) |
| `ceq.py` / `ceq_jax.py` | 휠-지형 접촉 구속 방정식 |
| `kin_sim.py` / `newton_solver.py` | 역기구학: 지형 프로파일 → 조인트 각도 (CPU `fsolve` / GPU 배치 Newton + `vmap`) |
| `calc_envelope.py` / `_jax.py` | 휠 반경에 대한 지형 Minkowski 합 |
| `calc_dynamics.py` / `_jax.py` | 모터 토크 / 부하 불균형 계산 |
| `calc_stability.py` / `_jax.py` | ZMP, Tip-Over Index, 휠 들림 비율, 충돌 |
| `calc_metrics.py` / `_jax.py` | 토크 신호 S/N 비 (dB) |
| `gen_terrain.py` | 4종 지형 프로파일 생성 (CPU/GPU 공유) |

상세 파이프라인, 14파라미터 정의, 목적함수 가중치, GPU 가속 전략, 발견된 GPU
버그 히스토리는 [`parameter_calc/CLAUDE.md`](parameter_calc/CLAUDE.md)에 정리되어
있다. 이 트랙을 수정하기 전에 반드시 읽을 것.

---

## motor_control — 실차 모터 제어 / 텔레옵 / 비전

`motor_control/` 의 스크립트는 가정하는 **센서·모터 모델**에 따라 두 트랙으로
나뉜다. **반드시 같은 트랙 안의 스크립트만 함께 사용**해야 한다 — 한 트랙에서
캘리한 ODrive를 다른 트랙 스크립트로 구동하면 게인·전류 한계·인코더 해석이
일치하지 않아 폭주/과전류 위험이 있다.

| 트랙 | 센서 | 모터 모델 가정 | 캘리 방식 |
| --- | --- | --- | --- |
| **HALL** | 모터 내장 홀센서 (3선) | D6374 150 KV (`MotorType.HIGH_CURRENT`, 5 pole pairs, cpr = 6 × 5 = 30) | `EncoderMode.HALL` 명시 + 폴라리티 → 오프셋 캘리, NVM 저장 후 재사용 |
| **엔코더** | 외장 증분형(쿼드라처) 엔코더 | ODrive 기본 모터 설정 (모터 타입·전류 한계 비명시) | 매 실행마다 `FULL_CALIBRATION_SEQUENCE` 호출 |

> **공통 가정**: ODrive 펌웨어 v0.5.x, 24 V DC 입력, 브레이크 저항 장착,
> 테스트 axis = `axis1`.

---

### 트랙 A — HALL 센서 + D6374 (실차 운영용)

D6374에 내장된 홀센서 3선을 ODrive HALL 입력에 직결한 구성. 별도 인코더가
없으므로 cpr이 30으로 매우 낮고, 그에 맞춰 게인을 보수적으로 잡고 vel
estimate 필터(엔코더 bandwidth)를 낮춰 노이즈를 억제한다.

#### 환경 / 캘리브레이션

| 파일 | 무엇을 하는가 | 비고 |
| --- | --- | --- |
| `odrive_calibration.py` | `axis1`에 `EncoderMode.HALL` · `pole_pairs=5` · `cpr=30` · `MotorType.HIGH_CURRENT` 강제 설정 후 풀 캘리 (모터 → 홀 폴라리티 → 오프셋), 게인 세팅, NVM 저장. 부팅 자동 실행도 `axis1` 비활성화 | 안전 헬퍼(`safe_set`, `wait_idle`, `dump_errors`) 포함. **HALL 트랙 모든 작업 전 1회 필수** |
| `odrive_diff_drive_test.py` | `axis1`에 동일한 HALL 설정 + `ENCODER_HALL_POLARITY_CALIBRATION` 시퀀스. 펌웨어/하드웨어 버전, GPIO9–11 모드, vbus 덤프, 10회전 자체 검증 | 차동 구동 사전 점검 단축 버전 |

#### 단축 동작 검증 (사전 캘리 필요, 모두 `axis1`)

| 파일 | 제어 모드 / 입력 | 검증 내용 |
| --- | --- | --- |
| `odrive_closed_loop_test.py` | 위치 / 기본 | 폐루프 진입 + 2회전 동작 |
| `odrive_basic_test.py` | 위치 / `TRAP_TRAJ` (사다리꼴 속도 프로파일) | 위치 제어 게인 + 트랩 궤적 (bandwidth 50, pos_gain 3.0, vel_gain 0.04) |
| `odrive_position_hold_test.py` | 위치 / `PASSTHROUGH` | 현재 위치 홀딩, 외력 인가 시 자동 복귀 (30 초 모니터) |
| `odrive_velocity_hold_test.py` | 속도 / `VEL_RAMP` | 8.0 rev/s 까지 램핑 후 유지. **HALL 노이즈 억제 게인** (bandwidth 20, vel_gain 0.05, integrator 0.1) |

#### HALL 트랙 절차 (체크리스트)

1. ODrive USB · 24 V · 브레이크 저항 · HALL 결선 확인
2. `python odrive_calibration.py` (구동 테스트 전 모터 보정)
3. `python odrive_closed_loop_test.py` 로 2바퀴 회전 확인 → axis.error 0x0
4. 용도별 단축 검증 후 차상위 텔레옵/비전 통합으로 진행

---

### 트랙 B — 엔코더 + 기본 모터 모델 (벤치/프로토타입 + 텔레옵 + 비전)

외장 증분 엔코더가 부착된 모터 구성. 인코더 모드와 모터 타입을 명시하지 않고
매 실행마다 `FULL_CALIBRATION_SEQUENCE`만 호출하는 단순 구조 — ODrive 기본
인코더 가정(증분형)과 기본 전류 한계로 동작한다. **D6374처럼 큰 모터를 이
스크립트로 직접 돌리지 말 것.** 캘리브레이션 전류가 모자라 INVALID_STATE 또는
MOTOR_FAILED 가 발생하거나, 반대로 작은 모터에 과전류가 흐를 수 있다.

#### 환경 / 비전 단독

| 파일 | 역할 |
| --- | --- |
| `setup_yolo_env.sh` | `yolo_env` 콘다 환경 생성 후 PyTorch(CUDA) · ultralytics(YOLOv8) · OpenVINO · OpenCV · ODrive · NumPy · matplotlib 일괄 설치. conda 사전 설치 필요 |
| `yolo_openvino_detection.py` | OpenVINO 백엔드의 YOLOv8n 모델로 카메라(V4L2, 1280×720, YUYV) 프레임 검출. 박스 중심을 임의 카메라 내부파라미터(fx=fy=500)로 3D 좌표 변환. **모터 명령 없음 (인지 단독 검증)** |

#### DualSense 게임패드 텔레옵 (USB로 ODrive 직결, `axis1`)

| 파일 | 제어 모드 / 입력 | 매핑 |
| --- | --- | --- |
| `odrive_dualsense_test.py` | 위치 (`POSITION_CONTROL`) | 좌스틱 → 누적 목표 위치 |
| `odrive_dualsense_vel_test.py` | 속도 (`VELOCITY_CONTROL` + `PASSTHROUGH`) | 트리거 → 속도 명령 (위치 필터 우회) |

#### YOLO 비전 통합 (`axis1`)

| 파일 | 역할 |
| --- | --- |
| `yolo_odrive_motor_test.py` | YOLO 검출 박스 중심 → `axis1` 목표 위치 (`SCALE_FACTOR=10.0`). 매 실행마다 `FULL_CALIBRATION_SEQUENCE` 자동 수행 |
| `odrive_yolo_object_tracking.py` | 운영 버전. 대상 = COCO `bottle`(39), `SCALE_FACTOR=5.0`, `MAX_TURNS=20.0`, `POS_DEADZONE=0.05`, **POS_FILTER**로 카메라 노이즈를 부드럽게 통과시킴. 캘리 상태 분기 (이미 캘리되어 있으면 스킵) |

#### 노트북 ↔ Pi 네트워크 텔레옵

로봇 측에서 ODrive를 직접 잡고, 노트북에서 DualSense 입력을 TCP로 전송하는
구성. **Pi 측 수신 스크립트는 본 저장소에 없음** — 별도 리포에서 관리되며,
이 트랙에는 클라이언트(노트북) 측만 포함된다.

| 파일 | 역할 | 포트 / 프로토콜 |
| --- | --- | --- |
| `robot_client.py` | DualSense 트리거(L2/R2) → 비율 0–1 → 텍스트 속도 명령으로 Pi 송신. `--detect` 옵션으로 트리거 축 인덱스 탐색 가능 | TCP `192.168.1.91:9000`, 20 Hz |
| `robot_client2.py` | 위와 동일하되 트리거→속도 단순 매핑 변형 (LT_AXIS=2, RT_AXIS=5) | TCP `:9000`, 20 Hz |
| `robot_laptop.py` | `robot_client2.py` + GStreamer JPEG 영상 수신 (`tcpclientsrc … ! jpegdec ! autovideosink`). Pi 캘리 끝날 때까지 자동 재연결 | 명령 `:9000`, 영상 `:5000` |

> Pi 측 IP는 모두 `192.168.1.91`로 하드코딩되어 있다. 환경에 맞게 수정 필요.

#### 엔코더 트랙 절차 (체크리스트)

1. `bash setup_yolo_env.sh` → `conda activate yolo_env`
2. 벤치 모터 + 외장 증분 엔코더 결선 확인 (D6374에 사용 금지)
3. ODrive USB · DC 전원 인가
4. 인지 단독 검증: `python yolo_openvino_detection.py`
5. 용도에 맞는 통합 스크립트 선택
   - 단일 PC 패드 텔레옵: `odrive_dualsense_vel_test.py`
   - 단일 PC 비전 추종: `odrive_yolo_object_tracking.py`
   - 노트북-Pi 원격: `robot_laptop.py` (Pi 측 수신 서비스 별도 구동 필요)

---

## 하드웨어 사양 (양 트랙 공통 기준값)

| 항목 | 값 |
| --- | --- |
| 휠 수 | 6 (로커-보기 서스펜션) |
| 휠 반경 | 100 mm |
| 차체 질량 | 30 kg |
| 모터 (HALL 트랙) | D6374 150 KV |
| 감속비 | 5 : 1 |
| 휠 토크 (피크) | 21 Nm (모터 4.95 Nm × 5) |
| 모터 컨트롤러 | ODrive (펌웨어 v0.5.x) |
| 입력 전압 | 24 V DC |
| 테스트 대상 axis | `axis1` (전 스크립트 통일) |

`parameter_calc/`의 정규화 상수 — `TAU_REF = 1.85 Nm`, `SN_REF = 35 dB`,
`WBOT ∈ [0.4, 0.7] m`, `P0_HEIGHT ≤ 0.5 m`, IK 실패율 ≤ 10 %.

---

## 기여 가이드

- `parameter_calc/`를 수정할 때는 `parameter_calc/CLAUDE.md`의 GPU 버그 히스토리
  섹션을 먼저 읽어 동일한 함정을 다시 만들지 말 것.
- `motor_control/` 스크립트는 독립 실행형 원칙을 유지 — 공용 모듈로 빼는
  리팩토링은 사전에 합의 후 진행.
- **HALL 트랙과 엔코더 트랙을 한 ODrive에서 번갈아 쓰지 말 것.** 캘리 설정이
  NVM에 남아 다음 트랙 실행 시 의도치 않게 덮여 쓰여진다. 같은 보드를 교차
  사용해야 한다면 `odrive_calibration.py`에서 모드를 강제 재설정한 뒤 시작.
- 모든 단축 테스트는 `axis1`을 사용. axis0은 사용하지 않는다.
- 결과 파일(`*.pkl`, `*.mat`, `*.mp4`, `fig*.png`)은 서버 검증본이므로 의도 없이
  덮어쓰지 말 것.

## 라이선스 / 연락

내부 프로젝트. 외부 공개·재배포 전 반드시 ZETIN 측에 확인.
