# Defence Robot — Power Train SW

ZETIN 6륜 로커-보기(rocker-bogie) 서스펜션 방위 로봇의 파워트레인 소프트웨어 저장소.
설계 측 코드와 실차 제어 측 코드가 한 저장소에 함께 들어 있으며, 두 트랙은 서로 독립적으로 동작한다.

| 트랙 | 폴더 | 역할 |
| --- | --- | --- |
| 파라미터 최적화 | `parameter_calc/` | 14차원 로커-보기 형상 파라미터 최적화 (MATLAB / NumPy / JAX-CUDA) |
| 모터 제어 | `motor_control/` | ODrive 기반 실차 구동, DualSense 텔레옵, YOLO 비전 트래킹, 노트북-Pi 네트워킹 |

> 본 저장소의 `parameter_calc/` 트리는 개발 서버에서 검증된 빌드를 그대로 옮긴 것으로,
> 결과물(`*.pkl`, `*.mat`)을 신뢰할 수 있는 기준 코드입니다.

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
    ├── odrive_calibration.py        # ODrive 캘리브레이션 (필수 1회)
    ├── odrive_basic_test.py         # 위치 제어 기본 동작 확인
    ├── odrive_closed_loop_test.py   # 폐루프 진입 + 2회전 테스트
    ├── odrive_position_hold_test.py # 현재 위치 홀딩
    ├── odrive_velocity_hold_test.py # 일정 속도 유지
    ├── odrive_diff_drive_test.py    # 차동 구동 검증
    ├── odrive_dualsense_test.py     # DualSense → 위치 제어
    ├── odrive_dualsense_vel_test.py # DualSense → 속도 제어
    ├── odrive_yolo_object_tracking.py # YOLO 객체 추종으로 위치 제어
    ├── yolo_odrive_motor_test.py    # YOLO + ODrive 통합 테스트
    ├── yolo_openvino_detection.py   # YOLO 검출 단독 (3D 좌표 변환)
    ├── robot_client.py              # 노트북: DualSense → TCP → Pi
    ├── robot_client2.py             # 위와 동일, 트리거→속도 전송 변형
    ├── robot_laptop.py              # 노트북: 명령 송신 + GStreamer 영상 수신
    └── setup_yolo_env.sh            # yolo_env 콘다 환경 자동 설치
```

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

각 스크립트는 **독립 실행형**이다 (공용 패키지 구조 없음). 보통 다음 순서로 사용한다:

1. 환경 셋업 (`setup_yolo_env.sh`)
2. ODrive 캘리브레이션 (`odrive_calibration.py`) — **최초 1회 또는 모터 교체 시**
3. 단축 동작 확인 (`odrive_basic_test.py`, `odrive_closed_loop_test.py` 등)
4. 텔레옵 / 비전 통합 (`odrive_dualsense_*.py`, `odrive_yolo_*.py`, `robot_*.py`)

> 모든 스크립트의 공통 가정: ODrive 펌웨어 v0.5.x, 모터 D6374 150 KV,
> **5 : 1 감속비** (`GEAR_RATIO = 5.0`), 24 V DC 입력. 대부분 `axis1`을 기본
> 사용하지만 일부는 `axis0`을 사용하므로 스크립트별 표를 참고.

### 환경 / 캘리브레이션

| 파일 | 무엇을 하는가 | 필수 입력 |
| --- | --- | --- |
| `setup_yolo_env.sh` | `yolo_env` 콘다 환경 생성 후 PyTorch(CUDA) · ultralytics(YOLOv8) · OpenVINO · OpenCV · ODrive · NumPy · matplotlib 일괄 설치 | conda 설치되어 있을 것 |
| `odrive_calibration.py` | `axis1` 풀 캘리브레이션 시퀀스(모터·엔코더), 에러 클리어, 게인 세팅, 결과 NVM 저장. 안전 헬퍼(`safe_set`, `wait_idle`, `dump_errors`) 포함 | ODrive USB 연결, 24 V DC |

### 단축(single-axis) 동작 검증 — 작은 단위부터 큰 단위로

| 파일 | 제어 모드 / 입력 | 무엇을 검증 | 사용 axis |
| --- | --- | --- | --- |
| `odrive_closed_loop_test.py` | 위치 / `PASSTHROUGH` 가정 | 캘리 끝난 모터가 폐루프에 들어가서 **2회전** 정상 수행되는지 | `axis1` |
| `odrive_basic_test.py` | 위치 / `TRAP_TRAJ` (사다리꼴 속도 프로파일) | 위치 제어 게인 검증 + 트랩 궤적 기본 동작 | `axis1` |
| `odrive_position_hold_test.py` | 위치 / `PASSTHROUGH` | 현재 위치를 그대로 홀딩 — 정지 안정성 확인 | `axis0` |
| `odrive_velocity_hold_test.py` | 속도 / `VEL_RAMP` | 목표 속도 8.0 rev/s 까지 램핑 후 유지, vel 게인 노이즈 억제 검증 | `axis1` |
| `odrive_diff_drive_test.py` | (구성 점검) | 펌웨어/하드웨어 버전, GPIO9-11 모드, vbus, POLE_PAIRS 등을 덤프해 차동 구동 사전 점검 | `axis0` |

### DualSense 게임패드 텔레옵 (USB로 ODrive 직결)

| 파일 | 제어 모드 | 매핑 | 비고 |
| --- | --- | --- | --- |
| `odrive_dualsense_test.py` | 위치 제어 (`POSITION_CONTROL`) | 스틱 → 목표 위치 누적 | 캘리부터 자동 수행 |
| `odrive_dualsense_vel_test.py` | 속도 제어 (`VELOCITY_CONTROL` + `PASSTHROUGH`) | 트리거 → 속도 명령 | 위치 필터를 우회해 속도값 직결 |

### YOLO 비전 통합

| 파일 | 역할 |
| --- | --- |
| `yolo_openvino_detection.py` | OpenVINO 백엔드의 YOLOv8n 모델로 카메라(V4L2, 1280×720, YUYV) 프레임 검출. 박스 중심을 임의 카메라 내부파라미터(fx=fy=500)로 3D 좌표 변환 (모터 제어 없음, 인지 단독 검증) |
| `yolo_odrive_motor_test.py` | 위 검출 결과 중 한 클래스를 골라 `axis1` 위치 명령으로 `SCALE_FACTOR=10.0` 비례 변환. ODrive 캘리도 자동 수행 |
| `odrive_yolo_object_tracking.py` | 운영 버전. 대상 클래스 = COCO `bottle`(39), `SCALE_FACTOR=5.0`, `MAX_TURNS=20.0`, `POS_DEADZONE=0.05` 으로 떨림/발진 억제. 캘리 상태에 따라 분기 (이미 캘리되어 있으면 스킵) |

### 노트북 ↔ Pi 네트워크 텔레옵

로봇 측에서 ODrive를 직접 잡고, 노트북에서 DualSense 입력을 TCP로 전송하는
구성. **Pi 측 수신 스크립트는 별도** (이 저장소에는 클라이언트 측만 포함).

| 파일 | 역할 | 포트 / 프로토콜 |
| --- | --- | --- |
| `robot_client.py` | DualSense 트리거(L2/R2) → 비율 0–1 → 속도 명령 텍스트로 Pi에 송신. `--detect` 옵션으로 트리거 축 인덱스 탐색 가능 | TCP `192.168.1.91:9000`, 20 Hz |
| `robot_client2.py` | 위와 동일하되 트리거→속도 단순 매핑 변형 (LT_AXIS=2, RT_AXIS=5) | TCP `:9000`, 20 Hz |
| `robot_laptop.py` | `robot_client2.py` + GStreamer JPEG 영상 수신 (`tcpclientsrc … ! jpegdec ! autovideosink`). Pi 캘리 끝날 때까지 자동 재연결 | 명령 `:9000`, 영상 `:5000` |

> Pi 측 IP는 모두 `192.168.1.91`로 하드코딩되어 있다. 환경에 맞게 수정 필요.

### 최소 동작 절차 (체크리스트)

1. `bash setup_yolo_env.sh` → `conda activate yolo_env`
2. ODrive USB·24 V·브레이크 저항 연결 확인
3. `python odrive_calibration.py` 실행, 에러 0x0 확인
4. `python odrive_closed_loop_test.py` 로 모터가 2바퀴 도는지 확인
5. 용도에 맞는 상위 스크립트 선택
   - 패드 텔레옵: `odrive_dualsense_vel_test.py` (단일 PC) 또는 `robot_laptop.py` (원격)
   - 비전 추종: `odrive_yolo_object_tracking.py`

---

## 하드웨어 사양 (양 트랙 공통 기준값)

| 항목 | 값 |
| --- | --- |
| 휠 수 | 6 (로커-보기 서스펜션) |
| 휠 반경 | 100 mm |
| 차체 질량 | 30 kg |
| 모터 | D6374 150 KV |
| 감속비 | 5 : 1 |
| 휠 토크 (피크) | 21 Nm (모터 4.95 Nm × 5) |
| 모터 컨트롤러 | ODrive (펌웨어 v0.5.x) |
| 입력 전압 | 24 V DC |

`parameter_calc/`의 정규화 상수 — `TAU_REF = 1.85 Nm`, `SN_REF = 35 dB`,
`WBOT ∈ [0.4, 0.7] m`, `P0_HEIGHT ≤ 0.5 m`, IK 실패율 ≤ 10 %.

---

## 기여 가이드

- `parameter_calc/`를 수정할 때는 `parameter_calc/CLAUDE.md`의 GPU 버그 히스토리
  섹션을 먼저 읽어 동일한 함정을 다시 만들지 말 것.
- `motor_control/` 스크립트는 독립 실행형 원칙을 유지 — 공용 모듈로 빼는
  리팩토링은 사전에 합의 후 진행.
- 결과 파일(`*.pkl`, `*.mat`, `*.mp4`, `fig*.png`)은 서버 검증본이므로 의도 없이
  덮어쓰지 말 것.

## 라이선스 / 연락

내부 프로젝트. 외부 공개·재배포 전 반드시 ZETIN 측에 확인.
