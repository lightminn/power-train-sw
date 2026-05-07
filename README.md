# Power Train SW

ZETIN 6륜 로커-보기(rocker-bogie) 방위 로봇의 파워트레인 SW 저장소. 두 트랙은
서로 독립적으로 동작한다.

| 트랙 | 폴더 | 역할 |
| --- | --- | --- |
| 파라미터 최적화 | `parameter_calc/` | 14차원 형상 파라미터 최적화 (MATLAB / NumPy / JAX-CUDA) |
| 모터 제어 | `motor_control/` | ODrive · DualSense · YOLO · 노트북-Pi 네트워킹 |

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
└── motor_control/
    │   ─── [HALL 트랙: D6374 + 내장 HALL] ───
    ├── odrive_calibration.py        # HALL 모드 캘리 (1회, NVM 저장)
    ├── odrive_diff_drive_test.py    # HALL 폴라리티 캘리 + 10회전 검증
    ├── odrive_basic_test.py         # 위치 제어 + TRAP_TRAJ 동작
    ├── odrive_closed_loop_test.py   # 폐루프 진입 + 2회전
    ├── odrive_position_hold_test.py # 위치 홀딩 + 외력 복귀
    ├── odrive_velocity_hold_test.py # 속도 유지 (HALL 노이즈 억제 게인)
    │   ─── [엔코더 트랙: 외장 증분형 + 기본 모터 모델] ───
    ├── setup_yolo_env.sh            # yolo_env 콘다 환경 설치
    ├── yolo_openvino_detection.py   # YOLO 검출 단독 (모터 명령 없음)
    ├── odrive_dualsense_test.py     # DualSense 위치 제어
    ├── odrive_dualsense_vel_test.py # DualSense 속도 제어
    ├── yolo_odrive_motor_test.py    # YOLO 추종 (테스트)
    ├── odrive_yolo_object_tracking.py # YOLO 추종 (운영)
    ├── robot_client.py              # 노트북: DualSense → TCP → Pi
    ├── robot_client2.py             # 트리거→속도 단순 매핑 변형
    └── robot_laptop.py              # 명령 송신 + GStreamer 영상 수신
```

> 모든 모터 제어 스크립트는 `axis1` 사용으로 통일.

---

## parameter_calc — 형상 파라미터 최적화

14차원 형상 파라미터 벡터 `x`를 4종 지형(계단 / 나무 블록 / 거친 노면 / 단차)에
대해 평가, 모터 토크·안정성(ZMP, TOI)·승차감(S/N) 가중합 비용을 최소화. MATLAB
원본을 NumPy/SciPy(CPU)와 JAX/CUDA(GPU)로 포팅, 동일한 물리식 공유.

### 빠른 실행

```bash
# GPU (RTX 3090 기준 약 3–10분, 첫 실행 JIT 30–60초)
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
| `gen_terrain.py` | 4종 지형 프로파일 (CPU/GPU 공유) |

상세 파이프라인, 14파라미터 정의, 목적함수 가중치, GPU 버그 히스토리는
[`parameter_calc/CLAUDE.md`](parameter_calc/CLAUDE.md). 수정 전 필독.

---

## motor_control — 실차 모터 제어

스크립트는 가정하는 **센서·모터 모델**에 따라 두 트랙으로 나뉘며, **반드시 같은
트랙 안의 스크립트만 함께 사용**해야 한다. 한 트랙으로 캘리한 ODrive를 다른
트랙 스크립트로 구동하면 게인·전류 한계·인코더 해석이 달라 폭주/과전류 위험.

| 트랙 | 센서 | 모터 모델 | 캘리 방식 |
| --- | --- | --- | --- |
| **HALL** | 모터 내장 홀센서 | D6374 150 KV (`HIGH_CURRENT`, 5 pp, cpr=30) | `EncoderMode.HALL` 명시 + 폴라리티/오프셋 캘리, NVM 저장 후 재사용 |
| **엔코더** | 외장 증분형 | ODrive 기본값 | 매 실행 `FULL_CALIBRATION_SEQUENCE` |

ODrive 펌웨어 v0.5.x, 테스트 axis = `axis1`.

---

### 트랙 A — HALL (실차 운영용)

cpr이 30으로 낮아 게인을 보수적으로 잡고 vel-estimate 필터 bandwidth를 낮춰 노이즈를 억제.

#### 캘리브레이션

| 파일 | 역할 |
| --- | --- |
| `odrive_calibration.py` | `axis1`에 HALL 모드·`pole_pairs=5`·`cpr=30`·`HIGH_CURRENT` 강제 후 풀 캘리(모터 → 폴라리티 → 오프셋), 게인 세팅, NVM 저장. **HALL 트랙 모든 작업 전 1회 필수** |
| `odrive_diff_drive_test.py` | 동일 HALL 설정 + `ENCODER_HALL_POLARITY_CALIBRATION` + 10회전 자체 검증 |

#### 단축 동작 검증 (사전 캘리 필요)

| 파일 | 제어 모드 | 검증 내용 |
| --- | --- | --- |
| `odrive_closed_loop_test.py` | 위치 / 기본 | 폐루프 진입 + 2회전 |
| `odrive_basic_test.py` | 위치 / `TRAP_TRAJ` | 게인 + 사다리꼴 궤적 (bandwidth 50, pos 3.0, vel 0.04) |
| `odrive_position_hold_test.py` | 위치 / `PASSTHROUGH` | 외력 인가 시 자동 복귀 (30 초 모니터) |
| `odrive_velocity_hold_test.py` | 속도 / `VEL_RAMP` | 8.0 rev/s 유지. **HALL 노이즈 억제 게인** (bandwidth 20, vel 0.05, integrator 0.1) |

#### 절차

1. `python odrive_calibration.py`
2. `python odrive_closed_loop_test.py` — 2회전 + axis.error 0x0 확인
3. 용도별 단축 검증

---

### 트랙 B — 엔코더 (벤치 / 텔레옵 / 비전)

외장 증분 엔코더가 부착된 모터에서 매 실행 `FULL_CALIBRATION_SEQUENCE`만 호출.
인코더 모드와 모터 타입을 명시하지 않아 ODrive 기본값으로 동작 — **D6374 같은
큰 모터를 이 스크립트로 직접 돌리지 말 것** (캘리 전류 부족 또는 과전류 위험).

#### 환경 / 검출 단독

| 파일 | 역할 |
| --- | --- |
| `setup_yolo_env.sh` | `yolo_env` 콘다 환경 + PyTorch(CUDA) · ultralytics(YOLOv8) · OpenVINO · OpenCV · ODrive 설치 |
| `yolo_openvino_detection.py` | OpenVINO YOLOv8n 카메라 검출(V4L2, 1280×720) + 임의 카메라 파라미터로 3D 좌표 변환. 모터 명령 없음 |

#### DualSense 직결 텔레옵

| 파일 | 제어 / 입력 | 매핑 |
| --- | --- | --- |
| `odrive_dualsense_test.py` | `POSITION_CONTROL` | 좌스틱 → 누적 목표 위치 |
| `odrive_dualsense_vel_test.py` | `VELOCITY_CONTROL` + `PASSTHROUGH` | 트리거 → 속도 명령 (위치 필터 우회) |

#### YOLO 비전 통합

| 파일 | 역할 |
| --- | --- |
| `yolo_odrive_motor_test.py` | YOLO 박스 중심 → 위치 명령 (`SCALE_FACTOR=10.0`) |
| `odrive_yolo_object_tracking.py` | 운영 버전. 대상 = COCO `bottle`, `SCALE_FACTOR=5.0`, `MAX_TURNS=20.0`, `POS_DEADZONE=0.05`, **POS_FILTER**로 카메라 노이즈 완화. 캘리 상태 분기(이미 캘리되어 있으면 스킵) |

#### 노트북 ↔ Pi 네트워크 텔레옵

노트북에서 DualSense 입력을 TCP로 Pi에 송신. **Pi 측 수신 스크립트는 별도 리포에서 관리**, 본 저장소는 클라이언트 측만 포함.

| 파일 | 역할 | 포트 |
| --- | --- | --- |
| `robot_client.py` | 트리거(L2/R2) → 0–1 비율 → 텍스트 속도 명령. `--detect`로 트리거 축 인덱스 탐색 | TCP `:9000`, 20 Hz |
| `robot_client2.py` | 트리거→속도 단순 매핑 변형 (LT_AXIS=2, RT_AXIS=5) | TCP `:9000`, 20 Hz |
| `robot_laptop.py` | `robot_client2.py` + GStreamer JPEG 영상 수신, Pi 캘리 종료까지 자동 재연결 | 명령 `:9000`, 영상 `:5000` |

> Pi IP `192.168.1.91`로 하드코딩 — 환경에 맞게 수정 필요.

#### 절차

1. `bash setup_yolo_env.sh` → `conda activate yolo_env`
2. 인지 단독: `python yolo_openvino_detection.py`
3. 통합 스크립트 선택
   - 패드 텔레옵: `odrive_dualsense_vel_test.py`
   - 비전 추종: `odrive_yolo_object_tracking.py`
   - 원격: `robot_laptop.py` (Pi 측 수신 서비스 별도 구동)

---

## 기여 가이드

- `parameter_calc/` 수정 전 `parameter_calc/CLAUDE.md`의 GPU 버그 히스토리 섹션 필독.
- `motor_control/` 스크립트는 독립 실행형 원칙 유지 — 공용 모듈 분리는 사전 합의.
- **HALL/엔코더 트랙을 한 ODrive에서 번갈아 쓰지 말 것.** NVM에 남은 캘리 설정이 의도치 않게 적용된다. 교차 사용 시 `odrive_calibration.py`에서 모드 강제 재설정 후 시작.
- 모든 테스트는 `axis1`. axis0 사용 금지.
- 결과 파일(`*.pkl`, `*.mat`, `*.mp4`, `fig*.png`)은 서버 검증본 — 의도 없이 덮어쓰지 말 것.

## 라이선스 / 연락

내부 프로젝트. 외부 공개·재배포 전 ZETIN 측 확인.
