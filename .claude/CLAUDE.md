# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working in this repository.

## Project Overview

ZETIN Defense Robot — a 6-wheeled rocker-bogie suspension robot. Work in this repository is split into two independent tracks:

1. **Parameter optimization** (`parameter_calc/`) — multi-criteria optimization that selects the optimal rocker-bogie geometry across terrain types. Current authoritative track is **v4** (`python_gpu_triangle/`): 15-dimensional, 7 terrains (stairs, wood, rough, step, curved ramp, 15°/30° incline), 면-기준 물리 수정본. v3 (`python_gpu/`): 14-dim, 4 terrains.
2. **Motor control** (`motor_control/`) — runtime control software for the physical robot: ODrive driver scripts, DualSense gamepad teleop, YOLO-based object tracking, and laptop↔robot networking.

## Directory Layout

```
Defence_Robot/
├── .claude/              Claude Code settings + this file
├── parameter_calc/       Geometry optimization (trusted server build)
│   ├── CLAUDE.md         → detailed docs for this track
│   ├── matlab/           Original MATLAB reference (functions/, *.m, *.mat)
│   ├── python/           CPU port (NumPy/SciPy)
│   ├── python_gpu/       GPU port (JAX/CUDA 12.x) — v3
│   ├── python_gpu_triangle/  GPU variant restricted to triangle mode — v4
│   └── scripts/          run_gpu.sh, run_gpu_triangle.sh
├── motor_control/        ODrive · AK 조향 · YOLO · US100 센서 · 텔레옵
│   ├── drive/            구동 모터
│   │   ├── x2212_test/   SunnySky X2212-13 + TLE5012B (ODrive USB · CAN)
│   │   └── bl70200/      BL70200 + 내장 HALL ×3 (실전, ODrive USB)
│   ├── steering/         AK40/AK45 조향 (CAN, 동일 API)
│   ├── vision/           모터 명령 없는 검출·스트리밍
│   ├── sensors/          US100 거리 (UART /dev/ttyTHS1)
│   ├── corner_module/    코너 모듈 패키지 (조향+구동 협조 제어 + DualSense 텔레옵)
│   ├── laptop/           Laptop-side TCP teleop clients (DualSense → robot)
│   └── pi/               Raspberry-Pi-side servers (paired 1:1 with laptop/)
├── docker/               Container definitions (x86 dev + Jetson Orin Nano deploy)
├── scripts/              Host-side helpers (e.g. recv_stream.sh — UDP H.264 receiver)
└── docs/
    ├── specs/            Per-task design docs (requirements, interfaces)
    └── plans/            Per-task implementation plans + verification logs
```

Detailed simulation pipeline, parameter space (v4 15-dim / v3 14-dim), objective weights, GPU acceleration strategy, and known GPU bug history live in `parameter_calc/CLAUDE.md`. Read that file before touching anything in `parameter_calc/`. Per-task background for Jetson / streaming work lives under `docs/specs/` and `docs/plans/` — read those before editing the matching scripts.

## Source-of-Truth Note

`parameter_calc/` was downloaded from the development server and is the authoritative implementation — its code and any persisted results (`*.pkl`, `*.mat`) should be trusted over historical local copies. Earlier local-only versions were removed during the directory cleanup.

## Working in `motor_control/`

Mostly self-contained scripts; one shared package (`corner_module/`). Three motor
hardware lines, isolated by subfolder. **Never mix tracks on the same ODrive** (calibration
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
- **steering/** (AK40-10 테스트 → AK45-36 실전, CAN socketcan can0): `ak_control.py` (메인 라이브러리 —
  python-can socketcan 직접 제어), `calibrate_ak.py` (기어비 1회성), `status_ak.py`
  (CAN RX 디버깅). 사전 준비: `bash scripts/can_setup.sh`.
- **vision/** (모터 명령 없음): `yolo_openvino_detection.py` (x86 OpenVINO),
  `yolo_cuda_stream.py` (Jetson CUDA/TRT + GStreamer UDP H.264 송신 — 수신은
  `scripts/recv_stream.sh`), `setup_yolo_env.sh` (x86 conda, Docker 권장).
- **sensors/** (UART `/dev/ttyTHS1`): `us100_basic.py` (US100 0x55 기본),
  `us100_robust.py` (Jetson UART TX 떨림 버그 우회 — 0xFF prefix).
- **corner_module/** (조향+구동 협조 제어 패키지, 코너 1개 = 로커보기 1/6): `corner_module.py`
  (`CornerModule` — 상태머신·워치독·estop·과전류 트립·폐루프 점프방지), `actuator.py`
  (트랜스포트 무관 `Actuator`/`SteerActuator`/`DriveActuator` ABC), `steer_ak40.py`(AK CAN)·
  `drive_odrive_usb.py`(현재)·`drive_odrive_can.py`(추후 스텁) 드라이버, `fake.py`(무하드웨어
  테스트 더블), `teleop_dualsense.py` (`python3 -m corner_module.teleop_dualsense`). 단위테스트
  24 + HIL(조향·구동·통합·텔레옵) 검증. 미래 4WS 애커만 키네마틱스 레이어의 빌딩블록.
- **Networked teleop** (1:1 pairs): `laptop/laptop_client_*.py` ↔ `pi/pi_server_*.py`
  (TCP `:9000`, newline-delimited `%.4f\n` velocity); `laptop_client_video.py` adds
  GStreamer JPEG video at `:5000`.

ODrive 펌웨어 v0.5.x (CAN 트랙 fw-v0.5.6 검증), all scripts use `axis1`. Jetson CAN
트랙 입력 전 `input_pos = origin` 설정으로 폐루프 진입 시 모터 점프 방지.

## Jetson Orin Nano deployment

`docker/Dockerfile.jetson` + `docker/docker-compose.jetson.yml` build on top of
`dustynv/l4t-pytorch:r36.4.0` (CUDA + cuDNN + TensorRT + ARM PyTorch). Compose
file mounts `/dev`, NVENC GStreamer plugin, and runs `privileged: true` so V4L2
cameras + USB devices are accessible from the container. Vision/streaming entry
point is `motor_control/vision/yolo_cuda_stream.py`; the laptop runs
`scripts/recv_stream.sh <port>` to display the decoded stream. Background and
verification log: `docs/specs/2026-05-08-jetson-yolo-stream-design.md`,
`docs/plans/2026-05-08-jetson-yolo-stream-plan.md`. Hardware reinventory + folder
reorg (5/20): `docs/specs/2026-05-20-motor-control-reorg-design.md`,
`docs/plans/2026-05-20-motor-control-reorg-plan.md`.

## Robot Specification (shared across both tracks)

- 6 wheels, rocker-bogie suspension
- Wheel radius: 100 mm
- Total mass: ~86 kg (설계 추정; 여유 포함 최대 100 kg, Notion 기준)
- Drive motor (test): SunnySky X2212-13 + TLE5012B 16384 CPR encoder
- Drive motor (real): BL70200 + internal HALL ×3 (pp=5, cpr=30)
- Steering: CubeMars AK40-10 (test, 10:1) / AK45-36 (real, 36:1; peak 24 Nm, rated 8 Nm, KV80, peak current 65 A, backlash 12 arcmin, back-drive 0.8 Nm), CAN bus, identical API. 모터 프로파일은 `ak_control.py`의 `MOTOR_PROFILES`/`ACTIVE_MOTOR`로 전환
