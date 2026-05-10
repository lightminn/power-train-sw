# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working in this repository.

## Project Overview

ZETIN Defense Robot — a 6-wheeled rocker-bogie suspension robot. Work in this repository is split into two independent tracks:

1. **Parameter optimization** (`parameter_calc/`) — multi-criteria optimization that selects the optimal 14-dimensional rocker-bogie geometry across terrain types (stairs, wood blocks, rough surfaces, steps).
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
├── motor_control/        ODrive + DualSense + YOLO control scripts
│   ├── *.py              Self-contained host-side scripts (HALL / encoder tracks)
│   ├── laptop/           Laptop-side TCP teleop clients (DualSense → robot)
│   └── pi/               Raspberry-Pi-side servers (paired 1:1 with laptop/)
├── docker/               Container definitions (x86 dev + Jetson Orin Nano deploy)
├── scripts/              Host-side helpers (e.g. recv_stream.sh — UDP H.264 receiver)
└── docs/
    ├── specs/            Per-task design docs (requirements, interfaces)
    └── plans/            Per-task implementation plans + verification logs
```

Detailed simulation pipeline, 14-parameter space, objective weights, GPU acceleration strategy, and known GPU bug history live in `parameter_calc/CLAUDE.md`. Read that file before touching anything in `parameter_calc/`. Per-task background for Jetson / streaming work lives under `docs/specs/` and `docs/plans/` — read those before editing the matching scripts.

## Source-of-Truth Note

`parameter_calc/` was downloaded from the development server and is the authoritative implementation — its code and any persisted results (`*.pkl`, `*.mat`) should be trusted over historical local copies. Earlier local-only versions were removed during the directory cleanup.

## Working in `motor_control/`

Self-contained Python scripts; no shared package structure. Two hardware tracks
share the directory and **must not be mixed on the same ODrive** (calibration /
gain / current limits diverge — see README "트랙 A / 트랙 B"):

- **HALL track** (D6374 + built-in hall, NVM-stored calibration):
  `odrive_calibration.py`, `odrive_diff_drive_test.py`, `odrive_basic_test.py`,
  `odrive_closed_loop_test.py`, `odrive_position_hold_test.py`,
  `odrive_velocity_hold_test.py`
- **Encoder track** (external incremental encoder, full calibration each run):
  `odrive_dualsense_test.py`, `odrive_dualsense_vel_test.py`,
  `yolo_odrive_motor_test.py`, `odrive_yolo_object_tracking.py`
- **Vision-only** (no motor command):
  `yolo_openvino_detection.py` (x86, OpenVINO),
  `yolo_cuda_stream.py` (Jetson, PyTorch-CUDA / TensorRT FP16, GStreamer UDP H.264 sender — paired with `scripts/recv_stream.sh` on the laptop)
- **Networked teleop** (1:1 pairs):
  `laptop/laptop_client_*.py` ↔ `pi/pi_server_*.py` (TCP `:9000`, newline-delimited
  `%.4f\n` velocity); `laptop_client_video.py` adds GStreamer JPEG video at `:5000`
- **Setup**: `setup_yolo_env.sh` (conda fallback when not using Docker)

All scripts use `axis1`. Hardware: ODrive controllers driving D6374 150 KV motors
(4.95 Nm peak × 5:1 gearbox = 21 Nm at the wheel).

## Jetson Orin Nano deployment

`docker/Dockerfile.jetson` + `docker/docker-compose.jetson.yml` build on top of
`dustynv/l4t-pytorch:r36.4.0` (CUDA + cuDNN + TensorRT + ARM PyTorch). Compose
file mounts `/dev`, NVENC GStreamer plugin, and runs `privileged: true` so V4L2
cameras + USB devices are accessible from the container. Vision/streaming entry
point is `motor_control/yolo_cuda_stream.py`; the laptop runs
`scripts/recv_stream.sh <port>` to display the decoded stream. Background and
verification log: `docs/specs/2026-05-08-jetson-yolo-stream-design.md`,
`docs/plans/2026-05-08-jetson-yolo-stream-plan.md`.

## Robot Specification (shared across both tracks)

- 6 wheels, rocker-bogie suspension
- Wheel radius: 100 mm
- Total mass: 30 kg
- Drive motor: D6374 150 KV via 5:1 gearbox (≈ 21 Nm at wheel, peak)
