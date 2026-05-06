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
└── motor_control/        ODrive + DualSense + YOLO control scripts
```

Detailed simulation pipeline, 14-parameter space, objective weights, GPU acceleration strategy, and known GPU bug history live in `parameter_calc/CLAUDE.md`. Read that file before touching anything in `parameter_calc/`.

## Source-of-Truth Note

`parameter_calc/` was downloaded from the development server and is the authoritative implementation — its code and any persisted results (`*.pkl`, `*.mat`) should be trusted over historical local copies. Earlier local-only versions were removed during the directory cleanup.

## Working in `motor_control/`

Self-contained Python scripts; no shared package structure. Key files:

- `odrive_calibration.py`, `odrive_basic_test.py`, `odrive_closed_loop_test.py`, `odrive_position_hold_test.py`, `odrive_velocity_hold_test.py` — ODrive bring-up and single-axis tests
- `odrive_diff_drive_test.py` — differential drive across two axes
- `odrive_dualsense_test.py`, `odrive_dualsense_vel_test.py` — DualSense (PS5) gamepad teleop
- `odrive_yolo_object_tracking.py`, `yolo_odrive_motor_test.py`, `yolo_openvino_detection.py` — YOLO/OpenVINO perception driving the motors
- `robot_client.py`, `robot_client2.py`, `robot_laptop.py` — networking between the on-robot controller and a laptop UI
- `setup_yolo_env.sh` — YOLO/OpenVINO environment setup

Hardware: ODrive controllers driving D6374 150 KV motors (4.95 Nm peak × 5:1 gearbox = 21 Nm at the wheel).

## Robot Specification (shared across both tracks)

- 6 wheels, rocker-bogie suspension
- Wheel radius: 100 mm
- Total mass: 30 kg
- Drive motor: D6374 150 KV via 5:1 gearbox (≈ 21 Nm at wheel, peak)
