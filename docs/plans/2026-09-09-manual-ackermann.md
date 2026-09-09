# Manual Ackermann and URDF Geometry Implementation Plan

> **For agentic workers:** Execute the approved spec task by task using superpowers:executing-plans; independent geometry extraction and ROS adapter work may run in parallel.

**Goal:** Apply verified URDF dimensions and independent manual steering through the production remote drive path.

**Architecture:** Retain chassis geometry/policy ownership in the pure core. Publish one typed speed+steering command, carry it atomically through authority, and resolve wheel angles around the fixed middle axle. Legacy Twist stays explicit.

**Tech Stack:** Python, ROS2 Humble rosidl messages, python-can, SocketCAN, pytest.

**Spec:** `docs/specs/2026-09-09-manual-ackermann.md`

## Global Constraints

Preserve the other Jetson checkout, calibrated signs, NVM, CAN configuration, and all unrelated work. Autonomy is outside physical acceptance. No automatic arm or physical motion during deployment.

## Tasks

- [x] Extract full-chain URDF geometry with `tools/extract_ackermann_geometry.py`; test rotated fixture, mesh scale, source identity, and continuous limits. Run extractor against the actual URDF and independently compare FK.
- [x] Write failing `motor_control/chassis/tests/test_manual_steering.py`: stationary drive exactly zero, front/rear steering signs, reverse angle invariance, same ICR, yaw/speed caps, stale and neutral gates. Run before implementation.
- [x] Implement `solve_steering(geom, v_mps, steering, max_omega_rad_s=1.2)` and precision coordinates in `kinematics.py`; verify through independent odometry forward reconstruction.
- [x] Extend `CommandAuthority.submit(..., steering=None)`, `Command.steering`, and `ChassisManager.set(..., steering=None)` with freshness/neutral/reset preservation. None means legacy Twist; invalid mixed commands enter HOLD.
- [x] Add `ManualDriveCommand.msg`, ROS pub/sub and startup format selection, gateway mapping, and steering-aware stop evidence. Run pure gateway RED/GREEN and installed ROS boundary tests.
- [x] Run installed session→pad frames→gateway→authority→real CAN drivers on isolated vcan77→wheel telemetry→client loop. Verify six stop/forward/reverse directions and zero raw drive at stationary steering; disconnect must stop and not re-arm.
- [x] Review scoped diff, run relevant regression suites, and record counts/limits. Update operating docs and paired project guidance with geometry/command contracts.
- [ ] Sync reviewed code to local main, GitHub, and canonical Jetson checkout; rebuild interfaces and nodes, compare installed sources, restart operational services and confirm healthy/no traceback.
- [ ] Open console ready for the user's physical steering acceptance; record it separately from emulated feedback.
