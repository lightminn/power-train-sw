# MKS CAN Reliability Implementation Plan

> **For agentic workers:** Use superpowers:subagent-driven-development for the isolated host scheduler task and review; remaining firmware and hardware integration stays with the controller.

**Goal:** Prevent the demonstrated MKS CAN TX failure path and reduce synchronized host queries, with explicit local stopping and reproducible evidence.

**Architecture:** Keep host motion commands in the actuator driver, share a phase-based bounded feedback schedule between active and idle polling, and patch the pinned MKS vendor firmware separately. Preserve original firmware/configuration before any device flash.

**Tech Stack:** Python, python-can, STM32F405/C++, FreeRTOS, vendor MKS ODrive 0.5.1, ARM GCC.

**Spec:** `docs/specs/2026-09-09-mks-can-reliability.md`

## Global Constraints

- Remote driving only; autonomy and intentionally absent US-100/L515 are excluded.
- Preserve all team Jetson files; worktree is `.worktrees/can-runtime-audit`.
- Physical gate confirmed by user: all wheels on ground under load, safe area and power cut ready.
  Total permitted motor-shaft travel is within one revolution; first validate zero setpoints.
- No firmware write without a verified original firmware/configuration restore path.
- Keep existing 200 ms feedback freshness and control commands independent of polling.
- Parent owns git integration, deployment and evidence; workers do not commit or operate hardware.

### Task 1: Phase and bound host feedback queries

**Files:** Modify `motor_control/corner_module/drive_odrive_can.py` and minimal shared-scheduler wiring
in `motor_control/chassis/chassis_manager.py`; create
`motor_control/corner_module/tests/test_can_poll_schedule.py`; adjust existing directly affected tests.

**Interfaces:** Keep public signatures compatible. `tick()` sends the current velocity every call,
then calls the same scheduler as `poll_feedback()`. Both paths in the same interval must not duplicate queries.

- [x] Write behavior tests with a manual clock and a recording CAN bus for nodes 11–16.
  At 50 Hz, require encoder maximum 2 requests per step after cold startup, all axes within
  80 ms initial coverage and maximum 80 ms steady interval, Iq at most 1 request per step and
  at most 2 per axis over 2 seconds. Preserve at least 1 per axis over 2 seconds.
  Assert every `tick()` sends one velocity command even if feedback is not due.
  Assert `tick(); poll_feedback()` in one interval does not duplicate requests and a 10-second
  clock jump never triggers a catch-up burst. Preserve stale flags when replies are missing.
- [x] Run the tests before implementation and record the expected failures.
- [x] Implement a shared logical-cycle schedule: one `begin_cycle()` per chassis tick, encoder
  phases `(node_id-11)%3`, Iq per-node phases in a 60-cycle period (nominal 1.2 seconds).
  Review rejected exact absolute slot matching: 15/15/30 ms jitter permanently skipped nodes13/16.
  Logical progression ensures fairness; standalone drivers use overdue deadlines. Arm services
  the three phases at 20 ms intervals within the existing total150ms deadline.
  Never sleep or create a polling thread in the actuator. A repeated stop must not reset every
  axis to an immediate polling burst. Keep armed and idle polling on the same schedule.
- [x] Run focused corner_module/chassis regression tests with
  `PYTHONPATH=.:motor_control:ros2/src/powertrain_ros python -m pytest motor_control/corner_module motor_control/chassis -q`.
  Use `/home/light/anaconda3/bin/python` on this workstation.
- [x] Write a report containing RED/GREEN output and exact changed files. Parent reviews and commits.

### Task 2: Reproducible MKS firmware repair

**Files:** Create `firmware/mks_odrive/` for pinned-source manifest, patch, build instructions,
host fault-injection tests and build/preparation tooling. Preserve historical diagnostic evidence unchanged.

- [x] Extract and hash the vendor package; inspect toolchain/build configuration and device backup APIs.
- [x] Execute unchanged vendor code with deterministic TX preemption and watchdog diagnostic input.
  The acceptance assertions require no sticky HAL error and no watchdog feed on RTR; record RED.
- [x] Protect check/AddTxMessage with a short RTOS critical section, exclude TX during reinit,
  record and bound error recovery, and latch motor-safe state before resuming CAN.
- [x] Validate control frame type/length/finite setpoints before feeding watchdog; reject telemetry feeds.
- [x] Run the same tests on patched sources and compile the complete Cortex-M4 firmware.
- [x] Review real-time behavior, startup/USB paths, NVM layout and reset semantics before deployment.

### Task 3: Board backup and Jetson validation

**Files:** Target-specific backup/evidence outside source first; sanitized results under
`docs/reports/2026-09-09-mks-can-reliability-evidence/`; report at
`docs/reports/2026-09-09-mks-can-reliability.md`.

- [x] Confirm serial `336A33523235`, IDLE, inputs zero, firmware identity; read all writable
  configuration and calibration fields and preserve endpoint description and checksum manifest.
- [x] Determine whether installed firmware and NVM can be read back without erase/unprotect.
  Do not substitute a vendor download for a backup of the installed unreleased image.
- [x] With recovery prerequisites met, validate one board; otherwise record exact blocked physical
  gate and finish build/testing/deployment that do not depend on flash.
- [x] Run the actual six-axis bounded query path on Jetson with no motion commands and continuous
  raw capture, then execute installed ROS/vcan loop if affected runtime contracts require it.
- [x] Review diff, update paired instructions/current report, integrate authorized changes and sync
  local/GitHub/Jetson canonical checkout; preserve team working tree.

Progress: final source/host/DFU/build review accepted; 1005 combined host tests passed.
Physical single-board flash/config/zero watchdog/CAN recovery passed. One power cycle retained
configuration/calibration but recorded Stuff error 0x08 and latched IDLE; stable 10/10 observation
followed by explicit clear passed. Bounded ±0.25 motor-turn movement FAILED target attainment
within four seconds, stopped IDLE/error0 within the authorized travel limit, and restored all
307 settings. Final actual six-axis query test ran 600 seconds: all ten motors continuous,
encoder stale0, host CAN errors/drops0. Firmware recovery_count stayed2, while TX drop count
increased2→4 across the combined clear/motion/query interval. Final USB/passive CAN readback
passed. Remaining physical scope: the other two boards, watchdog NVM, real driving and braking.
Code commit2bf5a28 is synchronized; the evidence/documentation handoff follows separately.
