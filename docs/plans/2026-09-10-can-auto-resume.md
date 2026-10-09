# CAN Automatic Resume Implementation Plan

> Root owns deployment and hardware checks. Workers implement/review only; they do not operate Jetson or motors.

**Goal:** restore remote manual driving after a bounded CAN-only interruption without replaying old input or releasing intentional stops.

**Architecture:** firmware communicates recovery provenance and atomically guards RESUME; the sole ChassisManager owner coordinates six axes. ROS preserves actual TCP receipt time and server connection identity.

**Spec:** `docs/specs/2026-09-10-can-auto-resume.md`

## Constraints

Preserve unrelated work and the closed optimization/simulation tracks. No autonomy changes, NVM setting changes, implicit operator E-stop clear, MCU reset resume or stale command replay. User authorized test/deployment while not driving. No additional permission gate is introduced; commit/push remain separate.

## Completed work

- [x] Firmware: failing real-source tests first, final patch3/A3 provenance/generation and guarded RESUME, prepare/build identity checks. Final candidate is in evidence/build-patch3, not the intermediate patch2 build.
- [x] Host: failing tests for marked/unmarked/mixed recovery, coordinated stop/rearm, missing-peer origins, fresh source input, disconnect/epoch changes, explicit stops and bounded timeout. Implemented pure helper, driver integration and thin ROS wiring.
- [x] Review: independent source and host review. Closed partial dispatch on stale steering, missing/rebooted peer exemption, rapid reconnect edge loss, cached TCP input and separate clear/arm race.
- [x] Acceptance: host1151, Jetson ROS231, firmware-source86, preparation6; installed ROS/TCP/vcan7cases. Counts overlap where stated in the report. Negative controls remain preserved.
- [x] Hardware: current full flash read twice; exact-serial application-only patch3 update/readback on13/14 and15/16; NVM/config/calibration preserved; physical IDLE guarded-request rejection and zero-output checks passed.
- [x] Deploy: canonical Jetson25file identity check,3package ROS rebuild, installed module/message verification, daily robot-start, component-mask restore, service/CAN/wheel/console verification. Test container stopped; production IDLE.
- [x] Report: final versions, backup hashes, evidence index, board coverage, physical limits and uncommitted local/Jetson/Git state documented in `docs/reports/2026-09-10-can-auto-resume.md`.

## Validation decisions

- Error0x4000 alone cannot prove CAN recovery origin; older unmarked firmware is not automatically eligible.
- Affected axes receive one atomic guarded RESUME; healthy continuously fresh peers receive no clear. The firmware checks generation, errors and pending HAL work inside the critical section.
- Fresh input means actual TCP receipt after re-arm completion, bound to payload and server accept UUID. DDS receipt and gateway republication cannot rejuvenate it.
- Every observed missing axis needs its own origin. All steering/drive samples are checked before releasing WAIT_INPUT.
- Physical acceptance in this task is application/config/readback and zero-output negative checks. Nonzero motion under a real HAL fault, node11/12 firmware update and repeated ground/power-cycle acceptance remain separate.
