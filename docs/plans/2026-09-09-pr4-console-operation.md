# PR #4 Console Operation Implementation Plan

> **For agentic workers:** Use superpowers:subagent-driven-development task-by-task.

**Goal:** Preserve PR #4's GUI while restoring complete console operation and correcting misleading status.

**Architecture:** Keep the existing session, authenticated ops, manual-start and hardware ownership boundaries. Add a reachable recovery window to the PR layout, correct display logic, and exercise the real local operation path with fixtures only at hardware boundaries.

**Tech Stack:** Python, GTK3/GStreamer, pytest, local TCP/UDP fixtures.

**Spec:** `docs/specs/2026-09-09-pr4-console-operation.md`

## Global Constraints

- Worktree only; no Jetson/SSH/real CAN/USB/camera/gamepad access, no deployment or service changes.
- No commit, push, merge, PR comments, or external writes before final user approval.
- Preserve two PR tabs, PiP and sensor/tool popups. Keep authenticated ops and explicit 1.5-second start.
- Read-only reference main is `b22cd9e`, PR base is `4ec2c5f`.
- Use system Python for GTK; conda base for ROS-free tests. Set PYTHONPATH to repo, motor_control and ros2/src/powertrain_ros as needed.
- Xvfb runs must use unique display numbers when run concurrently, and force X11 and GDK_SCALE=1.
- Keep RED/GREEN output and runtime evidence. Hardware acceptance is not implied.

## Task 1: Restore operational access and correct PR display errors

**Files:**
- Modify `operator_console/app.py`, `operator_console/status_view.py`, `operator_console/__main__.py`.
- Extend `operator_console/tests/test_integrated_app.py`, `test_interactions.py`, and appropriate entrypoint tests.
- Update `operator_console/README.md` and `docs/integrated-operation.md` for the new recovery entry.

**Interfaces:** Existing `OpsPanel`, `IntegratedOperationPanel`, `OperationRuntime` and `CompetitionStatusDashboard.update` stay compatible. Use `_ops_settings_button`, `_ops_settings_window` and `_show_ops_settings` for the new recovery window so the local E2E test can interact with it. No new network operation is needed.

- [x] Write failing GTK regression cases before code changes:

```python
window._ops_settings_button.clicked()
pump()
assert window._ops_panel._action_buttons['estop_reset'].get_mapped()
assert window._ops_panel._action_buttons['us100_enable'].get_mapped()
assert window._ops_panel._action_buttons['steer_mode_skid'].get_mapped()
assert window._operation_panel.start_button.get_mapped()
assert window._global_estop.get_mapped()
```

Also cover hiding/reopening the recovery window and parent shutdown; opening must send no action.
Keep the same ops panel/client instance when reopening and cancel active confirmation on hide.

```python
window._add_event('SYSTEM', 'ERROR 구동 고장')
window._add_event('SYSTEM', 'NOTICE 상태 수신')
window._mission_events._filters['INFO'].set_active(False)
window._refresh_health()
assert '구동 고장' in window._mission_event_latest.get_text()
```

Build fresh power/chassis/arm/metadata fixtures with otherwise normal states and independently
parametrize wheel stale, axis error, steer fault and CAN/drive unavailable. Assert the visible
competition summary is not ready, and fully healthy fixtures can be ready. For environment,
warmup=True must label both air readings as warming and reduce normal count by two; warmup=False
returns normal and stale still clears all values.

- [x] Run focused cases under Xvfb; record expected failures, not import/environment errors.
- [x] Implement small `복구 · 설정` button near the header navigation. Reuse the existing ops page
  inside a scrollable nonmodal transient Gtk.Window, close-to-hide with confirmation cancellation,
  parent-owned destruction. Keep the PR's two main tabs and existing start/stop/ESTOP visible.
- [x] Add drive health to the visible competition readiness condition using a shared existing
  decision/helper rather than duplicating its fault logic. Render warming SGP30 readings separately.
- [x] Read mission latest text from `_mission_events`; preserve independent filters.
- [x] Include `environment_telemetry_port=5008` in validated integrated config forwarding; add
  behavioral test for override and invalid port rejecting before network/session startup.
- [x] Address direct-render findings with scoped dark styles for integrated-operation/recovery
  controls and constrain the integrated window to a usable 1366x768 laptop layout. Preserve
  PR's two tabs, PiP and sidebar content; no broad theme or layout redesign.
- [x] Run focused tests green and full console suite once. Update user-facing docs for the exact
  recovery window route. Report files, test commands, RED/GREEN and remaining concerns.

## Task 2: Demonstrate the local complete driving path and prepare merge evidence

**Files:**
- Add `operator_console/tests/test_console_operation_e2e.py` using test helpers where appropriate.
- Improve `operator_console/integrated_runtime_smoke.py` only where needed to cover the integrated
  entrypoint/environment port without weakening existing startup/lifecycle assertions.
- Repair the reproduced actual SRT restart failure in `operator_console/app.py` and
  `operator_console/pipelines.py` if named demux/parser elements are needed. Add a real
  decoded-stream restart regression; preserve GTK sink ownership and cleared old-frame state.
- Reuse `powertrain_runtime/tests/test_integrated_operation.py` with a small optional hardware
  executor hook, preserving its existing default fixture behavior.
- Extend `drive_card_state` and `test_interactions.py` for the newly reproduced production
  `ESTOP/ESTOP` and wheel FAULT readiness omission. Preserve normal IDLE/RUN and ARMED/RUN;
  use the same ESTOP decision in the visible safety badge/ESTOP value. Do not change control
  policy or introduce unrelated mode classifications.
- Update `docs/reports/2026-09-09-pr4-console-operation.md` with actual results and limitations.
- Modify `docker/docker-compose.integrated.yml` for the two missing `unless-stopped` overrides
  (`powertrain_ros`, `powertrain_pdist80b_telemetry`) and test the effective merged Compose model
  in `scripts/tests/test_integrated_launchers.py`. No new host unit or daemon mutation.

**Interfaces:** Task 1 window button/window names above; production SessionServer,
ManualDriveService, OpsBrokerCore and console's real OpsClient. Only hardware execution, pad
input and external robot are fixtures. Reuse the protocol fixture shape in
`powertrain_runtime/tests/test_integrated_operation.py` and do not mock the ops/session transport.

- [x] Read current prepare/robot-start/restart policies and record the actual one-time prerequisites
  for console-only daily operation. Resolve any need for boot support from this evidence.
- [x] First assert the effective eight-service Compose model uses `unless-stopped`; confirm
  the current camera/power policies fail, apply only the two overrides, then confirm GREEN.
  Document first commissioning and explicit maintenance-stop behavior.
- [x] Add a local GTK→session TCP→manual start→OpsBrokerCore integration test. Start from ESTOP,
  open the visible recovery window, execute explicit confirmed reset, keep no automatic arm,
  then hold the real start button with fresh video/pad evidence and assert the ordered
  `authority_manual` then `arm` execution and terminal state. Click stop and assert IDLE/authority
  release, then visible ESTOP and explicit reset; reconnection never adds another arm.
- [x] Exercise actual GStreamer SRT test-source reception if available; otherwise label video
  freshness boundary fixture explicitly and record the unverified decoded-video gate.
- [x] Require fresh decoded video again after session reconnect. Reproduce demux not-linked
  after endpoint restart with the production sender options, then verify the minimal repair.
- [x] Run a negative control (temporarily restore unreachable recovery/incorrect readiness in
  this isolated worktree, run relevant test, confirm FAIL, restore) for new runtime assertions.
- [x] Run complete console suite, relevant runtime/ops tests, both runtime smokes and diff check.
- [x] Render and directly inspect integrated main screen, recovery window and system/environment
  states at normal laptop size. Keep screenshots in ignored validation artifacts.
- [x] Independent review with spec and quality verdicts, fix reproducible findings, then compare
  PR/current main SHAs again and document the final diff/test/merge handoff.
- [x] Stop before commit/push/merge/deployment and request the user's final permission with the
  concrete result, validation and outstanding physical acceptance limits.

## Final acceptance

Latest local main `7c41cd8` plus the complete PR and fixes applied without conflicts.
Final console: 363 passed / 3 pygame skips, all three passed with conda.
Runtime/launcher/smoke-helper/relevant CAN: 135 passed. Both actual smokes passed.
Independent final review: Ready to merge, no open findings. Preparation completed without
commit/push/merge/deploy. The user subsequently approved commit/push/PR #4 merge on 2026-09-09.
Deployment and hardware acceptance remain outside that approval. Validation evidence is preserved.
