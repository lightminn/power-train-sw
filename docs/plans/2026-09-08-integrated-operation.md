# Integrated Operation Implementation Plan

> **For agentic workers:** Use superpowers:subagent-driven-development. Follow RED → GREEN and review each task. Do not commit, push, deploy, or actuate hardware in this task.

**Goal:** One Jetson start command and one laptop console launch connect the existing operation features.

**Architecture:** Opt-in session proxies wrap existing raw-input and ops protocols. GTK manages a separate gamepad child. Telemetry destinations follow a single authenticated lease.

**Tech Stack:** Python standard library, existing GTK/GStreamer, pygame, ROS2 adapters, Docker Compose.

**Spec:** `docs/specs/2026-09-08-integrated-operation.md`

## Global constraints

- Preserve existing dirty work, retired tracks, raw-input schema, role tokens and ESTOP/reset separation.
- No automatic arm, calibration, component disabling or restored motion on reconnect.
- New integrated transport uses public 9000/9001/9002; inner ROS binds 127.0.0.1:19000/19001.
- Parent owns integration and telemetry adaptation; delegated tasks have disjoint files.

## Task 1: Robot session transport and client

Files: create `powertrain_runtime/{__init__,session,client,config,destination}.py`, `powertrain_runtime/tests/test_session.py`.
Interfaces: `SessionServer(config, on_start, on_stop, state_provider)` with `start()/close()`;
`SessionClient(config)` with `connect()/heartbeat()/request(op)/open_channel(kind)/close()`;
`open_session_channel(host,port,lease_id,ticket,kind)`; client `.snapshot` and `.host`.
Server config keys: robot_id, token_file, host, session_port, input_port, ops_port,
input_target_port, ops_target_port, destination_file, lease_timeout_s.
Client config keys: robot_id, hosts, session_port, token_file. Snapshot includes
connected, robot_id, boot_id, lease_id, ticket, input_port, ops_port and service state.
Channel auth is performed before forwarding to the inner service.

- [x] RED: run a real server with a wrong key and assert connect is rejected; assert a second client cannot acquire ownership and lease expiry closes proxied sockets.
- [x] GREEN: implement mutual challenge, single lease, heartbeats, bounded records, proxy cleanup, atomic destination record and paired IPv4 candidates.
- [x] Verify actual forwarding, wrong robot, changed IP, callback failures and shutdown with real sockets. No ROS dependency.

## Task 2: Supervised gamepad child

Files: create `motor_control/laptop/controller_process.py` and focused tests.
Executable: `python -m motor_control.laptop.controller_process`; JSONL stdin commands
`{"op":"connect","host":...,"input_port":9000,"lease_id":...,"ticket":...}`,
`{"op":"heartbeat"}`, `{"op":"disconnect"}`, `{"op":"shutdown"}`.
JSONL stdout status: `{"pad_connected":bool,"neutral":bool,"input_connected":bool,"detail":str}`.
Use Task 1 `open_session_channel`; existing `DualSenseInputAdapter`, mapping and
`encode_frame` are authoritative. Do not open a second ops channel or execute
controller recovery chords in integrated mode. Preserve pad ESTOP edge and haptics
when provided existing ops state by parent (`{"op":"ops_state","state":...}`).

- [x] RED: missing pad waits; unplug closes channel; stdin EOF/heartbeat expiry stops; no stale sample after reconnect.
- [x] GREEN: implement injectable pad source and command reader, bounded status, console singleton and deterministic child shutdown.
- [x] Verify a real subprocess and authenticated fake server; keep legacy CLI unchanged.

## Task 3: Prepared Jetson and laptop launchers

Files: create `scripts/robot-start`, `robot_prepare.sh`, `install_integrated_console.sh`, integrated Compose overlay, tests; modify only control launch host/port declarations and teleop/ops server bind parameters.
Runtime server entrypoint is `python3 -m powertrain_runtime.robot_service --config /etc/powertrain/robot.json` (parent implements).

- [x] RED: actual shell entrypoint must refuse missing prepared manifest/config, never build at startup, start idempotently without operator IP, preserve legacy compose.
- [x] GREEN: separate build/install from startup, service loopback ports, destination-file environment, literal CAN profile, desktop launcher uses `python -m operator_console`.
- [x] Verify Compose config and fake-PATH execution. No real install/system writes.

## Task 4: Drive orchestration, telemetry and GTK integration

Files: create `powertrain_runtime/{robot_service,drive_service}.py`, `operator_console/{operation_runtime,__main__}.py`; adapt app/ops client and three telemetry senders; tests and runbook.

- [x] RED: explicit held Start is required; stale/ESTOP/non-neutral state blocks; stages stop at failed/unknown ACK; reconnect never restarts driving.
- [x] GREEN: ManualDriveService uses existing OpsChannelClient and gates, SessionServer callbacks expose status; OperationRuntime owns config/session/child/ops and asynchronous UI actions.
- [x] RED: telemetry reaches new actual UDP endpoint after lease change and stops after expiry; GTK starts without robot/pad and never crashes.
- [x] GREEN: read destination record on send, wire readiness and Start/Stop into GTK, supervise parent/child lifetime and instance lock.
- [x] Execute integrated GTK and loopback server/UDP/child smoke, focused regression suites and independent whole-change review; document real-hardware blockers precisely.

## Handoff

Copy reviewed feature delta back to the user's original checkout only after checking its baseline hashes. Preserve all preexisting changes. Keep the isolated worktree for review; no commit/push. Update runbook and current-state references to distinguish implemented host paths from unperformed Jetson deployment/HIL.

## Verified implementation state — 2026-09-08

All four implementation tasks and the independent review are closed for the host/isolated-ROS scope. The final feature bundle is 86 passed; the GTK suite is 300 passed. The isolated ROS container passed 57 tests and the installed control launch start/stop smoke. Suite counts overlap and are not a combined total. See `docs/reports/2026-09-08-integrated-operation.md` for executed paths, fixes and acceptance limits.

Jetson hostname resolution failed in this session. Real deployment, both physical boot orders, SRT decode, gamepad/haptics, motor stopping, ESTOP and power-cycle calibration remain unperformed hardware acceptance. These unchecked hardware gates are not covered by the task checkboxes above.
