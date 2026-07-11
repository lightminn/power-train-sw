# L515 TUI Dashboard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Textual TUI that owns the L515 ROS publisher and H.264/SRT streamer as one safely supervised stack, exposes live diagnostics, switches three video modes, and leaves zero orphan processes on every exit path.

**Architecture:** Keep `l515_camera` as the only RealSense owner. The dashboard starts it in a dedicated process group, subscribes to ROS topics for diagnostics and video, and feeds the existing `gst_stream.py` SRT pipeline through a latest-frame worker. One idempotent supervisor shutdown path handles normal exit, signals, partial startup, child crashes, and escalation.

**Tech Stack:** Python 3.10, ROS 2 Humble/rclpy, Textual, NumPy, OpenCV, GStreamer/SRT, pytest.

## Global Constraints

- Work only on `feat/l515-lightweight-pipeline`; preserve unrelated user files and Jetson checkouts.
- `l515_camera` remains the only RealSense owner; the dashboard never opens `pyrealsense2`.
- Canonical serial is `00000000F0271544`; D435i fallback is forbidden.
- SRT defaults are port 5000, latency 60 ms, x264, 3000 kbit/s.
- Color/Depth are 640×480×30; side-by-side is 1280×480×30.
- States are `STARTING`, `RUNNING`, `DEGRADED`, `STOPPING`, `STOPPED`, `FAULT`.
- Every dashboard-owned process and process group must be gone after every exit path.
- Camera loss never repeats stale frames; recovery resumes only after fresh required topics.
- PointCloud2, IR, confidence, alignment, detection, odometry, recording, and web control are out of scope.

---

### Task 1: Package, configuration, and dependencies

**Files:**
- Create: `l515_dashboard/__init__.py`
- Create: `l515_dashboard/config.py`
- Create: `l515_dashboard/tests/__init__.py`
- Create: `l515_dashboard/tests/test_config.py`
- Modify: `docker/Dockerfile.ros`

**Interfaces:**
- Produces immutable `DashboardConfig` for all later tasks.

- [ ] Write RED tests asserting `(port, latency_ms, encoder) == (5000, 60, "x264")`, 640×480×30 defaults, and rejection of port 0, unknown encoder, and nonpositive timeouts.
- [ ] Run `python3 -m pytest -q l515_dashboard/tests/test_config.py`; expect import failure.
- [ ] Implement frozen `DashboardConfig` with bitrate 3000, startup timeout 10 s, graceful timeout 3 s, termination timeout 2 s, and exact validation.
- [ ] Add `textual`, `numpy`, `opencv-python-headless`, GStreamer CLI and base/good/bad/ugly plugins to the ROS image; retain the existing librealsense 2.50.0 build.
- [ ] Run the focused tests and `git diff --check`; expect PASS.
- [ ] Commit as `build(l515): scaffold TUI dashboard runtime`.

### Task 2: Diagnostic snapshot engine

**Files:**
- Create: `l515_dashboard/diagnostics.py`
- Create: `l515_dashboard/tests/test_diagnostics.py`

**Interfaces:**
- Produces `DiagnosticsTracker.observe(topic: str, stamp_ns: int, now_ns: int) -> None`.
- Produces immutable `DiagnosticsTracker.snapshot(now_ns: int) -> DiagnosticsSnapshot`.

- [ ] Write RED tests feeding three arrivals, including one equal stamp, and assert count, rolling FPS, age, maximum gap, and `nonincreasing_count == 1`.
- [ ] Define the six exact L515 topic constants once and bounded arrival deques per topic.
- [ ] Implement explicit video/CameraInfo/IMU freshness thresholds and an aggregate `healthy` result.
- [ ] Test that snapshots retain no ROS message or image object and old arrivals leave the rolling window.
- [ ] Run `python3 -m pytest -q l515_dashboard/tests/test_diagnostics.py`; expect PASS.
- [ ] Commit as `feat(l515): add dashboard diagnostics engine`.

### Task 3: Video modes and latest-frame handoff

**Files:**
- Create: `l515_dashboard/frame_modes.py`
- Create: `l515_dashboard/tests/test_frame_modes.py`

**Interfaces:**
- Produces `FrameMode.COLOR`, `FrameMode.DEPTH`, `FrameMode.SIDE_BY_SIDE`.
- Produces `render_frame(mode, color, depth, width, height) -> np.ndarray | None`.
- Produces thread-safe `LatestVideoFrames.put_color`, `put_depth`, and `take(mode)`.

- [ ] Write RED parameterized tests for contiguous uint8 BGR shapes 480×640×3, 480×640×3, and 480×1280×3.
- [ ] Implement Color passthrough, zero-aware fixed-range Depth normalization with OpenCV TURBO colormap, and horizontal composition.
- [ ] Test missing selected input returns `None`, never a black or previous frame.
- [ ] Test two puts before take return only the newest, and a second take returns `None`.
- [ ] Run the focused tests and commit as `feat(l515): add dashboard video modes`.

### Task 4: GStreamer SRT worker

**Files:**
- Create: `l515_dashboard/streamer.py`
- Create: `l515_dashboard/tests/test_streamer.py`
- Modify only if needed: `motor_control/vision/gst_stream.py`

**Interfaces:**
- Consumes `DashboardConfig`, `FrameMode`, `LatestVideoFrames`, and existing `build_gst_command`.
- Produces `SrtStreamer.start()`, `set_mode()`, `submit_color()`, `submit_depth()`, `stop()`.
- Produces immutable `StreamerSnapshot` with running, mode, sent, dropped, and last_error.

- [ ] Write RED fake-Popen tests asserting exact SRT argv and 640-wide versus 1280-wide output selection.
- [ ] Implement one condition-driven latest-frame worker; it may perform one bounded stdin write but may not queue or replay frames.
- [ ] Test `BrokenPipeError` and unexpected child exit set last_error and stop the worker.
- [ ] Test repeated stop closes stdin and reaps the process exactly once.
- [ ] Preserve the existing proven x264/openh264 command contract in `gst_stream.py`.
- [ ] Run focused tests and commit as `feat(l515): add ROS-fed SRT worker`.

### Task 5: Process identity, lock, and orphan-proof supervisor

**Files:**
- Create: `l515_dashboard/child_process.py`
- Create: `l515_dashboard/lockfile.py`
- Create: `l515_dashboard/supervisor.py`
- Create: `l515_dashboard/tests/test_child_process.py`
- Create: `l515_dashboard/tests/test_lockfile.py`
- Create: `l515_dashboard/tests/test_supervisor.py`

**Interfaces:**
- Produces `StackState` with the six approved values.
- Produces thread-safe `DashboardSupervisor.start()`, `restart()`, `shutdown(reason)`, `snapshot()`.
- `shutdown()` returns only after every owned child is reaped and is safe to call repeatedly.

- [ ] Write RED lock tests storing PID plus `/proc/<pid>/stat` start time; reject only a live matching identity and recover stale/mismatched locks.
- [ ] Implement `OwnedProcess` with `start_new_session=True` and Linux `PR_SET_PDEATHSIG=SIGTERM`; retain the originating Popen object and PGID.
- [ ] Write RED escalation tests for SIGINT→SIGTERM→SIGKILL timeouts using only the owned process group.
- [ ] Write a RED lifecycle matrix failing at preflight, ROS spawn, topic-ready wait, and GStreamer spawn; every case must end FAULT with zero owned children and no lock.
- [ ] Implement start order lock→preflight→ROS→topics-ready→streamer→RUNNING.
- [ ] Implement shutdown order stop frame intake→streamer→ROS→subscriber→lock and route q, signals, exceptions, child crashes, and atexit through it.
- [ ] Test concurrent repeated shutdown invokes each escalation once and `restart()` waits for STOPPED before spawning.
- [ ] Run the three focused test files and commit as `feat(l515): add orphan-proof dashboard supervisor`.

### Task 6: ROS bridge and Textual application

**Files:**
- Create: `l515_dashboard/ros_bridge.py`
- Create: `l515_dashboard/app.py`
- Create: `l515_dashboard/__main__.py`
- Create: `l515_dashboard/tests/test_ros_bridge.py`
- Create: `l515_dashboard/tests/test_app.py`

**Interfaces:**
- Produces `python3 -m l515_dashboard [--port N --latency-ms N --encoder NAME]`.

- [ ] Write RED fake-message tests: all six headers update diagnostics, BGR8/16UC1 images have exact shapes, and only image topics reach video slots.
- [ ] Implement a dedicated rclpy executor thread using sensor-data QoS; callbacks only update bounded trackers/latest slots.
- [ ] Write Textual pilot tests for six stream rows, state, SRT/resources/errors, `1/2/3`, `r`, `q`, and `?`.
- [ ] Implement snapshot-only TUI refresh and signal handlers for SIGINT, SIGTERM, SIGHUP that schedule the same shutdown.
- [ ] Keep `try/finally` and atexit as final idempotent guards.
- [ ] Run both focused test files and commit as `feat(l515): add diagnostic Textual dashboard`.

### Task 7: Real subprocess integration and operations docs

**Files:**
- Create: `l515_dashboard/tests/helpers/fake_child.py`
- Create: `l515_dashboard/tests/test_process_integration.py`
- Create: `l515_dashboard/README.md`
- Modify: `README.md`
- Modify: `ros2/README.md`
- Modify: `AGENTS.md`

**Interfaces:**
- Verifies the complete process contract locally without camera hardware.

- [ ] Create a child fixture that records signals, optionally ignores INT/TERM, spawns a grandchild, crashes on command, and exposes PID/PGID evidence.
- [ ] Test normal exit, partial startup, ROS crash, GStreamer crash, INT, TERM, HUP, forced KILL escalation, and repeated shutdown.
- [ ] After every scenario poll `/proc` and assert every owned child and grandchild is absent.
- [ ] Document the exact container command, keys, state meanings, receiver command, recovery behavior, and orphan audit; forbid concurrent `realsense_stream.py`.
- [ ] Run all dashboard tests, the existing clean ROS suite, flake8, bash syntax checks, and `git diff --check`.
- [ ] Commit as `docs(l515): document TUI dashboard operations`.

### Task 8: Jetson HIL, final review, and publication

**Files:**
- Create: `docs/reports/2026-07-11-l515-tui-dashboard-hil.md`
- Modify: `docs/plans/2026-07-02-autonomous-driving-kickoff.md`
- Modify after fetch: relevant active Software Notion page

**Interfaces:**
- Produces final hardware evidence and restores the original Jetson process/repository state.

- [ ] Audit both Jetson checkouts and running processes; transfer the exact commit with `git archive`, use a unique image tag, and never retag production.
- [ ] Start the TUI and laptop `scripts/recv_stream.sh`; verify diagnostics, 30 Hz-class input/SRT, and keys 1/2/3 without process restart.
- [ ] Record CPU/RAM, SRT rate, mode transitions, node/GStreamer errors, and USB error deltas.
- [ ] Ask immediately before one user-controlled L515 unplug/replug; verify DEGRADED, no stale replay, no D435 fallback, same process identities, and automatic SRT recovery.
- [ ] Run q, INT, TERM, and HUP as separate starts; after each prove dashboard-owned ROS, GStreamer, shell, child, and process group count is zero.
- [ ] Use a fake child for forced escalation evidence; do not force-kill a healthy camera SDK process.
- [ ] Remove only temporary containers and restore robot-arm/perception state exactly; recheck both dirty checkouts.
- [ ] Write the HIL report, update repo docs and active Software Notion pages with fetch-before/write/re-fetch verification.
- [ ] Request spec compliance review and whole-branch quality review; fix every Critical/Important finding.
- [ ] Run final clean dashboard and ROS suites, verify a clean worktree, commit, and push `feat/l515-lightweight-pipeline`.
