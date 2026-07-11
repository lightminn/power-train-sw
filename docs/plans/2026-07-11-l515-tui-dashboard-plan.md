# L515 Gateway and TUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build one headless L515 Gateway that owns SDK capture, existing ROS publication, RGB-aligned SRT streaming, and a Unix-socket control plane, plus a separate Textual Dashboard client suitable for remote driving.

**Architecture:** Gateway is the system-wide singleton and only `pyrealsense2` owner. It captures RGB 1280×720 and raw Depth 640×480, publishes the existing six ROS topics, aligns Depth internally for a fixed 1280×720 SRT canvas, and serves status/control over a Unix socket. Dashboard never owns hardware and can disconnect without stopping Gateway.

**Tech Stack:** Python 3.10, pyrealsense2 2.50.0, ROS 2 Humble/rclpy, Textual, NumPy/OpenCV, GStreamer/SRT, Unix sockets, pytest.

## Global Constraints

- Gateway is the sole L515 owner and exact serial remains `00000000F0271544`; D435i fallback is forbidden.
- Color input and every SRT mode are 1280×720×30; raw Depth is 640×480×30.
- Existing six ROS topics remain; no aligned-depth, PointCloud2, IR, confidence, or extra image topic is added.
- Modes are RGB, RGB-aligned Depth, and RGB+Depth alpha overlay; switching never restarts SDK or GStreamer.
- SRT defaults remain port 5000, latency 60 ms, x264, 3000 kbit/s.
- Dashboard/SSH exit must not stop Gateway, ROS, or SRT.
- Gateway shutdown is idempotent and leaves no SDK handle, child, abstract socket listener, or held flock.
- `resource_guard` may be reusable later but this plan changes no US-100, ODrive USB, CAN, or DualSense runtime.
- Preserve unrelated user and Jetson checkout changes.

---

### Task 1: Package, configuration, and dependencies — COMPLETE

**Commits:** `a763d47`, `d490657`, `c32c0ec`

- [x] Immutable strict numeric configuration and ROS image dependencies.
- [x] Textual, NumPy, OpenCV, and GStreamer dependencies.
- [ ] Extend configuration with abstract endpoint `@powertrain-l515-gateway`, color/depth profiles, overlay alpha, reconnect interval, message size, and persistent resource-lock path; use strict validation and tests.
- [ ] Commit the extension with the first Gateway task that consumes it.

### Task 2: Diagnostic snapshot engine — COMPLETE

**Commits:** `816ac0c`, `8ad0e64`

- [x] Six-topic bounded FPS/age/gap/timestamp diagnostics.
- [ ] Generalize input keys so Gateway can add SDK, ROS, SRT, and control-plane counters without retaining frame/message objects.
- [ ] Preserve existing public six-topic behavior and regression tests.

### Task 3: Frame modes — COMPLETE, ADAPTATION REQUIRED

**Commit:** `be74488`

- [x] Deterministic Depth colormap and latest-one-slot handoff.
- [ ] Replace 640/1280 output shapes with fixed 1280×720 RGB, aligned Depth, and overlay outputs.
- [ ] Accept only already aligned 1280×720 Depth in render layer; SDK alignment belongs to Gateway capture.
- [ ] Assert RGB identity, Depth dimensions, overlay alpha/content, contiguity, mode switching, and stale replay 0.

### Task 4: Singleton resource guard and Gateway SDK source

**Files:**
- Create: `l515_dashboard/resource_guard.py`
- Create: `l515_dashboard/gateway_source.py`
- Create: `l515_dashboard/tests/test_resource_guard.py`
- Create: `l515_dashboard/tests/test_gateway_source.py`
- Modify: `l515_dashboard/config.py`
- Modify: `l515_dashboard/frame_modes.py`
- Modify: corresponding tests

**Interfaces:**
- Produces `ResourceGuard.acquire()/release()` using an owner-checked persistent regular file and nonblocking exclusive `flock`; the file is never unlinked.
- Produces `GatewayFrames(raw_color, raw_depth, aligned_depth, accel, gyro, mapper)`.
- Produces `L515GatewaySource.start()/poll_latest()/stop()` with exact-serial reconnect.

- [ ] Write RED two-contender, release/reacquire, stale-file, symlink-rejection, and no-unlink tests.
- [ ] Implement reusable persistent `flock` guard; write/fsync owner metadata while locked and never signal unrelated processes.
- [ ] Bind-mount host `/run/powertrain` into `powertrain_ros` at the same path so replacement containers contend on one lock inode.
- [ ] Before compose deployment, run the root-only idempotent tmpfiles installer; verify root:root 0750 and use `bind.create_host_path: false` so missing provisioning fails closed across fresh boots.
- [ ] Write RED fake-SDK tests for exact color/depth profiles, `rs.align(color)`, raw versus aligned separation, IMU, latest-one-slot, dedup, disconnect/reconnect reset, and bounded stop.
- [ ] Implement one SDK pipeline owner based on proven `L515Source` lifecycle without duplicating its race bugs.
- [ ] Adapt frame modes/config to fixed 1280×720 and overlay; remove incompatible variable-width streamer assumptions.
- [ ] Run Tasks 1–4 regressions and existing L515 source tests; commit as `feat(l515): add singleton Gateway source`.

### Task 5: Gateway ROS publisher and fixed-canvas SRT worker

**Files:**
- Create: `l515_dashboard/gateway_ros.py`
- Modify: `l515_dashboard/streamer.py`
- Create/modify: `l515_dashboard/tests/test_gateway_ros.py`, `test_streamer.py`
- Modify: `ros2/src/powertrain_ros/setup.py` only to retire/redirect the old entrypoint when Gateway is ready.

**Interfaces:**
- Consumes `GatewayFrames` and existing L515 message adapters.
- Publishes only the six approved topics.
- `SrtStreamer` consumes fixed 1280×720 frames and never changes child caps.

- [ ] Write RED tests for color CameraInfo 1280×720, raw Depth CameraInfo 640×480, IMU, timestamp/dedup, and absence of extra topics.
- [ ] Implement nonblocking ROS publication from the Gateway frameset.
- [ ] Replace Task-4 variable-width GStreamer startup with one exact 1280×720 argv for every mode.
- [ ] Fix reviewed partial-start leak, wait timeout, concurrent stop, in-flight write, post-stop mutation, and independent argv contract tests.
- [ ] Test RGB→Depth→overlay switching preserves the same fake child identity and exact frame cadence.
- [ ] Run focused and ROS clean regressions; commit as `feat(l515): publish ROS and SRT from Gateway`.

### Task 6: Versioned Unix control server and Gateway lifecycle

**Files:**
- Create: `l515_dashboard/protocol.py`
- Create: `l515_dashboard/control_server.py`
- Create: `l515_dashboard/gateway.py`
- Create: `l515_dashboard/gateway_main.py`
- Create: protocol/server/gateway tests

**Interfaces:**
- Newline JSON envelope: `protocol_version`, `request_id`, `type`, `payload`.
- Commands: get_status, set_video_mode, set_streaming, restart_gateway, stop_gateway.
- Produces headless `python3 -m l515_dashboard.gateway_main`.

- [ ] Write RED framing/version/size/invalid-command and multiple-client backpressure tests.
- [ ] Implement bounded status snapshots and serialized state-changing commands.
- [ ] Implement Gateway states and lifecycle: shared guard→abstract socket bind→SDK→ROS→optional SRT→RUNNING. Duplicate lock/bind must fail before SDK access.
- [ ] Treat L515 loss as DEGRADED/reconnect, GStreamer crash as streaming-off DEGRADED, ROS fatal as FAULT shutdown, Dashboard disconnect as no-op.
- [ ] Route SIGINT/SIGTERM/container stop/exception through one idempotent cleanup order: frame intake→SRT→SDK→ROS→abstract socket close→flock unlock.
- [ ] Test partial starts, signals, concurrent commands, repeated shutdown, and zero owned resources.
- [ ] Run regressions and commit as `feat(l515): add headless Gateway control service`.

### Task 7: Textual Dashboard client and real-process integration

**Files:**
- Create: `l515_dashboard/client.py`
- Create: `l515_dashboard/app.py`
- Create: `l515_dashboard/__main__.py`
- Create: client/app/process integration tests and fake Gateway helper
- Create: `l515_dashboard/README.md`
- Modify: `README.md`, `ros2/README.md`, `AGENTS.md`, Docker entrypoint/compose as required

**Interfaces:**
- Dashboard is socket-only; it imports neither pyrealsense2 nor ROS Image types.
- `q` exits client only; `Shift+Q` confirms stop_gateway.

- [ ] Write RED reconnect, version mismatch, stale status, command acknowledgement, and disconnect tests.
- [ ] Write Textual pilot tests for state/SDK/ROS/SRT/resources/errors and keys 1/2/3, streaming toggle, restart, q, Shift+Q.
- [ ] Implement Dashboard with automatic abstract-socket reconnect and immutable snapshots; server enforces same-UID `SO_PEERCRED` authorization.
- [ ] Prove q/SIGHUP/client crash leave fake Gateway and its SRT child alive; explicit stop reaps them.
- [ ] Document remote-driving operation, singleton failures, receiver command, and maintenance exclusion.
- [ ] Run full software/ROS/process suites; commit as `feat(l515): add Gateway Dashboard client`.

### Task 8: Jetson HIL, final review, and publication

**Files:**
- Create: `docs/reports/2026-07-11-l515-gateway-dashboard-hil.md`
- Modify: autonomous plan, state report, active Software Notion page after fetch

- [ ] Audit and exact-archive deploy with unique image; preserve both Jetson checkouts and production containers.
- [ ] Verify Gateway singleton, RGB 1280×720, raw Depth 640×480, six ROS topics, and SRT 1280×720×30.
- [ ] Switch RGB/Depth/overlay while SDK/GStreamer PIDs remain unchanged; measure rates, latency, CPU/RAM, USB delta.
- [ ] Kill/close Dashboard and SSH; prove Gateway, ROS, and SRT continuity, then reconnect Dashboard.
- [ ] User-approved L515 unplug/replug: DEGRADED, stale replay 0, no D435 fallback, automatic ROS/SRT recovery.
- [ ] Crash GStreamer: ROS continues; restart streaming without Gateway/SDK restart.
- [ ] Stop Gateway via TERM/container stop; prove SDK, GStreamer, abstract listener, and held-flock count 0; persistent lock file remains.
- [ ] Run D435i perception concurrently and verify `/detected_objects` continuity and USB error delta 0.
- [ ] Run final clean tests and two-stage review; fix every Critical/Important finding.
- [ ] Update docs/Notion with fetch-before/write/re-fetch, commit, push feature branch, and restore Jetson state.
