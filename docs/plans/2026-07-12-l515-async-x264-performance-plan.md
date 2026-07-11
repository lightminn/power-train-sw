# L515 Async Capture and x264 Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver independent 1280×720 ROS color and SRT paths at a measured 29.0 Hz or better on the Jetson Orin Nano while bounding lower-priority Depth, alignment, and IMU work.

**Architecture:** One RealSense pipeline uses the SDK asynchronous callback and immediately routes color, depth, accel, and gyro into stream-specific bounded handoffs. Independent workers publish ROS color at camera cadence, publish raw Depth at a deadline-based 10 Hz, publish each IMU stream at no more than 100 Hz, and feed an isolated software-x264 SRT process at RGB cadence. Expensive alignment runs only while Depth or overlay SRT mode needs it; RGB mode suppresses alignment so it cannot starve the 30 fps writer. No consumer may block the SDK callback or another consumer.

**Tech Stack:** Python 3.10, pyrealsense2 2.50.0, ROS 2 Humble/rclpy, NumPy/OpenCV, GStreamer `x264enc`, SRT, pytest, Docker Compose on Jetson Orin Nano Super.

## Global Constraints

- Exact L515 serial remains `00000000F0271544`; D435i fallback is forbidden.
- Color profile is BGR8 1280×720×30; raw Depth is Z16 640×480×30.
- Existing six ROS topics remain; no aligned-depth or additional ROS topic is added.
- ROS color image/CameraInfo and the **RGB-mode** SRT receiver must each measure at least 29.0 Hz over every complete 5 s window in the final 60 s observation. Depth and overlay SRT are functional best-effort modes whose measured rate is documented.
- Raw Depth image/CameraInfo are published at 10 Hz; accel and gyro are each capped at 100 Hz.
- SRT is fixed at 1280×720×30 in RGB, Depth, and overlay modes; mode changes do not restart SDK or GStreamer.
- Aligned Depth may be reused for RGB-paced output only while its age is at most 250 ms; older data stops Depth/overlay output and marks streaming DEGRADED.
- Jetson Orin Nano has no NVENC. `nvv4l2h264enc` is forbidden; software `x264enc` is the only approved encoder and no silent encoder fallback is allowed.
- SDK callbacks perform only stream identification, deduplication, timestamp/frame-number capture, one required ownership copy, and bounded handoff.
- Shutdown remains idempotent and leaves no SDK owner, GStreamer child, benchmark process, abstract listener, or held flock.
- Preserve unrelated local, Jetson checkout, robot-arm checkout, and production-container state.

---

### Task 1: Reproducible Orin Nano conversion and x264 benchmark

**Files:**
- Create: `scripts/benchmark_l515_x264.py`
- Create: `l515_dashboard/tests/test_x264_benchmark.py`
- Modify: `motor_control/vision/gst_stream.py`
- Test: `l515_dashboard/tests/test_x264_benchmark.py`

**Interfaces:**
- Produces `build_x264_benchmark_command(width: int, height: int, fps: int, conversion: str, sink: str) -> list[str]`.
- Produces a JSON result containing `conversion`, `attempted`, `encoded`, `elapsed_s`, `fps`, `cpu_percent`, and `returncode`.
- The only accepted `conversion` values are `videoconvert` and `nvvidconv`; both still use `x264enc`.

- [ ] **Step 1: Write failing command-contract tests**

```python
def test_benchmark_commands_are_software_x264_only():
    cpu = build_x264_benchmark_command(1280, 720, 30, "videoconvert", "fakesink")
    nv = build_x264_benchmark_command(1280, 720, 30, "nvvidconv", "fakesink")
    assert "x264enc" in cpu and "x264enc" in nv
    assert "nvv4l2h264enc" not in cpu + nv
    assert [token for token in nv if token == "nvvidconv"] == ["nvvidconv"]
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_x264_benchmark.py`

Expected: FAIL because `build_x264_benchmark_command` does not exist.

- [ ] **Step 3: Implement deterministic benchmark command generation**

```python
def build_x264_benchmark_command(width, height, fps, conversion, sink):
    if conversion not in {"videoconvert", "nvvidconv"}:
        raise ValueError("conversion must be videoconvert or nvvidconv")
    convert = (["videoconvert", "!", "video/x-raw,format=I420"]
               if conversion == "videoconvert" else
               ["videoconvert", "!", "video/x-raw,format=BGRx", "!",
                "nvvidconv", "!", "video/x-raw,format=I420"])
    return ["gst-launch-1.0", "fdsrc", "fd=0", "do-timestamp=true", "!",
            "rawvideoparse", "format=bgr", f"width={width}", f"height={height}",
            f"framerate={fps}/1", "!", *convert, "!", "x264enc",
            "tune=zerolatency", "speed-preset=ultrafast", "threads=3",
            "bitrate=3000", "key-int-max=30", "!", sink]
```

- [ ] **Step 4: Implement the bounded frame writer and JSON report**

Generate one reusable 1280×720 BGR frame, write exactly 900 frames unless the child exits, measure child CPU from `/proc/<pid>/stat`, always close stdin and reap the child, and return nonzero when encoded fps is below 29.0 or any child remains.

- [ ] **Step 5: Run local tests and static validation**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_x264_benchmark.py l515_dashboard/tests/test_streamer.py`

Expected: all PASS and no command contains `nvv4l2h264enc`.

- [ ] **Step 6: Run both exact benchmarks on Jetson without L515 ownership**

Run both conversions in the exact feature image for 900 frames to `fakesink sync=false`; preserve JSON and process-cleanup evidence. Select `nvvidconv` only if it exists, exits 0, and its median of three runs is at least 5% faster or 10 percentage points lower CPU than `videoconvert`; otherwise select `videoconvert`.

- [ ] **Step 7: Commit the benchmark and selected conversion**

```bash
git add scripts/benchmark_l515_x264.py l515_dashboard/tests/test_x264_benchmark.py motor_control/vision/gst_stream.py
git commit -m "perf(l515): benchmark Orin Nano x264 path"
```

---

### Task 2: Stream-specific asynchronous SDK capture

**Files:**
- Modify: `l515_dashboard/gateway_source.py`
- Modify: `l515_dashboard/tests/test_gateway_source.py`
- Create: `l515_dashboard/stream_buffer.py`
- Create: `l515_dashboard/tests/test_stream_buffer.py`

**Interfaces:**
- Produces immutable `StreamSample(stream: str, frame_number: int, timestamp_ms: float, received_ns: int, frame: object)`.
- Produces `LatestSlot.publish(sample) -> None` and `LatestSlot.read_after(sequence: int) -> tuple[int, StreamSample | None]`.
- Produces `BoundedRing(capacity: int).publish(sample)` and `.read_after(sequence, limit)`, dropping oldest samples when full.
- `L515GatewaySource.start()` calls `pipeline.start(config, callback)` and exposes independent readers for color, depth, accel, and gyro.

- [ ] **Step 1: Write RED bounded-handoff tests**

```python
def test_latest_slot_keeps_only_newest_without_blocking_producer():
    slot = LatestSlot()
    for number in range(1000):
        slot.publish(sample(number))
    sequence, value = slot.read_after(0)
    assert sequence == 1000
    assert value.frame_number == 999

def test_ring_drops_oldest_and_reports_drop_count():
    ring = BoundedRing(capacity=4)
    for number in range(10):
        ring.publish(sample(number))
    assert [s.frame_number for s in ring.read_after(0, 10).samples] == [6, 7, 8, 9]
    assert ring.dropped == 6
```

- [ ] **Step 2: Run focused tests and verify RED**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_stream_buffer.py l515_dashboard/tests/test_gateway_source.py`

Expected: FAIL because stream-specific buffer interfaces and async callback start are absent.

- [ ] **Step 3: Implement bounded handoffs**

Use one `threading.Lock` per handoff, monotonic sequence numbers, no condition wait in `publish`, latest-one slots for color/depth, and capacity-32 rings for accel/gyro. Frame ownership is copied once in the callback before publishing.

- [ ] **Step 4: Replace `wait_for_frames()` with one async callback**

```python
def _on_frame(self, frame, generation):
    if not self._is_current(generation):
        return
    profile = frame.get_profile()
    stream = profile.stream_type()
    sample = self._sample_from_frame(stream, frame)
    self._buffers[stream].publish(sample)
```

The callback must accept single motion frames and composite video frames, split composite children, deduplicate by `(stream, frame_number)`, and never call align, NumPy, ROS, GStreamer, sleep, or reconnect logic.

- [ ] **Step 5: Preserve reconnect and bounded shutdown behavior**

On disconnect, invalidate all buffers and the timestamp mapper for the generation. `stop()` calls pipeline stop once, waits at most the existing timeout, rejects late callbacks by generation, and leaves every buffer empty.

- [ ] **Step 6: Run source and lifecycle regressions**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_stream_buffer.py l515_dashboard/tests/test_gateway_source.py l515_dashboard/tests/test_gateway.py`

Expected: all PASS, including late-callback rejection, reconnect reset, and stop idempotence.

- [ ] **Step 7: Commit async capture**

```bash
git add l515_dashboard/gateway_source.py l515_dashboard/stream_buffer.py l515_dashboard/tests/test_gateway_source.py l515_dashboard/tests/test_stream_buffer.py
git commit -m "perf(l515): capture SDK streams asynchronously"
```

---

### Task 3: Independent ROS cadence workers and bounded alignment

**Files:**
- Modify: `l515_dashboard/gateway.py`
- Modify: `l515_dashboard/gateway_ros.py`
- Create: `l515_dashboard/gateway_workers.py`
- Create: `l515_dashboard/tests/test_gateway_workers.py`
- Modify: `l515_dashboard/tests/test_gateway.py`
- Modify: `l515_dashboard/tests/test_gateway_ros.py`

**Interfaces:**
- Produces `ColorWorker` consuming the newest available color sample from a latest-one handoff,
  overwriting stale unread samples and reporting the overwrite count, then publishing color image
  plus CameraInfo with at most one frame of queue latency.
- Produces `DepthWorker(period_s=0.1)` publishing raw Depth plus CameraInfo and producing aligned Depth.
- Produces `ImuWorker(max_rate_hz=100)` per motion stream using bounded-ring sequence cursors.
- `Gateway.start()` owns workers after source/ROS startup and before optional SRT startup; cleanup stops workers before source and ROS.

- [ ] **Step 1: Write RED cadence and isolation tests**

```python
def test_slow_depth_never_reduces_color_publish_count():
    source = FakeAsyncSource(color_hz=30, depth_hz=30)
    depth = BlockingDepthPublisher(block_s=0.2)
    run_workers(source, color=CountingPublisher(), depth=depth, seconds=1.0)
    assert color.count >= 29
    assert depth.count <= 10

def test_imu_publish_rate_is_bounded_without_unbounded_backlog():
    result = run_imu_worker(input_hz=200, output_hz=100, seconds=1.0)
    assert 95 <= result.published <= 100
    assert result.buffered <= 32
```

- [ ] **Step 2: Run tests and verify RED**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_gateway_workers.py`

Expected: FAIL because independent cadence workers do not exist.

- [ ] **Step 3: Implement worker lifecycle and per-stream cursors**

Each worker uses `Event.wait(timeout)` for interruptible cadence, owns no SDK pipeline, catches its own recoverable stale-input condition, and reports fatal ROS exceptions to `Gateway.ros_fatal`. No worker holds the Gateway lifecycle lock while publishing or aligning.

- [ ] **Step 4: Move alignment out of the capture path**

Depth worker consumes the latest real SDK composite frameset, runs `rs.align(color)` no faster than
10 Hz and only in Depth/overlay SRT modes, publishes raw Depth from its independent stream slot at a deadline-based 10 Hz, and stores an immutable aligned
array with `created_ns`. It discards results when the capture generation or token changed during
alignment; it never synthesizes a composite from independent child frames.

- [ ] **Step 5: Split ROS publisher methods**

Expose `publish_color(sample, mapper)`, `publish_depth(sample, mapper)`, and `publish_imu(stream, sample, mapper)` while retaining one shared rclpy node and executor. Each method returns the exact topic names published so diagnostics counts remain authoritative.

- [ ] **Step 6: Integrate worker ownership into Gateway**

Remove frame draining from `Gateway.run_once`; it now observes component health and commands only. Startup order becomes guard → control server → SDK source → ROS → workers → optional SRT. Shutdown order becomes SRT → workers → SDK → ROS → server → guard.

- [ ] **Step 7: Run full Gateway/ROS tests**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_gateway_workers.py l515_dashboard/tests/test_gateway.py l515_dashboard/tests/test_gateway_ros.py`

Expected: all PASS, color count unaffected by blocked Depth, and repeated stop leaves zero worker threads.

- [ ] **Step 8: Commit worker separation**

```bash
git add l515_dashboard/gateway.py l515_dashboard/gateway_ros.py l515_dashboard/gateway_workers.py l515_dashboard/tests/test_gateway_workers.py l515_dashboard/tests/test_gateway.py l515_dashboard/tests/test_gateway_ros.py
git commit -m "perf(l515): isolate ROS and alignment cadences"
```

---

### Task 4: RGB-paced x264 streamer with fresh-depth reuse

**Files:**
- Modify: `l515_dashboard/streamer.py`
- Modify: `l515_dashboard/frame_modes.py`
- Modify: `l515_dashboard/config.py`
- Modify: `motor_control/vision/gst_stream.py`
- Modify: `l515_dashboard/tests/test_streamer.py`
- Modify: `l515_dashboard/tests/test_frame_modes.py`
- Modify: `l515_dashboard/tests/test_config.py`

**Interfaces:**
- `SrtStreamer.submit_color(frame, timestamp_ns)` is the only operation that schedules an encoded output frame.
- `SrtStreamer.submit_aligned_depth(frame, timestamp_ns)` updates reusable overlay state without scheduling output.
- `LatestVideoFrames.take(mode, now_ns, max_depth_age_ns) -> np.ndarray | None` returns None for stale Depth/overlay.
- Snapshot adds `input_color`, `sent`, `dropped`, `effective_fps`, `depth_age_ms`, and `pipeline_command`.

- [ ] **Step 1: Write RED RGB-paced and stale-depth tests**

```python
def test_overlay_outputs_each_color_using_fresh_reusable_depth():
    streamer.submit_aligned_depth(depth, timestamp_ns=0)
    for index in range(3):
        streamer.submit_color(color(index), timestamp_ns=index * 33_333_333)
    assert child.frames_written == 3
    assert child.pid == original_pid

def test_stale_depth_never_replays_overlay():
    frames.put_depth(depth, timestamp_ns=0)
    frames.put_color(color, timestamp_ns=300_000_000)
    assert frames.take(FrameMode.OVERLAY, 300_000_000, 250_000_000) is None
```

- [ ] **Step 2: Run focused tests and verify RED**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_streamer.py l515_dashboard/tests/test_frame_modes.py`

Expected: FAIL because Depth currently gates overlay output and freshness is not tracked.

- [ ] **Step 3: Make color the sole output clock**

Depth updates replace a latest immutable aligned frame. Every new color frame schedules exactly one RGB, Depth, or overlay canvas depending on mode. A pending unsent RGB frame may be overwritten once and increments `dropped`; queues never grow.

- [ ] **Step 4: Apply the Task-1 selected software pipeline**

Keep `x264enc tune=zerolatency speed-preset=ultrafast threads=3 bitrate=3000 key-int-max=30`, the real-frame HIL-selected conversion path, MPEG-TS, and SRT listener settings. Reject `encoder != "x264"` in production Gateway config; remove automatic openh264 fallback from this Gateway path without changing unrelated legacy callers. The earlier all-zero benchmark overstated `superfast`; final selection must use real L515 image content.

- [ ] **Step 5: Preserve crash isolation and exact restart behavior**

Broken pipe or nonzero child exit marks only SRT DEGRADED. `set_streaming` restart creates one new child under the same Gateway/source/ROS PIDs. Stop closes stdin, TERM/KILL bounds remain, and no writer thread survives.

- [ ] **Step 6: Run streamer and lifecycle regressions**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_streamer.py l515_dashboard/tests/test_frame_modes.py l515_dashboard/tests/test_config.py l515_dashboard/tests/test_gateway.py`

Expected: all PASS, three-mode output is RGB-paced, stale aligned Depth returns None, and child identity is stable across mode changes.

- [ ] **Step 7: Commit RGB-paced streaming**

```bash
git add l515_dashboard/streamer.py l515_dashboard/frame_modes.py l515_dashboard/config.py motor_control/vision/gst_stream.py l515_dashboard/tests/test_streamer.py l515_dashboard/tests/test_frame_modes.py l515_dashboard/tests/test_config.py
git commit -m "perf(l515): stream RGB cadence through x264"
```

---

### Task 5: Deployment, diagnostics, final HIL, and publication

**Files:**
- Modify: `docker/Dockerfile.ros`
- Modify: `docker/docker-compose.jetson.yml`
- Modify: `l515_dashboard/README.md`
- Modify: `README.md`
- Modify: `ros2/README.md`
- Modify: `AGENTS.md`
- Create: `docs/reports/2026-07-12-l515-gateway-performance-hil.md`
- Modify: `docs/plans/2026-07-02-autonomous-driving-kickoff.md`

**Interfaces:**
- Exact image contains `x264enc`, the Task-1 selected conversion element, pyrealsense2 2.50.0, and the existing entrypoint.
- Status exposes native callback rates, ROS per-topic rates, SRT submitted/sent/drop rates, aligned-depth age, and process CPU/RSS.

- [ ] **Step 1: Write RED deployment and status-contract tests**

Assert Dockerfile contains the selected GStreamer packages, Compose retains host networking/shared lock/runtime provisioning, status contains every new rate/age counter, and no deployment/config text references `nvv4l2h264enc` as available.

- [ ] **Step 2: Run focused tests and verify RED**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_entrypoint.py l515_dashboard/tests/test_gateway.py l515_dashboard/tests/test_diagnostics.py`

Expected: FAIL until new diagnostics fields and deployment contract are implemented.

- [ ] **Step 3: Implement deployment and status changes**

Keep the root-owned `/run/powertrain` tmpfiles contract and abstract socket unchanged. Add only packages proven by Task 1, expose no nonexistent NVENC device, and document that Orin Nano software x264 is intentional.

- [ ] **Step 4: Run all local and isolated ROS tests**

Run: `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests`

Run the isolated ROS three-package build/test and require powertrain_ros 91/91 or the new exact higher count with zero failures.

- [ ] **Step 5: Exact-archive deploy to a unique Jetson snapshot/image**

Record feature commit, image ID, SDK version, GStreamer plugin versions, model string, checkout/container before-state, and runtime-directory state. Do not retag or replace production images/containers.

- [ ] **Step 6: Run connected 60 s performance acceptance**

With RGB SRT enabled and alignment suppressed, record unique frame numbers and every complete 5 s window. Require ROS color image and CameraInfo ≥29.0 Hz, RGB SRT receiver ≥29.0 fps, raw Depth 9.5–10.5 Hz, accel/gyro ≤100.5 Hz with bounded queues, no SDK internal frame gap, and no process overrun/backlog growth. Measure Depth/overlay SRT separately as best-effort functional modes.

- [ ] **Step 7: Complete functional and fault HIL**

Verify singleton rejection before SDK, three modes with stable Gateway/GStreamer PIDs, Dashboard SIGHUP independence, GStreamer SIGKILL isolation and restart, and exact status counters. Then ask the user once for L515 unplug/replug; verify DEGRADED, stale replay 0, exact-serial recovery, and no D435 fallback. Run D435 perception concurrently and require its topic continuity plus USB error delta 0.

- [ ] **Step 8: Restore external state and prove cleanup**

Stop every test Gateway, receiver, benchmark, and GStreamer process. Require no SDK owner, no abstract listener, no held flock, persistent lock file retained, and exact pre-test production container/checkout state restored.

- [ ] **Step 9: Update project documents and Notion**

Write measured rates, CPU/RSS, selected conversion, known limits, HIL transitions, exact commit/image, and cleanup evidence. Fetch the active Software Notion page before writing and re-fetch after writing.

- [ ] **Step 10: Final review, commit, and push**

Run full clean software/ROS suites and two-stage spec/quality review. Fix every Critical or Important finding, commit the report/docs, push `feat/l515-lightweight-pipeline`, and record the pushed commit.
