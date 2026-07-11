# Task 3 — Independent ROS cadence workers

## Scope and result

Implemented independent Color, Depth/alignment, gyro, and accel workers. Gateway now owns their
lifecycle between ROS and optional SRT startup, and stops them before the SDK source and ROS.
`Gateway.run_once()` only observes health. No hardware or HIL was used.

## RED

Command:

```text
/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_gateway_workers.py
```

Observed before production implementation:

```text
ModuleNotFoundError: No module named 'l515_dashboard.gateway_workers'
1 error in 0.15s
```

The split ROS API was also observed RED:

```text
/home/light/anaconda3/bin/python -m pytest -q \
  l515_dashboard/tests/test_gateway_ros.py::test_split_publish_methods_preserve_exact_topic_contract
AttributeError: 'GatewayRosPublisher' object has no attribute 'publish_color'
1 failed in 0.14s
```

## GREEN and verification

Focused command:

```text
/home/light/anaconda3/bin/python -m pytest -q \
  l515_dashboard/tests/test_gateway_workers.py \
  l515_dashboard/tests/test_gateway.py \
  l515_dashboard/tests/test_gateway_ros.py
26 passed in 2.37s
```

Full dashboard command:

```text
/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests
209 passed in 7.16s
```

`git diff --check` also exited successfully with no output.

## Interfaces

- `ColorWorker`: latest-slot cursor, publishes color Image and CameraInfo at input cadence.
- `DepthWorker(period_s=0.1)`: publishes native raw Depth and CameraInfo, aligns the latest
  color/depth pair at the bounded cadence, generation-checks both inputs after alignment, and
  exposes a read-only `AlignedDepth(array, created_ns)` snapshot.
- `ImuWorker(max_rate_hz=100)`: independent accel/gyro bounded-ring cursors, publishes the latest
  unread sample per cadence tick so backlog remains bounded.
- `WorkerGroup`: repeatable start/stop ownership; worker waits use `Event.wait(timeout)`.
- `GatewayRosPublisher`: `publish_color(sample, mapper)`, `publish_depth(sample, mapper)`, and
  `publish_imu(stream, sample, mapper)` return the exact published topic tuple. The compatibility
  `publish(frames)` interface remains available.

The approved six ROS topic names and existing command/status lifecycle remain unchanged.

## Lifecycle and lock reasoning

Startup is guard → server → source → ROS → workers → optional SRT. Cleanup is optional SRT →
workers → source → ROS → server → guard. Restart uses the same workers-before-source teardown.

Worker threads call ROS publication, SDK alignment, and SRT submission without acquiring the
Gateway lifecycle lock. Only the short post-publication diagnostics/count update acquires it.
Fatal ROS/publication exceptions escape the individual worker guard and call `Gateway.ros_fatal`;
worker stop avoids joining its own fatal-reporting thread, allowing common cleanup to finish.

## Self-review / limitations

- No RealSense hardware/HIL was run, as required. The production default constructs one
  `rs.align(color)` processor and a composite from the retained latest frames; fake alignment tests
  cover cadence, invalidation during work, and immutable output, but the pinned pyrealsense2 2.50.0
  composite-construction binding remains an explicit HIL verification point.
- Depth work may exceed its nominal period when alignment or ROS Depth publication blocks; it does
  not catch up in bursts and cannot reduce Color or IMU cadence because each has its own thread.

## Reviewer remediation wave

The user resolved the cadence wording in favor of the proven D435i pattern in
`motor_control/vision/yolo_depth_3d.py`: video is latest-one, stale unread video is overwritten, and
queue latency is bounded to at most one frame. It is not an every-frame preservation contract.

### Additional RED evidence

The combined reviewer tests were first run with:

```text
/home/light/anaconda3/bin/python -m pytest -q \
  l515_dashboard/tests/test_gateway_source.py \
  l515_dashboard/tests/test_gateway_workers.py \
  l515_dashboard/tests/test_gateway.py
```

Collection failed because the fail-closed worker exception and real composite bundle interfaces did
not exist:

```text
ImportError: cannot import name 'WorkerStopTimeout' from
'l515_dashboard.gateway_workers'
2 errors in 0.29s
```

After the first implementation pass, the isolation test exposed that the mapper lock had been placed
around the entire ROS publish call: blocked Depth reduced Color to 18 publishes instead of at least
29. Root cause was lock scope, not worker scheduling. The dedicated lock was moved down to only the
shared `TimestampMapper.map_ms` call inside `GatewayRosPublisher`; ROS publication remains independent.

The first full-suite run then exposed a timing-test issue: the 10 ms startup readiness observation
window was included in a 120 ms wall-clock call-count assertion, allowing five correctly spaced
calls instead of four. The test now verifies the actual no-burst property: consecutive calls remain
at least 25 ms apart for a 20 ms overrun plus the positive cadence wait.

### Remediated design

- Task 2 now retains the actual SDK composite callback frame in a dedicated `LatestSlot` as immutable
  `VideoBundle(generation, capture_token, frameset, received_ns)`. `keep()` is called before handoff.
  Independent color/depth slots remain the raw ROS inputs.
- `LatestSlot.overwrites`, `source.color_overwrites`, and `source.video_bundle_overwrites` expose the
  intended stale-frame drop behavior. Gateway status reports both counters.
- `DepthWorker` passes only the retained real composite frameset to one `rs.align(color)` processor.
  Unsupported `rs.composite_frame(...)` synthesis was removed. Alignment output is accepted only if
  exact `(generation, capture_token)` identity matches before and after processing.
- `GatewayRosPublisher` serializes the shared `TimestampMapper` invocation with a dedicated lock;
  image conversion and ROS publication are outside that lock. A four-thread color/depth/gyro/accel
  test proves mapper calls never overlap.
- Every cadence worker waits one positive period from completion. An overrun therefore cannot cause
  an immediate catch-up burst.
- Worker startup has ready/error events plus a bounded immediate-error observation window. Missing
  publisher contracts or immediate reader failure make `start()` fail before optional SRT startup.
- `WorkerGroup.stop()` raises `WorkerStopTimeout` if any thread remains alive after bounded joins.
  Gateway cleanup stops at that boundary, keeps ownership retryable, and does not stop SDK/ROS until
  a later successful retry.
- The tracked performance plan and Task 3 brief now state latest-one overwrite semantics and real
  composite alignment explicitly.

Final fresh verification:

```text
/home/light/anaconda3/bin/python -m pytest -q \
  l515_dashboard/tests/test_gateway_source.py \
  l515_dashboard/tests/test_gateway_workers.py \
  l515_dashboard/tests/test_gateway.py \
  l515_dashboard/tests/test_gateway_ros.py
47 passed in 3.14s

/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests
217 passed in 7.70s

git diff --check
# no output, exit 0
```

## Second re-review remediation

The overwrite counter originally incremented whenever a slot already contained any sample. Because
`read_after()` did not mark that sample consumed, a normal publish → read → publish sequence was
incorrectly reported as a drop. `LatestSlot` now tracks an explicit unread bit: publish increments
`overwrites` only when replacing an unread value; a successful read and `clear()` clear the bit.
Sequence/cursor behavior remains unchanged.

Startup timeout also originally raised directly from `_Worker.start()` without stopping its newly
created thread. `_Worker.start()` now owns rollback: readiness timeout or immediate startup error
sets stop, performs the bounded join, and only then raises. `WorkerGroup` registers each worker in
its rollback list before calling `start()` as a second ownership safeguard. If a worker cannot join,
`WorkerStopTimeout` still preserves the existing fail-closed Gateway dependency boundary.

RED command and evidence:

```text
/home/light/anaconda3/bin/python -m pytest -q \
  l515_dashboard/tests/test_stream_buffer.py \
  l515_dashboard/tests/test_gateway_source.py \
  l515_dashboard/tests/test_gateway_workers.py \
  l515_dashboard/tests/test_gateway.py

3 failed, 47 passed in 3.22s
- consumed LatestSlot still reported overwrites == 1
- source/Gateway-facing counters inherited the same false overwrite
- non-ready worker remained alive after startup timeout
```

GREEN focused evidence:

```text
/home/light/anaconda3/bin/python -m pytest -q \
  l515_dashboard/tests/test_stream_buffer.py \
  l515_dashboard/tests/test_gateway_source.py \
  l515_dashboard/tests/test_gateway_workers.py \
  l515_dashboard/tests/test_gateway.py \
  l515_dashboard/tests/test_gateway_ros.py
55 passed in 3.14s
```

Tests now cover publish/read/publish = zero overwrites, publish/publish-before-read = exactly one,
exact source and Gateway status counters, a deliberately non-ready live worker leaving no thread,
and the existing blocked-worker fail-closed SDK/ROS cleanup plus release/retry ordering. Unused
worker-side mapper-lock constructor plumbing was removed; mapper serialization remains owned and
tested by `GatewayRosPublisher`.

Final fresh re-review verification:

```text
/home/light/anaconda3/bin/python -m pytest -q \
  l515_dashboard/tests/test_gateway_source.py \
  l515_dashboard/tests/test_gateway_workers.py \
  l515_dashboard/tests/test_gateway.py \
  l515_dashboard/tests/test_gateway_ros.py
49 passed in 3.09s

/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests
220 passed in 7.86s

git diff --check
# no output, exit 0
```
