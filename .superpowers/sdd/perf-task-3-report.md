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
