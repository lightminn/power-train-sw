# Revised Task 5 Report — Gateway ROS and fixed-canvas SRT

## Implementation

- Added `GatewayRosPublisher`, a nonblocking adapter for one drained `GatewayFrames` snapshot.
- It creates and publishes only the approved six topics. Color uses the native 1280x720 profile;
  raw depth and its CameraInfo use 640x480. Aligned depth is not published.
- Video Image and CameraInfo share a mapped device stamp. Equal timestamps are deduplicated per
  stream, while a new mapper generation resets dedup state. Gyro and accel remain separate Imu
  messages with the existing adapter semantics.
- SRT now has one independently tested exact 1280x720 GStreamer argv for RGB, Depth, and overlay.
  Mode changes preserve the same child and each accepted selection produces one frame-sized write.
- Stream cleanup now reaps a child created without stdin, owns wait timeout escalation through
  terminate and kill, makes concurrent stop callers wait for the same idempotent cleanup, and
  prevents an in-flight write from mutating counters after stop begins.
- `powertrain_ros/setup.py` remains unchanged because the old entrypoint cannot be redirected until
  the later Gateway orchestration task provides the runnable Gateway node.

## TDD evidence

- Gateway RED: import failed because `gateway_ros.py` did not exist; the initial implementation
  then passed the color/depth profile, six-topic, IMU, stamp, and dedup tests.
- Reconnect RED: a repeated device timestamp from a new mapper generation was incorrectly dropped;
  mapper-generation reset made it pass.
- Streamer RED: partial-start cleanup, timeout escalation, and post-stop sent-counter tests failed
  against the provisional streamer. Lifecycle ownership changes made all focused tests pass.

## Verification

- Focused Gateway/streamer/source/frame/config: 119 passed.
- Full dashboard suite: 134 passed.
- Clean isolated ROS build and test in `powertrain-sw:ros-task7-minors`: 3 packages built; 91 tests,
  0 errors, 0 failures, 0 skipped.
- Flake8 on all changed Python files: clean.
- Compileall and `git diff --check`: clean.

## Self-review and concerns

- ROS imports are lazy so dashboard-only tests do not require a ROS installation; production
  dependencies are loaded when the publisher is instantiated.
- Subprocess escalation belongs to `SrtStreamer.stop()`. Worker failure records state but does not
  race to reap the child.
- Hardware/SRT receiver behavior is not exercised here; it remains a later Jetson HIL concern.

## Review fix — immutable terminal snapshot and fixed cadence

- RED reproduced a late blocked write raising BrokenPipe after `stop()` returned and changing
  `last_error`; another RED showed `set_mode()` changing the terminal snapshot.
- `stop()` now invalidates the active worker generation under the condition lock. Every worker
  write to `sent`, `last_error`, and `running` requires the same live generation, and mode changes
  become a no-op after terminal stop. The complete snapshot is therefore immutable after stop.
- `DashboardConfig` now rejects every fps value except the approved fixed 30 fps.
- Focused streamer/config verification after the fix: 88 passed.
- Full dashboard verification: 137 passed. Clean isolated ROS regression: 91 tests, 0 errors,
  0 failures, 0 skipped. Changed-file Flake8, compileall, and diff checks are clean.
