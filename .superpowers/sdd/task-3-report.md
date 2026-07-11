# Task 3 Report

## Scope

- Added `powertrain_ros/l515_source.py` only for serial-locked SDK acquisition,
  latest-frame handoff, and reconnect state.
- Added `test/test_l515_source.py` with an injected fake `rs` module, clock,
  wait function, and mapper factory. No hardware or real sleep is used.

## TDD evidence

- RED: `/home/light/anaconda3/bin/python -m pytest -q test/test_l515_source.py`
  failed during collection with
  `ModuleNotFoundError: No module named 'powertrain_ros.l515_source'`.
- GREEN: the focused source suite passed after the minimal implementation.

## Requirement review

- Immutable configuration accepts only L515 serial `00000000F0271544`.
- Device enumeration never falls back to the D435 serial.
- SDK requests color BGR8 and depth Z16 at exactly 640x480x30, plus accel
  and gyro streams.
- `LatestFrames` uses a lock, overwrites one slot per stream, and atomically
  drains/clears without blocking.
- Disconnect clears queued frames, stops the pipeline best-effort, waits the
  configured 2.0 seconds, and retries only the expected serial.
- Each connection creates a new timestamp mapper; tests prove the second
  session payload carries the second mapper and no first-session frame.
- `stop()` signals shutdown, stops the pipeline best-effort, and joins with
  the configured bounded timeout.

## Fresh verification

- `python3 -m flake8 powertrain_ros/l515_source.py test/test_l515_source.py`:
  exit 0.
- `python3 -m pytest -q test/test_l515_source.py` in
  `powertrain-sw:ros-task7-minors`: `8 passed in 0.02s`.
- `colcon test --packages-select powertrain_ros` followed by
  `colcon test-result --verbose`: `52 tests, 0 errors, 0 failures, 0 skipped`.
- `git diff --check`: exit 0.

## Self-review

- Only the Task 3 production module, its tests, and this report are changed.
- SDK import is injected; ROS adapter import is lazy so unit tests remain
  hardware-independent.
- No Jetson, hardware, user file, launch/config, or other task file changed.

## Review-finding fixes

- `stop()` now signals and invalidates the active generation before doing any
  SDK work. SDK `pipeline.stop()` runs best-effort on a daemon helper, so a
  blocked native stop cannot exceed the configured join bound.
- Lifecycle generation checks prevent pipeline creation/start after a stop
  observed during discovery, prevent late frame enqueue/state regression, and
  only publish `STOPPED` after the worker is quiescent.
- A timed-out join leaves the last non-stopped state visible until the worker
  actually exits. `LatestFrames.empty` now reads its slots under the same lock
  used by put/drain/clear.
- Added deterministic regressions for blocked SDK stop, join timeout, late
  worker completion/no late payload, and the discovery-vs-stop race.

## Review-fix TDD and verification evidence

- RED command: `/home/light/anaconda3/bin/python -m pytest -q
  test/test_l515_source.py`. Result on the pre-fix implementation: seven tests
  reached completion, the join-timeout assertion failed, and execution then
  blocked in the new blocking-SDK-stop regression until terminated.
- Focused command: `/home/light/anaconda3/bin/python -m pytest -q
  test/test_l515_source.py`. Result: `11 passed in 0.05s`.
- Style command: `/home/light/anaconda3/bin/python -m flake8
  powertrain_ros/l515_source.py test/test_l515_source.py`. Result: exit 0.
- Full ROS command: `docker run --rm -v "$PWD:/workspace" -w /workspace/ros2
  powertrain-sw:ros-task7-minors bash -lc 'source
  /opt/ros/humble/setup.bash && colcon --log-base /tmp/l515-log build
  --build-base /tmp/l515-build --install-base /tmp/l515-install
  --packages-select powertrain_msgs robot_arm_msgs powertrain_ros && source
  /tmp/l515-install/setup.bash && colcon --log-base /tmp/l515-test-log test
  --build-base /tmp/l515-build --install-base /tmp/l515-install
  --packages-select powertrain_ros --event-handlers console_direct+ && colcon
  test-result --test-result-base /tmp/l515-build/powertrain_ros --verbose'`.
  Result: three packages built; `55 passed in 0.36s`; `55 tests, 0 errors, 0
  failures, 0 skipped`.
- Diff command: `git diff --check`. Result: exit 0.

- Host-only `/home/light/anaconda3/bin/python -m pytest -q test` was also
  attempted, but collection lacked ROS `launch.action` and
  `builtin_interfaces`; the isolated ROS image command above is the relevant
  full-suite result.

## Second review-fix wave

- Public `start()` and `stop()` are serialized separately from the short
  lifecycle critical sections. A concurrent start cannot clear the stop event
  or replace its generation until the bounded stop operation returns.
- Stop-event set, generation invalidation, and thread/pipeline snapshot are
  atomic under the lifecycle lock. No native SDK call occurs while that lock
  is held.
- Pipeline creation is followed by locked registration of an explicit starting
  generation. The worker revalidates after the pre-start barrier and again
  after native `pipeline.start()`; an invalidated start is cleaned up without
  publishing `STREAMING`.
- State changes, frame commits, and reconnect clears revalidate and mutate
  under the lifecycle lock, so stop invalidation cannot interleave between a
  successful generation check and its commit.
- Added deterministic barriers after frame preparation/before queue commit,
  immediately before native pipeline start, and across concurrent public
  stop/start.

## Second-wave TDD and verification evidence

- RED command: `/home/light/anaconda3/bin/python -m pytest -q
  test/test_l515_source.py`. Result before the second-wave implementation:
  `3 failed, 11 passed`; failures were the frame-commit barrier, pre-native-start
  barrier, and concurrent public start/stop serialization tests.
- Focused command: `/home/light/anaconda3/bin/python -m pytest -q
  test/test_l515_source.py`. Result: `14 passed in 0.08s`.
- Style command: `/home/light/anaconda3/bin/python -m flake8
  powertrain_ros/l515_source.py test/test_l515_source.py`. Result: exit 0.
- Full ROS command: `docker run --rm -v "$PWD:/workspace" -w /workspace/ros2
  powertrain-sw:ros-task7-minors bash -lc 'source
  /opt/ros/humble/setup.bash && colcon --log-base /tmp/l515-log build
  --build-base /tmp/l515-build --install-base /tmp/l515-install
  --packages-select powertrain_msgs robot_arm_msgs powertrain_ros && source
  /tmp/l515-install/setup.bash && colcon --log-base /tmp/l515-test-log test
  --build-base /tmp/l515-build --install-base /tmp/l515-install
  --packages-select powertrain_ros --event-handlers console_direct+ && colcon
  test-result --test-result-base /tmp/l515-build/powertrain_ros --verbose'`.
  Result: three packages built; `58 passed in 0.36s`; `58 tests, 0 errors, 0
  failures, 0 skipped`.
- Diff command: `git diff --check`. Result: exit 0.

## Final STARTING-handshake fix

- Replaced the scalar starting generation with a registered STARTING record
  containing generation, pipeline identity, and `cancel_requested`.
- `stop()` marks an active STARTING record cancelled in the same lifecycle
  critical section that invalidates the generation and snapshots the worker
  and pipeline.
- Added a deterministic barrier after successful locked pre-start validation
  and lock release. If stop lands there, a late native `pipeline.start()` may
  return, but the worker performs best-effort stop before clearing STARTING and
  exits without publishing `STREAMING` or committing frames.
- No native SDK start/stop/wait call is made while the lifecycle lock is held;
  the public stop path remains bounded by `stop_timeout`.

## Final-fix TDD and verification evidence

- RED command: `/home/light/anaconda3/bin/python -m pytest -q
  test/test_l515_source.py`. Result before the handshake implementation:
  `1 failed, 14 passed`; the post-validation/pre-native-start barrier was not
  reached because no such handshake boundary existed.
- Focused command: `/home/light/anaconda3/bin/python -m pytest -q
  test/test_l515_source.py`. Result: `15 passed in 0.08s`.
- Style command: `/home/light/anaconda3/bin/python -m flake8
  powertrain_ros/l515_source.py test/test_l515_source.py`. Result: exit 0.
- Full ROS command: `docker run --rm -v "$PWD:/workspace" -w /workspace/ros2
  powertrain-sw:ros-task7-minors bash -lc 'source
  /opt/ros/humble/setup.bash && colcon --log-base /tmp/l515-log build
  --build-base /tmp/l515-build --install-base /tmp/l515-install
  --packages-select powertrain_msgs robot_arm_msgs powertrain_ros && source
  /tmp/l515-install/setup.bash && colcon --log-base /tmp/l515-test-log test
  --build-base /tmp/l515-build --install-base /tmp/l515-install
  --packages-select powertrain_ros --event-handlers console_direct+ && colcon
  test-result --test-result-base /tmp/l515-build/powertrain_ros --verbose'`.
  Result: three packages built; `59 passed in 0.40s`; `59 tests, 0 errors, 0
  failures, 0 skipped`.
- Diff command: `git diff --check`. Result: exit 0.
