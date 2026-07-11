# Task 4 — RGB-paced x264 streamer report

## Scope

Implemented only the Task 4 streaming contract on baseline
`52b7fa699c569c51156be816b7c6d1a25dbfd7b2` without hardware/HIL:

- RGB is the sole encoded-output clock for RGB, Depth, and overlay modes.
- Latest aligned Depth is copied into reusable timestamped state and is rejected
  after 250 ms for Depth/overlay rendering.
- One pending RGB slot uses latest-one overwrite/drop accounting; no frame queue
  grows.
- Mode changes retain the same GStreamer child and do not restart SDK, ROS, or
  Gateway components.
- Gateway configuration accepts x264 only, while the shared legacy GStreamer
  builder still retains its unrelated openh264 caller path.
- Selected software pipeline is `videoconvert` → x264 superfast/zerolatency,
  `threads=2`, 3000 kbps, GOP 30 → MPEG-TS → SRT listener.
- Snapshot/status expose input RGB count, sent/drop counters, effective FPS,
  Depth age, and the exact pipeline command.

The Gateway adapter was necessarily updated so Task 3 worker samples invoke the
new timestamped Task 4 interfaces. Color arrival and aligned-Depth creation use
the same monotonic clock domain.

## RED

Command:

```text
/home/light/anaconda3/bin/python -m pytest -q \
  l515_dashboard/tests/test_streamer.py \
  l515_dashboard/tests/test_frame_modes.py \
  l515_dashboard/tests/test_config.py
```

Observed before production changes: `24 failed, 91 passed`. Failures were the
expected missing `submit_aligned_depth`, timestamp arguments, freshness-aware
`take`, snapshot fields, `threads=2`, and x264-only configuration behavior.

## GREEN and regression evidence

Focused Task 4 plus Gateway lifecycle/x264 regression:

```text
/home/light/anaconda3/bin/python -m pytest -q \
  l515_dashboard/tests/test_streamer.py \
  l515_dashboard/tests/test_frame_modes.py \
  l515_dashboard/tests/test_config.py \
  l515_dashboard/tests/test_gateway.py \
  l515_dashboard/tests/test_x264_benchmark.py
```

Observed: `143 passed in 0.92s`.

Full L515 dashboard suite:

```text
/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests
```

Observed: `233 passed in 7.99s` before the final additional Depth-copy regression;
the final verification command and count are recorded in the commit handoff.

Repository-wide unscoped `pytest -q` was attempted and stopped during collection
with 23 environment errors because the conda host lacks hardware/container/ROS
dependencies such as `python-can`, `odrive`, `pyrealsense2`, `rclpy`, and ROS
message packages. This is an environment limitation, not a test failure in the
Task 4/L515 suite.

## Self-review

- Confirmed Depth submission never notifies the writer.
- Confirmed Depth and overlay `take` require a newly consumed RGB sample and a
  fresh reusable Depth sample.
- Confirmed mode switching does not touch the child process or pending SDK/ROS
  lifecycle.
- Confirmed broken pipe/nonzero exit remain isolated to streamer health, and
  existing stop/restart/TERM/KILL/concurrent-cleanup tests remain covered.
- Confirmed `git diff --check` is clean.

## Remaining concern

No hardware/HIL was authorized. Actual Orin x264 throughput and SRT receiver
behavior remain deployment/HIL verification items; software behavior is covered
by fake-child and lifecycle tests.

## Review remediation — short writes and writer termination

Review found two important lifecycle gaps after `82dae42`.

### RED

Added regressions for short-count writes, zero/`None`/negative/oversized/non-int
counts, arbitrary writer exceptions, cooperative close-unblock, an uncooperative
writer, and replacement suppression while an old writer is not proven dead.

Focused streamer RED observed `8 failed, 16 passed`: short writes mixed the next
frame after an incomplete first frame, invalid counts were treated as successful,
and `stop()` returned with a live writer. The isolated Gateway replacement RED
observed `1 failed, 21 deselected`: a new streamer was started after old-writer
cleanup raised.

### GREEN

- `_write_all` now owns one `memoryview` until every byte is accepted. It checks
  every count, aborts on stop/generation cancellation, and never increments
  `sent` for incomplete or failed frames.
- All writer exceptions are converted into streamer-local failure state.
- `stop()` closes stdin/reaps the child, then performs a bounded second join. A
  live writer raises an explicit timeout, records the cleanup error, leaves
  `_cleanup_done` false, and permits a later cleanup retry.
- Concurrent cleanup callers wait for the current attempt, then retry only when
  the prior attempt did not complete.
- Gateway replacement returns immediately when old-streamer cleanup fails, so
  no new writer can overlap it.

Focused streamer plus Gateway result: `46 passed in 0.68s` before the additional
generic-exception regression. Final full L515 verification result: `243 passed
in 7.96s`. `git diff --check` was clean.

## Second review remediation — Gateway retry boundaries

Second review identified that the streamer itself preserved retryable cleanup
state, but two Gateway callers erased that guarantee.

### RED

Two real Gateway lifecycle regressions were added with an uncooperative old
writer:

- `restart_components()` had to retain original ownership, avoid stopping
  workers/source/ROS, avoid calling the factory, and remain retryable.
- `shutdown()` had to leave all downstream dependencies running and
  `_shutdown_done=False` until a later retry could prove the writer dead.

Focused RED observed `2 failed, 22 deselected`: restart replaced the old
streamer, and shutdown falsely committed `_shutdown_done=True`.

### GREEN

- Added the exported `StreamerStopTimeout` lifecycle contract and made
  `SrtStreamer.stop()` raise it when the bounded post-reap join cannot prove
  writer termination.
- Gateway common cleanup treats `StreamerStopTimeout` like
  `WorkerStopTimeout`: it stops the cleanup plan immediately, retains ownership,
  leaves `_shutdown_done=False`, and permits a later shutdown retry.
- Component restart now aborts on any streamer stop failure, keeps the old
  streamer owned, starts no replacement or downstream component, exposes
  DEGRADED state, and re-enables commands so the operation can be retried.
- The success retry proves the old writer is no longer alive before workers,
  source, ROS, or a replacement streamer are restarted.

Focused streamer/Gateway result: `49 passed in 0.73s`. Full L515 result before
the final commit gate: `245 passed in 8.17s`; `git diff --check` was clean.
