# Task 4 Report

## Outcome

Implemented `SrtStreamer` as a condition-driven latest-frame worker around the
existing `build_gst_command` contract. It exposes start, runtime mode selection,
color/depth submission, idempotent stop, and immutable snapshots.

## TDD evidence

- Initial RED: focused collection failed because `l515_dashboard.streamer` did
  not exist.
- First GREEN cycle exposed and fixed the stop-thread identity bug; the focused
  streamer plus frame-mode suite then passed 23 tests.
- A second RED removed the frame submission from the child-exit test. It failed
  because the worker only polled GStreamer on input.
- GREEN added condition timeout polling, so an idle unexpected child exit now
  records `GStreamer exited with code N` and stops the worker.

The fake-Popen tests prove exact x264/SRT argv for 640-wide Color/Depth and
1280-wide side-by-side starts, one in-flight write, overwrite instead of queue
or replay, BrokenPipe handling, child-exit handling, and exactly-once close/wait
across repeated stop calls.

## Task 3 minor coverage

Added the requested reviewer gaps: explicit side-by-side left/right pixel
content and newest-depth overwrite/consume behavior.

## Verification

Fresh verification before commit:

```text
python -m pytest -q l515_dashboard/tests
71 passed in 0.32s

python -m flake8 l515_dashboard/streamer.py \
  l515_dashboard/tests/test_streamer.py \
  l515_dashboard/tests/test_frame_modes.py
exit 0

git diff --check
exit 0
```

`motor_control/vision/gst_stream.py` was not modified, preserving its proven
x264/openh264 encoder and SRT command contract. No supervisor, ROS bridge, or
UI work was added.
