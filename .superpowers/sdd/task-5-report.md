# Task 5 Report — USB3/SDK Preflight and Operations Documentation

## Status

Complete. No Jetson access or modification was performed.

## Implementation

- Added `scripts/l515_preflight.sh`, which fails closed unless exactly one USB PID
  `8086:0b64` is present, its matching sysfs link is at least 5000 Mbps, and pyrealsense2 in
  `powertrain_ros` selects exactly serial `00000000F0271544`.
- Added deterministic subprocess tests with fake `lsusb`, `docker`, and sysfs for success,
  missing USB, 480 Mbps, missing SDK serial, and wrong SDK serial.
- Updated root README, ROS README, and the newest AGENTS override with the exact v2.50.0 SDK pin,
  L515/D435i ownership, build/source/launch commands, preflight command, topics, and explicit
  PointCloud2 absence.

## TDD Evidence

- RED: `python -m pytest -q .../test_l515_preflight.py` produced 5 failures because the preflight
  script did not exist.
- GREEN: the same test file passed 5/5 after implementation.

## Verification

- `bash -n scripts/l515_preflight.sh`: pass.
- `python -m flake8 ros2/src/powertrain_ros/test/test_l515_preflight.py`: pass.
- Isolated ROS image clean build of `robot_arm_msgs`, `powertrain_msgs`, and `powertrain_ros`: pass.
- Full clean `colcon test` result: 76 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: pass.

## Self-review

- The script maps the exact lsusb bus/device pair to sysfs rather than accepting an unrelated
  SuperSpeed device.
- Missing tools, malformed/missing sysfs speed, Docker/SDK errors, duplicate PID devices, absent
  serial, and wrong serial all reject launch.
- D435i is neither selected nor opened; its separate ownership remains explicit.

## Concerns

- Hardware behavior remains intentionally unverified until the separately authorized Jetson HIL
  task. The checks here are deterministic software tests only.

## Review-finding fix — 2026-07-11

- Root cause: the SDK selection was embedded in a shell heredoc, while subprocess tests replaced
  Docker and supplied canned stdout. The production selection logic therefore had no direct test.
- RED: expanded `test_l515_preflight.py` failed 9 tests because
  `scripts/l515_sdk_probe.py` did not exist and the shell still invoked stdin Python.
- GREEN: added `l515_sdk_probe.py` with injectable `select_exact_serial(context, serial_info,
  expected_serial)` and changed preflight to invoke it as
  `docker exec -i powertrain_ros python3 /workspace/scripts/l515_sdk_probe.py --serial ...`.
- The tests execute the real selection function with fake RealSense-like contexts: one exact L515
  succeeds while D435i coexists; empty, wrong, missing, and duplicate expected serial cases fail.
  Shell coverage validates the complete Docker argv/helper path and explicit nonzero Docker/SDK
  failure, while retaining the fail-closed USB PID/sysfs checks.
- AGENTS now explicitly supersedes every historical L515 `realsense-ros`/optional PointCloud2
  instruction. Current operation is the custom pyrealsense2 node and PointCloud2 is absent.

### Fresh verification

- `/home/light/anaconda3/bin/python -m pytest -q
  ros2/src/powertrain_ros/test/test_l515_preflight.py`: 12 passed.
- `bash -n scripts/l515_preflight.sh`: pass.
- `/home/light/anaconda3/bin/python -m flake8 scripts/l515_sdk_probe.py
  ros2/src/powertrain_ros/test/test_l515_preflight.py`: pass.
- Clean isolated ROS build/test in `powertrain-sw:ros-task7-minors`: 3 packages built; 83 tests,
  0 errors, 0 failures, 0 skipped.
- `git diff --check`: pass.
