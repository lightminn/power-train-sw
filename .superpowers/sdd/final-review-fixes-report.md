# L515 Final Broad Review Fixes — RED/GREEN Report

Date: 2026-07-12
Baseline: `9812569d4708e3b4bc7c73485c769d69882b87d5`

## Findings fixed

1. Retired the legacy production ROS camera runtime. `l515_node.main()` and the
   source-tree legacy launch now fail closed before ROS initialization, node
   construction, `L515Source`, or pyrealsense2 construction and direct operators
   to `python3 -m l515_dashboard.gateway_main`. The console entry point and launch
   install exposure were removed. Directly injected `L515Node` adapter tests remain.
2. Added exported `SourceStopTimeout`. `L515GatewaySource.stop()` now raises unless
   both its worker and native pipeline stopper are proven dead. A timed-out native
   stopper is retained and joined on retry; no duplicate stop thread or SDK owner
   can be started. Pipeline/lifecycle ownership remains intact until successful retry.
3. Gateway shutdown and internal component restart treat `SourceStopTimeout` as
   retry-blocking. Source, ROS, server, singleton guard, and remaining ownership are
   retained; `_shutdown_done` stays false; no replacement source is started.
4. Marked the original lightweight pipeline design prominently superseded for
   runtime architecture and pointed operations to the Gateway design/entry point.
   ROS README now states that legacy console/launch production paths are retired.

## RED evidence

- New source timeout test initially failed at collection because
  `SourceStopTimeout` did not exist.
- New shutdown/restart tests showed source timeout was not classified as retry
  blocking; restart propagated into fatal common cleanup.
- New duplicate-owner test initially failed because `start()` accepted a new
  generation while the previous native pipeline stop thread remained alive.
- Updated legacy ROS tests encode that `main()`/launch must fail before construction
  and that setup must not install their production invocation paths.

## GREEN evidence

- Focused source/Gateway timeout tests: `3 passed`.
- Full dashboard suite: `260 passed in 9.68s`.
- Static legacy runtime contract: `PASS` (AST parse, no console/launch install,
  both source-tree fail-closed paths point to Gateway, supersession marker present).
- Python syntax compilation for `l515_node.py` and `l515.launch.py`: pass using a
  writable external pycache.
- `git diff --check`: pass.

## Historical local environment limitation

An isolated ROS test run was attempted. Host `/usr/bin/python3` has no pytest, and
the running `powertrain_dev` container has no `/opt/ros/humble/setup.bash`, so the
ROS pytest/colcon suite could not execute in this workstation environment. The ROS
files received syntax and static contract verification above; the full ROS suite
required the Jetson `powertrain_ros` image containing Humble; that verification is
now completed in the following section.

## Jetson isolated ROS verification (concern resolved)

The prior environment limitation was resolved on the Jetson without modifying or
restarting its production checkout, containers, or image tags.

- Exact source: `git archive df4bd86a789548b6460a733d1dbc82979e8fd2ff`
- Unique read-only snapshot mount:
  `/tmp/l515-final-review-df4bd86-20260712T051002-503612`
- Existing verified image tag:
  `powertrain-sw:ros-l515-perf-e755b7b-20260712t0545`
- Image ID:
  `sha256:eef0a22a1745741b92bf2ad62074fed63ea08e806aa79cada469d7f83d096482`
- Isolation: one-shot `docker run --rm`, `--network none`, no device mounts,
  archive mounted read-only, copied to container-local `/tmp/src` for the build.

Exact successful container command body:

```bash
set -eo pipefail
cp -a /snapshot /tmp/src
cd /tmp/src/ros2
rm -rf build install log
source /opt/ros/humble/setup.bash
export PYTHONPATH=/tmp/src/motor_control:${PYTHONPATH:-}
colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros \
  --event-handlers console_direct+
source install/setup.bash
export PYTHONPATH=/tmp/src/motor_control:${PYTHONPATH:-}
colcon test --packages-select powertrain_ros --event-handlers console_direct+
colcon test-result --verbose
```

Results:

- Clean three-package build: `Summary: 3 packages finished`.
- `powertrain_ros`: `collected 91 items`, `91 passed in 1.64s`.
- `colcon test-result`: `91 tests, 0 errors, 0 failures, 0 skipped`.
- Retirement coverage executed explicitly in the full suite:
  `test_l515_launch_contract.py` 3/3 and `test_l515_node.py` 8/8 passed.

Two pre-result attempts were safely removed by `--rm`: the first stopped before
build because ROS setup rejects shell `set -u`; the second clean build succeeded
but collection exposed the missing archive-local `motor_control` PYTHONPATH. No
source change was required; the successful command supplied the normal repo runtime
dependency path shown above.

Cleanup verification:

- `snapshot_removed=yes`
- `test_containers_removed=yes`
- `test_processes_absent=yes`
- Production container state remained unchanged: `powertrain_ros` and
  `powertrain_jetson` remained running; `powertrain_canwatchdog` and `ros2_humble`
  remained stopped.
- Production checkout status remained unchanged:
  `main...origin/main [ahead 41]` plus pre-existing untracked
  `motor_control/vision/tests/`.
