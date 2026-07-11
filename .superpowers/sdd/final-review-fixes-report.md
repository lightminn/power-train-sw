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

## Environment limitation

An isolated ROS test run was attempted. Host `/usr/bin/python3` has no pytest, and
the running `powertrain_dev` container has no `/opt/ros/humble/setup.bash`, so the
ROS pytest/colcon suite could not execute in this workstation environment. The ROS
files received syntax and static contract verification above; the full ROS suite
should be rerun in the Jetson `powertrain_ros` image/CI that contains Humble.
