# Task 4 Report

## Status

Implemented the thin `rclpy` L515 publisher node, exact config and launch
contract, package installation/dependency wiring, and fake-source tests.

## TDD evidence

- RED: focused tests reported 5 failures for missing node/config/launch and
  setup wiring.
- GREEN: `test_l515_node.py` and `test_l515_launch_contract.py`: 5 passed.
- The timer calls only `poll_latest()`, runs at 200 Hz, and all six publishers
  use `qos_profile_sensor_data`.
- Image and CameraInfo pairs share the mapped device timestamp; all four frame
  IDs match the design contract; `destroy_node()` stops the source.

## Clean ROS verification

Ran in a fresh container-local workspace with only the three ROS packages
selected, plus read-only project dependencies required by existing tests:

```text
Summary: 3 packages finished [9.37s]
Summary: 3 packages finished [1.56s]
Summary: 65 tests, 0 errors, 0 failures, 0 skipped
```

Also verified `git diff --check` and confirmed the node contains no NumPy import
or SDK blocking wait call.

## Self-review

- Scope is limited to the seven Task 4 files plus this required report.
- YAML pins serial `00000000F0271544`, 640x480 at 30 Hz, and reconnect 2.0 s.
- Launch contains exactly one `powertrain_ros/l515_camera` node and passes the
  installed YAML.
- Setup installs config/launch and registers the requested console script;
  package.xml declares `sensor_msgs`.
- No Jetson, hardware, or unrelated user files were touched.

## Review-fix evidence (2026-07-11)

- RED after adding regression coverage: 4 failed, 5 passed. Failures proved
  the old `l515_camera` node name, missing source cleanup after `start()`
  failure, missing `rclpy.shutdown()` after constructor failure, and the old
  launch node name.
- GREEN focused verification after implementation and clean package install:
  `test_l515_node.py` plus `test_l515_launch_contract.py`: 9 passed in 0.34 s.
- Launch now passes `PathJoinSubstitution` directly. The test evaluates the
  resulting `ParameterFile` against the active ament index and proves it is
  exactly the installed `share/powertrain_ros/config/l515.yaml` regular file.
- The constructor and launch/config root use exactly `l515_camera_node`; the
  console executable remains `l515_camera`.
- Constructor/start failure stops the partial source and destroys the partial
  ROS node. Constructor and spin failures always reach `rclpy.shutdown()`;
  spin failure destroys the node first.
- All six publishers assert the complete sensor-data QoS contract: depth 5,
  best-effort reliability, keep-last history, and volatile durability. The
  registered 5 ms timer callback is proven to be exactly `_drain_source`; one
  invocation performs one `poll_latest()` without calling source start/stop.
- Fresh clean three-package verification:
  `colcon build`: 3 packages finished; `colcon test`: 3 packages finished;
  `colcon test-result --verbose`: 69 tests, 0 errors, 0 failures, 0 skipped.
- `ament_flake8` checked the four changed Python files: no problems found.
  `git diff --check` exited cleanly.
