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
