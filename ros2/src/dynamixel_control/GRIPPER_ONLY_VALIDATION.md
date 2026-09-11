# Gripper-only validation

Validated with only Dynamixel IDs 3 and 4 connected. No arm motors were connected.

## Command

```bash
source /opt/ros/humble/setup.bash
source /home/asd/extreme-robot-fresh/ros2_ws/install/setup.bash
ros2 run dynamixel_control moveit_dynamixel_bridge \
  --ros-args -p gripper_only_mode:=true
```

In this mode, bridge startup performs no torque-enable or position-register writes.
Arm FollowJointTrajectory, arm trajectory topic, and arm teleop commands are rejected.
Both configured gripper IDs must respond without a hardware error for
`/dynamixel/controller_fault` to be `false`.

## Observed result

- Startup reported `gripper_only_mode=True` and disabled startup torque/position writes.
- No `Torque enabled` message was emitted.
- `/joint_states` contained only `gripper_drive_joint`.
- `/dynamixel/controller_fault` was `false` with IDs 3 and 4 responding.
- Arm teleop ID 0 was rejected by the gripper-only guard.
- Unknown motor ID 99 was rejected before any Dynamixel write.
- No gripper action, torque-enable, or goal-position write was used in this validation.

## Read-only hardware snapshot

| ID | Model | Mode | Torque | HW error | Goal tick | Present tick |
|---:|---:|---:|---:|---:|---:|---:|
| 3 | 1060 | 4 | 0 | 0 | 1040 | 1040 |
| 4 | 1060 | 4 | 0 | 0 | 2359 | 2359 |

This snapshot records the tested assembly state; it is not an endpoint calibration.
