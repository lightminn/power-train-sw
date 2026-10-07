# ROS 2 manual hardware validation GUI

## Package layout

- `robot_manual_gui/ros_interface.py`: ROS subscriber/publisher/action frontend
- `robot_manual_gui/main_window.py`: PyQt5 widgets and safety enable/disable rules
- `robot_manual_gui/main.py`: Qt loop plus ROS executor thread
- `launch/manual_gui.launch.py`: optional interchangeable-tool stack include
- `test/test_ros_interface.py`: existing-interface/no-register-access contract tests

The GUI never imports `dynamixel_sdk` and never reads or writes a register. ID-level
diagnostics in `/tool/status` are populated from samples already read by
`moveit_dynamixel_bridge`.

## Run

Hardware-free:

```bash
cd /home/asd/extreme-robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_manual_gui manual_gui.launch.py mock_mode:=true
```

Safe hardware discovery (default is read-only):

```bash
ros2 launch robot_manual_gui manual_gui.launch.py \
  mock_mode:=false read_only:=true tool_type:=dual_motor_gripper
```

Only after read-only checks and profile review:

```bash
ros2 launch robot_manual_gui manual_gui.launch.py \
  mock_mode:=false read_only:=false tool_type:=dual_motor_gripper
```

If bridge/FSM are already running, prevent duplicate serial owners/nodes:

```bash
ros2 launch robot_manual_gui manual_gui.launch.py \
  start_stack:=false mock_mode:=false tool_type:=dual_motor_gripper
```

## Button-to-interface mapping

| GUI operation | Existing ROS interface |
|---|---|
| Arm `-`, `+`, `GO` | publish `trajectory_msgs/JointTrajectory` to `/arm_controller/joint_trajectory` |
| Gripper OPEN/CLOSE | `control_msgs/action/FollowJointTrajectory` on `/gripper_controller/follow_joint_trajectory` |
| Gripper STOP | cancel the active gripper action; bridge failure/cancel path stops tool torque |
| Cleaner START/STOP | publish `std_msgs/Bool` to `/cleaning/enable` |
| EMERGENCY STOP | publish `true` to `/tool/emergency_stop`; publish cleaner stop; manual arm hold trajectory |
| TOOL DETACHED | publish `true` to `/tool/detached` |
| MANUAL/FSM request | publish `std_msgs/String` to `/control/mode` |
| Monitoring | `/joint_states`, `/tool/type`, `/tool/status`, `/arm_status`, `/fsm/state`, `/control/mode_status`, `/sensors/contact_status` |

## Ownership

`arm_fsm_node` is the authority for `/control/mode_status`. It accepts MANUAL only
in `ToolManager.SAFE_CHANGE_STATES` (`IDLE`, `STOWED`, `STOWED_LOCKED`). While
MANUAL, its tick does not run actuator-producing state handlers and VLA task commands
are rejected. The GUI enables motion widgets only after the accepted status returns
MANUAL. In FSM mode all manual motion widgets are disabled.

## First connected test sequence

1. Mechanically stow the arm, support the tool, and prepare physical power removal.
2. Start with `mock_mode:=false read_only:=true` and the exact `tool_type`.
3. Confirm GUI bridge status TRUE, U2D2 TRUE, correct tool type, discovered IDs, and
   profile values. Motion allowed must remain FALSE in read-only mode.
4. Check the diagnostics table for stable ID 3/4 positions/load and online status.
5. Stop the read-only launch. Inspect calibration YAML and physical attachment.
6. Start `read_only:=false`; do not command until profile/discovered/motion are TRUE.
7. Request MANUAL while FSM is IDLE/STOWED.
8. Select 0.5 degree jog. Test one feedback-confirmed arm joint at a time, one click,
   with a second operator at emergency power-off.
9. Use gripper OPEN once, inspect both ID positions/load, then CLOSE on a compliant
   test object. Press STOP/E-stop on any divergence.
10. Return to FSM ownership before VLA/FSM testing.

## Deliberate limitations

- Runtime hardware reprovisioning is not available. The dropdown enforces the
  ToolManager safe-state policy, then instructs the operator to stop, physically
  replace, and relaunch with a new `tool_type`.
- There is no authenticated E-stop reset interface. RESET remains disabled; after
  an E-stop/detach the bridge must be inspected and restarted.
- The active bridge currently has verified mappings/feedback for `arm_joint_1..3`
  only. The GUI shows all five requested axes, but hardware mode enables a joint
  only after it appears in `/joint_states`. It does not invent IDs for joints 4/5.
- Calibration runs in the existing guarded `spur_gripper_calibration` executable.
  The GUI refuses to launch it while bridge status is live because the serial bus
  must have one owner.
- Cleaner real START remains disabled until its required actuator profile is fully
  configured and discovered.
