# CAN automatic resume — host implementation handoff

2026-09-10. Local host implementation and deterministic checks are complete. Installed ROS/vcan acceptance is being executed by root; its result is not claimed here. This worker did not access Jetson or motors, edit firmware/instructions/shared specs, or commit/push.

## Final contract

- Heartbeat standard prefix unchanged; only A3/patch3 is auto eligible. Byte6 latch/progress/eligible and byte7 uint8 generation are decoded. A2, unknown markers, truncated/unknown flag frames revoke cached eligibility.
- Automatic target command is one atomic firmware guarded RESUME: CANSimple cmd0x07, DLC8, `struct.pack('<IBBBB', 8, 0xA3, expected_generation, 0, 0)`. No automatic CLEAR_ERRORS is emitted. Error0 continuously observed peers use ordinary state8 without clear. Manual arm retains its ordinary clear→mode→zero→state8 sequence.
- Firmware must atomically validate eligibility/latch/generation/exact0x4000/sub-errors/IDLE/transport health, then clear+zero+request8. A rejected command must leave the requested state unchanged. This closes the demonstrated external-clear-after-host-check race that guarded clear followed by ordinary state8 did not close.
- Chassis owns all six axes. CAN-only interruption stops every output under `can_recovery`; fresh healthy manual input, continuous server connection ownership, and no other hold are required. Every observed lost/stale drive axis needs its own eligible A3 origin. Origins/generations already visible at entry are recorded before stop writes. Required nodes are observable.
- All six heartbeat and encoder ages must be ≤200ms, all wheels <0.1 wheel rev/s, and steering fresh/healthy for a continuous200ms. The recovery episode deadline is5s; generation changes reset the stability dwell, not the deadline.
- CAN-correlated steering stale can wait while stopped; isolated steering stale and actual steering faults remain strict. Each tick preflights all axes before dispatch. New commands in WAIT_INPUT are staged; only a tick with all drive/steer evidence valid may release the hold and distribute the command.
- Fresh manual source means the original server TCP receive time bound to the exact typed message, not a new DDS publication of a cached gateway result. CommandAuthority preserves both DDS receipt and TCP source receipt. The server accept UUID is bound to the same command through gateway→message→authority→manager. A reconnect with the same client-selected session ID still changes the server UUID and permanently cancels pending resume.
- Manual legacy Twist remains supported for driving, with no automatic resume provenance. Source/connection loss, disarm, E-stop, other hold, component/mode changes and timeout cancel pending resume; later packets cannot rearm. Deadman release remains a valid zero command.
- Normal snapshot healthy remains false throughout pending recovery. Existing hardware stop-proof change was preserved unchanged; a new USB/non-CAN0x4000 negative confirms the CAN-only exception does not leak.

## Files

Production:
- `motor_control/chassis/can_recovery.py` (new pure evidence/state helper)
- `motor_control/chassis/chassis_manager.py` (coordinator; preserves root's prior stop proof delta)
- `motor_control/chassis/authority.py` (original receipts + server connection UUID)
- `motor_control/corner_module/drive_odrive_can.py` (A3 decoder, guarded resume, arm-edge sample guard)
- `motor_control/corner_module/corner_module.py` (optional checked samples / automatic-arm guard; standalone defaults unchanged)
- `ros2/src/powertrain_msgs/msg/ManualDriveCommand.msg` (`source_received_s`, `connection_session_id`)
- `ros2/src/powertrain_ros/powertrain_ros/{chassis_node,teleop_command_node,remote_input_gateway}.py`

New tests/fixture:
- `motor_control/chassis/tests/test_can_auto_recovery.py`
- `motor_control/chassis/tests/test_authority_receipt.py`
- `motor_control/corner_module/tests/test_can_recovery_metadata.py`
- `ros2/src/powertrain_ros/test/test_can_recovery_receipt.py`
- `ros2/src/powertrain_ros/test/installed_can_auto_resume.py`

Updated existing tests:
- `motor_control/chassis/tests/test_hardware_stop_proof.py` (append USB negative; root's prior tests preserved)
- `ros2/src/powertrain_ros/test/test_manual_drive_adapter.py` (new fields, deterministic DDS age clock)
- `ros2/src/powertrain_ros/test/test_remote_input_gateway.py` (new output fields)

## Executed verification

TDD observed failures before each behavior:
- Initial A2-era metadata/authority receipt contracts:6 RED then6 GREEN.
- Initial coordinated recovery cases:16 RED (missing recovery API), then16 GREEN.
- Arm-edge real-error / steering send / whole-bus steering-stale cases:3 RED then GREEN.
- Original TCP receipt and gateway publication contracts:4 RED, corrected the test decoder invocation, then remaining3 RED→GREEN.
- A3 and guarded atomic resume transitions: explicit RED runs including A2 rejection, generic-clear prohibition and external-clear/revoke race.
- Independent-review P1 tests: WAIT_INPUT stale steering with new input, multiple lost nodes with one reboot/error0, initial origin disappearing before next tick:3 RED→GREEN.
- Server connection epoch: missing-API RED→GREEN; production event drain demonstrates collapsed disconnect/connect/new-neutral retains the new server epoch and cancels the old one.

Final combined command, from repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
/home/light/anaconda3/bin/python -m pytest -q --tb=short -p no:cacheprovider \
  motor_control/chassis motor_control/corner_module \
  ros2/src/powertrain_ros/test/test_remote_input_gateway.py \
  ros2/src/powertrain_ros/test/test_can_recovery_receipt.py \
  ros2/src/powertrain_ros/test/test_manual_drive_adapter.py \
  ros2/src/powertrain_ros/test/test_chassis_can_remediation.py \
  ros2/src/powertrain_ros/test/test_teleop_command_resilience.py \
  ros2/src/powertrain_ros/test/test_hardware_stop_proof.py
```

Result: **838 passed in3.74s**. Scoped `git diff --check` passed.

Host has no rclpy/geometry_msgs. Direct collection of installed ROS receipt/entrypoint tests failed at import; that is an environment boundary, not a passed ROS test. No autonomy suite was run.

## Installed ROS/vcan execution

Root owns this execution. Required: domain77, network-none privileged container with vcan77 only, no can0, runtime directory provisioned with the repository installer, rebuilt non-symlink install of powertrain_msgs/robot_arm_msgs/powertrain_ros. `source_received_s` and `connection_session_id` require rebuilding msgs, not just copying node Python.

Inside the isolated container:

```bash
cd /workspace
export ROS_DOMAIN_ID=77
export PYTHONPATH=/workspace:/workspace/motor_control
source /opt/ros/humble/setup.bash
source /evidence/install/setup.bash
python3 /workspace/ros2/src/powertrain_ros/test/installed_can_auto_resume.py \
  --case all --evidence-dir /evidence/auto-resume
```

The fixture rejects a physical/renamed CAN interface via `ip -details -json` kind verification, asserts the imported chassis module is installed, and launches installed `ros2 run powertrain_ros teleop_command`, `ops_broker`, and `chassis` processes. It uses real TCP frames, DDS, real six CAN driver parsers/commands, and simulated six ODrive/four AK devices on vcan77. Services are `/chassis_node/{authority_manual,arm,disarm,estop}`; safety topic remains `/chassis/safety_state`.

Cases: `resume`, `cached_input`, `disarm`, `disconnect`, `reconnect`, `timeout`, `estop`. Positive verifies all-stop→stable guarded rearm→old TCP command republished over DDS cannot move→new TCP input resumes. Negative cases check permanent cancellation with no auto clear or motion. Each case saves `result.json` and node logs and rejects Traceback during execution/cleanup. The reconnect case reuses the client session ID and checks that new commands cannot rearm. Rapid reconnect coalescing additionally has deterministic production event-drain/callback coverage on the host.

First root installed execution reached real node startup but exposed wrong fixture private-service prefix `/chassis/`; fixed to `/chassis_node/` before the final snapshot. Root rerun is pending at this report write.

## Remaining evidence boundary

Atomic firmware semantics were agreed directly with the firmware worker and must be built/tested/deployed by root. This report's emulator is a protocol boundary test, not proof of physical firmware, braking, steering or motion. The independent reviewer identified three substantive flaws; all were accepted and fixed with regressions before this handoff. Installed E2E and physical zero-command acceptance remain separately reported by root.
