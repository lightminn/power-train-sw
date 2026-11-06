#!/usr/bin/env bash
# Inside the arm container: SIGKILL the arm bridge and launch it again.
# Used by scripts/start_jetson_arm_bench.sh and by the ops broker's
# robot_arm_bridge_restart action (console "브릿지 재시작" button).
# SIGKILL, never a clean stop: a clean shutdown turns arm torque off, while a
# killed bridge leaves every servo holding and the new one adopts them.
# A servo in hardware error (e.g. overload) is rebooted first so it comes back.
# Exit 0 only when the new bridge started and read every arm servo.
set -u
params="${1:-/session/session.yaml}"
log="${2:-/session/bridge-rt.log}"

pkill -9 -f '[m]oveit_dynamixel_bridge'
sleep 1
if [ -f "$log" ]; then mv "$log" "$log.prev"; fi

set +u
source /opt/ros/humble/setup.bash
source /session/runtime/install/setup.bash
set -u
export PYTHONPATH="/usr/lib/python3/dist-packages:/session/python:/ws:/ws/motor_control:${PYTHONPATH:-}"
export PYTHONUNBUFFERED=1
# A servo latched in a hardware error stays dead until rebooted, and the bridge
# only finds servos at startup, so clear those before it opens the bus.
python3 "$(dirname "$0")/arm_clear_servo_errors.py" >> "$log.recover" 2>&1
setsid ros2 run dynamixel_control moveit_dynamixel_bridge \
    --ros-args --params-file "$params" > "$log" 2>&1 < /dev/null &

for _ in $(seq 1 30); do
    sleep 1
    grep -q "MoveIt Dynamixel bridge started" "$log" && break
    pgrep -f '[m]oveit_dynamixel_bridge' > /dev/null || break
done
grep -q "MoveIt Dynamixel bridge started" "$log" \
    && ! grep -q "startup torque read failed" "$log"
