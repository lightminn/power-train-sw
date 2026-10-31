#!/usr/bin/env bash
# Build exactly one Dynamixel package, alongside the shared messages and broker.
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_root="${1:-/tmp/arm-console-runtime}"
source /opt/ros/humble/setup.bash
export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$build_root"
build_root="$(cd -- "$build_root" && pwd)"
colcon --log-base "$build_root/log" build \
  --base-paths \
    "$repo_root/extreme-robot/ros2_ws/src/robot_arm_msgs" \
    "$repo_root/extreme-robot/ros2_ws/src/dynamixel_control" \
    "$repo_root/ros2/src/powertrain_msgs" \
    "$repo_root/ros2/src/powertrain_ros" \
  --build-base "$build_root/build" --install-base "$build_root/install" --symlink-install
source "$build_root/install/setup.bash"
python3 -c 'from dynamixel_control.tool_manual_control import ToolManualControl'
ros2 pkg prefix dynamixel_control
printf 'Runtime ready. Source this overlay last: %s/install/setup.bash\n' "$build_root"
