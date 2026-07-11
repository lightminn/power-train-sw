#!/bin/bash
set -euo pipefail

workspace=${WORKSPACE_ROOT:-/workspace}
ros_setup=${ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}
ros_workspace="$workspace/ros2"
install_setup="$ros_workspace/install/setup.bash"

source_setup() {
    set +u
    # ROS/ament setup scripts are not guaranteed to be nounset-safe.
    source "$1"
    set -u
}

source_setup "$ros_setup"
needs_build=false
if [[ ! -f "$install_setup" ]]; then
    needs_build=true
elif find "$ros_workspace/src" -type f -newer "$install_setup" -print -quit | grep -q .; then
    needs_build=true
fi

if [[ "$needs_build" == true ]]; then
    cd "$ros_workspace"
    colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros
fi

source_setup "$install_setup"
cd "$workspace"
exec python3 -m l515_dashboard.gateway_main
