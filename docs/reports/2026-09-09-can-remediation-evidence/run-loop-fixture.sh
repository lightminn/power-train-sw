#!/usr/bin/env bash
set -e
unset PYTHONPATH
source /opt/ros/humble/setup.bash
source /workspace/ros2/install/setup.bash
export PYTHONPATH=/workspace:/workspace/motor_control:${PYTHONPATH:-}
cd /workspace
exec python3 /evidence/jetson_loop_fixture.py
