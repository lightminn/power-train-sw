#!/bin/bash
set -euo pipefail
source /opt/ros/humble/setup.bash
if [[ -f /workspace/ros2/install/setup.bash ]]; then source /workspace/ros2/install/setup.bash; fi
cd /workspace
exec python3 -m l515_dashboard.gateway_main
