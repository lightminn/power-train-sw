#!/usr/bin/env bash
set -e
unset PYTHONPATH
source /opt/ros/humble/setup.bash
source /workspace/ros2/install/setup.bash
export PYTHONPATH=/workspace:/workspace/motor_control:$PYTHONPATH
export PYTHONDONTWRITEBYTECODE=1
cd /workspace
python3 - <<'PY'
from pathlib import Path
import subprocess
names=('approach','autonomy','lane_follower','lead_follower','mission_supervisor','section_','terrain_','wp6','wp8','l515','obstacle_zones')
selected=[str(p) for p in sorted(Path('ros2/src/powertrain_ros/test').glob('test_*.py')) if p.name!='test_install_space_imports.py' and not any(name in p.name for name in names)]
print('Selected ROS remote-operation test files:',len(selected),flush=True)
raise SystemExit(subprocess.call(['python3','-m','pytest','powertrain_runtime/tests',*selected,'-q','-p','no:cacheprovider','--disable-warnings']))
PY
python3 -m pytest ros2/src/powertrain_ros/test/test_install_space_imports.py -q -p no:cacheprovider
