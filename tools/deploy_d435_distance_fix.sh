#!/usr/bin/env bash
set -euo pipefail

jetson="${1:-zetin@192.168.8.106}"
arm_root="/home/kimseon/extreme-robot/ros2_ws/src/robot_arm_perception"
remote_root="extreme-robot/ros2_ws/src/robot_arm_perception"
stamp="$(date +%Y%m%d-%H%M%S)"
control_socket="$(mktemp -u /tmp/zetin-d435-ssh.XXXXXX)"
backup_root=".local/state/zetin-deploy-backups"

cleanup() {
  ssh -S "$control_socket" -O exit "$jetson" >/dev/null 2>&1 || true
}
trap cleanup EXIT

files=(
  "robot_arm_perception/perception_node.py"
  "robot_arm_perception/perception_quality.py"
  "robot_arm_perception/stream_node.py"
)

printf 'Jetson %s에 D435i 거리 수정본을 배포합니다.\n' "$jetson"
printf 'SSH 비밀번호는 처음 한 번만 입력하세요.\n\n'

ssh -M -S "$control_socket" -o ControlPersist=120 -fN "$jetson"

ssh -S "$control_socket" "$jetson" "
  set -e
  mkdir -p '$backup_root'
  for misplaced in extreme-robot/ros2_ws/src/robot_arm_perception.backup-*; do
    if [ -d \"\$misplaced\" ]; then
      mv \"\$misplaced\" '$backup_root/'
    fi
  done
  test -d '$remote_root'
  cp -a '$remote_root' '$backup_root/robot_arm_perception-${stamp}'
"

for relative in "${files[@]}"; do
  scp -o ControlPath="$control_socket" \
    "${arm_root}/${relative}" "${jetson}:${remote_root}/${relative}"
done

ssh -S "$control_socket" "$jetson" "
  set -e
  docker exec ros2_humble bash -lc '
    source /opt/ros/humble/setup.bash
    cd /root/ros2_ws
    colcon build --packages-select robot_arm_perception
  '
  docker exec ros2_humble pkill -f '[r]obot_arm_perception.perception_node' || true
  docker exec ros2_humble pkill -f '[r]obot_arm_perception.stream_node' || true
  docker exec -d ros2_humble bash -lc '
    source /opt/ros/humble/setup.bash
    source /root/ros2_ws/install/setup.bash
    exec ros2 run robot_arm_perception perception_node --ros-args \
      -p camera_mode:=realsense -p pick_min_conf:=0.55 -p require_depth:=true
  '
  docker exec -d ros2_humble bash -lc '
    source /opt/ros/humble/setup.bash
    source /root/ros2_ws/install/setup.bash
    exec ros2 run robot_arm_perception stream_node
  '
  sleep 4
  docker exec ros2_humble pgrep -af robot_arm_perception.perception_node
  docker exec ros2_humble pgrep -af robot_arm_perception.stream_node
"

printf '\n배포와 노드 재시작이 완료되었습니다.\n'
