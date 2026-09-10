#!/usr/bin/env bash
# 운영자 콘솔만 띄운다 (스택이 이미 떠 있을 때). 스택까지 새로 띄우려면 run_pick.sh.
#
# 이 스크립트가 있는 이유는 하나다 — `docker exec ... bash -lc 'source ...; ros2 run ...'`
# 한 줄을 손으로 치다가 **`/opt/ros/humble/setup.bash` 소싱을 빠뜨려** `ros2: command
# not found` 로 못 뜨는 일이 반복됐다. `bash -lc` 는 로그인 셸이라 ROS 를 소싱해 두는
# ~/.bashrc 를 읽지 않는다(대화형 `docker exec -it ... bash` 는 읽어서 되는 것과 차이).
#
# 사용법 (컨테이너 안, 진짜 tty 가 있는 터미널):
#   docker exec -it ros2_humble bash
#   cd /root/ros2_ws && bash src/dynamixel_control/scripts/run_console.sh
#
#   # 종료할 때 토크까지 풀고 싶으면
#   bash src/dynamixel_control/scripts/run_console.sh --ros-args -p release_torque_on_exit:=true

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
else
  echo "[run_console] ${WS_ROOT}/install/setup.bash 이 없습니다 — 먼저 빌드하세요." >&2
  exit 1
fi
set -u

if ! pgrep -f '(^|/| )arm_fsm( |$)' >/dev/null 2>&1; then
  echo "[run_console] ⚠️ arm_fsm 이 안 보입니다 — 픽 스택이 안 떠 있는 것 같습니다."
  echo "[run_console]    스택부터 띄우려면: bash src/dynamixel_control/scripts/run_pick.sh"
  echo
fi

cd "${WS_ROOT}"
echo "[run_console] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
exec ros2 run dynamixel_control mission_console "$@"
