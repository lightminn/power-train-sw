#!/usr/bin/env bash
# 관제 GUI 실행 래퍼 (기본 읽기 전용, control:=true 면 제어 콘솔).
#
# 환경 준비를 대신 해준다 — run_calib.sh 와 같은 이유로 만들었다. 이 저장소에서
# 아래 둘은 각각 여러 번 시간을 잡아먹은 실패 모드다:
#   - 워크스페이스 오버레이 미소싱 → ModuleNotFoundError / 패키지 못 찾음
#   - ROS_DOMAIN_ID 불일치        → 토픽이 하나도 안 보임(노드는 멀쩡히 떠 있음)
#
# 사용법 (컨테이너 안, 아무 터미널):
#   bash src/robot_arm_gui/scripts/run_monitor.sh
#   bash src/robot_arm_gui/scripts/run_monitor.sh fake:=true      # 하드웨어 없이 검증
#   bash src/robot_arm_gui/scripts/run_monitor.sh control:=true   # ⚠️ 실제로 팔이 움직인다
#   bash src/robot_arm_gui/scripts/run_monitor.sh bind:=0.0.0.0   # 현장 네트워크 노출
#
# 원격 PC 에서 보기(권장 — 포트를 열지 않는다):
#   ssh -L 8088:localhost:8088 <jetson>   후 브라우저에서 http://localhost:8088
#
# 다른 노드를 띄울 때 쓴 도메인과 같아야 한다. 이미 설정돼 있으면 존중한다.

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
  echo "[run_monitor] ${WS_ROOT}/install/setup.bash 이 없습니다 — 먼저 빌드하세요:" >&2
  echo "[run_monitor]   colcon build --packages-select robot_arm_gui" >&2
  exit 1
fi
set -u

if ! ros2 pkg prefix robot_arm_gui >/dev/null 2>&1; then
  echo "[run_monitor] robot_arm_gui 패키지를 찾을 수 없습니다." >&2
  echo "[run_monitor] ${WS_ROOT} 에서 'colcon build --packages-select robot_arm_gui' 실행" >&2
  exit 1
fi

cd "${WS_ROOT}"
echo "[run_monitor] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} / 워크스페이스=${WS_ROOT}"
if [[ " $* " == *"control:=true"* ]]; then
  echo "[run_monitor] ⚠️ 제어 모드 — /arm/teleop_jog·/arm/teleop_cmd 를 발행합니다."
  echo "[run_monitor]    teleop_core 가 떠 있으면 브라우저 조작으로 팔이 실제로 움직입니다."
  echo "[run_monitor]    계약 토픽(/arm_status 등)과 /dynamixel/goal_position 은 발행하지 않습니다."
else
  echo "[run_monitor] 읽기 전용 모니터입니다 — 이 노드는 어떤 토픽도 발행하지 않습니다."
fi
exec ros2 launch robot_arm_gui monitor.launch.py "$@"
