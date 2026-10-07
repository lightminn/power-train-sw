#!/usr/bin/env bash
# 캘리브레이션 스크립트(measure_gear_ratio.py / measure_zero_offset.py) 실행 래퍼.
#
# 환경 준비(ROS 소싱 + 워크스페이스 오버레이 + ROS_DOMAIN_ID)를 대신 해준다 —
# 2026-08-07 세션에서 이 세 가지가 각각 한 번씩 걸려서 만들었다:
#   - 오버레이 미소싱  → measure_zero_offset.py 가 ModuleNotFoundError
#   - ROS_DOMAIN_ID=0 → /joint_states 를 못 받음(브릿지는 77 에 떠 있음)
#   - 새 터미널마다 위 둘이 초기화됨
#
# 사용법 (컨테이너 안, 아무 터미널):
# 캘리브 순서는 gear_ratio → zero_offset → limits 다(뒤의 것이 앞의 것에 의존).
#   bash src/dynamixel_control/scripts/run_calib.sh gear_ratio arm_joint_2
#   bash src/dynamixel_control/scripts/run_calib.sh zero_offset
#   bash src/dynamixel_control/scripts/run_calib.sh zero_offset --reference arm_joint_3:1.5708
#   bash src/dynamixel_control/scripts/run_calib.sh limits arm_joint_4
#   bash src/dynamixel_control/scripts/run_calib.sh gripper
#   bash src/dynamixel_control/scripts/run_calib.sh campose   # 카메라 TF 다점 캘리브
#
# 브릿지 도메인이 77 이 아니면 ROS_DOMAIN_ID 를 미리 export 해두면 그 값을 존중한다.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# 브릿지를 띄울 때 쓴 도메인과 같아야 한다. 이미 설정돼 있으면 건드리지 않는다.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
else
  echo "[run_calib] ${WS_ROOT}/install/setup.bash 이 없습니다 — 먼저 빌드하세요:" >&2
  echo "[run_calib]   colcon build --packages-select dynamixel_control" >&2
  exit 1
fi
set -u

if [ $# -lt 1 ]; then
  echo "사용법: bash $0 {gear_ratio|zero_offset} [스크립트 인자...]" >&2
  exit 2
fi

case "$1" in
  gear_ratio)  TARGET="measure_gear_ratio.py" ;;
  zero_offset) TARGET="measure_zero_offset.py" ;;
  limits)      TARGET="measure_joint_limits.py" ;;
  gripper)     TARGET="measure_gripper_endpoints.py" ;;
  campose)     TARGET="calibrate_camera_pose.py" ;;
  *)
    echo "[run_calib] 모르는 대상: $1 (gear_ratio | zero_offset | limits | gripper | campose)" >&2
    exit 2
    ;;
esac
shift

cd "${WS_ROOT}"
echo "[run_calib] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ${TARGET} 실행"
echo "[run_calib] ⚠️ moveit_dynamixel_bridge 가 read_only:=true 로 떠 있어야 합니다(토크 OFF)."
exec python3 "src/dynamixel_control/scripts/${TARGET}" "$@"
