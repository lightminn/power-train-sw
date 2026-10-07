#!/usr/bin/env bash
# 연결된 영상 토픽을 rqt_image_view 창으로 한 번에 띄운다.
#
# run_calib.sh / run_monitor.sh 와 같은 이유로 만들었다 — 매번 4줄을 치는 대신
# 환경 준비(ROS 소싱 + 오버레이 + ROS_DOMAIN_ID + DISPLAY)를 한 곳에 모은다.
#
#   bash src/robot_arm_perception/scripts/run_views.sh              # 기본 4종
#   bash src/robot_arm_perception/scripts/run_views.sh /wrist/raw_image
#
# ⚠️ `*/debug_image` 는 **구독자가 있을 때만** 발행된다(perception_node·wrist_camera
#    의 구독자 게이트). 창을 여는 것만으로 그쪽 노드가 오버레이를 그리기 시작해
#    추론 대역을 쓴다 — 필요 없으면 raw 만 열 것.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export DISPLAY="${DISPLAY:-:0}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS_ROOT}/install/setup.bash"
set -u

TOPICS=("$@")
if [ ${#TOPICS[@]} -eq 0 ]; then
  TOPICS=(/perception/raw_image /perception/debug_image \
          /wrist/raw_image /wrist/debug_image)
fi

for t in "${TOPICS[@]}"; do
  echo "[run_views] ${t}"
  # 창마다 별도 프로세스다 — 하나를 닫아도 나머지는 그대로 산다.
  nohup ros2 run rqt_image_view rqt_image_view "${t}" \
    > "/tmp/view$(echo "${t}" | tr '/' '_').log" 2>&1 &
  sleep 2
done

echo
echo "[run_views] 창 ${#TOPICS[@]}개 요청 — 각 창은 독립 프로세스입니다."
echo "[run_views] 정리: pkill -f '[r]qt_image_view'"
