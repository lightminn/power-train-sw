#!/usr/bin/env bash
# OV5640 + YOLO Seg("박스") 테스트 창을 쉽게 띄우는 래퍼.
#
# 키보드 텔레옵(run_keyboard_bench.sh)과는 완전히 별개 프로세스다 — 같이 띄워도
# 되고 따로 띄워도 된다. vision_test_node 는 /vision_test/detected_objects 라는,
# 시스템 어디에서도 구독하지 않는 토픽에만 발행하므로 로봇팔 동작에 영향이 없다
# (vision_test_node.py 모듈 docstring 참고).
#
# 사용법 (컨테이너 안, 아무 터미널 — GUI 창이 뜨니 X 포워딩 필요, 호스트에서
# `xhost +local:docker` 는 이미 했다고 가정):
#   bash src/robot_arm_perception/scripts/run_vision_test.sh
#   # 카메라가 /dev/video0 이 아니면:
#   bash src/robot_arm_perception/scripts/run_vision_test.sh --ros-args -p camera_device:=2
#
# q 를 누르면(창에 포커스 있는 상태) 종료된다.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# 워크스페이스 오버레이 미소싱 대응 — run_keyboard_bench.sh 와 동일한 이유
# (dynamixel_control/scripts/run_keyboard_bench.sh 상단 주석 참고).
if ! ros2 pkg prefix robot_arm_perception >/dev/null 2>&1; then
  if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    echo "[run_vision_test] 워크스페이스 오버레이 미소싱 감지 → 자동 소싱: ${WS_ROOT}/install/setup.bash"
    set +u
    # shellcheck disable=SC1091
    source "${WS_ROOT}/install/setup.bash"
    set -u
  fi
fi

if ! ros2 pkg prefix robot_arm_perception >/dev/null 2>&1; then
  echo "[run_vision_test] robot_arm_perception 패키지를 찾을 수 없습니다." >&2
  echo "[run_vision_test] ${WS_ROOT} 에서 'colcon build --packages-select robot_arm_perception' 을 먼저 실행하세요." >&2
  exit 1
fi

# model_path 기본값이 "src/robot_arm_perception/models/best.pt" 상대경로라
# (model_presets.py, perception_node 와 동일한 관례) 워크스페이스 루트에서 실행돼야 한다.
cd "${WS_ROOT}"
echo "[run_vision_test] vision_test 시작 (워크스페이스: ${WS_ROOT})"
exec ros2 run robot_arm_perception vision_test "$@"
