#!/usr/bin/env bash
# 비전 → 픽을 한 줄로: 스택(pick.launch.py)을 백그라운드로 띄우고, 운영자 콘솔을
# 이 터미널의 foreground 에서 돌린다. run_keyboard_bench.sh 와 같은 구조다.
#
# 콘솔(mission_console)은 stdin 이 필요해 launch 안에 넣을 수 없다 — 그래서
# launch 출력은 로그 파일로 돌리고(터미널에 섞이면 프롬프트가 깨진다) 콘솔만
# 진짜 tty 를 물려받는다. 콘솔을 q/Ctrl-C 로 끝내면 trap 이 스택을 통째로 정리한다.
#
# 사용법 (컨테이너 안, 진짜 tty 가 있는 터미널):
#   docker exec -it ros2_humble bash
#   cd /root/ros2_ws && bash src/dynamixel_control/scripts/run_pick.sh
#
#   # launch 인자는 그대로 전달된다
#   bash src/dynamixel_control/scripts/run_pick.sh width:=848 height:=480 fps:=30
#
# ⚠️ 이 경로는 실서보를 구동한다. 콘솔에서 'pick' 을 치는 순간 팔이 움직인다.
# ⚠️ 벤치 텔레옵(position_node/teleop_core)과 **버스가 겹친다** — 이 스크립트가
#    시작 전에 그쪽을 정리한다(둘이 같이 /dev/ttyUSB0 을 쓰면 원인 불명 무응답).

set -euo pipefail
set -m   # 잡 컨트롤 on — 백그라운드 launch 가 자기 pgid 를 갖게 해서 통째로 정리 가능

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
  echo "[run_pick] ${WS_ROOT}/install/setup.bash 이 없습니다 — 먼저 빌드하세요:" >&2
  echo "[run_pick]   colcon build --packages-select dynamixel_control" >&2
  exit 1
fi
set -u

if ! ros2 pkg prefix dynamixel_control >/dev/null 2>&1; then
  echo "[run_pick] dynamixel_control 을 찾을 수 없습니다 — colcon build 후 다시." >&2
  exit 1
fi

# 정리 대상 두 갈래. 경계 앵커를 거는 이유는 run_keyboard_bench.sh 의 같은 상수
# 주석 참고 — 느슨하게 쓰면 이 스크립트 자신을 죽인다.
#
#  (1) 벤치 텔레옵: /dev/ttyUSB0 를 브릿지와 공유할 수 없다.
#  (2) **이전 픽 스택 자신**: 2026-08-12 실기에서 이걸 안 지워 사고가 났다 —
#      스택이 이미 떠 있는데 이 스크립트를 또 돌리면 브릿지 둘이 같은 시리얼
#      포트를 두드려 `result=-3002`(COMM_RX_TIMEOUT)로 **그리퍼 초기화만 조용히
#      실패**하고(팔 축은 우연히 성공), 그리퍼가 SyncRead 에서 빠져 /joint_states
#      에 아예 안 나온다 → FSM 이 `grasp effort 0.0` 으로 파지 실패 판정.
#      먼저 뜬 브릿지는 SerialException("multiple access on port")으로 죽는다.
#      증상이 "팔은 가는데 그리퍼가 안 닫힌다"로 보여서 원인을 찾기 어렵다.
CONFLICT_PATTERN='(^|/| )(position_node|dynamixel_position_node|teleop_core|keyboard_teleop|joystick_teleop)( |$)'
#      ⚠️ **pick.launch.py 가 띄우는 노드를 하나라도 빠뜨리면 안 된다.** 2026-08-19 에
#      `wrist_camera`/`rviz2` 를 launch 에 추가하면서 이 목록을 같이 안 고쳤더니,
#      새 스택을 띄워도 **옛 손목캠 프로세스가 살아남아 /dev/video8 을 계속 잡고**
#      있었다 → 새 wrist_camera 는 장치를 못 열고, 화면에는 그냥 '손목캠이 안 나온다'
#      로 보인다. RViz 도 두 창이 겹쳐 어느 쪽을 보는지 알 수 없게 된다.
STACK_PATTERN='(^|/| )(arm_fsm|moveit_dynamixel_bridge|perception_node|move_group|robot_state_publisher|static_transform_publisher|wrist_camera|rviz2|detection_markers)( |$)'
CONFLICTS="$(pgrep -f "${CONFLICT_PATTERN}|${STACK_PATTERN}" 2>/dev/null | grep -vx "$$" || true)"
if [ -n "${CONFLICTS}" ]; then
  echo "[run_pick] 이미 떠 있는 텔레옵/픽 스택을 정리합니다: ${CONFLICTS}"
  # shellcheck disable=SC2086
  kill ${CONFLICTS} 2>/dev/null || true
  sleep 3
  # 카메라(RealSense)와 시리얼 포트를 확실히 놓을 때까지. 안 놓으면 새 스택이
  # 'device busy' 로 죽거나, 더 나쁘게는 위의 조용한 그리퍼 실패로 이어진다.
  LEFTOVER="$(pgrep -f "${CONFLICT_PATTERN}|${STACK_PATTERN}" 2>/dev/null | grep -vx "$$" || true)"
  if [ -n "${LEFTOVER}" ]; then
    echo "[run_pick] 아직 남은 프로세스에 KILL: ${LEFTOVER}"
    # shellcheck disable=SC2086
    kill -9 ${LEFTOVER} 2>/dev/null || true
    sleep 2
  fi
fi

LOG_FILE="/tmp/pick_launch_$$.log"
LAUNCH_PID=""

cleanup() {
  if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[run_pick] 스택 정리 중..."
    kill -- "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
  fi
  # launch 가 놓친 자식(유령)까지. 이 저장소에서 반복된 실패 모드다.
  # wrist_camera 를 넣는 이유: 얘가 살아남으면 /dev/video8 을 계속 잡아 **다음 기동의
  # 손목캠이 조용히 죽는다**(2026-08-19 에 실제로 걸렸다). rviz2 는 일부러 뺐다 —
  # 위 프로세스그룹 kill 이 이 launch 의 rviz 는 이미 정리하고, 여기 넣으면 사용자가
  # 따로 띄운 RViz 창까지 같이 죽는다.
  pgrep -f '(^|/| )(arm_fsm|moveit_dynamixel_bridge|perception_node|wrist_camera)( |$)' 2>/dev/null \
    | grep -vx "$$" | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[run_pick] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} / 워크스페이스=${WS_ROOT}"
echo "[run_pick] 픽 스택 백그라운드 기동 (log: ${LOG_FILE})"
cd "${WS_ROOT}"
ros2 launch dynamixel_control pick.launch.py "$@" > "${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

# perception 의 YOLO 로드 + RealSense 오픈 + move_group 기동까지 기다린다.
echo -n "[run_pick] 스택 기동 대기"
for _ in $(seq 40); do
  if grep -q "arm_fsm_node started" "${LOG_FILE}" 2>/dev/null \
     && grep -q "RealSense started\|test 모드" "${LOG_FILE}" 2>/dev/null; then
    break
  fi
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo; echo "[run_pick] 스택이 기동 중 종료됐습니다 — 로그: ${LOG_FILE}" >&2
    tail -20 "${LOG_FILE}" >&2
    exit 1
  fi
  echo -n "."
  sleep 1
done
echo " 준비됨"
echo "[run_pick] 스택 로그를 보려면 다른 터미널에서:  tail -f ${LOG_FILE}"
echo

exec ros2 run dynamixel_control mission_console
