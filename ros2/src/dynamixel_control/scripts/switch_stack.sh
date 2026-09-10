#!/usr/bin/env bash
# 팔 런타임을 두 모드 사이에서 바꾼다. `/dev/ttyUSB0` 를 하나만 잡을 수 있어
# 두 모드는 **배타적**이고, 매번 어느 프로세스를 죽여야 하는지 외울 일이 없게 만들었다.
#
#   teleop : 브라우저(8088)/키보드/게임패드로 사람이 직접 조종
#            GUI → /arm/teleop_jog → teleop_core → /dynamixel/goal_position → position_node
#   pick   : 비전 → 파지 자동 실행
#            perception → arm_fsm → /arm_controller/joint_trajectory → moveit_dynamixel_bridge
#
# 사용법 (컨테이너 안):
#   bash src/dynamixel_control/scripts/switch_stack.sh teleop
#   bash src/dynamixel_control/scripts/switch_stack.sh pick
#   bash src/dynamixel_control/scripts/switch_stack.sh status
#   bash src/dynamixel_control/scripts/switch_stack.sh stop     # 둘 다 내린다
#
# 관제 GUI(8088)는 건드리지 않는다 — 버스를 안 잡으므로 어느 모드에서도 그대로 둔다.
#
# ⚠️ 두 모드를 **동시에 띄우지 말 것.** 브릿지와 position_node 가 같은 시리얼 포트를
#    두드리면 `result=-3002`(COMM_RX_TIMEOUT)로 그리퍼 초기화만 조용히 실패하고
#    (팔 축은 우연히 성공) "팔은 가는데 그리퍼가 안 닫힌다" 로 나타난다. 먼저 뜬 쪽은
#    SerialException("multiple access on port")으로 죽는다 — 2026-08-12 실기에서 겪었다.
#    이 스크립트가 항상 반대쪽을 먼저 정리하는 이유다.
#
# ⚠️ teleop 모드의 /joint_states 는 **관절 도메인이 아니다.** position_node 는
#    `rad = (tick - 2048) * 2π/4096` 으로 환산하는데 실측 영점(641/207/2510/985)도
#    기어비(9.034/4.040)도 안 들어간다. 그래서 이 모드에서는 RViz/TF 와
#    `campose --from-gripper` 같은 FK 기반 도구를 믿으면 안 된다(그건 pick 모드의
#    브릿지가 발행하는 값이라야 맞다).

set -euo pipefail
set -m

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

MODE="${1:-status}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
else
  echo "[switch_stack] ${WS_ROOT}/install/setup.bash 이 없습니다 — 먼저 빌드하세요." >&2
  exit 1
fi
set -u

# 경계 앵커를 거는 이유는 run_keyboard_bench.sh 의 같은 상수 주석 참고 —
# 느슨하게 쓰면 이 스크립트 자신의 인자('teleop')까지 매칭해 스스로를 죽인다.
TELEOP_PAT='(^|/| )(teleop_core|position_node|dynamixel_position_node|joy_node|joystick_teleop|keyboard_teleop)( |$)'
PICK_PAT='(^|/| )(arm_fsm|moveit_dynamixel_bridge|perception_node|move_group|robot_state_publisher|static_transform_publisher|mission_console)( |$)'
LAUNCH_PAT='(^|/| )(teleop\.launch\.py|bench\.launch\.py|pick\.launch\.py)( |$)'

reap() {   # reap <pattern> <라벨>
  local pids
  pids="$(pgrep -f "$1" 2>/dev/null | grep -vx "$$" || true)"
  [ -z "${pids}" ] && return 0
  echo "[switch_stack] $2 정리: ${pids}"
  # shellcheck disable=SC2086
  kill ${pids} 2>/dev/null || true
  sleep 3
  pids="$(pgrep -f "$1" 2>/dev/null | grep -vx "$$" || true)"
  if [ -n "${pids}" ]; then
    echo "[switch_stack] 남은 프로세스에 KILL: ${pids}"
    # shellcheck disable=SC2086
    kill -9 ${pids} 2>/dev/null || true
    sleep 2   # RealSense/시리얼 포트를 완전히 놓을 때까지
  fi
}

show_status() {
  echo "[switch_stack] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  if pgrep -f "${PICK_PAT}" >/dev/null 2>&1; then
    echo "  현재 모드 : pick   (arm_fsm + moveit_dynamixel_bridge + perception)"
  elif pgrep -f "${TELEOP_PAT}" >/dev/null 2>&1; then
    echo "  현재 모드 : teleop (teleop_core + position_node)"
  else
    echo "  현재 모드 : (둘 다 안 떠 있음)"
  fi
  if pgrep -f '(^|/| )monitor( |$)' >/dev/null 2>&1; then
    echo "  관제 GUI  : 실행 중 (http://localhost:8088)"
  else
    echo "  관제 GUI  : 꺼짐 — bash src/robot_arm_gui/scripts/run_monitor.sh control:=true"
  fi
}

cd "${WS_ROOT}"

case "${MODE}" in
  teleop)
    reap "${PICK_PAT}|${LAUNCH_PAT}" "pick 스택"
    LOG=/tmp/teleop_stack.log
    echo "[switch_stack] teleop 모드 기동 (log: ${LOG})"
    nohup ros2 launch dynamixel_control teleop.launch.py use_hardware:=true \
      > "${LOG}" 2>&1 &
    for _ in $(seq 30); do
      grep -q "teleop_core started" "${LOG}" 2>/dev/null && break
      sleep 1
    done
    echo
    echo "  → 브라우저 http://localhost:8088 에서 '조종권 획득' 후 조작하세요."
    echo "     (GUI 가 꺼져 있으면: bash src/robot_arm_gui/scripts/run_monitor.sh control:=true)"
    echo "  → 이 모드에서는 pick 을 할 수 없습니다."
    ;;

  pick)
    reap "${TELEOP_PAT}|${LAUNCH_PAT}" "teleop 스택"
    LOG=/tmp/pick_stack.log
    echo "[switch_stack] pick 모드 기동 (log: ${LOG})"
    nohup ros2 launch dynamixel_control pick.launch.py carry_home:=true \
      > "${LOG}" 2>&1 &
    for _ in $(seq 45); do
      grep -q "arm_fsm_node started" "${LOG}" 2>/dev/null && break
      sleep 1
    done
    echo
    echo "  → 미션 지시는 콘솔에서: bash src/dynamixel_control/scripts/run_console.sh"
    echo "  → 브라우저는 관측 전용으로 쓰세요(조종권을 안 잡으면 충돌하지 않습니다)."
    ;;

  stop)
    reap "${PICK_PAT}|${TELEOP_PAT}|${LAUNCH_PAT}" "전체 스택"
    echo "[switch_stack] 정리 완료 — 버스가 비었습니다."
    echo "  ⚠️ 서보 토크는 종료 시 해제됩니다(브릿지 destroy_node). 팔이 처질 수 있습니다."
    ;;

  status) ;;

  *)
    echo "[switch_stack] 모르는 모드: ${MODE}  (teleop | pick | stop | status)" >&2
    exit 2
    ;;
esac

echo
show_status
