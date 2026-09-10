#!/usr/bin/env bash
# 키보드 벤치 텔레옵을 한 번에 띄우는 래퍼.
#
# keyboard_teleop_node(curses TUI)는 실제 stdin/tty 포커스가 필요해서 ROS 2 launch
# 안에 넣을 수 없다(keyboard_teleop_node.py 문서 참고). 그래서 이 스크립트는:
#   1) bench.launch.py(joy_node/joystick_teleop/teleop_core/position_node 등)를
#      백그라운드로 띄우고 출력은 로그 파일로 돌린다 (터미널에 섞이면 curses 화면이 깨짐).
#   2) keyboard_teleop 를 이 터미널의 foreground 에서 실행한다(진짜 tty 를 그대로 물려받음).
#   3) keyboard_teleop 가 끝나면(q 또는 Ctrl-C) trap 이 백그라운드 launch 를 통째로 정리한다.
#
# 반드시 컨테이너 안, 실제 tty 가 있는 터미널에서 실행할 것 (docker exec -it ...).
#
# 사용법
#   colcon build --packages-select dynamixel_control 후:
#   bash src/dynamixel_control/scripts/run_keyboard_bench.sh
#   # bench.launch.py 인자를 직접 넘기고 싶으면 그대로 전달된다:
#   bash src/dynamixel_control/scripts/run_keyboard_bench.sh use_hardware:=true rviz:=true
#
# 기본값(인자 없이 실행 시): use_hardware:=true joy_node:=false
#   (모터 연결된 벤치 기준 — 패드 없이 키보드만 쓸 거라 joy_node 는 꺼둔다)
#
# port 자동 감지: USB 재연결마다 /dev/ttyUSB 번호가 밀리는 게(0→1→2...) 잦은
# 실패 원인이었다 — port:= 를 직접 안 넘기면 현재 꽂혀있는 /dev/ttyUSB* 를 스캔해서
# 자동으로 넣는다. 여러 개가 꽂혀있으면(카메라 등 다른 FTDI 장치 포함) 뭐가 맞는지
# 알 수 없으므로 자동 감지를 포기하고 직접 port:=/dev/ttyUSBn 을 넘기라고 안내한다.

set -euo pipefail
set -m  # 잡 컨트롤 on — 백그라운드 launch 가 자기 pgid 를 가지게 해서 통째로 죽일 수 있게 한다

# 워크스페이스 오버레이(install/setup.bash)가 안 소싱된 셸에서 실행하면 아래
# bench.launch.py 가 "Package 'dynamixel_control' not found" 로 즉시 죽는데, 3초
# 대기 로직 때문에 "이미 종료됐습니다" 라는 원인을 알 수 없는 메시지만 보이고
# 로그를 직접 열어봐야 진짜 이유를 알 수 있었다(2026-08-02 실사용 중 발견 — `docker
# exec -it` 로 새로 붙은 셸에 .bashrc 로 /opt/ros/humble 만 소싱되고 워크스페이스
# install/ 은 매번 수동으로 source 해야 하는데 빼먹기 쉬움). 이 스크립트가 스스로
# 감지해서 필요하면 소싱하고, 그래도 안 되면(빌드 자체가 안 된 경우) 이유를 바로 알려준다.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if ! ros2 pkg prefix dynamixel_control >/dev/null 2>&1; then
  if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    echo "[run_keyboard_bench] 워크스페이스 오버레이 미소싱 감지 → 자동 소싱: ${WS_ROOT}/install/setup.bash"
    # colcon 이 생성하는 setup.bash 는 set -u(nounset) 안전하게 짜여있지 않다
    # (예: COLCON_TRACE 를 기본값 없이 참조) — 소싱하는 동안만 -u 를 꺼서
    # "unbound variable" 로 이 스크립트 자체가 죽는 걸 막는다.
    set +u
    # shellcheck disable=SC1091
    source "${WS_ROOT}/install/setup.bash"
    set -u
  fi
fi

if ! ros2 pkg prefix dynamixel_control >/dev/null 2>&1; then
  echo "[run_keyboard_bench] dynamixel_control 패키지를 찾을 수 없습니다." >&2
  echo "[run_keyboard_bench] ${WS_ROOT} 에서 'colcon build --packages-select dynamixel_control' 을 먼저 실행하세요." >&2
  exit 1
fi

# ros2 launch/ros2 run 은 wrapper 프로세스만 죽고 실제 노드 자식 프로세스는 wrapper와
# 다른 pgid 에 남는 경우가 있다(CLAUDE.md "Watch out for" 참고 — kill/Ctrl-C 가 wrapper만
# 죽이고 python 노드는 유령으로 계속 돈다). pgid 신호 하나만 믿지 않고, 알려진 노드
# 실행 파일 이름으로 확실히 정리한다 — 시작 전(이전 실행이 남긴 유령 정리)과 종료 시
# (이번 실행이 유령을 안 남기게) 둘 다에서 쓴다.
#
# ⚠️ 이 패턴을 느슨하게 "키워드 그대로" 쓰면(예: 'joy_node') 자기 자신을 죽이는
# 사고가 난다 — 실사용 중 실제로 재현됨(2026-08-02): `run_keyboard_bench.sh
# joy_node:=false` 처럼 이 스크립트 자신의 인자에 'joy_node' 문자열이 그대로
# 들어있으면, pkill -f 가 그 인자를 가진 **이 스크립트 프로세스 자신**까지
# 매칭해서 SIGTERM 을 보내버린다 → "이전 실행이 남긴 프로세스 정리 중..." 에서
# 조용히 멈추고 다시는 실행이 안 되는 원인이었다. 그래서 경계 앵커(/ 또는
# 공백/문자열 끝)를 걸어 "joy_node:=false" 같은 launch 인자는 안 걸리고 실제
# 실행 파일 경로(.../dynamixel_control/joy_node, 뒤에 공백+--ros-args 등)만
# 걸리게 했다. 그래도 혹시 몰라 이 스크립트 자신의 PID($$)는 명시적으로도 제외한다.
NODE_NAME_PATTERN='(^|/| )(joy_node|joystick_teleop|teleop_core|position_node|dynamixel_position_node|keyboard_teleop|robot_state_publisher|rviz2)( |$)'

reap_ghosts() {
  local pids
  pids="$(pgrep -f "${NODE_NAME_PATTERN}" 2>/dev/null | grep -vx "$$" || true)"
  if [ -n "${pids}" ]; then
    # shellcheck disable=SC2086
    kill ${pids} 2>/dev/null || true
  fi
}

echo "[run_keyboard_bench] 이전 실행이 남긴 프로세스 정리 중..."
reap_ghosts
sleep 0.5

LAUNCH_ARGS=("$@")
if [ ${#LAUNCH_ARGS[@]} -eq 0 ]; then
  LAUNCH_ARGS=(use_hardware:=true joy_node:=false)
fi

if ! printf '%s\n' "${LAUNCH_ARGS[@]}" | grep -q '^port:='; then
  mapfile -t TTY_CANDIDATES < <(ls /dev/ttyUSB* 2>/dev/null || true)
  if [ ${#TTY_CANDIDATES[@]} -eq 1 ]; then
    echo "[run_keyboard_bench] port 미지정 → 자동 감지: ${TTY_CANDIDATES[0]}"
    LAUNCH_ARGS+=("port:=${TTY_CANDIDATES[0]}")
  elif [ ${#TTY_CANDIDATES[@]} -eq 0 ]; then
    echo "[run_keyboard_bench] /dev/ttyUSB* 가 하나도 없습니다 — USB 연결을 확인하세요." >&2
    exit 1
  else
    echo "[run_keyboard_bench] /dev/ttyUSB* 가 여러 개(${TTY_CANDIDATES[*]}) 라 자동 감지를 포기합니다." >&2
    echo "[run_keyboard_bench] port:=/dev/ttyUSBn 을 직접 지정해서 다시 실행하세요." >&2
    exit 1
  fi
fi

LOG_FILE="/tmp/bench_launch_$$.log"
echo "[run_keyboard_bench] bench.launch.py 백그라운드 기동 (log: ${LOG_FILE})"
echo "[run_keyboard_bench] launch args: ${LAUNCH_ARGS[*]}"

ros2 launch dynamixel_control bench.launch.py "${LAUNCH_ARGS[@]}" > "${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  echo
  echo "[run_keyboard_bench] bench.launch.py 종료 중... (pgid ${LAUNCH_PID})"
  kill -INT -"${LAUNCH_PID}" 2>/dev/null || true

  # 정상 종료(SIGINT 전파) 최대 5초 대기
  for _ in $(seq 1 10); do
    kill -0 "${LAUNCH_PID}" 2>/dev/null || break
    sleep 0.5
  done

  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[run_keyboard_bench] 정상 종료 실패 — 프로세스 그룹 강제 종료(SIGKILL)"
    kill -KILL -"${LAUNCH_PID}" 2>/dev/null || true
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true

  # pgid 신호가 못 닿은 유령이 있을 수 있으니 이름으로 한 번 더 확실히 정리 —
  # 이걸 해야 바로 다음 실행이 포트/노드이름 충돌 없이 깨끗하게 시작된다.
  reap_ghosts

  echo "[run_keyboard_bench] 종료 완료. 로그: ${LOG_FILE}"
}
trap cleanup EXIT INT TERM

echo "[run_keyboard_bench] 하드웨어 브릿지 기동 대기 중... (3초)"
sleep 3

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  echo "[run_keyboard_bench] bench.launch.py 가 이미 종료됐습니다 — ${LOG_FILE} 확인하세요." >&2
  exit 1
fi

echo "[run_keyboard_bench] keyboard_teleop 시작 (q 또는 Ctrl-C 로 전체 종료)"
echo
ros2 run dynamixel_control keyboard_teleop
