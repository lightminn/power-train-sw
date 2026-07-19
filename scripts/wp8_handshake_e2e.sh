#!/usr/bin/env bash
# WP8 실팔 핸드셰이크 E2E 하네스
#
# 사용법:
#   bash scripts/wp8_handshake_e2e.sh [--phase1-only|--phase2-only]
#       [--negative-control] [--timeout SEC]
#
# 협조 세션 실서보 재실행:
#   1. 팔팀 입회 아래 실서보 전원·작업공간 안전을 확인한다.
#   2. 팔 설치본을 기본 파라미터 그대로 준비하고 이 스크립트를 실행한다.
#   3. Phase 1 로그에서 CARRYING_LOCKED와 STOWED_LOCKED 도달 증적을 보존한다.
#   4. 같은 명령을 한 번 더 실행해 멱등성을 확인한 뒤 음성 대조를 실행한다.

set -euo pipefail

TIMEOUT=8
RUN_PHASE1=1
RUN_PHASE2=1
NEGATIVE_CONTROL=0

usage() {
  sed -n '2,12p' "$0"
}

while (( $# > 0 )); do
  case "$1" in
    --phase1-only)
      RUN_PHASE1=1
      RUN_PHASE2=0
      shift
      ;;
    --phase2-only)
      RUN_PHASE1=0
      RUN_PHASE2=1
      shift
      ;;
    --negative-control)
      NEGATIVE_CONTROL=1
      shift
      ;;
    --timeout)
      if (( $# < 2 )); then
        echo "오류: --timeout 값이 필요합니다."
        exit 1
      fi
      TIMEOUT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "오류: 알 수 없는 인자: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ ! "$TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "오류: --timeout은 양의 초 단위 숫자여야 합니다."
  exit 1
fi
timeout_nonzero="${TIMEOUT//[.0]/}"
if [[ -z "$timeout_nonzero" ]]; then
  echo "오류: --timeout은 0보다 커야 합니다."
  exit 1
fi

RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_ROOT="${WP8_E2E_LOG_ROOT:-/tmp}"
LOG_DIR="${LOG_ROOT}/wp8_e2e_${RUN_STAMP}"
RUN_PREFIX="/tmp/wp8_e2e_${RUN_STAMP}_$$"
CHASSIS_PID_FILE="${RUN_PREFIX}_chassis.pgid"
ARM_PID_FILE="${RUN_PREFIX}_arm.pgid"
RESUME_T_FILE="${RUN_PREFIX}_resume_t"
mkdir -p "$LOG_DIR"

CHASSIS_PGID=""
ARM_PGID=""
CHASSIS_ACTIVE=0
ARM_ACTIVE=0
PHASE1_SKIPPED=0
SUMMARY_ROWS=()

add_result() {
  SUMMARY_ROWS+=("$1|$2|$3")
}

container_running() {
  local value
  value="$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null || true)"
  [[ "$value" == "true" ]]
}

wait_for_pid_file() {
  local container="$1"
  local path="$2"
  local end_s=$((SECONDS + ${TIMEOUT%%.*} + 1))
  while (( SECONDS <= end_s )); do
    if docker exec "$container" test -s "$path" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

check_group_gone() {
  local container="$1"
  local pgid="$2"
  local label="$3"
  if [[ -z "$pgid" ]]; then
    return 0
  fi
  if docker exec "$container" bash -lc \
      "kill -0 -- -${pgid} >/dev/null 2>&1" >/dev/null 2>&1; then
    echo "잔류 검사: ${label} PGID ${pgid} 잔류"
  else
    echo "잔류 검사: ${label} PGID ${pgid} 없음"
  fi
}

stop_chassis() {
  if (( CHASSIS_ACTIVE == 0 )); then
    return 0
  fi
  docker exec powertrain_ros pkill -TERM -g "$CHASSIS_PGID" >/dev/null 2>&1 || true
  sleep 0.2
  docker exec powertrain_ros pkill -KILL -g "$CHASSIS_PGID" >/dev/null 2>&1 || true
  check_group_gone powertrain_ros "$CHASSIS_PGID" chassis
  CHASSIS_ACTIVE=0
}

stop_arm() {
  if (( ARM_ACTIVE == 0 )); then
    return 0
  fi
  docker exec ros2_humble pkill -TERM -g "$ARM_PGID" >/dev/null 2>&1 || true
  sleep 0.2
  docker exec ros2_humble pkill -KILL -g "$ARM_PGID" >/dev/null 2>&1 || true
  check_group_gone ros2_humble "$ARM_PGID" arm_fsm
  ARM_ACTIVE=0
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  set +e
  stop_arm
  stop_chassis
  echo "로그 디렉터리: $LOG_DIR"
  exit "$rc"
}
trap cleanup EXIT INT TERM

start_chassis() {
  local command
  command="source /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash && export ROS_DOMAIN_ID=77 && exec setsid bash -lc 'echo \$\$ > ${CHASSIS_PID_FILE}; exec ros2 run powertrain_ros chassis --ros-args -p fake:=true -p safety_required:=false -p contract_v2_verified:=true -p mission_contract_owner:=chassis_supervisor -p mission_id_path:=/tmp/wp8_mission_id'"
  if ! docker exec -d powertrain_ros bash -lc "$command"; then
    echo "오류: chassis 기동 실패"
    return 1
  fi
  if ! wait_for_pid_file powertrain_ros "$CHASSIS_PID_FILE"; then
    echo "오류: chassis PGID 기록 시간 초과"
    return 1
  fi
  CHASSIS_PGID="$(docker exec powertrain_ros cat "$CHASSIS_PID_FILE")"
  if [[ ! "$CHASSIS_PGID" =~ ^[0-9]+$ ]]; then
    echo "오류: 잘못된 chassis PGID: $CHASSIS_PGID"
    return 1
  fi
  CHASSIS_ACTIVE=1

  local topic_info=""
  local end_s=$((SECONDS + ${TIMEOUT%%.*} + 1))
  while (( SECONDS <= end_s )); do
    topic_info="$(docker exec powertrain_ros bash -lc \
      'source /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash && export ROS_DOMAIN_ID=77 && topic=$(python3 -c "from powertrain_ros import contract; print(contract.TOPIC_ARRIVAL)") && ros2 topic info "$topic"' 2>&1 || true)"
    if [[ "$topic_info" == *"Publisher count: 1"* ]]; then
      echo "chassis 준비 완료: /arrival_status 발행자 1"
      return 0
    fi
    sleep 0.1
  done
  echo "오류: /arrival_status 발행자 수가 1이 아님"
  echo "$topic_info"
  return 1
}

start_arm() {
  local executables
  executables="$(docker exec ros2_humble bash -lc \
    'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 pkg executables dynamixel_control' 2>&1)" || {
      echo "오류: 팔 실행 파일 조회 실패"
      return 1
    }
  if [[ "$executables" != *"dynamixel_control arm_fsm"* ]]; then
    echo "오류: dynamixel_control arm_fsm 실행 파일 없음"
    return 1
  fi

  local command
  command="source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=77 && exec setsid bash -lc 'echo \$\$ > ${ARM_PID_FILE}; exec ros2 run dynamixel_control arm_fsm'"
  if ! docker exec -d ros2_humble bash -lc "$command"; then
    echo "오류: arm_fsm 기동 실패"
    return 1
  fi
  if ! wait_for_pid_file ros2_humble "$ARM_PID_FILE"; then
    echo "오류: arm_fsm PGID 기록 시간 초과"
    return 1
  fi
  ARM_PGID="$(docker exec ros2_humble cat "$ARM_PID_FILE")"
  if [[ ! "$ARM_PGID" =~ ^[0-9]+$ ]]; then
    echo "오류: 잘못된 arm_fsm PGID: $ARM_PGID"
    return 1
  fi
  ARM_ACTIVE=1
}

run_probe() {
  local label="$1"
  local subcommand="$2"
  local observe_only="$3"
  shift 3
  local container_json="${RUN_PREFIX}_${label}.json"
  local host_json="${LOG_DIR}/${label}.json"
  local host_log="${LOG_DIR}/${label}.log"
  local command="source /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash && export ROS_DOMAIN_ID=77 &&"
  if (( observe_only == 1 )); then
    command+=" WP8_PROBE_OBSERVE_ONLY=1"
  fi
  command+=" ros2 run powertrain_ros wp8_handshake_probe"

  local argument
  local quoted
  local arguments=("$subcommand" "--timeout-s" "$TIMEOUT" "--json" "$container_json" "$@")
  for argument in "${arguments[@]}"; do
    printf -v quoted '%q' "$argument"
    command+=" $quoted"
  done

  docker exec powertrain_ros bash -lc "$command" 2>&1 | tee "$host_log"
  local rc=${PIPESTATUS[0]}
  docker cp "powertrain_ros:${container_json}" "$host_json" >/dev/null 2>&1 || true
  return "$rc"
}

print_summary() {
  echo
  echo "WP8 핸드셰이크 E2E 요약"
  echo "단계 | 결과 | 상세"
  echo "--- | --- | ---"
  local row
  for row in "${SUMMARY_ROWS[@]}"; do
    IFS='|' read -r step result detail <<<"$row"
    echo "$step | $result | $detail"
  done
}

if ! container_running powertrain_ros; then
  echo "오류: powertrain_ros 컨테이너가 running 상태가 아닙니다."
  exit 1
fi

if (( NEGATIVE_CONTROL == 1 )); then
  start_chassis || exit 1
  set +e
  run_probe negative_control pickup 1
  negative_rc=$?
  set -e
  stop_chassis
  if (( negative_rc == 1 )); then
    add_result "음성 대조" "PASS" "pickup 무자극 판정을 probe가 거부"
    echo "음성 대조 OK: probe FAIL(exit 1) 확인"
    print_summary
    exit 0
  fi
  if (( negative_rc == 0 )); then
    add_result "음성 대조" "FAIL" "살아있는 chassis 무자극 관측이 PASS — 게이트 불량"
  else
    add_result "음성 대조" "FAIL" "기대 exit 1, 실제 exit ${negative_rc}"
  fi
  print_summary
  exit 1
fi

if (( RUN_PHASE1 == 1 )); then
  if ! container_running ros2_humble; then
    PHASE1_SKIPPED=1
    add_result "Phase 1" "SKIP" "ros2_humble 부재"
    echo "Phase 1 SKIP: ros2_humble 컨테이너 부재"
  else
    start_chassis || exit 1
    start_arm || exit 1
    run_probe phase1_baseline baseline 0 || exit 1
    run_probe phase1_pickup pickup 0 || exit 1

    phase1_branch="$(
      python3 -c \
        'import json, sys; print(json.load(open(sys.argv[1], encoding="utf-8"))["branch"])' \
        "${LOG_DIR}/phase1_pickup.json"
    )"
    if [[ "$phase1_branch" == "fail_closed" ]]; then
      add_result "Phase 1" "PASS" "fail-closed — headless 잠금자세 게이트, 스펙 §3-7 한계"
      echo "Phase 1 PASS(fail-closed — headless 잠금자세 게이트, 스펙 §3-7 한계)"
    elif [[ "$phase1_branch" == "work_accepted" ]]; then
      # -f 부분일치는 같은 컨테이너의 운용(도메인 0) chassis_telemetry cmdline에도 걸린다.
      docker exec powertrain_ros pkill -STOP -g "$CHASSIS_PGID" || exit 1
      sleep 1.5
      docker exec powertrain_ros pkill -CONT -g "$CHASSIS_PGID" || exit 1
      docker exec powertrain_ros bash -lc \
        "python3 -c 'import time; print(time.monotonic())' > ${RESUME_T_FILE}" || exit 1
      run_probe phase1_resume resume 0 --resume-t-file "$RESUME_T_FILE" || exit 1
      add_result "Phase 1" "PASS" "baseline→pickup→watchdog resume"
    else
      echo "오류: 알 수 없는 Phase 1 pickup branch: $phase1_branch"
      exit 1
    fi
    stop_arm
    stop_chassis
  fi
fi

if (( RUN_PHASE2 == 1 )); then
  start_chassis || exit 1
  run_probe phase2_full_cycle full-cycle 0 || exit 1
  stop_chassis

  for scenario in no_response late_done failed_latch dup_done; do
    start_chassis || exit 1
    run_probe "phase2_fault_${scenario}" fault 0 --scenario "$scenario" || exit 1
    stop_chassis
  done
  add_result "Phase 2" "PASS" "full-cycle + fault 4종"
fi

print_summary
if (( PHASE1_SKIPPED == 1 )); then
  exit 3
fi
exit 0
