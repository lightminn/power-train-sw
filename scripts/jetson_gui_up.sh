#!/usr/bin/env bash
# 젯슨 호스트에서 운용 콘솔 의존 스택을 한 번에 기동한다.
# 사용 예 (운영 PC에서):
#   ssh zetin@<jetson> 'bash ~/power-train-sw/scripts/jetson_gui_up.sh'
# 옵션:
#   --fresh              파워트레인 컨테이너를 강제로 재생성
#   --no-arm             로봇팔 compose와 perception/stream 기동 생략
#   --operator-host IP   운용 PC IPv4 주소 지정
#   --timeout SEC        파워트레인 헬스 대기 시간(기본 420초)

set -uo pipefail

usage() {
  cat <<'EOF'
사용법: bash scripts/jetson_gui_up.sh [옵션]

  --fresh              compose에 --force-recreate 추가
  --no-arm             로봇팔 스택 기동 생략
  --operator-host IP   운용 PC IPv4 주소
  --timeout SEC        헬스 대기 제한(기본 420)
  -h, --help            도움말
EOF
}

die_usage() {
  printf '오류: %s\n\n' "$*" >&2
  usage >&2
  exit 64
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" \
  || { printf '스크립트 경로를 확인할 수 없습니다.\n' >&2; exit 1; }
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)" \
  || { printf '레포 루트를 확인할 수 없습니다.\n' >&2; exit 1; }
cd "$REPO_ROOT" \
  || { printf '레포 루트로 이동할 수 없습니다: %s\n' "$REPO_ROOT" >&2; exit 1; }

FRESH=0
NO_ARM=0
CLI_OPERATOR_HOST=""
TIMEOUT=420

while (( $# > 0 )); do
  case "$1" in
    --fresh)
      FRESH=1
      shift
      ;;
    --no-arm)
      NO_ARM=1
      shift
      ;;
    --operator-host)
      (( $# >= 2 )) || die_usage '--operator-host 뒤에 IP가 필요합니다.'
      CLI_OPERATOR_HOST="$2"
      shift 2
      ;;
    --timeout)
      (( $# >= 2 )) || die_usage '--timeout 뒤에 초가 필요합니다.'
      TIMEOUT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die_usage "알 수 없는 인자: $1"
      ;;
  esac
done

[[ "$TIMEOUT" =~ ^[0-9]+$ ]] && (( TIMEOUT > 0 )) \
  || die_usage '--timeout은 1 이상의 정수여야 합니다.'
if [ -n "$CLI_OPERATOR_HOST" ] \
  && ! [[ "$CLI_OPERATOR_HOST" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
  die_usage '--operator-host는 IPv4 형식이어야 합니다.'
fi

POLL_S="${GUI_UP_POLL_S:-5}"
[[ "$POLL_S" =~ ^[0-9]+([.][0-9]+)?$ ]] \
  || die_usage 'GUI_UP_POLL_S는 0 이상의 숫자여야 합니다.'
ARM_REPO="${ARM_REPO:-${HOME:-}/extreme-robot}"

declare -a RESULT_ICONS=()
declare -a RESULT_NAMES=()
declare -a RESULT_DETAILS=()
CRITICAL_FAILURES=0
NON_GREEN_RESULTS=0

add_result() {
  local icon="$1"
  local name="$2"
  local detail="$3"
  local critical="${4:-0}"
  detail="${detail//$'\n'/ | }"
  RESULT_ICONS+=("$icon")
  RESULT_NAMES+=("$name")
  RESULT_DETAILS+=("$detail")
  if [ "$icon" != '✅' ]; then
    (( NON_GREEN_RESULTS += 1 ))
  fi
  if [ "$icon" = '❌' ] && [ "$critical" = 1 ]; then
    (( CRITICAL_FAILURES += 1 ))
  fi
}

banner() {
  printf '\n========== %s ==========\n' "$1"
}

can0_is_ready() {
  local details
  details="$(ip -details link show can0 2>/dev/null)" || return 1
  [[ "$details" == *'state UP'* && "$details" == *'bitrate 500000'* ]]
}

read_operator_host() {
  local environment_file="$1"
  awk -F= '$1 == "OPERATOR_HOST" { print substr($0, index($0, "=") + 1); exit }' \
    "$environment_file" 2>/dev/null
}

wait_for_active_unit() {
  local unit="$1"
  local limit_s="$2"
  local deadline
  local status
  deadline=$(( $(date +%s) + limit_s ))
  while :; do
    status="$(systemctl is-active "$unit" 2>/dev/null || true)"
    [ "$status" = active ] && return 0
    (( $(date +%s) >= deadline )) && return 1
    sleep "$POLL_S"
  done
}

port_is_listening() {
  local listing="$1"
  local port="$2"
  printf '%s\n' "$listing" | grep -Eq "(^|[^0-9])${port}([^0-9]|$)"
}

# SRT 송신기(srtsink listener)는 첫 프레임 이후에야 뜨므로(카메라 워밍업 수 초)
# UDP 포트는 짧게 폴링해서 판정한다.
wait_udp_listen() {
  local port="$1"
  local limit_s="$2"
  local deadline
  deadline=$(( $(date +%s) + limit_s ))
  while :; do
    if port_is_listening "$(ss -uln 2>/dev/null || true)" "$port"; then
      return 0
    fi
    (( $(date +%s) >= deadline )) && return 1
    sleep "$POLL_S"
  done
}

banner '1/7 호스트 준비'

if can0_is_ready; then
  add_result '✅' 'can0' 'UP / 500000 bps' 1
else
  printf 'can0가 준비되지 않아 sudo -n으로 설정을 시도합니다.\n'
  sudo -n bash scripts/can_setup.sh
  can_setup_rc=$?
  if [ "$can_setup_rc" -eq 0 ] && can0_is_ready; then
    add_result '✅' 'can0' '자동 복구됨: UP / 500000 bps' 1
  else
    printf '수동: sudo bash scripts/can_setup.sh\n'
    add_result '❌' 'can0' '자동 설정 실패 — 수동: sudo bash scripts/can_setup.sh' 1
  fi
fi

if [ -d /run/powertrain ] && [ -d /var/lib/powertrain ]; then
  add_result '✅' '런타임 디렉터리' '/run/powertrain, /var/lib/powertrain 존재'
else
  printf '파워트레인 런타임 디렉터리 설치를 시도합니다.\n'
  sudo -n bash scripts/install_powertrain_runtime_dir.sh
  runtime_install_rc=$?
  if [ "$runtime_install_rc" -eq 0 ] \
    && [ -d /run/powertrain ] && [ -d /var/lib/powertrain ]; then
    add_result '✅' '런타임 디렉터리' '자동 설치 완료'
  else
    printf '수동: sudo bash scripts/install_powertrain_runtime_dir.sh\n'
    add_result '❌' '런타임 디렉터리' \
      '자동 설치 실패 — 수동: sudo bash scripts/install_powertrain_runtime_dir.sh'
  fi
fi

if [ -e /etc/powertrain/powertrain.env ]; then
  add_result '✅' 'powertrain.env' '존재(내용 미확인)' 1
else
  add_result '❌' 'powertrain.env' \
    '없음 — powertrain_chassis 기동 거부 사유' 1
fi
if [ -e /etc/powertrain/ops_console.token ]; then
  add_result '✅' 'ops_console.token' '존재(내용 미확인)'
else
  add_result '❌' 'ops_console.token' '없음 — ops 인증 토큰을 프로비저닝해야 함'
fi

existing_operator_host=""
if [ -e /etc/default/powertrain-chassis-telemetry ]; then
  existing_operator_host="$(
    read_operator_host /etc/default/powertrain-chassis-telemetry || true
  )"
fi

OPERATOR_HOST=""
if [ -n "$CLI_OPERATOR_HOST" ]; then
  OPERATOR_HOST="$CLI_OPERATOR_HOST"
  if [ -n "$existing_operator_host" ] \
    && [ "$CLI_OPERATOR_HOST" != "$existing_operator_host" ]; then
    printf 'OPERATOR_HOST 변경을 두 텔레메트리 환경 파일에 반영합니다.\n'
    sudo -n bash -c '
      set -euo pipefail
      new_host="$1"
      for environment_file in \
        /etc/default/powertrain-chassis-telemetry \
        /etc/default/powertrain-pdist80b-telemetry; do
        [ -f "$environment_file" ]
        grep -q "^OPERATOR_HOST=" "$environment_file"
        sed -i -E "s|^OPERATOR_HOST=.*$|OPERATOR_HOST=$new_host|" "$environment_file"
      done
    ' bash "$CLI_OPERATOR_HOST"
    operator_update_rc=$?
    if [ "$operator_update_rc" -eq 0 ]; then
      sudo -n systemctl restart \
        powertrain-chassis-telemetry.service \
        powertrain-pdist80b-telemetry.service
      operator_restart_rc=$?
    else
      operator_restart_rc=1
    fi
    if [ "$operator_update_rc" -eq 0 ] && [ "$operator_restart_rc" -eq 0 ]; then
      add_result '✅' 'OPERATOR_HOST' "$OPERATOR_HOST로 갱신하고 유닛 재시작"
    else
      OPERATOR_HOST="$existing_operator_host"
      printf '⚠️ sudo 자동 변경 실패. 기존 값 %s로 계속합니다.\n' "$OPERATOR_HOST"
      printf '수동 재설치: sudo bash scripts/install_chassis_telemetry_service.sh %s\n' \
        "$CLI_OPERATOR_HOST"
      add_result '⚠️' 'OPERATOR_HOST' \
        "자동 변경 실패 — 기존 값 $OPERATOR_HOST 사용; sudo bash scripts/install_chassis_telemetry_service.sh $CLI_OPERATOR_HOST"
    fi
  else
    add_result '✅' 'OPERATOR_HOST' "$OPERATOR_HOST (--operator-host)"
  fi
elif [ -n "$existing_operator_host" ]; then
  OPERATOR_HOST="$existing_operator_host"
  add_result '✅' 'OPERATOR_HOST' "$OPERATOR_HOST (설치된 텔레메트리 설정)"
else
  printf '⚠️ OPERATOR_HOST를 결정할 수 없어 브리지 기동을 건너뜁니다.\n'
  add_result '⚠️' 'OPERATOR_HOST' '미확정 — arm_console_bridge 기동 스킵'
fi

if [ -n "$OPERATOR_HOST" ] \
  && ! [[ "$OPERATOR_HOST" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
  printf '⚠️ 설치된 OPERATOR_HOST가 IPv4 형식이 아니어서 브리지 기동을 건너뜁니다.\n'
  OPERATOR_HOST=""
  add_result '⚠️' 'OPERATOR_HOST 형식' 'IPv4 형식 아님 — 브리지 기동 스킵'
fi

banner '2/7 파워트레인 compose'

powertrain_services=(
  canwatchdog
  powertrain_ros
  powertrain_control
  powertrain_chassis
  powertrain_observability
)
compose_command=(
  docker compose -f docker/docker-compose.jetson.yml up -d
)
if [ "$FRESH" -eq 1 ]; then
  compose_command+=(--force-recreate)
fi
compose_command+=("${powertrain_services[@]}")

# 서비스명 없는 bare `up -d`는 금지한다. 미프로비저닝 fail-closed 서비스와
# autonomy 프로파일 함정 때문에 GUI에 필요한 정확한 5종만 명시적으로 올린다.
"${compose_command[@]}"
compose_up_rc=$?
if [ "$compose_up_rc" -ne 0 ]; then
  add_result '❌' '파워트레인 compose' \
    "up 명령 실패(exit $compose_up_rc)" 1
fi

banner '3/7 파워트레인 헬스 대기'

health_containers=(
  powertrain_ros
  powertrain_control
  powertrain_chassis
  powertrain_observability
)
declare -A health_done=()
declare -A last_health=()
completed_containers=0
powertrain_ros_healthy=0
health_deadline=$(( $(date +%s) + TIMEOUT ))

while (( completed_containers < 5 )); do
  for container in "${health_containers[@]}"; do
    [ "${health_done[$container]:-0}" -eq 1 ] && continue
    state="$(docker inspect -f '{{.State.Status}}' "$container" 2>/dev/null || true)"
    if [ "$state" = exited ] || [ "$state" = dead ]; then
      log_tail="$(docker logs --tail 5 "$container" 2>&1 || true)"
      printf '%s 최근 로그(5줄):\n%s\n' "$container" "$log_tail"
      detail="$state; 최근 로그: ${log_tail//$'\n'/ | }"
      if [ "$container" = powertrain_ros ]; then
        detail="$detail; L515 미연결 가능성 (lsusb 8086:0b64 확인)"
        printf '힌트: L515 미연결 가능성 (lsusb 8086:0b64 확인)\n'
      fi
      add_result '❌' "$container" "$detail" 1
      health_done[$container]=1
      (( completed_containers += 1 ))
      continue
    fi
    health="$(
      docker inspect -f '{{.State.Health.Status}}' "$container" 2>/dev/null || true
    )"
    last_health[$container]="${health:-unknown}"
    if [ "$health" = healthy ]; then
      add_result '✅' "$container" 'healthy' 1
      health_done[$container]=1
      (( completed_containers += 1 ))
      if [ "$container" = powertrain_ros ]; then
        powertrain_ros_healthy=1
      fi
    fi
  done

  if [ "${health_done[canwatchdog]:-0}" -ne 1 ]; then
    watchdog_state="$(
      docker inspect -f '{{.State.Status}}' powertrain_canwatchdog 2>/dev/null || true
    )"
    if [ "$watchdog_state" = exited ] || [ "$watchdog_state" = dead ]; then
      log_tail="$(docker logs --tail 5 powertrain_canwatchdog 2>&1 || true)"
      printf 'powertrain_canwatchdog 최근 로그(5줄):\n%s\n' "$log_tail"
      add_result '❌' 'canwatchdog' \
        "$watchdog_state; 최근 로그: ${log_tail//$'\n'/ | }" 1
      health_done[canwatchdog]=1
      (( completed_containers += 1 ))
    else
      watchdog_running="$(
        docker inspect -f '{{.State.Running}}' powertrain_canwatchdog \
          2>/dev/null || true
      )"
      last_health[canwatchdog]="running=${watchdog_running:-unknown}"
      if [ "$watchdog_running" = true ]; then
        add_result '✅' 'canwatchdog' 'running=true' 1
        health_done[canwatchdog]=1
        (( completed_containers += 1 ))
      fi
    fi
  fi

  (( completed_containers >= 5 )) && break
  (( $(date +%s) >= health_deadline )) && break
  sleep "$POLL_S"
done

for container in "${health_containers[@]}"; do
  if [ "${health_done[$container]:-0}" -ne 1 ]; then
    detail="타임아웃(${TIMEOUT}s), 현재 health=${last_health[$container]:-unknown}"
    if [ "$container" = powertrain_ros ]; then
      detail="$detail; L515 미연결 가능성 (lsusb 8086:0b64 확인)"
      printf '힌트: L515 미연결 가능성 (lsusb 8086:0b64 확인)\n'
    fi
    add_result '❌' "$container" "$detail" 1
  fi
done
if [ "${health_done[canwatchdog]:-0}" -ne 1 ]; then
  add_result '❌' 'canwatchdog' \
    "타임아웃(${TIMEOUT}s), 현재 ${last_health[canwatchdog]:-unknown}" 1
fi

banner '4/7 텔레메트리 유닛과 장치'

# Restart=always/3s가 powertrain_ros 회복 뒤 스스로 재기동하므로 restart하지 않는다.
if wait_for_active_unit powertrain-chassis-telemetry.service 30; then
  add_result '✅' 'powertrain-chassis-telemetry' 'active' 1
else
  printf '수동: sudo systemctl restart powertrain-chassis-telemetry.service\n'
  add_result '❌' 'powertrain-chassis-telemetry' \
    '비active — sudo systemctl restart powertrain-chassis-telemetry.service' 1
fi

if [ "$(systemctl is-active powertrain-pdist80b-telemetry.service 2>/dev/null || true)" \
  = active ]; then
  add_result '✅' 'powertrain-pdist80b-telemetry' 'active' 1
else
  printf '수동: sudo systemctl restart powertrain-pdist80b-telemetry.service\n'
  add_result '❌' 'powertrain-pdist80b-telemetry' \
    '비active — sudo systemctl restart powertrain-pdist80b-telemetry.service' 1
fi

if [ "$(systemctl is-active powertrain-bringup-preflight.service 2>/dev/null || true)" \
  = active ]; then
  add_result '✅' 'powertrain-bringup-preflight' 'active'
else
  add_result '❌' 'powertrain-bringup-preflight' '비active — 부팅 전제조건 유닛 확인 필요'
fi

usb_devices="$(lsusb 2>/dev/null || true)"
if [[ "$usb_devices" == *'8086:0b64'* ]]; then
  add_result '✅' 'L515' 'USB 8086:0b64 감지'
else
  add_result '⚠️' 'L515' '미감지 — 케이블/전원 확인'
fi

d435_present=0
if printf '%s\n' "$usb_devices" | grep -Eqi '8086:0b3[0-9a-f]'; then
  d435_present=1
  add_result '✅' 'D435i' 'USB 8086:0b3x 감지'
else
  add_result '⚠️' 'D435i' '미감지 — perception/stream 기동 스킵'
fi

if [ -e /dev/powertrain-pdist80b ]; then
  add_result '✅' 'PDIST80B' '/dev/powertrain-pdist80b 존재'
else
  add_result '⚠️' 'PDIST80B' \
    '/dev/powertrain-pdist80b 없음 — PDIST 미연결 또는 ch341 미로드'
fi

banner '5/7 로봇팔 스택'

arm_stack_ready=0
perception_running=0
stream_running=0

if [ "$NO_ARM" -eq 1 ]; then
  add_result '✅' 'ros2_humble' '--no-arm 요청으로 스킵'
  add_result '✅' 'perception' '--no-arm 요청으로 스킵'
  add_result '✅' 'stream' '--no-arm 요청으로 스킵'
elif [ ! -f "$ARM_REPO/docker-compose.yml" ]; then
  add_result '⚠️' 'ros2_humble' "$ARM_REPO/docker-compose.yml 없음 — 팔 스택 스킵"
  add_result '⚠️' 'perception' '팔 compose 없음 — 기동 스킵'
  add_result '⚠️' 'stream' '팔 compose 없음 — 기동 스킵'
else
  docker compose \
    -f "$ARM_REPO/docker-compose.yml" \
    -f "$ARM_REPO/docker-compose.gpu.yml" \
    up -d
  arm_compose_rc=$?
  if [ "$arm_compose_rc" -ne 0 ]; then
    add_result '❌' 'ros2_humble' "팔 compose 실패(exit $arm_compose_rc)"
    add_result '⚠️' 'perception' '팔 compose 실패 — 기동 스킵'
    add_result '⚠️' 'stream' '팔 compose 실패 — 기동 스킵'
  else
    if docker exec ros2_humble bash -lc \
      'test -f /root/ros2_ws/install/setup.bash' >/dev/null 2>&1; then
      arm_build_ok=1
    else
      printf '팔 워크스페이스 install이 없어 colcon build를 실행합니다.\n'
      if arm_build_output="$(
        docker exec ros2_humble bash -lc \
          'source /opt/ros/humble/setup.bash && cd /root/ros2_ws && colcon build' \
          2>&1
      )"; then
        arm_build_ok=1
      else
        arm_build_ok=0
      fi
      printf '%s\n' "$arm_build_output" | tail -n 20
    fi

    if [ "$arm_build_ok" -ne 1 ]; then
      add_result '❌' 'ros2_humble' 'colcon build 실패 — 팔 노드 기동 스킵'
      add_result '⚠️' 'perception' '팔 빌드 실패 — 기동 스킵'
      add_result '⚠️' 'stream' '팔 빌드 실패 — 기동 스킵'
    else
      arm_stack_ready=1
      add_result '✅' 'ros2_humble' 'compose 실행 및 install 확인'
      if [ "$d435_present" -ne 1 ]; then
        add_result '⚠️' 'perception' 'D435i 미감지 — 기동 스킵'
        add_result '⚠️' 'stream' 'D435i 미감지 — 기동 스킵'
      else
        # 팔팀 metadata_sender는 소유권 밖이므로 이 정리 대상에 절대 포함하지 않는다.
        docker exec ros2_humble bash -lc \
          'pkill -f robot_arm_perception.perception_node; pkill -f robot_arm_perception.stream_node; true'
        arm_cleanup_rc=$?
        if [ "$arm_cleanup_rc" -ne 0 ]; then
          printf '⚠️ 기존 perception/stream 정리 명령이 실패했지만 기동 검증은 계속합니다.\n'
        fi

        docker exec -d ros2_humble bash -lc \
          'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && exec ros2 run robot_arm_perception perception_node --ros-args -p model_name:=box -p camera_mode:=realsense -p pick_min_conf:=0.5 -p require_depth:=true'
        perception_start_rc=$?
        docker exec -d ros2_humble bash -lc \
          'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && exec ros2 run robot_arm_perception stream_node'
        stream_start_rc=$?

        sleep 3
        if [ "$perception_start_rc" -eq 0 ] \
          && docker exec ros2_humble pgrep -f \
            robot_arm_perception.perception_node >/dev/null 2>&1; then
          perception_running=1
          add_result '✅' 'perception' '3초 생존 확인'
        else
          add_result '❌' 'perception' '기동 실패 또는 3초 내 종료'
        fi
        if [ "$stream_start_rc" -eq 0 ] \
          && docker exec ros2_humble pgrep -f \
            robot_arm_perception.stream_node >/dev/null 2>&1; then
          stream_running=1
          add_result '✅' 'stream' '3초 생존 확인 (:5002 기본값)'
        else
          add_result '❌' 'stream' '기동 실패 또는 3초 내 종료'
        fi
      fi
    fi
  fi
fi

banner '6/7 단일 송신 원칙과 arm_console_bridge'

metadata_sender_running=0
if [ "$NO_ARM" -eq 1 ]; then
  add_result '✅' 'metadata_sender' '--no-arm: 검사 스킵, 브리지 기동 허용'
elif [ "$arm_stack_ready" -eq 1 ]; then
  if docker exec ros2_humble pgrep -f metadata_sender_node >/dev/null 2>&1; then
    metadata_sender_running=1
    printf '\n⚠️ 팔팀 metadata_sender 가동 중 — :5003 이중 송신 금지, 브리지 기동 스킵. 팔팀과 조율 필요\n'
    add_result '⚠️' 'metadata_sender' \
      '팔팀 sender 가동 중 — :5003 이중 송신 금지, 브리지 스킵; 팔팀과 조율 필요'
  else
    add_result '✅' 'metadata_sender' '부재 — 단일 송신 OK'
  fi
else
  add_result '✅' 'metadata_sender' 'ros2_humble 부재 — 검사 대상 없음, 브리지 허용'
fi

container_is_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null || true)" = true ]
}

# 정본 위치는 powertrain_ros. L515 부재 등으로 그 컨테이너가 죽어 있으면 팔
# 텔레메트리까지 연쇄로 죽지 않도록 동일 이미지·동일 /workspace 마운트인
# powertrain_control에서 폴백 기동한다.
bridge_container=""
if container_is_running powertrain_ros; then
  bridge_container=powertrain_ros
elif container_is_running powertrain_control; then
  bridge_container=powertrain_control
fi

bridge_running=0
if [ -z "$OPERATOR_HOST" ]; then
  add_result '⚠️' 'arm_console_bridge' 'OPERATOR_HOST 미확정 — 기동 스킵'
elif [ "$metadata_sender_running" -eq 1 ]; then
  add_result '⚠️' 'arm_console_bridge' ':5003 이중 송신 방지를 위해 기동 스킵'
elif [ -z "$bridge_container" ]; then
  add_result '❌' 'arm_console_bridge' \
    'powertrain_ros·powertrain_control 모두 미가동 — 기동 불가'
else
  # :5003/:5007 이중 송신 방지 — 어느 쪽에서 기동하든 두 컨테이너의 기존
  # 브리지를 모두 정리한다(이전 실행이 폴백으로 다른 컨테이너에 남겼을 수 있음).
  docker exec powertrain_ros pkill -f arm_console_bridge >/dev/null 2>&1
  docker exec powertrain_control pkill -f arm_console_bridge >/dev/null 2>&1
  docker exec -d "$bridge_container" bash -lc \
    "source /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash && exec ros2 run powertrain_ros arm_console_bridge --ros-args -p console_host:=${OPERATOR_HOST}"
  bridge_start_rc=$?
  sleep 3
  if [ "$bridge_start_rc" -eq 0 ] \
    && docker exec "$bridge_container" pgrep -f arm_console_bridge >/dev/null 2>&1; then
    bridge_running=1
    if [ "$bridge_container" = powertrain_ros ]; then
      add_result '✅' 'arm_console_bridge' "3초 생존 확인 → $OPERATOR_HOST"
    else
      add_result '⚠️' 'arm_console_bridge' \
        "powertrain_ros 다운 — powertrain_control 폴백 기동, 3초 생존 확인 → $OPERATOR_HOST"
    fi
  else
    add_result '❌' 'arm_console_bridge' "기동 실패 또는 3초 내 종료($bridge_container)"
  fi
fi

banner '7/7 포트 검증'

tcp_listeners="$(ss -tln 2>/dev/null || true)"

for tcp_port in 9000 9001; do
  if port_is_listening "$tcp_listeners" "$tcp_port"; then
    add_result '✅' "TCP :$tcp_port" 'LISTEN' 1
  else
    add_result '❌' "TCP :$tcp_port" 'LISTEN 아님' 1
  fi
done

if [ "$powertrain_ros_healthy" -eq 1 ]; then
  if wait_udp_listen 5000 15; then
    add_result '✅' 'UDP :5000' 'L515 SRT 리슨'
  else
    add_result '⚠️' 'UDP :5000' 'powertrain_ros healthy지만 L515 SRT 포트 미확인(15s 대기)'
  fi
else
  add_result '⚠️' 'UDP :5000' 'powertrain_ros 비정상 — L515 SRT 기대 조건 미충족'
fi

if [ "$stream_running" -eq 1 ]; then
  if wait_udp_listen 5002 20; then
    add_result '✅' 'UDP :5002' '팔 stream 리슨'
  else
    add_result '⚠️' 'UDP :5002' 'stream 생존하지만 SRT 포트 미확인(20s 대기 — 첫 프레임 전일 수 있음)'
  fi
else
  add_result '✅' 'UDP :5002' '팔 stream 미기동 — 기대 제외'
fi

banner '최종 요약'
printf '%-4s | %-34s | %s\n' '결과' '항목' '상세'
printf '%s\n' '-----+------------------------------------+------------------------------------------'
for index in "${!RESULT_NAMES[@]}"; do
  printf '%-4s | %-34s | %s\n' \
    "${RESULT_ICONS[$index]}" \
    "${RESULT_NAMES[$index]}" \
    "${RESULT_DETAILS[$index]}"
done

if (( CRITICAL_FAILURES > 0 )); then
  exit_code=1
  exit_reason="크리티컬 실패 ${CRITICAL_FAILURES}건"
elif (( NON_GREEN_RESULTS > 0 )); then
  exit_code=2
  exit_reason="크리티컬 항목은 정상이나 경고/비크리티컬 실패 ${NON_GREEN_RESULTS}건"
else
  exit_code=0
  exit_reason='모든 기대 항목 정상'
fi

jetson_addresses="$(hostname -I 2>/dev/null || true)"
jetson_ip="${jetson_addresses%% *}"
[ -n "$jetson_ip" ] || jetson_ip='<JETSON_IP>'

printf '\n종료 코드: %d — %s\n' "$exit_code" "$exit_reason"
printf '운영 PC에서: /usr/bin/python3 -m operator_console.app --host %s\n' "$jetson_ip"
exit "$exit_code"
