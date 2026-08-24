#!/usr/bin/env bash
# USB 스키드 주행 원클릭 배포 — 노트북에서 실행한다.
#
# 젯슨에 붙으면 인터넷이 끊기므로 git pull 을 못 쓴다. 이 스크립트는 로컬 링크로
# 파일을 직접 밀어넣고, 그 자리에서 레지스트리 생성·검증까지 끝낸다.
#
#   bash scripts/deploy_usb_skid_to_jetson.sh [젯슨IP]
#
# 하는 일:
#   1) 젯슨 접속 확인 + 원격 레포 상태 표시(미커밋 작업 경고)
#   2) 덮어쓸 파일 백업 (원격, 타임스탬프)
#   3) 이번 작업 파일 전송 (tar over ssh — rsync 불필요)
#   4) 충돌 스택 정지 (같은 모터를 잡는 컨테이너들)  ※ KEEP_OTHERS=1 로 생략
#   5) powertrain 컨테이너 기동 (없으면 compose up, 정지면 start)
#   6) 컨테이너 안 좀비 제어 프로세스 kill
#   7) odrive·pyusb 확인
#   8) 보드 레지스트리 자동 생성 (USB 로 can_node_id 읽음, CAN 버스 무관)
#   9) 무하드웨어 검증 + 실행 명령 출력
#
# 로봇팔·US-100 은 안 쓰는 구성이다 — 팔 스택은 건드리지 않고, 실행 명령은
# --no-us100 --confirm-arm-stowed 로 나온다.
# ⚠️ US-100 이 없으면 접근 시 자동 정지가 없다. 바퀴 들고 하는 벤치 전용.
#
# 모터는 돌리지 않는다. 마지막에 나오는 명령을 사람이 직접 친다.
set -euo pipefail

REPO_LOCAL="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_USER="${JETSON_USER:-zetin}"
REMOTE_REPO="${JETSON_REPO:-/home/zetin/power-train-sw}"
CONTAINER="${JETSON_CONTAINER:-powertrain_jetson}"
CANDIDATES=("$@" "${JETSON_HOST:-}" jetson-orin.local 192.168.8.106 192.168.50.98)

say()  { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok()   { printf '   \033[32m✅ %s\033[0m\n' "$*"; }
warn() { printf '   \033[33m⚠️  %s\033[0m\n' "$*"; }
die()  { printf '\n\033[31m❌ %s\033[0m\n' "$*" >&2; exit 1; }

# sshpass 가 있으면 비번을, 없으면 키 인증을 쓴다.
if command -v sshpass >/dev/null && [ -n "${JETSON_SSH_PASS:-}" ]; then
  SSH=(sshpass -p "$JETSON_SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=6)
else
  SSH=(ssh -o StrictHostKeyChecking=no -o ConnectTimeout=6)
fi

# ── 1. 호스트 찾기 ────────────────────────────────────────────────────────
say "젯슨 찾는 중"
HOST=""
for c in "${CANDIDATES[@]}"; do
  [ -z "$c" ] && continue
  if "${SSH[@]}" "${REMOTE_USER}@${c}" true 2>/dev/null; then HOST="$c"; break; fi
  printf '   · %s 응답 없음\n' "$c"
done
[ -n "$HOST" ] || die "젯슨에 접속할 수 없다. IP 를 인자로 주거나 JETSON_HOST 를 설정하라.
   시도한 주소: ${CANDIDATES[*]}"
ok "접속: ${REMOTE_USER}@${HOST}"
RSH=("${SSH[@]}" "${REMOTE_USER}@${HOST}")

# 원격 실행 헬퍼 — 스크립트를 stdin 으로 흘려보낸다. 중첩 따옴표는 현장에서
# 디버깅 못 하는 자리라 인용 규칙을 최대한 단순하게 유지한다.
run_host()   { "${RSH[@]}" "sh -s"; }               # 젯슨 호스트 셸
run_remote() { "${RSH[@]}" "$REMOTE_SH"; }          # 컨테이너 안 (REMOTE_SH 설정 후)

"${RSH[@]}" "test -d '$REMOTE_REPO'" \
  || die "원격에 레포가 없다: $REMOTE_REPO  (JETSON_REPO 로 지정)"

# ── 2. 원격 상태 확인 ─────────────────────────────────────────────────────
say "원격 레포 상태"
"${RSH[@]}" "cd '$REMOTE_REPO' && git log --oneline -1 && git status --short | head -20" || true
DIRTY=$("${RSH[@]}" "cd '$REMOTE_REPO' && git status --porcelain | wc -l" || echo 0)
if [ "${DIRTY:-0}" -gt 0 ]; then
  warn "원격에 미커밋 변경 ${DIRTY}건 있다 — 아래 파일만 덮어쓰고 나머지는 안 건드린다"
  warn "덮어쓸 파일은 백업된다(3단계)"
fi

# ── 3. 보낼 파일 ──────────────────────────────────────────────────────────
# 젯슨이 쓰는 것만. operator_console 은 운용 PC 몫이라 제외한다.
FILES=(
  motor_control/chassis/kinematics.py
  motor_control/chassis/odometry.py
  motor_control/chassis/chassis_manager.py
  motor_control/chassis/teleop_server.py
  motor_control/corner_module/drive_odrive_usb_axis.py
  ros2/src/powertrain_ros/powertrain_ros/chassis_node.py
  ros2/src/powertrain_ros/powertrain_ros/steering_contract.py
  ros2/src/powertrain_ros/powertrain_ros/transport_mode.py
  ros2/src/powertrain_ros/powertrain_ros/state_estimation.py
  ros2/src/powertrain_ros/powertrain_ros/odometry_node.py
  ros2/src/powertrain_ros/powertrain_ros/imu_tilt_node.py
  ros2/src/powertrain_ros/powertrain_ros/ops_contract.py
  ros2/src/powertrain_ros/launch/autonomy.launch.py
  scripts/usb_skid_gen_registry.py
  config/README-bl70200-boards.md
  docs/reports/2026-08-05-usb-skid-bringup.md
)
# 벤치 검증된 USB 브링업 스크립트 (미추적이라 젯슨에 없을 수 있다)
[ -f "$REPO_LOCAL/motor_control/drive/bl70200/dualsense_usb_teleop.py" ] \
  && FILES+=(motor_control/drive/bl70200/dualsense_usb_teleop.py)

say "전송할 파일 ${#FILES[@]}개"
for f in "${FILES[@]}"; do
  [ -f "$REPO_LOCAL/$f" ] || die "로컬에 없다: $f"
done
ok "로컬 존재 확인"

STAMP=$(date +%Y%m%d-%H%M%S)
BACKUP="$REMOTE_REPO/.deploy-backup/$STAMP"
say "원격 백업 → $BACKUP"
"${RSH[@]}" "mkdir -p '$BACKUP' && cd '$REMOTE_REPO' && \
  for f in ${FILES[*]}; do
    if [ -f \"\$f\" ]; then mkdir -p \"$BACKUP/\$(dirname \$f)\"; cp \"\$f\" \"$BACKUP/\$f\"; fi
  done; echo '   백업한 파일:' \$(find '$BACKUP' -type f | wc -l)"

say "전송 중 (tar over ssh)"
tar -czf - -C "$REPO_LOCAL" "${FILES[@]}" \
  | "${RSH[@]}" "mkdir -p '$REMOTE_REPO' && tar -xzf - -C '$REMOTE_REPO'"
ok "전송 완료"

# ── 4. 충돌 정리 (이미 돌고 있는 것 끄기) ─────────────────────────────────
# 같은 모터를 잡는 스택이 떠 있으면 새로 띄운 teleop 과 싸운다. 좀비 제어루프가
# v=0 을 계속 명령해 반나절을 날린 전례가 있다(2026-07-05).
#
# 로봇팔은 이번 구성에서 안 쓴다. 팔 스택은 팀이 별도 레포로 돌리므로 여기서
# 건드리지 않고, 우리 쪽 팔 연동(arm_console_bridge 등)이 든 컨테이너만 내린다.
CONFLICTS=(powertrain_control powertrain_chassis powertrain_ros powertrain_autonomy canwatchdog)
if [ "${KEEP_OTHERS:-0}" = "1" ]; then
  warn "KEEP_OTHERS=1 — 다른 컨테이너를 그대로 둔다 (모터 충돌 주의)"
else
  say "충돌 스택 정리"
  for c in "${CONFLICTS[@]}"; do
    st=$("${RSH[@]}" "docker inspect -f '{{.State.Running}}' '$c' 2>/dev/null || echo missing" | tr -d '\r')
    if [ "$st" = "true" ]; then
      "${RSH[@]}" "docker stop '$c'" >/dev/null 2>&1 && ok "정지: $c" || warn "정지 실패: $c"
    else
      printf '   · %s 안 떠 있음\n' "$c"
    fi
  done
fi

# ── 5. 컨테이너 기동 ──────────────────────────────────────────────────────
# odrive·pyusb 는 컨테이너에만 있다. 호스트 파이썬으로는 USB 열거가 안 된다.
# CAN 을 안 쓰므로 canwatchdog 서비스는 띄우지 않는다 — powertrain 하나면 된다.
say "컨테이너 $CONTAINER 확인"
container_state() {
  "${RSH[@]}" "docker inspect -f '{{.State.Running}}' '$CONTAINER' 2>/dev/null || echo missing"
}
STATE=$(container_state | tr -d '\r')
case "$STATE" in
  true)
    ok "이미 가동 중" ;;
  false)
    warn "정지 상태 — 시작한다"
    "${RSH[@]}" "docker start '$CONTAINER'" >/dev/null \
      || die "docker start 실패. 젯슨에서 직접 확인하라: docker logs $CONTAINER" ;;
  *)
    warn "컨테이너가 없다 — compose 로 띄운다 (서비스 powertrain)"
    # ⚠️ powertrain 서비스에는 build: 가 있다. 이미지가 없으면 compose 가 빌드를
    #    시도하는데 그건 인터넷이 필요하다 — 젯슨에 붙은 지금은 못 한다.
    #    그래서 이미지 존재를 먼저 확인하고 --no-build 로 띄운다.
    if ! "${RSH[@]}" "docker image inspect powertrain-sw:jetson >/dev/null 2>&1"; then
      die "이미지 powertrain-sw:jetson 이 없다.
   컨테이너를 처음 만들려면 빌드가 필요하고 빌드는 인터넷이 있어야 한다.
   인터넷 되는 곳에서 먼저:
     cd $REMOTE_REPO && docker compose -f docker/docker-compose.jetson.yml build powertrain"
    fi
    set +e
    run_host <<REMOTE
set -e
cd '$REMOTE_REPO'
if docker compose version >/dev/null 2>&1; then
  docker compose -f docker/docker-compose.jetson.yml up -d --no-build powertrain
else
  docker-compose -f docker/docker-compose.jetson.yml up -d --no-build powertrain
fi
REMOTE
    UP_RC=$?
    set -e
    [ $UP_RC -eq 0 ] || die "compose up 실패. 젯슨에서 직접: \
cd $REMOTE_REPO && docker compose -f docker/docker-compose.jetson.yml up -d powertrain"
    ;;
esac

# 뜰 때까지 기다린다 (이미지 로드·초기화에 몇 초 걸린다)
for i in $(seq 1 30); do
  [ "$(container_state | tr -d '\r')" = "true" ] && break
  sleep 2
done
[ "$(container_state | tr -d '\r')" = "true" ] \
  || die "컨테이너가 뜨지 않았다. 젯슨에서: docker logs $CONTAINER"
ok "컨테이너 가동 확인"

# ── 6. 컨테이너 안 좀비 정리 ──────────────────────────────────────────────
REMOTE_SH="docker exec -i $CONTAINER sh -s"

say "좀비 제어 프로세스 정리"
run_remote <<'REMOTE'
found=$(ps -eo pid,args 2>/dev/null | grep -E 'teleop_server|teleop_dualsense|chassis\.|chassis_node|dualsense_usb_teleop' | grep -v grep)
if [ -n "$found" ]; then
  echo "$found" | sed 's/^/   죽일 것: /'
  echo "$found" | awk '{print $1}' | xargs -r kill 2>/dev/null
  sleep 1
  echo "$found" | awk '{print $1}' | xargs -r kill -9 2>/dev/null
  echo "   정리 완료"
else
  echo "   좀비 없음"
fi
REMOTE

# ── 7. 컨테이너 안 환경 확인 ──────────────────────────────────────────────
say "컨테이너 안 파이썬 환경"
run_remote <<'REMOTE'
python3 - <<'PY'
import importlib
for mod in ("odrive", "usb.core"):
    try:
        importlib.import_module(mod)
        print("   ✅ %s" % mod)
    except Exception as exc:
        print("   ❌ %s — %s" % (mod, exc))
PY
REMOTE

# ── 8. 레지스트리 자동 생성 ───────────────────────────────────────────────
say "보드 레지스트리 생성 (USB 로 can_node_id 읽음 — CAN 버스 무관)"
set +e
run_remote <<REMOTE
set -e
cd /workspace 2>/dev/null || cd '$REMOTE_REPO'
python3 scripts/usb_skid_gen_registry.py
REMOTE
GEN_RC=$?
set -e
if [ $GEN_RC -ne 0 ]; then
  warn "레지스트리 자동 생성 실패 (위 메시지 참조)"
  warn "수동 작성법: config/README-bl70200-boards.md"
else
  ok "레지스트리 생성·검증 완료"
fi

# ── 9. 무하드웨어 검증 ────────────────────────────────────────────────────
say "원격 검증 (모터 안 돌림)"
set +e
run_remote <<REMOTE
cd /workspace 2>/dev/null || cd '$REMOTE_REPO'
cd motor_control
python3 - <<'PY'
from chassis.kinematics import skid_geometry
from chassis.chassis_manager import build_usb_skid_corners
g = skid_geometry()
steerable = sum(1 for w in g.wheels if w.steerable)
print("기하: 바퀴 %d개, 조향륜 %d개 (스키드는 0이어야 정상)" % (len(g.wheels), steerable))
print("import OK — teleop_server 가 쓸 경로 전부 살아있음")
PY
REMOTE
[ $? -eq 0 ] || warn "검증 실패 — 위 오류 확인"
set -e

# ── 10. 다음 단계 ──────────────────────────────────────────────────────────
cat <<EOF

$(printf '\033[1m══ 다음: 실제 주행 ══\033[0m')

  ⚠️  먼저 바퀴가 전부 들려 있는지 확인하라. 조향축(AK)이 무통전 상태로
      스키드를 돌리므로, 각이 밀리는지 육안으로 봐야 한다.

  [젯슨]  docker exec -it $CONTAINER sh -lc \\
            "cd /workspace/motor_control && python3 -m chassis.teleop_server \\
               --skid-usb --no-us100 --diagnostic-direct-can --confirm-arm-stowed"

          --no-us100        : US-100 미사용 (컨테이너에 pyserial 없음)
          --confirm-arm-stowed : 로봇팔 미사용이라 확인 프롬프트 생략

  ⚠️  US-100 을 끄면 접근 시 자동 정지가 없다. 바퀴 들고 하는 벤치 전용이며,
      지상 주행 전에는 다시 켜거나 다른 안전장치를 둘 것.

  [노트북] python3 motor_control/laptop/laptop_client_chassis.py \\
             --host $HOST --port 9000

  조작: RT=전진 LT=후진 좌스틱X=선회 □=arm/disarm ○=종료

  기대 부호(모터 프레임): 전진 좌+/우−   ·   제자리선회 6축 동일 부호
  절차 전문: docs/reports/2026-08-05-usb-skid-bringup.md

  되돌리려면: ssh ${REMOTE_USER}@${HOST} "cp -r $BACKUP/* $REMOTE_REPO/"

EOF
