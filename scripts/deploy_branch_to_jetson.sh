#!/usr/bin/env bash
# 노트북의 브랜치를 젯슨 체크아웃으로 올리고 실행 준비를 한다 — 노트북에서 실행한다.
#
# 로봇 AP 에 붙은 젯슨은 인터넷이 없을 수 있어 git fetch 대신 git bundle 을 ssh 로 보낸다.
#
#   bash scripts/deploy_branch_to_jetson.sh [--branch NAME] [--host HOST]... [--prepare]
#                                            [--start-command] [-- robot_prepare 추가 인자...]
#
# 하는 일:
#   1) 젯슨 찾기 (--host, $JETSON_HOST, jetson-orin.local, 192.168.8.106, 192.168.50.98)
#   2) 원격 체크아웃 상태 표시 — 현재 브랜치, 미커밋 파일, 원격에만 있는 커밋
#   3) 번들 전송 → 원격 refs/heads/<브랜치> 갱신 (현재 체크아웃된 브랜치면 fast-forward 만)
#   4) git switch — 팀원의 미커밋 파일과 충돌하면 git 이 멈춘다. 강제·reset·stash 는 하지 않는다.
#   5) --prepare: sudo scripts/robot_prepare.sh --use-existing-images (ROS 소스 재빌드)
#   6) 읽기 전용 확인: I2C 버스 목록, AS5048B scan, 실행 중 컨테이너
#   7) 서비스 기동 명령을 출력한다. 모터를 돌리거나 서비스를 띄우지는 않는다.
#
# 비밀번호는 환경변수로만 받는다: $JETSON_SSH_PASS(sshpass -e), $JETSON_SUDO_PASS(ssh 표준입력).
set -euo pipefail

REPO_LOCAL="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_USER="${JETSON_USER:-zetin}"
REMOTE_REPO="${JETSON_REPO:-/home/zetin/power-train-sw}"
ROBOT_ID="${ROBOT_ID:-zetin-rover}"
TOKEN_FILE="${OPS_TOKEN_FILE:-/etc/powertrain/ops_console.token}"
BRANCH="$(git -C "$REPO_LOCAL" branch --show-current)"
HOSTS=()
PREPARE=0
PREPARE_ARGS=()

say()  { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok()   { printf '   \033[32m✅ %s\033[0m\n' "$*"; }
warn() { printf '   \033[33m⚠️  %s\033[0m\n' "$*"; }
die()  { printf '\n\033[31m❌ %s\033[0m\n' "$*" >&2; exit 1; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    --branch) BRANCH="$2"; shift 2 ;;
    --host) HOSTS+=("$2"); shift 2 ;;
    --prepare) PREPARE=1; shift ;;
    --) shift; PREPARE_ARGS=("$@"); break ;;
    -h|--help) sed -n '2,18p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "알 수 없는 인자: $1" ;;
  esac
done
[ -n "$BRANCH" ] || die "브랜치를 정할 수 없다 (--branch 로 지정)"
git -C "$REPO_LOCAL" rev-parse --verify --quiet "refs/heads/$BRANCH" >/dev/null \
  || die "로컬 브랜치가 없다: $BRANCH"
[ -z "$(git -C "$REPO_LOCAL" status --porcelain --untracked-files=no)" ] \
  || warn "로컬에 커밋 안 된 변경이 있다 — 커밋된 $BRANCH 만 올라간다"
HOSTS+=("${JETSON_HOST:-}" jetson-orin.local 192.168.8.106 192.168.50.98)

SSH=(ssh -o ConnectTimeout=6 -o BatchMode=no)
if [ -n "${JETSON_SSH_PASS:-}" ] && command -v sshpass >/dev/null; then
  export SSHPASS="$JETSON_SSH_PASS"
  SSH=(sshpass -e ssh -o ConnectTimeout=6)
fi

say "젯슨 찾는 중"
HOST=""
for candidate in "${HOSTS[@]}"; do
  [ -n "$candidate" ] || continue
  if "${SSH[@]}" "$REMOTE_USER@$candidate" true </dev/null 2>/dev/null; then HOST="$candidate"; break; fi
  printf '   · %s 응답 없음\n' "$candidate"
done
[ -n "$HOST" ] || die "젯슨에 접속할 수 없다. 로봇 AP 연결을 확인하고 --host 로 주소를 준다."
ok "접속: $REMOTE_USER@$HOST"
remote() { "${SSH[@]}" "$REMOTE_USER@$HOST" "$@"; }

remote "test -d '$REMOTE_REPO/.git'" </dev/null || die "원격 저장소가 없다: $REMOTE_REPO (JETSON_REPO 로 지정)"

say "원격 체크아웃 상태"
remote "cd '$REMOTE_REPO' && echo \"브랜치: \$(git branch --show-current)\" && git log --oneline -1 \
  && echo '--- 미커밋 파일' && git status --short | head -30 \
  && echo '--- upstream 에 없는 커밋' && (git log --oneline '@{u}..' 2>/dev/null | head -10 || true)" </dev/null || true
CURRENT="$(remote "git -C '$REMOTE_REPO' branch --show-current" </dev/null)"

say "번들 전송: $BRANCH ($(git -C "$REPO_LOCAL" rev-parse --short "$BRANCH"))"
BUNDLE="$(mktemp --suffix=.bundle)"
trap 'rm -f "$BUNDLE"' EXIT
git -C "$REPO_LOCAL" bundle create -q "$BUNDLE" "$BRANCH"
REMOTE_BUNDLE="/tmp/powertrain-deploy-$$.bundle"
remote "cat > '$REMOTE_BUNDLE'" < "$BUNDLE"
if [ "$CURRENT" = "$BRANCH" ]; then
  # 체크아웃된 브랜치는 작업 트리를 함께 옮겨야 하므로 fast-forward 병합만 허용한다.
  remote "cd '$REMOTE_REPO' && git fetch -q '$REMOTE_BUNDLE' '$BRANCH' && git merge --ff-only FETCH_HEAD" </dev/null \
    || die "fast-forward 불가 — 원격 $BRANCH 에 따로 커밋이 있다. 원격 상태를 먼저 확인한다."
else
  remote "cd '$REMOTE_REPO' && git fetch -q '$REMOTE_BUNDLE' '$BRANCH:refs/heads/$BRANCH'" </dev/null \
    || die "원격 $BRANCH 갱신 실패 — 원격에서 따로 바뀐 브랜치면 덮어쓰지 않는다."
  remote "cd '$REMOTE_REPO' && git switch '$BRANCH'" </dev/null \
    || die "git switch 중단 — 원격 미커밋 파일과 충돌한다. 파일을 보존한 채 사람에게 확인받는다."
fi
remote "rm -f '$REMOTE_BUNDLE'" </dev/null
ok "원격 $(remote "git -C '$REMOTE_REPO' log --oneline -1" </dev/null)"

if [ "$PREPARE" -eq 1 ]; then
  say "robot_prepare (기존 이미지 사용, ROS 소스 재빌드)"
  [ -n "${JETSON_SUDO_PASS:-}" ] || die "JETSON_SUDO_PASS 가 없다"
  # 비밀번호는 원격 명령줄이 아니라 ssh 표준입력으로만 넘긴다.
  printf '%s\n' "$JETSON_SUDO_PASS" | remote "cd '$REMOTE_REPO' && sudo -S -p '' bash scripts/robot_prepare.sh \
    --robot-id '$ROBOT_ID' --token-file '$TOKEN_FILE' --use-existing-images ${PREPARE_ARGS[*]:-}"
fi

say "읽기 전용 확인"
remote "ls /dev/i2c-* 2>/dev/null | tr '\n' ' '; echo; \
  (command -v i2cdetect >/dev/null && i2cdetect -l) || echo 'i2cdetect 없음'; \
  cd '$REMOTE_REPO' && PYTHONPATH=motor_control timeout 20 python3 -m suspension_encoders scan || true; \
  docker ps --format '{{.Names}}\t{{.Status}}' | head -20" </dev/null || true

say "다음 단계 (사람이 직접 실행)"
cat <<EOF
   젯슨 접속:  ssh $REMOTE_USER@$HOST
   저장소:     cd $REMOTE_REPO
   지형 지도·오버레이까지 켜서 기동:
     INTEGRATED_PROFILES="terrain-overlay terrain-map" \\
     INTEGRATED_SERVICES="powertrain_terrain_overlay powertrain_terrain_map" \\
       scripts/robot-start
   노트북 콘솔: python -m operator_console (docs/integrated-operation.md 5단계)
EOF
