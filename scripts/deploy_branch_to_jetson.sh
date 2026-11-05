#!/usr/bin/env bash
# 노트북의 브랜치를 젯슨 체크아웃으로 올리고 실행 준비를 한다 — 노트북에서 실행한다.
#
# 로봇 AP 에 붙은 젯슨은 인터넷이 없을 수 있어 git fetch 대신 git bundle 을 ssh 로 보낸다.
#
#   bash scripts/deploy_branch_to_jetson.sh [--branch NAME] [--host HOST]... [--prepare]
#                                            [-- robot_prepare 추가 인자...]
#
# 하는 일:
#   1) 젯슨 찾기 (--host, $JETSON_HOST, jetson-orin.local, 192.168.8.106, 192.168.50.98)
#   2) 원격 체크아웃 상태 표시 — 현재 브랜치, 미커밋 파일, upstream 에 없는 커밋
#   3) 번들 전송 → 원격 <브랜치>를 노트북 커밋으로 갱신 (fast-forward 만)
#   4) git switch/merge — 팀원 미커밋·ignored 파일과 충돌하면 git 이 멈춘다.
#      강제·reset·stash·autostash 는 하지 않는다. 원격 전용 커밋이 있으면 중단한다.
#   5) 원격 HEAD 가 노트북 커밋과 같은지 대조한다.
#   6) --prepare: sudo scripts/robot_prepare.sh --use-existing-images (ROS 소스 재빌드).
#      robot_prepare 는 부팅 CAN 준비 서비스(powertrain-bringup-preflight, can_setup)를
#      한 번 실행한다. 모터 지령·통합 스택 기동은 하지 않는다.
#   7) 읽기 전용 확인: I2C 버스 목록, AS5048B scan, 실행 중 컨테이너
#   8) 통합 스택 기동 명령을 출력한다. 기동은 사람이 직접 한다.
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
    --branch) [ "$#" -ge 2 ] || die "--branch 값이 없다"; BRANCH="$2"; shift 2 ;;
    --host) [ "$#" -ge 2 ] || die "--host 값이 없다"; HOSTS+=("$2"); shift 2 ;;
    --prepare) PREPARE=1; shift ;;
    --) shift; PREPARE_ARGS=("$@"); break ;;
    -h|--help) sed -n '2,23p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "알 수 없는 인자: $1" ;;
  esac
done

# 원격 셸 명령에 들어가는 값은 따옴표·공백·셸 메타문자가 없는 것만 받는다.
plain() { [[ "$2" =~ $3 ]] || die "$1 값이 허용 형식이 아니다: $2"; }
plain 브랜치 "$BRANCH" '^[A-Za-z0-9][A-Za-z0-9._/-]*$'
git check-ref-format --branch "$BRANCH" >/dev/null 2>&1 || die "브랜치 이름이 올바르지 않다: $BRANCH"
plain JETSON_USER "$REMOTE_USER" '^[a-z_][a-z0-9_-]*$'
plain JETSON_REPO "$REMOTE_REPO" '^/[A-Za-z0-9._/-]+$'
plain ROBOT_ID "$ROBOT_ID" '^[A-Za-z0-9][A-Za-z0-9._-]*$'
plain OPS_TOKEN_FILE "$TOKEN_FILE" '^/[A-Za-z0-9._/-]+$'
for arg in "${PREPARE_ARGS[@]}"; do plain "robot_prepare 인자" "$arg" '^[A-Za-z0-9/._=:-]+$'; done

TARGET="$(git -C "$REPO_LOCAL" rev-parse --verify --quiet "refs/heads/$BRANCH^{commit}")" \
  || die "로컬 브랜치가 없다: $BRANCH"
[ -z "$(git -C "$REPO_LOCAL" status --porcelain --untracked-files=no)" ] \
  || warn "로컬에 커밋 안 된 변경이 있다 — 커밋된 $BRANCH 만 올라간다"
HOSTS+=("${JETSON_HOST:-}" jetson-orin.local 192.168.8.106 192.168.50.98)
for host in "${HOSTS[@]}"; do [ -z "$host" ] || plain 호스트 "$host" '^[A-Za-z0-9._-]+$'; done

SSH=(ssh -o ConnectTimeout=6)
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
repo() { remote "cd '$REMOTE_REPO' && $1" </dev/null; }

remote "test -d '$REMOTE_REPO/.git'" </dev/null || die "원격 저장소가 없다: $REMOTE_REPO (JETSON_REPO 로 지정)"

say "원격 체크아웃 상태"
repo "echo \"브랜치: \$(git branch --show-current)\" && git log --oneline -1 \
  && echo '--- 미커밋 파일' && git status --short | head -30 \
  && echo '--- upstream 에 없는 커밋' && (git log --oneline '@{u}..' 2>/dev/null | head -10 || true)" || true
CURRENT="$(repo "git branch --show-current")" || die "원격 브랜치를 읽지 못했다"

say "번들 전송: $BRANCH ($(git -C "$REPO_LOCAL" rev-parse --short "$TARGET"))"
BUNDLE="$(mktemp --suffix=.bundle)"
REMOTE_BUNDLE="$(remote "mktemp /tmp/powertrain-deploy.XXXXXX" </dev/null)" || die "원격 임시 파일을 만들지 못했다"
plain "원격 임시 파일" "$REMOTE_BUNDLE" '^/tmp/powertrain-deploy\.[A-Za-z0-9]+$'
cleanup() { rm -f "$BUNDLE"; remote "rm -f '$REMOTE_BUNDLE'" </dev/null 2>/dev/null || true; }
trap cleanup EXIT
git -C "$REPO_LOCAL" bundle create -q "$BUNDLE" "$BRANCH"
remote "cat > '$REMOTE_BUNDLE'" < "$BUNDLE" || die "번들 전송 실패"

if [ "$CURRENT" = "$BRANCH" ]; then
  # 체크아웃된 브랜치에 원격 전용 커밋이 있으면 노트북 커밋으로 준비할 수 없으니 멈춘다.
  repo "git fetch -q '$REMOTE_BUNDLE' '$BRANCH' && git merge-base --is-ancestor HEAD FETCH_HEAD" \
    || die "원격 $BRANCH 에 노트북에 없는 커밋이 있다. 원격 상태를 먼저 확인한다."
  repo "git merge --ff-only --no-autostash --no-overwrite-ignore FETCH_HEAD" \
    || die "fast-forward 중단 — 원격 미커밋·ignored 파일과 충돌한다. 파일을 보존한 채 확인받는다."
else
  # 일반 refspec 은 non-fast-forward 갱신을 거부한다 — 원격에서 따로 커밋된 브랜치는 덮지 않는다.
  repo "git fetch -q '$REMOTE_BUNDLE' '$BRANCH:refs/heads/$BRANCH'" \
    || die "원격 $BRANCH 갱신 거부 — 원격에 따로 커밋된 브랜치다. 원격 상태를 먼저 확인한다."
  repo "git switch --no-overwrite-ignore '$BRANCH'" \
    || die "git switch 중단 — 원격 미커밋·ignored 파일과 충돌한다. 파일을 보존한 채 확인받는다."
fi
HEAD_NOW="$(repo "git rev-parse HEAD")" || die "원격 HEAD 를 읽지 못했다"
[ "$HEAD_NOW" = "$TARGET" ] || die "원격 HEAD $HEAD_NOW 가 노트북 $TARGET 과 다르다"
ok "원격 HEAD = $(git -C "$REPO_LOCAL" log --oneline -1 "$TARGET")"

if [ "$PREPARE" -eq 1 ]; then
  say "robot_prepare (기존 이미지 사용, ROS 소스 재빌드, CAN 준비 서비스 1회)"
  [ -n "${JETSON_SUDO_PASS:-}" ] || die "JETSON_SUDO_PASS 가 없다"
  # 비밀번호는 ssh 표준입력으로만 넘긴다. sudo -v 가 소비하고, 캐시돼 남은 줄이
  # robot_prepare 의 입력으로 새지 않도록 본 실행은 /dev/null 을 표준입력으로 쓴다.
  printf '%s\n' "$JETSON_SUDO_PASS" | remote "cd '$REMOTE_REPO' && sudo -S -p '' -v && \
    sudo -n bash scripts/robot_prepare.sh --robot-id '$ROBOT_ID' --token-file '$TOKEN_FILE' \
    --use-existing-images ${PREPARE_ARGS[*]:-} </dev/null" || die "robot_prepare 실패"
fi

say "읽기 전용 확인"
repo "ls /dev/i2c-* 2>/dev/null | tr '\n' ' '; echo; \
  (command -v i2cdetect >/dev/null && i2cdetect -l) || echo 'i2cdetect 없음'; \
  PYTHONPATH=motor_control timeout 20 python3 -m suspension_encoders scan || true; \
  docker ps --format '{{.Names}}\t{{.Status}}' | head -20" || warn "읽기 전용 확인 일부 실패"

say "다음 단계 (사람이 직접 실행)"
cat <<EOF
   젯슨 접속:  ssh $REMOTE_USER@$HOST
   저장소:     cd $REMOTE_REPO
   지형 지도·오버레이까지 켜서 기동:
     INTEGRATED_PROFILES="terrain-overlay terrain-map" \\
     INTEGRATED_SERVICES="powertrain_terrain_overlay powertrain_terrain_map" \\
       scripts/robot-start
   노트북 콘솔: docs/integrated-operation.md 5단계
EOF
