#!/usr/bin/env bash
# 새 젯슨에 OPERATOR_HOST 전용 root 헬퍼와 좁은 sudoers 통로를 설치한다.
set -euo pipefail

usage='사용법: sudo bash scripts/install_operator_host_helper.sh [--user NAME]'

fail() {
  printf 'operator-host helper 설치 실패: %s\n' "$*" >&2
  exit 1
}

target_user=""
case "$#" in
  0) ;;
  1)
    case "$1" in
      -h | --help)
        printf '%s\n' "$usage"
        exit 0
        ;;
      *) fail "$usage" ;;
    esac
    ;;
  2)
    [[ "$1" == --user ]] || fail "$usage"
    target_user="$2"
    ;;
  *) fail "$usage" ;;
esac

if [[ "$(id -u)" -ne 0 ]]; then
  printf '오류: root 권한이 필요합니다. 다음처럼 실행하세요:\n' >&2
  printf '  sudo bash scripts/install_operator_host_helper.sh [--user NAME]\n' >&2
  exit 77
fi

if [[ -z "$target_user" ]]; then
  target_user="${SUDO_USER:-zetin}"
fi
[[ "$target_user" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] \
  || fail "유효하지 않은 사용자 이름: $target_user"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
helper_source="$script_dir/powertrain-set-operator-host"
sbin_dir="${POWERTRAIN_SBIN_DIR:-/usr/local/sbin}"
sudoers_dir="${POWERTRAIN_SUDOERS_DIR:-/etc/sudoers.d}"
helper_destination="$sbin_dir/powertrain-set-operator-host"
sudoers_destination="$sudoers_dir/powertrain-operator-host"

[[ -f "$helper_source" ]] || fail "헬퍼 원본이 없습니다: $helper_source"
[[ -d "$sbin_dir" ]] || fail "설치 디렉터리가 없습니다: $sbin_dir"
[[ -d "$sudoers_dir" ]] || fail "sudoers 디렉터리가 없습니다: $sudoers_dir"
command -v install >/dev/null 2>&1 || fail 'install 명령을 찾을 수 없습니다.'
command -v visudo >/dev/null 2>&1 || fail 'visudo 명령을 찾을 수 없습니다.'

install -m 0755 "$helper_source" "$helper_destination"
chown root:root "$helper_destination"

sudoers_temporary="$(mktemp "$sudoers_dir/.powertrain-operator-host.XXXXXX")"
cleanup() {
  rm -f -- "$sudoers_temporary"
}
trap cleanup EXIT

printf '%s ALL=(root) NOPASSWD: %s\n' \
  "$target_user" "$helper_destination" > "$sudoers_temporary"
if ! visudo -cf "$sudoers_temporary"; then
  fail 'visudo 문법 검증 실패 — sudoers 드롭인을 설치하지 않았습니다.'
fi

chmod 0440 "$sudoers_temporary"
chown root:root "$sudoers_temporary"
mv -f -- "$sudoers_temporary" "$sudoers_destination"
trap - EXIT

printf '설치 완료:\n'
printf '  헬퍼: %s (root:root 0755)\n' "$helper_destination"
printf '  sudoers: %s (root:root 0440, 사용자 %s)\n' \
  "$sudoers_destination" "$target_user"
printf '확인 명령: sudo -n /usr/local/sbin/powertrain-set-operator-host <IP>\n'
