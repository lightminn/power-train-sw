#!/usr/bin/env bash
# One-time laptop setup for the integrated operator console.
set -euo pipefail

fail() {
  printf 'integrated console install FAIL: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
usage: scripts/install_integrated_console.sh --robot-id ID --host HOST
       --token-file PATH [--host HOST ...] [--console-python PATH]
       [--controller-python PATH] [--profile can-4ws] [--replace-legacy]
EOF
}

robot_id=""
hosts=()
token_file=""
console_python=""
controller_python=""
profile=can-4ws
replace_legacy=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --robot-id) [ "$#" -ge 2 ] || fail '--robot-id requires a value'; robot_id="$2"; shift 2 ;;
    --host) [ "$#" -ge 2 ] || fail '--host requires a value'; hosts+=("$2"); shift 2 ;;
    --token-file) [ "$#" -ge 2 ] || fail '--token-file requires a value'; token_file="$2"; shift 2 ;;
    --console-python) [ "$#" -ge 2 ] || fail '--console-python requires a value'; console_python="$2"; shift 2 ;;
    --controller-python) [ "$#" -ge 2 ] || fail '--controller-python requires a value'; controller_python="$2"; shift 2 ;;
    --profile) [ "$#" -ge 2 ] || fail '--profile requires a value'; profile="$2"; shift 2 ;;
    --replace-legacy) replace_legacy=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; fail "unknown argument: $1" ;;
  esac
done

[[ "$robot_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || fail 'portable --robot-id is required'
[ "${#hosts[@]}" -gt 0 ] || fail 'at least one --host is required'
for host in "${hosts[@]}"; do
  [[ "$host" =~ ^[A-Za-z0-9][A-Za-z0-9.-]*$ ]] || fail "invalid host: $host"
done
[ "$profile" = can-4ws ] || fail 'only the initial supported can-4ws profile is available'
[ -s "$token_file" ] || fail "paired console token is missing or empty: $token_file"

if [ -z "$console_python" ]; then
  console_python="$(command -v python3)" || fail 'python3 is not available'
fi
if [ -z "$controller_python" ]; then
  controller_python="$console_python"
fi
for interpreter in "$console_python" "$controller_python"; do
  [ -x "$interpreter" ] || fail "Python interpreter is not executable: $interpreter"
  [[ "$interpreter" = /* ]] || fail "Python interpreter must be an absolute path: $interpreter"
done

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
if ! PYTHONPATH="$repo_root${PYTHONPATH:+:$PYTHONPATH}" \
  "$console_python" -c '
import gi
gi.require_version("Gtk", "3.0")
gi.require_version("Gst", "1.0")
gi.require_version("Rsvg", "2.0")
from gi.repository import Gst
import operator_console
Gst.init(None)
assert Gst.ElementFactory.find("srtsrc")
assert Gst.ElementFactory.find("gtksink")
' >/dev/null 2>&1; then
  fail "console Python lacks operator_console GTK/GStreamer SRT/gtksink dependencies: $console_python"
fi
if ! PYGAME_HIDE_SUPPORT_PROMPT=1 \
  "$controller_python" -c 'import pygame' >/dev/null 2>&1; then
  fail "controller Python lacks pygame: $controller_python"
fi

legacy=0
if systemctl --user is-enabled --quiet powertrain-operator-console.service 2>/dev/null \
  || systemctl --user is-active --quiet powertrain-operator-console.service 2>/dev/null; then
  legacy=1
fi
if [ "$legacy" -eq 1 ] && [ "$replace_legacy" -ne 1 ]; then
  fail 'legacy Restart=always 콘솔이 활성화되어 있습니다. 충돌 방지를 위해 이 명령에 --replace-legacy를 추가하십시오.'
fi
if [ "$legacy" -eq 1 ]; then
  systemctl --user disable --now powertrain-operator-console.service
fi

config_dir="$HOME/.config/powertrain"
bin_dir="$HOME/.local/bin"
desktop_dir="$HOME/.local/share/applications"
mkdir -p "$config_dir" "$bin_dir" "$desktop_dir"

python3 - "$config_dir/operator.json" "$robot_id" "$token_file" "$controller_python" "$profile" "${hosts[@]}" <<'PY'
import json
from pathlib import Path
import sys

path, robot_id, token_file, controller_python, profile, *hosts = sys.argv[1:]
config = {
    "robot_id": robot_id,
    "hosts": hosts,
    "session_port": 9002,
    "token_file": str(Path(token_file).expanduser().resolve()),
    "controller_python": controller_python,
    "profile": profile,
}
Path(path).write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
PY
chmod 0600 "$config_dir/operator.json"

launcher="$bin_dir/powertrain-integrated-console"
printf '#!/usr/bin/env bash\nset -euo pipefail\ncd %q\nexec %q -m operator_console\n' \
  "$repo_root" "$console_python" >"$launcher"
chmod 0755 "$launcher"

desktop="$desktop_dir/powertrain-integrated-console.desktop"
{
  printf '[Desktop Entry]\n'
  printf 'Type=Application\n'
  printf 'Name=Powertrain Integrated Console\n'
  printf 'Comment=ZETIN integrated robot operation\n'
  printf 'Exec=%s\n' "$launcher"
  printf 'Terminal=false\n'
  printf 'Categories=Utility;\n'
} >"$desktop"
chmod 0644 "$desktop"

printf 'integrated console install PASS: %s\n' "$desktop"
