#!/usr/bin/env bash
# Install the local read-only operator GUI as a user service.
set -euo pipefail

fail() {
  printf 'operator console install FAIL: %s\n' "$*" >&2
  exit 1
}

if [ "$#" -ne 1 ]; then
  fail 'usage: bash scripts/install_operator_console_service.sh JETSON_IPV4'
fi

operator_host="$1"
case "$operator_host" in
  *[!0-9.]* | '') fail 'JETSON_IPV4 must be an IPv4 address' ;;
esac

for variable in DISPLAY WAYLAND_DISPLAY XAUTHORITY; do
  [ -n "${!variable:-}" ] || fail "$variable is not set; run from the graphical login session"
done

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unit_source="$repo_root/scripts/systemd/powertrain-operator-console.service"
unit_dir="$HOME/.config/systemd/user"
config_dir="$HOME/.config/powertrain"
environment_file="$config_dir/operator-console.env"

[ -f "$unit_source" ] || fail "missing unit: $unit_source"
mkdir -p "$unit_dir" "$config_dir"
install -m 0644 "$unit_source" "$unit_dir/powertrain-operator-console.service"
cat >"$environment_file" <<EOF
DISPLAY=$DISPLAY
WAYLAND_DISPLAY=$WAYLAND_DISPLAY
XAUTHORITY=$XAUTHORITY
OPERATOR_HOST=$operator_host
EOF

systemctl --user import-environment DISPLAY WAYLAND_DISPLAY XAUTHORITY
systemctl --user daemon-reload
systemctl --user enable powertrain-operator-console.service
systemctl --user restart powertrain-operator-console.service
systemctl --user --no-pager --full status powertrain-operator-console.service
