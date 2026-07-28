#!/usr/bin/env bash
# Install the Jetson read-only chassis telemetry sender.
set -euo pipefail

fail() {
  printf 'chassis telemetry install FAIL: %s\n' "$*" >&2
  exit 1
}

usage='usage: sudo bash scripts/install_chassis_telemetry_service.sh [--operator-host IPV4]'
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=operator_defaults.sh
source "$script_dir/operator_defaults.sh"

operator_host="$DEFAULT_OPERATOR_HOST"
operator_host_source=default
case "$#" in
  0) ;;
  1)
    case "$1" in
      --help | -h)
        printf '%s\n' "$usage"
        exit 0
        ;;
      --*) fail "$usage" ;;
      *)
        operator_host="$1"
        operator_host_source=positional
        ;;
    esac
    ;;
  2)
    [ "$1" = --operator-host ] || fail "$usage"
    operator_host="$2"
    operator_host_source=--operator-host
    ;;
  *) fail "$usage" ;;
esac

case "$operator_host" in
  *[!0-9.]* | '') fail 'OPERATOR_IPV4 must be an IPv4 address' ;;
esac
printf 'operator host: %s (%s)\n' "$operator_host" "$operator_host_source"

if [ "$EUID" -ne 0 ]; then
  fail 'must run as root'
fi

repo_root="$(cd -- "$script_dir/.." && pwd)"
unit_source="$repo_root/scripts/systemd/powertrain-chassis-telemetry.service"
unit_destination=/etc/systemd/system/powertrain-chassis-telemetry.service
environment_destination=/etc/default/powertrain-chassis-telemetry

[ -f "$unit_source" ] || fail "missing unit: $unit_source"
install -D -o root -g root -m 0644 "$unit_source" "$unit_destination"
install -D -o root -g root -m 0644 /dev/null "$environment_destination"
cat >"$environment_destination" <<EOF
OPERATOR_HOST=$operator_host
OPERATOR_PORT=5005
EOF

systemctl daemon-reload
systemctl enable powertrain-chassis-telemetry.service
systemctl restart powertrain-chassis-telemetry.service
systemctl --no-pager --full status powertrain-chassis-telemetry.service
