#!/usr/bin/env bash
# Install the Jetson read-only chassis telemetry sender.
set -euo pipefail

fail() {
  printf 'chassis telemetry install FAIL: %s\n' "$*" >&2
  exit 1
}

if [ "$EUID" -ne 0 ]; then
  fail 'must run as root'
fi
if [ "$#" -ne 1 ]; then
  fail 'usage: sudo bash scripts/install_chassis_telemetry_service.sh OPERATOR_IPV4'
fi

operator_host="$1"
case "$operator_host" in
  *[!0-9.]* | '') fail 'OPERATOR_IPV4 must be an IPv4 address' ;;
esac

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unit_source="$repo_root/docker/powertrain-chassis-telemetry.service"
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
