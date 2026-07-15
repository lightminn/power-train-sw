#!/usr/bin/env bash
set -euo pipefail

fail() {
  printf 'PDIST80B telemetry install FAIL: %s\n' "$*" >&2
  exit 1
}

if [ "$EUID" -ne 0 ]; then
  fail 'must run as root'
fi

if [ "$#" -ne 1 ]; then
  fail 'usage: sudo bash scripts/install_pdist80b_telemetry_service.sh OPERATOR_HOST'
fi

operator_host="$1"
case "$operator_host" in
  *[!0-9.]* | '') fail 'OPERATOR_HOST must be an IPv4 address' ;;
esac

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unit_source="$repo_root/scripts/systemd/powertrain-pdist80b-telemetry.service"
udev_rule_source="$repo_root/scripts/systemd/99-powertrain-pdist80b.rules"
sender="$repo_root/scripts/pdist80b_telemetry_sender.py"
unit_destination=/etc/systemd/system/powertrain-pdist80b-telemetry.service
environment_destination=/etc/default/powertrain-pdist80b-telemetry
udev_rule_destination=/etc/udev/rules.d/99-powertrain-pdist80b.rules

[ -f "$unit_source" ] || fail "missing unit: $unit_source"
[ -f "$udev_rule_source" ] || fail "missing udev rule: $udev_rule_source"
[ -f "$sender" ] || fail "missing sender: $sender"

install -D -o root -g root -m 0644 "$unit_source" "$unit_destination"
install -D -o root -g root -m 0644 "$udev_rule_source" "$udev_rule_destination"
install -D -o root -g root -m 0644 /dev/null "$environment_destination"
cat >"$environment_destination" <<EOF
OPERATOR_HOST=$operator_host
OPERATOR_PORT=5004
PDIST_PORT=/dev/powertrain-pdist80b
PDIST_DEVICE_ID=1
PDIST_HZ=2.0
EOF

udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
systemctl daemon-reload
systemctl enable powertrain-pdist80b-telemetry.service
systemctl restart powertrain-pdist80b-telemetry.service
systemctl status --no-pager powertrain-pdist80b-telemetry.service || true
