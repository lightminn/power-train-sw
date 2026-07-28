#!/usr/bin/env bash
set -euo pipefail

fail() {
  printf 'PDIST80B telemetry install FAIL: %s\n' "$*" >&2
  exit 1
}

usage='usage: sudo bash scripts/install_pdist80b_telemetry_service.sh [--operator-host IPV4] PDIST_ID_PATH'
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=operator_defaults.sh
source "$script_dir/operator_defaults.sh"

operator_host="$DEFAULT_OPERATOR_HOST"
operator_host_source=default
case "$#" in
  1)
    case "$1" in
      --help | -h)
        printf '%s\n' "$usage"
        exit 0
        ;;
      --*) fail "$usage" ;;
      *) pdist_id_path="$1" ;;
    esac
    ;;
  2)
    case "$1" in
      --*) fail "$usage" ;;
      *)
        operator_host="$1"
        operator_host_source=positional
        pdist_id_path="$2"
        ;;
    esac
    ;;
  3)
    [ "$1" = --operator-host ] || fail "$usage"
    operator_host="$2"
    operator_host_source=--operator-host
    pdist_id_path="$3"
    ;;
  *) fail "$usage" ;;
esac

case "$operator_host" in
  *[!0-9.]* | '') fail 'OPERATOR_HOST must be an IPv4 address' ;;
esac
case "$pdist_id_path" in
  *[!A-Za-z0-9._:/-]* | '') fail 'PDIST_ID_PATH contains unsupported characters' ;;
esac
printf 'operator host: %s (%s)\n' "$operator_host" "$operator_host_source"

if [ "$EUID" -ne 0 ]; then
  fail 'must run as root'
fi

repo_root="$(cd -- "$script_dir/.." && pwd)"
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
rendered_udev_rule="$(mktemp)"
trap 'rm -f "$rendered_udev_rule"' EXIT
sed "s|@PDIST_ID_PATH@|$pdist_id_path|g" \
  "$udev_rule_source" >"$rendered_udev_rule"
install -D -o root -g root -m 0644 "$rendered_udev_rule" "$udev_rule_destination"
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
if ! systemctl is-active --quiet powertrain-pdist80b-telemetry.service; then
  systemctl status --no-pager powertrain-pdist80b-telemetry.service || true
  journalctl --no-pager -u powertrain-pdist80b-telemetry.service -n 50 || true
  fail 'installed service is not active'
fi
systemctl status --no-pager powertrain-pdist80b-telemetry.service
