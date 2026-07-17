#!/usr/bin/env bash
# Install the Jetson boot-time CAN and environment preflight.
set -euo pipefail

fail() {
  printf 'bring-up preflight install FAIL: %s\n' "$*" >&2
  exit 1
}

if [ "$EUID" -ne 0 ]; then
  fail 'must run as root'
fi
if [ "$#" -ne 0 ]; then
  fail 'usage: sudo bash scripts/install_bringup_preflight.sh'
fi

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unit_source="$repo_root/scripts/systemd/powertrain-bringup-preflight.service"
unit_destination=/etc/systemd/system/powertrain-bringup-preflight.service
environment_destination=/etc/powertrain/powertrain.env

[ -f "$unit_source" ] || fail "missing unit: $unit_source"

install -D -o root -g root -m 0644 "$unit_source" "$unit_destination"
if [ ! -e "$environment_destination" ]; then
  install -D -o root -g root -m 0640 /dev/null "$environment_destination"
else
  chown root:root "$environment_destination"
  chmod 0640 "$environment_destination"
fi

systemctl daemon-reload
systemctl enable powertrain-bringup-preflight.service

# Do not create a separate stack auto-start unit. Compose restart policy owns
# stack resurrection after commissioning has deliberately left it enabled.
printf 'bring-up preflight installed; provision %s before starting it\n' \
  "$environment_destination"
