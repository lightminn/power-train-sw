#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C

fail() {
  printf 'powertrain runtime-dir install FAIL: %s\n' "$*" >&2
  exit 1
}

if [ "$EUID" -ne 0 ]; then
  fail "must run as root"
fi

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SOURCE="$SCRIPT_DIR/../docker/powertrain-gateway-tmpfiles.conf"
readonly DESTINATION="/etc/tmpfiles.d/powertrain-gateway.conf"
# Shared by the L515 gateway lock and the real SocketCAN owner locks.
# This script remains the only authority that creates /run/powertrain.
readonly RUNTIME_DIR="/run/powertrain"
# Mission IDs must survive reboot; unlike /run this directory is installed
# directly and is never delegated to systemd-tmpfiles.
readonly PERSISTENT_DIR="/var/lib/powertrain"
# Per-daemon append-only mission journals live below the persistent root.
readonly RUNS_DIR="/var/lib/powertrain/runs"

[ -f "$SOURCE" ] || fail "missing tmpfiles source: $SOURCE"
command -v install >/dev/null 2>&1 || fail "install command is unavailable"
command -v systemd-tmpfiles >/dev/null 2>&1 \
  || fail "systemd-tmpfiles is unavailable"
command -v stat >/dev/null 2>&1 || fail "stat command is unavailable"

install -D -o root -g root -m 0644 "$SOURCE" "$DESTINATION"
systemd-tmpfiles --create "$DESTINATION"

[ -d "$RUNTIME_DIR" ] || fail "$RUNTIME_DIR was not created"
actual="$(stat -c '%U:%G:%a:%F' "$RUNTIME_DIR")"
[ "$actual" = "root:root:750:directory" ] \
  || fail "$RUNTIME_DIR must be root:root mode 0750 directory; got $actual"

install -d -o root -g root -m 0750 "$PERSISTENT_DIR"
[ -d "$PERSISTENT_DIR" ] || fail "$PERSISTENT_DIR was not created"
persistent_actual="$(stat -c '%U:%G:%a:%F' "$PERSISTENT_DIR")"
[ "$persistent_actual" = "root:root:750:directory" ] \
  || fail "$PERSISTENT_DIR must be root:root mode 0750 directory; got $persistent_actual"

install -d -o root -g root -m 0750 "$RUNS_DIR"
[ -d "$RUNS_DIR" ] || fail "$RUNS_DIR was not created"
runs_actual="$(stat -c '%U:%G:%a:%F' "$RUNS_DIR")"
[ "$runs_actual" = "root:root:750:directory" ] \
  || fail "$RUNS_DIR must be root:root mode 0750 directory; got $runs_actual"

printf 'powertrain runtime-dir install PASS: runtime=%s persistent=%s runs=%s\n' \
  "$actual" "$persistent_actual" "$runs_actual"
