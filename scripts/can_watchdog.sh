#!/usr/bin/env bash
# Host wrapper uses the same owner-aware recovery as the Compose service.
# It never resets an active controller; that owner calls step() after ESTOP.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
channel="${1:-can0}"
if [ "$#" -gt 0 ]; then shift; fi
export PYTHONPATH="$script_dir/../motor_control${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -u -m corner_module.can_watchdog --channel "$channel" "$@"
