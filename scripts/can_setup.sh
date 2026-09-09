#!/usr/bin/env bash
# Healthy repeats are read-only. Reconfiguration requires exclusive CAN ownership.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
lock_path="${POWERTRAIN_ROOT:-}/run/powertrain/can0.lock"
exec sudo python3 "$script_dir/../motor_control/chassis/can_interface.py" \
  --setup --lock-path "$lock_path" "$@"
