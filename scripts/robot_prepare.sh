#!/usr/bin/env bash
# One-time build and provisioning for scripts/robot-start.
set -euo pipefail

fail() {
  printf 'robot_prepare FAIL: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
usage: scripts/robot_prepare.sh --robot-id ID --token-file ABSOLUTE_PATH
       [--drive-ops-token-file ABSOLUTE_PATH] [--lease-timeout SEC]
       [--with-arm-d435-bridge]
       [--replace-legacy-telemetry]
       [--use-existing-images]
       [--optional-compose-file PATH --optional-profile NAME
        --optional-service NAME ...]

The optional stack is only for explicitly prepared existing D435/arm observation
services. Service names containing autonomy or teleop are rejected.
--use-existing-images requires locally prepared runtime images; ROS source is
still rebuilt. Without this flag, image build failures remain fatal.
EOF
}

robot_id=""
token_file=""
drive_ops_token_file=""
lease_timeout=2.0
with_arm_bridge=0
replace_legacy_telemetry=0
use_existing_images=0
optional_compose_file=""
optional_profiles=()
optional_services=()

while [ "$#" -gt 0 ]; do
  case "$1" in
    --robot-id) [ "$#" -ge 2 ] || fail '--robot-id requires a value'; robot_id="$2"; shift 2 ;;
    --token-file) [ "$#" -ge 2 ] || fail '--token-file requires a value'; token_file="$2"; shift 2 ;;
    --drive-ops-token-file) [ "$#" -ge 2 ] || fail '--drive-ops-token-file requires a value'; drive_ops_token_file="$2"; shift 2 ;;
    --lease-timeout) [ "$#" -ge 2 ] || fail '--lease-timeout requires a value'; lease_timeout="$2"; shift 2 ;;
    --with-arm-d435-bridge) with_arm_bridge=1; shift ;;
    --replace-legacy-telemetry) replace_legacy_telemetry=1; shift ;;
    --use-existing-images) use_existing_images=1; shift ;;
    --optional-compose-file) [ "$#" -ge 2 ] || fail '--optional-compose-file requires a value'; optional_compose_file="$2"; shift 2 ;;
    --optional-profile) [ "$#" -ge 2 ] || fail '--optional-profile requires a value'; optional_profiles+=("$2"); shift 2 ;;
    --optional-service) [ "$#" -ge 2 ] || fail '--optional-service requires a value'; optional_services+=("$2"); shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; fail "unknown argument: $1" ;;
  esac
done

[[ "$robot_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || fail 'robot ID is required and must be portable'
[[ "$token_file" = /* ]] || fail '--token-file must be an absolute path'
if [ -n "$drive_ops_token_file" ]; then
  [[ "$drive_ops_token_file" = /* ]] || fail '--drive-ops-token-file must be an absolute path'
fi
python3 - "$lease_timeout" <<'PY' || fail '--lease-timeout must be between 0.2 and 30 seconds'
import math, sys
value = float(sys.argv[1])
raise SystemExit(0 if math.isfinite(value) and .2 <= value <= 30 else 1)
PY

for item in "${optional_profiles[@]}" "${optional_services[@]}"; do
  [ -z "$item" ] && continue
  [[ "$item" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || fail "invalid optional name: $item"
done
for service in "${optional_services[@]}"; do
  case "$service" in
    *autonomy*|*teleop*) fail "unfinished autonomy/arm teleop service is forbidden: $service" ;;
  esac
done
if [ -n "$optional_compose_file" ]; then
  [ -f "$optional_compose_file" ] || fail "optional compose file not found: $optional_compose_file"
  [ "${#optional_services[@]}" -gt 0 ] || fail 'optional compose file requires --optional-service'
elif [ "${#optional_profiles[@]}" -gt 0 ] || [ "${#optional_services[@]}" -gt 0 ]; then
  fail '--optional-profile/--optional-service requires --optional-compose-file'
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
cd "$repo_root"
target_root="${POWERTRAIN_ROOT:-}"
etc_dir="${target_root}/etc/powertrain"
if [ -z "$target_root" ] && [ "$EUID" -ne 0 ]; then
  fail 'one-time Jetson provisioning must run as root (sudo bash scripts/robot_prepare.sh ...)'
fi

actual_path() {
  if [ -n "$target_root" ]; then
    printf '%s/%s\n' "$target_root" "${1#/}"
  else
    printf '%s\n' "$1"
  fi
}

actual_token="$(actual_path "$token_file")"
[ -s "$actual_token" ] || fail "paired console token is missing or empty: $actual_token"
if [ -n "$drive_ops_token_file" ]; then
  actual_ops_token="$(actual_path "$drive_ops_token_file")"
  [ -s "$actual_ops_token" ] || fail "drive ops token is missing or empty: $actual_ops_token"
fi
env_file="$etc_dir/powertrain.env"
[ -f "$env_file" ] || fail "explicit STOP_MM environment is missing: $env_file"
PYTHONPATH="$repo_root/ros2/src/powertrain_ros" python3 - "$env_file" "$etc_dir" <<'PY' \
  || fail 'STOP_MM/provenance or existing ops token validation failed'
from pathlib import Path
import sys
from powertrain_ros.preflight import check_preflight, load_env_file

env_path, token_dir = Path(sys.argv[1]), Path(sys.argv[2])
result = check_preflight(load_env_file(env_path), lambda: {p.name for p in token_dir.iterdir() if p.is_file()})
for warning in result.warnings:
    print(f"powertrain preflight warning: {warning}", file=sys.stderr)
for failure in result.failures:
    print(f"powertrain preflight failed: {failure}", file=sys.stderr)
raise SystemExit(0 if result.ok else 1)
PY

# Fail before rebuilding installed code or replacing active session configuration.
# can_setup independently holds the owner lock through every link mutation, so
# a later owner startup cannot turn this early check into unsafe CAN down/up.
if [ -d "${target_root}/run/powertrain" ]; then
  python3 "$repo_root/motor_control/chassis/can_interface.py" --require-idle \
    --lock-path "${target_root}/run/powertrain/can0.lock" \
    || fail '활성 CAN/USB owner를 먼저 확인·정지한 뒤 준비 작업을 다시 실행하십시오.'
fi

if [ "$use_existing_images" -eq 1 ]; then
  docker image inspect powertrain-sw:jetson powertrain-sw:ros >/dev/null \
    || fail 'existing runtime images are required: powertrain-sw:jetson and powertrain-sw:ros'
fi

legacy_telemetry_units=()
for unit in \
  powertrain-chassis-telemetry.service \
  powertrain-pdist80b-telemetry.service; do
  if systemctl is-enabled --quiet "$unit" 2>/dev/null \
    || systemctl is-active --quiet "$unit" 2>/dev/null; then
    legacy_telemetry_units+=("$unit")
  fi
done
if [ "${#legacy_telemetry_units[@]}" -gt 0 ] \
  && [ "$replace_legacy_telemetry" -ne 1 ]; then
  fail "legacy telemetry service conflicts with integrated ownership (${legacy_telemetry_units[*]}). Re-run with --replace-legacy-telemetry."
fi
if [ "${#legacy_telemetry_units[@]}" -gt 0 ]; then
  if [ -n "$target_root" ]; then
    for unit in "${legacy_telemetry_units[@]}"; do
      systemctl disable --now "$unit"
    done
  else
    for unit in "${legacy_telemetry_units[@]}"; do
      sudo systemctl disable --now "$unit"
    done
  fi
fi

if [ -n "$target_root" ]; then
  mkdir -p "$etc_dir" "$target_root/run/powertrain" "$target_root/var/lib/powertrain"
else
  sudo bash scripts/install_powertrain_runtime_dir.sh
  sudo bash scripts/install_bringup_preflight.sh
fi

# The legacy unit embeds /home/zetin/power-train-sw. Override only its path
# fields so the prepared checkout is also the checkout used after reboot.
dropin_tmp="$(mktemp)"
trap 'rm -f "$dropin_tmp"' EXIT
{
  printf '[Service]\n'
  printf 'WorkingDirectory=%s\n' "$repo_root"
  printf 'Environment="PYTHONPATH=%s/ros2/src/powertrain_ros"\n' "$repo_root"
} >"$dropin_tmp"
dropin_relative="etc/systemd/system/powertrain-bringup-preflight.service.d/integrated-repo.conf"
if [ -n "$target_root" ]; then
  install -D -m 0644 "$dropin_tmp" "$target_root/$dropin_relative"
else
  sudo install -D -o root -g root -m 0644 "$dropin_tmp" "/$dropin_relative"
  sudo systemctl daemon-reload
fi

compose=(docker compose -f docker/docker-compose.jetson.yml -f docker/docker-compose.integrated.yml)
if [ "$use_existing_images" -eq 1 ]; then
  printf 'Reusing existing runtime images; rebuilding installed ROS source.\n'
else
  "${compose[@]}" build powertrain powertrain_ros
fi
"${compose[@]}" run --rm --no-deps --entrypoint /bin/bash powertrain_control -lc \
  'set -e; source /opt/ros/humble/setup.bash; set -u; cd /workspace/ros2; colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros'

config_tmp="$(mktemp)"
manifest_tmp="$(mktemp)"
trap 'rm -f "$config_tmp" "$manifest_tmp" "$dropin_tmp"' EXIT
python3 - "$config_tmp" "$robot_id" "$token_file" "$lease_timeout" "$drive_ops_token_file" <<'PY'
import json
from pathlib import Path
import sys

path, robot_id, token_file, lease_timeout, ops_token_file = sys.argv[1:]
config = {
    "robot_id": robot_id,
    "token_file": token_file,
    "host": "0.0.0.0",
    "session_port": 9002,
    "input_port": 9000,
    "ops_port": 9001,
    "input_target_port": 19000,
    "ops_target_port": 19001,
    "destination_file": "/run/powertrain/operator-session.json",
    "lease_timeout_s": float(lease_timeout),
}
if ops_token_file:
    config["ops_token_file"] = ops_token_file
Path(path).write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
PY

{
  printf 'POWERTRAIN_INTEGRATED_VERSION=1\n'
  printf 'POWERTRAIN_PROFILE=can-4ws\n'
  if [ "$with_arm_bridge" -eq 1 ]; then
    printf 'INTEGRATED_PROFILES=arm-d435\n'
    printf 'INTEGRATED_SERVICES=powertrain_arm_console_bridge\n'
  else
    printf 'INTEGRATED_PROFILES=\nINTEGRATED_SERVICES=\n'
  fi
  printf 'OPTIONAL_COMPOSE_FILE=%q\n' "$optional_compose_file"
  printf 'OPTIONAL_PROFILES=%q\n' "${optional_profiles[*]}"
  printf 'OPTIONAL_SERVICES=%q\n' "${optional_services[*]}"
} >"$manifest_tmp"

if [ -n "$target_root" ]; then
  install -D -m 0644 "$config_tmp" "$etc_dir/robot.json"
  install -D -m 0644 "$manifest_tmp" "$etc_dir/integrated-prepared.env"
else
  sudo install -D -o root -g root -m 0644 "$config_tmp" /etc/powertrain/robot.json
  sudo install -D -o root -g root -m 0644 "$manifest_tmp" /etc/powertrain/integrated-prepared.env
  sudo systemctl start powertrain-bringup-preflight.service
fi

printf 'robot_prepare PASS: profile=can-4ws; daily command: scripts/robot-start\n'
