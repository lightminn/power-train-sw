#!/usr/bin/env bash
# Manual WP5.3 Task 7 bench wrapper. Production owners are replaced through
# their supervisor; the L515 Gateway RSUSB pipeline is never reused in-process.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
CORE="$SCRIPT_DIR/channel_matrix.py"
COMPOSE_FILE="${CHANNEL_MATRIX_COMPOSE_FILE:-$REPO_ROOT/docker/docker-compose.jetson.yml}"
export PYTHONPATH="$REPO_ROOT/ros2/src/powertrain_ros:$REPO_ROOT/motor_control:$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"

usage() {
  echo "usage: $0 --list | <channel> [--plan|--execute]" >&2
}

if [[ $# -eq 1 && "$1" == "--list" ]]; then
  "$PYTHON_BIN" "$CORE" --list
  exit 0
fi
if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 64
fi

channel="$1"
mode="${2:---plan}"
if [[ "$mode" != "--plan" && "$mode" != "--execute" ]]; then
  usage
  exit 64
fi

plan_json="$($PYTHON_BIN "$CORE" --channel "$channel")"
if [[ "$mode" == "--plan" ]]; then
  printf '%s\n' "$plan_json"
  exit 0
fi

if [[ "${CHANNEL_MATRIX_APPROVED:-}" != "YES" ]]; then
  echo "refusing a real kill: export CHANNEL_MATRIX_APPROVED=YES after bench approval" >&2
  exit 65
fi
if [[ -z "${CHANNEL_MATRIX_ASSERT_CMD:-}" || ! -x "${CHANNEL_MATRIX_ASSERT_CMD}" ]]; then
  echo "CHANNEL_MATRIX_ASSERT_CMD must be an executable hold/journal probe" >&2
  exit 66
fi

target_kind="$($PYTHON_BIN "$CORE" --channel "$channel" --field kill_kind)"
target_value="$($PYTHON_BIN "$CORE" --channel "$channel" --field kill_value)"
expected_holds="$($PYTHON_BIN "$CORE" --channel "$channel" --field expected_holds)"
expected_effect="$($PYTHON_BIN "$CORE" --channel "$channel" --field expected_effect)"
expected_journal="$($PYTHON_BIN "$CORE" --channel "$channel" --field expected_journal_event_type)"
replacement_timeout_s="${CHANNEL_MATRIX_REPLACEMENT_TIMEOUT_S:-30}"

old_pid=""
new_pid=""
if [[ "$target_kind" == "compose_service" ]]; then
  old_pid="$(docker compose -f "$COMPOSE_FILE" ps -q "$target_value" | xargs -r docker inspect -f '{{.State.Pid}}')"
  if [[ -z "$old_pid" || "$old_pid" == "0" ]]; then
    echo "compose target is not running: $target_value" >&2
    exit 67
  fi
  docker compose -f "$COMPOSE_FILE" kill -s SIGKILL "$target_value"
  # Compose creates the supervised replacement. Do not restart the Gateway's
  # RSUSB pipeline inside the killed process.
  docker compose -f "$COMPOSE_FILE" up -d --no-deps "$target_value"
  for ((attempt = 0; attempt < replacement_timeout_s; attempt++)); do
    new_pid="$(docker compose -f "$COMPOSE_FILE" ps -q "$target_value" | xargs -r docker inspect -f '{{.State.Pid}}')"
    if [[ -n "$new_pid" && "$new_pid" != "0" && "$new_pid" != "$old_pid" ]]; then
      break
    fi
    sleep 1
  done
  running_count="$(docker compose -f "$COMPOSE_FILE" ps -q "$target_value" | sed '/^$/d' | wc -l)"
else
  mapfile -t old_pids < <(pgrep -f -- "$target_value" || true)
  if [[ "${#old_pids[@]}" -ne 1 ]]; then
    echo "process target must have exactly one current owner: $target_value" >&2
    exit 68
  fi
  old_pid="${old_pids[0]}"
  kill -KILL "$old_pid"
  for ((attempt = 0; attempt < replacement_timeout_s; attempt++)); do
    mapfile -t replacement_pids < <(pgrep -f -- "$target_value" || true)
    if [[ "${#replacement_pids[@]}" -eq 1 && "${replacement_pids[0]}" != "$old_pid" ]]; then
      new_pid="${replacement_pids[0]}"
      break
    fi
    sleep 1
  done
  mapfile -t final_pids < <(pgrep -f -- "$target_value" || true)
  running_count="${#final_pids[@]}"
fi

if [[ -z "$new_pid" || "$new_pid" == "$old_pid" ]]; then
  echo "supervised replacement did not produce a new process" >&2
  exit 69
fi
if [[ "$running_count" -ne 1 ]]; then
  echo "orphan/overlap check failed: expected one replacement, found $running_count" >&2
  exit 70
fi

# The bench probe must assert operation holds/gates, the visible effect, and
# the required journal event. It returns nonzero on any mismatch.
"$CHANNEL_MATRIX_ASSERT_CMD" \
  "$channel" "$expected_holds" "$expected_effect" "$expected_journal" "0"

echo "PASS channel=$channel supervised replacement old_pid=$old_pid new_pid=$new_pid orphan_processes=0"
