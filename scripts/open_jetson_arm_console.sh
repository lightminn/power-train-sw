#!/usr/bin/env bash
# Open the console on this laptop against the Jetson arm bench (spec §2.6).
# The broker listens on Jetson 127.0.0.1:29001 only, so it is reached through
# an SSH tunnel. The Jetson mirror must send UDP to this laptop
# (arm_console_bridge console_host:=<laptop IP>).
# Optional: ARM_PAD_PYTHONPATH=<dir with pygame> when the system python lacks it.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
jetson="${JETSON:-zetin@jetson-orin.local}"
session="${ARM_SESSION:-pr12-arm-session}"
token_dir="${XDG_RUNTIME_DIR:-/tmp}/jetson-arm-console"
token="$token_dir/ops_console.token"

mkdir -p -m 700 "$token_dir"
(umask 077 && scp -q "$jetson:$session/ops_console.token" "$token")
if ! ss -ltn | grep -q '127.0.0.1:29001 '; then
    ssh -f -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=5 \
        -L 29001:127.0.0.1:29001 "$jetson"
fi

export ARM_LOCAL_ALL_AXES=1 ARM_LOCAL_WRIST_IGNORE_LOCK=1
export PYTHONPATH="$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros${ARM_PAD_PYTHONPATH:+:$ARM_PAD_PYTHONPATH}"
exec /usr/bin/python3 -m operator_console.app \
    --host "${jetson#*@}" --ops-host 127.0.0.1 --ops-port 29001 \
    --ops-token-file "$token" --arm-telemetry-port 15707 --metadata-port 15703
