#!/usr/bin/env bash
# Bring up the arm bench on the Jetson and open the console (spec §2.6).
# Run from the laptop:
#   ssh -Y -C -o ServerAliveInterval=5 zetin@jetson-orin.local \
#       '~/power-train-sw-pr12/scripts/start_jetson_arm_bench.sh'
# Starts the arm container, bridge, ops broker and console mirror when they are
# not running, then runs the console in the foreground. Closing the console
# leaves the bridge running: a clean bridge stop turns arm torque off.
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
session="$HOME/${ARM_SESSION:-pr12-arm-session}"
container="${ARM_CONTAINER:-arm_pr12_jetson}"
ros_env='source /opt/ros/humble/setup.bash && source /session/runtime/install/setup.bash &&
export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-} PYTHONUNBUFFERED=1'

say() { echo "[arm-bench] $*"; }
inside() { docker exec "$container" bash -c "$1"; }
running() { docker exec "$container" pgrep -f "$1" >/dev/null 2>&1; }
spawn() { docker exec -d "$container" bash -c "$ros_env && exec $2 > /session/$1.log 2>&1"; }

if pgrep -f "[o]perator_console.app --host 127.0.0.1 --ops-host 127.0.0.1 --ops-port 29001" >/dev/null; then
    say "콘솔이 이미 실행 중이다."; exit 1
fi
docker start "$container" >/dev/null || { say "컨테이너 $container 시작 실패"; exit 1; }

# The bridge looks for the servos only once at startup. One started before
# servo power (e.g. right after a robot power cycle) never recovers, so it is
# replaced. SIGKILL keeps the servo torque as it is.
bridge_ok() {
    inside 'grep -q "MoveIt Dynamixel bridge started" /session/bridge-rt.log 2>/dev/null &&
            ! grep -q "startup torque read failed" /session/bridge-rt.log'
}
start_bridge() {
    inside 'pkill -9 -f "[m]oveit_dynamixel_bridge"; sleep 1
            if [ -f /session/bridge-rt.log ]; then mv /session/bridge-rt.log /session/bridge-rt.log.prev; fi'
    spawn bridge-rt "ros2 run dynamixel_control moveit_dynamixel_bridge --ros-args --params-file /session/session.yaml"
    for _ in $(seq 1 30); do
        sleep 1
        inside 'grep -q "MoveIt Dynamixel bridge started" /session/bridge-rt.log 2>/dev/null' && return 0
        running "[m]oveit_dynamixel_bridge" || return 1
    done
    return 1
}

if running "[m]oveit_dynamixel_bridge" && bridge_ok; then
    say "브리지 정상 실행 중"
else
    ok=0
    for attempt in 1 2 3 4 5; do
        say "브리지 기동 ($attempt/5)"
        if start_bridge && bridge_ok; then ok=1; break; fi
        say "서보 응답 없음. 서보 전원을 확인한다. 3초 뒤 재시도"
        sleep 3
    done
    if [ "$ok" = 1 ]; then say "브리지 정상: 팔 5축 토크 확인"
    else say "⚠️ 브리지가 서보를 찾지 못했다. 콘솔은 띄우지만 팔은 미수신 상태다."; fi
fi

if ! running "[o]ps_broker"; then
    say "ops 브로커 기동"
    spawn ops-rt "env ARM_LOCAL_ALL_AXES=1 ros2 run powertrain_ros ops_broker --ros-args \
-p host:=127.0.0.1 -p port:=29001 -p token_dir:=/session"
fi
if ! running "[a]rm_console_bridge"; then
    say "콘솔 미러 기동"
    spawn mirror-rt "ros2 run powertrain_ros arm_console_bridge --ros-args -p console_host:=127.0.0.1 \
-p telemetry_port:=15707 -p metadata_port:=15703 -p send_detection_metadata:=false"
fi
sleep 3

gst="$session/gst"
export ARM_LOCAL_ALL_AXES=1 ARM_LOCAL_WRIST_IGNORE_LOCK=1
export GST_PLUGIN_PATH="$gst/plugins" LD_LIBRARY_PATH="$gst/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros"
say "콘솔 실행"
exec /usr/bin/python3 -m operator_console.app --host 127.0.0.1 --ops-host 127.0.0.1 \
    --ops-port 29001 --ops-token-file "$session/ops_console.token" \
    --arm-telemetry-port 15707 --metadata-port 15703
