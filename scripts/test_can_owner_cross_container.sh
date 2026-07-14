#!/usr/bin/env bash
set -euo pipefail

readonly LOCK_PATH="/run/powertrain/can0.lock"
readonly TOKEN="can-owner-test-$$"
readonly READY_PATH="/run/powertrain/${TOKEN}.ready"
readonly STOP_PATH="/run/powertrain/${TOKEN}.stop"
readonly PYTHONPATH_VALUE="/workspace/motor_control"

fail() {
  printf 'cross-container CAN owner test FAIL: %s\n' "$*" >&2
  exit 1
}

cleanup() {
  docker exec powertrain_ros touch "$STOP_PATH" >/dev/null 2>&1 || true
  sleep 0.1
  docker exec powertrain_ros rm -f "$READY_PATH" "$STOP_PATH" \
    >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

[ -d /run/powertrain ] \
  || fail "/run/powertrain missing; run scripts/install_powertrain_runtime_dir.sh"

for container in powertrain_jetson powertrain_ros; do
  running="$(docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null || true)"
  [ "$running" = "true" ] || fail "$container is not running"
done

docker exec powertrain_ros rm -f "$READY_PATH" "$STOP_PATH"

readonly HOLDER_CODE='import pathlib, sys, time
from chassis.runtime_lock import RealCanSession
ready, stop = map(pathlib.Path, sys.argv[1:3])
with RealCanSession(channel="can0", owner="cross_container_holder"):
    ready.write_text("ready\n", encoding="utf-8")
    while not stop.exists():
        time.sleep(0.05)'

docker exec -d \
  -e "PYTHONPATH=$PYTHONPATH_VALUE" \
  powertrain_jetson \
  python3 -c "$HOLDER_CODE" "$READY_PATH" "$STOP_PATH"

for _ in $(seq 1 100); do
  docker exec powertrain_ros test -f "$READY_PATH" 2>/dev/null && break
  sleep 0.05
done
docker exec powertrain_ros test -f "$READY_PATH" 2>/dev/null \
  || fail "powertrain_jetson holder did not become ready"

readonly CONTENDER_CODE='from chassis.runtime_lock import CanOwnershipError, RealCanSession
try:
    with RealCanSession(channel="can0", owner="cross_container_contender"):
        pass
except CanOwnershipError as exc:
    print(f"EXPECTED CanOwnershipError: {exc}")
    raise SystemExit(23)
raise SystemExit("unexpectedly acquired held CAN lock")'

set +e
contender_output="$(docker exec \
  -e "PYTHONPATH=$PYTHONPATH_VALUE" \
  powertrain_ros \
  python3 -c "$CONTENDER_CODE" 2>&1)"
contender_status=$?
set -e
printf '%s\n' "$contender_output"
[ "$contender_status" -eq 23 ] \
  || fail "contender exit=$contender_status (expected 23)"
printf '%s' "$contender_output" | grep -q 'EXPECTED CanOwnershipError' \
  || fail "contender did not report CanOwnershipError"

docker exec powertrain_ros touch "$STOP_PATH"

readonly REACQUIRE_CODE='from chassis.runtime_lock import RealCanSession
with RealCanSession(channel="can0", owner="cross_container_reacquire") as session:
    print(f"REACQUIRED pid={session.owner_snapshot.pid} path={session.owner_snapshot.lock_path}")'

reacquired=""
for _ in $(seq 1 100); do
  if reacquired="$(docker exec \
      -e "PYTHONPATH=$PYTHONPATH_VALUE" \
      powertrain_ros \
      python3 -c "$REACQUIRE_CODE" 2>/dev/null)"; then
    break
  fi
  sleep 0.05
done
printf '%s\n' "$reacquired"
printf '%s' "$reacquired" | grep -q "REACQUIRED .*path=$LOCK_PATH" \
  || fail "powertrain_ros did not reacquire after holder exit"

printf 'cross-container CAN owner test PASS\n'
