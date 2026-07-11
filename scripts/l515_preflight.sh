#!/usr/bin/env bash
set -euo pipefail

readonly USB_ID="8086:0b64"
readonly EXPECTED_SERIAL="00000000F0271544"
readonly MIN_USB_SPEED_MBPS=5000
readonly SYSFS_ROOT="${L515_SYSFS_ROOT:-/sys/bus/usb/devices}"

fail() {
  printf 'L515 preflight FAIL: %s\n' "$*" >&2
  exit 1
}

command -v lsusb >/dev/null 2>&1 || fail "lsusb is unavailable"
command -v docker >/dev/null 2>&1 || fail "docker is unavailable"

mapfile -t usb_rows < <(lsusb -d "$USB_ID" 2>/dev/null || true)
if [ "${#usb_rows[@]}" -ne 1 ]; then
  fail "expected exactly one USB device $USB_ID, found ${#usb_rows[@]}"
fi

if [[ "${usb_rows[0]}" =~ ^Bus[[:space:]]+([0-9]+)[[:space:]]+Device[[:space:]]+([0-9]+): ]]; then
  bus=$((10#${BASH_REMATCH[1]}))
  device=$((10#${BASH_REMATCH[2]}))
else
  fail "cannot parse lsusb row: ${usb_rows[0]}"
fi

speed_file=""
for candidate in "$SYSFS_ROOT"/*; do
  [ -f "$candidate/busnum" ] || continue
  [ -f "$candidate/devnum" ] || continue
  [ "$(<"$candidate/busnum")" -eq "$bus" ] || continue
  [ "$(<"$candidate/devnum")" -eq "$device" ] || continue
  [ -f "$candidate/speed" ] || fail "USB sysfs speed is missing for bus $bus device $device"
  speed_file="$candidate/speed"
  break
done
[ -n "$speed_file" ] || fail "USB sysfs device is missing for bus $bus device $device"

speed=$(<"$speed_file")
[[ "$speed" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail "invalid USB sysfs speed: $speed"
awk -v speed="$speed" -v minimum="$MIN_USB_SPEED_MBPS" \
  'BEGIN { exit !(speed >= minimum) }' \
  || fail "USB link must be >= ${MIN_USB_SPEED_MBPS} Mbps, got ${speed} Mbps"

sdk_serials=$(docker exec -i powertrain_ros \
  python3 /workspace/scripts/l515_sdk_probe.py --serial "$EXPECTED_SERIAL") \
  || fail "SDK enumeration failed in powertrain_ros for serial $EXPECTED_SERIAL"

[ "$sdk_serials" = "$EXPECTED_SERIAL" ] \
  || fail "SDK must select only L515 serial $EXPECTED_SERIAL; got '${sdk_serials:-none}'"

printf 'L515 preflight PASS: %s, %s Mbps, SDK serial %s\n' \
  "$USB_ID" "$speed" "$EXPECTED_SERIAL"
