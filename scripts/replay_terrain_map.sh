#!/usr/bin/env bash
# 노트북 — L515 지형 지도 녹화 재생 (스펙 docs/specs/2026-10-17-l515-terrain-map-design.md §6).
#
# 두 가지 방식이 있다. 콘솔은 같은 노트북에서 :5010으로 받는다.
#   결정적 재생(기본, ROS 통신 없음 — 같은 런타임에 stamp 순으로 넣는다):
#     scripts/replay_terrain_map.sh <bag> [--decisions-from processed.jsonl] [--rate 1.0] [그 밖의 terrain_map_replay 옵션]
#   ROS 재생(노드 자체 시험 — 배율 1.0만):
#     scripts/replay_terrain_map.sh --ros <bag>
#
# 재생 bag의 /wheel_states가 실제 텔레메트리 송신기 같은 다른 구독자에 들어가지 않도록
# ROS_DOMAIN_ID를 $TERRAIN_REPLAY_DOMAIN(기본 78)으로 강제하고, 그 값이 생산 도메인
# ($PRODUCTION_ROS_DOMAIN_ID, 기본 0 — 통합 compose가 도메인을 정하지 않음)과 같으면 거부한다.
# ROS 환경과 powertrain_ros 설치 공간이 잡힌 셸(로컬 ROS 시험 이미지 등)에서 실행한다.
set -euo pipefail

domain="${TERRAIN_REPLAY_DOMAIN:-78}"
production="${PRODUCTION_ROS_DOMAIN_ID:-0}"
console="${TERRAIN_MAP_CONSOLE:-127.0.0.1:5010}"
if [[ "$domain" == "$production" ]]; then
  echo "refusing: replay domain $domain equals the production ROS domain" >&2
  exit 2
fi
export ROS_DOMAIN_ID="$domain"

if [[ "${1:-}" == "--ros" ]]; then
  bag="${2:?usage: $0 --ros <bag>}"
  shift 2
  if (( $# )); then
    echo "refusing: ROS replay takes no options (rate 1.0 only — receive-time rules, spec §6)" >&2
    exit 2
  fi
  host="${console%:*}"
  port="${console##*:}"
  ros2 run powertrain_ros terrain_map --ros-args -p operator_host:="$host" -p operator_port:="$port" &
  node=$!
  trap 'kill -INT "$node" 2>/dev/null || true; wait "$node" 2>/dev/null || true' EXIT
  sleep 2
  ros2 bag play --rate 1.0 --delay 2.0 "$bag"   # --delay: match subscribers before the first message
  exit 0
fi

bag="${1:?usage: $0 <bag> [terrain_map_replay options] | --ros <bag>}"
shift
exec python3 -m powertrain_ros.terrain_map_replay "$bag" --send "$console" "$@"
