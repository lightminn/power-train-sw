#!/usr/bin/env bash
# 젯슨 — L515 지형 지도 입력 녹화 (스펙 docs/specs/2026-10-17-l515-terrain-map-design.md §6).
#
# 지도 노드가 쓰는 원 입력 6개 토픽만 녹화한다(accel은 기준면 기울기 표시용). 자세는 재생 때 같은 평면 추측 항법으로
# 다시 계산하므로 /odom 등은 넣지 않는다. 압축 없음(CPU 절약).
#   depth 640×480×2 B × 10 Hz ≈ 6.1 MB/s ≈ 369 MB/분.
#
# 사용(ROS 환경이 잡힌 셸 — 예: powertrain-sw:ros 컨테이너 안): Ctrl+C로 끝낸다.
#   scripts/record_terrain_map_bag.sh <이름> [저장 폴더]
#   저장 폴더 기본값: $TERRAIN_BAG_DIR 또는 ~/terrain_bags
#   시작 전 여유 공간이 $TERRAIN_BAG_MIN_FREE_MB(기본 4000 MB, 약 10분) 이상인지 확인한다.
set -euo pipefail

TOPICS=(
  /l515/depth/image_rect_raw
  /l515/depth/camera_info
  /l515/color/camera_info
  /l515/gyro/sample
  /l515/accel/sample
  /wheel_states
)

name="${1:-}"
dir="${2:-${TERRAIN_BAG_DIR:-$HOME/terrain_bags}}"
min_free_mb="${TERRAIN_BAG_MIN_FREE_MB:-4000}"

if [[ ! "$name" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "usage: $0 <name: letters, digits, . _ -> [dir]" >&2
  exit 2
fi
mkdir -p "$dir"
if [[ -e "$dir/$name" ]]; then
  echo "refusing: $dir/$name already exists" >&2
  exit 2
fi
free_mb="$(df -Pm "$dir" | awk 'NR==2 {print $4}')"
if (( free_mb < min_free_mb )); then
  echo "refusing: ${free_mb} MB free in $dir, need ${min_free_mb} MB (≈369 MB per minute)" >&2
  exit 3
fi
echo "recording ${TOPICS[*]} → $dir/$name (${free_mb} MB free, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
exec ros2 bag record -o "$dir/$name" "${TOPICS[@]}"
