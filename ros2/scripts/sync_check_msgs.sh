#!/bin/bash
# 벤더링된 robot_arm_msgs 가 로봇팔 팀 정본(ksp118/extreme-robot)과 일치하는지 확인.
# 그들이 .msg 를 바꾸면(= 계약 변경) 여기서 드리프트가 잡힌다.
#
# 사용 (Jetson, ~/extreme-robot 체크아웃 존재 가정):
#   bash ros2/scripts/sync_check_msgs.sh                 # 로컬 체크아웃 대비
#   bash ros2/scripts/sync_check_msgs.sh ~/extreme-robot
# ⚠️ 로컬 체크아웃이 origin/main 보다 뒤일 수 있음 — 정확히 하려면 먼저:
#   git -C ~/extreme-robot fetch origin && git -C ~/extreme-robot checkout origin/main -- ros2_ws/src/robot_arm_msgs/msg
set -e
ARM_REPO="${1:-$HOME/extreme-robot}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"                # ros2/
UPSTREAM="$ARM_REPO/ros2_ws/src/robot_arm_msgs/msg"
LOCAL="$HERE/src/robot_arm_msgs/msg"

if [ ! -d "$UPSTREAM" ]; then
    echo "❌ 정본 msg 폴더 없음: $UPSTREAM"
    echo "   로봇팔 레포 경로를 인자로 주세요: sync_check_msgs.sh <extreme-robot 경로>"
    exit 2
fi

if diff -rq "$UPSTREAM" "$LOCAL"; then
    echo "✅ robot_arm_msgs 동기화됨 (드리프트 없음)"
else
    echo ""
    echo "⚠️ 드리프트 감지 — 계약이 바뀌었습니다."
    echo "   임의 재복사 금지: 먼저 팀 합의 → 재벤더 + VENDORED.md·계획서 계약 절 갱신."
    exit 1
fi
