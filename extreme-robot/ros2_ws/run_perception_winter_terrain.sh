#!/bin/bash
# 미션4 노면 주행 (빙판/제설 주행). 관찰 전용(pick_classes 비움) — /detected_objects만
# 발행, /pick_target은 나오지 않음(파워트레인이 /detected_objects를 구독해 판단).
# ⚠️ TODO: models/winter_terrain_best.pt 아직 없음(placeholder) — 모델 첨부 전엔 실행 불가.
ros2 run robot_arm_perception perception_node --ros-args -p model_name:=winter_terrain -p camera_mode:=realsense
