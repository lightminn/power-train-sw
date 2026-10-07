#!/bin/bash
# 미션5 선도차 추종 (선도차 PID 추종). 관찰 전용(pick_classes 비움) — /detected_objects만
# 발행, /pick_target은 나오지 않음(파워트레인이 /detected_objects를 구독해 추종 판단).
# ⚠️ TODO: models/lead_robot_best.pt 아직 없음(placeholder) — 모델 첨부 전엔 실행 불가.
ros2 run robot_arm_perception perception_node --ros-args -p model_name:=lead_robot -p camera_mode:=realsense
