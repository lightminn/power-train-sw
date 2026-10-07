#!/bin/bash
# 미션3 마커 식별 (비전마커 식별). 관찰 전용(pick_classes 비움) — /detected_objects만
# 발행, /pick_target은 나오지 않음(파워트레인이 /detected_objects를 구독해 자세 락 판단).
ros2 run robot_arm_perception perception_node --ros-args -p model_name:=vision_marker -p camera_mode:=realsense
