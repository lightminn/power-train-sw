#!/bin/bash
# 미션2 박스 운반 (박스 파지). 다른 구간용은 run_perception_traffic_light.sh 참고.
ros2 run robot_arm_perception perception_node --ros-args -p model_name:=box -p camera_mode:=realsense -p pick_min_conf:=0.5 -p require_depth:=true
