#!/bin/bash
# D435i(뎁스카메라) raw 프레임 → H.264/SRT 스트리밍(계약 v2 포트 :5002).
# perception_node가 이미 떠서 /perception/raw_image를 발행 중이어야 함 —
# 이 스크립트는 그걸 구독해 gst-launch로 인코딩/송출만 한다.
# PC 수신: power-train-sw/scripts/recv_stream.sh 5002 <JetsonIP>
ros2 run robot_arm_perception stream_node --ros-args -p port:=5002 -p fps:=30 -p encoder_threads:=4
