#!/bin/bash
# 로컬 디버그용 — YOLO 오버레이(bbox/마스크/거리, /perception/debug_image)를 SRT로 송출.
# 계약 v2 포트(L515 :5000, D435i raw :5002, D435i YOLO metadata UDP :5003)와 안 겹치게 :5004 사용.
# /perception/debug_image는 구독자가 있을 때만 발행되므로, 이 스크립트가 붙는 순간 perception_node가
# 오버레이 발행을 시작한다 — 갱신 속도는 raw 스트림이 아니라 YOLO 추론 속도를 따라간다.
# PC 수신: power-train-sw/scripts/recv_stream.sh 5004 <JetsonIP>
ros2 run robot_arm_perception stream_node --ros-args \
  -p port:=5004 -p image_topic:=/perception/debug_image -p fps:=30 -p encoder_threads:=2
