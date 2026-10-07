#!/usr/bin/env python3
"""/detected_objects에서 지정 클래스의 3D 위치를 N초간 샘플링해 평균/표준편차 출력.

실측(줄자 등) 기준값과 비교해 markerless YOLO segmentation 3D 위치 오차를
측정하기 위한 헬퍼. 컨테이너 안에서 perception_node가 이미 돌고 있어야 한다.

사용 (컨테이너 안):
    python3 /root/ros2_ws/measure_position_error.py --class-name cup --duration 3
"""
import argparse
import statistics
import time

import rclpy
from rclpy.node import Node

from robot_arm_msgs.msg import DetectedObjectArray


class PositionSampler(Node):
    def __init__(self, class_name):
        super().__init__('position_sampler')
        self.class_name = class_name
        self.samples = []
        self.sub = self.create_subscription(
            DetectedObjectArray, '/detected_objects', self._cb, 10)

    def _cb(self, msg):
        for obj in msg.objects:
            if obj.class_name == self.class_name and obj.pose.position.z != 0.0:
                p = obj.pose.position
                self.samples.append((p.x, p.y, p.z, obj.confidence))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--class-name', required=True)
    ap.add_argument('--duration', type=float, default=3.0)
    args = ap.parse_args()

    rclpy.init()
    node = PositionSampler(args.class_name)
    # rclpy.spin(node) + create_timer(duration, rclpy.shutdown)로 종료하던 이전 버전은
    # 타이머 콜백에서 shutdown()을 불러도 spin()의 wait-set 대기가 즉시 깨어나지
    # 않아 duration이 지나도 프로세스가 안 죽는 버그가 있었다(2026-07-22 실측). 대신
    # 짧은 timeout의 spin_once를 직접 루프 돌며 wall clock으로 종료 시점을 판단한다.
    deadline = time.time() + args.duration
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    n = len(node.samples)
    print(f'\n샘플 수: {n} (class={args.class_name}, {args.duration}s)')
    if n < 2:
        print('샘플 부족 (물체가 안 보이거나 depth 실패) — 카메라 각도/거리 확인')
        return

    xs = [s[0] for s in node.samples]
    ys = [s[1] for s in node.samples]
    zs = [s[2] for s in node.samples]
    confs = [s[3] for s in node.samples]

    for label, vals in (('x', xs), ('y', ys), ('z', zs)):
        mean = statistics.mean(vals)
        std = statistics.pstdev(vals)
        print(f'  {label}: mean={mean:+.4f}m  std={std:.4f}m  '
              f'min={min(vals):+.4f}  max={max(vals):+.4f}')
    print(f'  confidence: mean={statistics.mean(confs):.2f}')


if __name__ == '__main__':
    main()
