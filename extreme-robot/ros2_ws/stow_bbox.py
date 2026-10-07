#!/usr/bin/env python3
"""후보 자세의 축별 실제 점유 범위(bbox). 차체가 정사각형이 아니면 j1 을 돌려 긴 쪽에
눕힐 수 있으므로, 반경 하나로는 판정이 안 된다 — x/y 를 따로 본다."""
import sys

import numpy as np
import rclpy

sys.path.insert(0, '/root/ros2_ws')
import stow_eval as S

CANDS = [
    ('all-zero (팀 확정 stow)', [0, 0, 0, 0, 0]),
    ('near-zero 회피책', [0, 0.15, 0.15, 0, 0]),
    ('수직마스트 (대안)', [0, 1.40, 2.85, 0, 0]),
]


def main():
    links, joints = S.parse_urdf()
    rclpy.init()
    ev = S.Evaluator(links, joints)
    for name, q in CANDS:
        poses, g = ev.geometry(q)
        lo = np.array([1e9] * 3)
        hi = np.array([-1e9] * 3)
        for nm, e in links.items():
            if e['verts'] is None or nm not in poses or nm not in ev.moving:
                continue
            t, R = poses[nm]
            w = e['verts'] @ R.T + t
            lo = np.minimum(lo, w.min(0))
            hi = np.maximum(hi, w.max(0))
        span = (hi - lo) * 1000
        t2 = ev.torque(q, poses, 'arm_joint_2')
        t3 = ev.torque(q, poses, 'arm_joint_3')
        print(f'--- {name}   j2..j4 = {q[1]:.3f}, {q[2]:.2f}, {q[3]:.2f}')
        print(f'    x [{lo[0]*1000:7.1f}, {hi[0]*1000:7.1f}]  폭 {span[0]:6.1f} mm')
        print(f'    y [{lo[1]*1000:7.1f}, {hi[1]*1000:7.1f}]  폭 {span[1]:6.1f} mm')
        print(f'    z [{lo[2]*1000:7.1f}, {hi[2]*1000:7.1f}]  높이 {span[2]:6.1f} mm')
        print(f'    → j1 최적 회전 시 필요한 차체 최소 치수: '
              f'짧은변 {2*min(max(abs(lo[0]),abs(hi[0])), max(abs(lo[1]),abs(hi[1])))*1000:.0f}mm × '
              f'긴변 {2*max(max(abs(lo[0]),abs(hi[0])), max(abs(lo[1]),abs(hi[1])))*1000:.0f}mm')
        print(f'    반경 {g["rmax"]*1000:.1f}mm  토크 {t2+t3:.2f}N·m')
    ev.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
