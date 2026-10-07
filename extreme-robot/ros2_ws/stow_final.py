#!/usr/bin/env python3
"""stow 후보 확정 검증 — 자세 자체 + 홈에서 거기까지 가는 경로.

arm_fsm 의 STOWING 은 MoveIt 을 안 거치고 관절궤적을 직접 발행하므로(경유점 없는 선형
보간), **경로 전체가 충돌 없이 지나가는지**까지 봐야 한다. 도착점만 검사하면 도중에
긁고 지나가는 걸 놓친다.

사용: python3 stow_final.py [j2 j3 j4]   (생략 시 확정값 all-zero)
"""
import sys

import numpy as np
import rclpy

sys.path.insert(0, '/root/ros2_ws')
import stow_eval as S

# 팀 확정 stow(2026-07-29) = all-zero. 대안 '수직 마스트'는 [0, 1.40, 2.85, 0, 0].
DEFAULT_STOW = [0.0, 0.0, 0.0, 0.0, 0.0]
CHASSIS_HALF_WIDTH = 0.150   # 차체 폭 300mm


def main():
    stow = list(DEFAULT_STOW)
    for i, v in enumerate(sys.argv[1:4]):
        stow[i + 1] = float(v)

    links, joints = S.parse_urdf()
    rclpy.init()
    ev = S.Evaluator(links, joints)

    print('=== 대상 자세 ===')
    S.report(ev, stow)

    if all(abs(v) < 1e-9 for v in stow):
        print('\n(all-zero 라 홈에서의 이동 경로가 없다 — 경로 검사 생략)')
    else:
        print('\n=== all-zero → stow 선형보간 경로 (21스텝) ===')
        worst_r, bad = 0.0, []
        for a in np.linspace(0, 1, 21):
            q = [a * v for v in stow]
            poses, g = ev.geometry(q)
            nc = ev.new_collisions(q)
            worst_r = max(worst_r, g['rmax'])
            flag = ''
            if nc:
                bad.append((round(a, 2), sorted(nc)))
                flag = f'  ⚠️ 충돌 {sorted(nc)}'
            if g['zmin'] < 0.005:
                flag += f'  ⚠️ 차체 근접 {g["zmin"]*1000:.1f}mm'
            tau = (ev.torque(q, poses, 'arm_joint_2')
                   + ev.torque(q, poses, 'arm_joint_3'))
            print(f'  {a*100:5.0f}%  j2={q[1]:5.2f} j3={q[2]:5.2f}  '
                  f'반경 {g["rmax"]*1000:6.1f}mm  높이 {g["zmax"]*1000:6.1f}mm'
                  f'  토크 {tau:5.2f}{flag}')
        over = worst_r > CHASSIS_HALF_WIDTH
        print(f'\n경로 중 최대 반경 {worst_r*1000:.1f}mm '
              f'({"차체 반폭 초과 — 정지 상태에서만 전개할 것" if over else "차체 반폭 이내"})')
        print(f'경로 충돌: {"없음" if not bad else bad}')

    ev.publish(stow)
    print('\n→ RViz(mock)에 발행함')
    ev.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
