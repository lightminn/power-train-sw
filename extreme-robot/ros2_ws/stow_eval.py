#!/usr/bin/env python3
"""stow 자세 평가 툴 (2026-07-29 랙피니언 그리퍼 URDF 기준으로 재작성).

기존 stow_probe/stow_lowest/stow_torque_viz 를 하나로 합치고, 다음을 고쳤다.
  * 링크 목록을 URDF 에서 직접 파싱 (link_001~057 하드코딩 → 51개 실제 링크)
  * 그리퍼 서브트리를 link_041~050 으로 명시 (옛 prefix 'link_04' 는 팔 링크 link_040 까지 잘못 제외했음)
  * 높이/침범 판정을 **링크 원점이 아니라 메쉬 정점**으로 계산 (fusion2urdf 는 프레임 원점을
    파트와 무관한 곳에 두므로 원점 기준 Z 는 의미가 없다)
  * 중력토크를 arm_joint_2/3 실제 축(월드) 기준으로 distal 링크만 합산

사용:
  python3 stow_eval.py sweep            # j2·j3 격자 탐색
  python3 stow_eval.py eval J2 J3 [J4]  # 단일 자세 상세 평가
  python3 stow_eval.py show J2 J3 [J4]  # RViz(mock)로 해당 자세 표시
"""
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PKG = '/root/ros2_ws/src/robot_arm_description'
URDF = f'{PKG}/urdf/robot_arm.urdf'
ARM = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
GRIPPER_JOINT = 'gripper_left_pinion_joint'
GRIPPER_LINKS = {f'link_{i:03d}' for i in range(41, 51)}
G = 9.81


# ---------------------------------------------------------------- URDF 파싱
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def load_stl(path):
    """binary STL → (N,3) 정점 배열."""
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        buf = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    tri = buf[:, 12:48].copy().view(np.float32).reshape(n, 3, 3)
    return tri.reshape(-1, 3).astype(np.float64)


def hull_points(v):
    """볼록껍질 정점만 남긴다 — 최대/최소 Z·반경은 전부 껍질 위에서 나오므로 결과는 동일하고,
    자세마다 회전시켜야 할 점 수가 수십분의 1로 줄어든다."""
    if len(v) < 8:
        return v
    try:
        from scipy.spatial import ConvexHull
        return v[np.unique(ConvexHull(v).vertices)]
    except Exception:
        return v


def parse_urdf():
    root = ET.parse(URDF).getroot()
    links = {}
    for link in root.findall('link'):
        name = link.get('name')
        entry = {'mass': 0.0, 'com': np.zeros(3), 'verts': None}
        ine = link.find('inertial')
        if ine is not None:
            entry['mass'] = float(ine.find('mass').get('value'))
            o = ine.find('origin')
            if o is not None:
                entry['com'] = np.array([float(v) for v in o.get('xyz').split()])
        col = link.find('collision') if link.find('collision') is not None else link.find('visual')
        mesh = col.find('geometry/mesh') if col is not None else None
        if mesh is not None:
            fn = mesh.get('filename').split('robot_arm_description/')[-1]
            scale = np.array([float(v) for v in (mesh.get('scale') or '1 1 1').split()])
            v = load_stl(os.path.join(PKG, fn)) * scale
            o = col.find('origin')
            if o is not None:
                t = np.array([float(x) for x in o.get('xyz').split()])
                R = rpy_to_R(*[float(x) for x in o.get('rpy').split()])
                v = v @ R.T + t
            entry['verts'] = hull_points(v)
        links[name] = entry

    joints = {}
    for j in root.findall('joint'):
        joints[j.get('name')] = {
            'type': j.get('type'),
            'parent': j.find('parent').get('link'),
            'child': j.find('child').get('link'),
            'axis': np.array([float(v) for v in j.find('axis').get('xyz').split()])
            if j.find('axis') is not None else None,
        }
    return links, joints


def rigid_clusters(joints, link_names):
    """fixed 조인트로 묶인 링크 덩어리(= 실제 강체) 를 찾는다.

    SRDF 의 Adjacent disable_collisions 가 **URDF 직계 parent/child 만** 보고 생성돼서,
    같은 강체에 용접된 형제 링크나 가동 조인트를 사이에 둔 강체끼리는 등록이 안 됐다.
    (예: arm_joint_4 는 link_006↔link_038 만 disable 되고, 거기 fixed 로 붙은
    link_023↔link_027 은 남아 j4 를 조금만 꺾어도 '충돌'로 뜬다.)
    여기서 강체 단위로 다시 계산해 그 허위 양성을 걷어낸다."""
    parent = {n: n for n in link_names}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for j in joints.values():
        if j['type'] == 'fixed':
            ra, rb = find(j['parent']), find(j['child'])
            if ra != rb:
                parent[ra] = rb
    cid = {n: find(n) for n in link_names}
    # 강체 그래프: 가동 조인트가 잇는 강체 쌍
    adj = {}
    for j in joints.values():
        if j['type'] != 'fixed':
            a, b = cid[j['parent']], cid[j['child']]
            adj.setdefault(a, set()).add(b)
            adj.setdefault(b, set()).add(a)
    return cid, adj


def distal_links(joints, joint_name):
    """joint_name 하위(distal) 모든 링크 이름."""
    kids = {}
    for j in joints.values():
        kids.setdefault(j['parent'], []).append(j['child'])
    out, stack = set(), [joints[joint_name]['child']]
    while stack:
        n = stack.pop()
        if n in out:
            continue
        out.add(n)
        stack += kids.get(n, [])
    return out


# ---------------------------------------------------------------- ROS 클라이언트
class Evaluator(Node):
    def __init__(self, links, joints):
        super().__init__('stow_eval')
        self.links, self.joints = links, joints
        self.link_names = list(links.keys())
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.sv = self.create_client(GetStateValidity, '/check_state_validity')
        for c, n in ((self.fk, '/compute_fk'), (self.sv, '/check_state_validity')):
            if not c.wait_for_service(timeout_sec=10):
                raise SystemExit(f'{n} 서비스 없음 — demo.launch.py 가 떠 있는지 확인')
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.base = None
        self.moving = distal_links(joints, 'arm_joint_1')
        self.cid, self.cadj = rigid_clusters(joints, self.link_names)

    def _state(self, q):
        names = list(ARM) + [GRIPPER_JOINT]
        pos = list(q) + [0.0]  # 그리퍼는 닫힘(0)으로 stow
        return RobotState(joint_state=JointState(name=names, position=pos))

    def fk_poses(self, q):
        r = GetPositionFK.Request()
        r.header.frame_id = 'base_link'
        r.fk_link_names = self.link_names
        r.robot_state = self._state(q)
        f = self.fk.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        res = f.result()
        out = {}
        for nm, ps in zip(res.fk_link_names, res.pose_stamped):
            p, o = ps.pose.position, ps.pose.orientation
            out[nm] = (np.array([p.x, p.y, p.z]), quat_to_R(o.x, o.y, o.z, o.w))
        return out

    def collisions(self, q):
        r = GetStateValidity.Request()
        r.robot_state = self._state(q)
        r.group_name = 'arm'
        f = self.sv.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        return {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in f.result().contacts}

    def benign(self, pair):
        """SRDF 가 놓친 허위 양성인가 — 같은 강체거나, 가동 조인트 하나를 사이에 둔 이웃 강체."""
        a, b = self.cid.get(pair[0]), self.cid.get(pair[1])
        if a is None or b is None:
            return False
        if a == b or b in self.cadj.get(a, ()):
            return True
        # 그리퍼 내부(mimic 평행링크)끼리는 자세와 무관한 상시 접촉
        return pair[0] in GRIPPER_LINKS and pair[1] in GRIPPER_LINKS

    def new_collisions(self, q):
        """홈(all-zero) 기준선 대비 새로 생긴 쌍 중 허위 양성을 뺀 것."""
        if self.base is None:
            self.base = self.collisions([0.0] * 5)
        new = self.collisions(q) - self.base
        return {p for p in new if not self.benign(p)}

    def geometry(self, q):
        """(poses, 지표dict). 지표는 **움직이는 링크**(arm_joint_1 하위) 기준이다 — 고정 받침대까지
        섞으면 어떤 자세든 같은 하한이 깔려 자세 간 비교가 안 된다."""
        poses = self.fk_poses(q)
        zmax, zmin, rmax = -1e9, 1e9, 0.0
        msum, mcom = 0.0, np.zeros(3)
        for nm, e in self.links.items():
            if nm not in poses:
                continue
            t, R = poses[nm]
            if e['verts'] is not None and nm in self.moving:
                w = e['verts'] @ R.T + t
                zmax, zmin = max(zmax, w[:, 2].max()), min(zmin, w[:, 2].min())
                rmax = max(rmax, np.sqrt(w[:, 0] ** 2 + w[:, 1] ** 2).max())
            if e['mass'] > 0:
                msum += e['mass']
                mcom += e['mass'] * (R @ e['com'] + t)
        return poses, {'zmax': zmax, 'zmin': zmin, 'rmax': rmax,
                       'mass': msum, 'cg': mcom / msum}

    def torque(self, q, poses, joint_name):
        """joint_name 축 기준 중력 홀딩토크(N·m)."""
        jt = self.joints[joint_name]
        t, R = poses[jt['child']]
        axis = R @ jt['axis']
        axis /= np.linalg.norm(axis)
        tau = 0.0
        for nm in distal_links(self.joints, joint_name):
            e = self.links.get(nm)
            if not e or e['mass'] <= 0 or nm not in poses:
                continue
            lt, lR = poses[nm]
            r = (lR @ e['com'] + lt) - t
            tau += np.dot(axis, np.cross(r, np.array([0, 0, -e['mass'] * G])))
        return abs(tau)

    def publish(self, q):
        m = JointTrajectory()
        m.joint_names = list(ARM)
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start.sec = 2
        m.points = [p]
        self.traj_pub.publish(m)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ---------------------------------------------------------------- 명령
def report(ev, q, tag=''):
    poses, g = ev.geometry(q)
    nc = ev.new_collisions(q)
    t2 = ev.torque(q, poses, 'arm_joint_2')
    t3 = ev.torque(q, poses, 'arm_joint_3')
    cg = g['cg']
    print(f'{tag}q = [{", ".join(f"{v:.3f}" for v in q)}]')
    print(f'  가동부 반경 {g["rmax"]*1000:7.1f} mm  ← 차체 밖으로 나가는 정도(작을수록 좋음)')
    print(f'  최고점 Z    {g["zmax"]*1000:7.1f} mm   최저점 Z {g["zmin"]*1000:7.1f} mm'
          f'{"   ⚠️ base_link 아래(차체 침범)" if g["zmin"] < -0.001 else ""}')
    print(f'  CG      ({cg[0]*1000:.1f}, {cg[1]*1000:.1f}, {cg[2]*1000:.1f}) mm   총질량 {g["mass"]:.3f} kg')
    print(f'  중력토크 j2 {t2:5.2f} N·m   j3 {t3:5.2f} N·m   합 {t2+t3:5.2f}')
    print(f'  새 자기충돌 {len(nc)}쌍' + (f' → {sorted(nc)}' if nc else ''))
    return g, len(nc), t2 + t3


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'sweep'
    links, joints = parse_urdf()
    rclpy.init()
    ev = Evaluator(links, joints)

    if cmd in ('eval', 'show'):
        q = [0.0] * 5
        for i, v in enumerate(sys.argv[2:5]):
            q[i + 1] = float(v)
        report(ev, q)
        if cmd == 'show':
            ev.publish(q)
            print('  → RViz(mock)로 발행함')
    elif cmd == 'baseline':
        base = ev.collisions([0.0] * 5)
        print(f'홈(all-zero) 기준선 충돌쌍 {len(base)}개')
        for p in sorted(base):
            print('  ', p)
        report(ev, [0.0] * 5, 'all-zero  ')
    elif cmd == 'mast':
        # 차체 폭 300mm → 반경 150mm 안에 들어가는 건 '수직 마스트' 계열뿐이다. 그 안에서 고른다.
        LIM = 0.150
        print(f'마스트 영역 세밀 sweep — 반경 {LIM*1000:.0f}mm 이내만 채택')
        rows = []
        for j2 in np.arange(1.00, 1.751, 0.025):
            for j3 in np.arange(2.10, 3.101, 0.05):
                for j4 in np.arange(-0.61, 0.871, 0.1):
                    q = [0.0, float(j2), float(j3), float(j4), 0.0]
                    poses, g = ev.geometry(q)
                    if g['rmax'] > LIM or g['zmin'] < 0.005:
                        continue
                    if ev.new_collisions(q):
                        continue
                    t = ev.torque(q, poses, 'arm_joint_2') + ev.torque(q, poses, 'arm_joint_3')
                    rows.append((j2, j3, j4, g['rmax'], g['zmax'], g['zmin'], 0, t, g['cg'][2]))
            print(f'  j2={j2:.3f} 완료 (누적 {len(rows)})', flush=True)
        np.save('/root/ros2_ws/stow_mast.npy', np.array(rows))
        hdr = (f'{"j2":>7}{"j3":>7}{"j4":>7}{"반경mm":>9}{"높이mm":>9}'
               f'{"최저mm":>9}{"CGzmm":>8}{"토크":>8}')

        def tab(title, key, n=12):
            print(f'\n=== {title} ===\n{hdr}')
            for j2, j3, j4, r, zx, zn, _, t, cgz in sorted(rows, key=key)[:n]:
                print(f'{j2:>7.3f}{j3:>7.2f}{j4:>7.2f}{r*1000:>9.1f}{zx*1000:>9.1f}'
                      f'{zn*1000:>9.1f}{cgz*1000:>8.1f}{t:>8.2f}')

        print(f'\n반경 {LIM*1000:.0f}mm 이내 유효 자세 {len(rows)}개')
        tab('반경 작은 순(차체 여유 큰 순)', lambda r: r[3])
        tab('중력토크 작은 순', lambda r: r[7])
        tab('높이 낮은 순', lambda r: r[4])
    elif cmd == 'refine':
        # '접어 올렸다 되접는' 영역을 촘촘히 — 낮으면서 발자국 작은 자세를 찾는다
        print('세밀 sweep — j2 ∈ [0.4,1.4] / j3 ∈ [0,1.2] / j4 ∈ [-0.61,0.87], step 0.05·0.05·0.2')
        rows = []
        for j2 in np.arange(0.40, 1.401, 0.05):
            for j3 in np.arange(0.00, 1.201, 0.05):
                for j4 in np.arange(-0.61, 0.871, 0.2):
                    q = [0.0, float(j2), float(j3), float(j4), 0.0]
                    poses, g = ev.geometry(q)
                    if g['zmin'] < 0.005:          # 차체 상판에 5mm 이상 여유
                        continue
                    if ev.new_collisions(q):
                        continue
                    t = ev.torque(q, poses, 'arm_joint_2') + ev.torque(q, poses, 'arm_joint_3')
                    rows.append((j2, j3, j4, g['rmax'], g['zmax'], g['zmin'], 0, t, g['cg'][2]))
            print(f'  j2={j2:.2f} 완료 (누적 {len(rows)})', flush=True)
        arr = np.array(rows)
        np.save('/root/ros2_ws/stow_refine.npy', arr)
        # (반경, 높이) 파레토 front
        pareto = [r for r in rows
                  if not any(o[3] <= r[3] and o[4] <= r[4] and o[:3] != r[:3] for o in rows)]
        pareto.sort(key=lambda r: r[4])
        print(f'\n=== (반경, 높이) 파레토 front {len(pareto)}개 — 높이 낮은 순 ===')
        print(f'{"j2":>6}{"j3":>6}{"j4":>7}{"반경mm":>9}{"높이mm":>9}{"최저mm":>9}{"CGzmm":>8}{"토크":>8}')
        for j2, j3, j4, r, zx, zn, _, t, cgz in pareto:
            print(f'{j2:>6.2f}{j3:>6.2f}{j4:>7.2f}{r*1000:>9.1f}{zx*1000:>9.1f}'
                  f'{zn*1000:>9.1f}{cgz*1000:>8.1f}{t:>8.2f}')
    else:
        J4 = [-0.61, -0.30, 0.0, 0.44, 0.87]
        print(f'격자 sweep — j2,j3 ∈ [0, π] step 0.15, j4 ∈ {J4}, 그리퍼 닫힘')
        rows = []
        for j2 in np.arange(0.0, 3.1416, 0.15):
            for j3 in np.arange(0.0, 3.1416, 0.15):
                for j4 in J4:
                    q = [0.0, float(j2), float(j3), float(j4), 0.0]
                    poses, g = ev.geometry(q)
                    nc = ev.new_collisions(q)
                    t = ev.torque(q, poses, 'arm_joint_2') + ev.torque(q, poses, 'arm_joint_3')
                    rows.append((j2, j3, j4, g['rmax'], g['zmax'], g['zmin'],
                                 len(nc), t, g['cg'][2]))
            print(f'  j2={j2:.2f} 완료', flush=True)

        arr = np.array(rows)
        np.save('/root/ros2_ws/stow_sweep.npy', arr)
        ok = [r for r in rows if r[6] == 0 and r[5] >= -0.001]
        print(f'\n충돌 0 + 차체 미침범: {len(ok)}/{len(rows)} 자세')

        hdr = (f'{"j2":>6}{"j3":>6}{"j4":>7}{"반경(mm)":>10}{"maxZ":>8}'
               f'{"minZ":>8}{"CGz":>8}{"토크":>8}')

        def table(title, key, n=12):
            sel = sorted(ok, key=key)[:n]
            print(f'\n=== {title} ===\n{hdr}')
            for j2, j3, j4, r, zx, zn, _, t, cgz in sel:
                print(f'{j2:>6.2f}{j3:>6.2f}{j4:>7.2f}{r*1000:>10.1f}{zx*1000:>8.1f}'
                      f'{zn*1000:>8.1f}{cgz*1000:>8.1f}{t:>8.2f}')

        table('A. 가장 컴팩트 (반경 최소)', lambda r: r[3])
        table('B. 가장 낮음 (최고점 Z 최소)', lambda r: r[4])
        table('C. 중력토크 최소', lambda r: r[7])
        # 반경·높이·토크를 각각 정규화해 합산 — 셋 다 어중간하게 좋은 절충안
        rr = np.array([r[3] for r in ok]); zz = np.array([r[4] for r in ok])
        tt = np.array([r[7] for r in ok])
        def norm(a):
            return (a - a.min()) / (a.max() - a.min() + 1e-9)
        score = {id(r): s for r, s in zip(ok, norm(rr) + norm(zz) + norm(tt))}
        table('D. 절충 (반경+높이+토크 정규화 합)', lambda r: score[id(r)])
        print('\n원자료 → /root/ros2_ws/stow_sweep.npy '
              '(열: j2 j3 j4 rmax zmax zmin ncoll torque cgz)')

    ev.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
