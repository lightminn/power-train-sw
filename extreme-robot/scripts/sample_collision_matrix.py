#!/usr/bin/env python3
"""SRDF Default collision 후보 샘플링 스크립트.

`/check_state_validity`(move_group)로 arm_joint_1~5 관절 리밋 안쪽 무작위 자세를
다수 검사해, SRDF에 아직 등록 안 된 링크쌍 중 (a) 항상 충돌하는 쌍 → Default 등록
후보, (b) 가끔만 충돌하는 쌍 → 진짜 self-collision 위험(절대 disable 금지)으로
분류한다. 2026-07-16 커밋(24f4c4c)에서 40샘플로 급하게 만든 초기 버전을 재현 가능한
스크립트로 남긴 것 — 표본 수를 늘려 근거를 강화하는 용도.

사용법 (컨테이너 안, move_group이 이미 떠 있어야 함 — demo.launch.py use_rviz:=false):
    python3 scripts/sample_collision_matrix.py --samples 2000
"""
import argparse
import random
import re
import sys
from collections import Counter

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

# URDF(robot_arm_description/urdf/robot_arm.urdf)에서 추출한 값.
# arm_joint_1/5는 continuous(리밋 태그 없음) — 샘플링을 위해 -pi..pi 임의 범위 사용.
JOINT_LIMITS = {
    "arm_joint_1": (-3.141592653589793, 3.141592653589793),
    "arm_joint_2": (0.0, 3.141592653589793),
    "arm_joint_3": (0.0, 3.141592653589793),
    "arm_joint_4": (-0.610865238198015, 0.872664625997165),
    "arm_joint_5": (-3.141592653589793, 3.141592653589793),
}
JOINT_NAMES = list(JOINT_LIMITS.keys())


def load_known_pairs(srdf_path):
    known = set()
    with open(srdf_path) as f:
        content = f.read()
    for m in re.finditer(r'<disable_collisions link1="([^"]+)" link2="([^"]+)"', content):
        known.add(frozenset((m.group(1), m.group(2))))
    return known


class Sampler(Node):
    def __init__(self):
        super().__init__('collision_matrix_sampler')
        self.cli = self.create_client(GetStateValidity, '/check_state_validity')

    def wait(self, timeout=15.0):
        return self.cli.wait_for_service(timeout_sec=timeout)

    def check(self, positions):
        req = GetStateValidity.Request()
        req.group_name = 'arm'
        rs = RobotState()
        js = JointState()
        js.name = JOINT_NAMES
        js.position = positions
        rs.joint_state = js
        req.robot_state = rs
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument(
        "--srdf",
        default="/root/ros2_ws/src/robot_arm_moveit_config/config/robot_arm.srdf",
    )
    args = ap.parse_args()

    random.seed(args.seed)
    known_pairs = load_known_pairs(args.srdf)
    print(f"기존 SRDF에 등록된 쌍: {len(known_pairs)}개 (결과에서 제외)", file=sys.stderr)

    rclpy.init()
    node = Sampler()
    if not node.wait():
        print("ERROR: /check_state_validity 서비스 없음 — move_group이 떠 있는지 확인", file=sys.stderr)
        sys.exit(1)

    pair_counts = Counter()
    valid_samples = 0
    for i in range(args.samples):
        positions = [random.uniform(*JOINT_LIMITS[n]) for n in JOINT_NAMES]
        result = node.check(positions)
        if result is None:
            print(f"[{i}] 서비스 호출 실패/타임아웃, 스킵", file=sys.stderr)
            continue
        valid_samples += 1
        if not result.valid:
            for contact in result.contacts:
                pair = frozenset((contact.contact_body_1, contact.contact_body_2))
                if pair in known_pairs:
                    continue
                pair_counts[pair] += 1
        if (i + 1) % 200 == 0:
            print(f"진행: {i + 1}/{args.samples}", file=sys.stderr)

    node.destroy_node()
    rclpy.shutdown()

    print(f"\n총 유효 샘플: {valid_samples}")
    print(f"기존 SRDF 미등록 충돌쌍: {len(pair_counts)}개\n")

    always = []
    sometimes = []
    for pair, count in pair_counts.most_common():
        l1, l2 = tuple(pair)
        if count == valid_samples:
            always.append((l1, l2, count))
        else:
            sometimes.append((l1, l2, count))

    print(f"=== 항상 충돌 (Default 등록 후보, {len(always)}쌍) ===")
    for l1, l2, count in always:
        print(
            f'    <disable_collisions link1="{l1}" link2="{l2}" reason="Default"/>'
            f"  <!-- {count}/{valid_samples} -->"
        )

    print(f"\n=== 애매하게 충돌 (진짜 self-collision 위험, disable 금지, {len(sometimes)}쌍) ===")
    for l1, l2, count in sorted(sometimes, key=lambda x: -x[2]):
        print(f"    {l1} / {l2}: {count}/{valid_samples} ({count / valid_samples:.1%})")


if __name__ == "__main__":
    main()
