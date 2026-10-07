#!/usr/bin/env python3
"""SRDF 의 누락된 Adjacent disable_collisions 를 강체 클러스터 기준으로 산출/추가한다.

## 왜 필요한가

MoveIt Setup Assistant 의 "Adjacent" 판정은 **URDF 직계 parent/child 쌍**만 본다.
그런데 이 로봇은 CAD 자동 export 라 하나의 물리 강체가 fixed 조인트로 이어진
링크 여러 개(`link_0XX`)로 쪼개져 있다. 그래서

  - 같은 강체 안의 형제 링크끼리, 그리고
  - 가동 조인트 하나를 사이에 둔 강체끼리(= 경첩의 양쪽)

가 disable 목록에서 통째로 빠진다. 이 쌍들은 축 근처에서 메시가 항상 겹치므로
**어떤 자세에서도 충돌로 뜬다** — 2026-08-09 실측으로 home(전 관절 0) 자세조차
6쌍이 충돌로 판정돼 MoveIt 이 모든 계획을 거부하는 상태였다.

`sample_collision_matrix.py` 의 자동 판정으로는 이걸 못 잡는다. 그쪽은 "샘플 100%
에서 충돌" 을 Default 후보로 삼는데, 이 쌍들은 88~98% 라 기준에 안 걸린다(축 각도에
따라 메시가 아주 가끔 떨어진다). 그래서 기구학 구조로 직접 판정한다.

## 무엇을 disable 하나

  same-cluster     — fixed 로 용접돼 상대 자세가 절대 안 변하는 쌍. 충돌 판정이
                     자세와 무관하게 항상 같으므로 명백한 허위 양성이다.
  adjacent-cluster — 가동 조인트 하나로 이어진 강체 쌍. MoveIt Setup Assistant 의
                     "Adjacent" 범주와 같은 것이며, 표준적으로 항상 disable 한다.

⚠️ **그 외에는 건드리지 않는다.** 가동 조인트 두 개 이상 떨어진 쌍(예: 팔이 접혀
   베이스를 때리는 경우)은 진짜 self-collision 이라 남겨둔다. 기존 항목도 지우지
   않고 추가만 한다.

사용:
    python3 scripts/fix_adjacent_collisions.py            # 산출만(dry-run)
    python3 scripts/fix_adjacent_collisions.py --apply    # SRDF 에 기록
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from itertools import combinations

DEFAULT_URDF = "ros2_ws/src/robot_arm_description/urdf/robot_arm.urdf"
DEFAULT_SRDF = "ros2_ws/src/robot_arm_moveit_config/config/robot_arm.srdf"


def parse_urdf(path):
    root = ET.parse(path).getroot()
    links = [l.get("name") for l in root.findall("link")]
    joints = {}
    for j in root.findall("joint"):
        joints[j.get("name")] = {
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
        }
    return links, joints


def rigid_clusters(joints, link_names):
    """fixed 조인트로 묶인 링크 덩어리(= 실제 강체). stow_eval.py 와 같은 union-find."""
    parent = {n: n for n in link_names}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for j in joints.values():
        if j["type"] == "fixed":
            ra, rb = find(j["parent"]), find(j["child"])
            if ra != rb:
                parent[ra] = rb

    cid = {n: find(n) for n in link_names}
    adj = {}
    for j in joints.values():
        if j["type"] != "fixed":
            a, b = cid[j["parent"]], cid[j["child"]]
            adj.setdefault(a, set()).add(b)
            adj.setdefault(b, set()).add(a)
    return cid, adj


def existing_pairs(srdf_text):
    known = set()
    for m in re.finditer(r'<disable_collisions\s+link1="([^"]+)"\s+link2="([^"]+)"', srdf_text):
        known.add(frozenset(m.groups()))
    return known


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--srdf", default=DEFAULT_SRDF)
    ap.add_argument("--apply", action="store_true", help="SRDF 에 실제로 기록")
    args = ap.parse_args()

    links, joints = parse_urdf(args.urdf)
    cid, adj = rigid_clusters(joints, links)

    with open(args.srdf) as f:
        srdf_text = f.read()
    known = existing_pairs(srdf_text)

    n_clusters = len(set(cid.values()))
    print(f"링크 {len(links)}개 → 강체 클러스터 {n_clusters}개")
    print(f"SRDF 기존 disable_collisions: {len(known)}쌍\n")

    additions = []  # (link1, link2, reason)
    for a, b in combinations(sorted(links), 2):
        if frozenset((a, b)) in known:
            continue
        ca, cb = cid[a], cid[b]
        if ca == cb:
            additions.append((a, b, "same-cluster"))
        elif cb in adj.get(ca, ()):
            additions.append((a, b, "adjacent-cluster"))

    same = [x for x in additions if x[2] == "same-cluster"]
    adjc = [x for x in additions if x[2] == "adjacent-cluster"]
    print(f"추가할 쌍: {len(additions)}개 "
          f"(same-cluster {len(same)}, adjacent-cluster {len(adjc)})")

    if not args.apply:
        print("\n(dry-run — 기록하려면 --apply)")
        for a, b, why in additions[:20]:
            print(f"    {a} / {b}  [{why}]")
        if len(additions) > 20:
            print(f"    ... 외 {len(additions) - 20}개")
        return 0

    if not additions:
        print("추가할 것 없음")
        return 0

    block = [
        "",
        "    <!-- 2026-08-09 추가: 강체 클러스터 기준 Adjacent 보정 "
        "(scripts/fix_adjacent_collisions.py).",
        "         CAD 자동 export 라 하나의 물리 강체가 fixed 로 이어진 link_0XX 여러 개로",
        "         쪼개져 있는데, Setup Assistant 의 Adjacent 판정은 URDF 직계 parent/child 만",
        "         봐서 (a) 같은 강체 안의 형제 링크, (b) 가동 조인트 하나를 사이에 둔 강체 쌍이",
        "         통째로 빠졌다. 그 결과 home(전 관절 0) 자세조차 6쌍이 충돌로 떠 MoveIt 이",
        "         모든 계획을 거부했다. 가동 조인트 2개 이상 떨어진 쌍(진짜 self-collision)은",
        "         건드리지 않았다. -->",
    ]
    for a, b, why in additions:
        block.append(f'    <disable_collisions link1="{a}" link2="{b}" reason="{why}" />')
    block.append("")

    marker = "</robot>"
    idx = srdf_text.rindex(marker)
    new_text = srdf_text[:idx] + "\n".join(block) + srdf_text[idx:]
    with open(args.srdf, "w") as f:
        f.write(new_text)
    print(f"\n{args.srdf} 에 {len(additions)}쌍 추가 완료")
    return 0


if __name__ == "__main__":
    sys.exit(main())
