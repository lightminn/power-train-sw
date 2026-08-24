#!/usr/bin/env python3
"""보드 레지스트리 자동 생성 — 젯슨에서 실행.

각 ODrive 는 자기 CAN node id 를 NVM 에 갖고 있다(`axis.config.can_node_id`,
빌드에 따라 `axis.config.can.node_id`). USB 로 그 값을 읽어
`config/bl70200_boards.json` 을 만든다 — 손으로 적을 필요가 없다.

    python3 scripts/usb_skid_gen_registry.py            # 생성 + 검증
    python3 scripts/usb_skid_gen_registry.py --dry-run  # 읽기만 (파일 안 씀)

⚠️ USB 만 쓴다. can0 을 열지 않으므로 CAN 버스 상태와 무관하게 돈다.
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_REPO, "motor_control"))

DEFAULT_OUT = os.path.join(_REPO, "config", "bl70200_boards.json")
EXPECTED_NODES = {11, 12, 13, 14, 15, 16}


def _node_id(axis):
    """이 펌웨어 빌드에 맞는 경로로 CAN node id 를 읽는다."""
    try:
        return int(axis.config.can_node_id)
    except AttributeError:
        return int(axis.config.can.node_id)


def read_boards(find_timeout):
    from corner_module.drive_odrive_usb_axis import UsbBoardPool

    pool = UsbBoardPool(find_timeout=find_timeout)
    serials = pool.discover_serials()
    if not serials:
        raise SystemExit(
            "❌ USB 에서 ODrive 를 하나도 못 찾았다.\n"
            "   · 케이블·전원 확인\n"
            "   · pyusb 미설치면 열거가 안 된다: python3 -c 'import usb.core'")
    print("🔌 발견한 보드 %d장: %s" % (len(serials), ", ".join(serials)))

    registry = {}
    for serial in serials:
        board = pool.board(serial)
        pair = (_node_id(board.axis0), _node_id(board.axis1))
        registry[serial] = list(pair)
        print("   %s  axis0=node %-3d axis1=node %-3d  vbus %.1fV"
              % (serial, pair[0], pair[1], board.vbus_voltage))
    return registry


def check(registry):
    """생성한 표가 실제로 쓸 수 있는지 본다. 문제를 전부 모아서 돌려준다."""
    problems = []
    nodes = [n for pair in registry.values() for n in pair]

    if len(nodes) != len(set(nodes)):
        problems.append("node 번호가 중복이다: %s" % sorted(nodes))
    missing = EXPECTED_NODES - set(nodes)
    extra = set(nodes) - EXPECTED_NODES
    if missing:
        problems.append(
            "구동 node %s 가 없다 — 보드가 덜 붙었거나 node id 가 안 박혔다"
            % sorted(missing))
    if extra:
        problems.append("예상 밖 node %s (구동은 11~16)" % sorted(extra))

    for serial, (n0, n1) in registry.items():
        # axis1 = 로봇 우측 = 짝수 node 라는 게 실물 배선 규약이다
        # (2026-07-28 확인). 어긋나면 좌우 부호가 뒤집힌다.
        if n0 % 2 != 1 or n1 % 2 != 0 or n1 != n0 + 1:
            problems.append(
                "%s 의 node 쌍 (axis0=%d, axis1=%d) 이 규약과 다르다 — "
                "axis0 은 홀수(좌), axis1 은 그 다음 짝수(우)여야 한다"
                % (serial, n0, n1))
    return problems


def verify_builder(path):
    """실제 빌더로 코너가 만들어지는지까지 확인한다 (미러 규약 교차검증 포함)."""
    from chassis.chassis_manager import build_usb_skid_corners

    class _NoTouchPool:
        """USB 를 건드리지 않는 스텁 — 표 해석만 검증한다."""

        def axis(self, serial, axis_index):
            return object()

        def close(self):
            pass

    corners = build_usb_skid_corners(path, pool=_NoTouchPool())
    inverted = sorted(n for n, c in corners.items() if c.drive.invert)
    print("✅ 빌더 통과 — 바퀴 %d개, 반전 대상 %s" % (len(corners), inverted))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=DEFAULT_OUT, help="저장 경로")
    parser.add_argument("--find-timeout", type=float, default=20.0)
    parser.add_argument("--dry-run", action="store_true", help="읽기만 하고 안 쓴다")
    parser.add_argument("--force", action="store_true",
                        help="검증에 실패해도 파일을 쓴다 (권장하지 않음)")
    args = parser.parse_args()

    registry = read_boards(args.find_timeout)
    problems = check(registry)
    if problems:
        print("\n⚠️ 검증에서 걸린 것:")
        for p in problems:
            print("   · %s" % p)
        if not args.force:
            print("\n❌ 파일을 쓰지 않았다. 배선/node id 를 확인하고 다시 돌려라.")
            print("   (그래도 쓰려면 --force)")
            return 1

    if args.dry_run:
        print("\n(--dry-run) 쓰지 않은 내용:")
        print(json.dumps(registry, indent=2))
        return 0

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(registry, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print("\n💾 저장: %s" % args.out)

    verify_builder(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
