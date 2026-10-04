#!/usr/bin/env python3
"""팔 관절의 안전 가동범위를 양쪽 하드스톱까지 손으로 밀어 실측한다 (2026-08-07).

## 왜 필요한가

`dynamixel_control/joint_limits.py` 가 관절 리밋의 단일 출처인데, 지금 값 중 실제로
이 캘리브 도메인에서 잰 건 하나도 없다 — `arm_joint_3/4` 는 2026-08-02 **서보 도메인**
측정을 환산한 것이고(`derived`), `arm_joint_2/5` 는 아예 실측이 없어 좁게 잠가뒀다
(`provisional`). 이 스크립트가 그걸 `measured` 로 바꾼다.

## 순서 (바꾸지 말 것)

기어비 → 영점 → **가동범위**. 앞의 둘이 확정돼야 관절 도메인이 정해지고, 그래야
여기서 잰 값이 의미가 있다. `center`/`gear_ratio` 를 다시 만지면 여기 값도 무효다.

## 쓰는 법

**`moveit_dynamixel_bridge` 를 `read_only:=true` 로** 띄운 뒤(토크 OFF — 손으로 밀 수
있어야 한다), 축 하나씩:

    bash src/dynamixel_control/scripts/run_calib.sh limits arm_joint_4

관절을 양쪽 끝까지 **천천히 왕복**시키면 현재값과 누적 최소/최대가 실시간으로
표시된다. **양쪽 다 값이 더 이상 안 변하는 걸 확인한 뒤** Enter 를 누른다.

> 초기 버전은 "한쪽 끝에서 Enter, 반대쪽에서 Enter" 로 순간값만 잡았는데, 미는
> 도중 어디까지 갔는지 볼 수가 없어 하드스톱에 못 미친 지점을 스톱으로 기록했다
> (2026-08-07 실기: arm_joint_4 가 기록된 상한보다 46° 위까지 실제로 움직였고,
> 3개 축에서 home 이 범위 밖으로 나오는 값이 나왔다). 그래서 라이브 방식으로 바꿨다.

⚠️ **하드스톱에 세게 부딪히지 말 것.** 기구가 상하고, 억지로 밀어 넣은 위치가
   리밋으로 박히면 실제 운용에서 매번 그 지점까지 가려 든다.
⚠️ 출력은 안전 마진을 뺀 값이다(`--margin`, 기본 2°). 하드스톱 바로 앞까지 리밋을
   열어두면 서보가 정지 오차 때문에 매번 스톱을 때린다.
"""
import argparse
import math
import os
import select
import sys
from datetime import date

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
    from dynamixel_control import joint_limits
except ImportError:
    sys.stderr.write(
        "dynamixel_control 패키지를 import 할 수 없습니다 — 워크스페이스 오버레이가\n"
        "소싱되지 않은 셸로 보입니다. 다음을 먼저 실행하세요:\n\n"
        "    source /root/ros2_ws/install/setup.bash\n\n"
        "빌드를 안 했다면: colcon build --packages-select dynamixel_control\n"
    )
    sys.exit(1)


class JointLimitMeasurer(Node):
    def __init__(self, joint_name):
        super().__init__("measure_joint_limits")
        self.joint_name = joint_name
        self.latest = None
        self.seen_min = None
        self.seen_max = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if name == self.joint_name and i < len(msg.position):
                rad = msg.position[i]
                self.latest = rad
                # 이동 중 지나간 극값도 같이 추적한다 — 사용자가 스톱에서 살짝
                # 되돌아온 뒤 Enter 를 눌러도 실제 도달점을 놓치지 않게.
                self.seen_min = rad if self.seen_min is None else min(self.seen_min, rad)
                self.seen_max = rad if self.seen_max is None else max(self.seen_max, rad)
                return

    def read_now(self, timeout_s=10.0):
        self.latest = None
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout_s
        while rclpy.ok() and self.latest is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                return None
        return self.latest

    def reset_extremes(self):
        self.seen_min = None
        self.seen_max = None

    def sweep_until_enter(self):
        """Enter 를 누를 때까지 스핀하며 현재값·누적 최소/최대를 한 줄에 라이브 표시.

        측정자가 "값이 더 이상 안 변한다"를 눈으로 확인하고 끝낼 수 있어야 한다 —
        이게 없으면 하드스톱에 도달했는지 감으로 판단하게 되고, 실제로 그래서
        틀린 값이 나왔다(모듈 상단 주석 참고).
        """
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is not None:
                sys.stdout.write(
                    f"\r  현재 {math.degrees(self.latest):+8.2f}°   "
                    f"최소 {math.degrees(self.seen_min):+8.2f}°   "
                    f"최대 {math.degrees(self.seen_max):+8.2f}°   "
                    f"(span {math.degrees(self.seen_max - self.seen_min):6.2f}°)  "
                )
                sys.stdout.flush()
            # stdin 에 입력이 들어왔는지 논블로킹 확인 — spin 을 멈추면 라이브 표시가
            # 죽으므로 input() 을 쓸 수 없다.
            if select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.readline()
                return


def main():
    parser = argparse.ArgumentParser(
        description="관절 가동범위를 하드스톱까지 밀어 실측한다.")
    parser.add_argument("joint_name", help=f"측정할 관절 ({', '.join(JOINT_CONFIG)})")
    parser.add_argument("--margin", type=float, default=2.0, metavar="DEG",
                        help="하드스톱에서 뺄 안전 마진 [도] (기본 2.0)")
    args = parser.parse_args()

    if args.joint_name not in JOINT_CONFIG:
        print(f"모르는 관절: {args.joint_name!r} (가능: {', '.join(JOINT_CONFIG)})",
              file=sys.stderr)
        return 1
    if args.margin < 0:
        print("--margin 은 0 이상이어야 합니다.", file=sys.stderr)
        return 1

    rclpy.init()
    node = JointLimitMeasurer(args.joint_name)

    try:
        domain = os.environ.get("ROS_DOMAIN_ID", "0(미설정)")
        print(f"[{args.joint_name}] /joint_states 수신 대기 중... (ROS_DOMAIN_ID={domain})")
        if node.read_now() is None:
            print(
                "관절각을 못 받았습니다. 확인 순서:\n"
                f"  1) ROS_DOMAIN_ID 가 bridge 와 같은가? (지금 이 셸은 {domain})\n"
                "  2) bridge 가 read_only:=true 로 떠 있는가?",
                file=sys.stderr,
            )
            return 1

        current = joint_limits.get_limits(args.joint_name)
        if current is not None:
            entry = joint_limits.JOINT_LIMITS[args.joint_name]
            print(f"  현재 등록값: [{current[0]:+.4f}, {current[1]:+.4f}] rad "
                  f"({entry['confidence']})")
        print("\n⚠️ 하드스톱에 세게 부딪히지 말고 닿는 지점까지만 부드럽게 미세요.\n")

        # 라이브 스윕 방식. Enter 두 번으로 순간값만 잡던 초기 방식은 2026-08-07 실기에서
        # 실패했다 — 미는 도중 실제로 어디까지 갔는지 볼 수가 없어서 하드스톱에 못 미친
        # 지점을 스톱으로 기록했고(arm_joint_4 는 기록된 상한보다 46° 위까지 실제로
        # 움직였다), 그 결과 home 이 범위 밖으로 나오는 값이 나왔다. 지금은 현재값과
        # 누적 최소/최대를 계속 띄워서, 값이 더 안 변하는 걸 **눈으로 확인하고** 끝낸다.
        print("관절을 양쪽 끝까지 천천히 왕복시키세요. 다 됐으면 Enter.\n")
        node.reset_extremes()
        node.sweep_until_enter()

        lower, upper = node.seen_min, node.seen_max
        if lower is None or upper is None:
            print("스윕 중 관절각을 못 받았습니다.", file=sys.stderr)
            return 1
        span = upper - lower
        print(f"\n   끝단 = {lower:+.4f} / {upper:+.4f} rad "
              f"({math.degrees(lower):+.2f}° / {math.degrees(upper):+.2f}°)")
        print(f"\n   도달 범위 = [{lower:+.4f}, {upper:+.4f}] rad "
              f"(span {math.degrees(span):.2f}°)")

        if span < 0.05:
            print("\n관절이 거의 안 움직였습니다 — 실제로 양쪽 끝단까지 미셨나요?",
                  file=sys.stderr)
            return 1

        margin = math.radians(args.margin)
        if span <= 2 * margin:
            print(f"\n가동범위({math.degrees(span):.2f}°)가 안전 마진 양쪽 합"
                  f"({2 * args.margin:.1f}°)보다 좁습니다 — --margin 을 줄이거나 "
                  "측정을 다시 하세요.", file=sys.stderr)
            return 1

        safe_lower = lower + margin
        safe_upper = upper - margin

        print("\n" + "=" * 70)
        print(f"  {args.joint_name}: [{safe_lower:+.4f}, {safe_upper:+.4f}] rad")
        print(f"      = [{math.degrees(safe_lower):+.2f}°, {math.degrees(safe_upper):+.2f}°]"
              f"  (하드스톱에서 각 {args.margin:.1f}° 뺌)")
        print("=" * 70)

        if not safe_lower <= 0.0 <= safe_upper:
            print("\n  ⚠️ home(0 rad)이 이 범위 밖입니다 — 영점(center)이 틀렸거나")
            print("     측정이 잘못됐습니다. 이대로 쓰면 IK 가 home 근처를 못 씁니다.")

        print("\njoint_limits.py 의 JOINT_LIMITS 에 반영:\n")
        print(f'    "{args.joint_name}": {{')
        print(f'        "lower": {safe_lower:+.4f},')
        print(f'        "upper": {safe_upper:+.4f},')
        print('        "confidence": "measured",')
        # 날짜는 **측정 시점**을 찍는다. 예전엔 "2026-08-07" 이 문자열에 박혀 있어서,
        # 그 뒤 회차의 측정값도 전부 8/7 에 잰 것처럼 기록됐다(2026-08-19 재조립
        # 회차에서 발견). 영점·기어비가 바뀌면 리밋도 무효라 "언제 잰 값인가"가
        # 이 필드의 존재 이유인데, 고정 문자열이면 그걸 정확히 못 쓴다.
        print(f'        "source": "{date.today().isoformat()} 하드스톱 실측 '
              f'(span {math.degrees(span):.1f}°, 마진 {args.margin:.1f}°)",')
        print("    },")
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
