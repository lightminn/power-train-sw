#!/usr/bin/env python3
"""그리퍼 개폐 끝단 tick 을 손으로 움직여 실측한다 (2026-08-07).

## 왜 필요한가

`gripper_presets.py` 의 `gripper_open_tick=2446` / `gripper_close_tick=3186` 은
HW-8 시절 **단일 서보 ID 5** 로 잰 유산값이라 현재 조립(ID 3 랙피니언)과 맞지 않는다
— CLAUDE.md 에도 "미검증" 으로 적혀 있었고, 2026-08-07 읽기 검증에서 실제로 틀린 게
확인됐다: 그리퍼가 tick 974 에 있는데 이 캘리브로는 **5.81 rad** 로 보고된다
(URDF 가동범위 0~1.9444 rad 의 3배).

이 상태로 FSM 이 "열어/닫아" 를 명령하면 1500~2200 tick 을 예상 못 한 방향으로
움직인다. 파지 동작 자체가 이 값에 걸려 있다.

## 무엇을 재는가

rad 끝단(`gripper_open_rad`=1.9444 / `gripper_close_rad`=0.0)은 URDF 가 정하므로
건드리지 않는다. **그 두 자세에 대응하는 서보 tick 두 개**만 다시 잰다.

## 쓰는 법

**`moveit_dynamixel_bridge` 를 `read_only:=true` 로** 띄운 뒤(토크 OFF — 손으로 움직일
수 있어야 한다):

    bash src/dynamixel_control/scripts/run_calib.sh gripper

완전히 닫은 자세에서 Enter, 완전히 연 자세에서 Enter. 각 단계마다 현재 tick 이
실시간으로 표시되니 **값이 더 이상 안 변하는 걸 확인하고** 누른다.

⚠️ 랙 끝단에 억지로 밀어 넣지 말 것. 거기가 캘리브 끝단으로 박히면 운용 중 매번
   그 지점까지 가려 들고, 그리퍼는 특히 과전류 토크 트립이 잘 난다.

## 원리 (포트를 열지 않는다)

브릿지가 `/joint_states` 로 내보내는 그리퍼 rad 를 **현재 캘리브로 역산**해 tick 을
복원한다(`gripper_tick_to_pos` 의 역). 그래서 브릿지가 시리얼 포트를 잡고 있는 채로
그대로 쓸 수 있다 — `gripper_calibration.py`(포트 직접 오픈, 2모터 전제)와 다른 점이다.
"""
import argparse
import os
import select
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from dynamixel_control import calib_math
    from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset
except ImportError:
    sys.stderr.write(
        "dynamixel_control 패키지를 import 할 수 없습니다 — 워크스페이스 오버레이가\n"
        "소싱되지 않은 셸로 보입니다. 다음을 먼저 실행하세요:\n\n"
        "    source /root/ros2_ws/install/setup.bash\n"
    )
    sys.exit(1)


class GripperEndpointMeasurer(Node):
    def __init__(self, joint_name, preset):
        super().__init__("measure_gripper_endpoints")
        self.joint_name = joint_name
        self.open_tick = int(preset["gripper_open_tick"])
        self.close_tick = int(preset["gripper_close_tick"])
        self.open_rad = float(preset["gripper_open_rad"])
        self.close_rad = float(preset["gripper_close_rad"])
        self.latest_tick = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

    def _rad_to_tick(self, rad):
        """브릿지 `gripper_tick_to_pos` 의 역 — 발행된 rad 에서 raw tick 을 복원."""
        denom = self.open_rad - self.close_rad
        if denom == 0.0:
            return None
        frac = (rad - self.close_rad) / denom
        return self.close_tick + frac * (self.open_tick - self.close_tick)

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if name == self.joint_name and i < len(msg.position):
                self.latest_tick = self._rad_to_tick(msg.position[i])
                return

    def wait_for_sample(self, timeout_s=10.0):
        self.latest_tick = None
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout_s
        while rclpy.ok() and self.latest_tick is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                return None
        return self.latest_tick

    def capture_until_enter(self, label):
        """Enter 까지 현재 tick 을 라이브 표시하고, 누른 순간의 값을 반환.

        측정자가 "더 이상 안 변한다"를 눈으로 확인하고 끝낼 수 있어야 한다 —
        관절 리밋 측정에서 이게 없어 끝단에 못 미친 값을 기록한 전례가 있다.
        """
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_tick is not None:
                sys.stdout.write(f"\r  [{label}] 현재 tick {self.latest_tick:9.1f}   ")
                sys.stdout.flush()
            if select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.readline()
                return self.latest_tick
        return None


def main():
    parser = argparse.ArgumentParser(
        description="그리퍼 개폐 끝단 tick 을 실측한다.")
    parser.add_argument("--gripper-type", default=DEFAULT_GRIPPER,
                        help=f"gripper_presets 의 preset 이름 (기본 {DEFAULT_GRIPPER})")
    parser.add_argument("--margin", type=int, default=0, metavar="TICK",
                        help="양 끝단에서 뺄 안전 마진 [tick] (기본 0 — "
                             "끝단까지 실제로 쓰려면 0, 여유를 두려면 20~50)")
    args = parser.parse_args()

    preset = get_preset(args.gripper_type)
    joint_name = preset["gripper_joints"][0]

    rclpy.init()
    node = GripperEndpointMeasurer(joint_name, preset)

    try:
        domain = os.environ.get("ROS_DOMAIN_ID", "0(미설정)")
        print(f"[{joint_name}] /joint_states 수신 대기 중... (ROS_DOMAIN_ID={domain})")
        if node.wait_for_sample() is None:
            print(
                "그리퍼 상태를 못 받았습니다. 확인 순서:\n"
                f"  1) ROS_DOMAIN_ID 가 bridge 와 같은가? (지금 이 셸은 {domain})\n"
                "  2) bridge 가 read_only:=true 로 떠 있는가?\n"
                f"  3) preset '{args.gripper_type}' 의 gripper_ids 서보가 응답하는가?",
                file=sys.stderr,
            )
            return 1

        print(f"  현재 등록값: close_tick={node.close_tick}, open_tick={node.open_tick} "
              f"(HW-8 유산값 — 이번에 덮어쓸 대상)")
        print("\n⚠️ 랙 끝단에 억지로 밀어 넣지 마세요.\n")

        input("준비되면 Enter (다음 단계부터 tick 이 실시간 표시됩니다) ")

        print("\n1) 그리퍼를 **완전히 닫은** 자세로 두세요. 값이 멈추면 Enter.")
        closed = node.capture_until_enter("닫힘")
        if closed is None:
            print("\n닫힘 tick 수신 실패", file=sys.stderr)
            return 1
        print(f"\n   닫힘 tick = {closed:.1f}")

        print("\n2) 그리퍼를 **완전히 연** 자세로 두세요. 값이 멈추면 Enter.")
        opened = node.capture_until_enter("열림")
        if opened is None:
            print("\n열림 tick 수신 실패", file=sys.stderr)
            return 1
        print(f"\n   열림 tick = {opened:.1f}")

        # 마진 부호 처리와 경고 판정은 calib_math 한 곳에 있다 — 관제 GUI 의 그리퍼
        # 마법사도 같은 함수를 부르므로 두 경로가 갈라질 수 없다.
        try:
            result = calib_math.gripper_endpoints(closed, opened, args.margin)
        except ValueError as exc:
            print(f"\n{exc}", file=sys.stderr)
            return 1
        close_final, open_final = result["close"], result["open"]

        print("\n" + "=" * 70)
        print(f"  gripper_close_tick = {close_final}")
        print(f"  gripper_open_tick  = {open_final}")
        print(f"  (stroke {result['stroke_tick']} tick "
              f"= {result['stroke_deg']:.1f}° 서보축"
              + (f", 양끝 마진 {args.margin} tick 적용" if args.margin else "") + ")")
        print("=" * 70)

        for warning in result["warnings"]:
            print(f"\n  ⚠️ {warning}")

        print("\ngripper_presets.py 의 GRIPPER_PRESETS['%s'] 에 반영:\n"
              % args.gripper_type)
        print(f'        "gripper_open_tick": {open_final},')
        print(f'        "gripper_close_tick": {close_final},')
        print("\n⚠️ 반영 후 read_only 로 다시 띄워 /joint_states 의 그리퍼 값이")
        print(f"   {node.close_rad}~{node.open_rad} rad 범위 안에 들어오는지 확인할 것.")
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
