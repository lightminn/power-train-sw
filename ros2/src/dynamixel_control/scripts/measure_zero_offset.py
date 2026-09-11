#!/usr/bin/env python3
"""팔 관절의 영점(JOINT_CONFIG 의 `center` tick)을 기준 자세에서 실측하는 도구 (2026-08-07).

## 왜 필요한가

`JOINT_CONFIG` 의 `center: 2048` 은 전부 **"서보 중앙값이 곧 관절 0도"라는 가정**이고
확인된 적이 없다(`teleop_core_node.py` 의 `DEFAULT_CENTERS = [2048]*6` 도 마찬가지).
기어비를 맞춰도 영점이 틀리면 IK 가 낸 관절각이 통째로 그만큼 어긋나므로, 파지에는
기어비만큼이나 치명적이다.

실제로 2026-08-07 기어비 실측 직후 `arm_joint_4` 가 2.342 rad 로 읽혔는데 이 축의
URDF CAD 리밋은 [-0.611, 0.698] 이다 — 영점 미캘리브의 증상이다.

## 쓰는 법

**`moveit_dynamixel_bridge` 를 `read_only:=true` 로 띄운 상태에서** 쓴다(토크가 꺼져
있어야 팔을 기준 자세로 세울 수 있고, 이 스크립트는 아무것도 명령하지 않는다).

    ros2 run dynamixel_control moveit_dynamixel_bridge --ros-args -p read_only:=true
    python3 src/dynamixel_control/scripts/measure_zero_offset.py

1. 팔을 **기준 자세**로 세운다 (기본값은 URDF `home` = 전 관절 0도)
2. Enter → 그 순간의 tick 을 읽어 새 `center` 를 계산
3. 출력된 값을 `moveit_dynamixel_bridge.py` 의 JOINT_CONFIG 에 반영

기준 자세를 전 관절 0도로 세우기 어려우면(구조상 불가능한 축이 있다) `--reference`
로 축별 기준각을 rad 로 지정한다:

    python3 ... measure_zero_offset.py --reference arm_joint_3:1.5708

## 원리

브릿지가 발행하는 `/joint_states` 의 관절각을 tick 으로 역산한 뒤
(`tick = center_old + direction * rad * TICKS_PER_RAD * gear_ratio`),
기준 자세에서 그 tick 이 기준각에 대응하도록 center 를 다시 푼다:

    center_new = tick_now - direction * rad_ref * TICKS_PER_RAD * gear_ratio

⚠️ 기어비가 먼저 확정돼 있어야 한다(`measure_gear_ratio.py`). 기어비가 틀리면 영점도
   같이 틀어진다 — 순서를 바꾸지 말 것.
⚠️ 이 스크립트는 **실행 중인 브릿지의 `gear_ratios` 파라미터를 직접 조회**해서 쓴다.
   브릿지를 다른 기어비로 띄웠어도 자동으로 맞춰진다.
"""
import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState

try:
    from dynamixel_control import calib_math
    from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
except ImportError:
    # measure_gear_ratio.py 와 달리 이 스크립트는 JOINT_CONFIG(ID/방향/기어비)가 필요해서
    # 워크스페이스 오버레이가 소싱돼 있어야 한다 — 안 되어 있으면 raw traceback 대신
    # 무엇을 해야 하는지 알려준다(같은 셸에서 gear_ratio 스크립트는 잘 돌았는데 이건
    # 안 된다는 상황이 헷갈리기 쉬움).
    sys.stderr.write(
        "dynamixel_control 패키지를 import 할 수 없습니다 — 워크스페이스 오버레이가\n"
        "소싱되지 않은 셸로 보입니다. 다음을 먼저 실행하세요:\n\n"
        "    source /root/ros2_ws/install/setup.bash\n\n"
        "빌드를 안 했다면: colcon build --packages-select dynamixel_control\n"
    )
    sys.exit(1)

BRIDGE_NODE = "moveit_dynamixel_bridge"


class ZeroOffsetMeasurer(Node):
    def __init__(self):
        super().__init__("measure_zero_offset")
        self.latest = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if name in JOINT_CONFIG and i < len(msg.position):
                self.latest[name] = msg.position[i]

    def read_now(self, timeout_s=10.0):
        """버퍼를 비우고 팔 관절 전체가 담긴 새 샘플을 하나 받는다."""
        self.latest = {}
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout_s
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if set(self.latest) >= set(JOINT_CONFIG):
                return dict(self.latest)
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                return dict(self.latest) if self.latest else None
        return None

    def fetch_bridge_overrides(self, timeout_s=5.0):
        """실행 중인 브릿지의 `gear_ratios`·`centers` 파라미터를 조회.

        브릿지가 CLI 로 기어비나 영점을 덮어쓴 채 떠 있으면 JOINT_CONFIG 기본값과
        달라지는데, 그걸 모르고 역산하면 tick 이 틀린다 — 그래서 추측하지 않고 직접
        물어본다. **발행된 관절각은 브릿지가 지금 쓰는 center 로 환산된 값**이라
        영점도 기어비와 똑같이 조회해야 한다.
        조회 실패 시 None(호출부가 기본값으로 폴백하고 경고한다).
        """
        client = self.create_client(GetParameters, f"/{BRIDGE_NODE}/get_parameters")
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        future = client.call_async(
            GetParameters.Request(names=["gear_ratios", "centers"]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        response = future.result()
        if response is None or len(response.values) < 2:
            return None

        def parse(values, cast):
            out = {}
            for entry in values.string_array_value:
                name, _, value = str(entry).partition(":")
                if name in JOINT_CONFIG:
                    try:
                        out[name] = cast(value)
                    except ValueError:
                        pass
            return out

        return (parse(response.values[0], float),
                parse(response.values[1], lambda v: int(round(float(v)))))


def parse_reference(entries):
    """["arm_joint_3:1.5708", ...] → {"arm_joint_3": 1.5708}"""
    reference = {}
    for entry in entries:
        name, _, value = str(entry).partition(":")
        if name not in JOINT_CONFIG:
            raise ValueError(f"모르는 관절: {name!r} (가능: {', '.join(JOINT_CONFIG)})")
        try:
            reference[name] = float(value)
        except ValueError:
            raise ValueError(f"각도가 숫자가 아닙니다: {entry!r}")
    return reference


def main():
    parser = argparse.ArgumentParser(
        description="기준 자세에서 팔 관절 영점(center tick)을 실측한다.")
    parser.add_argument(
        "--reference", nargs="*", default=[], metavar="JOINT:RAD",
        help="축별 기준각(rad). 생략한 축은 0.0(URDF home). 예: arm_joint_3:1.5708")
    args = parser.parse_args()

    try:
        reference = parse_reference(args.reference)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1

    rclpy.init()
    node = ZeroOffsetMeasurer()

    try:
        domain = os.environ.get("ROS_DOMAIN_ID", "0(미설정)")
        print(f"/joint_states 수신 대기 중... (ROS_DOMAIN_ID={domain})")
        sample = node.read_now()
        if not sample:
            print(
                "관절각을 못 받았습니다. 확인 순서:\n"
                f"  1) ROS_DOMAIN_ID 가 bridge 와 같은가? (지금 이 셸은 {domain})\n"
                "  2) bridge 가 read_only:=true 로 떠 있는가? "
                "(ros2 topic hz /joint_states)",
                file=sys.stderr,
            )
            return 1

        missing = set(JOINT_CONFIG) - set(sample)
        if missing:
            print(f"⚠️ 이번 샘플에 없는 관절: {sorted(missing)} — 그 축은 건너뜁니다.")

        # 브릿지가 실제로 쓰고 있는 기어비·영점을 그대로 가져온다(추측 금지).
        fetched = node.fetch_bridge_overrides()
        if fetched is None:
            print("⚠️ 브릿지의 gear_ratios/centers 파라미터를 조회하지 못했습니다 — "
                  "JOINT_CONFIG 기본값으로 역산합니다. 브릿지를 CLI 로 다른 값으로 "
                  "띄웠다면 결과가 틀립니다.")
            fetched = ({}, {})
        ratio_overrides, center_overrides = fetched

        def ratio_of(name):
            return ratio_overrides.get(name, JOINT_CONFIG[name]["gear_ratio"])

        def center_of(name):
            return center_overrides.get(name, JOINT_CONFIG[name]["center"])

        print("\n현재 기어비: "
              + ", ".join(f"{n}={ratio_of(n):.3f}:1" for n in JOINT_CONFIG))
        if center_overrides:
            print("현재 영점(파라미터로 덮어쓴 축): "
                  + ", ".join(f"{n}={c}" for n, c in center_overrides.items()))
        print("기준 자세:   "
              + ", ".join(f"{n}={reference.get(n, 0.0):+.4f} rad" for n in JOINT_CONFIG))

        input("\n팔을 기준 자세로 세운 뒤 Enter ")
        sample = node.read_now()
        if not sample:
            print("기준 자세 샘플 수신 실패", file=sys.stderr)
            return 1

        print("\n" + "=" * 70)
        results = {}
        for name, config in JOINT_CONFIG.items():
            if name not in sample:
                continue
            ratio = ratio_of(name)
            center_old = center_of(name)

            # 식은 calib_math 한 곳에만 있다 — 관제 GUI 의 영점 마법사도 같은 함수를
            # 부르므로 두 경로가 갈라질 수 없다.
            center_new = calib_math.center_from_measurement(
                center_old, config["direction"], ratio,
                rad_measured=sample[name], rad_ref=reference.get(name, 0.0))
            results[name] = center_new

            shift = center_new - center_old
            print(f"  {name}: center {center_old} → {round(center_new)} "
                  f"({shift:+.0f} tick, "
                  f"{calib_math.center_shift_deg(shift, ratio):+.2f}° 관절)")

            reason = calib_math.center_out_of_range(center_new, config["extended"])
            if reason is not None:
                print(f"      ⚠️ {reason}")
        print("=" * 70)

        print("\nmoveit_dynamixel_bridge.py 의 JOINT_CONFIG 에 반영:\n")
        for name, center_new in results.items():
            config = JOINT_CONFIG[name]
            print(calib_math.format_joint_config_entry(
                name, config["id"], center_new, config["direction"],
                ratio_of(name), config["extended"]))

        # 재빌드 없이 그 자리에서 시험해 볼 수 있는 경로도 같이 알려준다.
        # 항목 구분자는 **쉼표**다 — 공백으로 붙이면 ROS 가 배열로 파싱하지 못한다.
        centers = ",".join(f"'{n}:{round(c)}'" for n, c in results.items())
        print("\n재빌드 없이 브릿지에 바로 넣어 시험:\n")
        print(f'    -p centers:="[{centers}]"')

        print("\n⚠️ 반영 후 반드시 read_only 로 다시 띄워 /joint_states 가 기준 자세에서")
        print("   기준각을 가리키는지 확인할 것. 구동은 그 확인 뒤에.")
        print("⚠️ teleop_core_node.py 의 DEFAULT_CENTERS 는 **서보축 도메인**이라 이 값과")
        print("   다르다 — 그쪽은 별도로 두고, 여기 값을 그대로 복사하지 말 것.")
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
