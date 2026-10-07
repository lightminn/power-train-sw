#!/usr/bin/env python3
"""HW-7 그리퍼 반응 테스트: 병 인식 → 그리퍼 닫기 / 병 없음 → 그리퍼 열기.

랙피니언 그리퍼(XL430 2개, 기본 ID 3·4)를 브릿지(moveit_dynamixel_bridge) 경유 없이
dynamixel_sdk로 직접 제어한다. 두 모터는 같은 랙을 **같은 방향**으로 구동하므로 동일한
goal position(각도)을 양쪽에 쓴다. 포지션은 각도로 지정: 닫힘 280deg / 열림 215deg
(기본값, --close-deg/--open-deg로 변경 가능).

또한 두 모터의 PRESENT_CURRENT(126,2 signed)를 읽어 **max-abs(절댓값이 큰 쪽)** 전류를
주기적으로 출력한다 — effort threshold 실측·캘리브레이션(Notion 절차)에 그대로 쓰라고
넣었다. 런타임(moveit_dynamixel_bridge)이 파지/DROP 판정에 쓰는 값과 동일한 집계 방식이다.

/pick_target(DetectedObject)은 transient_local(latched)라 병이 사라져도 마지막 값이 남아있어
"병 없음"을 감지할 수 없다 → 매 프레임 발행되는 /detected_objects(DetectedObjectArray)를 구독해
현재 프레임에 target-class(기본 bottle)가 있는지로 판단한다.

사전 준비 (컨테이너 안, perception_node 먼저 기동):
    ros2 run robot_arm_perception perception_node --ros-args \\
        -p model_path:=/root/ros2_ws/yolov8n-seg.pt -p camera_mode:=realsense \\
        -p pick_classes:=bottle -p pick_min_conf:=0.5

실행 (기본 60초 동안 반응 테스트 후 자동 종료):
    python3 /root/ros2_ws/hw7_gripper_bottle_test.py --gripper-ids 3 4 --duration 60

원인 확인(2026-07-08): Profile Velocity/Acceleration이 기본 0(=최고속 즉시 이동)이라 매
동작마다 순간 전류가 튀어 과부하 보호로 토크가 자동 해제됨(Hardware Error Status는 조건
해소 후 바로 0으로 복귀해 겉으로는 안 보임). accel=10/velocity=30으로 낮추니 트립 없음을
실측 확인, accel=25/velocity=80까지 올려도 트립 없이 더 빠르게(약 0.6초) 도달함을 재확인
→ 기본값으로 반영. 토크 재확인/재활성화 하트비트는 방어용으로 계속 유지.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler

from robot_arm_msgs.msg import DetectedObjectArray

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
LEN_PRESENT_CURRENT = 2

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyUSB0"

POSITION_CONTROL_MODE = 3
TICKS_PER_REV = 4096
TICK_MIN = 0
TICK_MAX = 4095


def deg_to_tick(deg):
    tick = int(round(deg / 360.0 * TICKS_PER_REV))
    return max(TICK_MIN, min(TICK_MAX, tick))


def to_signed(value, byte_len):
    """무부호 정수를 byte_len 바이트 2의 보수 부호 정수로 변환 (PRESENT_CURRENT 용)."""
    bits = byte_len * 8
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value


class GripperBottleTest(Node):
    def __init__(self, args):
        super().__init__("hw7_gripper_bottle_test")
        self.gripper_ids = args.gripper_ids
        self.target_class = args.target_class
        self.min_conf = args.min_conf
        self.hold_frames = args.hold_frames

        self.close_tick = deg_to_tick(args.close_deg)
        self.open_tick = deg_to_tick(args.open_deg)

        self.port = PortHandler(DEVICENAME)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        if not self.port.openPort():
            raise RuntimeError(f"포트 열기 실패: {DEVICENAME}")
        if not self.port.setBaudRate(BAUDRATE):
            raise RuntimeError(f"보드레이트 설정 실패: {BAUDRATE}")

        for gid in self.gripper_ids:
            mode, result, error = self.packet.read1ByteTxRx(
                self.port, gid, ADDR_OPERATING_MODE
            )
            if result != 0 or error != 0:
                raise RuntimeError(
                    f"서보 응답 없음: id={gid}, result={result}, error={error} "
                    "— 그리퍼 서보 ID를 확인하라(--gripper-ids)"
                )
            if mode != POSITION_CONTROL_MODE:
                self.get_logger().warn(
                    f"id={gid} Operating Mode={mode} (Position Control=3 아님) "
                    "— 각도 명령이 의도대로 동작하지 않을 수 있음"
                )

            # 급가속으로 인한 순간 전류 스파이크(과부하 트립)를 막기 위해 천천히 움직이게
            # 설정 (실측: accel/vel=0=최고속일 때 매 동작마다 토크 자동 해제됨, 낮추니 해결)
            self.packet.write4ByteTxRx(
                self.port, gid, ADDR_PROFILE_ACCELERATION, args.profile_accel
            )
            self.packet.write4ByteTxRx(
                self.port, gid, ADDR_PROFILE_VELOCITY, args.profile_velocity
            )

            result, error = self.packet.write1ByteTxRx(
                self.port, gid, ADDR_TORQUE_ENABLE, 1
            )
            if result != 0 or error != 0:
                raise RuntimeError(
                    f"토크 활성화 실패: id={gid}, result={result}, error={error}"
                )
            self.get_logger().info(
                f"토크 활성화: gripper id={gid} "
                f"(profile accel={args.profile_accel}, velocity={args.profile_velocity})"
            )

        self.consec_present = 0
        self.consec_absent = 0

        # 시작 상태: 그리퍼 열림(병 없음 가정)
        self._write_position(self.open_tick)
        self.bottle_present = False
        self.get_logger().info(
            f"시작: 그리퍼 열림({args.open_deg}deg, tick={self.open_tick}) | "
            f"닫힘={args.close_deg}deg(tick={self.close_tick}) | "
            f"gripper_ids={self.gripper_ids} target_class={self.target_class!r} "
            f"min_conf={self.min_conf} hold_frames={self.hold_frames}"
        )

        self.sub = self.create_subscription(
            DetectedObjectArray, "/detected_objects", self.on_detections, 10
        )

        # 토크가 원인 불명으로 풀리는 현상 방어용 하트비트 (실측: 위치는 도달하는데
        # Torque Enable 레지스터가 0으로 읽힌 사례가 있었음)
        self.torque_check_timer = self.create_timer(2.0, self._reassert_torque)

        # effort 캘리브레이션용 전류 리드아웃 (0 이하면 비활성)
        if args.current_readout_hz > 0:
            self.create_timer(1.0 / args.current_readout_hz, self._print_current)

    def _read_current(self, gid):
        """모터 하나의 PRESENT_CURRENT(signed) 읽기. 실패 시 None."""
        raw, result, error = self.packet.read2ByteTxRx(
            self.port, gid, ADDR_PRESENT_CURRENT
        )
        if result != 0 or error != 0:
            return None
        return to_signed(raw, LEN_PRESENT_CURRENT)

    def _print_current(self):
        """두 모터 전류 + max-abs(런타임 파지/DROP 판정과 동일 집계)를 출력."""
        currents = {gid: self._read_current(gid) for gid in self.gripper_ids}
        valid = [c for c in currents.values() if c is not None]
        if not valid:
            self.get_logger().warn("전류 읽기 실패(전 모터 무응답)")
            return
        max_abs = max(valid, key=abs)
        per_motor = ", ".join(
            f"id{gid}={'--' if c is None else c}" for gid, c in currents.items()
        )
        self.get_logger().info(f"[전류] {per_motor} | max-abs={abs(max_abs)}")

    def _reassert_torque(self):
        for gid in self.gripper_ids:
            torque, result, error = self.packet.read1ByteTxRx(
                self.port, gid, ADDR_TORQUE_ENABLE
            )
            if result != 0 or error != 0 or torque != 1:
                self.get_logger().warn(
                    f"토크 꺼짐 감지(id={gid}, torque={torque}) -> 재활성화 시도"
                )
                self.packet.write1ByteTxRx(self.port, gid, ADDR_TORQUE_ENABLE, 1)

    def on_detections(self, msg):
        found = any(
            obj.class_name == self.target_class and obj.confidence >= self.min_conf
            for obj in msg.objects
        )
        if found:
            self.consec_present += 1
            self.consec_absent = 0
        else:
            self.consec_absent += 1
            self.consec_present = 0

        if not self.bottle_present and self.consec_present >= self.hold_frames:
            self.bottle_present = True
            self._write_position(self.close_tick)
            self.get_logger().info(
                f"[{self.target_class} 인식] 그리퍼 닫기 -> tick {self.close_tick}"
            )
        elif self.bottle_present and self.consec_absent >= self.hold_frames:
            self.bottle_present = False
            self._write_position(self.open_tick)
            self.get_logger().info(
                f"[{self.target_class} 없음] 그리퍼 열기 -> tick {self.open_tick}"
            )

    def _write_position(self, tick):
        # 두 모터에 동일 goal position(같은 방향 랙피니언). 위치 명령 직전에
        # 토크를 재확인/재활성화 (토크 풀림 방어).
        for gid in self.gripper_ids:
            self.packet.write1ByteTxRx(self.port, gid, ADDR_TORQUE_ENABLE, 1)
            result, error = self.packet.write4ByteTxRx(
                self.port, gid, ADDR_GOAL_POSITION, tick
            )
            if result != 0 or error != 0:
                self.get_logger().warn(
                    f"그리퍼 위치 명령 실패: id={gid}, result={result}, error={error}"
                )

    def destroy_node(self):
        for gid in self.gripper_ids:
            self.packet.write1ByteTxRx(self.port, gid, ADDR_TORQUE_ENABLE, 0)
        self.port.closePort()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--gripper-ids",
        type=int,
        nargs="+",
        default=[3, 4],
        help="랙피니언 그리퍼 XL430 다이나믹셀 ID들(같은 방향 구동, 동일 각도 명령)",
    )
    parser.add_argument("--target-class", default="bottle", help="인식 대상 YOLO 클래스명")
    parser.add_argument("--min-conf", type=float, default=0.5, help="인식 최소 confidence")
    parser.add_argument("--close-deg", type=float, default=280.0, help="닫힘 포지션(도)")
    parser.add_argument("--open-deg", type=float, default=215.0, help="열림 포지션(도)")
    parser.add_argument(
        "--hold-frames",
        type=int,
        default=3,
        help="상태 전환 전 연속 프레임 요구 수(디바운스, 검출 플리커 방지)",
    )
    parser.add_argument(
        "--duration", type=float, default=60.0, help="테스트 지속 시간(초), 이후 자동 종료"
    )
    parser.add_argument(
        "--current-readout-hz",
        type=float,
        default=2.0,
        help="두 모터 전류 max-abs 출력 주기(Hz). effort 캘리브레이션용. 0이면 비활성",
    )
    parser.add_argument(
        "--profile-accel",
        type=int,
        default=25,
        help="Profile Acceleration (0=최고속 즉시 이동 → 과부하 트립 유발. 10은 안전하나 느림, "
        "25까지 올려도 트립 없음을 실측 확인)",
    )
    parser.add_argument(
        "--profile-velocity",
        type=int,
        default=80,
        help="Profile Velocity (0=최고속 → 과부하 트립 유발. 30은 안전하나 느림, "
        "80까지 올려도 트립 없음을 실측 확인)",
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GripperBottleTest(args)
    deadline = time.time() + args.duration
    try:
        node.get_logger().info(f"{args.duration:.0f}초 동안 반응 테스트 실행...")
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info("테스트 시간 종료 -> 정지")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
