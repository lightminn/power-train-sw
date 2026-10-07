#!/usr/bin/env python3
"""하드웨어 없이 관제 GUI 를 end-to-end 로 검증하는 가짜 발행자.

`ros2 topic pub` 으로는 이미지(sensor_msgs/Image)를 실용적으로 만들 수 없고,
QoS 를 토픽마다 맞춰 주는 것도 번거롭다. 이 노드는 GUI 가 구독하는 토픽 전부를
**실제와 같은 QoS·주기·페이로드 규약**으로 발행한다.

특히 아래 함정 케이스를 **일부러** 재현한다 — 화면이 이것들을 올바로 그리는지가
인수 기준이다.

- ID 12 의 온도를 계속 `0` 으로 둔다 → "0°C 정상"이 아니라 `—`(미수신)여야 한다.
- hw 에러 문자열에 `전류급변(SW,비상정지)` 를 넣는다 → 라벨 안의 쉼표 때문에
  단순 split 이면 모터가 하나 더 생긴다.
- 검출 하나는 `position.z = 0.0` 으로 둔다 → "원점"이 아니라 `깊이 없음`이다.
- `/pick_target` 은 transient_local 로 **한 번만** 발행한다 → GUI 를 나중에 띄워도
  받아야 한다.
- 주기적으로 전류를 급상승시켜 트립 임계에 근접시킨다 → 여유 미터와 트립
  블랙박스가 동작하는지 본다.

⚠️ 이 노드는 **벤치 검증 전용**이다. 실기와 함께 띄우면 가짜 `/dynamixel/state`
가 진짜와 섞인다. 대회 launch 에 넣지 말 것.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from control_msgs.msg import JointJog
from sensor_msgs.msg import Image, JointState, Joy
from std_msgs.msg import Bool, Int32MultiArray, String

from robot_arm_msgs.msg import ArmStatus, ArrivalStatus, ChassisMode
from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray

from dynamixel_control.qos_profiles import ARRIVAL_QOS, HEARTBEAT_QOS


LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

#: 실기 배선(`dynamixel_position_node` 기본값)과 같은 순서.
MOTORS = [
    (11, 'arm_joint_1'),
    (14, 'arm_joint_2'),
    (13, 'arm_joint_3'),
    (12, 'arm_joint_4'),
    (16, 'arm_joint_5'),
    (3, 'gripper_left_pinion_joint'),
]

HW_ERROR_TEXT = ('arm_joint_2(ID14):과부하,'
                 'gripper_left_pinion_joint(ID3):전류급변(SW,비상정지)|과열')


class FakePublisher(Node):

    def __init__(self):
        super().__init__('robot_arm_gui_fake_publisher')

        self.declare_parameter('width', 848)
        self.declare_parameter('height', 480)
        self.declare_parameter('image_fps', 15.0)
        # 이 초가 지나면 hw 에러를 한 번 올렸다가(상승 엣지 + 트립 블랙박스)
        # 다시 내린다(하강 엣지). 0 이면 에러를 아예 안 낸다.
        self.declare_parameter('fault_at_s', 12.0)
        self.declare_parameter('fault_clear_after_s', 8.0)

        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.fault_at = float(self.get_parameter('fault_at_s').value)
        self.fault_clear = float(self.get_parameter('fault_clear_after_s').value)

        self.pub_state = self.create_publisher(Int32MultiArray, '/dynamixel/state', 10)
        self.pub_hw = self.create_publisher(String, '/dynamixel/hardware_error', 10)
        self.pub_goal = self.create_publisher(
            Int32MultiArray, '/dynamixel/goal_position', 10)
        self.pub_limits = self.create_publisher(
            Int32MultiArray, '/dynamixel/tick_limits', LATCHED)
        self.pub_fault = self.create_publisher(Bool, '/dynamixel/controller_fault', 10)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_arm = self.create_publisher(ArmStatus, '/arm_status', HEARTBEAT_QOS)
        self.pub_chassis = self.create_publisher(ChassisMode, '/chassis_mode', HEARTBEAT_QOS)
        self.pub_arrival = self.create_publisher(
            ArrivalStatus, '/arrival_status', ARRIVAL_QOS)
        self.pub_det = self.create_publisher(DetectedObjectArray, '/detected_objects', 10)
        self.pub_pick = self.create_publisher(DetectedObject, '/pick_target', LATCHED)
        self.pub_jog = self.create_publisher(JointJog, '/arm/teleop_jog', 10)
        self.pub_cmd = self.create_publisher(String, '/arm/teleop_cmd', 10)
        self.pub_poses = self.create_publisher(String, '/arm/teleop_poses', LATCHED)
        # 캘리브 패널을 하드웨어 없이 검증하려면 이 셋이 필요하다 — 실기에서는
        # teleop_core(/arm/calib_status)와 position_node(임계 설정)가 낸다.
        self.pub_calib = self.create_publisher(String, '/arm/calib_status', LATCHED)
        self.pub_trip_cfg = self.create_publisher(
            Int32MultiArray, '/dynamixel/current_trip_config', 10)
        self.pub_spike_cfg = self.create_publisher(
            Int32MultiArray, '/dynamixel/current_spike_config', 10)
        self.pub_joy = self.create_publisher(Joy, '/joy', 10)
        self.pub_debug_img = self.create_publisher(Image, '/perception/debug_image', 1)
        self.pub_raw_img = self.create_publisher(Image, '/perception/raw_image', 1)

        self.t0 = time.monotonic()
        self.tick = 0
        self.fault_raised = False
        self.fault_cleared = False

        # latched 는 한 번만 — GUI 를 나중에 띄워도 받아야 한다.
        self._publish_latched_once()

        self.create_timer(1.0 / 30.0, self._on_30hz)     # 서보 드라이버 주기
        self.create_timer(1.0 / 10.0, self._on_10hz)     # 계약 heartbeat
        self.create_timer(1.0 / 5.0, self._on_5hz)       # 인식·텔레옵
        self.create_timer(1.0 / float(self.get_parameter('image_fps').value),
                          self._on_image)

        self.get_logger().info(
            f'가짜 발행자 시작 — 모터 {len(MOTORS)}개, 영상 {self.w}x{self.h}, '
            f'{self.fault_at:.0f}초 뒤 hw 에러 주입 (벤치 검증 전용)')

    # ------------------------------------------------------------ latched
    def _publish_latched_once(self):
        limits = Int32MultiArray()
        for dxl_id, _ in MOTORS:
            limits.data.extend([dxl_id, 100, 3900])
        self.pub_limits.publish(limits)

        self.pub_poses.publish(String(data='home,stow,pick_ready'))

        pick = DetectedObject()
        pick.class_id = 0
        pick.class_name = 'box'
        pick.confidence = 0.91
        pick.pose.position.x = 0.31
        pick.pose.position.y = -0.05
        pick.pose.position.z = 0.42
        pick.pose.orientation.w = 1.0
        pick.bbox.x_offset, pick.bbox.y_offset = 300, 200
        pick.bbox.width, pick.bbox.height = 120, 100
        self.pub_pick.publish(pick)

    # ------------------------------------------------------------ 30Hz
    def _on_30hz(self):
        self.tick += 1
        elapsed = time.monotonic() - self.t0
        msg = Int32MultiArray()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        for i, (dxl_id, name) in enumerate(MOTORS):
            phase = self.tick * 0.03 + i
            tick_pos = int(2048 + 800 * math.sin(phase))
            velocity = int(120 * math.cos(phase))
            current = int(120 + 90 * math.sin(phase * 0.7))
            # ID14 는 주기적으로 급상승시켜 트립 여유/급변 미터를 움직인다.
            if dxl_id == 14 and (self.tick // 90) % 3 == 2:
                current = int(430 + 40 * math.sin(phase * 3))
            # ⚠️ ID12 의 온도는 계속 0 — "미수신"으로 그려져야 한다(0°C 아님).
            temp = 0 if dxl_id == 12 else int(38 + 8 * math.sin(phase * 0.2) + i)
            msg.data.extend([dxl_id, tick_pos, velocity, current, temp])

            js.name.append(name)
            js.position.append((tick_pos - 2048) * (2.0 * math.pi / 4096.0))
            js.velocity.append(float(velocity))
            js.effort.append(float(current))

            if self.tick % 6 == 0:
                goal = Int32MultiArray()
                goal.data = [dxl_id, tick_pos + 40]
                self.pub_goal.publish(goal)

        self.pub_state.publish(msg)
        self.pub_js.publish(js)

        # hw 에러: 상승 엣지 한 번(트립 블랙박스 동결) → 잠시 뒤 하강 엣지.
        text = ''
        if self.fault_at > 0 and elapsed >= self.fault_at:
            if elapsed < self.fault_at + self.fault_clear:
                text = HW_ERROR_TEXT
                if not self.fault_raised:
                    self.fault_raised = True
                    self.get_logger().warn('가짜 hw 에러 주입 (상승 엣지)')
            elif not self.fault_cleared:
                self.fault_cleared = True
                self.get_logger().info('가짜 hw 에러 해제 (하강 엣지)')
        # 실제 노드도 에러가 없으면 빈 문자열을 30Hz 로 계속 발행한다.
        self.pub_hw.publish(String(data=text))
        self.pub_fault.publish(Bool(data=bool(text)))

    # ------------------------------------------------------------ 10Hz
    def _on_10hz(self):
        elapsed = time.monotonic() - self.t0
        arm = ArmStatus()
        arm.header.stamp = self.get_clock().now().to_msg()
        arm.mission_id = 7
        arm.status = 'EXECUTING' if int(elapsed) % 20 < 12 else 'STOWED_LOCKED'
        self.pub_arm.publish(arm)

        chassis = ChassisMode()
        chassis.header.stamp = arm.header.stamp
        chassis.mode = 'MISSION_STOP' if int(elapsed) % 20 < 12 else 'DRIVING'
        self.pub_chassis.publish(chassis)

    #: 리밋 측정 진행 시나리오. `teleop_core._publish_calib_status` 의 형식 그대로다.
    #: (하드웨어 없이 캘리브 패널의 진행률·완료·거절 표시를 확인하려고 한 바퀴 돈다.)
    CALIB_CYCLE = (
        'idle',
        'active,arm_joint_1,lower,1,4', 'active,arm_joint_1,upper,1,4',
        'active,arm_joint_2,lower,2,4', 'active,arm_joint_2,upper,2,4',
        'active,arm_joint_3,lower,3,4', 'active,arm_joint_3,upper,3,4',
        'active,arm_joint_4,lower,4,4', 'active,arm_joint_4,upper,4,4',
        'done,3,1',            # 거절 1축 — 화면이 "재측정 필요"를 띄워야 한다
        'cancelled',
    )

    def _publish_calib_cycle(self, elapsed):
        """8초마다 다음 단계로 — latched 라 GUI 를 나중에 띄워도 마지막 값을 받는다."""
        index = int(elapsed // 8) % len(self.CALIB_CYCLE)
        if index != getattr(self, '_calib_index', None):
            self._calib_index = index
            self.pub_calib.publish(String(data=self.CALIB_CYCLE[index]))

    # ------------------------------------------------------------ 5Hz
    def _on_5hz(self):
        elapsed = time.monotonic() - self.t0

        det = DetectedObjectArray()
        det.header.stamp = self.get_clock().now().to_msg()
        det.header.frame_id = 'camera_color_optical_frame'

        near = DetectedObject()
        near.class_id = 0
        near.class_name = 'box'
        near.confidence = 0.87
        near.pose.position.x = 0.31 + 0.02 * math.sin(elapsed)
        near.pose.position.y = -0.05
        near.pose.position.z = 0.42
        near.pose.orientation.w = 1.0
        near.bbox.x_offset, near.bbox.y_offset = 300, 200
        near.bbox.width, near.bbox.height = 120, 100
        det.objects.append(near)

        # ⚠️ z = 0.0 은 "원점"이 아니라 **깊이 없음** 규약이다.
        nodepth = DetectedObject()
        nodepth.class_id = 0
        nodepth.class_name = 'box'
        nodepth.confidence = 0.62
        nodepth.pose.orientation.w = 1.0
        nodepth.bbox.x_offset, nodepth.bbox.y_offset = 560, 120
        nodepth.bbox.width, nodepth.bbox.height = 90, 80
        det.objects.append(nodepth)
        self.pub_det.publish(det)

        jog = JointJog()
        jog.header.stamp = det.header.stamp
        # velocity 프론트엔드 규약: 매 발행마다 전 관절을 싣는다.
        jog.joint_names = [name for _, name in MOTORS[:5]]
        jog.velocities = [0.3 * math.sin(elapsed + i) for i in range(5)]
        self.pub_jog.publish(jog)

        self._publish_calib_cycle(elapsed)

        # 임계 설정 — 실기에서는 position_node 가 변경 시에만 낸다. 화면의 '트립까지
        # 남은 여유' 가 이 값 없이는 계산되지 않아 되풀이해 발행한다.
        # (값은 dynamixel_position_node 의 기본 트립/급변 임계와 같은 자리수다.)
        self.pub_trip_cfg.publish(Int32MultiArray(data=[1, 500]))
        self.pub_spike_cfg.publish(Int32MultiArray(data=[1, 350]))

        joy = Joy()
        joy.header.stamp = det.header.stamp
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 13
        # 데드맨(buttons[9])을 주기적으로 눌렀다 뗀다.
        joy.buttons[9] = 1 if int(elapsed) % 8 < 5 else 0
        self.pub_joy.publish(joy)

        if int(elapsed) % 17 == 0 and self.tick % 5 == 0:
            self.pub_cmd.publish(String(data='stop'))

        if int(elapsed) % 23 == 0 and self.tick % 5 == 0:
            arrival = ArrivalStatus()
            arrival.header.stamp = det.header.stamp
            arrival.mission_id = 7
            arrival.status = 'ARRIVED_PICKUP'
            self.pub_arrival.publish(arrival)

    # ------------------------------------------------------------ 영상
    def _on_image(self):
        # 구독자가 없으면 만들지 않는다 — perception_node 와 같은 게이트라
        # GUI 의 동적 구독이 실제로 발행을 멈추는지도 여기서 같이 검증된다.
        want_debug = self.pub_debug_img.get_subscription_count() > 0
        want_raw = self.pub_raw_img.get_subscription_count() > 0
        if not (want_debug or want_raw):
            return

        import numpy as np

        elapsed = time.monotonic() - self.t0
        frame = np.full((self.h, self.w, 3), 24, dtype=np.uint8)
        # 움직이는 세로 막대 — 프레임이 실제로 갱신되는지 눈으로 확인용.
        x = int((elapsed * 160) % max(1, self.w - 60))
        frame[:, x:x + 60] = (90, 90, 90)
        # pick 타겟(초록) / 나머지(파랑) — perception_node 의 색 규약과 같다.
        frame[200:300, 300:420] = (60, 200, 80)
        frame[120:200, 560:650] = (240, 140, 60)

        if want_debug:
            self.pub_debug_img.publish(self._to_msg(frame, 'debug'))
        if want_raw:
            self.pub_raw_img.publish(self._to_msg(frame[:, ::-1].copy(), 'raw'))

    def _to_msg(self, frame, tag):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'fake_{tag}'
        msg.height, msg.width = frame.shape[0], frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FakePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
