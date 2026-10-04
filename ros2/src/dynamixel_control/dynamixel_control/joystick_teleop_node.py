#!/usr/bin/env python3
"""조이스틱(DualSense) 원격조종 프론트엔드 — 5축 직접 매핑.

`/joy`(sensor_msgs/Joy)를 표준 control_msgs/JointJog(+ 이산 명령 String)로 변환해
teleop_core 로 보낸다. 시리얼/모터 로직은 전혀 없다 — 순수 입력 어댑터.
키보드 프론트엔드(keyboard_teleop_node)와 같은 토픽을 쓰므로 서로 대체 가능하다.

키맵 (2026-07-29 DualSense 블루투스 실측 — 축·버튼 인덱스는 전부 파라미터)
  buttons[9] (hold)  데드맨. 누르고 있는 동안만 움직인다. 떼면 전 축 즉시 0.
  왼스틱  ↔ (axes 0)  arm_joint_1
  왼스틱  ↕ (axes 1)  arm_joint_2
  오른스틱 ↕ (axes 3) arm_joint_3
  오른스틱 ↔ (axes 2) arm_joint_4
  buttons[2] / [0]    arm_joint_5        + / −
  buttons[3] / [1]    gripper_left_pinion_joint 열기 / 닫기

  이산 명령(home/stop/estop/mode)은 기본 비활성(-1) — 위 버튼들과 겹친다.
  터보도 비활성.

⚠️ 실물 패드의 축·버튼 인덱스는 커널 드라이버(hid-sony vs hid-playstation)에 따라
   다르다. 위 값은 이 패드에서 직접 재서 확정한 것이고, joy 문서의 표준 배치와
   다르다(표준은 axes 2=L2, 3=오른스틱X, 4=오른스틱Y). 패드가 바뀌면
   `ros2 topic echo /joy` 로 다시 확인해 **파라미터만** 바꾸면 된다.

⚠️ L2/R2 축(4·5)은 **안 누른 상태에서 +1.0** 을 낸다(첫 조작 전에만 0.0).
   관절에 물리면 아무도 안 만졌는데 풀속으로 돈다 — 쓰려면 오프셋 보정이 필요하다.

⚠️ 벤치 전용 — 이 경로(teleop_core → /dynamixel/goal_position → position_node)는
   파워트레인 계약상 "direct dynamixel goal publisher" 라 production 금지다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from control_msgs.msg import JointJog


DEFAULT_JOINT_NAMES = [
    'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5',
    'gripper_left_pinion_joint',
]

#: 관절별 축 인덱스. -1 = 축 없음(버튼으로 조작).
#: 실측(2026-07-29, DualSense BT + joy 3.3): 0=왼스틱X, 1=왼스틱Y,
#: 2=오른스틱X, 3=오른스틱Y, 4=L2, 5=R2 — joy 문서의 표준 배치와 다르다.
#: (표준은 2=L2, 3=오른스틱X, 4=오른스틱Y 다. 이 패드는 legacy joydev 배치.)
DEFAULT_AXIS_IDS = [0, 1, 3, 2, -1, -1]

#: 풀 스틱일 때의 속도 [rad/s]. teleop_core 가 velocities 를 rad/s 로 해석한다.
DEFAULT_AXIS_SCALES = [0.6, 0.4, 0.4, 0.6, 0.8, 0.5]

#: 축 방향 반전. 스틱을 위로 밀었을 때 관절이 의도한 방향으로 가도록.
DEFAULT_AXIS_INVERTED = [False, True, False, False, False, False]

#: 축이 없는 관절(-1)을 굴릴 버튼. 관절별로 따로 둔다 — 예전엔 joint5_plus/minus
#: 하나를 축 없는 **모든** 관절이 공유해서, 손목과 그리퍼가 같이 움직였다.
#: -1 = 버튼 없음.
#:
#: 그리퍼를 L2/R2 **축**(4·5)에 물리지 않은 이유: 이 패드의 트리거 축은 안 누른
#: 상태에서 **+1.0** 을 낸다(첫 조작 전에만 0.0). 축으로 쓰면 아무도 안 만졌는데
#: 그리퍼가 풀속으로 돈다.
DEFAULT_BUTTON_PLUS_IDS = [-1, -1, -1, -1, 2, 3]
DEFAULT_BUTTON_MINUS_IDS = [-1, -1, -1, -1, 0, 1]


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        # ── 관절 매핑 ─────────────────────────────
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('axis_ids', DEFAULT_AXIS_IDS)
        self.declare_parameter('axis_scales', DEFAULT_AXIS_SCALES)
        self.declare_parameter('axis_inverted', DEFAULT_AXIS_INVERTED)
        self.declare_parameter('deadzone', 0.15)

        # ── 버튼 ──────────────────────────────────
        self.declare_parameter('deadman_button', 9)      # 실측 확정 — 누르고 있어야 움직임
        self.declare_parameter('turbo_button', -1)       # 비활성
        self.declare_parameter('turbo_scale', 2.5)
        self.declare_parameter('button_plus_ids', DEFAULT_BUTTON_PLUS_IDS)
        self.declare_parameter('button_minus_ids', DEFAULT_BUTTON_MINUS_IDS)
        # 이산 명령은 전부 비활성(-1)이 기본이다. 버튼 0·1·2·3 을 joint_5/그리퍼에
        # 배정했더니 예전 기본값(✕=estop, ○=stop, △=home)과 정면으로 겹쳤다 —
        # joint_5 를 내리려다 비상정지가 걸리는 식. 안전은 **데드맨**이 맡는다
        # (놓으면 전 축 즉시 0). 쓰려면 안 겹치는 인덱스를 파라미터로 지정할 것.
        self.declare_parameter('home_button', -1)        # 벤치 전용(계약상 production 금지)
        self.declare_parameter('stop_button', -1)        # E-stop 해제도 겸함
        self.declare_parameter('estop_button', -1)       # latched
        self.declare_parameter('mode_button', -1)        # DRIVE/ARM 전환(스텁)

        # ── 그리퍼 (이번 범위 제외 — 자리만 예약) ──
        # 그리퍼는 moveit_dynamixel_bridge 가 이미 같은 서보(랙피니언 2모터 id 3,4)를 구동한다.
        # 여기서 또 구동하면 같은 버스의 같은 서보를 두 노드가 만지게 된다(owner 중복).
        # 배선하려면 그리퍼 경로를 한 곳으로 먼저 정리할 것.
        # ⚠️ 그때 반드시: Profile Acceleration(108)/Velocity(112) 를 25/80 으로 설정할 것.
        #    기본값 0(=최고속 즉시 이동)이면 움직일 때마다 과전류로 토크가 풀린다
        #    (HW-8 실기 검증, 재현율 100%, 명령 후 0.3초 내 트립).
        self.declare_parameter('gripper_close_axis', -1)   # L2 = 2 (비활성)
        self.declare_parameter('gripper_open_axis', -1)    # R2 = 5 (비활성)

        # ── 타이밍 ────────────────────────────────
        self.declare_parameter('publish_rate_hz', 50.0)
        # /joy 가 이 시간 넘게 안 오면 패드 사망으로 보고 전 축 0 → 폭주 방지.
        self.declare_parameter('joy_timeout_s', 0.5)

        g = self.get_parameter
        self.joint_names = list(g('joint_names').value)
        self.axis_ids = [int(v) for v in g('axis_ids').value]
        self.axis_scales = [float(v) for v in g('axis_scales').value]
        self.axis_inverted = [bool(v) for v in g('axis_inverted').value]
        self.deadzone = float(g('deadzone').value)

        self.btn_plus = [int(v) for v in g('button_plus_ids').value]
        self.btn_minus = [int(v) for v in g('button_minus_ids').value]

        n = len(self.joint_names)
        if not (len(self.axis_ids) == len(self.axis_scales) == len(self.axis_inverted)
                == len(self.btn_plus) == len(self.btn_minus) == n):
            raise RuntimeError(
                'joint_names/axis_ids/axis_scales/axis_inverted/'
                'button_plus_ids/button_minus_ids 길이가 서로 다릅니다')

        self.deadman_button = int(g('deadman_button').value)
        self.turbo_button = int(g('turbo_button').value)
        self.turbo_scale = float(g('turbo_scale').value)
        self.home_button = int(g('home_button').value)
        self.stop_button = int(g('stop_button').value)
        self.estop_button = int(g('estop_button').value)
        self.mode_button = int(g('mode_button').value)

        self.rate = float(g('publish_rate_hz').value)
        self.joy_timeout_s = float(g('joy_timeout_s').value)

        # ── 상태 ──────────────────────────────────
        self._joy = None                 # 최신 Joy 메시지
        self._joy_t = None               # 최신 Joy 수신 시각
        self._prev_buttons = []          # edge-trigger 용
        self._estop = False              # 비상정지 latch
        self._stale_warned = False
        self._zeroed = False             # 정지 상태에서 0 을 이미 보냈는지

        self.sub_joy = self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.pub_jog = self.create_publisher(JointJog, '/arm/teleop_jog', 10)
        self.pub_cmd = self.create_publisher(String, '/arm/teleop_cmd', 10)

        # /joy 콜백이 아니라 타이머에서 발행한다.
        # 스틱을 가만히 붙들고 있으면 /joy 이벤트가 안 올 수 있는데, 그러면
        # teleop_core 의 deadman(0.5초)이 걸려 팔이 서 버린다.
        self.create_timer(1.0 / self.rate, self._on_timer)

        self.get_logger().info(
            f'joystick_teleop started (joints={self.joint_names}, '
            f'axes={self.axis_ids}, deadman=button[{self.deadman_button}], '
            f'rate={self.rate}Hz)')
        self.get_logger().info(
            f'buttons[{self.deadman_button}] 을 누르고 있어야 팔이 움직입니다 '
            f'(놓으면 전 축 즉시 0).')

    # ------------------------------------------------------------------ 입력
    def _on_joy(self, msg):
        self._joy = msg
        self._joy_t = self.get_clock().now()
        self._stale_warned = False

    def _btn(self, idx):
        """버튼이 현재 눌려 있나."""
        if self._joy is None or idx < 0 or idx >= len(self._joy.buttons):
            return False
        return bool(self._joy.buttons[idx])

    def _btn_pressed(self, idx):
        """버튼이 이번에 새로 눌렸나 (edge-trigger)."""
        if idx < 0:
            return False
        now = self._btn(idx)
        was = self._prev_buttons[idx] if idx < len(self._prev_buttons) else False
        return now and not was

    def _axis(self, idx):
        """데드존을 적용한 축 값. 데드존 밖은 [0,1] 로 다시 편다(경계에서 튀지 않게)."""
        if self._joy is None or idx < 0 or idx >= len(self._joy.axes):
            return 0.0
        v = float(self._joy.axes[idx])
        if abs(v) <= self.deadzone:
            return 0.0
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    # ------------------------------------------------------------------ 발행
    def _publish_velocities(self, velocities):
        """전 관절을 매번 싣는다 — 일부만 보내면 놓은 관절이 마지막 속도로 계속 돈다."""
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.joint_names)
        msg.velocities = [float(v) for v in velocities]
        self.pub_jog.publish(msg)

    def _publish_zero(self):
        self._publish_velocities([0.0] * len(self.joint_names))

    def _cmd(self, cmd):
        self.pub_cmd.publish(String(data=cmd))

    # ------------------------------------------------------------------ 주기
    def _on_timer(self):
        # 1) 패드가 죽었나 — 마지막 축 값을 계속 재발행하면 팔이 폭주한다.
        if self._joy is None or self._joy_t is None:
            return
        age = (self.get_clock().now() - self._joy_t).nanoseconds * 1e-9
        if age > self.joy_timeout_s:
            if not self._stale_warned:
                self.get_logger().warn(
                    f'/joy 끊김 ({age:.1f}s) — 전 축 정지. 패드 연결을 확인하세요.')
                self._stale_warned = True
                self._publish_zero()     # 마지막으로 0 을 한 번 보내고 발행 중단
            return

        buttons = list(self._joy.buttons)

        # 2) 이산 명령 (edge-trigger)
        if self._btn_pressed(self.estop_button):
            self._estop = True
            self._publish_zero()
            self._cmd('stop')
            self.get_logger().error('✕ 비상정지 — ○ 를 눌러 해제하세요')
        if self._btn_pressed(self.stop_button):
            if self._estop:
                self._estop = False
                self.get_logger().warn('비상정지 해제')
            self._cmd('stop')
        if self._btn_pressed(self.home_button) and not self._estop:
            self._cmd('home')
        if self._btn_pressed(self.mode_button):
            # TODO: DRIVE/ARM 전환. 파워트레인 계약상 전환은 wheel-stop·MISSION_STOP·
            #       stow-before-drive 등 선결 조건이 있다. 지금은 스텁.
            self.get_logger().info('PS: DRIVE/ARM 전환 요청 — 미구현(스텁)')

        self._prev_buttons = buttons

        # 3) 데드맨 / 비상정지 — 둘 중 하나라도 걸리면 전 축 0
        if self._estop or not self._btn(self.deadman_button):
            if not self._zeroed:
                self._publish_zero()
                self._zeroed = True
            return
        self._zeroed = False

        # 4) 축 → 관절 속도
        scale = self.turbo_scale if self._btn(self.turbo_button) else 1.0
        vels = []
        for i, _name in enumerate(self.joint_names):
            axis = self.axis_ids[i]
            if axis >= 0:
                v = self._axis(axis) * self.axis_scales[i]
                if self.axis_inverted[i]:
                    v = -v
            else:
                # 축이 없는 관절은 관절별 +/- 버튼으로. 누르는 동안 등속.
                v = 0.0
                if self._btn(self.btn_plus[i]):
                    v += self.axis_scales[i]
                if self._btn(self.btn_minus[i]):
                    v -= self.axis_scales[i]
            vels.append(v * scale)

        self._publish_velocities(vels)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
