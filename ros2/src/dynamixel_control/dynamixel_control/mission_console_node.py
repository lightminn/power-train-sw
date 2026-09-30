#!/usr/bin/env python3
"""벤치 운영자 콘솔 — 파워트레인 없이 사람이 직접 팔에 미션을 지시한다.

## 왜 필요한가

`arm_fsm` 은 계약상 **`/chassis_mode`(MISSION_STOP) + `/arrival_status` 두 개가 모두
성립할 때만** 움직인다(`_try_advance()` 의 conjunction). 실전에서는 파워트레인이 그
둘을 발행하지만, 벤치에는 파워트레인이 없다 — 그래서 팔만 놓고 "비전 → 픽"을
시켜보려면 그 대역을 대신 서줄 무언가가 필요하다. 이 노드가 그것이다.

## ⚠️ 벤치 전용 — production 금지

이 노드는 **계약이 파워트레인을 owner 로 지정한 토픽**(`/chassis_mode`,
`/arrival_status`)을 발행한다. 실전 launch 에 절대 넣지 말 것. owner 가 둘이 되면
파워트레인의 상태와 이 콘솔의 상태가 어긋나며, 파워트레인은 `header.stamp` 가
0.5초 이상 역행하면 **프로세스 재시작 전까지 안 풀리는 영구 latch** 를 건다.

그래서 기동 시 `/chassis_mode` 에 **다른 발행자가 이미 있으면 거부**한다. 이건
"테스트 모드면 건너뛰기" 분기가 아니라 그 반대다 — 실전 구성에서는 아예 못 뜬다.

## 왜 `ros2 topic pub` 로는 안 되는가

`arm_fsm._stamp_is_fresh()` 는 `header.stamp` 의 **단조 증가**를 요구하는데 CLI 는
stamp 를 0 으로 고정해서 보낸다 → 첫 샘플만 통과하고 두 번째부터 전부 버려진다.
그 뒤 `chassis_mode_timeout`(기본 1.0초)이 지나면 FSM 이 락을 건다. 겉보기엔
"명령을 계속 보내는데 팔이 안 움직인다" 로만 보여서 원인을 찾기 어렵다.
그래서 이 노드는 매 발행마다 현재 시각을 찍고 10Hz 로 heartbeat 를 유지한다.

## 명령

    pick    새 미션 시작 (mission_id 자동 증가) → ARRIVED_PICKUP
            FSM: STOWED_LOCKED → PERCEIVE → PLAN → APPROACH → DESCEND → GRASP → LIFT → CARRY
    drop    현재 미션 하역 → ARRIVED_DROP  (CARRY 상태에서만 의미 있다)
    stow    STOW_REQUEST — 진행 중이던 작업을 접고 잠근다. 화물을 든 채면
            그리퍼를 바로 열지 않고 **파지 높이까지 내린 뒤** 놓는다(LOWER_RELEASE).
    drive   DRIVING — 계약상 팔 잠금(주행 모드). 언락은 pick 으로만 된다.
    target  /pick_target 을 base_link 로 변환해 **팔 평면과의 방위각 차**를 보여준다.
            픽 전에 이걸 먼저 볼 것 — 아래 "왜 방위각인가" 참고.
    torque off / torque on
            전 축 토크 해제 / 인가. 해제하면 손으로 자세를 잡을 수 있다(팔이 처진다).
    status  현재 mode / arm_status / 마지막 미션
    q       종료. **토크를 풀고**(release_torque_on_exit, 기본 true) 나간다 —
            콘솔을 닫는 건 '사람이 팔을 만지겠다'는 뜻일 때가 대부분이라 그쪽을
            기본값으로 잡았다. heartbeat 도 끊기므로 FSM 은 스스로 락을 건다.

## 왜 방위각인가

`arm_joint_1`(베이스 요축)에 모터가 없어 이 팔은 **평면 로봇**이다. 좌우로 못
비키므로 타겟이 팔 평면에서 벗어난 각도는 IK 가 흡수하지 못하고 그대로 파지
오차가 된다. 그래서 `target` 은 타겟 방위각과 **현재 tip 방위각**을 같이 재서 그
차이를 보여준다(절대 방위각은 팔 장착 방향에 따라 달라져 의미가 없다 — 실제로
tip 방위각이 -80.7° 인 배치다). 차이가 크면 캘리브가 아니라 **박스를 옮기는 게**
답이다. 둘을 혼동하면 캘리브를 아무리 다시 해도 안 맞는다.
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node

import tf2_ros

from std_msgs.msg import Int32MultiArray

from robot_arm_msgs.msg import ArmStatus, ArrivalStatus, ChassisMode, DetectedObject

from . import contract
from . import joint_limits
from .qos_profiles import ARRIVAL_QOS, HEARTBEAT_QOS

#: chassis_mode heartbeat 주기. FSM 의 chassis_mode_timeout(1.0s)보다 충분히 빨라야
#: 한 샘플 유실이 곧바로 락으로 이어지지 않는다.
HEARTBEAT_HZ = 10.0

#: 기동 시 다른 발행자(=진짜 파워트레인) 탐지에 줄 시간. DDS discovery 는 즉시가 아니다.
DISCOVERY_WAIT_S = 2.0

PROMPT = (
    "\n명령 (pick/drop/stow/drive/target/torque off|on/status/q) > "
)


class MissionConsole(Node):
    def __init__(self):
        super().__init__('bench_mission_console')

        # arm_fsm 의 같은 이름 파라미터와 맞춰 둘 것 — 어긋나면 방위각 판정이 통째로 틀린다.
        self.declare_parameter('pick_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('tip_link', 'link_043')
        self.pick_frame_id = str(self.get_parameter('pick_frame_id').value)
        self.tip_link = str(self.get_parameter('tip_link').value)

        self.mode = contract.MODE_MISSION_STOP
        self.mission_id = 0
        self.arm_status = None
        self.pick_target = None

        # 종료할 때 팔을 손으로 만질 수 있게 토크를 풀지 여부. 브릿지/position_node 가
        # 공유하는 `/dynamixel/torque_request` 를 그대로 쓴다(어휘를 새로 만들지 않는다).
        # ⚠️ 기본값을 **false 로 되돌렸다**(2026-08-12). 원래 true 였는데, 종료할 때마다
        # 팔이 중력으로 처지고 그 자리가 `arm_joint_2` 하한(0.0) **아래**라 다음 pick 이
        # APPROACH 의 analytic IK 에서 막혔다("목표 도달 불가"). 목표가 멀어서가 아니라
        # 출발 자세가 리밋 밖이라 야코비안 반복이 clamp 에 걸리는 것이고, 실기에서
        # 세 번 연속 같은 증상으로 시간을 잃었다. 토크 해제는 필요할 때 `torque off`
        # 로 명시적으로 하는 게 맞다 — 종료의 부수효과로 두면 안 된다.
        self.declare_parameter('release_torque_on_exit', False)
        self.release_torque_on_exit = bool(
            self.get_parameter('release_torque_on_exit').value)
        # ⚠️ 종료 시 푸는데 시작 시 안 걸면 **비대칭**이라, 콘솔을 다시 띄운 다음
        # pick 을 눌러도 팔이 조용히 안 움직인다(2026-08-12 실기에서 그대로 당했다 —
        # FSM 은 전 구간을 수행하고 브릿지도 goal 을 쓰는데 서보만 무시한다).
        # "콘솔이 떠 있다 = 팔에 힘이 들어와 있다" 로 짝을 맞춘다.
        self.declare_parameter('acquire_torque_on_start', True)
        self.acquire_torque_on_start = bool(
            self.get_parameter('acquire_torque_on_start').value)

        self.mode_pub = self.create_publisher(
            ChassisMode, '/chassis_mode', HEARTBEAT_QOS)
        self.torque_pub = self.create_publisher(
            Int32MultiArray, '/dynamixel/torque_request', 10)
        self.arrival_pub = self.create_publisher(
            ArrivalStatus, '/arrival_status', ARRIVAL_QOS)

        self.create_subscription(
            ArmStatus, '/arm_status', self._on_arm_status, HEARTBEAT_QOS)
        self.create_subscription(
            DetectedObject, '/pick_target', self._on_pick_target,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / HEARTBEAT_HZ, self._beat)

    # ── 발행 ────────────────────────────────────────────────
    def _beat(self):
        msg = ChassisMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = self.mode
        self.mode_pub.publish(msg)

    def send_torque(self, enable):
        """`[enable]` — id 를 안 실으면 브릿지가 등록된 전 축에 적용한다."""
        msg = Int32MultiArray()
        msg.data = [1 if enable else 0]
        self.torque_pub.publish(msg)

    def send_arrival(self, status, mission_id):
        msg = ArrivalStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = mission_id
        msg.status = status
        self.arrival_pub.publish(msg)

    # ── 구독 ────────────────────────────────────────────────
    def _on_arm_status(self, msg):
        self.arm_status = msg.status

    def _on_pick_target(self, msg):
        self.pick_target = msg

    # ── 조회 ────────────────────────────────────────────────
    def target_report(self):
        """타겟 방위각과 tip 방위각의 차 — 픽 성공 가능성의 사전 지표."""
        if self.pick_target is None:
            return '아직 /pick_target 을 못 받았습니다 (perception_node 와 박스 확인).'

        # ⚠️ DetectedObject 에는 header 가 없다 — header 는 DetectedObjectArray 쪽에만
        # 있고, /pick_target 은 단일 객체라 프레임을 스스로 말해주지 않는다.
        # arm_fsm 도 같은 이유로 `pick_frame_id` 파라미터로 프레임을 받는다.
        frame = self.pick_frame_id
        # /tf_static 은 latched 지만 구독 직후엔 아직 버퍼에 안 들어와 있을 수 있다.
        # 콘솔을 띄우자마자 target 을 치면 매번 "TF 없음" 이 나와서 캘리브가 깨진 줄
        # 오해하게 된다 — 잠깐 기다렸다 다시 본다(진짜 없으면 그때 안내한다).
        tf = None
        deadline = time.time() + 3.0
        while tf is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', frame, rclpy.time.Time())
            except Exception as exc:                   # noqa: BLE001
                if time.time() >= deadline:
                    return (f'TF 없음 ({frame} → base_link): {exc} — '
                            'camera_tf.launch.py 가 떠 있는지 확인하세요.')
                time.sleep(0.2)

        p = self.pick_target.pose.position
        tx, ty, tz = _apply_tf(tf, (p.x, p.y, p.z))
        t_az = math.degrees(math.atan2(ty, tx))
        t_r = math.hypot(tx, ty)

        try:
            tip = self.tf_buffer.lookup_transform(
                'base_link', self.tip_link, rclpy.time.Time()).transform.translation
            tip_az = math.degrees(math.atan2(tip.y, tip.x))
            diff = (t_az - tip_az + 180.0) % 360.0 - 180.0
            verdict = ('양호 — 진행해도 됩니다' if abs(diff) <= 5.0
                       else '⚠️ 팔 평면에서 벗어났습니다 — 박스를 옮기는 게 캘리브보다 빠릅니다')
            tip_line = (f'  tip 방위각    : {tip_az:+7.1f}°\n'
                        f'  차이          : {diff:+7.1f}°   {verdict}\n')
        except Exception:                              # noqa: BLE001
            tip_line = '  tip 방위각    : TF 없음 (브릿지/robot_state_publisher 확인)\n'

        return (
            f'  클래스        : {self.pick_target.class_name} '
            f'(conf {self.pick_target.confidence:.3f})\n'
            f'  base_link 좌표: ({tx:+.4f}, {ty:+.4f}, {tz:+.4f}) m\n'
            f'  방위각/반경   : {t_az:+7.1f}° / {t_r:.3f} m\n'
            + tip_line
            + self._reach_line((tx, ty, tz))
        )

    @staticmethod
    def _reach_line(xyz):
        """목표가 top-down 도달 범위 안인지 — pick 을 누르기 **전에** 걸러준다.

        ⚠️ 왜 있는가 (2026-08-19 실기): pick 을 눌렀는데 팔이 전혀 안 움직였다. 원인은
        목표가 도달 범위 밖(y 로 10cm 초과)이라 APPROACH 의 IK 가 실패한 것이었는데,
        그 이유가 `arm_fsm` 로그 → launch 로그 **파일**에만 남아서(run_pick.sh 가 스택
        출력을 /tmp 로 돌린다) 콘솔에는 arm_status=FAILED 밖에 안 보였다. 즉 사용자
        입장에서는 "그냥 안 움직인다" 였다. 여기서 미리 알려주면 팔을 돌리기 전에 안다.

        판정 근거는 `joint_limits.REACH_TOPDOWN` (관절 리밋 + URDF FK 산출물, 단일 출처).
        경계상자는 **필요조건**이라 "밖 = 확실히 불가" 만 단정하고, 안쪽은 IK 에 맡긴다.
        """
        breaches = joint_limits.outside_topdown_reach(xyz)
        if not breaches:
            return ('  도달 판정     : 범위 안 — 시도해도 좋습니다 '
                    '(최종 판정은 IK 가 합니다)\n')
        detail = ', '.join(
            f'{ax}={val:+.3f} 이 [{lo:+.3f}, {hi:+.3f}] 밖 ({over * 100:.1f}cm 초과)'
            for ax, val, lo, hi, over in breaches)
        return ('  도달 판정     : ❌ **도달 불가** — ' + detail + '\n'
                '                  pick 을 눌러도 IK 가 실패해 팔이 안 움직입니다. '
                '박스를 팔 쪽으로 당기거나 차체를 붙이세요.\n'
                '                  (팔은 base_link -Y 로만 뻗습니다 — '
                'arm_joint_1 이 ±14.3° 뿐이라 방위 회전으로 거리를 못 법니다)\n')

    def status_report(self):
        return (
            f'  chassis_mode  : {self.mode}\n'
            f'  arm_status    : {self.arm_status or "(아직 못 받음 — arm_fsm 확인)"}\n'
            f'  mission_id    : {self.mission_id if self.mission_id else "(아직 없음)"}\n'
        )


def _apply_tf(tf, xyz):
    """geometry_msgs/TransformStamped 를 점 하나에 적용 (tf2_geometry_msgs 불필요)."""
    q = tf.transform.rotation
    t = tf.transform.translation
    x, y, z = xyz
    # 쿼터니언 회전 (v' = q v q*) 전개
    xx, yy, zz, ww = q.x, q.y, q.z, q.w
    tx = 2.0 * (yy * z - zz * y)
    ty = 2.0 * (zz * x - xx * z)
    tz = 2.0 * (xx * y - yy * x)
    rx = x + ww * tx + (yy * tz - zz * ty)
    ry = y + ww * ty + (zz * tx - xx * tz)
    rz = z + ww * tz + (xx * ty - yy * tx)
    return rx + t.x, ry + t.y, rz + t.z


def _guard_single_owner(node):
    """진짜 파워트레인이 이미 /chassis_mode 를 밀고 있으면 뜨지 않는다."""
    end = node.get_clock().now().nanoseconds * 1e-9 + DISCOVERY_WAIT_S
    while node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    others = node.count_publishers('/chassis_mode') - 1   # 자기 자신 제외
    if others > 0:
        node.get_logger().error(
            f'/chassis_mode 에 다른 발행자가 {others}개 있습니다 — 파워트레인이 '
            '이미 떠 있는 구성으로 보입니다. 이 콘솔은 벤치 전용이며 owner 가 '
            '둘이 되면 파워트레인이 영구 latch 를 겁니다. 종료합니다.')
        return False
    return True


def main():
    rclpy.init()
    node = MissionConsole()

    if not _guard_single_owner(node):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(__doc__.split('## 명령')[1].split('## 왜 방위각')[0].rstrip())
    print(f'\nchassis_mode={node.mode} 를 {HEARTBEAT_HZ:.0f}Hz 로 발행 중입니다.')
    if node.acquire_torque_on_start:
        # 브릿지가 토크를 켜기 전에 goal 을 현재 위치로 맞추므로 팔이 튀지 않는다.
        node.send_torque(True)
        print('· 전 축 토크 인가 — 팔이 현재 자세를 유지합니다.')
    print('⚠️ pick 을 누르면 팔이 실제로 움직입니다 — 주변을 확인하세요.')

    try:
        while rclpy.ok():
            try:
                cmd = input(PROMPT).strip().lower()
            except EOFError:
                break

            if cmd in ('q', 'quit', 'exit'):
                break

            if cmd == 'pick':
                node.mode = contract.MODE_MISSION_STOP
                node.mission_id += 1
                node.send_arrival(contract.ARRIVED_PICKUP, node.mission_id)
                print(f'  → ARRIVED_PICKUP (mission_id={node.mission_id}) 발행. '
                      '팔이 움직입니다.')
            elif cmd == 'drop':
                if not node.mission_id:
                    print('  아직 시작한 미션이 없습니다 — 먼저 pick.')
                    continue
                node.send_arrival(contract.ARRIVED_DROP, node.mission_id)
                print(f'  → ARRIVED_DROP (mission_id={node.mission_id}) 발행.')
            elif cmd == 'stow':
                node.mode = contract.MODE_STOW_REQUEST
                print('  → chassis_mode=STOW_REQUEST. 접고 잠급니다 '
                      '(화물을 들고 있으면 내린 뒤 놓습니다).')
            elif cmd == 'drive':
                node.mode = contract.MODE_DRIVING
                print('  → chassis_mode=DRIVING. 팔 잠금 — 언락은 pick 뿐입니다.')
            elif cmd == 'target':
                print(node.target_report())
            elif cmd in ('torque off', 'torque_off', 'free'):
                node.send_torque(False)
                print('  → 전 축 토크 해제. ⚠️ 팔이 중력으로 처집니다 — 받치세요.')
            elif cmd in ('torque on', 'torque_on', 'hold'):
                node.send_torque(True)
                print('  → 전 축 토크 인가(현재 자세 홀드).')
            elif cmd == 'status':
                print(node.status_report())
            elif cmd:
                print(f'  모르는 명령: {cmd!r}')
    except KeyboardInterrupt:
        pass
    finally:
        print('\n종료합니다.')
        if node.release_torque_on_exit:
            # 토크 해제 → 조종권을 사람에게 넘긴다. 발행 직후 바로 죽으면 DDS 가 아직
            # 안 보냈을 수 있어 잠깐 기다린다.
            # ⚠️ 여기서 spin_once 를 부르면 안 된다 — 백그라운드 스레드가 이미 같은
            #    노드를 spin 하고 있어 이중 spin 이 되고, 종료 시 C++ 쪽에서
            #    `terminate called without an active exception` 으로 죽는다(실측).
            #    발행은 그 스레드가 알아서 내보내므로 sleep 만으로 충분하다.
            node.send_torque(False)
            time.sleep(0.5)
            print('  · 전 축 토크 해제 — 손으로 자세를 잡을 수 있습니다. '
                  '⚠️ 팔이 처지니 받치세요.')
            print('  · 다시 잡으려면: 콘솔을 띄우고 "torque on", 또는 스택 재기동.')
        print('  · chassis_mode heartbeat 중단 — arm_fsm 이 스스로 락을 겁니다.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
