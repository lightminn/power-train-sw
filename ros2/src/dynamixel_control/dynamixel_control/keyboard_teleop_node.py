#!/usr/bin/env python3
"""키보드 원격조종 프론트엔드 — curses TUI 콘솔.

터미널 키 입력을 표준 control_msgs/JointJog(+ 이산 명령 String)로 변환해
teleop_core 로 보낸다. 시리얼/모터 로직은 전혀 없다 — 순수 입력 어댑터.
게임패드/네트워크 프론트엔드도 같은 토픽으로 발행하면 그대로 대체된다.

화면에 매 프레임 다음을 보여준다:
  - 마지막으로 누른 키 / 그로 인한 동작
  - 관절별 현재 각도(/joint_states 구독, deg+rad)와 수신 최신성
  - 현재 선택된 관절(반전 표시) · 스텝 스케일

키맵
  1 2 3 4 5   : 관절 선택 (arm_joint_1..5)
  6           : 그리퍼 선택 (gripper_left_pinion_joint)
  ↑ / w       : 선택 관절 + 방향 jog (누르고 있는 동안 계속, 떼면 즉시 정지)
  ↓ / s       : 선택 관절 - 방향 jog (누르고 있는 동안 계속, 떼면 즉시 정지)
  [ / ]       : jog 속도 감소 / 증가 (rad/s)
  h           : 저장된 자세 "home" 으로 이동 (= g 로 "home" 입력하는 것과 동일)
  p           : 팔 관절 토크 OFF(freedrive) → 손으로 자세 잡기 → 슬롯(1-9)에 저장(그리퍼 제외,
                저장과 동시에 그 자세 그대로 토크 복귀) — 숫자 한 키, 그 외 키=이름 직접입력,
                ESC 로 취소하면 저장 없이 토크만 복귀
  g           : 슬롯(1-9)에 저장된 자세로 이동(그리퍼 제외, 그리퍼는 그대로 유지) — 숫자 한 키, 그 외 키=이름 직접입력
  r           : 선택된 관절의 서보 reboot (과부하 등으로 트립됐을 때 복구)
  R           : 현재 하드웨어 에러가 뜬 서보 전체 reboot
  c           : 리미트 측정 모드 시작(팔 관절 토크 OFF, 축 순서대로 하한→상한 측정)
  m           : (리미트 측정 중) 현재 각도를 현재 축의 현재 단계 값으로 기록, 다음 단계로
  x           : (리미트 측정 중) 측정 취소 — 기록값 폐기하고 토크 복귀
  e           : 전류 급변 감지(비상정지) on/off 토글(2026-08-02 추가) — 조그 시작 시
                가속 전류로 오탐 나면 잠깐 꺼두고 테스트할 때 씀. threshold 는 키보드로는
                못 바꾸고 teleop_core 에 "spike <숫자>" 명령으로만(예: 별도 도구로 발행)
  l           : 관절 리밋(joint_limits) on/off 토글(2026-08-02 추가) — off 는 소프트
                clamp 와 position_node 의 tick_limits 방어선을 동시에 푼다. 다회전 축
                (arm_joint_2/3)은 tick_limits 없인 안 움직여 off 중에도 여전히 안전.
  y           : 절대 과전류 트립 on/off 토글(2026-08-02 추가) — 그리퍼 정상 파지 중에도
                트립되는 게 실기로 재현돼 추가. threshold 는 e 와 마찬가지로 teleop_core
                에 "trip <숫자>" 명령으로만 바꿀 수 있다.
  space       : **즉시** 정지 + 전 관절(그리퍼 포함) 토크 즉시 차단(지연 없음, E-stop) —
                팔이 힘을 잃고 처질 수 있음. 다시 움직이려면 t 로 재개.
  t           : space 로 끊은 토크를 그 자리에서(현재 위치 홀드) 복귀
  q / Ctrl-C  : 종료

자세 저장/이동 — 슬롯 (2026-08-01 추가, 같은 날 이름 프롬프트 → 슬롯으로 변경,
2026-08-02 arm_joint_5 제외 추가)
  p 를 누르고 숫자 1-9 중 하나를 누르면(Enter 불필요) 팔 관절(arm_joint_1..4,
  그리퍼+arm_joint_5 제외 — arm_joint_5 는 자세 저장/불러오기 오작동 보고로 제외)
  현재 측정 각도가 그 슬롯 번호 이름("1".."9")으로 teleop_core 의 poses_file(JSON)
  에 저장된다. g 를 누르면 그 슬롯으로 팔만 이동한다(그리퍼·arm_joint_5 는 항상
  별도 조작 — 각각 6/5번 키로 선택 후 jog). p/g 뒤에 숫자가
  아닌 키(Enter 등)를 누르면 기존처럼 이름을 직접 입력할 수 있다(예: "home").
  예) 1번 자세에서 p → 1 눌러 슬롯 1 저장 → 조그로 새 자세로 이동 →
      p → 2 눌러 슬롯 2 저장 → 이후 g → 1 / g → 2 로 오갈 수 있다.
  화면 하단에 teleop_core 가 알고 있는 저장된 자세(슬롯+이름) 목록이 표시된다.

프리드라이브 저장 (2026-08-02 추가)
  p 를 누르는 순간 "freedrive" 명령이 먼저 나가 팔 관절(그리퍼+arm_joint_5 제외) 토크가
  즉시 꺼진다 — 이 상태에서 손으로 원하는 자세를 잡고 슬롯/이름을 입력하면
  "save"가 그 각도를 저장함과 동시에 teleop_core 가 그 자세 그대로 토크를
  복귀시킨다(자세히는 teleop_core_node.py 모듈 docstring 참고). ESC 로
  프롬프트를 취소하면 "freedrive_cancel"이 나가 저장 없이 토크만 복귀한다.
  프롬프트가 뜬 동안(블로킹 입력 대기) 얼마든지 시간을 들여 손으로 자세를
  잡아도 된다 — torque OFF 는 슬롯/이름을 확정하기 전까지 유지된다.

리미트 측정 모드 (2026-08-02 추가)
  c 를 누르면 teleop_core 에 "calib_start"가 나가 팔 관절(그리퍼+arm_joint_5 제외)
  토크가 전부 꺼지고, 첫 축(arm_joint_1)의 "하한" 측정 대기 상태가 된다. 화면 상단에
  배너로 "리미트 측정 중: <축> (n/전체) — 하한/상한선까지 손으로 이동 후 m" 가
  뜬다 — 이 배너가 현재 몇 번째 축의 어느 단계인지 알려주는 유일한 표시라
  잘 보고 진행할 것. 손으로 그 축을 실제 기구적 하한까지 밀고 m 을 누르면
  "calib_mark"가 나가 그 각도가 하한으로 기록되고, 배너가 "상한"으로 바뀐다.
  다시 손으로 상한까지 밀고 m 을 누르면 상한이 기록되고 다음 축으로 자동 진행된다.
  이 과정을 arm_joint_1..4(연결된 축만, arm_joint_5·그리퍼는 대상 아님) 전부 반복하면
  teleop_core 가 자동으로 결과를 적용하고 joint_limits_file 에 저장한 뒤 그 자리에서
  토크를 복귀시킨다("완료" 배너). 중간에 잘못 눌렀거나 그만두고 싶으면 x 로
  취소한다 — 그때까지 기록한 값은 버려지고 토크만 복귀한다(재시작은 c 부터 처음부터).
  m/x 는 리미트 측정 중이 아닐 때 눌러도 teleop_core 가 경고만 찍고 무시한다.

jog 를 velocity 방식으로 전환 — "떼도 계속 도는" 버그 수정 (2026-08-02)
  예전엔 ↑/↓ 를 누를 때마다 즉시 displacement(위치 증분)를 하나씩 보냈다. 터미널
  auto-repeat 로 큐에 쌓인 키를 한 프레임에 다 비우게 고쳤는데도(위 run() 참고)
  여전히 "꾹 눌렀다 떼도 계속 돈다" 는 문제가 남아있었다 — 원인이 큐가 아니라
  구조 자체였다: 오래 누르고 있으면 매 프레임 goal_rad 가 계속 앞으로 밀리고,
  손을 뗀 시점엔 이미 실제 위치보다 한참 앞선 목표가 잡혀있어서, profile
  velocity(느리게 설정돼 있음)로 그 목표까지 따라가는 동안 계속 도는 것처럼
  보였다 — 손을 뗀 뒤에도 "이미 커밋된 목표"를 향해 정직하게 움직이고 있었을
  뿐이다.
  joystick_teleop_node(이미 이 문제가 없다)를 참고해 같은 방식으로 바꿨다:
  displacement(즉시 스텝) 대신 **velocity**(rad/s)를 쓴다. ↑/↓ 를 누르는 동안은
  "눌려있는 상태"로 간주해 매 프레임(우리 루프 주기, ~10ms) teleop_core 에
  velocity 를 계속 재발행하고, teleop_core 의 on_timer 가 그 velocity 로
  goal_rad 를 매 틱 아주 조금씩만(v*dt) 앞으로 민다 — 그래서 목표가 실제
  위치보다 크게 앞서는 법이 없다. 손을 뗀 순간(다음 프레임에 그 키가 안 보임)
  즉시 velocity=0 을 보내므로 사실상 지연 없이 멈춘다.
  ⚠️ curses 는 "키를 뗐다"는 이벤트가 따로 없다(눌림 이벤트만 옴) — 그래서
  HOLD_TIMEOUT_S(짧게, 기본 0.15초) 동안 그 키가 다시 안 보이면 "뗐다"로
  간주한다. 이 값이 너무 짧으면 터미널의 최초 auto-repeat 지연(첫 반복 시작
  전 대기, 보통 0.3~0.5초) 사이에 살짝 끊겼다 이어지는 것처럼 보일 수 있고,
  너무 길면 뗀 뒤 정지가 그만큼 늦어진다 — "뗐을 때 바로 서는 것"이 이번 요청의
  핵심이라 짧은 쪽을 택했다(길게 눌렀을 때 시작 부분에 아주 짧은 끊김이 한 번
  있을 수 있는 대신, 놓았을 때는 확실히 바로 선다).
  [ / ] 는 이제 jog_step_rad(위치 증분 크기)가 아니라 jog_velocity_rad_s(속도)를
  조절한다 — teleop_core 의 max_vel_rad_s 가 여전히 최종 상한이다.

하드웨어 에러(과부하 등) 표시 · reboot (2026-08-01 추가)
  position_node 가 Hardware Error Status(과부하/과열/입력전압/엔코더/전기충격)를
  감지하면 /dynamixel/hardware_error 로 발행한다 — 이 노드는 그걸 그대로 구독해
  화면 맨 아래에 빨간 배너로 보여준다. 과부하 트립은 발생 직후(약 0.3초 내) 레지스터가
  자동으로 0 복귀하므로, position_node 가 한 번이라도 뜬 에러를 reboot 전까지
  소프트웨어에서 latch 해서 계속 보여준다(안 그러면 화면 갱신 타이밍에 놓칠 수 있음).
  r 로 트립된 관절을 reboot 하면 배너가 사라진다(재초기화까지 자동으로 수행됨).

* 반드시 실제 tty 가 있는 터미널에서 `ros2 run dynamixel_control keyboard_teleop` 로
  실행할 것 (launch 안에 넣으면 stdin 포커스를 못 받는다).
* bench.launch.py(하드웨어 브릿지) 와 한 번에 띄우려면 별도 터미널 2개 대신
  `scripts/run_keyboard_bench.sh` 를 쓸 것 — launch 를 백그라운드로 띄우고
  이 노드를 foreground(tty)에서 실행한 뒤, 종료 시 launch 트리까지 정리해준다.
* joint_names 기본값은 teleop_core/joystick_teleop 와 동일한 실기 관절명
  (arm_joint_1..5 + gripper_left_pinion_joint) 이다 — 어긋나면 teleop_core 가
  "알 수 없는 관절" 로 무시하고 모터가 안 움직인다.
* 각도 표시는 /joint_states 를 그대로 읽는다 — position_node(use_hardware:=true)가
  떠 있어야 실측값이 보인다. 안 떠 있으면 전 관절 "미수신"으로 보인다.
"""

import curses
import locale
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog


#: teleop_core_node.POSES_LIST_QOS 와 동일 프로파일 — transient_local 이라
#: 이 노드가 나중에 뜨거나 재시작해도 마지막 자세 목록을 바로 받는다.
POSES_LIST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


DEFAULT_JOINT_NAMES = [
    "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5",
    "gripper_left_pinion_joint",
]

STALE_AFTER_S = 0.5  # 이보다 오래된 /joint_states 샘플은 "지연"으로 표시

# curses 는 키를 뗀 이벤트가 없어서(눌림만 옴), 이 시간 동안 그 키가 다시 안
# 보이면 "뗐다"로 간주한다 — 위 모듈 docstring "jog 를 velocity 방식으로 전환"
# 참고. 짧게 잡아야 뗐을 때 바로 선다(대신 아주 긴 hold 시작 부분에 터미널
# 최초 auto-repeat 지연만큼 짧은 끊김이 한 번 있을 수 있음 — 의도된 트레이드오프).
HOLD_TIMEOUT_S = 0.15


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        # jog 속도(rad/s) — teleop_core 의 max_vel_rad_s(1.0)가 최종 상한이라
        # 여기서 너무 크게 잡아도 서버에서 clamp 된다. [ / ] 로 실시간 조절 가능.
        # 2026-08-02: "위자드처럼 지연 없이 부드럽게" 요청으로 0.4→0.7 상향
        # (position_node profile_velocity 상향과 짝 — 같이 봐야 체감 속도가 맞음).
        self.declare_parameter("jog_velocity_rad_s", 0.7)
        self.declare_parameter("jog_velocity_delta", 0.2)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.jog_velocity = float(self.get_parameter("jog_velocity_rad_s").value)
        self.jog_velocity_delta = float(self.get_parameter("jog_velocity_delta").value)

        self.selected = 0  # 선택된 관절 인덱스
        # jog "눌림" 상태 재구성용(위 HOLD_TIMEOUT_S 주석 참고). 0 이면 안 눌림.
        self.held_direction = 0        # -1 | 0 | +1
        self.held_last_seen_t = 0.0    # time.monotonic() — 그 방향 키를 마지막으로 본 시각
        # position_node 의 실제 상태를 구독하지 않고 이 노드가 낙관적으로 추정한
        # 값(2026-08-02 추가) — 'e' 키를 누를 때마다 반전해서 "spike on"/"spike
        # off" 를 보낸다. 화면 표시용일 뿐 실제 on/off 판단 기준은 position_node.
        self.spike_enabled = True
        # 관절 리밋 마스터 스위치의 낙관적 추정치(2026-08-02 추가, spike_enabled
        # 와 같은 성격) — 'l' 키를 누를 때마다 반전해서 "limit on"/"limit off"
        # 를 보낸다. 실제 on/off 판단 기준은 teleop_core.
        self.limits_enabled = True
        # 절대 과전류 트립 on/off 의 낙관적 추정치(2026-08-02 추가, spike_enabled
        # 와 같은 성격) — 'y' 키를 누를 때마다 반전해서 "trip on"/"trip off" 를
        # 보낸다. 실제 on/off 판단 기준은 position_node.
        self.trip_enabled = True

        # 화면 표시용 상태
        self.last_key_label = None
        self.last_action = None
        self.joint_pos = {}     # name -> rad (최신 /joint_states)
        self.joint_pos_t = {}   # name -> time.monotonic() 수신 시각
        self.known_poses = []   # teleop_core 가 발행하는 저장된 자세 이름 목록
        self.hw_error_text = ""  # position_node 가 발행하는 latch 된 하드웨어 에러 목록
        self.calib_state = "idle"   # "idle" | "active" | "done" | "cancelled"
        self.calib_axis = None      # active 일 때: 현재 측정 중인 관절 이름
        self.calib_step = None      # active 일 때: "lower" | "upper"
        self.calib_progress = ""    # active 일 때: "n/전체" 표시용
        self.calib_done_summary = ""  # done 일 때: "적용 n개, 거부 m개"

        self.jog_pub = self.create_publisher(JointJog, "/arm/teleop_jog", 10)
        self.cmd_pub = self.create_publisher(String, "/arm/teleop_cmd", 10)
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)
        self.poses_sub = self.create_subscription(
            String, "/arm/teleop_poses", self._on_poses_list, POSES_LIST_QOS)
        self.hw_error_sub = self.create_subscription(
            String, "/dynamixel/hardware_error", self._on_hw_error, 10)
        self.calib_status_sub = self.create_subscription(
            String, "/arm/calib_status", self._on_calib_status, POSES_LIST_QOS)

    # ------------------------------------------------------------------ ROS
    def _on_joint_states(self, msg):
        now = time.monotonic()
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.joint_pos[name] = float(pos)
                self.joint_pos_t[name] = now

    def _on_poses_list(self, msg):
        self.known_poses = [n for n in msg.data.split(",") if n]

    def _on_hw_error(self, msg):
        self.hw_error_text = msg.data

    def _on_calib_status(self, msg):
        """teleop_core_node.TeleopCore._publish_calib_status 가 만드는 포맷 파싱.

        "idle" | "active,<축>,<lower|upper>,<n>,<전체>" | "done,<적용수>,<거부수>"
        | "cancelled". active 일 때는 화면 선택 표시도 현재 측정 축으로 맞춰서
        (1-6 수동 선택보다 우선) 지금 뭘 만지고 있는지 헷갈리지 않게 한다.
        done 에 거부수가 있으면(값이 이상해서 teleop_core 가 적용을 거부한 축이
        있다는 뜻, 2026-08-02 추가) 배너에서 로그를 보라고 알려준다.
        """
        parts = msg.data.split(",")
        self.calib_state = parts[0]
        self.calib_done_summary = ""
        if self.calib_state == "active" and len(parts) == 5:
            _, axis, step, idx, total = parts
            self.calib_axis = axis
            self.calib_step = step
            self.calib_progress = f"{idx}/{total}"
            if axis in self.joint_names:
                self.selected = self.joint_names.index(axis)
        else:
            self.calib_axis = None
            self.calib_step = None
            self.calib_progress = ""
            if self.calib_state == "done" and len(parts) == 3:
                self.calib_done_summary = f"적용 {parts[1]}개, 거부 {parts[2]}개"

    def _publish_velocity_jog(self):
        """현재 held_direction 을 전 관절 velocity 메시지로 매 프레임 재발행한다.

        displacement 대신 velocity 를 쓰는 이유와 held_direction 재구성 방식은
        모듈 docstring "jog 를 velocity 방식으로 전환" 참고. 매 프레임(안 눌려
        있어도) 전 관절을 싣고 선택 안 된 축은 0 을 명시한다 —
        teleop_core_node 모듈 docstring 의 경고("메시지에 없는 관절의 velocity
        는 안 건드려진다") 그대로, joystick_teleop_node._publish_velocities 와
        동일한 패턴이라 stray velocity 가 남을 수 없다.
        """
        name = self.joint_names[self.selected]
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self.joint_names)
        v = float(self.held_direction) * self.jog_velocity
        msg.velocities = [v if n == name else 0.0 for n in self.joint_names]
        self.jog_pub.publish(msg)
        if self.held_direction != 0:
            self.last_action = (
                f"jog {name} {'+' if self.held_direction > 0 else '-'} "
                f"(v={self.jog_velocity:.2f} rad/s)"
            )

    def _publish_cmd(self, cmd):
        self.cmd_pub.publish(String(data=cmd))
        self.last_action = f"명령 전송: {cmd}"

    # ------------------------------------------------------------------ input
    def handle_key(self, key, stdscr):
        """curses 키코드 하나 처리. 종료하려면 False 반환."""
        # jog 키가 아닌 다른 키가 들어왔다는 건 그 손가락이 화살표에서 떨어졌다는
        # 뜻이다 — 특히 p/g 처럼 블로킹 프롬프트로 들어가는 키는 그 안에서
        # _publish_velocity_jog() 가 한동안 안 불리므로, 여기서 먼저 꺼두지
        # 않으면 프롬프트가 열려있는 동안 teleop_core 에 마지막 velocity 가
        # 그대로 남아 deadman_timeout_s(0.5초)까지 계속 움직일 수 있다.
        if key not in (curses.KEY_UP, ord("w"), ord("W"), curses.KEY_DOWN, ord("s"), ord("S")):
            if self.held_direction != 0:
                self.held_direction = 0
                # p/g 처럼 이 아래에서 블로킹 프롬프트로 들어가면 run() 루프가
                # 한동안 안 돌아서 _publish_velocity_jog() 가 안 불린다 — 여기서
                # 즉시 한 번 보내서 프롬프트가 열려있는 동안 teleop_core 에
                # 옛 velocity 가 안 남게 한다.
                self._publish_velocity_jog()

        if key in (ord("q"), ord("Q"), 3):  # q or Ctrl-C
            self.last_key_label = "q"
            return False
        elif key in (curses.KEY_UP, ord("w"), ord("W")):
            self.last_key_label = "↑ / w"
            self.held_direction = 1
            self.held_last_seen_t = time.monotonic()
        elif key in (curses.KEY_DOWN, ord("s"), ord("S")):
            self.last_key_label = "↓ / s"
            self.held_direction = -1
            self.held_last_seen_t = time.monotonic()
        elif key == ord("["):
            self.jog_velocity = max(0.05, self.jog_velocity - self.jog_velocity_delta)
            self.last_key_label = "["
            self.last_action = f"jog 속도 감소 → {self.jog_velocity:.2f} rad/s"
        elif key == ord("]"):
            self.jog_velocity += self.jog_velocity_delta
            self.last_key_label = "]"
            self.last_action = f"jog 속도 증가 → {self.jog_velocity:.2f} rad/s"
        elif key in (ord("h"), ord("H")):
            self.last_key_label = "h"
            self._publish_cmd("home")
        elif key in (ord("p"), ord("P")):
            self.last_key_label = "p"
            # freedrive: 프롬프트 뜨기 전에 먼저 토크부터 끈다 — 프롬프트가
            # 블로킹으로 대기하는 동안 손으로 자세를 잡을 수 있게.
            self._publish_cmd("freedrive")
            self.last_action = "freedrive: 팔 관절 토크 OFF — 손으로 자세를 잡은 뒤 슬롯 선택"
            name = self._prompt_slot_or_name(stdscr, "저장")
            if name:
                self._publish_cmd(f"save {name}")
                self.last_action = f"자세 저장 요청: '{name}' (저장된 자세로 토크 복귀)"
            else:
                self._publish_cmd("freedrive_cancel")
                self.last_action = "자세 저장 취소 (토크 복귀)"
        elif key in (ord("g"), ord("G")):
            self.last_key_label = "g"
            name = self._prompt_slot_or_name(stdscr, "이동")
            if name:
                self._publish_cmd(f"goto {name}")
                self.last_action = f"자세 이동 요청: '{name}'"
            else:
                self.last_action = "자세 이동 취소"
        elif key == ord("r"):
            self.last_key_label = "r"
            name = self.joint_names[self.selected]
            self._publish_cmd(f"reboot {name}")
            self.last_action = f"reboot 요청: {name}"
        elif key == ord("R"):
            self.last_key_label = "R"
            self._publish_cmd("reboot all")
            self.last_action = "reboot 요청: 에러난 서보 전체"
        elif key in (ord("c"), ord("C")):
            self.last_key_label = "c"
            self._publish_cmd("calib_start")
            self.last_action = "리미트 측정 모드 시작 요청 (팔 관절 토크 OFF)"
        elif key in (ord("m"), ord("M")):
            self.last_key_label = "m"
            self._publish_cmd("calib_mark")
            self.last_action = "리미트 측정: 현재 각도 기록 요청"
        elif key in (ord("x"), ord("X")):
            self.last_key_label = "x"
            self._publish_cmd("calib_cancel")
            self.last_action = "리미트 측정 취소 요청"
        elif key in (ord("e"), ord("E")):
            self.last_key_label = "e"
            self.spike_enabled = not self.spike_enabled
            self._publish_cmd("spike on" if self.spike_enabled else "spike off")
            self.last_action = f"전류 급변 감지 {'ON' if self.spike_enabled else 'OFF'} 요청"
        elif key in (ord("l"), ord("L")):
            self.last_key_label = "l"
            self.limits_enabled = not self.limits_enabled
            self._publish_cmd("limit on" if self.limits_enabled else "limit off")
            self.last_action = f"관절 리밋 {'ON' if self.limits_enabled else 'OFF'} 요청"
        elif key in (ord("y"), ord("Y")):
            self.last_key_label = "y"
            self.trip_enabled = not self.trip_enabled
            self._publish_cmd("trip on" if self.trip_enabled else "trip off")
            self.last_action = f"절대 과전류 트립 {'ON' if self.trip_enabled else 'OFF'} 요청"
        elif key == ord(" "):
            self.last_key_label = "space"
            self._publish_cmd("stop")
            self.last_action = "정지 요청: 즉시 정지 + 전 관절 토크 차단 (재개: t)"
        elif key in (ord("t"), ord("T")):
            self.last_key_label = "t"
            self._publish_cmd("resume")
            self.last_action = "재개 요청: 전 관절 토크 복귀"
        elif ord("1") <= key <= ord("9"):
            idx = key - ord("1")
            self.last_key_label = chr(key)
            if idx < len(self.joint_names):
                self.selected = idx
                self.last_action = f"관절 선택: {self.joint_names[idx]}"
            else:
                self.last_action = f"관절 {chr(key)} 없음"
        return True

    def _prompt_slot_or_name(self, stdscr, action_label):
        """자세 저장/이동을 슬롯(1-9)으로 빠르게 고른다 (2026-08-01 추가).

        숫자 1-9 한 키로 즉시 슬롯 이름("1".."9")을 확정한다(Enter 불필요).
        그 외 키(Enter 등)를 누르면 기존 자유 이름 입력(_prompt_text)으로 넘어가
        'home' 같은 이름 자세도 그대로 저장/이동할 수 있다. ESC 는 취소.
        """
        h, w = stdscr.getmaxyx()
        row = max(h - 1, 0)
        prompt = f"{action_label} 슬롯(1-9) — 그 외 키=이름 직접입력, ESC=취소: "
        self._addstr(stdscr, row, 0, " " * max(w - 1, 0))
        self._addstr(stdscr, row, 0, prompt[: max(w - 1, 0)])
        stdscr.refresh()

        stdscr.nodelay(False)
        stdscr.timeout(-1)
        try:
            key = stdscr.getch()
        except curses.error:
            key = -1
        finally:
            stdscr.nodelay(True)
            stdscr.timeout(10)

        if key == 27:  # ESC
            return None
        if ord("1") <= key <= ord("9"):
            return chr(key)
        return self._prompt_text(stdscr, f"{action_label}할 자세 이름 (Enter=취소): ")

    def _prompt_text(self, stdscr, prompt):
        """화면 맨 아래 줄에서 한 줄 텍스트를 blocking 으로 입력받는다.

        입력 중엔 rclpy.spin_once 가 멈추므로(메인 루프가 여기서 대기) 길게
        끌지 않도록 이름 하나만 받고 바로 돌아간다. 빈 입력은 취소로 처리.
        """
        h, w = stdscr.getmaxyx()
        row = max(h - 1, 0)
        self._addstr(stdscr, row, 0, " " * max(w - 1, 0))
        self._addstr(stdscr, row, 0, prompt[: max(w - 1, 0)])
        stdscr.refresh()

        curses.curs_set(1)
        curses.echo()
        stdscr.nodelay(False)
        stdscr.timeout(-1)
        try:
            raw = stdscr.getstr(row, min(len(prompt), max(w - 1, 0)))
            text = raw.decode("utf-8", errors="ignore").strip()
        except (curses.error, UnicodeDecodeError):
            text = ""
        finally:
            curses.noecho()
            curses.curs_set(0)
            stdscr.nodelay(True)
            stdscr.timeout(10)
        return text or None

    # ------------------------------------------------------------------ render
    def _addstr(self, stdscr, y, x, text, attr=0):
        try:
            stdscr.addstr(y, x, text, attr)
        except curses.error:
            pass  # 터미널 경계 밖 — 무시(리사이즈 등)

    def render(self, stdscr):
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        self._addstr(stdscr, 0, 0, " 로봇팔 키보드 텔레옵 ".center(max(w - 1, 0), "="), curses.A_BOLD)
        header = (
            f" 선택: {self.joint_names[self.selected]:<28} "
            f"jog속도: {self.jog_velocity:.2f}rad/s   "
            f"마지막 키: {self.last_key_label or '-':<8} "
            f"마지막 동작: {self.last_action or '-'}"
        )
        self._addstr(stdscr, 1, 0, header[: max(w - 1, 0)])
        self._addstr(stdscr, 2, 0, "-" * min(w - 1, 100))

        col = f"{'#':>2}  {'관절':<28}{'선택':^6}{'각도(deg)':>12}{'각도(rad)':>12}{'수신':>10}"
        self._addstr(stdscr, 3, 0, col[: max(w - 1, 0)], curses.A_UNDERLINE)

        now = time.monotonic()
        for i, name in enumerate(self.joint_names):
            row = 4 + i
            pos = self.joint_pos.get(name)
            stamp = self.joint_pos_t.get(name)
            marker = "◀ 선택" if i == self.selected else ""

            attr = curses.A_REVERSE if i == self.selected else 0
            if pos is None:
                deg_s, rad_s, status = "--", "--", "미수신"
                status_attr = curses.color_pair(2)
            else:
                deg_s = f"{math.degrees(pos):+.2f}"
                rad_s = f"{pos:+.3f}"
                age = (now - stamp) if stamp is not None else None
                if age is not None and age < STALE_AFTER_S:
                    status = f"{age * 1000:.0f}ms"
                    status_attr = curses.color_pair(1)
                else:
                    status = "지연⚠"
                    status_attr = curses.color_pair(2)

            line = f"{i + 1:>2}  {name:<28}{marker:^6}{deg_s:>12}{rad_s:>12}"
            self._addstr(stdscr, row, 0, line[: max(w - 1, 0)], attr)
            self._addstr(
                stdscr, row, min(len(line), max(w - 1, 0)),
                f"{status:>10}", status_attr | attr)

        sep = "-" * min(w - 1, 100)

        def _sep(r):
            self._addstr(stdscr, r, 0, sep)
            return r + 1

        help_row = 4 + len(self.joint_names) + 1
        row = _sep(help_row)
        # 2026-08-02: 한 줄로 몰아넣던 키맵을 폭이 좁은 터미널에서도 안 잘리게
        # 의미 단위(이동/자세·리부트·측정/정지·토글류)로 줄바꿈하고, 블록 사이에
        # 구분선을 넣었다(둘 다 사용자 요청).
        help_lines = [
            "1-6 관절선택 | ↑/w +jog  ↓/s -jog(누르는 동안 계속) | [ ] jog속도조절 | h 홈이동",
            "p+숫자 슬롯저장 | g+숫자 슬롯이동 | r/R reboot | c/m/x 리미트측정",
            "space 즉시정지+토크차단 | t 재개 | e 전류급변on/off | l 관절리밋on/off | "
            "y 과전류트립on/off | q 종료",
        ]
        for text in help_lines:
            self._addstr(stdscr, row, 0, text[: max(w - 1, 0)])
            row += 1
        row = _sep(row)

        spike_label = "ON" if self.spike_enabled else "OFF"
        limit_label = "ON" if self.limits_enabled else "OFF"
        trip_label = "ON" if self.trip_enabled else "OFF"
        status_text = (
            f"전류 급변 감지(추정): {spike_label}   관절 리밋(추정): {limit_label}   "
            f"과전류 트립(추정): {trip_label}")
        status_attr = curses.color_pair(2) | curses.A_BOLD if (
            not self.spike_enabled or not self.limits_enabled or not self.trip_enabled) else 0
        self._addstr(stdscr, row, 0, status_text[: max(w - 1, 0)], status_attr)
        row += 1
        row = _sep(row)

        poses_text = "저장된 자세: " + (
            ", ".join(self.known_poses)
            if self.known_poses else "(없음 — p 로 저장)")
        self._addstr(stdscr, row, 0, poses_text[: max(w - 1, 0)])
        row += 1
        row = _sep(row)

        any_fresh = any(
            (now - t) < STALE_AFTER_S for t in self.joint_pos_t.values()
        )
        conn_text = "/joint_states 수신 정상" if any_fresh else \
            "/joint_states 수신 없음 — position_node(use_hardware:=true) 및 모터 연결 확인"
        conn_attr = curses.color_pair(1) if any_fresh else curses.color_pair(2)
        self._addstr(stdscr, row, 0, conn_text[: max(w - 1, 0)], conn_attr)
        row += 1

        if self.hw_error_text:
            err_text = f"⚠ 하드웨어 에러 — r(선택관절) / R(전체) 로 reboot: {self.hw_error_text}"
            self._addstr(
                stdscr, row, 0, err_text[: max(w - 1, 0)],
                curses.color_pair(2) | curses.A_BOLD | curses.A_REVERSE)
        else:
            self._addstr(
                stdscr, row, 0, "하드웨어 에러 없음"[: max(w - 1, 0)],
                curses.color_pair(1))
        row += 1
        row = _sep(row)

        if self.calib_state == "active":
            step_label = "하한선" if self.calib_step == "lower" else "상한선"
            calib_text = (
                f"[리미트 측정 중] {self.calib_axis} ({self.calib_progress}) — "
                f"{step_label}까지 손으로 이동 후 m, 취소는 x"
            )
            self._addstr(
                stdscr, row, 0, calib_text[: max(w - 1, 0)],
                curses.color_pair(2) | curses.A_BOLD)
        elif self.calib_state == "done":
            has_rejected = bool(self.calib_done_summary) and "거부 0개" not in self.calib_done_summary
            done_text = f"리미트 측정 완료 ({self.calib_done_summary}), 토크 복귀"
            if has_rejected:
                done_text += " — 거부된 축은 값이 이상해서 안 걸렸습니다, 로그 확인 후 재측정하세요"
            self._addstr(
                stdscr, row, 0, done_text[: max(w - 1, 0)],
                (curses.color_pair(2) | curses.A_BOLD) if has_rejected else curses.color_pair(1))
        elif self.calib_state == "cancelled":
            self._addstr(
                stdscr, row, 0,
                "리미트 측정 취소됨 — 토크 복귀"[: max(w - 1, 0)])

        stdscr.refresh()

    # ------------------------------------------------------------------ main loop
    def run(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)
        stdscr.timeout(10)  # ms — getch 최대 대기, 이게 곧 프레임 주기(2026-08-01: 지연 최소화, 30→10)

        if curses.has_colors():
            curses.start_color()
            curses.use_default_colors()
            curses.init_pair(1, curses.COLOR_GREEN, -1)  # 신선한 데이터
            curses.init_pair(2, curses.COLOR_RED, -1)    # 미수신/지연

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            # 터미널 auto-repeat 이 우리 루프보다 빨리 키를 쌓으면(2026-08-02
            # 실사용 보고 — "화살표를 꾹 눌렀다 떼도 계속 움직임") 큐에 쌓인 걸
            # 한 프레임에 하나씩 다 처리하다 보니, 손을 뗀 뒤에도 쌓인 만큼
            # 계속 움직인 것처럼 보였다. 매 프레임 큐를 전부 비우고 **가장
            # 최근 키 하나만** 적용한다 — 그러면 우리 루프 한 바퀴당 최대 한
            # 스텝만 나가고, 실제로 키를 놓는 순간(다음 getch 가 즉시 -1) 바로
            # 멈춘다. 딜레이를 넣는 건 반대 방향이다 — 생산 속도(터미널
            # auto-repeat)는 그대로인데 소비만 느려지면 큐가 오히려 더
            # 쌓인다. 그래서 딜레이가 아니라 드레인이 맞는 해법이다.
            key = stdscr.getch()
            latest_key = -1
            while key != -1:
                latest_key = key
                key = stdscr.getch()

            if latest_key != -1:
                if not self.handle_key(latest_key, stdscr):
                    break

            # jog 키가 HOLD_TIMEOUT_S 동안 다시 안 보이면 "뗐다"로 간주하고
            # 끈다 — curses 에 키를 뗀 이벤트가 따로 없어서(모듈 docstring
            # "jog 를 velocity 방식으로 전환" 참고) 이 타임아웃으로 재구성한다.
            if (self.held_direction != 0
                    and time.monotonic() - self.held_last_seen_t > HOLD_TIMEOUT_S):
                self.held_direction = 0

            # 매 프레임 재발행 — 눌려있는 동안은 velocity 를 계속 신선하게
            # 유지하고, 안 눌려있어도(held_direction=0) 0 을 계속 보내서
            # teleop_core 쪽에 stray velocity 가 남지 않게 한다.
            self._publish_velocity_jog()

            self.render(stdscr)


def main(args=None):
    locale.setlocale(locale.LC_ALL, "")  # 한글 등 wide-char 출력을 위해 필요
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        curses.wrapper(node.run)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
