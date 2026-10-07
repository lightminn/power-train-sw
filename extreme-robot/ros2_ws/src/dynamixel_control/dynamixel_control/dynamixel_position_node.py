import math
import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import JointState

from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupSyncRead, GroupSyncWrite,
)

from dynamixel_control import bus_lock


#: teleop_core_node.POSES_LIST_QOS 와 동일 — tick_limits 는 transient_local 로 오므로
#: (teleop_core 재기동 없이 이 노드만 늦게/다시 뜨는 경우에도) 놓치지 않고 받는다.
TICK_LIMITS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


# =========================
# Dynamixel 기본 설정
# =========================
DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# =========================
# XL430 Control Table 주소
# =========================
ADDR_OPERATING_MODE = 11        # EEPROM — 토크 OFF 상태에서만 써진다
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146
ADDR_HARDWARE_ERROR_STATUS = 70  # 과부하 등 보호 트립 상태 — 126~135 블록과 안 붙어있어 별도 SyncRead

# 126~135 는 연속이다 (current 2 + velocity 4 + position 4) → SyncRead 한 번에 읽는다.
SYNC_READ_ADDR = ADDR_PRESENT_CURRENT
SYNC_READ_LEN = 10
LEN_HARDWARE_ERROR_STATUS = 1

LEN_GOAL_POSITION = 4

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

OP_MODE_POSITION = 3            # 0~4095 단일회전 위치제어
# 다회전(멀티턴) 위치제어 — 감속기(기어박스)가 달려서 서보 자신은 여러 바퀴
# 돌아야 관절이 그 비율만큼만 도는 축용(2026-08-02 사용자 확인: arm_joint_2/3
# 에 약 10:1 기어박스). 단일회전 모드(3)는 raw position 이 0~4095 로 캡되므로
# 기어비만큼의 회전을 못 담는다 — 이 모드는 그 캡이 없다(대략 ±256회전).
OP_MODE_EXTENDED_POSITION = 4

# Hardware Error Status(70) 비트 — X 시리즈(XL430/XM430) 공통.
# ⚠️ 과부하 트립은 발생 즉시 토크가 풀리며 이 레지스터가 잠깐 세워졌다가 자동으로
# 0 으로 복귀한다(HW-8 실기 관찰, 트립 후 0.3초 내) — 폴링만으로는 놓칠 수 있어서
# 한 번이라도 뜨면 reboot 될 때까지 소프트웨어에서 latch 해서 계속 보여준다.
#
# bit 6 은 실제 서보 레지스터 비트가 아니라, 이 노드가 PRESENT_CURRENT 를 직접
# 감시하다 current_trip_threshold 를 넘으면 합성으로 세우는 값이다(2026-08-02
# 추가 — 리미트 측정 모드 등에서 실제 이동 가능 범위를 넘어서 뭔가에 걸렸을 때,
# 서보 자체의 과부하 보호(위 5번 비트)보다 먼저/확실하게 토크를 끊기 위한 소프트
# 안전장치). error_latched/HW_ERROR_BITS 인프라를 그대로 재사용해서 기존 hw_error
# 배너·reboot 로 해제하는 흐름에 자연스럽게 올라탄다.
HW_ERROR_BITS = {
    0: "입력전압",
    2: "과열",
    3: "엔코더",
    4: "전기충격",
    5: "과부하",
    6: "전류초과(SW)",
    7: "전류급변(SW,비상정지)",
}
CURRENT_TRIP_BIT = 6
# 2026-08-02 실기(그리퍼)에서 정상적인 파지(오므리기) 중 전류가 계속 오르다가
# 400에서 트립되는 게 재현됨(current_spike 는 별개로 껐는데도 발생 — 이건
# 절대 트립 쪽). 400 은 XL430-W250 정격전류(540mA) 대비 너무 타이트했다고
# 보고 여유를 더 둔다. current_spike_delta_threshold 재조정 때와 같은 이유로,
# 절대적으로 "이 값이 맞다"는 실측은 아직 없다 — 런타임 on/off·조정(아래
# current_trip_config 콜백)으로 실기 튜닝하며 좁혀갈 것.
DEFAULT_CURRENT_TRIP_THRESHOLD = 500

# bit 7 도 bit 6 과 같은 합성 비트(실제 레지스터 아님) — 2026-08-02 추가.
# current_trip_threshold(절대값)는 테스트 모터로는 "정상 범위"가 뭔지 알 수
# 없어서 의미가 약하다는 사용자 판단으로, **절대값 대신 변화량**을 본다.
# 이 급변을 감지하면 그 서보 하나만이 아니라 **전 관절 비상정지**(disable_all_
# torque 재사용)를 건다 — 사용자가 명시적으로 "비상정지"를 요청했고, 급변은
# 팔 전체가 위험한 자세일 수 있다는 신호라 한 관절만 끄는 걸로는 부족하다고
# 판단. abs(current)-baseline 으로 봐서(부호와 무관하게 "힘을 더 쓰는 쪽"만
# 본다) 토크를 방금 끄거나 켜서 생기는 자연스러운 큰 낙폭(effort 감소)은
# 오탐하지 않는다.
#
# ⚠️ 2026-08-02 실기 재조정 히스토리:
# 1차: "직전 딱 한 번 읽은 값"과만 비교 — 손으로 직접 반대 토크를 걸어서
#      실측해보니(사용자 요청 실험) 전류가 -153~206 까지 크게 튀었는데도 연속
#      두 읽기(30Hz, ~33ms) 사이 최대 변화폭은 42밖에 안 나와서 안 걸림(사람
#      손 힘은 300~500ms 에 걸쳐 서서히 실림). → 최근 CURRENT_SPIKE_WINDOW
#      개(약 0.3초 창)의 최솟값을 기준선으로 삼는 윈도우 방식으로 교체.
# 2차: 그래도 실기에서 재차 안 걸림(윈도우를 몇 배로 늘려도(최대 10초) 마찬가지).
#      150 초 원본 트레이스(타임스탬프별 전류)를 저장해 직접 분석한 결과: 이
#      트레이스는 전 구간이 사실상 **정상 조그 주행**(velocity ±28~30 raw)이었고,
#      **정지 구간(velocity=0)에서는 전류가 완전히 평평**했다(예: 33, -34 가
#      20초+ 동안 소수점까지 고정) — 반대 토크를 준 흔적(정지 상태의 전류
#      흔들림)이 트레이스에 없었다. 결정적으로: 조그 주행 자체만으로도(관절
#      각도에 따른 중력 부하 변화) 어떤 윈도우 크기(0.3~10초)로 봐도 전류가
#      최대 160까지 튄다 — 이는 그동안 실기에서 관측된 "손 토크" 신호(-153→206,
#      -166→85)와 크기가 겹친다. 즉 **절대/변화량만으로는 "정상 조그 중 부하
#      변화"와 "사람이 미는 힘"을 구분할 수 없다**는 게 근본 원인이었다.
# 3차(같은 날): 그래서 "정지 중일 때만 검사"로 한 번 고쳤는데 — 그러면 **주행
#      중엔 검사 자체가 꺼지는 셈**이라, 주행 중에 급변이 나도 남는 안전장치는
#      current_trip_threshold(절대값 400)뿐인데 이건 정상 주행 중 최대치(160)
#      보다 훨씬 높아서 사실상 주행 중엔 손 힘을 아무리 줘도 안 걸림(사용자
#      실기로 재확인) → 그래서 "정지↔주행을 다른 threshold로" 로 바꿨는데,
#      이번엔 **정지↔주행 전환 시점에 창을 비운다는 규칙 자체가 결함**이었다:
#      사람 손이 모터를 이겨서 속도가 0으로 떨어지는 바로 그 순간이 곧 스파이크가
#      난 순간인데, 그 전환 프레임을 "새 기준선"으로 저장해버려서 baseline 자체가
#      이미 스파이크된 값이 되어(예: 186) 그 다음 샘플들과 델타가 안 남 — 실기
#      재현: 두 번의 실제 저항 이벤트(아래 4차) 모두 이 버그로 미탐됐다.
# 4차(같은 날, 40초 짧은 트레이스 재실측): **속도 게이팅을 완전히 걷어내고**
#      단일 연속 윈도우로 되돌렸다 — 다만 윈도우를 훨씬 크게(10→90 샘플,
#      0.33초→약 3초) 키웠다. 근거: 이 40초 트레이스에 사람이 조그 중 실제로
#      두 번 저항을 준 구간이 뚜렷이 잡혔다(위치가 멈춘 채 전류가 t≈16.3초에
#      190, t≈28.3초에 249까지 치솟음) — 큰 연속 윈도우(60~90 샘플)로 최솟값
#      기준선을 잡으면 두 이벤트 모두 델타 186~249 로 뚜렷이 검출된다. 반면
#      이전 150초 순수 조그 트레이스는 **윈도우를 아무리 키워도(최대 300 샘플=
#      9.9초) 델타가 160을 못 넘는다** — 즉 90 샘플(~3초) 윈도우 + threshold
#      185 는 정상 조그(상한 160)와 실측 저항 이벤트(186~249)를 명확히 가른다.
#      게이팅이 없으니 "전환 순간에 기준선이 오염되는" 문제 자체가 사라진다.
CURRENT_SPIKE_BIT = 7
# 5차(2026-08-02, 실서보 전환 후): 위 4차 값(185)은 **테스트 모터**(가벼운 부하)
# 실측 기준이었다 — 실서보로 바꾸니 진짜 팔 링크의 무게/기어박스 부하 때문에
# 조그를 처음 시작하는 순간(정지→가속) 자체의 정상적인 전류 상승만으로도 이
# 문턱을 넘어 비상정지가 걸렸다(사용자 실기 보고). 실서보의 "정상 기동 전류
# 상한"을 아직 실측하지 못한 상태라 감으로 낮춰 잡느니 넉넉히 올려둔다 —
# current_trip_threshold(절대값 트립, 기본 400)보다는 낮게 유지해 최소한의
# 구분은 남긴다. 실서보 정상 조그/기동 전류 로그를 모아 다시 좁혀야 한다.
DEFAULT_CURRENT_SPIKE_DELTA = 350

# 전류 가드(절대 트립 + 급변 감지)에서 제외할 서보 ID — 선언부 주석 참고.
DEFAULT_CURRENT_GUARD_EXEMPT_IDS = [11]
# read_rate_hz=30 기준 약 3초 — 짧으면(과거 10샘플/0.33초) 사람 손처럼
# 300ms~1초 넘게 걸리는 힘을 놓친다(위 히스토리 1~2차). 길면 최솟값 기준선이
# 더 오래된 값을 참조하게 되지만, 실측상 정상 조그의 델타 상한(160, 테스트
# 모터 기준)은 윈도우를 늘려도 더 안 커져서(150초 트레이스로 확인) 부작용
# 없이 키울 수 있었다.
CURRENT_SPIKE_WINDOW = 90


def _set_ftdi_latency(port, value, logger):
    """FTDI 의 latency_timer 를 낮춘다.

    기본값 16ms 는 '보낼 데이터가 적으면 최대 16ms 모았다가 올린다'는 뜻이라,
    요청-응답으로 도는 다이나믹셀 통신에서는 **왕복마다 16ms** 가 그대로 붙는다.
    6관절 상태 읽기가 2.6Hz 까지 떨어졌던 원인이 이것이다.

    USB 를 다시 꽂으면 16 으로 되돌아가므로 기동할 때마다 직접 맞춘다.
    (컨테이너가 privileged 라 sysfs 에 쓸 수 있다. 안 되면 경고만 남기고 진행.)
    """
    name = os.path.basename(port)
    path = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    try:
        with open(path, "r") as f:
            before = f.read().strip()
        if before == str(value):
            return
        with open(path, "w") as f:
            f.write(str(value))
        logger.info(f"FTDI latency_timer {before} → {value} ms ({name})")
    except OSError as exc:
        logger.warn(
            f"latency_timer 설정 실패 ({path}): {exc} — "
            f"통신이 느리면 이 값을 1 로 낮추세요")


def _signed(value, byte_count):
    """dynamixel_sdk 의 read*ByteTxRx 는 unsigned 로 준다 → 2의 보수로 되돌린다.

    velocity/current 는 음수가 정상값인데, 그대로 두면 -1 이 4294967295 가 되어
    Int32MultiArray(int32 범위) 대입에서 노드가 죽는다.
    """
    bits = byte_count * 8
    value &= (1 << bits) - 1
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


# 검증된 4축 arm 버스 매핑. arm_joint_1(ID 11)과 구형 gripper ID 3은 등록하지 않는다.
#
# ⚠️ 2026-08-02 그리퍼 단일 모터로 전환: 원래 물리 서보 2개(id 3,4)를 랙피니언
# 한 레일에 물려 이름을 두 번 등록해 항상 같은 tick 을 동시 발행했는데, 실기
# 테스트 결과 두 독립 위치제어 루프가 같은 강체 레일을 밀며 서로 힘을 겨뤄
# (미세한 추종 오차가 누적) current_trip_threshold 를 트립시키고, 트립 후엔
# goal_position 이 계속 갱신돼도 두 모터 다 응답을 멈추는 증상이 재현됨(손으로
# 돌려보면 걸리는 곳 없이 정상이라 기구적 문제 아님, 사용자 확인). 레일에 피니언
# 2개가 물려있어도 하나만 구동하면 나머지는 자유롭게 딸려 돈다 — 그래서 ID4를
# 아예 목록에서 뺐다(미등록 → 토크 안 걸림 → 자유회전, 레일에 저항 안 줌).
# teleop_core_node.py 의 DEFAULT_MOTOR_IDS 도 같이 뺐다.
DEFAULT_MOTOR_IDS = [14, 13, 12, 16]

# URDF(robot_arm.urdf)의 구동 관절 이름과 모터 ID 순서를 맞춤.
# 구동 조인트는 gripper_left_pinion_joint 하나뿐 — 나머지 그리퍼 관절(우 피니언·좌우 랙)은
# 전부 mimic 이라 여기 없다. 그리퍼는 ID 3 하나만 구동(위 DEFAULT_MOTOR_IDS 주석 참고).
DEFAULT_JOINT_NAMES = [
    "arm_joint_2",
    "arm_joint_3",
    "arm_joint_4",
    "arm_joint_5",
]

# 프로파일 가감속 기본값. **0(=최고속 즉시 이동)으로 두지 말 것** — 명령마다
# 순간 과전류로 토크가 풀린다(HW-8 실기 검증, 재현율 100%, 명령 후 0.3초 내 트립).
# accel 은 HW-8 검증값(25)을 그대로 유지 — 여긴 손대면 트립 재현 위험이 있다.
# velocity 는 이 버스의 모든 목표 이동(조그·자세 불러오기(goto)·freedrive 저장 홀드)이
# 공유하는 유일한 속도 레지스터다 — "자세 저장/불러오기 전용 속도" 같은 건 따로 없다.
# 2026-08-01: 사용자 요청으로 절반(80→40)으로 낮춤. 2026-08-02: 자세 불러오기(goto)가
# 여전히 빠르다는 사용자 요청으로 다시 절반(40→20).
# 2026-08-02(같은 날 재조정): "다이나믹셀 위자드처럼 지연 없이 부드럽게" 요청으로
# 20→50 상향. 지금 조그는 매 틱(publish_rate_hz=100Hz) goal_position 을 아주
# 조금씩만 미는데(velocity*dt), profile_velocity 가 너무 낮으면 서보가 그 작은
# 목표를 매번 가속 구간 안에서만 쫓아가다 다음 목표로 넘어가버려서 — "계속
# 가속만 하다 마는" 상태가 반복돼 오히려 뚝뚝 끊기고 굼떠 보인다("부드러움"과
# "속도 하한" 은 별개 축이었다). accel(가속도)은 그대로 두므로(트립 원인은
# "가속도 0" 이었지 velocity 크기가 아님) 트립 안전 마진은 그대로 유지된다.
# 참고로 이 값은 원래도 "1.0" 이 아니라 정수였다(Dynamixel Profile Velocity 는
# 0.229rev/min 단위, 0.0~1.0 스케일이 아니다).
DEFAULT_PROFILE_ACCELERATION = 25
DEFAULT_PROFILE_VELOCITY = 50


class DynamixelPositionNode(Node):
    def __init__(self):
        super().__init__("dynamixel_position_node")

        self.declare_parameter("port", DEVICENAME)
        self.declare_parameter("baudrate", BAUDRATE)
        self.declare_parameter("motor_ids", DEFAULT_MOTOR_IDS)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("profile_acceleration", DEFAULT_PROFILE_ACCELERATION)
        self.declare_parameter("profile_velocity", DEFAULT_PROFILE_VELOCITY)
        # 기동 시 operating mode 를 위치제어(3)로 맞출지. 버스에 velocity/extended
        # position 모드로 남아 있는 서보가 섞여 있으면 goal_position 이 먹지 않는다.
        self.declare_parameter("force_position_mode", True)
        # 감속기 달린 축의 모터 ID — force_position_mode 가 이 ID들만 예외로
        # 다회전(OP_MODE_EXTENDED_POSITION=4)을 강제한다(위 상수 주석 참고).
        # [14, 13] = arm_joint_2/arm_joint_3 의 실모터 ID(10:1 감속기).
        # 3(그리퍼) 은 감속기 때문이 아니라 랙피니언 스트로크가 거의 한 바퀴에
        # 맞먹어 단일회전 wrap 경계(0/4095)에 양쪽 다 걸리는 문제라 2026-08-02
        # 추가(teleop_core_node.py 의 EXTENDED_POSITION_NAMES 주석 참고, 이 파라미터
        # 와 짝 — 바뀌면 같이 바꿀 것).
        self.declare_parameter("extended_position_ids", [14, 13])
        # 2026-08-01: 지연 최소화 요청으로 read/write 둘 다 상향(20→30Hz, 50→100Hz).
        # flush_goals 는 pending_goals 가 비어있으면 그냥 return 이라 write_rate_hz를
        # 올려도 유휴 트래픽이 늘지 않는다 — 실제 명령이 있을 때만 주기가 짧아진다.
        self.declare_parameter("read_rate_hz", 30.0)
        # 목표값 flush 주기. 텔레옵(50Hz)보다 빨라야 명령이 한 틱씩 밀리지 않는다.
        self.declare_parameter("write_rate_hz", 100.0)
        # 온도는 SyncRead 묶음(126~135) 밖(146)이라 따로 읽어야 한다. 자주 읽을
        # 이유가 없으므로 매 주기 한 개씩 돌아가며 읽는다.
        self.declare_parameter("temperature_poll_hz", 2.0)
        # FTDI latency_timer [ms]. 0 이면 건드리지 않는다.
        self.declare_parameter("ftdi_latency_timer", 1)
        # PRESENT_CURRENT(126) 실측값이 이 절대값을 넘으면 그 서보 토크를 즉시 끈다
        # (2026-08-02 추가 — 리미트 측정 모드 등에서 실제 이동 가능 범위 밖으로
        # 밀다가 뭔가에 걸렸을 때를 대비). 단위는 raw 레지스터 값(서보 자체 스케일,
        # X 시리즈는 대략 1 unit ≈ 1mA 로 알려져 있으나 실측 검증된 값은 아니다).
        # ⚠️ 기본값은 DEFAULT_CURRENT_TRIP_THRESHOLD 참고 — 그리퍼 정상 파지
        # 중에도 트립되는 게 실기로 재현돼 상향 조정한 히스토리가 있다. 너무
        # 낮으면 정상 동작에서도 오탐 트립이 날 수 있다. 0 이하로 설정하면
        # 기능 자체가 꺼진다. 런타임 on/off·조정은 /dynamixel/current_trip_config
        # (아래 current_trip_config_callback) 참고.
        self.declare_parameter("current_trip_threshold", DEFAULT_CURRENT_TRIP_THRESHOLD)
        self.declare_parameter("current_trip_enabled", True)
        # 연속 읽기(1/read_rate_hz 간격) 사이 |전류| 변화량이 이보다 크면
        # "급변"으로 보고 **전 관절 비상정지**를 건다(2026-08-02 추가 —
        # current_trip_threshold 같은 절대값은 테스트 모터로는 "정상치"를 몰라
        # 의미가 약하다는 사용자 판단). 위 CURRENT_SPIKE_BIT 주석 참고.
        # 0 이하로 설정하면 기능 자체가 꺼진다.
        self.declare_parameter("current_spike_delta_threshold", DEFAULT_CURRENT_SPIKE_DELTA)
        # 급변 감지 기능 자체를 켜고 끈다(2026-08-02 추가 — 실서보로 바꾸니 기동
        # 가속 전류만으로 오탐이 나서, threshold 를 실기로 다시 잡는 동안 기능
        # 자체를 잠깐 끌 수 있게 해달라는 요청). 절대 트립(current_trip_threshold)
        # 은 이 스위치와 무관하게 항상 켜져 있다 — 이건 "급변(변화량)" 감지만
        # 끈다. 런타임에는 /dynamixel/current_spike_config 로 토글(아래
        # current_spike_config_callback 참고).
        # 🔁 **2026-08-19 기본값 True → False.** 실서보에서 오탐이 계속 났다 —
        # baseline 이 min(window) 라 정지 중이면 0 이 되고, 거기서 조그를 시작하면
        # **정상 기동 전류**가 그대로 Δ가 되어 문턱을 넘는다(ID 11 에서 Δ396 ≥ 350
        # 으로 전 관절 비상정지). 문턱을 실측으로 다시 잡기 전까지는 꺼두는 쪽이
        # 맞다 — 절대 트립(current_trip_threshold)이 백스톱으로 남는다.
        self.declare_parameter("current_spike_enabled", False)

        # 두 전류 가드(절대 트립 + 급변 감지)에서 **통째로 빼는 서보 ID 목록**.
        #
        # ⚠️ 문턱값이 모터 계열을 가리지 않는다는 게 근본 문제다. 주소 126 의 의미가
        #    계열마다 다르다 — XL430(그리퍼 3·4, 손목 16)은 `Present Load`(0.1% 단위),
        #    XM540(팔 11·12·13·14)은 `Present Current`(2.69mA 단위)다. 지금 문턱
        #    (500/350)은 XL430 기준으로 잡힌 값이라 XM540 축에 그대로 적용하면
        #    1A 남짓한 정상 구동 전류에도 걸린다(XM540-W270 정격 2A대, current_limit
        #    레지스터 2047 ≈ 5.5A).
        #
        # 기본값 [11] = arm_joint_1(베이스 요축). 2026-08-19 재조립으로 새로 편입된
        # XM540 축이고 실기에서 실제로 오탐 트립이 났다(사용자 요청으로 제외).
        # ⚠️ **ID 12/13/14 도 같은 XM540 이라 같은 오탐 소지가 있다** — 아직 안 걸린
        #    건 그 축들을 덜 움직여서일 가능성이 크다. 제대로 고치려면 계열별로
        #    문턱을 나눠야 하고(XM540=mA 도메인, XL430=% 도메인), 그러려면 각 축의
        #    정상 조그 전류 상한 실측이 필요하다. 그 전까지의 임시 조치다.
        self.declare_parameter("current_guard_exempt_ids", DEFAULT_CURRENT_GUARD_EXEMPT_IDS)

        port = self.get_parameter("port").value
        baudrate = int(self.get_parameter("baudrate").value)
        self.motor_ids = [int(v) for v in self.get_parameter("motor_ids").value]
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.profile_acc = int(self.get_parameter("profile_acceleration").value)
        self.profile_vel = int(self.get_parameter("profile_velocity").value)
        self.force_position_mode = bool(self.get_parameter("force_position_mode").value)
        self.extended_position_ids = {
            int(v) for v in self.get_parameter("extended_position_ids").value
        }
        read_rate = float(self.get_parameter("read_rate_hz").value)
        write_rate = float(self.get_parameter("write_rate_hz").value)
        temp_rate = float(self.get_parameter("temperature_poll_hz").value)
        self.current_trip_threshold = int(self.get_parameter("current_trip_threshold").value)
        self.current_trip_enabled = bool(self.get_parameter("current_trip_enabled").value)
        self.current_spike_delta_threshold = int(
            self.get_parameter("current_spike_delta_threshold").value)
        self.current_spike_enabled = bool(self.get_parameter("current_spike_enabled").value)
        self.current_guard_exempt_ids = set(
            int(v) for v in self.get_parameter("current_guard_exempt_ids").value)

        if len(self.motor_ids) != len(self.joint_names):
            raise RuntimeError(
                f"motor_ids({len(self.motor_ids)})와 "
                f"joint_names({len(self.joint_names)}) 길이가 다릅니다"
            )

        # ⚠️ 포트를 열기 **전에** 배타 잠금을 잡는다 — 이 노드와
        # moveit_dynamixel_bridge 가 같은 버스를 동시에 쓰면 "축 하나만 조용히
        # 빠지는" 형태로 망가진다(bus_lock 모듈 docstring 에 실기 사고 기록).
        # fd 는 인스턴스가 들고 있어야 한다 — GC 되면 잠금도 풀린다.
        self._bus_lock_fd = bus_lock.acquire(port, self.get_logger())

        latency = int(self.get_parameter("ftdi_latency_timer").value)
        if latency > 0:
            _set_ftdi_latency(port, latency, self.get_logger())

        # 포트/패킷 핸들러 생성
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # 포트 열기
        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port: {port}")
            raise RuntimeError("Failed to open Dynamixel port")

        # Baudrate 설정
        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().error(f"Failed to set baudrate: {baudrate}")
            raise RuntimeError("Failed to set baudrate")

        self.get_logger().info(f"Dynamixel port opened: {port} @ {baudrate}bps")

        # 응답하는 서보만 등록한다 — 빠진 서보 하나가 나머지 readback 을 망치지 않게.
        self.active = []
        for index, dxl_id in enumerate(self.motor_ids):
            if self._init_motor(dxl_id):
                self.active.append((dxl_id, self.joint_names[index]))

        if not self.active:
            raise RuntimeError("응답하는 서보가 없습니다 — 배선/전원을 확인하세요")

        # 개별 read/write 는 매 건마다 status 패킷을 기다린다. FTDI latency_timer
        # 까지 겹치면 6관절 × 4항목 = 24 왕복에 수백 ms 가 든다(실측 2.6Hz).
        # → 상태는 SyncRead 한 번, 목표는 SyncWrite 한 번으로 묶는다.
        self.sync_read = GroupSyncRead(
            self.port_handler, self.packet_handler, SYNC_READ_ADDR, SYNC_READ_LEN)
        for dxl_id, _name in self.active:
            if not self.sync_read.addParam(dxl_id):
                self.get_logger().warn(f"SyncRead addParam 실패 ID {dxl_id}")
        self.sync_write = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        # Hardware Error Status 는 126~135 블록과 안 붙어있어 별도 SyncRead 그룹.
        self.sync_read_err = GroupSyncRead(
            self.port_handler, self.packet_handler,
            ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS)
        for dxl_id, _name in self.active:
            if not self.sync_read_err.addParam(dxl_id):
                self.get_logger().warn(f"SyncRead(hw_error) addParam 실패 ID {dxl_id}")

        # goal_callback 은 시리얼을 만지지 않고 여기에 최신값만 남긴다.
        # 콜백에서 바로 쓰면 20Hz × 6관절 = 120 왕복/초가 읽기와 버스를 두고 다툰다.
        self.pending_goals = {}
        self.temperature = {dxl_id: 0 for dxl_id, _ in self.active}
        self._temp_cursor = 0
        # 한 번이라도 뜬 하드웨어 에러 비트마스크 — reboot 하기 전까지 유지(latch).
        self.error_latched = {}
        # id -> (min_tick, max_tick). teleop_core 가 /dynamixel/tick_limits 로
        # 채워준다(2026-08-02 추가) — teleop_core 가 계산한 관절 리밋(rad)을 이
        # 노드가 알아듣는 tick 단위로 미리 변환해서 보내주는 것. **teleop_core 를
        # 거치지 않고 누군가/무언가 /dynamixel/goal_position 에 직접 쏘는 경우에도**
        # 여기서 한 번 더 막아야 진짜 방어선이 된다 — goal_callback 의 0~4095 clamp는
        # "서보가 안 부서지는 물리적 전체 범위"일 뿐 "이 로봇의 이 관절이 실제로
        # 안전한 범위"가 아니다. 비어있으면(아직 안 받았거나 그 축이 리밋 없음)
        # 기존처럼 0~4095 전체 범위만 적용한다.
        self.tick_limits = {}
        # teleop_core 가 /dynamixel/tick_limits 로 **명시적으로 빈 목록**을
        # 보낸 적이 있는지(2026-08-02 추가 — 그리퍼를 다회전으로 바꾼 뒤 "리밋을
        # 걸지 말아달라"는 사용자 요청 대응). 다회전 축은 아래 goal_callback 에서
        # tick_limits 가 없으면 거부하는 게 원칙(무제한 다회전을 아무 정보 없이
        # 도는 게 위험해서)인데, 이러면 "limit off" 로 리밋을 명시적으로 풀어도
        # 다회전 축(그리퍼 등)만은 여전히 막힌다 — teleop_core 는 "limit off" 일
        # 때만 **완전히 빈** 배열을 보낸다(그 외엔 계산된 축이 하나라도 있으면
        # 비지 않음, __init__ 기동 시의 일상적인 재발행도 포함) — 그래서 "메시지를
        # 한 번이라도 받았는지"가 아니라 **"빈 메시지를 받은 적이 있는지"**로
        # 판단해야 한다. 그래야 이미 다른 축(arm_joint_2/3/4)이 캘리브레이션돼
        # 있는 정상 기동 시의 (비어있지 않은) 자동 재발행에는 걸리지 않고,
        # 사용자가 진짜 "limit off" 를 눌렀을 때만 다회전 축도 같이 풀린다.
        # ⚠️ 단, 어떤 축도 한 번도 캘리브레이션된 적 없는 완전히 새 시스템에서는
        # 정상 기동 시의 재발행도 (계산할 게 없어서) 똑같이 빈 배열이라 이 구분이
        # 안 먹는다 — 그런 새 시스템에서 다회전 축을 추가한다면 이 로직만으로는
        # 부족하니 실기 투입 전 재검토할 것.
        self.tick_limits_confirmed = False

        # id -> deque(최근 CURRENT_SPIKE_WINDOW 개의 |PRESENT_CURRENT|) —
        # current_spike_delta_threshold 급변 감지용 베이스라인(read_state 참고,
        # 위 CURRENT_SPIKE_WINDOW 주석도 참고). 토크 on/off 전환 시 비교 기준이
        # 무의미해지므로 그때마다 지운다(torque_request_callback/reboot_callback).
        self.current_history = {}
        # 급변으로 전 관절 비상정지가 이미 걸렸는지 — 반복 트리거 방지(토크를
        # 이미 다 껐으니 다음 읽기부터 전류가 떨어져 자연히 재트립은 안 되지만,
        # 혹시 몰라 명시적으로 막아둔다). resume/torque_request(enable=1) 시 해제.
        self.emergency_stopped = False

        self.get_logger().info(
            "구동 대상: "
            + ", ".join(f"{name}(ID {i})" for i, name in self.active)
        )

        # 위치 명령 구독
        # 메시지 형식: [모터ID, 목표위치]
        self.subscription = self.create_subscription(
            Int32MultiArray,
            "/dynamixel/goal_position",
            self.goal_callback,
            10,
        )

        # 모터 상태 publish
        # 데이터 형식: [id, position, velocity, current, temperature, ...]
        self.state_pub = self.create_publisher(
            Int32MultiArray,
            "/dynamixel/state",
            10,
        )

        # RViz / MoveIt용 joint_states publish
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        # 현재 latch 된 하드웨어 에러 목록. "joint_name(ID n):과부하" 콤마 구분,
        # 없으면 빈 문자열. teleop_core/키보드 TUI 가 그대로 구독해서 보여준다.
        self.error_pub = self.create_publisher(
            String, "/dynamixel/hardware_error", 10)

        # reboot 요청: Int32MultiArray 로 대상 ID 목록. 빈 배열이면 "현재 latch 된
        # 에러 전체" 로 해석한다(teleop_core._cmd_reboot 의 "reboot all").
        self.reboot_sub = self.create_subscription(
            Int32MultiArray, "/dynamixel/reboot_request", self.reboot_callback, 10)

        # 토크 on/off 요청: data = [enable(0|1), id, id, ...] (teleop_core 의
        # "자세 저장(freedrive)" 흐름 전용 — 2026-08-02 추가). enable=0 이면 손으로
        # 자세를 잡을 수 있게 토크만 끈다. enable=1 이면 torque_request_callback
        # 참고 — 켜기 *직전* 현재 위치를 goal_position 에 다시 써서 이전 목표로
        # 안 튀고 그 자리에서 홀드한다.
        self.torque_sub = self.create_subscription(
            Int32MultiArray, "/dynamixel/torque_request", self.torque_request_callback, 10)

        # Profile Velocity(112) 런타임 조정 요청: data = [velocity, id, id, ...]
        # (teleop_core 전용 — 2026-08-02 추가). "조그는 빠르게, 자세 불러오기
        # (goto)는 느리게" 처럼 같은 버스를 쓰는 동작별로 속도를 다르게 쓰기
        # 위함 — Profile Velocity 는 RAM 레지스터라 토크 끄지 않고도 즉시
        # 바뀐다(운영모드 EEPROM 과 다름). teleop_core_node.py 의
        # jog_profile_velocity/goto_profile_velocity 참고.
        self.profile_velocity_sub = self.create_subscription(
            Int32MultiArray, "/dynamixel/profile_velocity_request",
            self.profile_velocity_request_callback, 10)

        # 전류 급변 감지 런타임 설정: data = [enabled(0|1), threshold]. threshold
        # 가 0 이하면 "지금 값 유지, enabled 만 바꿈" 으로 해석한다(teleop_core
        # 의 "spike on"/"spike off"/"spike <threshold>" 명령 전용 — 2026-08-02
        # 추가, 위 current_spike_enabled 선언부 주석 참고).
        self.current_spike_config_sub = self.create_subscription(
            Int32MultiArray, "/dynamixel/current_spike_config",
            self.current_spike_config_callback, 10)

        # 절대 과전류 트립 런타임 설정: data = [enabled(0|1), threshold]. 형식/의미는
        # 위 current_spike_config 와 동일(teleop_core 의 "trip on"/"trip off"/
        # "trip <threshold>" 명령 전용 — 2026-08-02 추가, 그리퍼 정상 파지 중에도
        # 트립되는 게 실기로 재현돼 급변 감지와 같은 방식으로 런타임 조정 가능하게 함).
        self.current_trip_config_sub = self.create_subscription(
            Int32MultiArray, "/dynamixel/current_trip_config",
            self.current_trip_config_callback, 10)

        # 관절 리밋(tick 단위) — teleop_core 가 자기 joint_limits(rad)를 tick 으로
        # 변환해서 보내준다. data = [id, min_tick, max_tick, id, min_tick, max_tick, ...].
        # 매번 "지금 유효한 전체 목록"을 통째로 보내므로 여기서도 통째로 교체한다
        # (증분 아님). goal_callback 최종 방어선 — 위 tick_limits 선언부 주석 참고.
        self.tick_limits_sub = self.create_subscription(
            Int32MultiArray, "/dynamixel/tick_limits", self.tick_limits_callback,
            TICK_LIMITS_QOS)

        self.write_timer = self.create_timer(
            1.0 / max(1.0, write_rate), self.flush_goals)
        self.timer = self.create_timer(1.0 / max(0.1, read_rate), self.read_state)
        self.temp_timer = self.create_timer(
            1.0 / max(0.1, temp_rate), self.poll_temperature)

        self.get_logger().info(
            f"Dynamixel node started (read={read_rate}Hz, write={write_rate}Hz, "
            f"current_trip_threshold={self.current_trip_threshold}, "
            f"current_trip_enabled={self.current_trip_enabled}, "
            f"current_spike_delta_threshold={self.current_spike_delta_threshold}, "
            f"current_spike_enabled={self.current_spike_enabled}, "
            f"current_guard_exempt_ids={sorted(self.current_guard_exempt_ids)})")

    # ------------------------------------------------------------------ 기동
    def _comm_detail(self, result, error):
        """SDK 의 두 실패 축을 **구분해서** 문자열로 만든다.

        `result`(통신 계층: 응답 없음·패킷 깨짐)와 `error`(서보가 보낸 패킷 에러:
        Access Error 등)는 원인이 전혀 다르다 — 전자는 배선/버스 경합, 후자는
        "토크가 켜진 채 EEPROM 을 쓰려 했다" 같은 절차 문제다. 뭉뚱그리면 어느 쪽을
        고쳐야 하는지 알 수 없다.
        """
        parts = []
        if result != 0:
            parts.append(f"comm={self.packet_handler.getTxRxResult(result)}")
        if error != 0:
            parts.append(f"servo={self.packet_handler.getRxPacketError(error)}")
        return ", ".join(parts) or "원인 불명"

    def _init_motor(self, dxl_id):
        """서보 하나를 위치제어 + 프로파일 가감속 상태로 만들고 토크를 켠다.

        operating mode 는 EEPROM 이라 **토크 OFF 상태에서만** 써진다.
        반환값: 이 서보를 구동 대상으로 등록해도 되는가.
        """
        model, result, error = self.packet_handler.ping(self.port_handler, dxl_id)
        if result != 0:
            self.get_logger().warn(
                f"ID {dxl_id} 응답 없음 — 건너뜁니다 "
                f"({self.packet_handler.getTxRxResult(result)})"
            )
            return False

        # 모드 변경/EEPROM 쓰기를 위해 일단 토크를 내린다.
        # ⚠️ 결과를 반드시 확인한다 — 이 쓰기가 조용히 실패하면 토크가 켜진 채로
        # 남아서 다음 EEPROM 쓰기가 Access Error 로 거절되는데, 로그만 봐서는
        # "모드 변경 실패"로 보여 원인이 한 단계 가려진다.
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if result != 0 or error != 0:
            self.get_logger().error(
                f"ID {dxl_id} 토크 OFF 실패 — 건너뜁니다 "
                f"({self._comm_detail(result, error)})")
            return False

        if self.force_position_mode:
            target_mode = (
                OP_MODE_EXTENDED_POSITION if dxl_id in self.extended_position_ids
                else OP_MODE_POSITION
            )
            # ⚠️ 읽기의 result/error 를 반드시 본다. dynamixel_sdk 는 통신이 깨져도
            # 데이터 자리에 **0 을 돌려준다** — 그걸 실제 모드로 믿고 로그에 찍으면
            # "서보가 Current Control Mode(0) 에 있다"는 존재하지 않는 원인을
            # 가리키게 된다(2026-08-12 실기에서 실제로 오진을 유발했다).
            mode, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE)
            if result != 0 or error != 0:
                self.get_logger().error(
                    f"ID {dxl_id} operating mode 읽기 실패 — 건너뜁니다 "
                    f"({self._comm_detail(result, error)}). "
                    f"읽힌 값 {mode} 는 실제 모드가 아니라 통신 실패의 기본값입니다 "
                    "— 버스를 두 프로세스가 나눠 쓰고 있지 않은지 먼저 확인하세요")
                return False
            if mode != target_mode:
                result, error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, dxl_id,
                    ADDR_OPERATING_MODE, target_mode)
                if result != 0 or error != 0:
                    self.get_logger().error(
                        f"ID {dxl_id} operating mode 변경 실패 "
                        f"({mode} → {target_mode}) — 건너뜁니다 "
                        f"({self._comm_detail(result, error)})")
                    return False
                mode_label = "다회전" if target_mode == OP_MODE_EXTENDED_POSITION else "위치제어"
                self.get_logger().info(
                    f"ID {dxl_id} operating mode {mode} → {target_mode}({mode_label})")

        # Torque ON 전에 반드시 Present→Goal 동기화와 readback을 끝낸다.
        present, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if result != 0 or error != 0:
            self.get_logger().error(
                f"ID {dxl_id} startup Present Position 읽기 실패 — Torque ON 차단")
            return False
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_POSITION,
            present & 0xFFFFFFFF)
        if result != 0 or error != 0:
            self.get_logger().error(
                f"ID {dxl_id} startup Goal Position 동기화 실패 — Torque ON 차단")
            return False
        goal, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_POSITION)
        if result != 0 or error != 0 or goal != present:
            self.get_logger().error(
                f"ID {dxl_id} startup Goal readback 불일치 "
                f"(present={present}, goal={goal}) — Torque ON 차단")
            return False

        # 프로파일 가감속 — 동기화 검증 후, Torque ON 전에만 쓴다.
        self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PROFILE_ACCELERATION, self.profile_acc)
        self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PROFILE_VELOCITY, self.profile_vel)

        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if result != 0:
            self.get_logger().error(
                f"Torque enable failed ID {dxl_id}: "
                f"{self.packet_handler.getTxRxResult(result)}")
            return False
        if error != 0:
            self.get_logger().error(
                f"Torque enable error ID {dxl_id}: "
                f"{self.packet_handler.getRxPacketError(error)}")
            return False

        self.get_logger().info(
            f"Torque enabled : ID {dxl_id} (model={model}, "
            f"prof_acc={self.profile_acc}, prof_vel={self.profile_vel})")
        return True

    # ------------------------------------------------------------------ 명령
    def tick_limits_callback(self, msg):
        """teleop_core 가 계산한 관절 리밋(tick)을 통째로 교체 저장한다."""
        data = list(msg.data)
        if len(data) % 3 != 0:
            self.get_logger().error(
                f"tick_limits 형식 오류(3의 배수여야 함, 길이={len(data)}) — 무시")
            return
        limits = {}
        for i in range(0, len(data), 3):
            dxl_id, lo, hi = data[i], data[i + 1], data[i + 2]
            if lo > hi:
                lo, hi = hi, lo
            limits[int(dxl_id)] = (int(lo), int(hi))
        self.tick_limits = limits
        # "limit off" 로 명시적으로 빈 목록을 받은 경우에만 confirmed 로 latch —
        # 위 tick_limits_confirmed 선언부 주석 참고(정상 기동 시의 비어있지 않은
        # 재발행에는 안 걸려야 한다).
        if not limits:
            self.tick_limits_confirmed = True
        self.get_logger().info(f"tick_limits 갱신: {limits}")

    def goal_callback(self, msg):
        """목표 위치 명령을 받아 Dynamixel에 전송."""
        if len(msg.data) < 2:
            self.get_logger().error("Message must be [id, goal_position]")
            return

        dxl_id = int(msg.data[0])
        goal_position = int(msg.data[1])

        if dxl_id not in [i for i, _ in self.active]:
            self.get_logger().error(f"Unknown Dynamixel ID: {dxl_id}")
            return

        # 다회전(Extended Position) 축은 0~4095 clamp 가 의미 없다(여러 바퀴
        # 돌아야 하니까) — 그 대신 tick_limits(teleop_core 가 실측 캘리브레이션
        # 으로 계산해 보내주는 값)가 있어야 움직인다. 단, **teleop_core 와 한 번도
        # 얘기해본 적 없는(tick_limits_confirmed=False) 상태**에서만 거부한다 —
        # "얼마나 돌려도 안전한지 전혀 모르는 채로 다회전 서보를 돌리는" 진짜
        # 위험한 경우는 이때뿐이다. teleop_core 가 "limit off" 등으로 명시적으로
        # 빈 tick_limits 를 보낸 적이 있다면(tick_limits_confirmed=True) 그건
        # 사용자가 리밋을 걸지 않기로 결정한 것으로 신뢰하고 통과시킨다(2026-08-02
        # 추가 — 그리퍼를 다회전으로 바꾼 뒤 "리밋을 걸지 말아달라"는 사용자 요청
        # 대응. 위 tick_limits_confirmed 선언부 주석 참고).
        is_extended = dxl_id in self.extended_position_ids
        if is_extended and dxl_id not in self.tick_limits and not self.tick_limits_confirmed:
            self.get_logger().warn(
                f"ID {dxl_id} 는 다회전(Extended Position) 축인데 tick_limits 가 "
                "아직 없습니다 — teleop_core 에서 리미트 측정(calib_start) 을 "
                "하거나 'limit off' 로 명시적으로 리밋을 풀어주세요. 이 목표는 무시합니다."
            )
            return

        if is_extended:
            if dxl_id in self.tick_limits:
                lo, hi = self.tick_limits[dxl_id]
                clamped = max(lo, min(hi, goal_position))
            else:
                # 여기 도달했다는 건 위 거부 검사를 tick_limits_confirmed=True 로
                # 통과했다는 뜻(리밋 없이 쓰기로 명시적으로 동의한 상태) — clamp
                # 할 기준 자체가 없으니 그대로 통과시킨다.
                clamped = goal_position
        else:
            # 서보 물리 전체 범위(0~4095)로 먼저 clamp — 단일회전 축만 의미 있다.
            clamped = max(0, min(4095, goal_position))

            # ⚠️ 최종 방어선: teleop_core 가 알려준 관절별 안전 범위(tick_limits)로
            # 한 번 더 clamp. teleop_core 를 거치지 않고 /dynamixel/goal_position 에
            # 직접(버그·오조작·다른 노드 등으로) 쏜 값이어도 여기서 걸린다 — "브레인"
            # 하나만 믿지 않고 "손"(이 노드) 도 스스로 방어한다.
            if dxl_id in self.tick_limits:
                lo, hi = self.tick_limits[dxl_id]
                limited = max(lo, min(hi, clamped))
                if limited != clamped:
                    self.get_logger().warn(
                        f"ID {dxl_id} 목표({clamped})가 tick_limits[{lo},{hi}] 밖이라 "
                        f"{limited} 로 clamp 했습니다 — teleop_core 를 거치지 않은 "
                        "goal_position 이거나 소프트리밋 설정을 확인하세요"
                    )
                clamped = limited

        # 시리얼은 여기서 만지지 않는다 — flush_goals 가 SyncWrite 로 한 번에 보낸다.
        self.pending_goals[dxl_id] = clamped

    def flush_goals(self):
        """쌓인 목표를 SyncWrite 한 방으로 보낸다. 없으면 버스를 건드리지 않는다."""
        if not self.pending_goals:
            return

        goals = self.pending_goals
        self.pending_goals = {}

        self.sync_write.clearParam()
        for dxl_id, tick in goals.items():
            param = [
                tick & 0xFF,
                (tick >> 8) & 0xFF,
                (tick >> 16) & 0xFF,
                (tick >> 24) & 0xFF,
            ]
            if not self.sync_write.addParam(dxl_id, param):
                self.get_logger().warn(f"SyncWrite addParam 실패 ID {dxl_id}")

        result = self.sync_write.txPacket()
        if result != 0:
            self.get_logger().warn(
                f"SyncWrite 실패: {self.packet_handler.getTxRxResult(result)}")

    def poll_temperature(self):
        """온도(146)는 SyncRead 묶음(126~135) 밖이라 따로 읽는다.

        전부 매 주기 읽으면 왕복이 그만큼 늘어 텔레옵이 끊긴다 → 한 번에 하나씩.
        """
        if not self.active:
            return
        self._temp_cursor %= len(self.active)
        dxl_id = self.active[self._temp_cursor][0]
        self._temp_cursor += 1

        value, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_TEMPERATURE)
        if result == 0 and error == 0:
            self.temperature[dxl_id] = int(value)

    def _describe_hw_error(self, byte):
        names = [label for bit, label in HW_ERROR_BITS.items() if byte & (1 << bit)]
        return "|".join(names) if names else f"알수없음(0x{byte:02X})"

    def _publish_hardware_errors(self):
        parts = [
            f"{joint_name}(ID{dxl_id}):{self._describe_hw_error(self.error_latched[dxl_id])}"
            for dxl_id, joint_name in self.active
            if self.error_latched.get(dxl_id)
        ]
        self.error_pub.publish(String(data=",".join(parts)))

    def reboot_callback(self, msg):
        """특정 서보(또는 latch 된 에러 전체)를 reboot 하고 재초기화한다.

        Dynamixel reboot 는 EEPROM/RAM 을 공장 부팅 상태로 되돌리며 Hardware
        Error Status 를 지운다 — 과부하 트립 복구의 표준 절차. reboot 직후 서보가
        재부팅되는 동안(수백 ms) 통신이 안 되므로 짧게 대기한 뒤 torque/profile
        을 다시 건다(_init_motor 재사용).
        """
        ids = [int(v) for v in msg.data]
        if not ids:
            ids = [dxl_id for dxl_id, mask in self.error_latched.items() if mask]
            if not ids:
                self.get_logger().info("reboot 'all' 요청 — 현재 에러 latch 된 서보 없음")
                return

        active_ids = {dxl_id for dxl_id, _ in self.active}
        for dxl_id in ids:
            if dxl_id not in active_ids:
                self.get_logger().warn(f"reboot 요청 무시 — 등록 안 된 ID {dxl_id}")
                continue
            result, error = self.packet_handler.reboot(self.port_handler, dxl_id)
            if result != 0:
                self.get_logger().error(
                    f"ID {dxl_id} reboot 실패: {self.packet_handler.getTxRxResult(result)}")
                continue
            self.get_logger().info(f"ID {dxl_id} reboot 완료 — 재초기화 중")
            time.sleep(0.3)  # 서보 재부팅 대기(스펙상 수백 ms)
            if self._init_motor(dxl_id):
                self.error_latched.pop(dxl_id, None)
                # 재부팅으로 토크가 다시 걸렸으니(_init_motor) 급변 감지 베이스라인도
                # 새로 시작하고, 비상정지 래치도 전역으로 풀어준다(torque_request_
                # callback 의 enable=1 처리와 동일한 이유).
                self.current_history.pop(dxl_id, None)
                self.emergency_stopped = False
                self.get_logger().info(f"ID {dxl_id} 재초기화 완료 — 에러 해제")
            else:
                self.get_logger().error(f"ID {dxl_id} 재초기화 실패 — 배선/전원 확인")
        self._publish_hardware_errors()

    def torque_request_callback(self, msg):
        """토크 on/off 요청. data = [enable(0|1), id, id, ...].

        enable=0: 그냥 토크를 끈다 — 자세 저장(freedrive) 중 손으로 움직일 수 있게.
        enable=1: 토크를 켜기 **직전** 현재 위치(PRESENT_POSITION)를 GOAL_POSITION
        에 그대로 되써준다. 순서를 반대로 하면(그냥 토크만 켜면) 서보가 꺼지기
        전 마지막 goal_position 으로 되돌아가려 튄다 — freedrive 로 손으로 옮겨둔
        자세가 무의미해진다. pending_goals 에 남아있던 옛 목표도 같이 지워서,
        다음 flush_goals 가 그 옛 값으로 또 덮어쓰는 걸 막는다.
        """
        if len(msg.data) < 1:
            self.get_logger().error("torque_request: [enable, id...] 형식이어야 합니다")
            return

        enable = bool(int(msg.data[0]))
        ids = [int(v) for v in msg.data[1:]]
        active_ids = {dxl_id for dxl_id, _ in self.active}

        for dxl_id in ids:
            if dxl_id not in active_ids:
                self.get_logger().warn(f"torque_request 무시 — 등록 안 된 ID {dxl_id}")
                continue

            # 토크가 꺼졌다 켜지면 전류가 자연히 크게 뛰는 게 정상이라(꺼진 동안은
            # ~0) 급변 감지의 비교 기준이 무의미해진다 — 전환마다 지워서 다음
            # 읽기가 새 베이스라인으로 시작하게 한다(첫 읽기는 비교 없이 기록만).
            self.current_history.pop(dxl_id, None)

            if enable:
                pos, result, error = self.packet_handler.read4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
                if result != 0 or error != 0:
                    self.get_logger().error(
                        f"ID {dxl_id} 현재 위치 읽기 실패 — Torque ON 차단")
                    continue
                result, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION,
                    pos & 0xFFFFFFFF)
                if result != 0 or error != 0:
                    self.get_logger().error(
                        f"ID {dxl_id} Goal 동기화 실패 — Torque ON 차단")
                    continue
                goal, result, error = self.packet_handler.read4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION)
                if result != 0 or error != 0 or goal != pos:
                    self.get_logger().error(
                        f"ID {dxl_id} Goal readback 불일치 "
                        f"(present={pos}, goal={goal}) — Torque ON 차단")
                    continue
                self.pending_goals.pop(dxl_id, None)

            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id,
                ADDR_TORQUE_ENABLE, TORQUE_ENABLE if enable else TORQUE_DISABLE)
            if result != 0 or error != 0:
                self.get_logger().error(
                    f"ID {dxl_id} 토크 {'ON' if enable else 'OFF'} 실패: "
                    f"{self.packet_handler.getTxRxResult(result)}")
            else:
                self.get_logger().info(
                    f"ID {dxl_id} 토크 {'ON (현재 위치 홀드)' if enable else 'OFF (freedrive)'}")
                if enable:
                    # "resume" 등으로 뭔가가 다시 켜졌다는 건 사용자가 상황을
                    # 확인하고 재개했다는 뜻이라, 급변 비상정지 래치도 여기서
                    # 전역으로 풀어준다(다음 급변부터 다시 감지 가능해짐).
                    self.emergency_stopped = False
                    if dxl_id in self.error_latched:
                        # 전류 임계값/급변 트립(bit 6/7)은 우리가 직접 걸고 직접
                        # 푸는 소프트 값이라 — 실제 HW 에러 비트와 달리 reboot
                        # 없이도 여기서 확실하게 재활성화됐으니 latch 를 지운다.
                        # 진짜 HW 에러 비트는 그대로 남겨둔다(그건 여전히 reboot
                        # 로만 지워야 신뢰할 수 있음).
                        self.error_latched[dxl_id] &= ~(
                            (1 << CURRENT_TRIP_BIT)
                            | (1 << CURRENT_SPIKE_BIT))
                        if not self.error_latched[dxl_id]:
                            del self.error_latched[dxl_id]
        self._publish_hardware_errors()

    def profile_velocity_request_callback(self, msg):
        """Profile Velocity(112) 를 지정 ID들에 즉시 적용. data = [velocity, id, id, ...].

        RAM 레지스터라 토크를 끄지 않고도 바로 반영된다(EEPROM 인 operating
        mode 와 다름) — 진행 중인 이동에는 영향 없고, 그 다음 새 goal_position
        부터 이 속도로 움직인다.
        """
        if len(msg.data) < 1:
            self.get_logger().error(
                "profile_velocity_request: [velocity, id...] 형식이어야 합니다")
            return

        velocity = int(msg.data[0])
        ids = [int(v) for v in msg.data[1:]]
        active_ids = {dxl_id for dxl_id, _ in self.active}

        for dxl_id in ids:
            if dxl_id not in active_ids:
                self.get_logger().warn(
                    f"profile_velocity_request 무시 — 등록 안 된 ID {dxl_id}")
                continue
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, ADDR_PROFILE_VELOCITY, velocity)
            if result != 0 or error != 0:
                self.get_logger().error(
                    f"ID {dxl_id} profile_velocity={velocity} 설정 실패: "
                    f"{self.packet_handler.getTxRxResult(result)}")

    def current_spike_config_callback(self, msg):
        """전류 급변 감지 on/off·threshold 를 런타임에 바꾼다.

        data = [enabled(0|1), threshold]. threshold 가 0 이하면 "지금 값 유지"
        로 해석해 enabled 만 바꾼다 — teleop_core 의 "spike on"/"spike off" 는
        threshold 를 안 건드리고, "spike <숫자>" 는 값을 바꾸면서 자동으로
        enabled=1 도 같이 보낸다(teleop_core_node.py 의 on_cmd 참고).
        """
        if len(msg.data) < 2:
            self.get_logger().error(
                "current_spike_config: [enabled, threshold] 형식이어야 합니다")
            return
        enabled = bool(int(msg.data[0]))
        threshold = int(msg.data[1])
        self.current_spike_enabled = enabled
        if threshold > 0:
            self.current_spike_delta_threshold = threshold
        self.get_logger().info(
            f"전류 급변 감지: {'ON' if enabled else 'OFF'} "
            f"(threshold={self.current_spike_delta_threshold})")

    def current_trip_config_callback(self, msg):
        """절대 과전류 트립 on/off·threshold 를 런타임에 바꾼다.

        형식/의미는 current_spike_config_callback 과 동일 — teleop_core 의
        "trip on"/"trip off"/"trip <threshold>" 명령 전용.
        """
        if len(msg.data) < 2:
            self.get_logger().error(
                "current_trip_config: [enabled, threshold] 형식이어야 합니다")
            return
        enabled = bool(int(msg.data[0]))
        threshold = int(msg.data[1])
        self.current_trip_enabled = enabled
        if threshold > 0:
            self.current_trip_threshold = threshold
        self.get_logger().info(
            f"절대 과전류 트립: {'ON' if enabled else 'OFF'} "
            f"(threshold={self.current_trip_threshold})")

    def read_state(self):
        """전 서보의 현재 전류/속도/위치를 SyncRead 한 번으로 읽어서 publish."""
        result = self.sync_read.txRxPacket()
        if result != 0:
            self.get_logger().warn(
                f"SyncRead 실패: {self.packet_handler.getTxRxResult(result)}")
            return

        # Hardware Error Status — 별도 SyncRead. 실패해도 위치/속도 읽기는 계속한다.
        err_result = self.sync_read_err.txRxPacket()
        if err_result != 0:
            self.get_logger().warn(
                f"SyncRead(hw_error) 실패: {self.packet_handler.getTxRxResult(err_result)}")

        state_msg = Int32MultiArray()
        state_data = []

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        # 그리퍼처럼 이름 하나에 ID가 여럿(3,4) 매핑된 경우 JointState 에는
        # 이름당 한 번만 실어야 한다(중복 이름은 robot_state_publisher/TF 를 깨뜨림).
        # /dynamixel/state(Int32MultiArray) 쪽은 ID별 원자료라 dedupe 하지 않는다.
        published_names = set()

        for dxl_id, joint_name in self.active:
            if not self.sync_read.isAvailable(
                    dxl_id, SYNC_READ_ADDR, SYNC_READ_LEN):
                self.get_logger().warn(f"SyncRead 데이터 없음 ID {dxl_id}")
                continue

            current = _signed(
                self.sync_read.getData(dxl_id, ADDR_PRESENT_CURRENT, 2), 2)
            velocity = _signed(
                self.sync_read.getData(dxl_id, ADDR_PRESENT_VELOCITY, 4), 4)
            position = _signed(
                self.sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, 4), 4)

            # 전류 급증(리미트 측정 중 실제 기구적 끝단에 걸리는 등) 즉시 대응 —
            # 서보 자체의 과부하 보호(Hardware Error Status bit 5)보다 먼저/확실하게
            # 끊기 위한 소프트 안전장치. read_state 는 매 주기(기본 30Hz) 이 검사를
            # 하므로 다음 SyncWrite 를 기다리지 않고 바로 여기서 torque 를 끈다.
            if (self.current_trip_enabled
                    and dxl_id not in self.current_guard_exempt_ids
                    and self.current_trip_threshold > 0
                    and abs(current) >= self.current_trip_threshold
                    and not (self.error_latched.get(dxl_id, 0) & (1 << CURRENT_TRIP_BIT))):
                trip_result, _trip_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                self.error_latched[dxl_id] = (
                    self.error_latched.get(dxl_id, 0) | (1 << CURRENT_TRIP_BIT))
                # ⚠️ 2026-08-02 추가: 트립된 서보의 GOAL_POSITION 을 **지금 위치로**
                # 즉시 되돌린다(+ 아직 안 나간 pending_goals 도 지운다) — 안 그러면
                # 트립 원인이었던 옛(무리한) 목표가 그대로 레지스터에 남아있다가,
                # 나중에 resume/reboot 로 토크가 돌아오는 순간 다시 그 목표를
                # 향해 밀어붙여 즉시 재트립되는 걸 사용자가 실기로 재현했다("풀어도
                # 다시 오므리려다 또 걸림"). 목표를 트립 지점으로 맞춰두면 재개
                # 직후엔 그 자리서 홀드만 하고, 그 다음은(더 열든 더 닫든) 새로
                # 들어오는 조그 명령을 그대로 따른다.
                self.pending_goals.pop(dxl_id, None)
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION, position & 0xFFFFFFFF)
                self.get_logger().error(
                    f"전류 임계값 초과 ID {dxl_id}({joint_name}): current={current} "
                    f"(threshold={self.current_trip_threshold}) — 토크 즉시 해제 "
                    f"({self.packet_handler.getTxRxResult(trip_result)}), 목표를 "
                    f"현재 위치({position})로 되돌림 — "
                    "reboot 전까지 화면에 계속 표시됩니다"
                )

            # 전류 급변 감지 → 전 관절 비상정지(2026-08-02 추가/재조정, 위
            # CURRENT_SPIKE_BIT/CURRENT_SPIKE_WINDOW 주석의 4~5차 히스토리 참고) —
            # 속도 게이팅(정지/주행 구분) 없이 **단일 연속 윈도우**로 최근
            # CURRENT_SPIKE_WINDOW(~3초) 개의 |전류| 최솟값을 기준선으로 삼는다.
            # current_spike_enabled=False 면 창을 아예 안 쌓는다 — 다시 켰을 때
            # 옛(다른 상태에서 쌓인) 값이 기준선으로 섞이지 않게, torque on/off
            # 전환과 같은 방식으로 첫 샘플은 비교 없이 기록만 하고 시작한다.
            if (not self.current_spike_enabled
                    or dxl_id in self.current_guard_exempt_ids):
                self.current_history.pop(dxl_id, None)
            else:
                window = self.current_history.setdefault(
                    dxl_id, deque(maxlen=CURRENT_SPIKE_WINDOW))
                baseline = min(window) if window else None
                window.append(abs(current))
                if (baseline is not None
                        and self.current_spike_delta_threshold > 0
                        and not self.emergency_stopped):
                    delta = abs(current) - baseline  # 부호 무시, "더 힘쓰는 쪽"만
                    if delta >= self.current_spike_delta_threshold:
                        self._trigger_emergency_stop(
                            dxl_id, joint_name, current, baseline, delta)

            # raw position 0~4095를 radian으로 근사 변환
            # 2048을 중앙, 한 바퀴를 2pi로 가정
            rad = (position - 2048) * (2.0 * math.pi / 4096.0)

            state_data.extend([
                int(dxl_id),
                int(position),
                int(velocity),
                int(current),
                int(self.temperature.get(dxl_id, 0)),
            ])

            if joint_name not in published_names:
                joint_msg.name.append(joint_name)
                joint_msg.position.append(rad)
                joint_msg.velocity.append(float(velocity))
                joint_msg.effort.append(float(current))
                published_names.add(joint_name)

            if err_result == 0 and self.sync_read_err.isAvailable(
                    dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS):
                err_byte = self.sync_read_err.getData(
                    dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS)
                if err_byte:
                    newly_set = err_byte & ~self.error_latched.get(dxl_id, 0)
                    if newly_set:
                        self.get_logger().error(
                            f"하드웨어 에러 감지 ID {dxl_id}({joint_name}): "
                            f"{self._describe_hw_error(err_byte)} (raw=0x{err_byte:02X}) — "
                            "reboot 전까지 화면에 계속 표시됩니다")
                    self.error_latched[dxl_id] = self.error_latched.get(dxl_id, 0) | err_byte

        state_msg.data = state_data

        self.state_pub.publish(state_msg)
        self._publish_hardware_errors()
        self.joint_state_pub.publish(joint_msg)

    # ------------------------------------------------------------------ 종료
    def disable_all_torque(self):
        """전 서보 토크를 즉시 끈다. Ctrl-C 등으로 노드가 죽을 때 마지막 목표
        위치를 계속 붙들고 있지 않도록 종료 경로(main 의 finally)에서 부른다.
        전류 급변 비상정지(_trigger_emergency_stop)에서도 노드가 계속 살아있는
        채로 재사용한다."""
        for dxl_id, joint_name in self.active:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if result != 0 or error != 0:
                self.get_logger().error(
                    f"ID {dxl_id}({joint_name}) 토크 OFF 실패: "
                    f"{self.packet_handler.getTxRxResult(result)}")
            else:
                self.get_logger().info(f"ID {dxl_id}({joint_name}) 토크 OFF")

    def _trigger_emergency_stop(self, dxl_id, joint_name, current, baseline, delta):
        """전류 급변 감지 → 전 관절 비상정지. read_state 에서만 호출한다.

        한 관절만 끊지 않고 disable_all_torque() 로 **전부** 끈다 — 사용자
        요청("비상정지 + 토크 해제")이자, 급변은 팔 전체가 위험한 자세일 수
        있다는 신호라 한 관절만 끄는 걸로는 부족하다는 판단(위
        CURRENT_SPIKE_BIT 주석 참고). baseline 은 직전 한 번의 값이 아니라
        최근 CURRENT_SPIKE_WINDOW 개 창의 최솟값이다.
        """
        self.emergency_stopped = True
        self.error_latched[dxl_id] = (
            self.error_latched.get(dxl_id, 0) | (1 << CURRENT_SPIKE_BIT))
        self.get_logger().error(
            f"⚠ 전류 급변 감지 ID {dxl_id}({joint_name}): |current| 최근 최소 "
            f"{baseline} → {abs(current)} (Δ{delta} ≥ threshold "
            f"{self.current_spike_delta_threshold}) — 전 관절 비상정지(토크 해제)"
        )
        self.disable_all_torque()
        self._publish_hardware_errors()


def main(args=None):
    rclpy.init(args=args)

    # 버스 경합은 traceback 이 아니라 **읽히는 한 줄**로 죽어야 한다 — 이 실패는
    # 사용자가 곧바로 조치할 수 있는 종류이고(다른 노드를 내린다), 파이썬 스택은
    # 그 조치를 오히려 가린다.
    try:
        node = DynamixelPositionNode()
    except bus_lock.BusInUseError as exc:
        print(f"[dynamixel_position_node] 기동 거부: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 포트를 닫기 전에 토크부터 끈다 — 순서가 바뀌면 마지막 goal_position 을
        # 붙든 채로 서보가 계속 힘을 준다(Ctrl-C 로 팔이 안 풀리는 사고 방지).
        node.disable_all_torque()
        node.port_handler.closePort()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
