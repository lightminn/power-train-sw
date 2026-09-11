#!/usr/bin/env python3

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray, String
from control_msgs.action import FollowJointTrajectory
from robot_arm_msgs.action import ArmRecordedPath, ArmTestMove, EndEffectorRotate
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead
from ament_index_python.packages import get_package_share_directory

from dynamixel_control.tool_manager import (
    BusToolIdentityProvider, ParameterToolIdentityProvider, ToolManager)
from dynamixel_control.tool_fsm.cleaner_fsm import CleanerFSM
from dynamixel_control.spur_manual_control import SpurManualControl
from dynamixel_control.tool_profiles import (
    load_profiles, ToolProfileError, validate_control_scope)
from dynamixel_control import calib_math
from dynamixel_control import joint_limits
from dynamixel_control import bus_lock
from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset
from dynamixel_control.calibration_session import CalibrationSession
from dynamixel_control.dual_manual_recovery import (
    DualManualRecovery, DualManualRecoveryError)
from dynamixel_control.dual_calibration_session import (
    DualCalibrationSession, DualCalibrationError)
from dynamixel_control.tool_fsm.base import ToolState

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
#: Hardware Error Status(70) 의 Overload 비트. 파지 중 토크가 끊기는 주범이다.
HWERR_OVERLOAD = 0x20
MODE_POSITION = 3           # 단일회전 0~4095
MODE_EXTENDED_POSITION = 4  # 다회전, tick 이 범위를 넘고 음수도 된다
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_PWM = 100
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146

LEN_GOAL_POSITION = 4
LEN_GOAL_VELOCITY = 4
LEN_HARDWARE_ERROR_STATUS = 1
LEN_PRESENT_LOAD = 2
LEN_PRESENT_POSITION = 4

# X-시리즈(XL430/XC430/XM 공통) Present Velocity 데이터시트 고정값: signed, 1 LSB = 0.229 rev/min.
# PRESENT_VELOCITY 는 이미 아래 SyncRead 범위(70~135) 안에 있어 버스 트랜잭션 추가 없이
# 파싱만 하면 된다 — 그동안 버려지던 바이트를 꺼내 쓰는 것뿐(Notion "그리퍼 tick/
# wrist_to_gripper/PRESENT_VELOCITY 실측·검증 절차" §2-3).
VELOCITY_LSB_TO_RAD_S = 0.229 * 2.0 * math.pi / 60.0

# HARDWARE_ERROR_STATUS(70,1) ~ PRESENT_POSITION(132,4) 은 X-시리즈 컨트롤 테이블에서
# 연속 주소 범위라, 70부터 66바이트를 한 번의 SyncRead 로 받아 fault/load/position 을
# 함께 추출(버스 트랜잭션 1회). 중간의 다른 필드(Profile Accel/Velocity 등)도 같이
# 읽히지만 안 쓰고 버림 — 주소가 연속이기만 하면 여분을 읽는 건 무해함.
# (XL430/XC430/XM 계열 공통. 다른 모델이면 주소 재확인 필요 — CLAUDE.md §8 모터모델 미확정.)
ADDR_SYNC_READ_START = ADDR_HARDWARE_ERROR_STATUS
LEN_SYNC_READ = (ADDR_PRESENT_POSITION + LEN_PRESENT_POSITION) - ADDR_HARDWARE_ERROR_STATUS  # = 66

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
# A freshly reattached gripper can rest a few ticks beyond its saved endpoint
# while torque is off.  That observed position is still safer than an old goal
# register; accept only this narrow tolerance when synchronising before enable.
ENABLE_POSITION_TOLERANCE_TICKS = 32

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyUSB0"

# tick 상수와 캘리브 측정식의 단일 출처는 calib_math 다(ROS 비의존 → pytest 로 고정).
# 여기서 재노출하는 이유는 `scripts/measure_*.py` 와 외부 코드가 예전부터 이 모듈에서
# 가져다 쓰고 있어서다 — import 경로를 깨지 않으면서 정의는 한 곳으로 모은다.
DXL_MINIMUM_POSITION_VALUE = calib_math.DXL_MINIMUM_POSITION_VALUE
DXL_MAXIMUM_POSITION_VALUE = calib_math.DXL_MAXIMUM_POSITION_VALUE
DXL_CENTER_POSITION = calib_math.DXL_CENTER_POSITION

TICKS_PER_RAD = calib_math.TICKS_PER_RAD
DXL_TICKS_PER_REV = calib_math.DXL_TICKS_PER_REV  # Present Velocity 환산용


# 팔 관절 ↔ 다이나믹셀 ID 매핑.
#
# 2026-08-07 실기 버스 스캔에 맞춰 갱신. 그 전까지 이 dict 는 id 0/1/2 를 가리켰는데
# **버스에 존재하지 않는 ID** 라 팔 서보 토크 인가가 전부 실패했다. 값의 출처는
# teleop_core_node.py 의 DEFAULT_MOTOR_IDS/DEFAULT_DIRECTIONS(2026-08-01/08-02 벤치
# 실측) 이며, 이 두 파일은 같은 물리 버스를 가리키므로 **한쪽을 바꾸면 다른 쪽도
# 같이 바꿔야 한다.**
#
# ⚠️ arm_joint_1(ID 11, 베이스 요축)은 **모터가 물리적으로 없다**(2026-08-07 사용자
#    확인). 스캔에도 안 잡혀서 아예 등록하지 않는다 — 등록해두면 매 tick 무응답으로
#    fault 가 서게 된다. 모터가 붙으면 여기에 한 줄 추가하면 된다.
#
# `gear_ratio` = 서보축 회전 / 관절 회전. 1.0 이면 직결.
#
# arm_joint_2/3 값은 2026-08-07 `scripts/measure_gear_ratio.py` 로 실측했다(토크를
# 끄고 관절을 손으로 돌려 서보각 변화 / 관절각 변화). **두 축의 감속비는 서로 다르다**
# (사용자 확인) — 그 전까지 teleop_core_node.py 주석이 두 축을 묶어 "약 10:1" 로
# 추정했지만 실측 결과 arm_joint_2 만 10:1 에 가깝고 arm_joint_3 은 절반 이하다.
#
# ⚠️ 실측 정밀도는 관절각을 얼마나 정확히 쟀는지에 달려 있다(90° 를 ±9° 오차로 재면
#    기어비도 약 ±10% 흔들린다). 파지 위치가 계통적으로 어긋나면 이 값부터 의심할 것.
#    재측정 없이 시험할 땐 `gear_ratios` 파라미터로 덮어쓸 수 있다.
#    `center`(영점)도 같은 방식으로 `centers` 파라미터가 덮어쓴다
#    (`scripts/measure_zero_offset.py` 또는 관제 GUI 의 영점 마법사 결과를 바로 시험).
#
# 🔒 **모를 때는 낮은 값을 쓴다.** 기어비를 실제보다 낮게 잡으면 관절이 명령보다 덜
#    움직여(언더슈트) 안전하지만, 높게 잡으면 그 배수만큼 과주행해 구조물을 때린다.
#
# `extended` = Extended Position Control Mode(다회전) 축. tick 이 0~4095 를 넘어가고
# 음수도 나오므로 부호 있는 정수로 해석해야 한다(teleop_core 의 EXTENDED_POSITION_NAMES 와 짝).
#
# `center` = 관절 0도에 해당하는 tick. 2026-08-07 `scripts/measure_zero_offset.py` 로
# URDF home 자세(전 관절 0도)에서 실측했다. 그 전까지는 전 축 2048(서보 중앙값)이라는
# **검증된 적 없는 가정**이었고, 실제로 축마다 최대 1100 tick(≈97°) 어긋나 있었다.
# 기어비와 마찬가지로 영점이 틀리면 IK 결과가 통째로 그만큼 어긋난다.
# ⚠️ 팔을 분해·재조립하거나 서보를 뿔에서 뺐다 끼우면 이 값은 무효다 — 다시 측정할 것.
#
# 🔁 **2026-08-09 재측정.** `extended` 축(arm_joint_2/3)의 다회전 카운트는 전원을 내리면
#    초기화되므로, 한 바퀴(4096) 밖의 center 는 그걸 잰 전원 세션 안에서만 유효하다.
#    실제로 arm_joint_3 의 구 center=4281 은 전원 사이클 후 관절각을 -1.58 rad 로
#    읽게 만들었다(안전범위는 0~2.034) — 그대로 구동하면 틀린 기준점 위에서 +90°
#    스윙한다. 아래 값은 그래서 다시 잰 것이다.
#      교차검증: 순수 카운트 초기화라면 새 center 는 4281-4096=185 여야 하는데 278 이
#      나왔다(관절 2.0° 차) — 즉 -87° 편차는 전부 카운트 초기화分이고 자세 재현
#      오차는 2° 수준이었다는 뜻.
#
# ⚠️ **직결(1:1) 축인 arm_joint_4/5 는 토크를 끄면 중력으로 흘러내린다** — 감속기 축
#    (9:1, 4:1)은 역구동이 안 돼 손으로 세운 자세가 유지되지만 이 둘은 손을 떼면 처진다.
#    영점 측정은 반드시 **팔을 붙잡은 상태에서** 할 것.
#
# 🔁 **2026-08-19 3차 재측정 (현재 값).** 팔 전체 재조립 → 전 축 영점 무효.
#    같은 회차에 arm_joint_1(ID 11, XM540-W270)이 새로 실장된 것도 확인됐다.
#    ⚠️ extended 축(arm_joint_2/3)의 아래 center 는 **이 전원 세션 안에서만** 유효하다.
#
# 🔁 **2026-08-09 2차 재측정** (이력). 1차 측정 후 실기 구동 중 손목(arm_joint_5)
#    결합이 물리적으로 빠져 재조립했고, 서보 전원도 내려갔다 — 둘 다 영점 무효 사유다.
#    재조립 후 측정에서는 붙잡은 상태의 드리프트가 8초간 arm_joint_2/5 **정확히 0.000°**
#    로 나왔다.
#    ⚠️ 1차 측정 때 arm_joint_5 가 붙잡고 있는데도 +1.5°/s 로 미끄러졌던 것은 중력이
#       아니라 **결합이 이미 헐거웠다는 신호**였다(수리 후 그 드리프트가 완전히 사라진
#       것으로 확인). 어떤 축이 "잡아도 계속 흐르면" 측정을 계속하지 말고 결합부터
#       점검할 것 — 그대로 두면 구동 중 빠진다.
JOINT_CONFIG = {
    # 🆕 **2026-08-19 신설.** 이 축은 그동안 "모터가 물리적으로 없다"는 전제로
    # STATIC_JOINTS 에 0.0 고정 발행돼 있었는데, 재조립 후 버스 스캔에서 ID 11
    # (XM540-W270)이 정상 응답했다. 사용자 확인 결과 기구에도 물려 있다.
    # gear_ratio 1.0 = **감속기 없는 직결**이다(2026-08-19 사용자 확인) — 측정으로
    # 나온 값이 아니라 기구가 그렇다. center 는 같은 날 zero_offset 실측값.
    "arm_joint_1": {"id": 11, "center": 2081, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
    # 2026-08-07 실측: 9.034:1 (관절 90° 회전 기준)
    "arm_joint_2": {"id": 14, "center": 506, "direction": -1,
                    "gear_ratio": 9.034, "extended": True},
    # 2026-08-07 실측: 4.040:1 — arm_joint_2 와 다른 감속기다(오타 아님)
    "arm_joint_3": {"id": 13, "center": 1855, "direction": 1,
                    "gear_ratio": 4.040, "extended": True},
    # 🔁 **2026-08-19 2차 정정: center 1184 → 1573.**
    # zero_offset 실측(1184)으로는 이 축이 **34.2° 어긋나 있었다** — 발행값이 0 일 때
    # 실물은 URDF -35° 자세였다(사용자 육안 대조). 영점을 잴 때 팔을 손으로 home 에
    # 놓는데, 이 축은 그 판단이 그만큼 빗나가 있었던 것으로 보인다.
    # ⚠️ 아래 리밋(joint_limits.py)도 같은 도메인에서 잰 값이라 **함께 이동**시켰다.
    #    tick 자체는 안 바뀌므로 teleop_core 의 서보축 리밋은 손댈 필요가 없다
    #    (그쪽은 center 가 아니라 tick 2048 에 앵커돼 있다).
    "arm_joint_4": {"id": 12, "center": 1573, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
    "arm_joint_5": {"id": 16, "center": 675, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
}
ARM_IDS = {config["id"] for config in JOINT_CONFIG.values()}
ARM_ID_SEQUENCE = [config["id"] for config in JOINT_CONFIG.values()]
ARM_TEST_SEQUENCE = ((14, 5), (13, 10), (12, 10), (16, 20))
RANDOM_ARM_RANGES = {14: 20, 13: 40, 12: 40, 16: 80}
RECORDED_PATH_IDS = (14, 13, 12)
RECORDED_PATH_START_TOLERANCE = {14: 20, 13: 30, 12: 20}
RECORDED_PATH_MAX_WAYPOINT_STEP = 50

# 모터가 없어 실측할 수 없지만 **URDF 상으로는 존재하는** 관절 — 고정값으로 발행한다.
#
# arm_joint_1(베이스 요축)은 서보가 물리적으로 없다(2026-08-07). 그런데 URDF 에서는
# `link_002 → link_004` 를 잇는 관절이라, /joint_states 에 값이 없으면
# `robot_state_publisher` 가 이 관절을 못 넘어가 **TF 트리가 두 조각으로 갈린다**
# ("Tf has two or more unconnected trees") → `base_link → link_043`(tip) 변환이 아예
# 안 만들어져서 arm_fsm 의 IK/carry pose 계산이 전부 실패한다. MoveIt 도 5축 전체
# 관절값을 기대하므로 같은 이유로 필요하다.
#
# 값의 근거:
#   arm_joint_2/3/4 는 축이 서로 평행해서 이 팔은 **평면 로봇**이고, 그 평면의 방위를
#   정하는 유일한 관절이 arm_joint_1 이다(축 0 0 1, 회전중심은 base_link 원점 위
#   z=0.0465). 즉 이 값이 틀리면 팔이 향하는 방향 전체가 틀린다.
#
# 🔁 **2026-08-12 정정: 1.405 → 0.0.**
#   이 값은 그동안 +1.405 rad 였다. 근거는 "0.0 이면 FK 가 그리퍼를 방위각 -80.5°
#   (거의 정오른쪽)에 놓는데, 실기의 팔은 정면(+x)을 향하므로 방위각을 0 으로 돌려야
#   한다" 는 것이었다 — 그 **전제가 틀렸다. 팔은 실제로 오른쪽으로 틀어져 있다**
#   (2026-08-12 사용자 확인).
#
#   확인 방법(같은 방식으로 재확인 가능): 카메라 TF 캘리브를 마친 상태에서 박스를
#   그리퍼 정면에 놓고 RViz 를 본다. joint_1=0 인 모델의 tip(link_043)은 방위각
#   -80.7°/반경 15.5cm 에 서고, 카메라가 본 박스는 -90.7°/46.1cm 에 찍혔다 —
#   **두 방향이 10° 안쪽으로 일치**했고 화면상으로도 박스가 그리퍼 앞이었다.
#   1.405 를 쓰면 이 관계가 통째로 80° 어긋난다.
#
#   ⚠️ 그 오차는 조용하다: 브릿지가 떠 있는 동안에만 TF 가 80° 돌아가므로, RViz 만
#      띄워 보면(jsp_gui 가 0 을 발행) 멀쩡해 보이고 arm_fsm 을 붙였을 때만 목표가
#      틀어진다. "인식·캘리브는 맞는데 팔이 엉뚱한 데로 간다" 면 여기를 볼 것.
#
# ⚠️ 이 축은 모터가 없다. **기구적으로 고정돼 있다는 전제**이며, 만약 자유회전
#    상태라면 팔의 평면이 운용 중 돌아가고 이 값은 무의미해진다 — 그 경우 IK 목표가
#    조용히 틀어지므로, 물리적으로 고정돼 있는지 반드시 확인할 것.
#    팔을 재장착했다면 위 확인 절차를 다시 밟을 것.
# 🔁 **2026-08-19 비움.** arm_joint_1 은 ID 11 서보가 실재하는 것이 확인돼
#    JOINT_CONFIG 로 옮겨졌다(위 참고). 이제 실측값이 /joint_states 로 나가므로
#    고정 발행이 필요 없다 — 고정값을 남겨두면 실제 서보각과 충돌한다.
#    아래 dict 와 발행 경로는 다음에 모터 없는 축이 생길 때를 위해 남겨둔다.
STATIC_JOINTS = {}

# X 시리즈 Extended Position Control Mode 의 raw tick 한계(약 ±256회전).
DXL_EXTENDED_MIN_TICK = calib_math.DXL_EXTENDED_MIN_TICK
DXL_EXTENDED_MAX_TICK = calib_math.DXL_EXTENDED_MAX_TICK


#: 캘리브 파라미터의 "비어 있음" 기본값.
#:
#: ⚠️ **`[]` 을 쓰면 안 된다.** rclpy 는 빈 리스트에서 타입을 추론하지 못해
#: `BYTE_ARRAY` 로 선언해 버리고, 그러면 런타임 `set_parameters` 가
#: *"Wrong parameter type, expected 'Type.BYTE_ARRAY' got 'Type.STRING_ARRAY'"* 로
#: **거절된다**(2026-08-12 실기 확인). CLI `-p gear_ratios:=` 는 선언 시점에 값을
#: 덮어써서 멀쩡히 동작하므로, **런타임에 처음 바꿔 볼 때까지 드러나지 않는다.**
#: `ParameterDescriptor(type=...)` 로도 추론을 못 바꾼다 — 빈 문자열 하나가 답이다.
#: 파서가 이름 없는 항목을 건너뛰므로 의미상으로는 "없음" 그대로다.
EMPTY_STR_ARRAY = [""]


# 캘리브 파라미터(`gear_ratios`·`centers`)의 파서. rclpy 에 dict 타입 파라미터가 없어
# "<joint>:<값>" 문자열 배열로 받는다. 기동 시와 런타임 변경(파라미터 콜백)이 **같은
# 검증**을 쓰도록 함수로 뺐다 — 한쪽만 느슨하면 기동은 되는데 변경은 거절되는 식이 된다.
def _parse_gear_ratios(entries):
    """`["arm_joint_2:9.034", …]` → `({이름: 비}, [오류 사유])`."""
    out, errors = {}, []
    for entry in entries or []:
        name, _, value = str(entry).partition(":")
        if not name:
            continue                       # 빈 문자열은 "없음" 으로 본다(기본값 [""])
        if name not in JOINT_CONFIG:
            errors.append(f"모르는 관절 '{name}'")
            continue
        try:
            ratio = float(value)
        except ValueError:
            errors.append(f"'{entry}' 파싱 실패")
            continue
        if ratio <= 0.0:
            errors.append(f"'{entry}' 은 양수여야 함")
            continue
        out[name] = ratio
    return out, errors


def _parse_centers(entries):
    """`["arm_joint_2:1627", …]` → `({이름: tick}, [오류 사유])`."""
    out, errors = {}, []
    for entry in entries or []:
        name, _, value = str(entry).partition(":")
        if not name:
            continue
        if name not in JOINT_CONFIG:
            errors.append(f"모르는 관절 '{name}'")
            continue
        try:
            center = int(round(float(value)))
        except ValueError:
            errors.append(f"'{entry}' 파싱 실패")
            continue
        reason = calib_math.center_out_of_range(center, JOINT_CONFIG[name]["extended"])
        if reason is not None:
            errors.append(f"{name}: {reason}")
            continue
        out[name] = center
    return out, errors


# Profile Acceleration(108) / Velocity(112). 기본값 0 은 "최고속 즉시 이동" 이라
# 그리퍼가 움직일 때마다 순간 과전류로 토크가 풀린다(HW-8 실기, 재현율 100%,
# 명령 후 0.3초 내 트립). 트립이 풀리면 Hardware Error Status 도 0 으로 돌아가
# 나중에 보면 흔적이 안 남는다 — 반드시 기동 시 넣어야 한다(CLAUDE.md).
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
PROFILE_ACCELERATION = 25
PROFILE_VELOCITY = 80


def to_signed(value, byte_len):
    """무부호 정수를 byte_len 바이트 2의 보수 부호 정수로 변환."""
    bits = byte_len * 8
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value


class MoveItDynamixelBridge(Node):
    def __init__(self):
        super().__init__("moveit_dynamixel_bridge")

        # --- 그리퍼 파라미터 (랙피니언 2모터 동일방향 구동, ID 3/4) ---
        # gripper_type 이 gripper_presets.GRIPPER_PRESETS 의 기본값을 고르고,
        # 아래 개별 파라미터는 필요 시 CLI/런치로 여전히 개별 오버라이드 가능.
        self.declare_parameter("gripper_type", DEFAULT_GRIPPER)
        self.gripper_type = self.get_parameter("gripper_type").value
        preset = get_preset(self.gripper_type, self.get_logger())

        self.declare_parameter("gripper_joints", preset["gripper_joints"])
        self.declare_parameter("gripper_ids", preset["gripper_ids"])  # 빈 배열이면 그리퍼 비활성
        self.declare_parameter("gripper_open_rad", preset["gripper_open_rad"])
        self.declare_parameter("gripper_close_rad", preset["gripper_close_rad"])
        self.declare_parameter("gripper_open_tick", preset["gripper_open_tick"])
        self.declare_parameter("gripper_close_tick", preset["gripper_close_tick"])
        # 다회전 그리퍼 여부. preset 에 없으면 단일회전으로 본다(보수적 — 다회전을
        # 잘못 켜면 tick 이 wrap 없이 계속 나가 랙 끝단을 밀어붙인다).
        self.declare_parameter("gripper_extended", bool(preset.get("extended", False)))
        # 0 이면 쓰지 않는다(서보 기본 885=100% 유지). preset 주석에 값 근거 있음.
        self.declare_parameter("gripper_goal_pwm", int(preset.get("gripper_goal_pwm", 0)))
        # 캘리브 범위 밖으로 미끄러진 그리퍼 자동 복구(_recover_gripper_range 참고).
        # 종료 시 토크가 풀리면 그리퍼가 닫힘 끝단을 지나쳐 미끄러지는데, 그 상태에서는
        # gripper_goal_pwm 의 힘으로 못 빠져나온다 — 재기동마다 재발하므로 기본 활성.
        # 모션 프로파일. 단위는 데이터시트 기준 Profile Velocity = 0.229 rev/min,
        # Profile Acceleration = 214.577 rev/min^2.
        # ⚠️ 팔 속도는 **여기서만** 정해진다(_write_motion_profile 주석 참고).
        #    2026-08-12: 40(절반)으로 낮췄다가 **80 으로 되돌렸다.** 느리게 하면
        #    arm_fsm 의 모션 완료 판정과 어긋난다 — `_publish_joint_trajectory` 가
        #    `arm_move_speed`(0.5 rad/s)로 duration 을 추정해 그만큼만 기다리는데,
        #    서보가 그보다 느려지면 **도착 전에 다음 상태로 넘어간다**(하강 도중
        #    파지 등). 속도를 정말 낮추려면 arm_fsm 의 `arm_move_speed` 를 같은
        #    비율로 낮춰 둘을 함께 맞춰야 한다.
        self.declare_parameter("arm_profile_velocity", 80)
        self.declare_parameter("gripper_profile_velocity", 80)
        self.declare_parameter("profile_acceleration", 25)
        self.declare_parameter("gripper_auto_recover", True)
        # 파지 중 Overload(HW error 0x20) 트립이 나면 **REBOOT 로 되살리고 파지를 다시
        # 건다**(2026-08-19 사용자 지시: "꽉 잡고 Overload 나면 재부팅해서 그 상태 유지").
        #
        # ⚠️ **이건 파지를 '유지' 하는 게 아니다.** 트립하는 순간 토크가 끊기므로 재부팅이
        #    끝나기 전에 화물은 이미 떨어진다. 이 기능의 값어치는 "그리퍼가 죽은 채로
        #    미션이 끝나는 것"을 막는 것이지, 화물을 붙잡아 두는 게 아니다.
        # ⚠️ **Overload 를 반복해서 때리면 서보가 상한다.** 그 보호는 코일이 타는 걸
        #    막으려고 있는 것이다. 그래서 무한 재부팅을 하지 않고
        #    `gripper_overload_max_reboots` 회를 넘기면 포기하고 크게 알린다.
        # ⚠️ **REBOOT 은 RAM 레지스터를 전부 날린다** — 특히 Goal PWM 이 885(무제한)로
        #    돌아간다. 되살린 뒤 반드시 다시 써야 하며(_recover_gripper_overload), 안 하면
        #    다음 파지는 885 로 물어 3.5초 만에 또 트립하는 악순환이 된다.
        self.declare_parameter("gripper_overload_reboot", True)
        self.declare_parameter("gripper_overload_max_reboots", 3)
        self.declare_parameter("gripper_overload_window_s", 60.0)
        # REBOOT 후 서보가 다시 응답하기까지 기다리는 시간 [s].
        self.declare_parameter("gripper_reboot_settle_s", 1.0)
        # ⚠️ 885(최대)로 두지 말 것. 2026-08-12 에 885 로 열림 끝단까지 밀어붙였다가
        # **랙이 피니언에서 미끄러진** 것으로 보인다(직후 재캘리브에서 오프셋이 통째로
        # ~1880 tick 이동). 실측상 500 이면 범위 밖에서 끌어내는 데 충분하다
        # (PWM 500 으로 -938 → -434 를 1초). 끝단을 때리지 않는 것이 더 중요하다.
        self.declare_parameter("gripper_recover_pwm", 500)
        self.declare_parameter("gripper_recover_timeout", 6.0)
        self.declare_parameter("read_only", False)
        self.declare_parameter("mock_mode", False)
        self.declare_parameter("tool_type", "spur_1motor_gripper")
        # Bench-only single-side mode for a mechanically linked dual gripper.
        # 0 keeps normal dual operation; 3 or 4 makes the other actuator
        # torque-off before accepting any OPEN/CLOSE command.
        self.declare_parameter("dual_single_axis_id", 0)
        self.declare_parameter("auto_tool_detection", True)
        self.declare_parameter("tool_detection_confirmations", 2)
        self.declare_parameter("tool_detection_period_s", 0.5)
        self.declare_parameter("control_scope", "FULL_ROBOT")
        # Explicit, ID5-only bench path.  It bypasses GUI/FSM ownership
        # ceremony, never the hardware-health, E-stop, detached, range, or
        # read-only gates enforced by SpurManualControl.
        self.declare_parameter("developer_direct_mode", False)
        self.declare_parameter("temporary_jog_mode", False)
        self.declare_parameter("temporary_jog_safe_min_tick", 2867)
        self.declare_parameter("temporary_jog_safe_max_tick", 3807)
        self.declare_parameter("temporary_jog_mechanical_open_tick", 2817)
        self.declare_parameter("temporary_jog_mechanical_close_tick", 3857)
        # 1모터 스퍼기어 벤치 검증 전용의 보수적 프로파일이다. 토크는 여기서
        # 절대 켜지 않으며, GUI의 명시적 Enable 요청으로만 인가된다.
        self.declare_parameter("temporary_jog_profile_velocity", 5)
        self.declare_parameter("temporary_jog_profile_acceleration", 1)
        # ID5 endpoint calibration is deliberately separate from the old
        # temporary range: it never configures a register at startup and it
        # has no invented endpoint values.
        self.declare_parameter("calibration_jog_mode", False)
        self.declare_parameter("calibration_max_jog_ticks", 12)
        self.declare_parameter("gripper_target_tolerance_ticks", 20)
        default_profiles = str(Path(get_package_share_directory(
            'dynamixel_control')) / 'config' / 'tool_profiles.yaml')
        self.declare_parameter("tool_profile_file", default_profiles)
        # 털털이 ZIP에 모터 ID/방향/속도가 없어 모두 fail-closed 기본값이다.
        self.declare_parameter("cleaning_actuator_joint", "")
        self.declare_parameter("cleaning_actuator_id", -1)
        self.declare_parameter("cleaning_direction", 0)
        self.declare_parameter("cleaning_velocity_raw", 0)

        self.declare_parameter('centers', EMPTY_STR_ARRAY)
        self.declare_parameter('gear_ratios', EMPTY_STR_ARRAY)
        self.centers, errors = _parse_centers(
            self.get_parameter('centers').value)
        for reason in errors:
            self.get_logger().warn(f"centers: {reason} — 무시")
        self.gear_ratios, errors = _parse_gear_ratios(
            self.get_parameter('gear_ratios').value)
        for reason in errors:
            self.get_logger().warn(f"gear_ratios: {reason} — 무시")
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.gripper_joints = list(self.get_parameter("gripper_joints").value)
        self.gripper_ids = list(self.get_parameter("gripper_ids").value)
        self.gripper_open_rad = float(self.get_parameter("gripper_open_rad").value)
        self.gripper_close_rad = float(self.get_parameter("gripper_close_rad").value)
        self.gripper_open_tick = int(self.get_parameter("gripper_open_tick").value)
        self.gripper_close_tick = int(self.get_parameter("gripper_close_tick").value)
        self.gripper_extended = bool(self.get_parameter("gripper_extended").value)
        self.gripper_goal_pwm = int(self.get_parameter("gripper_goal_pwm").value)
        self.arm_profile_velocity = int(
            self.get_parameter("arm_profile_velocity").value)
        self.gripper_profile_velocity = int(
            self.get_parameter("gripper_profile_velocity").value)
        self.profile_acceleration = int(
            self.get_parameter("profile_acceleration").value)
        self.gripper_auto_recover = bool(
            self.get_parameter("gripper_auto_recover").value)
        self.gripper_overload_reboot = bool(
            self.get_parameter("gripper_overload_reboot").value)
        self.gripper_overload_max_reboots = int(
            self.get_parameter("gripper_overload_max_reboots").value)
        self.gripper_overload_window_s = float(
            self.get_parameter("gripper_overload_window_s").value)
        self.gripper_reboot_settle_s = float(
            self.get_parameter("gripper_reboot_settle_s").value)
        # Overload 재부팅 상태 — 마지막 그리퍼 goal tick(재부팅 후 되걸기용),
        # 최근 재부팅 시각들(창 안 횟수 제한용), 복구 중 플래그.
        self._last_gripper_goal_tick = None
        self._gripper_reboot_times = []
        self._gripper_recovering = False
        self._gripper_overload_gave_up = False
        self.gripper_recover_pwm = int(
            self.get_parameter("gripper_recover_pwm").value)
        self.gripper_recover_timeout = float(
            self.get_parameter("gripper_recover_timeout").value)
        self.read_only = bool(self.get_parameter("read_only").value)
        self.mock_mode = bool(self.get_parameter("mock_mode").value)
        self.tool_type = str(self.get_parameter("tool_type").value)
        self.auto_tool_detection = bool(
            self.get_parameter('auto_tool_detection').value)
        self.tool_detection_confirmations = int(
            self.get_parameter('tool_detection_confirmations').value)
        self.tool_detection_period_s = float(
            self.get_parameter('tool_detection_period_s').value)
        if self.tool_detection_confirmations < 1:
            raise ValueError('tool_detection_confirmations must be positive')
        if self.tool_detection_period_s <= 0.0:
            raise ValueError('tool_detection_period_s must be positive')
        self._tool_change_lock = threading.Lock()
        self._tool_change_pending = None
        self._tool_change_error = ''
        self._physical_tool_detached = False
        self._tool_detection_observation = None
        self._tool_detection_count = 0
        self._tool_detection_reason = ''
        self._arm_fsm_state = None
        self.control_scope = validate_control_scope(
            self.get_parameter("control_scope").value)
        self.dual_single_axis_id = int(
            self.get_parameter('dual_single_axis_id').value)
        if self.dual_single_axis_id not in (0, 3, 4):
            raise ValueError('dual_single_axis_id must be 0, 3, or 4')
        self.developer_direct_mode = bool(
            self.get_parameter('developer_direct_mode').value)
        if self.developer_direct_mode and (
                self.control_scope != 'END_EFFECTOR_ONLY'
                or self.tool_type != 'spur_1motor_gripper'):
            raise ValueError(
                'developer_direct_mode requires spur_1motor_gripper '
                'with END_EFFECTOR_ONLY scope')
        # The isolated end-effector stack must never poll or command arm IDs.
        self.gripper_only_mode = self.control_scope == 'END_EFFECTOR_ONLY'
        self.temporary_jog_mode = bool(
            self.get_parameter("temporary_jog_mode").value)
        self.temporary_jog_safe_min = int(
            self.get_parameter("temporary_jog_safe_min_tick").value)
        self.temporary_jog_safe_max = int(
            self.get_parameter("temporary_jog_safe_max_tick").value)
        self.temporary_jog_enabled = bool(
            self.temporary_jog_mode
            and self.control_scope == 'END_EFFECTOR_ONLY'
            and self.tool_type == 'spur_1motor_gripper'
            and self.temporary_jog_safe_min < self.temporary_jog_safe_max)
        self.temporary_jog_profile_velocity = int(
            self.get_parameter("temporary_jog_profile_velocity").value)
        self.temporary_jog_profile_acceleration = int(
            self.get_parameter("temporary_jog_profile_acceleration").value)
        self.calibration_jog_mode = bool(
            self.get_parameter("calibration_jog_mode").value)
        self.calibration_max_jog_ticks = int(
            self.get_parameter("calibration_max_jog_ticks").value)
        self.calibration_jog_enabled = bool(
            self.calibration_jog_mode and self.tool_type == 'spur_1motor_gripper'
            and self.control_scope == 'END_EFFECTOR_ONLY'
            and self.calibration_max_jog_ticks in (6, 12))
        self.calibration_endpoints = {}
        self.gripper_target_tolerance = int(
            self.get_parameter("gripper_target_tolerance_ticks").value)
        if self.gripper_target_tolerance < 0:
            raise ValueError('gripper_target_tolerance_ticks must be non-negative')
        try:
            profiles = load_profiles(
                self.get_parameter('tool_profile_file').value)
            self._tool_profiles = profiles
            self.tool_manager = ToolManager(
                profiles, ParameterToolIdentityProvider(self.tool_type),
                mock_mode=self.mock_mode)
            self.tool_selection = self.tool_manager.refresh('IDLE')
            self.tool_profile = self.tool_selection.profile
            self.tool_motion_allowed = self.tool_selection.valid
        except ToolProfileError as exc:
            self.get_logger().error(f'tool profile rejected: {exc}')
            self.tool_manager = None
            self.tool_selection = None
            self.tool_profile = {}
            self.tool_motion_allowed = False
            self._tool_profiles = {}
        if self.tool_profile.get('mock_only') and not self.mock_mode:
            raise RuntimeError('mock_only profiles cannot open physical hardware')
        self.control_mode = 'FSM'
        self.emergency_stop_active = False
        self.tool_detached = False
        self._tool_samples = {}
        self._gripper_goal_active = False
        self._gripper_goal_lock = threading.Lock()
        self._bus_lock = threading.RLock()
        self.cleaning_actuator_joint = self.get_parameter("cleaning_actuator_joint").value
        self.cleaning_actuator_id = int(self.get_parameter("cleaning_actuator_id").value)
        self.cleaning_direction = int(self.get_parameter("cleaning_direction").value)
        self.cleaning_velocity_raw = int(self.get_parameter("cleaning_velocity_raw").value)
        if self.tool_type == 'cleaner' and self.tool_profile.get('actuator_ids'):
            ids = self.tool_profile['actuator_ids']
            joints = self.tool_profile.get('joint_names', [])
            if len(ids) != 1 or len(joints) != 1 or ids[0] in ARM_IDS:
                raise ToolProfileError('cleaner requires one non-arm actuator and one joint')
            self.cleaning_actuator_id = ids[0]
            self.cleaning_actuator_joint = joints[0]
            self.cleaning_direction = int(self.tool_profile.get('direction') or 0)
            self.cleaning_velocity_raw = int(self.tool_profile.get('profile_velocity') or 0)
        self.cleaning_running = False
        self.cleaning_configured = (
            bool(self.cleaning_actuator_joint) and self.cleaning_actuator_id >= 0
            and self.cleaning_direction in (-1, 1) and self.cleaning_velocity_raw > 0
        )
        unregistered = [n for n in JOINT_CONFIG if joint_limits.get_limits(n) is None]
        if unregistered:
            self.get_logger().warn(
                f"joint_limits 에 없는 축 {unregistered} — **리밋 없이 그대로 나간다.** "
                "joint_limits.py 에 추가할 것."
            )
        provisional = [n for n in joint_limits.provisional_joints() if n in JOINT_CONFIG]
        if provisional:
            self.get_logger().warn(
                f"관절 {provisional} 은 가동범위 실측이 없어 보수적으로 좁혀둔 상태다"
                f"(±{joint_limits.PROVISIONAL_HALF_RANGE} rad). 이 축이 거의 안 움직이면 "
                "리밋 탓이다 — scripts/measure_joint_limits.py 로 실측할 것."
            )
        # ⚠️ provisional 과 **위험 방향이 반대**라 따로 띄운다. 저쪽은 좁아서 축을 덜
        # 쓰는 것(최악이 "조금밖에 안 돎")이고, 이쪽은 실측 스톱보다 넓혀둔 것이라
        # 최악이 **하드스톱 충돌**이다. 같은 문장으로 뭉치면 심각도가 뒤바뀐다.
        asserted = [n for n in joint_limits.user_asserted_joints() if n in JOINT_CONFIG]
        if asserted:
            spans = ', '.join(
                f"{n}=[{joint_limits.get_limits(n)[0]:+.4f}, {joint_limits.get_limits(n)[1]:+.4f}]"
                for n in asserted)
            self.get_logger().warn(
                f"관절 {asserted} 의 리밋은 **실측 하드스톱보다 넓혀둔 사용자 확인 값**이다 "
                f"({spans}). 스윕 실측으로 확정된 값이 아니므로 이 축이 끝까지 갈 때 "
                "기구가 부딪히지 않는지 눈으로 확인하면서 쓰고, "
                "scripts/measure_joint_limits.py 로 재측정해 확정할 것."
            )

        # ⚠️ 포트를 열기 **전에** 배타 잠금. 이 브릿지와 position_node 는 같은
        # /dev/ttyUSB0 을 잡으므로 "동시에 띄우지 말 것"이 계약인데, 지금까지는
        # 규율로만 지켜졌고 어기면 축 하나만 조용히 빠지는 형태로 망가졌다
        # (bus_lock 모듈 docstring 참고). fd 는 살려둬야 잠금이 유지된다.
        self._bus_lock_fd = None if self.mock_mode else bus_lock.acquire(
            DEVICENAME, self.get_logger())

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self.port_connected = self.mock_mode
        if not self.mock_mode:
            try:
                self.port_connected = bool(self.port_handler.openPort())
                if self.port_connected:
                    self.port_connected = bool(
                        self.port_handler.setBaudRate(BAUDRATE))
            except Exception as exc:
                self.port_connected = False
                self.get_logger().error(f'Cannot open {DEVICENAME}: {exc}')
            if not self.port_connected and not self.read_only:
                raise RuntimeError(f"Failed to open/configure port: {DEVICENAME}")

        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        # hardware error+address 126 feedback+position 블록을 한 번에 읽는 SyncRead
        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_SYNC_READ_START,
            LEN_SYNC_READ,
        )

        # SyncRead 등록 ID와 이 프로세스가 토크를 켠 ID를 별도로 추적한다.
        # gripper-only/read-only에서는 register write 없이 그리퍼 ID만 active_ids에 등록된다.
        self.active_ids = set()
        self.torque_enabled_ids = set()
        self._mock_arm_positions = {name: 0.0 for name in JOINT_CONFIG}
        if self.control_scope == 'FULL_ROBOT' and (self.mock_mode or self.read_only):
            for config in JOINT_CONFIG.values():
                self.active_ids.add(config['id'])
                if not self.mock_mode and self.port_connected:
                    self.group_sync_read.addParam(config['id'])

        if not self.read_only and not self.mock_mode:
            if self.control_scope == 'FULL_ROBOT':
                # 팔 서보: 토크 ON 성공한 ID만 SyncRead 등록
                for joint_name, config in JOINT_CONFIG.items():
                    if self._enable_torque(config["id"], joint_name):
                        self.group_sync_read.addParam(config["id"])
                        self.active_ids.add(config["id"])
            if self.cleaning_configured:
                self._configure_cleaning_actuator()

        self.tool_ids = list(self.tool_profile.get('actuator_ids', []))
        self.tool_discovered = self.mock_mode
        if self.mock_mode:
            self.active_ids.update(self.tool_ids)
            # Provide a deterministic, in-range feedback sample so the GUI can
            # capture its zero/reference without touching a serial device.
            joint_names = self.tool_profile.get('joint_names') or ['']
            for dxl_id in self.tool_ids:
                endpoints = self.tool_profile.get('motor_endpoints') or {}
                endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
                seed_tick = (int(round((endpoint['open'] + endpoint['close']) / 2))
                             if endpoint else 3320)
                self._tool_samples[dxl_id] = {
                    'id': dxl_id, 'joint': joint_names[0],
                    'position': seed_tick, 'effort': 0.0, 'online': True,
                    'hardware_error': 0, 'torque_state': 'OFF'}
        if not self.mock_mode and self.port_connected:
            self.tool_discovered = self._discover_tool_ids()
            if self.read_only:
                for dxl_id in self.tool_ids:
                    self.group_sync_read.addParam(dxl_id)
                    self.active_ids.add(dxl_id)
            elif self.tool_type == 'spur_1motor_gripper' and self.tool_ids == [5]:
                # A spur startup is observation-only.  It must never rewrite
                # torque, operating mode, profile, or a goal before an
                # operator explicitly uses the calibration/FSM controls.
                self.group_sync_read.addParam(5)
                self.active_ids.add(5)
            elif self.tool_type == 'dual_motor_gripper' and self.tool_ids == [3, 4]:
                # Preserve the dual FollowJointTrajectory architecture while
                # making startup observation-only.  Torque/mode/profile writes
                # are an explicit operator action, never a GUI launch side
                # effect.
                for dxl_id in self.tool_ids:
                    self.group_sync_read.addParam(dxl_id)
                    self.active_ids.add(dxl_id)
            elif self.calibration_jog_enabled:
                # Calibration observes the pre-existing hardware state first.
                # In particular, do not disable torque or rewrite mode/profile.
                self.group_sync_read.addParam(5)
                self.active_ids.add(5)
            elif self.tool_motion_allowed and self.tool_discovered:
                if self.temporary_jog_enabled:
                    self._configure_temporary_jog_actuator()
                else:
                    self._configure_tool_actuators()
            elif self.tool_ids and not self.tool_discovered:
                self.tool_motion_allowed = False

        # These are command adapters, not action clients.  In particular, the
        # spur path never re-enters legacy FollowJointTrajectory policy.
        self.tool_fsm = None
        self.calibration_session = None
        self.dual_manual_recovery = None
        self.dual_calibration_session = None
        if self.tool_type in ('spur_1motor_gripper', 'dual_motor_gripper') \
                and self.tool_manager:
            self.tool_fsm = self.tool_manager.create_fsm(self)
            self.tool_fsm.startup()
            # Startup validation is read-only.  Preserve its actual ID5
            # observations for status even when the regular SyncRead path is
            # unavailable, rather than substituting torque ownership cache.
            snapshot = getattr(self.tool_fsm, 'snapshot', None)
            if self.tool_type == 'spur_1motor_gripper' and isinstance(snapshot, dict):
                self._tool_samples[5] = {
                    'id': 5,
                    'joint': (self.tool_profile.get('joint_names') or [''])[0],
                    'position': snapshot.get('position'),
                    'effort': None,
                    'online': True,
                    'torque_state': ('ON' if snapshot.get('torque') == 1 else 'OFF'),
                    'hardware_error': snapshot.get('hardware_error'),
                    'model': snapshot.get('model'),
                }
            if self.tool_type == 'spur_1motor_gripper':
                self.calibration_session = CalibrationSession(
                    self, self.tool_profile)
        if self.tool_type == 'dual_motor_gripper':
            self.dual_manual_recovery = DualManualRecovery(self)
            self.dual_calibration_session = DualCalibrationSession(
                self, self.tool_profile)

        if self.tool_type == 'cleaner':
            self.tool_fsm = CleanerFSM(self.tool_profile, self)
            self.tool_fsm.startup()

        self.spur_manual_control = SpurManualControl(self)
        self.create_timer(0.1, self._spur_manual_watchdog)

        # Serial feedback can take longer than its 50 ms timer period on real
        # U2D2 hardware.  Keep operator/safety commands in their own callback
        # group so a continuously-ready feedback timer cannot starve them.
        self._command_group = MutuallyExclusiveCallbackGroup()
        # Bench cleaner START/STOP is deliberately latency-sensitive.  It has
        # its own executor group and still serializes actual bus access with
        # _bus_lock, so feedback cannot corrupt a packet while policy/FSM work
        # in the normal command queue cannot delay the operator's button.
        self._cleaner_direct_group = ReentrantCallbackGroup()
        # Latest-command-wins mailbox for the bench cleaner.  A new button
        # press never waits behind older LEFT/RIGHT requests; the active
        # writer drains only the newest requested velocity after its current
        # single serial transaction completes.
        self._cleaner_command_state_lock = threading.Lock()
        self._cleaner_command_active = False
        self._cleaner_command_generation = 0
        self._cleaner_requested_velocity = 0

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.trajectory_callback,
            10,
            callback_group=self._command_group,
        )
        self.create_subscription(
            Bool, "/cleaning/enable", self._on_cleaning_enable, 10,
            callback_group=self._cleaner_direct_group)
        self.create_subscription(
            String, '/cleaning/direction', self._on_cleaning_direction, 10,
            callback_group=self._cleaner_direct_group)
        self.create_subscription(
            Bool, "/tool/emergency_stop", self._on_emergency_stop, 10,
            callback_group=self._command_group)
        self.create_subscription(
            Bool, "/tool/detached", self._on_tool_detached, 10,
            callback_group=self._command_group)
        self.create_subscription(
            String, "/control/mode_status", self._on_control_mode, 10,
            callback_group=self._command_group)
        self.create_subscription(
            String, "/control/mode", self._on_control_mode_request, 10,
            callback_group=self._command_group)
        self.create_subscription(
            String, '/fsm/state', self._on_arm_fsm_state, 10,
            callback_group=self._command_group)

        # 벤치 teleop_core의 단일 관절 명령. 메시지는 [motor_id, goal_tick].
        # FSM/MoveIt 경로와 같은 GroupSyncWrite를 사용하되 알려진 팔 ID만 허용한다.
        # 토크 on/off 요청 — `position_node` 와 **같은 토픽·같은 포맷**을 쓴다
        # (`[enable, id...]`). 벤치 텔레옵 쪽에만 있던 인터페이스라 브릿지 경로에서는
        # 팔을 손으로 만지려면 스택을 통째로 내리는 수밖에 없었다. 어휘를 새로 만들지
        # 않고 기존 것을 그대로 받아, `teleop_core` 의 stop/freedrive 나 관제 GUI 버튼이
        # 어느 런타임에서든 같은 뜻을 갖게 한다.
        # 확장 한 가지: id 목록을 생략하면(`[enable]`) **등록된 전 축**에 적용한다 —
        # 요청자가 서보 ID 를 몰라도 되게 하기 위함이다(mission_console 이 이걸 쓴다).
        self.torque_request_sub = self.create_subscription(
            Int32MultiArray, "/dynamixel/torque_request",
            self.torque_request_callback, 10,
            callback_group=self._command_group)
        self.fsm_command_sub = self.create_subscription(
            String, '/tool/fsm_command', self.fsm_command_callback, 10,
            callback_group=self._command_group)
        self.tool_change_sub = self.create_subscription(
            String, '/tool/change', self.tool_change_callback, 10,
            callback_group=self._command_group)
        self.calibration_command_sub = self.create_subscription(
            String, '/tool/calibration_command',
            self.calibration_command_callback, 10,
            callback_group=self._command_group)
        self.manual_recovery_sub = self.create_subscription(
            String, '/tool/manual_recovery_jog',
            self.manual_recovery_callback, 10,
            callback_group=self._command_group)
        self.dual_calibration_command_sub = self.create_subscription(
            String, '/tool/dual_calibration_command',
            self.dual_calibration_command_callback, 10,
            callback_group=self._command_group)

        self.teleop_goal_sub = self.create_subscription(
            Int32MultiArray,
            "/dynamixel/goal_position",
            self.teleop_goal_callback,
            10,
            callback_group=self._command_group,
        )

        self._action_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.gripper_action_server = ActionServer(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_gripper,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.gripper_cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        if (
            not self.mock_mode
            and not self.calibration_jog_enabled
            and self.control_scope != 'END_EFFECTOR_ONLY'
        ):
            self.rotate_action_server = ActionServer(
                self, EndEffectorRotate, "/end_effector/rotate",
                execute_callback=self.execute_rotate,
                goal_callback=self.rotate_goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self._action_group)
            self.arm_test_action_server = ActionServer(
                self, ArmTestMove, "/arm/test_move",
                execute_callback=self.execute_arm_test_move,
                goal_callback=self.arm_test_goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self._action_group)
            self.arm_recorded_path_action_server = ActionServer(
                self, ArmRecordedPath, "/arm/recorded_path",
                execute_callback=self.execute_arm_recorded_path,
                goal_callback=self.arm_recorded_path_goal_callback,
                cancel_callback=self.cancel_callback,
                callback_group=self._action_group)

        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )
        # MoveIt/robot_state_publisher의 로봇 모델은 launch 시점에 고정된다.
        # 런타임에 교체된 도구 조인트를 /joint_states에 섞으면 모델에
        # 없는 이름으로 move_group이 종료할 수 있다. 도구 피드백은 별도 토픽과
        # /tool/status로 보존하고, /joint_states는 검증된 팔 모델만 유지한다.
        self.tool_joint_state_pub = self.create_publisher(
            JointState,
            "/tool/joint_states",
            10,
        )
        self.tool_type_pub = self.create_publisher(String, '/tool/type', 10)
        self.tool_status_pub = self.create_publisher(String, '/tool/status', 10)
        self.control_mode_status_pub = self.create_publisher(
            String, '/control/mode_status', 10)

        # 계약 §5.1 "locked heartbeat는 ... controller fault 0 ... 을 실제 확인한다" 대응.
        # arm_fsm 이 CARRYING_LOCKED/STOWED_LOCKED 발행 전 게이트로 구독(내부용 — 파워트레인
        # 쪽 DDS 경계를 넘지 않음, robot_arm_msgs 계약과 무관).
        self.fault_pub = self.create_publisher(
            Bool,
            "/dynamixel/controller_fault",
            10,
        )
        # 그리퍼 Overload 재부팅 복구 중임을 알린다(내부용, DDS 계약과 무관).
        # arm_fsm 이 이 구간의 effort 붕괴를 DROP 으로 오판해 GRIP_LOST 를 래치하지
        # 않도록 게이트로 쓴다 — 복구 중엔 토크가 끊겨 effort 가 당연히 0 이다.
        self.gripper_recovering_pub = self.create_publisher(
            Bool,
            "/dynamixel/gripper_recovering",
            10,
        )

        self.feedback_timer = self.create_timer(0.05, self.publish_joint_states)
        self.tool_status_timer = self.create_timer(
            0.5, self._publish_tool_status_safely,
            callback_group=ReentrantCallbackGroup())
        self._bus_tool_identity = None
        if self.auto_tool_detection and not self.mock_mode and self.port_connected:
            self._bus_tool_identity = BusToolIdentityProvider(
                self._tool_profiles, self._probe_tool_id,
                excluded_ids={config['id'] for config in JOINT_CONFIG.values()})
            self.create_timer(
                self.tool_detection_period_s, self._poll_physical_tool,
                callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(
            f"MoveIt Dynamixel bridge started (arm={list(JOINT_CONFIG)}, "
            f"cleaning_actuator={self.cleaning_actuator_joint or 'UNCONFIGURED'}, "
            f"tool_type={self.tool_type}, tool_ready={self.tool_motion_allowed}, "
            f"control_scope={self.control_scope}, "
            f"temporary_jog={self.temporary_jog_enabled}, "
            f"read_only={self.read_only}, mock_mode={self.mock_mode})"
        )

    # ------------------------------------------------------------------ helpers
    def _write_motion_profile(self, dxl_id, label, velocity=None):
        """Profile Acceleration/Velocity 설정 — 토크 인가 **전에** 호출한다.

        기본값 0(=최고속 즉시 이동)이면 그리퍼가 움직일 때마다 순간 과전류로 토크가
        풀린다(HW-8 실기 검증, 재현율 100%). 팔 축도 같은 이유로 완만하게 둔다.

        ⚠️ **여기가 팔의 실제 속도를 정하는 유일한 곳이다.** `trajectory_callback` 은
        `time_from_start` 를 쓰지 않고 goal tick 만 SyncWrite 하므로, 궤적의 duration
        (arm_fsm 의 `arm_move_speed`)은 FSM 내부 타임아웃 추정에만 쓰이고 서보 속도에는
        영향이 없다. 속도를 바꾸려면 `arm_profile_velocity` 를 조정할 것.

        ⚠️ 그리퍼는 팔과 **따로** 둔다(`gripper_profile_velocity`). 그리퍼 속도를 낮추면
        완전 개폐 시간이 늘어나는데, `gripper_presets.gripper_action_time`(2.5s)은 그
        시간을 넘겨야 파지 effort 를 제대로 읽는다 — 같이 낮추면 "닫히는 도중에 판정"
        해서 grasp effort 가 0 으로 읽히는 알려진 실패로 돌아간다.
        """
        if velocity is None:
            velocity = self.arm_profile_velocity
        for addr, value, field in (
            (ADDR_PROFILE_ACCELERATION, self.profile_acceleration, "Profile Acceleration"),
            (ADDR_PROFILE_VELOCITY, velocity, "Profile Velocity"),
        ):
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, addr, value
            )
            if result != 0 or error != 0:
                self.get_logger().warn(
                    f"{field} 설정 실패: {label}, id={dxl_id}, "
                    f"result={result}, error={error} — 과전류 토크 트립 위험"
                )

    def _write_gripper_goal_pwm(self, dxl_id):
        """그리퍼 토크 상한(Goal PWM) 설정 — 파지 중 Overload 트립 방지.

        물체를 문 채 목표에 도달 못 하면 서보는 무한정 밀어붙이다 Overload 로 토크가
        끊긴다(2026-08-09 실기). 여기서 상한을 걸면 그 힘에서 멈춰 계속 물고 있는다.
        값 근거와 조정 방향은 `gripper_presets.py` 의 `gripper_goal_pwm` 주석 참고.
        """
        if self.gripper_goal_pwm <= 0:
            return
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_PWM, self.gripper_goal_pwm)
        if result != 0 or error != 0:
            self.get_logger().warn(
                f"Goal PWM 쓰기 실패: id={dxl_id}, result={result}, error={error} — "
                "토크 상한이 안 걸려 파지 중 Overload 트립 가능")
        else:
            self.get_logger().info(
                f"Goal PWM 설정: id={dxl_id} -> {self.gripper_goal_pwm} "
                f"(최대 885, 파지 토크 상한)")

    def _required_gripper_mode(self, dxl_id):
        return self.gripper_required_operating_modes.get(
            dxl_id, self.gripper_required_operating_mode)

    def _warn_if_torque_off(self):
        """토크가 꺼진 채 모션 명령이 들어오면 크게 알린다.

        ⚠️ 2026-08-12 실기: 콘솔이 종료하며 토크를 풀어둔 상태에서 픽을 돌렸더니
        FSM 은 PERCEIVE→…→GRASP 전 구간을 정상 수행하고 브릿지도 goal 을 다 썼는데
        **서보가 전부 무시**해서 팔이 한 tick 도 안 움직였다. 어디에도 에러가 없어
        "프로그램은 도는데 안 움직인다" 로만 보인다 — 이 저장소가 반복해서 밟는
        조용한 실패다. 여기서 한 번은 말해준다.

        자동으로 토크를 켜지는 **않는다**. 사람이 팔을 만지려고 일부러 푼 것일 수
        있고, 그때 명령 하나에 팔이 다시 잠기면 손을 다친다.
        """
        off = sorted(self.active_ids - self.torque_enabled_ids)
        if not off:
            return
        self.get_logger().error(
            f"모션 명령을 받았지만 ID {off} 의 토크가 꺼져 있습니다 — 서보가 무시하므로 "
            "팔은 움직이지 않습니다(에러 없이 조용히). 켜려면 mission_console 의 "
            "'torque on' 또는 /dynamixel/torque_request 에 [1] 발행.")

    def torque_request_callback(self, msg):
        """`[enable, id...]` → 해당 ID 토크 on/off. id 생략 시 등록된 전 축.

        ⚠️ 끄면 팔이 중력으로 처진다. 그래서 "요청받았으니 끈다" 이상은 하지 않는다 —
        여기서 자세를 미리 접거나 하는 배려를 넣으면, 정작 급히 끊고 싶을 때 그 동작이
        먼저 나가버린다(안전 게이트에 부가 동작을 넣지 않는다는 이 저장소의 원칙).
        """
        data = list(msg.data)
        if not data:
            self.get_logger().error("torque_request: [enable, id...] 형식이어야 합니다")
            return
        if self.read_only:
            self.get_logger().warn("torque_request 무시 — read_only 모드는 레지스터를 쓰지 않습니다")
            return

        enable = 1 if data[0] else 0
        ids = data[1:] or sorted(self.active_ids)
        if self.tool_type == 'dual_motor_gripper' and self.dual_single_axis_id:
            selected = self.dual_single_axis_id
            other = 4 if selected == 3 else 3
            if enable:
                # The idle side is made torque-free first.  Do not merely
                # omit it from an enable request: it may still hold torque
                # from a preceding dual session.
                with self._bus_lock:
                    result, error = self.packet_handler.write1ByteTxRx(
                        self.port_handler, other, ADDR_TORQUE_ENABLE,
                        TORQUE_DISABLE)
                    actual, read_result, read_error = (
                        self.packet_handler.read1ByteTxRx(
                            self.port_handler, other, ADDR_TORQUE_ENABLE))
                if (result != 0 or error != 0 or read_result != 0
                        or read_error != 0 or int(actual) != TORQUE_DISABLE):
                    self.get_logger().error(
                        f'single-side enable rejected: ID {other} torque OFF '
                        'could not be verified')
                    return
                self.torque_enabled_ids.discard(other)
                ids = [selected]
            else:
                ids = [dxl_id for dxl_id in ids if dxl_id == selected]
        if enable and (self.emergency_stop_active or self.tool_detached):
            self.get_logger().warn(
                'torque enable rejected: emergency stop or tool-detached latch is active')
            return
        if enable and self.tool_type == 'dual_motor_gripper':
            expected_ids = ([self.dual_single_axis_id]
                            if self.dual_single_axis_id else [3, 4])
            samples = [self._tool_samples.get(dxl_id, {}) for dxl_id in expected_ids]
            dual_enable_ready = (
                ids == expected_ids
                and self.control_scope in ('END_EFFECTOR_ONLY', 'FULL_ROBOT')
                and self._tool_enable_allowed()
                and all(sample.get('online')
                        and sample.get('hardware_error') == 0
                        and sample.get('position') is not None
                        for sample in samples))
            if not dual_enable_ready:
                self.get_logger().warn(
                    'dual torque enable rejected: profile/scope/online/HW gate closed')
                return
        if self.mock_mode:
            # Mock must exercise the same explicit-enable state machine, while
            # remaining strictly free of serial/register writes.
            if set(ids) - set(self.tool_ids):
                self.get_logger().warn(
                    f"Mock torque request rejected outside selected tool IDs: {ids}")
                return
            if enable:
                self.torque_enabled_ids.update(ids)
            else:
                self.torque_enabled_ids.difference_update(ids)
            for dxl_id in ids:
                self._tool_samples[dxl_id]['torque_state'] = 'ON' if enable else 'OFF'
            self.get_logger().info(
                f"Mock torque {'enabled' if enable else 'disabled'}: ID {ids}")
            return
        applied, failed = [], []
        for dxl_id in ids:
            if dxl_id not in self.active_ids:
                self.get_logger().warn(f"torque_request 무시 — 등록 안 된 ID {dxl_id}")
                continue
            if enable:
                try:
                    self._prepare_dual_gripper_enable(dxl_id)
                except RuntimeError as exc:
                    self.get_logger().error(
                        f'ID {dxl_id} torque enable rejected: {exc}')
                    failed.append(dxl_id)
                    continue
                # ⚠️ 토크를 켜기 **전에** Goal Position 을 현재 위치로 덮어쓴다.
                # 토크가 꺼진 동안 팔은 중력으로 처지는데 Goal 레지스터에는 마지막
                # 명령값이 그대로 남아 있다 — 그냥 켜면 서보가 그 옛 목표로 **튄다**
                # (teleop_core 의 resume 이 _sync_goal_to_measured 를 하는 것과 같은 이유).
                with self._bus_lock:
                    pos, res, err = self.packet_handler.read4ByteTxRx(
                        self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
                cached_valid = False
                if res != 0 or err != 0:
                    # Sync-read already validated this actuator and carries a
                    # recent measured position.  A one-off direct read can
                    # lose a packet during profile switching; use that cache
                    # only when it is online, error-free, and within profile
                    # limits.  Never invent a position; otherwise fail closed.
                    cached = self._tool_samples.get(dxl_id, {})
                    cached_pos = cached.get('position')
                    safe_min = self.tool_profile.get('safe_min_tick')
                    safe_max = self.tool_profile.get('safe_max_tick')
                    cached_valid = (
                        cached.get('online') is True
                        and cached.get('hardware_error') in (0, None)
                        and isinstance(cached_pos, (int, float))
                        and (safe_min is None or cached_pos >= safe_min
                             - ENABLE_POSITION_TOLERANCE_TICKS)
                        and (safe_max is None or cached_pos <= safe_max
                             + ENABLE_POSITION_TOLERANCE_TICKS)
                    )
                    if not cached_valid:
                        self.get_logger().error(
                            f"ID {dxl_id} 현재 위치를 못 읽어 토크 인가를 거부합니다")
                        failed.append(dxl_id)
                        continue
                    pos = int(cached_pos)
                    self.get_logger().warn(
                        f"ID {dxl_id} 직접 위치 읽기 실패 — 최근 검증 위치 "
                        f"{pos} tick으로 토크 인가를 계속합니다")
                if res == 0 and err == 0 or cached_valid:
                    with self._bus_lock:
                        self.packet_handler.write4ByteTxRx(
                            self.port_handler, dxl_id, ADDR_GOAL_POSITION, pos)
            requested_torque = TORQUE_ENABLE if enable else TORQUE_DISABLE
            verify_detail = ''
            with self._bus_lock:
                result, error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, dxl_id, ADDR_TORQUE_ENABLE,
                    requested_torque)
                actual, read_result, read_error = (
                    self.packet_handler.read1ByteTxRx(
                        self.port_handler, dxl_id, ADDR_TORQUE_ENABLE))
            verified = False
            verified = read_result == 0 and read_error == 0 \
                and int(actual) == requested_torque
            verify_detail = (
                f'actual={actual}, read_result={read_result}, '
                f'read_error={read_error}')
            if not verified:
                if result == 0 and error == 0:
                    # A transient status packet loss can leave a successful
                    # write without a trustworthy readback.  Retry once, but
                    # never mark the actuator enabled on write-only success.
                    with self._bus_lock:
                        result, error = self.packet_handler.write1ByteTxRx(
                            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE,
                            requested_torque)
                        if result == 0 and error == 0:
                            actual, read_result, read_error = (
                                self.packet_handler.read1ByteTxRx(
                                    self.port_handler, dxl_id,
                                    ADDR_TORQUE_ENABLE))
                        verified = read_result == 0 and read_error == 0 \
                            and int(actual) == requested_torque
                        verify_detail = (
                            f'actual={actual}, read_result={read_result}, '
                            f'read_error={read_error}')
            if result != 0 or error != 0 or not verified:
                self.get_logger().error(
                    f'ID {dxl_id} torque readback mismatch: '
                    f'write_result={result}, write_error={error}, {verify_detail}')
                failed.append(dxl_id)
                continue
            applied.append(dxl_id)
            if enable:
                self.torque_enabled_ids.add(dxl_id)
            else:
                self.torque_enabled_ids.discard(dxl_id)

        word = "인가" if enable else "해제"
        if applied:
            self.get_logger().warn(f"토크 {word}: ID {applied}")
        if failed:
            self.get_logger().error(f"토크 {word} 실패: ID {failed}")
            if enable and self.tool_type == 'dual_motor_gripper':
                # A partial dual enable is unsafe: make the pair torque-free.
                for dxl_id in applied:
                    with self._bus_lock:
                        self.packet_handler.write1ByteTxRx(
                            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE,
                            TORQUE_DISABLE)
                    self.torque_enabled_ids.discard(dxl_id)

    def _prepare_dual_gripper_enable(self, dxl_id):
        """Apply only validated dual RAM profile on an explicit enable request.

        Startup remains observation-only.  This runs only after the operator
        presses enable, with torque still OFF, and never touches non-tool IDs.
        """
        if self.tool_type != 'dual_motor_gripper':
            return
        if self.tool_ids != [3, 4] or dxl_id not in self.tool_ids:
            raise RuntimeError('dual profile setup is restricted to IDs [3, 4]')
        required = (self.tool_profile.get('required_operating_modes') or {}).get(
            dxl_id, (self.tool_profile.get('required_operating_modes') or {}).get(
                str(dxl_id)))
        accel = int(self.tool_profile['profile_acceleration'])
        velocity = int(self.tool_profile['profile_velocity'])
        goal_pwm = int(self.tool_profile['goal_pwm'])
        with self._bus_lock:
            hardware_error, comm, packet_error = \
                self.packet_handler.read1ByteTxRx(
                    self.port_handler, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
            if comm != 0 or packet_error != 0 or hardware_error != 0:
                raise RuntimeError(
                    f'hardware error preflight failed: {hardware_error}')
            torque, comm, packet_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE)
            if comm != 0 or packet_error != 0 or torque != TORQUE_DISABLE:
                raise RuntimeError('actual torque must be OFF before profile setup')
            mode, comm, packet_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE)
            if comm != 0 or packet_error != 0 or mode != int(required):
                raise RuntimeError(
                    f'operating mode {mode} does not match required {required}')
            for address, value, size, label in (
                    (ADDR_PROFILE_ACCELERATION, accel, 4, 'profile acceleration'),
                    (ADDR_PROFILE_VELOCITY, velocity, 4, 'profile velocity'),
                    (ADDR_GOAL_PWM, goal_pwm, 2, 'goal PWM')):
                writer = (self.packet_handler.write2ByteTxRx
                          if size == 2 else self.packet_handler.write4ByteTxRx)
                comm, packet_error = writer(
                    self.port_handler, dxl_id, address, value)
                if comm != 0 or packet_error != 0:
                    raise RuntimeError(f'{label} write failed')
                reader = (self.packet_handler.read2ByteTxRx
                          if size == 2 else self.packet_handler.read4ByteTxRx)
                actual, comm, packet_error = reader(
                    self.port_handler, dxl_id, address)
                if (comm != 0 or packet_error != 0 or int(actual) != value):
                    raise RuntimeError(f'{label} readback failed: {actual} != {value}')

    def fsm_command_callback(self, msg):
        """Route normal OPEN/CLOSE/STOP only to the selected tool FSM."""
        command = str(msg.data).strip().upper()
        if str(msg.data).lstrip().startswith('{'):
            try:
                request = json.loads(msg.data)
                if request['tool_type'] != self.tool_type:
                    self.get_logger().warn('Ignoring command from a replaced tool context')
                    return
                command = str(request['command']).strip().upper()
            except (ValueError, KeyError, TypeError):
                return
        if self.tool_type == 'cleaner':
            self._cleaner_direction_command(command)
            return
        if self.tool_fsm is None or self.tool_type not in (
                'spur_1motor_gripper', 'dual_motor_gripper'):
            self.get_logger().warn('tool FSM command rejected: no gripper FSM')
            return
        stopping = command in ('STOP', 'DISABLE')
        if self.read_only:
            self.get_logger().warn('tool FSM command rejected: bridge is read-only')
            return
        if (not stopping and (
                self.control_scope not in ('END_EFFECTOR_ONLY', 'FULL_ROBOT')
                or self.control_mode != 'MANUAL'
                or self.emergency_stop_active or self.tool_detached)):
            self.get_logger().warn('tool FSM command rejected by ingress safety gate')
            return
        if (self.tool_type == 'dual_motor_gripper'
                and self.dual_single_axis_id and command in ('OPEN', 'CLOSE')):
            dxl_id = self.dual_single_axis_id
            try:
                if self.read_hardware_error(dxl_id) != 0:
                    raise RuntimeError(f'ID {dxl_id} hardware error')
                if self.read_torque(dxl_id) != TORQUE_ENABLE:
                    raise RuntimeError(f'ID {dxl_id} Torque Enable is OFF')
                endpoints = self.tool_profile.get('motor_endpoints') or {}
                endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
                if not endpoint:
                    raise RuntimeError(f'ID {dxl_id} endpoint missing')
                self.goal_position(dxl_id, endpoint[command.lower()])
                self.tool_fsm.state = (
                    ToolState.OPEN if command == 'OPEN' else ToolState.CLOSED)
                self.get_logger().info(
                    f'single-side dual command {command}: ID {dxl_id} only')
            except Exception as exc:
                self.get_logger().warn(f'single-side dual command rejected: {exc}')
            return
        try:
            state = self.tool_fsm.command(command)
            self.get_logger().info(
                f'{self.tool_type} FSM command {msg.data!r} -> {state.name}')
        except Exception as exc:  # command rejection makes no register write
            self.get_logger().warn(f'tool FSM command rejected: {exc}')

    def manual_recovery_callback(self, msg):
        """Manual dual resync ingress; never participates in OPEN/CLOSE policy."""
        if (self.dual_manual_recovery is None
                or self.tool_type != 'dual_motor_gripper'
                or self.tool_ids != [3, 4]
                or self.control_scope != 'END_EFFECTOR_ONLY'
                or self.control_mode != 'MANUAL'
                or self.read_only or self.emergency_stop_active
                or self.tool_detached):
            self.get_logger().warn('manual dual recovery jog rejected by safety gate')
            return
        try:
            request = json.loads(msg.data)
            target = self.dual_manual_recovery.jog(
                request['actuator_id'], request['delta_deg'])
            self.get_logger().info(
                f'manual dual recovery: ID{request["actuator_id"]} -> {target}')
        except (KeyError, TypeError, ValueError, RuntimeError,
                DualManualRecoveryError) as exc:
            self.get_logger().warn(f'manual dual recovery jog rejected: {exc}')

    def dual_calibration_command_callback(self, msg):
        """Narrow ingress for witnessed dual endpoint calibration only."""
        session = self.dual_calibration_session
        if (session is None or self.tool_type != 'dual_motor_gripper'
                or self.tool_ids != [3, 4]
                or self.control_scope != 'END_EFFECTOR_ONLY'
                or self.control_mode != 'MANUAL' or self.read_only
                or self.emergency_stop_active or self.tool_detached):
            self.get_logger().warn('dual calibration command rejected by safety gate')
            return
        try:
            request = json.loads(msg.data)
            command = str(request['command']).lower()
            if command == 'start':
                session.start()
            elif command == 'stop':
                session.stop()
            elif command == 'jog_motor_degrees':
                session.jog_motor_degrees(
                    request['actuator_id'], request['delta_deg'])
            elif command == 'jog_pair_degrees':
                session.jog_pair_degrees(request['delta_deg'])
            elif command == 'hold':
                session.hold()
            elif command == 'capture_open':
                session.capture_open()
            elif command == 'capture_close':
                session.capture_close()
            elif command == 'validate':
                session.validate()
            elif command == 'save':
                session.save(self.get_parameter('tool_profile_file').value)
                self._reload_dual_profile()
            else:
                raise ValueError(f'unsupported dual calibration command {command!r}')
            self.get_logger().info(f'dual calibration command completed: {command}')
        except (KeyError, TypeError, ValueError, RuntimeError,
                DualCalibrationError) as exc:
            self.get_logger().warn(f'dual calibration command rejected: {exc}')

    def _reload_dual_profile(self):
        if self.tool_type != 'dual_motor_gripper':
            raise RuntimeError('only dual profile may reload here')
        profiles = load_profiles(self.get_parameter('tool_profile_file').value)
        manager = ToolManager(
            profiles, ParameterToolIdentityProvider(self.tool_type),
            mock_mode=self.mock_mode)
        selection = manager.refresh('IDLE')
        if selection.profile.get('actuator_ids') != [3, 4]:
            raise RuntimeError('saved dual profile actuator allowlist is not [3, 4]')
        self.tool_manager = manager
        self.tool_selection = selection
        self.tool_profile = selection.profile
        self.tool_ids = [3, 4]
        self.tool_motion_allowed = selection.valid
        self.dual_calibration_session.reload(self.tool_profile)

    def _spur_manual_watchdog(self):
        try:
            with self._bus_lock:
                self.spur_manual_control.watchdog()
        except Exception as exc:
            self.get_logger().warn(f'ID5 manual HOLD failed: {exc}')

    def calibration_command_callback(self, msg):
        """Narrow JSON ingress for one CalibrationSession operation at a time."""
        session = self.calibration_session
        if session is None:
            self.get_logger().warn('calibration command rejected: ID5 session unavailable')
            return
        try:
            request = json.loads(msg.data)
            command = str(request['command']).lower()
            if command.startswith('manual_'):
                with self._bus_lock:
                    self.spur_manual_control.command(command, request.get('delta_deg', 0))
                return
            operations = {
                'start': session.start, 'stop': session.stop,
                'enable': session.enable, 'disable': session.disable,
                'capture_open': session.capture_open,
                'capture_close': session.capture_close,
            }
            if command == 'jog_motor_degrees':
                session.jog_motor_degrees(request['delta_deg'])
            elif command == 'validate':
                session.validate()
            elif command == 'save':
                session.save(self.get_parameter('tool_profile_file').value)
                self._reload_spur_profile()
            elif command in operations:
                operations[command]()
            else:
                raise ValueError(f'unsupported calibration command {command!r}')
        except Exception as exc:
            self.get_logger().warn(f'calibration command rejected: {exc}')

    def _reload_spur_profile(self):
        """Reload only an explicitly saved ID5 profile, then revalidate FSM."""
        if self.tool_type != 'spur_1motor_gripper':
            raise RuntimeError('only spur ID5 profile may be reloaded here')
        profiles = load_profiles(self.get_parameter('tool_profile_file').value)
        manager = ToolManager(
            profiles, ParameterToolIdentityProvider(self.tool_type),
            mock_mode=self.mock_mode)
        selection = manager.refresh('IDLE')
        if selection.profile.get('actuator_ids') != [5]:
            raise RuntimeError('saved spur profile actuator allowlist is not [5]')
        self.tool_manager = manager
        self.tool_selection = selection
        self.tool_profile = selection.profile
        self.tool_ids = [5]
        self.tool_motion_allowed = selection.valid
        self.tool_fsm = self.tool_manager.create_fsm(self)
        state = self.tool_fsm.startup()
        if state.name != 'READY':
            raise RuntimeError(f'saved profile did not produce READY: {state.name}')
        self.calibration_session.profile = dict(self.tool_profile)

    def _manual_recovery_id_allowed(self, dxl_id):
        if (self.tool_type != 'dual_motor_gripper' or self.tool_ids != [3, 4]
                or int(dxl_id) not in (3, 4)):
            raise ValueError('manual recovery actuator is outside dual allowlist')

    def read_dual_calibration_state(self):
        """Fresh actual state for capture/validation; never cached GUI data."""
        if (self.tool_type != 'dual_motor_gripper' or self.tool_ids != [3, 4]
                or self.control_scope != 'END_EFFECTOR_ONLY'):
            raise RuntimeError('dual calibration is restricted to IDs [3, 4]')
        state = {}
        with self._bus_lock:
            for dxl_id in self.tool_ids:
                state[dxl_id] = {
                    'position': self._read_register(
                        dxl_id, ADDR_PRESENT_POSITION, 4,
                        'dual calibration present position', signed=True),
                    'torque': self._read_register(
                        dxl_id, ADDR_TORQUE_ENABLE, 1,
                        'dual calibration torque'),
                    'hardware_error': self._read_register(
                        dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                        'dual calibration hardware error'),
                    'model': self._read_register(
                        dxl_id, 0, 2, 'dual calibration model'),
                }
        return state

    def dual_calibration_jog(self, dxl_id, delta_deg):
        session = self.dual_calibration_session
        if session is None or not session.active:
            raise RuntimeError('dual calibration session is not active')
        return self.dual_manual_recovery.jog(
            dxl_id, delta_deg, allowed_degrees=session.ALLOWED_DEGREES,
            goal_writer=self.dual_calibration_goal_position)

    def dual_calibration_pair_jog(self, delta_deg):
        """Paired variant of the existing witnessed calibration jog."""
        session = self.dual_calibration_session
        if self.read_only or session is None or not session.active:
            raise RuntimeError('dual calibration pair jog is unavailable')
        delta_tick = int(round(float(delta_deg) * 4096 / 360.0))
        if delta_tick == 0:
            raise RuntimeError('dual calibration pair jog produced zero ticks')
        state = self.read_dual_calibration_state()
        targets = {
            dxl_id: int(state[dxl_id]['position']) + delta_tick
            for dxl_id in (3, 4)}
        self.get_logger().info(
            f'dual calibration pair jog: delta_deg={delta_deg:+.1f}, '
            f'present={{3: {state[3]["position"]}, 4: {state[4]["position"]}}}, '
            f'targets={targets}')
        with self._bus_lock:
            self.group_sync_write.clearParam()
            try:
                for dxl_id in (3, 4):
                    data = self.int_to_little_endian_4bytes(targets[dxl_id])
                    if not self.group_sync_write.addParam(dxl_id, data):
                        raise RuntimeError(
                            f'ID{dxl_id} calibration pair staging failed')
                result = self.group_sync_write.txPacket()
                if result != 0:
                    raise RuntimeError(
                        f'dual calibration pair GroupSyncWrite failed: {result}')
            finally:
                self.group_sync_write.clearParam()
        return targets

    def dual_calibration_hold(self):
        """Read both motors and replace pending calibration goals with HOLD."""
        session = self.dual_calibration_session
        if self.read_only or session is None or not session.active:
            raise RuntimeError('dual calibration hold is unavailable')
        positions = {}
        with self._bus_lock:
            for dxl_id in (3, 4):
                positions[dxl_id] = self._read_register(
                    dxl_id, ADDR_PRESENT_POSITION, 4,
                    'dual calibration hold position', signed=True)
            for dxl_id in (3, 4):
                self._write_register(
                    dxl_id, ADDR_GOAL_POSITION, 4,
                    int(positions[dxl_id]) & 0xffffffff,
                    'dual calibration hold goal')
        self.get_logger().info(
            f'dual calibration HOLD: present/targets={positions}')
        return positions

    def read_manual_position(self, dxl_id):
        self._manual_recovery_id_allowed(dxl_id)
        return self._read_register(
            dxl_id, ADDR_PRESENT_POSITION, 4,
            'manual recovery present position', signed=True)

    def read_manual_torque(self, dxl_id):
        self._manual_recovery_id_allowed(dxl_id)
        return self._read_register(
            dxl_id, ADDR_TORQUE_ENABLE, 1, 'manual recovery torque')

    def read_manual_hardware_error(self, dxl_id):
        self._manual_recovery_id_allowed(dxl_id)
        return self._read_register(
            dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
            'manual recovery hardware error')

    def manual_goal_position(self, dxl_id, tick):
        self._manual_recovery_id_allowed(dxl_id)
        if self.read_only:
            raise RuntimeError('read-only bridge rejects manual recovery goal')
        endpoint = (self.tool_profile.get('motor_endpoints') or {}).get(
            int(dxl_id), (self.tool_profile.get('motor_endpoints') or {}).get(
                str(dxl_id)))
        if not endpoint:
            raise RuntimeError(f'missing manual recovery endpoint for ID {dxl_id}')
        low, high = sorted((int(endpoint['open']), int(endpoint['close'])))
        current = self.read_manual_position(dxl_id)
        tick = int(tick)
        # A motor that is already outside its calibrated span may only move
        # inward.  Once in range, every recovery target must remain in range.
        if ((current < low and not current < tick <= high)
                or (current > high and not low <= tick < current)
                or (low <= current <= high and not low <= tick <= high)):
            raise RuntimeError(
                f'ID{dxl_id} recovery target {tick} is not inward/in-range '
                f'for [{low}, {high}] from current {current}')
        self._write_register(dxl_id, ADDR_GOAL_POSITION, 4,
                             tick & 0xffffffff,
                             'manual recovery goal position')

    def dual_calibration_goal_position(self, dxl_id, tick):
        """One witnessed calibration click: selected ID only, no old endpoints.

        Existing endpoint normalization is deliberately not an ingress gate
        here.  Its purpose is to calibrate a mechanism whose old endpoints no
        longer describe reality; the normal OPEN/CLOSE paths retain that gate.
        """
        self._manual_recovery_id_allowed(dxl_id)
        session = self.dual_calibration_session
        if self.read_only or session is None or not session.active:
            raise RuntimeError('dual calibration relative goal is unavailable')
        self._write_register(dxl_id, ADDR_GOAL_POSITION, 4,
                             int(tick) & 0xffffffff,
                             'dual calibration relative goal position')

    def _check_gripper_in_calibrated_range(self, dxl_id):
        """그리퍼가 캘리브 tick 범위 **밖**에 있으면 크게 경고한다.

        ⚠️ 2026-08-12 실기: 토크를 끄고 팔을 손으로 다루는 동안 그리퍼가 닫힘 끝단
        (-401)보다 786 tick 아래(-1187)까지 밀려 들어갔다. 그 영역에서는
        `gripper_goal_pwm`(280, 파지 토크 상한)의 힘으로 **되돌아 나올 수 없다** —
        실측으로 tick -890 부근에서 전류 316 을 뽑으며 스톨했고, 양방향 모두 막혔다.
        정상 범위 안에서는 같은 PWM 280 으로 전 구간을 2.5초에 여닫는다(실측).

        증상이 지독하다: 그리퍼가 "안 닫히고", `/joint_states` effort 는 스톨 전류
        316 을 계속 보고해 `grasp_effort_thresh`(250)를 넘으므로 FSM 은 **빈손인데
        파지 성공으로 판정**한다. 어느 로그에도 에러가 안 뜬다.

        복구는 Goal PWM 을 일시적으로 올려(500 이상) 범위 안으로 끌어낸 뒤 되돌리는
        것이다. 자동으로 하지 않는 이유는 그 상한이 Overload 트립을 막는 안전장치라,
        올릴지는 사람이 상황을 보고 정해야 하기 때문이다.
        """
        pos, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if result != 0 or error != 0:
            return
        tick = pos - (1 << 32) if pos >= (1 << 31) else pos
        lo = min(self.gripper_close_tick, self.gripper_open_tick)
        hi = max(self.gripper_close_tick, self.gripper_open_tick)
        margin = max(1, int(0.05 * (hi - lo)))
        if lo - margin <= tick <= hi + margin:
            return
        self.get_logger().error(
            f"그리퍼(id={dxl_id})가 캘리브 범위 밖입니다: tick={tick} "
            f"(정상 {lo}~{hi}). 이 상태에서는 Goal PWM {self.gripper_goal_pwm} 의 힘으로 "
            "빠져나오지 못해 '안 닫히는' 것처럼 보이고, 스톨 전류가 파지 임계를 넘어 "
            "**빈손인데 파지 성공으로 오판**합니다.")
        if self.gripper_auto_recover:
            self._recover_gripper_range(dxl_id, tick)
        else:
            self.get_logger().error(
                "gripper_auto_recover=false 이므로 자동 복구하지 않습니다 — Goal PWM 을 "
                "일시적으로 500 이상으로 올려 범위 안으로 되돌린 뒤 다시 시작하세요.")

    def _recover_gripper_overload(self, dxl_id):
        """파지 중 Overload 트립을 REBOOT 로 되살리고 마지막 파지 목표를 다시 건다.

        2026-08-19 사용자 지시("꽉 잡고 Overload 나면 재부팅해서 그 상태 유지")로 추가.

        ## 이게 무엇을 하고 무엇을 못 하는가

        Overload(HW error 0x20)가 래치되면 서보는 **토크를 끊고** REBOOT 전까지 어떤
        명령에도 응답하지 않는다. 즉 트립하는 순간 이미 손이 풀린 것이라, 재부팅은
        **화물을 붙잡아 두지 못한다.** 이 함수의 값어치는 "그리퍼가 죽은 채로 남아
        이후 미션이 전부 실패하는 것"을 막는 데 있다.

        ## REBOOT 이 날리는 것 (제일 중요)

        REBOOT 은 RAM 을 초기화한다 — **Goal PWM 이 885(무제한)로, Profile 이 0(최고속)
        으로, 토크가 OFF 로, Goal Position 이 초기값으로** 돌아간다. 되살린 뒤 이걸 다시
        안 써주면 다음 파지는 885 로 물어 **3.5초 만에 또 트립**하고, 프로파일 0 은
        순간 과전류로 토크가 풀리는 별개의 알려진 실패(그리퍼 Profile 25/80 규칙)를
        일으킨다. 그래서 기동 시와 **똑같은 순서**로 다시 세운다:
            _enable_torque(모드/프로파일/토크) → _write_gripper_goal_pwm → Goal Position

        Operating Mode(EEPROM)는 살아남지만 `_enable_torque` 가 어차피 확인·설정한다.

        ## 왜 무한 재시도를 안 하는가

        Overload 보호는 코일이 타는 걸 막으려고 있는 장치다. 반복해서 때리면 서보가
        상한다. `gripper_overload_window_s` 안에서 `gripper_overload_max_reboots` 회를
        넘기면 포기하고 크게 알린다 — 그 시점엔 파지력 설정이 그 물체에 안 맞는
        것이므로, 재부팅이 아니라 사람이 PWM 을 낮추거나 마찰 패드를 붙여야 한다.
        """
        now = time.time()
        self._gripper_reboot_times = [
            t for t in self._gripper_reboot_times
            if now - t <= self.gripper_overload_window_s]
        if len(self._gripper_reboot_times) >= self.gripper_overload_max_reboots:
            if not self._gripper_overload_gave_up:
                self._gripper_overload_gave_up = True
                self.get_logger().error(
                    f"그리퍼(id={dxl_id}) Overload 가 "
                    f"{self.gripper_overload_window_s:.0f}초 안에 "
                    f"{len(self._gripper_reboot_times)}회 반복돼 자동 재부팅을 멈춥니다. "
                    "재부팅을 더 반복하면 서보 코일이 상합니다. 지금 설정으로는 이 물체를 "
                    "못 뭅니다 — gripper_goal_pwm 을 낮추고 손가락 마찰(고무/실리콘 패드)을 "
                    "올리세요. 미끄럼 힘은 μ×법선력인데 PWM 은 법선력만 건드립니다.")
            return
        self._gripper_reboot_times.append(now)

        self._gripper_recovering = True
        self.gripper_recovering_pub.publish(Bool(data=True))
        self.get_logger().error(
            f"그리퍼(id={dxl_id}) Overload 트립 — 토크가 끊겼습니다(화물을 들고 있었다면 "
            f"이미 놓쳤습니다). REBOOT 으로 되살립니다 "
            f"({len(self._gripper_reboot_times)}/{self.gripper_overload_max_reboots}회).")
        try:
            self.packet_handler.reboot(self.port_handler, dxl_id)
            time.sleep(self.gripper_reboot_settle_s)
            # 기동 시와 같은 순서로 재설정. 하나라도 빠지면 다음 파지가 더 빨리 트립한다.
            if not self._enable_torque(
                    dxl_id, f"gripper(id {dxl_id}) 재부팅 후",
                    self._required_gripper_mode(dxl_id),
                    self.gripper_profile_velocity):
                self.get_logger().error(
                    f"그리퍼(id={dxl_id}) 재부팅 후 토크 인가 실패 — 그리퍼가 죽은 "
                    "상태입니다. 스택을 재기동하세요.")
                return
            self._write_gripper_goal_pwm(dxl_id)
            if self._last_gripper_goal_tick is not None:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION,
                    self._last_gripper_goal_tick & 0xFFFFFFFF)
                self.get_logger().warn(
                    f"그리퍼(id={dxl_id}) 되살아났습니다 — 마지막 목표 tick "
                    f"{self._last_gripper_goal_tick} 로 다시 뭅니다. "
                    "⚠️ 화물이 이미 떨어졌다면 빈손을 쥐는 것이니 눈으로 확인하세요.")
        finally:
            self._gripper_recovering = False
            self.gripper_recovering_pub.publish(Bool(data=False))

    def _enable_torque(self, dxl_id, label, required_mode=None, velocity=None):
        """현재 위치를 goal로 검증 동기화한 뒤에만 torque를 켠다.

        실기 기준선의 급작스런 모션 방지 순서와 runtime tool profile의
        operating-mode 검증을 함께 유지한다. 어떤 readback이든 다르면
        torque는 OFF인 채로 남는다.
        """
        try:
            with self._bus_lock:
                torque = self._read_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1, "startup torque")
                if torque != TORQUE_DISABLE:
                    raise RuntimeError(
                        f"startup requires Torque OFF, readback={torque}")
                if required_mode is not None:
                    mode = self._read_register(
                        dxl_id, ADDR_OPERATING_MODE, 1,
                        "startup operating mode")
                    if mode != required_mode:
                        raise RuntimeError(
                            f"operating mode mismatch: expected "
                            f"{required_mode}, read {mode}; automatic mode "
                            "writes are disabled")
                present = self._read_register(
                    dxl_id, ADDR_PRESENT_POSITION, 4,
                    "startup present position", signed=True)
                self._write_register(
                    dxl_id, ADDR_GOAL_POSITION, 4,
                    present & 0xFFFFFFFF, "startup synchronize goal")
                goal_readback = self._read_register(
                    dxl_id, ADDR_GOAL_POSITION, 4,
                    "startup goal readback", signed=True)
                if goal_readback != present:
                    raise RuntimeError(
                        f"Present->Goal readback mismatch: "
                        f"present={present}, goal={goal_readback}")
                self._write_motion_profile(dxl_id, label, velocity)
                self._write_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1,
                    TORQUE_ENABLE, "startup torque enable")
                torque_readback = self._read_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1,
                    "startup torque readback")
                if torque_readback != TORQUE_ENABLE:
                    raise RuntimeError(
                        f"Torque ON readback failed: {torque_readback}")
        except Exception as exc:
            self.get_logger().error(
                f"Torque enable blocked: {label}, id={dxl_id}: {exc}")
            return False
        self.get_logger().info(f"Torque enabled safely: {label} -> id {dxl_id}")
        return True

    def _recover_gripper_range(self, dxl_id, tick):
        """캘리브 범위 밖으로 미끄러진 그리퍼를 열림 끝단으로 끌어낸다.

        ⚠️ 왜 매번 필요한가: `destroy_node()` 가 종료 시 전 ID 토크를 해제하는데,
        그리퍼는 힘을 잃으면 닫힘 방향으로 미끄러져 **끝단을 지나쳐 버린다**(2026-08-12
        실측: +1070 → -1259). 즉 스택을 재기동할 때마다 재발한다. 사람이 매번 손으로
        PWM 을 올려 빼내는 건 현실적이지 않아 자동화했다.

        복구는 파지 토크 상한(`gripper_goal_pwm`)을 **일시적으로** 올려서 한다 — 그
        상한은 물체를 문 채 무한정 미는 걸 막는 장치지, 빈 그리퍼를 옮기는 데 필요한
        힘까지 제한하려던 게 아니다. 실측으로 PWM 885 에서 1.5초 만에 끝나고 움직이는
        중 전류는 40~90(무부하 수준)까지 떨어진다. 끝나면 반드시 원래 값으로 되돌린다.
        """
        # 끝단(open_tick) 자체를 겨냥하지 않는다 — 거기는 기구적 스토퍼라 밀어붙이면
        # 랙이 미끄러진다(2026-08-12, 그때 오프셋이 통째로 ~1880 tick 이동했다).
        # 범위 안쪽 15% 지점이면 "밖에서 안으로" 라는 목적은 그대로 달성하면서
        # 스토퍼를 때리지 않는다.
        span = self.gripper_open_tick - self.gripper_close_tick
        target = int(self.gripper_open_tick - 0.15 * span)
        self.get_logger().warn(
            f"자동 복구 시도: Goal PWM {self.gripper_goal_pwm} → "
            f"{self.gripper_recover_pwm} 로 일시 상향, tick {tick} → {target} 로 이동")
        try:
            with self._bus_lock:
                self.packet_handler.write2ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_PWM,
                    self.gripper_recover_pwm)
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION,
                    target & 0xFFFFFFFF)
                deadline = time.time() + self.gripper_recover_timeout
                reached = False
                while time.time() < deadline:
                    time.sleep(0.2)
                    pos, result, error = self.packet_handler.read4ByteTxRx(
                        self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
                    if result != 0 or error != 0:
                        continue
                    now = to_signed(pos, LEN_PRESENT_POSITION)
                    if abs(now - target) <= 40:
                        reached = True
                        break
        finally:
            # 성공하든 실패하든 상한을 돌려놓는다. 높은 PWM으로 파지하면
            # Overload 트립 위험이 다시 생긴다.
            with self._bus_lock:
                self.packet_handler.write2ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_PWM,
                    self.gripper_goal_pwm)
        if reached:
            self.get_logger().info(
                f"자동 복구 성공 — 그리퍼가 범위 안({target})으로 복귀. "
                f"Goal PWM {self.gripper_goal_pwm} 복원됨.")
        else:
            self.get_logger().error(
                "자동 복구 실패 — 그리퍼가 여전히 범위 밖입니다. 기구적 걸림일 수 "
                "있으니 손으로 확인하세요(파지 판정을 신뢰하지 말 것).")

    def _joint_center(self, joint_name):
        return self.centers.get(joint_name, JOINT_CONFIG[joint_name]['center'])

    def _joint_gear_ratio(self, joint_name):
        """실측으로 덮어쓸 수 있는 기어비(`gear_ratios` 파라미터 > JOINT_CONFIG 기본값)."""
        return self.gear_ratios.get(joint_name, JOINT_CONFIG[joint_name]["gear_ratio"])

    def _on_set_parameters(self, params):
        """검증한 캘리브 값만 원자적으로 런타임 변환식에 반영한다."""
        centers, ratios = None, None
        gripper = {}
        for param in params:
            if param.name == 'centers':
                centers, errors = _parse_centers(param.value)
                if errors:
                    return SetParametersResult(
                        successful=False,
                        reason=f"centers: {'; '.join(errors)}")
            elif param.name == 'gear_ratios':
                ratios, errors = _parse_gear_ratios(param.value)
                if errors:
                    return SetParametersResult(
                        successful=False,
                        reason=f"gear_ratios: {'; '.join(errors)}")
            elif param.name in ('gripper_open_tick', 'gripper_close_tick'):
                gripper[param.name] = int(param.value)

        if gripper:
            open_tick = gripper.get(
                'gripper_open_tick', self.gripper_open_tick)
            close_tick = gripper.get(
                'gripper_close_tick', self.gripper_close_tick)
            if abs(open_tick - close_tick) < calib_math.MIN_GRIPPER_SPAN_TICK:
                return SetParametersResult(
                    successful=False,
                    reason=(f"그리퍼 개폐 tick 차이가 "
                            f"{abs(open_tick - close_tick)} 밖에 안 됩니다 — "
                            "잘못 측정된 값입니다"))

        changed = []
        if centers is not None:
            self.centers = centers
            changed.append(f'centers={centers}')
        if ratios is not None:
            self.gear_ratios = ratios
            changed.append(f'gear_ratios={ratios}')
        for name, value in gripper.items():
            setattr(self, name, value)
            changed.append(f'{name}={value}')

        if changed:
            self.get_logger().info('캘리브 런타임 반영: ' + ', '.join(changed))
            if not self.read_only:
                self.get_logger().warn(
                    '⚠️ 토크가 살아 있는 상태에서 캘리브가 바뀌었다 — 다음 '
                    '명령부터 rad↔tick 변환이 달라진다. 측정은 '
                    'read_only:=true 에서 할 것.')
        return SetParametersResult(successful=True)

    def _read_register(self, dxl_id, address, size, label, signed=False):
        reader = {
            1: self.packet_handler.read1ByteTxRx,
            2: self.packet_handler.read2ByteTxRx,
            4: self.packet_handler.read4ByteTxRx,
        }[size]
        try:
            with self._bus_lock:
                value, result, error = reader(
                    self.port_handler, dxl_id, address)
        except Exception as exc:
            # Dynamixel SDK can raise SerialException directly when the USB
            # adapter is unplugged or another process owns the port.  Turn it
            # into the same recoverable command/feedback failure as a packet
            # timeout; a ROS callback must never take the bridge down.
            raise RuntimeError(
                f'ID {dxl_id} {label} read transport failed: {exc}') from exc
        if result != 0 or error != 0:
            raise RuntimeError(
                f"ID {dxl_id} {label} read failed: result={result}, error={error}")
        return to_signed(value, size) if signed else value

    def _write_register(self, dxl_id, address, size, value, label):
        writer = {
            1: self.packet_handler.write1ByteTxRx,
            4: self.packet_handler.write4ByteTxRx,
        }[size]
        with self._bus_lock:
            result, error = writer(
                self.port_handler, dxl_id, address, value)
        if result != 0 or error != 0:
            raise RuntimeError(
                f"ID {dxl_id} {label} write failed: result={result}, error={error}")

    # Narrow hardware API for tool FSMs.  These methods contain no endpoint or
    # OPEN/CLOSE policy; allowlist enforcement lives at this boundary.
    def set_allowlist(self, actuator_ids):
        ids = tuple(int(item) for item in actuator_ids)
        if set(ids) - set(self.tool_ids):
            raise ValueError('tool FSM requested actuator outside selected profile')
        self._fsm_allowlist = set(ids)

    def _fsm_id_allowed(self, dxl_id):
        if int(dxl_id) not in getattr(self, '_fsm_allowlist', set()):
            raise ValueError(f'FSM actuator ID {dxl_id} is not allowlisted')

    def read_position(self, dxl_id):
        self._fsm_id_allowed(dxl_id)
        if self.mock_mode:
            return self._tool_samples.get(int(dxl_id), {}).get('position')
        with self._bus_lock:
            return self._read_register(dxl_id, ADDR_PRESENT_POSITION, 4,
                                       'FSM present position', signed=True)

    def read_torque(self, dxl_id):
        self._fsm_id_allowed(dxl_id)
        if self.mock_mode:
            return int(self._tool_samples.get(int(dxl_id), {}).get(
                'torque_state') == 'ON')
        with self._bus_lock:
            return self._read_register(dxl_id, ADDR_TORQUE_ENABLE, 1, 'FSM torque')

    def read_hardware_error(self, dxl_id):
        self._fsm_id_allowed(dxl_id)
        if self.mock_mode:
            return int(self._tool_samples.get(int(dxl_id), {}).get(
                'hardware_error', 0) or 0)
        with self._bus_lock:
            return self._read_register(dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                                       'FSM hardware error')

    def read_model(self, dxl_id):
        self._fsm_id_allowed(dxl_id)
        if self.mock_mode:
            return self._tool_samples.get(int(dxl_id), {}).get('model')
        try:
            with self._bus_lock:
                model, result, error = self.packet_handler.ping(
                    self.port_handler, dxl_id)
        except Exception as exc:
            raise RuntimeError(f'ID {dxl_id} model read transport failed: {exc}') from exc
        if result != 0 or error != 0:
            raise RuntimeError(f'ID {dxl_id} model read failed')
        return model

    def goal_position(self, dxl_id, tick):
        self._fsm_id_allowed(dxl_id)
        if self.read_only:
            raise RuntimeError('read-only bridge rejects goal write')
        if self.mock_mode:
            self._tool_samples[int(dxl_id)]['position'] = int(tick)
            return
        with self._bus_lock:
            self._write_register(dxl_id, ADDR_GOAL_POSITION, 4,
                                 int(tick) & 0xffffffff, 'FSM goal position')

    def command_dual_targets(self, targets):
        """Synchronously dispatch and supervise one FSM endpoint command."""
        return self._command_dual_targets_impl(targets)

    def start_dual_jog(self, targets):
        """Start endpoint-directed motion; GUI release must call HOLD."""
        self._validate_dual_motion_request(targets)
        self._dispatch_dual_sync_targets(targets)

    def hold_dual_position(self):
        """Stop a hold-to-run jog by making fresh positions the new goals."""
        positions = {}
        with self._bus_lock:
            for dxl_id in (3, 4):
                positions[dxl_id] = self._read_tool_state(dxl_id)[0]
        self._validate_dual_motion_request(positions, require_range=False)
        self._dispatch_dual_sync_targets(positions)

    def set_torque(self, dxl_id, enabled):
        self._fsm_id_allowed(dxl_id)
        if self.read_only:
            raise RuntimeError('read-only bridge rejects torque write')
        if self.mock_mode:
            self._tool_samples[int(dxl_id)]['torque_state'] = (
                'ON' if enabled else 'OFF')
            if enabled:
                self.torque_enabled_ids.add(int(dxl_id))
            else:
                self.torque_enabled_ids.discard(int(dxl_id))
            return
        with self._bus_lock:
            if (enabled and self.tool_type == 'spur_1motor_gripper'
                    and int(dxl_id) == 5):
                self._prepare_spur_gripper_enable()
            self._write_register(dxl_id, ADDR_TORQUE_ENABLE, 1,
                                 TORQUE_ENABLE if enabled else TORQUE_DISABLE,
                                 'FSM torque')
        if enabled:
            self.torque_enabled_ids.add(int(dxl_id))
        else:
            self.torque_enabled_ids.discard(int(dxl_id))

    def _prepare_spur_gripper_enable(self):
        """Safely park and configure ID5 immediately before explicit torque-on."""
        if self.tool_type != 'spur_1motor_gripper' or self.tool_ids != [5]:
            raise RuntimeError('spur profile setup is restricted to ID5')
        required_modes = self.tool_profile.get('required_operating_modes') or {}
        required_mode = required_modes.get(5, required_modes.get('5'))
        acceleration = int(self.tool_profile.get('profile_acceleration') or 0)
        velocity = int(self.tool_profile.get('profile_velocity') or 0)
        low = int(self.tool_profile['safe_min_tick'])
        high = int(self.tool_profile['safe_max_tick'])
        if required_mode is None or acceleration <= 0 or velocity <= 0:
            raise RuntimeError('spur mode/profile values are not configured')

        hardware_error = self._read_register(
            5, ADDR_HARDWARE_ERROR_STATUS, 1, 'spur hardware error')
        if hardware_error != 0:
            raise RuntimeError(f'ID5 hardware error: {hardware_error}')
        torque = self._read_register(5, ADDR_TORQUE_ENABLE, 1, 'spur torque')
        if torque != TORQUE_DISABLE:
            raise RuntimeError('actual ID5 torque must be OFF before profile setup')
        mode = self._read_register(5, ADDR_OPERATING_MODE, 1, 'spur mode')
        if mode != int(required_mode):
            raise RuntimeError(
                f'ID5 operating mode {mode} does not match required {required_mode}')
        current_raw = self._read_register(
            5, ADDR_PRESENT_POSITION, 4, 'spur present position')
        current = self._tool_position_tick(5, current_raw)
        if not low <= current <= high:
            raise RuntimeError(
                f'ID5 position {current} outside safe range [{low}, {high}]')

        for address, value, label in (
                (ADDR_PROFILE_ACCELERATION, acceleration,
                 'spur profile acceleration'),
                (ADDR_PROFILE_VELOCITY, velocity, 'spur profile velocity')):
            self._write_register(5, address, 4, value, label)
            actual = self._read_register(5, address, 4, f'{label} readback')
            if actual != value:
                raise RuntimeError(
                    f'{label} readback mismatch: expected {value}, got {actual}')

        # Goal Position may contain a value from an earlier power/session.
        # Park it at the measured position before torque is enabled so the
        # mechanism cannot jump when the control loop engages.
        self._write_register(
            5, ADDR_GOAL_POSITION, 4, current & 0xffffffff,
            'spur current-position goal')
        goal = self._read_register(
            5, ADDR_GOAL_POSITION, 4, 'spur goal readback')
        if self._tool_position_tick(5, goal) != current:
            raise RuntimeError('ID5 current-position goal readback mismatch')

    def _discover_tool_ids(self):
        """Observe every configured actuator; any missing ID closes the backend.

        Discovery is invoked by startup, an explicit GUI request and the
        physical-detector timer.  It must share the serial lock with the
        one-ID probes; otherwise a periodic probe can interleave packets with
        this loop and turn a present motor into a false "not discovered".
        """
        if not self.tool_ids:
            return False
        missing = [dxl_id for dxl_id in self.tool_ids
                   if not self._probe_tool_id(dxl_id)]
        if missing:
            self.get_logger().error(f'tool actuator IDs not discovered: {missing}')
            return False
        return True

    def _probe_tool_id(self, dxl_id):
        """Observe one candidate under the shared serial-bus lock.

        Some installed Protocol 2.0 tools reply reliably to control-table
        reads but intermittently drop the separate ping status packet.  A
        successful hardware-error register read is equally direct proof that
        the requested ID is present; it is only used after ping fails.
        """
        with self._bus_lock:
            _model, result, error = self.packet_handler.ping(
                self.port_handler, int(dxl_id))
            if result == 0 and error == 0:
                return True
            _value, read_result, read_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, int(dxl_id), ADDR_HARDWARE_ERROR_STATUS)
        return read_result == 0 and read_error == 0

    def _rescan_physical_tool(self):
        """Probe every configured tool signature and retain the observation.

        This is intentionally an ID/signature scan, rather than trusting the
        GUI's selected profile.  It is used both by the periodic detector and
        by an explicit tool-change request, so the latter is a real rescan as
        an operator reasonably expects.
        """
        provider = self._bus_tool_identity
        if provider is None or self.mock_mode or not self.port_connected:
            return None
        detected = provider.detected_tool_type()
        self._tool_detection_reason = provider.last_reason
        return detected

    def _observe_active_tool_signature(self):
        """Cheap steady-state check before a full replacement scan.

        Full signature discovery pings every supported profile ID.  On a
        sparse bus, each absent ID can consume a serial timeout while holding
        ``_bus_lock``.  Running that scan every 0.5 seconds made a cleaner
        START/STOP command wait behind the detector.  A fitted active tool
        only needs its own complete signature checked; scan every candidate
        only after that signature disappears.
        """
        provider = self._bus_tool_identity
        active_ids = frozenset(
            int(dxl_id) for dxl_id in getattr(self, 'tool_ids', ()))
        if (provider is None or not active_ids
                or getattr(provider, '_signatures', {}).get(self.tool_type)
                != active_ids):
            return None
        # Fresh feedback is stronger evidence than a separate ping.  The
        # dual XL430 profile can answer the read registers while a concurrent
        # protocol ping loses its status packet after a runtime tool swap.
        # Do not turn that transient detector failure into a detach latch and
        # torque-off event.  A real removal clears these samples on the next
        # feedback cycle, then the normal ping/rescan path still detects it.
        samples = [self._tool_samples.get(dxl_id, {}) for dxl_id in active_ids]
        if all(sample.get('online') is True
               and sample.get('hardware_error') == 0
               and sample.get('position') is not None
               for sample in samples):
            provider.last_present_ids = active_ids
            provider.last_reason = ''
            self._tool_detection_reason = ''
            return self.tool_type
        if not all(self._probe_tool_id(dxl_id) for dxl_id in active_ids):
            return None
        provider.last_present_ids = active_ids
        provider.last_reason = ''
        self._tool_detection_reason = ''
        return self.tool_type

    def _revalidate_current_tool(self):
        """Handle an explicit request for the already selected tool safely.

        A same-name request used to return immediately.  That left a freshly
        reconnected actuator offline until the background poll happened to run
        and made the GUI misleadingly report an undetected motor.  Re-register
        feedback and re-run FSM validation here, but never enable torque.
        """
        if self.mock_mode:
            return
        if not self.port_connected:
            self.tool_discovered = False
            self.tool_motion_allowed = False
            raise RuntimeError('tool bus is not connected')
        self.tool_discovered = self._discover_tool_ids()
        if not self.tool_discovered:
            self.tool_motion_allowed = False
            raise RuntimeError(
                f'{self.tool_type} re-scan found none of expected actuator IDs '
                f'{self.tool_ids}')
        for dxl_id in self.tool_ids:
            self.group_sync_read.addParam(dxl_id)
            self.active_ids.add(dxl_id)
        self.tool_motion_allowed = bool(
            self.tool_selection and self.tool_selection.valid)
        if self.tool_fsm is not None:
            state = self.tool_fsm.startup()
            if state == ToolState.FAULT:
                raise RuntimeError(self.tool_fsm.fault_reason or
                                   'tool re-scan validation failed')

    def _confirmed_tool_observation(self, detected):
        """Return true once one observation is stable for the configured count."""
        if detected != self._tool_detection_observation:
            self._tool_detection_observation = detected
            self._tool_detection_count = 1
        else:
            self._tool_detection_count += 1
        return self._tool_detection_count >= self.tool_detection_confirmations

    def _automatic_switch_safe(self):
        if self._gripper_goal_active:
            return False
        if self.control_scope == 'END_EFFECTOR_ONLY':
            return True
        return self._arm_fsm_state in ToolManager.SAFE_CHANGE_STATES

    def _poll_physical_tool(self):
        """Fail closed on detach, then switch only after a stable new signature."""
        provider = self._bus_tool_identity
        if provider is None or self.mock_mode or not self.port_connected:
            return
        if self.tool_type not in provider.supported_tool_types:
            self._tool_detection_reason = (
                f'{self.tool_type} has no physical actuator signature')
            return
        # While the fitted tool is present this is one (or two for the dual
        # gripper) fast pings.  A missing signature falls back to the complete
        # scan so unplug/replacement detection remains unchanged.
        detected = self._observe_active_tool_signature()
        if detected is None:
            detected = self._rescan_physical_tool()
        if not self._confirmed_tool_observation(detected):
            return

        if not self._physical_tool_detached:
            if detected == self.tool_type:
                self._tool_detection_reason = ''
                return
            # A different/ambiguous signature cannot switch directly.  First
            # latch physical removal and stop the selected tool fail-closed.
            if not self._tool_change_lock.acquire(blocking=False):
                return
            try:
                self._physical_tool_detached = True
                self.tool_discovered = False
                self._tool_detection_reason = provider.last_reason or (
                    f'physical tool changed: {self.tool_type}->{detected}')
                self._stop_tool('physical end-effector removal detected')
                self._tool_detection_observation = None
                self._tool_detection_count = 0
            finally:
                self._tool_change_lock.release()
            return

        if detected is None:
            self._tool_detection_reason = provider.last_reason
            return
        # A stable, supported signature is physical evidence that a replacement
        # tool is now fitted.  It is safe to retire the *tool-detached* latch
        # here: torque stays off until the new profile/configuration succeeds.
        # An emergency-stop latch is deliberately different and survives any
        # tool swap; it still requires the explicit program restart path.
        if self.emergency_stop_active:
            self._tool_detection_reason = 'automatic switch blocked by emergency stop latch'
            return
        if self.tool_detached:
            self.tool_detached = False
            self._tool_detection_reason = 'confirmed replacement tool clears detach latch'
        if not self._automatic_switch_safe():
            self._tool_detection_reason = (
                f'automatic switch waiting for safe arm state; '
                f'current={self._arm_fsm_state or "UNKNOWN"}')
            return
        if not self._tool_change_lock.acquire(blocking=False):
            return
        try:
            self._tool_change_pending = detected
            self._tool_change_error = ''
            try:
                if detected == self.tool_type:
                    # A same-type reattachment is observation-only.  Restore
                    # registration/FSM validation but never re-enable torque.
                    self.tool_discovered = self._discover_tool_ids()
                    if not self.tool_discovered:
                        raise RuntimeError('reattached tool discovery failed')
                    for dxl_id in self.tool_ids:
                        self.group_sync_read.addParam(dxl_id)
                        self.active_ids.add(dxl_id)
                    self.tool_motion_allowed = bool(
                        self.tool_selection and self.tool_selection.valid)
                    if self.tool_fsm is not None:
                        state = self.tool_fsm.startup()
                        if state == ToolState.FAULT:
                            raise RuntimeError(
                                self.tool_fsm.fault_reason or
                                'reattached tool validation failed')
                else:
                    self._switch_tool_runtime(detected)
                self._physical_tool_detached = False
                self._tool_detection_reason = ''
            except Exception as exc:
                self._tool_change_error = str(exc)
                self._tool_detection_reason = str(exc)
                self.get_logger().error(
                    f'automatic tool change rejected: {exc}')
            finally:
                self._tool_change_pending = None
        finally:
            self._tool_change_lock.release()

    def _configure_tool_actuators(self):
        """Apply profile motion limits only after strict validation and discovery."""
        if self.tool_profile.get('backend') == 'cleaner':
            return
        modes = self.tool_profile.get('required_operating_modes', {})
        for dxl_id in self.tool_ids:
            mode = modes.get(dxl_id, modes.get(str(dxl_id), 3))
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE, int(mode))
            if result != 0 or error != 0:
                self.tool_motion_allowed = False
                self.get_logger().error(f'operating mode setup failed: id={dxl_id}')
                return
            for address, value in (
                    (ADDR_PROFILE_ACCELERATION,
                     self.tool_profile['profile_acceleration']),
                    (ADDR_PROFILE_VELOCITY,
                     self.tool_profile['profile_velocity'])):
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, address, int(value))
            goal_pwm = int(self.tool_profile.get('goal_pwm', 0))
            if goal_pwm > 0:
                self.packet_handler.write2ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_PWM, goal_pwm)
            if self._enable_torque(dxl_id, f'{self.tool_type} tool'):
                self.group_sync_read.addParam(dxl_id)
                self.active_ids.add(dxl_id)
            else:
                self.tool_motion_allowed = False

    def _stop_tool(self, reason):
        """Best-effort stop for emergency, detach, cancellation, and shutdown."""
        self.tool_motion_allowed = False
        self.cleaning_running = False
        if isinstance(getattr(self, 'tool_fsm', None), CleanerFSM):
            self.tool_fsm.state = ToolState.STOPPED
            for sample in self._tool_samples.values():
                sample['velocity'] = 0
        if self.mock_mode or self.read_only:
            return
        with self._bus_lock:
            for dxl_id in self.tool_ids:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_VELOCITY, 0)
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        tool_fsm = getattr(self, 'tool_fsm', None)
        if tool_fsm is not None:
            tool_fsm.state = ToolState.STOPPED
        self.get_logger().warn(f'tool actuator stopped: {reason}')

    def _on_emergency_stop(self, msg):
        if msg.data:
            self.emergency_stop_active = True
            self._stop_tool('emergency stop')

    def _on_tool_detached(self, msg):
        if msg.data:
            self.tool_detached = True
            self._stop_tool('tool detach')

    def _on_control_mode(self, msg):
        mode = msg.data.strip().upper()
        if mode not in ('MANUAL', 'FSM'):
            self.get_logger().warn(f'unknown control mode ignored: {msg.data!r}')
            return
        self.control_mode = mode

    def _on_control_mode_request(self, msg):
        """Echo ownership requests; this is ROS state only, never a motor write."""
        self._on_control_mode(msg)
        self.control_mode_status_pub.publish(String(data=self.control_mode))

    def _on_arm_fsm_state(self, msg):
        self._arm_fsm_state = str(msg.data).strip().upper()

    def _publish_tool_status_safely(self):
        """Keep a status serialization fault from silently stopping updates."""
        try:
            self.publish_tool_status()
        except Exception as exc:  # pragma: no cover - hardware/runtime guard
            self.get_logger().error(f'/tool/status publish failed: {exc}')

    def tool_change_callback(self, msg):
        """Rebuild the selected gripper profile and FSM without restarting ROS."""
        requested = str(msg.data).strip()
        with self._tool_change_lock:
            self._tool_change_pending = requested
            self._tool_change_error = ''
            try:
                self._switch_tool_runtime(requested)
            except Exception as exc:
                self._tool_change_error = str(exc)
                self.get_logger().error(f'runtime tool change rejected: {exc}')
            finally:
                self._tool_change_pending = None

    def _switch_tool_runtime(self, requested):
        supported = ('spur_1motor_gripper', 'dual_motor_gripper', 'cleaner')
        if requested not in supported:
            raise ToolProfileError(
                f'runtime switching supports only {supported}, got {requested!r}')
        if requested == self.tool_type:
            self._revalidate_current_tool()
            return
        if self.emergency_stop_active or self.tool_detached:
            raise RuntimeError('runtime tool change blocked by emergency stop or detached latch')
        if self._gripper_goal_active:
            raise RuntimeError('runtime tool change blocked while gripper motion is active')
        old_fsm = self.tool_fsm
        if self.tool_type == 'cleaner':
            old_fsm = None  # Switching stops the velocity adapter before unregistering it.
        if old_fsm and old_fsm.state not in (
                ToolState.READY, ToolState.OPEN, ToolState.CLOSED,
                ToolState.CALIBRATION_REQUIRED, ToolState.STOPPED):
            offline_safe = (old_fsm.state == ToolState.FAULT
                            and not self.tool_discovered
                            and not (self.torque_enabled_ids & set(self.tool_ids)))
            if not offline_safe:
                raise RuntimeError(
                    f'runtime tool change unavailable in {old_fsm.state.name}')

        profiles = load_profiles(self.get_parameter('tool_profile_file').value)
        manager = ToolManager(
            profiles, ParameterToolIdentityProvider(requested),
            mock_mode=self.mock_mode)
        selection = manager.refresh('IDLE')
        new_ids = [int(item) for item in selection.profile.get('actuator_ids', [])]
        expected_ids = {'spur_1motor_gripper': [5], 'dual_motor_gripper': [3, 4]}.get(requested, new_ids)
        if new_ids != expected_ids:
            raise ToolProfileError(
                f'{requested} profile actuator_ids must be {expected_ids}, got {new_ids}')

        if requested == 'cleaner' and new_ids:
            if len(new_ids) != 1 or len(selection.profile.get('joint_names', [])) != 1:
                raise ToolProfileError('cleaner requires one actuator and one joint')
            if new_ids[0] in ARM_IDS:
                raise ToolProfileError('cleaner actuator conflicts with an arm joint')

        # An operator's request is also an explicit physical rescan.  Do this
        # before stopping/unregistering the active tool so a wrong selection
        # cannot leave a healthy tool half-switched.  The error names the
        # observed IDs instead of incorrectly calling the motor "undetected".
        if not self.mock_mode and self.port_connected:
            detected = self._rescan_physical_tool()
            present = sorted(self._bus_tool_identity.last_present_ids)
            if detected != requested:
                raise ToolProfileError(
                    f'requested {requested} expects actuator IDs {new_ids}; '
                    f'bus re-scan found IDs {present} '
                    f'({detected or self._bus_tool_identity.last_reason})')

        # Stop and unregister the old tool before changing the allowlist.  This
        # New cleaner velocity-mode setup follows discovery below.
        old_ids = list(self.tool_ids)
        old_torque_on = (bool(self.torque_enabled_ids & set(old_ids))
                         or bool(getattr(self, 'cleaning_running', False))
                         or any(sample.get('torque_state') == 'ON'
                                for sample in getattr(self, '_tool_samples', {}).values()))
        if old_torque_on:
            self._stop_tool(f'runtime tool change to {requested}')
        else:
            self.tool_motion_allowed = False
            if old_fsm is not None:
                old_fsm.state = ToolState.STOPPED
        for dxl_id in old_ids:
            try:
                self.group_sync_read.delParam(dxl_id)
            except Exception:
                pass
            self.active_ids.discard(dxl_id)
            self.torque_enabled_ids.discard(dxl_id)

        self.cleaning_running = False
        self.cleaning_configured = False
        self.cleaning_actuator_id = -1
        self.cleaning_actuator_joint = ''
        self.cleaning_direction = 0
        self.cleaning_velocity_raw = 0
        if requested == 'cleaner':
            self.cleaning_actuator_id = new_ids[0] if new_ids else -1
            self.cleaning_actuator_joint = (selection.profile.get('joint_names') or [''])[0]
            self.cleaning_direction = int(selection.profile.get('direction') or 0)
            self.cleaning_velocity_raw = int(selection.profile.get('profile_velocity') or 0)
            self.cleaning_configured = bool(
                selection.valid and new_ids and self.cleaning_actuator_joint
                and self.cleaning_direction in (-1, 1) and self.cleaning_velocity_raw > 0)

        if hasattr(self, 'spur_manual_control'):
            self.spur_manual_control.deadline = None
        self.tool_type = requested
        self.tool_manager = manager
        self.tool_selection = selection
        self.tool_profile = selection.profile
        self.tool_ids = new_ids
        self.gripper_ids = new_ids if requested != 'cleaner' else []
        self.gripper_joints = list(self.tool_profile.get('joint_names', []))
        self.gripper_open_rad = float(self.tool_profile.get('open_position', 1.0))
        self.gripper_close_rad = float(self.tool_profile.get('close_position', 0.0))
        self.tool_motion_allowed = bool(selection.valid)
        self.tool_discovered = self.mock_mode
        self._tool_samples = {}
        if self.mock_mode:
            self.active_ids.update(self.tool_ids)
            joint_names = self.tool_profile.get('joint_names') or ['']
            endpoints = self.tool_profile.get('motor_endpoints') or {}
            for dxl_id in self.tool_ids:
                endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
                if endpoint:
                    seed_tick = int(round((endpoint['open'] + endpoint['close']) / 2))
                else:
                    seed_tick = int(self.tool_profile.get('open_tick', 0))
                self._tool_samples[dxl_id] = {
                    'id': dxl_id, 'joint': joint_names[0],
                    'position': seed_tick, 'effort': 0.0, 'online': True,
                    'hardware_error': 0, 'torque_state': 'OFF'}
        elif self.port_connected:
            self.tool_discovered = self._discover_tool_ids()
            if self.tool_discovered:
                for dxl_id in self.tool_ids:
                    self.group_sync_read.addParam(dxl_id)
                    self.active_ids.add(dxl_id)
        else:
            self.tool_motion_allowed = False

        if (requested == 'cleaner' and self.cleaning_configured
                and self.tool_discovered and not self.mock_mode and not self.read_only):
            self._configure_cleaning_actuator()

        self._fsm_allowlist = set()
        # Retain the existing mission FSM and its /cleaning/enable adapter.
        self.tool_fsm = None
        if requested == 'cleaner':
            self.tool_fsm = CleanerFSM(self.tool_profile, self)
            self.tool_fsm.startup()
        else:
            self.tool_fsm = manager.create_fsm(self)
            self.tool_fsm.startup()
        self.calibration_session = None
        self.dual_manual_recovery = None
        self.dual_calibration_session = None
        if requested == 'spur_1motor_gripper':
            self.calibration_session = CalibrationSession(self, self.tool_profile)
        elif requested == 'dual_motor_gripper':
            self.dual_manual_recovery = DualManualRecovery(self)
            self.dual_calibration_session = DualCalibrationSession(
                self, self.tool_profile)
        self.get_logger().info(
            f'runtime tool change complete: tool_type={requested}, ids={self.tool_ids}, '
            f'fsm={self.tool_fsm.__class__.__name__}')

    def publish_tool_status(self):
        self.tool_type_pub.publish(String(data=self.tool_type))
        reason = ''
        if not self.tool_selection:
            reason = 'profile load failed'
        elif not self.tool_selection.valid:
            reason = self.tool_selection.reason
        elif not self.tool_discovered and not self.mock_mode:
            reason = 'actuator not discovered'
        elif self.read_only:
            reason = 'read-only diagnostic mode'
        id5 = self._tool_samples.get(5, {})
        torque_states = [self._tool_samples.get(dxl_id, {}).get(
            'torque_state', 'UNKNOWN') for dxl_id in self.tool_ids]
        if torque_states and all(state == 'ON' for state in torque_states):
            tool_torque_state = 'ON'
        elif torque_states and all(state == 'OFF' for state in torque_states):
            tool_torque_state = 'OFF'
        elif any(state == 'UNKNOWN' for state in torque_states):
            tool_torque_state = 'UNKNOWN'
        else:
            tool_torque_state = 'MIXED'
        fsm_fault = (self.tool_fsm.fault_reason
                     if self.tool_fsm and self.tool_fsm.state.name == 'FAULT'
                     else None)
        synchronization = {'state': 'NOT_APPLICABLE', 'spread': None,
                           'limit': 0.05, 'recalibration_required': False}
        if self.tool_type == 'dual_motor_gripper':
            positions = {dxl_id: self._tool_samples.get(dxl_id, {}).get('position')
                         for dxl_id in (3, 4)}
            try:
                if any(value is None for value in positions.values()):
                    raise RuntimeError('dual position feedback unavailable')
                _fractions, spread = self._dual_normalized_spread(positions)
                synchronized = spread <= 0.05
                synchronization = {
                    'state': 'SYNCHRONIZED' if synchronized else 'FAULT',
                    'spread': spread, 'limit': 0.05,
                    'recalibration_required': not synchronized}
            except RuntimeError as exc:
                synchronization = {
                    'state': 'UNKNOWN', 'spread': None, 'limit': 0.05,
                    'recalibration_required': True, 'reason': str(exc)}
        status = {
            'control_scope': self.control_scope,
            'tool_type': self.tool_type,
            'tool_profile': self.tool_profile,
            'backend': self.tool_profile.get('backend', 'invalid'),
            'profile_valid': bool(self.tool_selection and self.tool_selection.valid),
            'calibrated': bool(self.tool_profile.get('calibrated')),
            'endpoint_calibration_verified': bool(
                self.tool_profile.get('endpoint_calibration_verified')),
            'temporary_jog_mode': self.temporary_jog_enabled,
            'developer_direct_mode': self.developer_direct_mode,
            'temporary_jog_ready': self._tool_backend_ready(),
            'tool_enable_allowed': self._tool_enable_allowed(),
            # This is a register observation, never the bridge's ownership
            # bookkeeping.  UNKNOWN stays distinct from OFF in the GUI.
            'tool_torque_state': tool_torque_state,
            'tool_torque_enabled': tool_torque_state == 'ON',
            'actuators_discovered': self.tool_discovered,
            'motion_allowed': self._tool_backend_ready(),
            'read_only': self.read_only, 'mock_mode': self.mock_mode,
            'bridge_connected': True,
            'u2d2_connected': self.port_connected,
            'control_mode': self.control_mode,
            'emergency_stop': self.emergency_stop_active,
            'tool_detached': self.tool_detached,
            'physical_tool_detached': self._physical_tool_detached,
            'automatic_detection': {
                'enabled': bool(self._bus_tool_identity),
                'present_ids': (sorted(self._bus_tool_identity.last_present_ids)
                                if self._bus_tool_identity else []),
                'reason': self._tool_detection_reason,
                'arm_fsm_state': self._arm_fsm_state,
            },
            'actuators': [self._tool_samples.get(dxl_id, {
                'id': dxl_id, 'joint': '', 'position': None,
                'effort': 0.0 if self.mock_mode else None,
                'online': self.mock_mode}) for dxl_id in self.tool_ids],
            'reason': reason,
            'calibration_jog_enabled': bool(self.calibration_session),
            'calibration': (self.calibration_session.snapshot()
                            if self.calibration_session else None),
            'dual_calibration': (self.dual_calibration_session.snapshot()
                                 if self.dual_calibration_session else None),
            'fsm_state': (self.tool_fsm.state.name if self.tool_fsm else
                          (self.dual_calibration_session.state
                           if self.dual_calibration_session else None)),
            # Flat ID5 fields are retained alongside the older actuator list
            # so a GUI can safely render a read-only single-motor diagnosis.
            'actuator_id': 5 if self.tool_ids == [5] else None,
            'online': id5.get('online'),
            'position': id5.get('position'),
            'torque_enabled': ({'ON': True, 'OFF': False}.get(
                id5.get('torque_state'))),
            'hardware_error': id5.get('hardware_error'),
            'model': id5.get('model'),
            'fault': fsm_fault,
            'synchronization': synchronization,
            'cleaning_running': self.cleaning_running,
            'cleaning_configured': self.cleaning_configured,
            'tool_change': {
                'pending': self._tool_change_pending is not None,
                'requested': self._tool_change_pending,
                'error': self._tool_change_error,
            },
        }
        self.tool_status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def _tool_actuators_online(self):
        """Return true only when every profile-selected actuator is online."""
        if not self.tool_ids:
            return False
        return all(
            (self._tool_samples.get(dxl_id) or {}).get('id') == dxl_id
            and bool((self._tool_samples.get(dxl_id) or {}).get('online'))
            for dxl_id in self.tool_ids)

    def _tool_backend_ready(self):
        if self.tool_type == 'cleaner' and self.mock_mode:
            return bool(self.tool_motion_allowed and not self.read_only
                        and not self.emergency_stop_active and not self.tool_detached)
        if self.tool_type == 'spur_1motor_gripper' and self.tool_ids == [5]:
            return bool(
                self._tool_enable_allowed()
                and self._tool_samples.get(5, {}).get('online')
                and self._tool_samples.get(5, {}).get('torque_state') == 'ON')
        dual_ready = (self.dual_calibration_session is None
                      or self.dual_calibration_session.is_ready)
        return bool(
            self._tool_enable_allowed()
            and set(self.tool_ids).issubset(self.torque_enabled_ids)
            and dual_ready)

    def _calibration_motion_ready(self):
        """Fail closed unless ID5 is already safely configured by the operator.

        No torque/mode/profile write is made here.  The velocity and
        acceleration registers must already hold the conservative values.
        """
        sample = self._tool_samples.get(5, {})
        return bool(
            self.calibration_jog_enabled and not self.read_only
            and self.tool_ids == [5] and self.tool_discovered
            and sample.get('online') and sample.get('torque_state') == 'ON'
            and sample.get('operating_mode') == MODE_POSITION
            and sample.get('profile_velocity') is not None
            and sample.get('profile_acceleration') is not None
            and 0 < sample['profile_velocity'] <= 5
            and 0 < sample['profile_acceleration'] <= 1
            and not self.emergency_stop_active and not self.tool_detached)

    def _tool_enable_allowed(self):
        """Safety gate for explicit torque enable; does not require torque yet."""
        if self.mock_mode:
            return True
        profile_ready = bool(self.tool_selection and self.tool_selection.valid
                             and self.tool_profile.get('calibrated'))
        temporary_ready = bool(
            self.temporary_jog_enabled
            and self.tool_type == 'spur_1motor_gripper'
            and self.tool_ids == [5])
        return bool(
            (profile_ready or temporary_ready)
            and self.tool_discovered and self._tool_actuators_online()
            and self.tool_motion_allowed and not self.read_only
            and not self.emergency_stop_active and not self.tool_detached)

    def _configure_temporary_jog_actuator(self):
        """Register only ID 5 for a guarded, torque-off bench session."""
        if self.tool_ids != [5]:
            self.tool_motion_allowed = False
            return
        dxl_id = self.tool_ids[0]
        if (self.temporary_jog_profile_velocity <= 0
                or self.temporary_jog_profile_acceleration <= 0):
            self.tool_motion_allowed = False
            self.get_logger().error('temporary jog profile must be positive')
            return
        with self._bus_lock:
            # Operating-mode/profile writes require torque OFF.  Do not call
            # _enable_torque here: activation must be a deliberate GUI action.
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            mode_result, mode_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE, 3)
            if mode_result != 0 or mode_error != 0:
                self.tool_motion_allowed = False
                self.get_logger().error('temporary jog position-mode setup failed')
                return
            for address, value in (
                    (ADDR_PROFILE_ACCELERATION,
                     self.temporary_jog_profile_acceleration),
                    (ADDR_PROFILE_VELOCITY,
                     self.temporary_jog_profile_velocity)):
                result, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, address, value)
                if result != 0 or error != 0:
                    self.tool_motion_allowed = False
                    self.get_logger().error('temporary jog profile setup failed')
                    return
        self.group_sync_read.addParam(dxl_id)
        self.active_ids.add(dxl_id)
        self.torque_enabled_ids.discard(dxl_id)
        self.get_logger().info(
            'spur temporary jog ready: ID 5 is torque-disabled; '
            'use the GUI Enable button before sending a goal')

    def _configure_cleaning_actuator(self):
        """Dynamixel Protocol 2.0 velocity mode(Operating Mode=1)로 설정한다."""
        dxl_id = self.cleaning_actuator_id
        try:
            with self._bus_lock:
                self._write_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1, TORQUE_DISABLE,
                    'cleaner torque disable')
                self._write_register(
                    dxl_id, ADDR_OPERATING_MODE, 1, 1,
                    'cleaner velocity mode')
                # A replacement motor can retain a non-zero Goal Velocity
                # from a previous owner.  Zero/read it before torque-on so a
                # profile transition never starts the cleaner by itself.
                self._write_register(
                    dxl_id, ADDR_GOAL_VELOCITY, 4, 0,
                    'cleaner zero goal velocity')
                if self._read_register(
                        dxl_id, ADDR_GOAL_VELOCITY, 4,
                        'cleaner zero goal velocity readback', signed=True) != 0:
                    raise RuntimeError('cleaner zero goal velocity readback failed')
        except Exception as exc:
            self.get_logger().error(
                f"Cleaning actuator velocity-mode setup failed: id={dxl_id}: {exc}")
            self.cleaning_configured = False
            return
        if self._enable_cleaner_torque(dxl_id):
            self.group_sync_read.addParam(dxl_id)
            self.active_ids.add(dxl_id)
        else:
            self.cleaning_configured = False

    def _enable_cleaner_torque(self, dxl_id):
        """Enable a velocity-mode cleaner without position-goal synchronization."""
        try:
            with self._bus_lock:
                if self._read_register(
                        dxl_id, ADDR_TORQUE_ENABLE, 1,
                        'cleaner torque preflight') != TORQUE_DISABLE:
                    raise RuntimeError('cleaner torque must be OFF before setup')
                if self._read_register(
                        dxl_id, ADDR_OPERATING_MODE, 1,
                        'cleaner velocity mode readback') != 1:
                    raise RuntimeError('cleaner operating mode is not velocity control')
                for address, value, label in (
                        (ADDR_PROFILE_ACCELERATION,
                         int(self.tool_profile['profile_acceleration']),
                         'cleaner profile acceleration'),
                        (ADDR_PROFILE_VELOCITY,
                         int(self.tool_profile['profile_velocity']),
                         'cleaner profile velocity')):
                    self._write_register(dxl_id, address, 4, value, label)
                    if self._read_register(
                            dxl_id, address, 4, f'{label} readback') != value:
                        raise RuntimeError(f'{label} readback mismatch')
                # Recheck immediately before torque-on; nothing may have
                # restored an old non-zero speed during the setup sequence.
                if self._read_register(
                        dxl_id, ADDR_GOAL_VELOCITY, 4,
                        'cleaner goal velocity preflight', signed=True) != 0:
                    raise RuntimeError('cleaner goal velocity is not zero')
                self._write_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1, TORQUE_ENABLE,
                    'cleaner torque enable')
                if self._read_register(
                        dxl_id, ADDR_TORQUE_ENABLE, 1,
                        'cleaner torque enable readback') != TORQUE_ENABLE:
                    raise RuntimeError('cleaner torque enable readback failed')
        except Exception as exc:
            self.get_logger().error(
                f'Cleaning torque enable blocked: id={dxl_id}: {exc}')
            return False
        self.torque_enabled_ids.add(int(dxl_id))
        self.get_logger().info(f'Cleaning torque enabled safely: id={dxl_id}')
        return True

    def _cleaner_direction_command(self, command):
        if command == 'STOP':
            self._on_cleaning_enable(Bool(data=False))
            return
        direct_bench = bool(
            self.developer_direct_mode
            and self.control_scope == 'END_EFFECTOR_ONLY')
        if (command not in ('LEFT', 'RIGHT')
                or (not direct_bench and self.control_mode != 'MANUAL')):
            return
        if not self.cleaning_configured or not self.tool_discovered:
            self.get_logger().warn('Cleaner direction command requires a configured actuator')
            return
        self._on_cleaning_enable(Bool(data=True), 1 if command == 'LEFT' else -1)

    def _on_cleaning_direction(self, msg):
        """Dedicated immediate LEFT/RIGHT/STOP ingress for the GUI buttons."""
        self._cleaner_direction_command(str(msg.data).strip().upper())

    def _on_cleaning_enable(self, msg, rotation=1):
        if self.tool_type != 'cleaner' or self.read_only:
            return
        direct_bench = bool(
            self.developer_direct_mode
            and self.control_scope == 'END_EFFECTOR_ONLY')
        # Developer direct mode deliberately skips ownership/FSM readiness so
        # a bench START/STOP has no status-poll or mode-transition wait.  It
        # does not bypass the physical tool identity, read-only mode, e-stop,
        # detach latch, configured actuator, or the shared serial-bus lock.
        if msg.data and (self.emergency_stop_active or self.tool_detached
                         or (not direct_bench and not self.tool_motion_allowed)
                         or (not direct_bench
                             and self.control_mode not in ('MANUAL', 'FSM'))):
            return
        if self.mock_mode:
            self.cleaning_running = bool(msg.data)
            if isinstance(self.tool_fsm, CleanerFSM):
                self.tool_fsm.observe_command(msg.data)
            for sample in self._tool_samples.values():
                sample['velocity'] = (rotation * self.cleaning_direction * self.cleaning_velocity_raw
                                      if msg.data else 0)
            return
        if (not self.cleaning_configured or not self.tool_discovered
                or (msg.data and not direct_bench
                    and not self._tool_backend_ready())):
            self.get_logger().error('Cleaning actuator/profile is not ready')
            return
        velocity = (rotation * self.cleaning_direction * self.cleaning_velocity_raw
                    if msg.data else 0)
        self._submit_cleaner_velocity(int(velocity))

    def _submit_cleaner_velocity(self, velocity):
        """Preempt older cleaner requests; perform only the latest write.

        The callback that owns the mailbox may be writing one Protocol 2.0
        packet already.  Concurrent button presses only replace the desired
        velocity, then that owner loops once more with the newest value.  This
        makes STOP and direction changes interruptible without concurrent
        writes to the serial port.
        """
        with self._cleaner_command_state_lock:
            self._cleaner_command_generation += 1
            self._cleaner_requested_velocity = int(velocity)
            if self._cleaner_command_active:
                return
            self._cleaner_command_active = True
        completed = False
        try:
            while True:
                with self._cleaner_command_state_lock:
                    generation = self._cleaner_command_generation
                    requested = self._cleaner_requested_velocity
                    actuator_id = self.cleaning_actuator_id
                if (self.tool_type != 'cleaner'
                        or actuator_id < 0 or not self.cleaning_configured):
                    return
                try:
                    # This remains serialized with feedback and detection,
                    # but no stale motion command can be queued behind it.
                    with self._bus_lock:
                        self._write_register(
                            actuator_id, ADDR_GOAL_VELOCITY, 4,
                            requested & 0xffffffff,
                            'cleaner goal velocity')
                except Exception as exc:
                    self.get_logger().error(
                        f'Cleaning velocity write failed: {exc}')
                    return
                self.cleaning_running = bool(requested)
                if isinstance(self.tool_fsm, CleanerFSM):
                    self.tool_fsm.observe_command(bool(requested))
                with self._cleaner_command_state_lock:
                    if generation == self._cleaner_command_generation:
                        self._cleaner_command_active = False
                        completed = True
                        return
        finally:
            # Covers exceptions/early returns before the normal completion
            # path.  Holding the state lock closes the submit-vs-release race.
            if not completed:
                with self._cleaner_command_state_lock:
                    self._cleaner_command_active = False

    def rad_to_tick(self, joint_name, rad):
        """관절 rad → 서보 tick. 안전 리밋 clamp 후 기어비를 곱해 서보축 도메인으로 올린다.

        tick 범위 clamp(아래)는 "서보가 표현할 수 있는 값" 일 뿐 "관절이 안 부딪히는
        범위" 가 아니다 — 그래서 joint_limits 를 **먼저** 적용한다.
        """
        config = JOINT_CONFIG[joint_name]
        # clamp 를 조용히 하면 IK 버그가 "왜 목표에 안 닿지?" 로만 보인다 — 반드시 남긴다.
        rad, was_clamped = joint_limits.clamp(joint_name, rad)
        if was_clamped:
            lower, upper = joint_limits.get_limits(joint_name)
            self.get_logger().warn(
                f"{joint_name}: 목표각이 안전 범위를 벗어나 clamp 됨 "
                f"→ {rad:+.4f} rad (범위 [{lower:+.4f}, {upper:+.4f}])"
            )
        tick = int(round(calib_math.rad_to_tick(
            self._joint_center(joint_name), config["direction"],
            self._joint_gear_ratio(joint_name), rad)))
        if config["extended"]:
            return max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def tick_to_rad(self, joint_name, tick):
        """서보 tick → 관절 rad. rad_to_tick 의 역변환."""
        return calib_math.tick_to_rad(
            self._joint_center(joint_name), JOINT_CONFIG[joint_name]["direction"],
            self._joint_gear_ratio(joint_name), tick)

    def int_to_little_endian_4bytes(self, value):
        return [
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF,
        ]

    def _tool_position_tick(self, dxl_id, raw_tick):
        endpoints = self.tool_profile.get('motor_endpoints') or {}
        endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id), {}))
        if any(value is not None and value < 0 for value in endpoint.values()):
            return to_signed(raw_tick, LEN_PRESENT_POSITION)
        return int(raw_tick)

    def goal_callback(self, goal_request):
        if self.control_scope == 'END_EFFECTOR_ONLY':
            self.get_logger().warn(
                'Arm trajectory rejected: control_scope=END_EFFECTOR_ONLY')
            return GoalResponse.REJECT
        if (self.read_only or not self.tool_selection
                or not self.tool_selection.valid
                or (not self.mock_mode and not self.tool_motion_allowed)):
            self.get_logger().warn(
                "Arm trajectory rejected: read-only or tool interlock not ready")
            return GoalResponse.REJECT
        self.get_logger().info("Received FollowJointTrajectory goal")
        return GoalResponse.ACCEPT

    def _gripper_commands_allowed(self):
        """캘리브레이션된 위치 명령 동작일 때만 참을 반환한다."""
        if (not self.gripper_command_calibrated
                or self.gripper_open_rad == self.gripper_close_rad):
            return False
        if self.gripper_required_operating_modes:
            return (set(self.gripper_required_operating_modes)
                    == set(self.gripper_ids)
                    and self.gripper_observed_operating_modes
                    == self.gripper_required_operating_modes)
        return (self.gripper_observed_operating_mode
                == self.gripper_required_operating_mode)

    def _require_gripper_command_mapping(self):
        """안전성이 확인되지 않은 rad→tick 변환을 fail-closed로 차단한다."""
        if not self.gripper_command_calibrated:
            raise RuntimeError("gripper command calibration is not verified")
        if self.gripper_open_rad == self.gripper_close_rad:
            raise RuntimeError("gripper open/close rad endpoints are identical")

    def _gripper_startup_torque_allowed(self):
        """듀얼 그리퍼에만 기존 기동 동작을 허용한다."""
        return self.end_effector_kind == "gripper" \
            and self._gripper_commands_allowed()

    def rotate_goal_callback(self, goal_request):
        """선택한 단일축 회전 프리셋에 대해서만 회전을 수락한다."""
        if (self.read_only or self.end_effector_kind != "rotary"
                or self.gripper_ids != [5]
                or not self._gripper_commands_allowed()):
            self.get_logger().warn(
                "Rejecting rotate goal: rotary_id5 preset is not active")
            return GoalResponse.REJECT
        if goal_request.max_abs_current < 0 or goal_request.timeout < 0.0:
            return GoalResponse.REJECT
        if not goal_request.relative and not 0 <= goal_request.ticks <= 4095:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def arm_test_goal_callback(self, goal_request):
        """명시적으로 승인된 4축 tick 시퀀스만 수락한다."""
        requested = tuple(zip(goal_request.motor_ids, goal_request.delta_ticks))
        fixed_sequence = requested == ARM_TEST_SEQUENCE
        random_sequence = (
            bool(goal_request.random_demo)
            and self.random_demo_enabled
            and tuple(goal_request.motor_ids) == tuple(ARM_ID_SEQUENCE)
            and len(goal_request.delta_ticks) == len(ARM_ID_SEQUENCE)
            and all(abs(int(delta)) <= 2 * RANDOM_ARM_RANGES[dxl_id]
                    for dxl_id, delta in requested)
        )
        if (self.read_only or not self.integrated_test_mode
                or self.gripper_only_mode
                or self.end_effector_kind != "rotary"
                or self.gripper_ids != [5]
                or not (fixed_sequence and not goal_request.random_demo
                        or random_sequence)):
            self.get_logger().warn(
                "Rejecting arm test goal: mode/preset/sequence gate closed")
            return GoalResponse.REJECT
        if (goal_request.max_abs_current < 0
                or goal_request.stall_timeout < 0.0
                or goal_request.step_timeout < 0.0):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def split_recorded_path_request(goal_request):
        """평탄화된 signed 기록 경로 요청을 검증하고 분리한다."""
        motor_ids = tuple(int(v) for v in goal_request.motor_ids)
        counts = tuple(int(v) for v in goal_request.waypoint_counts)
        flat = tuple(int(v) for v in goal_request.signed_waypoints)
        if motor_ids != RECORDED_PATH_IDS:
            raise ValueError(
                f"motor_ids must be exactly {list(RECORDED_PATH_IDS)}")
        if len(counts) != len(motor_ids) or any(count <= 0 for count in counts):
            raise ValueError("one positive waypoint_count is required per motor")
        if sum(counts) != len(flat):
            raise ValueError("waypoint_counts do not match signed_waypoints")

        paths = []
        offset = 0
        for dxl_id, count in zip(motor_ids, counts):
            waypoints = flat[offset:offset + count]
            offset += count
            if dxl_id == 12 and any(not 0 <= value <= 4095
                                    for value in waypoints):
                raise ValueError("ID 12 Mode 3 waypoint outside [0, 4095]")
            deltas = [b - a for a, b in zip(waypoints, waypoints[1:])]
            if any(delta == 0 or abs(delta) > RECORDED_PATH_MAX_WAYPOINT_STEP
                   for delta in deltas):
                raise ValueError("waypoint step must be in [1, 50] ticks")
            signs = {1 if delta > 0 else -1 for delta in deltas}
            if len(signs) > 1:
                raise ValueError(f"ID {dxl_id} waypoint direction reverses")
            paths.append((dxl_id, waypoints))
        return paths

    def arm_recorded_path_goal_callback(self, goal_request):
        """명시적인 3축 signed 기록 경로 액션만 수락한다."""
        try:
            self.split_recorded_path_request(goal_request)
        except ValueError as exc:
            self.get_logger().warn(f"Rejecting recorded path: {exc}")
            return GoalResponse.REJECT
        if (self.read_only or not self.integrated_test_mode
                or self.gripper_only_mode
                or self.end_effector_kind != "rotary"
                or self.gripper_ids != [5]
                or int(goal_request.max_abs_current) <= 0
                or float(goal_request.stall_timeout) <= 0.0
                or float(goal_request.step_timeout) <= 0.0
                or not 1 <= int(goal_request.goal_tolerance) <= 10):
            self.get_logger().warn(
                "Rejecting recorded path: mode/preset/safety gate closed")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def arm_goal_callback(self, goal_request):
        if getattr(self, "integrated_test_mode", False):
            self.get_logger().error(
                "Integrated test mode: rejecting normal arm trajectory")
            return GoalResponse.REJECT
        if self.gripper_only_mode:
            self.get_logger().error(
                "Gripper-only mode: rejecting arm FollowJointTrajectory goal")
            return GoalResponse.REJECT
        return self.goal_callback(goal_request)

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        self._stop_tool('action cancelled')
        return CancelResponse.ACCEPT

    def gripper_goal_callback(self, goal_request):
        if self.tool_type == 'spur_1motor_gripper':
            self.get_logger().warn(
                'spur gripper action rejected: use the FSM or CalibrationSession ingress')
            return GoalResponse.REJECT
        if self.tool_profile.get('backend') != 'gripper':
            self.get_logger().error('gripper goal rejected: selected tool is not a gripper')
            return GoalResponse.REJECT
        ready = (self._calibration_motion_ready()
                 if self.calibration_jog_enabled else self._tool_backend_ready())
        if self.control_mode != 'MANUAL' or not ready:
            self.get_logger().error(
                'gripper goal rejected: MANUAL ownership or tool backend '
                'interlock not ready')
            return GoalResponse.REJECT
        if self.tool_type == 'dual_motor_gripper':
            try:
                _fractions, spread = self._dual_normalized_spread()
            except RuntimeError as exc:
                self.get_logger().error(
                    f'gripper goal rejected: cannot verify dual spread: {exc}')
                return GoalResponse.REJECT
            if spread > 0.05:
                self.get_logger().error(
                    f'gripper goal rejected: normalized spread {spread:.4f} > 0.0500')
                return GoalResponse.REJECT
        with self._gripper_goal_lock:
            if self._gripper_goal_active:
                self.get_logger().warn(
                    'gripper goal rejected: another gripper goal is active')
                return GoalResponse.REJECT
            self._gripper_goal_active = True
        self.get_logger().info('gripper goal accepted')
        return GoalResponse.ACCEPT

    def gripper_cancel_callback(self, _goal_handle):
        self.get_logger().info('gripper cancel requested')
        return CancelResponse.ACCEPT

    def execute_gripper(self, goal_handle):
        try:
            return self._execute_gripper(goal_handle)
        finally:
            with self._gripper_goal_lock:
                self._gripper_goal_active = False

    def _execute_gripper(self, goal_handle):
        """Map one logical gripper joint to one or two calibrated actuators."""
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        if not trajectory.points or not trajectory.points[-1].positions:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'empty gripper trajectory'
            goal_handle.abort()
            return result
        if self.mock_mode:
            if self.temporary_jog_enabled:
                target = int(round(trajectory.points[-1].positions[0]))
                if not (self.temporary_jog_safe_min <= target
                        <= self.temporary_jog_safe_max):
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = 'mock target outside temporary jog range'
                    goal_handle.abort()
                    return result
                self._tool_samples[self.tool_ids[0]]['position'] = target
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = 'mock gripper action'
            goal_handle.succeed()
            return result
        position = float(trajectory.points[-1].positions[0])
        if self.calibration_jog_enabled:
            target = int(round(position))
            if len(self.calibration_endpoints) == 2:
                if target not in self.calibration_endpoints.values():
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = 'target is not a captured ID5 endpoint'
                    goal_handle.abort()
                    return result
            else:
                current = self._tool_samples.get(5, {}).get('position')
                if current is None or abs(target - int(current)) > self.calibration_max_jog_ticks:
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = 'calibration jog exceeds one-click limit'
                    goal_handle.abort()
                    return result
            targets = {5: target}
        elif self.temporary_jog_enabled:
            target = int(round(position))
            if not (self.temporary_jog_safe_min <= target
                    <= self.temporary_jog_safe_max):
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = (
                    f'temporary jog target {target} outside '
                    f'[{self.temporary_jog_safe_min}, '
                    f'{self.temporary_jog_safe_max}]')
                goal_handle.abort()
                self.get_logger().error(result.error_string)
                return result
            targets = {self.tool_ids[0]: target}
        else:
            targets = None
        open_pos = (1.0 if (self.temporary_jog_enabled or self.calibration_jog_enabled) else
                    float(self.tool_profile.get('open_position', 1.0)))
        close_pos = (0.0 if (self.temporary_jog_enabled or self.calibration_jog_enabled) else
                     float(self.tool_profile.get('close_position', 0.0)))
        denominator = open_pos - close_pos
        if self.temporary_jog_enabled or self.calibration_jog_enabled:
            denominator = 1.0
        if denominator == 0.0:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'invalid logical gripper endpoints'
            goal_handle.abort()
            return result
        ratio = max(0.0, min(1.0, (position - close_pos) / denominator))
        endpoints = self.tool_profile.get('motor_endpoints') or {
            self.tool_ids[0]: {
                'open': self.tool_profile['open_tick'],
                'close': self.tool_profile['close_tick']}}
        if self.calibration_jog_enabled:
            low, high = sorted(self.calibration_endpoints.values()) if len(
                self.calibration_endpoints) == 2 else (0, 4095)
        elif self.temporary_jog_enabled:
            low, high = self.temporary_jog_safe_min, self.temporary_jog_safe_max
        else:
            low = int(self.tool_profile['safe_min_tick'])
            high = int(self.tool_profile['safe_max_tick'])
        targets = targets or {}
        try:
            with self._bus_lock:
                for dxl_id in self.tool_ids:
                    if self.temporary_jog_enabled or self.calibration_jog_enabled:
                        tick = targets[dxl_id]
                    else:
                        ep = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
                        tick = int(round(ep['close'] + ratio *
                                         (ep['open'] - ep['close'])))
                    if not low <= tick <= high:
                        raise RuntimeError(
                            f'id {dxl_id} goal {tick} outside [{low},{high}]')
                    comm, error = self.packet_handler.write4ByteTxRx(
                        self.port_handler, dxl_id, ADDR_GOAL_POSITION,
                        tick & 0xffffffff)
                    if comm != 0 or error != 0:
                        raise RuntimeError(f'goal write failed for id {dxl_id}')
                    targets[dxl_id] = tick
            self.get_logger().info(
                f'gripper targets dispatched: normalized={ratio:.6f}, '
                f'targets={targets}')
            deadline = time.monotonic() + float(
                self.tool_profile.get('action_time', 0.0))
            max_effort = float(self.tool_profile.get(
                'max_abs_effort', float('inf')))
            errors = {}
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    self._hold_tool_position()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'gripper goal canceled and held'
                    goal_handle.canceled()
                    self.get_logger().info(result.error_string)
                    return result
                with self._bus_lock:
                    for dxl_id in self.tool_ids:
                        position, load = self._read_tool_state(dxl_id)
                        if abs(load) > max_effort:
                            raise RuntimeError(
                                f'id {dxl_id} effort limit exceeded')
                        errors[dxl_id] = targets[dxl_id] - position
                    if self.tool_type == 'dual_motor_gripper':
                        _fractions, spread = self._dual_normalized_spread(
                            positions={dxl_id: targets[dxl_id] - errors[dxl_id]
                                       for dxl_id in self.tool_ids})
                        if spread > 0.05:
                            raise RuntimeError(
                                f'normalized spread {spread:.4f} > 0.0500')
                if all(abs(error) <= self.gripper_target_tolerance
                       for error in errors.values()):
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = (
                        f'gripper targets reached: targets={targets}, '
                        f'errors={errors}')
                    goal_handle.succeed()
                    self.get_logger().info(result.error_string)
                    return result
                time.sleep(0.05)
            raise RuntimeError(
                f'gripper target tolerance not reached: targets={targets}, '
                f'errors={errors}, tolerance={self.gripper_target_tolerance}')
        except Exception as exc:
            self._stop_tool(str(exc))
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(exc)
            goal_handle.abort()
            return result

    def _read_tool_state(self, dxl_id):
        if self.mock_mode:
            sample = self._tool_samples[int(dxl_id)]
            return int(sample['position']), float(sample.get('effort', 0.0))
        hw, comm, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
        load, load_comm, load_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_LOAD)
        position, pos_comm, pos_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if (comm != 0 or error != 0 or load_comm != 0 or load_error != 0
                or pos_comm != 0 or pos_error != 0 or hw != 0):
            raise RuntimeError(
                f'fault reading id {dxl_id}: hw={hw}, '
                f'comm/error={(comm, error)}, '
                f'load={(load_comm, load_error)}, '
                f'position={(pos_comm, pos_error)}')
        return self._tool_position_tick(dxl_id, position), to_signed(load, 2)

    def _command_dual_targets_impl(self, targets):
        self._validate_dual_motion_request(targets)
        if self.mock_mode:
            for dxl_id, tick in targets.items():
                self._tool_samples[dxl_id]['position'] = int(tick)
            return
        try:
            self._dispatch_dual_sync_targets(targets)
            deadline = time.monotonic() + float(
                self.tool_profile['action_time'])
            max_effort = float(self.tool_profile.get(
                'max_abs_effort', float('inf')))
            while time.monotonic() < deadline:
                positions = {}
                errors = {}
                with self._bus_lock:
                    for dxl_id in (3, 4):
                        position, effort = self._read_tool_state(dxl_id)
                        if abs(effort) > max_effort:
                            raise RuntimeError(
                                f'ID{dxl_id} effort limit exceeded')
                        positions[dxl_id] = position
                        errors[dxl_id] = int(targets[dxl_id]) - position
                _fractions, spread = self._dual_normalized_spread(positions)
                if spread > 0.05:
                    raise RuntimeError(
                        f'dual synchronization fault: normalized spread '
                        f'{spread:.4f} > 0.0500; recalibration required')
                if all(abs(error) <= self.gripper_target_tolerance
                       for error in errors.values()):
                    # Successful endpoint completion leaves the pair enabled
                    # so the FSM can remain OPEN/CLOSED and hold position.
                    self.tool_motion_allowed = True
                    return
                time.sleep(0.05)
            raise RuntimeError(
                f'dual target tolerance not reached: errors={errors}')
        except Exception:
            self._stop_tool('dual FSM motion fault; recalibration required')
            raise

    def _validate_dual_motion_request(self, targets, require_range=True):
        if (self.tool_type != 'dual_motor_gripper' or self.tool_ids != [3, 4]
                or set(targets) != {3, 4}):
            raise RuntimeError('dual FSM target set must be exactly IDs [3, 4]')
        if (self.control_scope not in ('END_EFFECTOR_ONLY', 'FULL_ROBOT')
                or self.control_mode != 'MANUAL'
                or not self._tool_backend_ready()
                or self.emergency_stop_active or self.tool_detached):
            raise RuntimeError('dual FSM motion safety gate is not ready')
        if require_range:
            low = int(self.tool_profile['safe_min_tick'])
            high = int(self.tool_profile['safe_max_tick'])
            if any(not low <= int(tick) <= high for tick in targets.values()):
                raise RuntimeError('dual FSM target outside calibrated safe range')
        _fractions, spread = self._dual_normalized_spread()
        if spread > 0.05:
            raise RuntimeError(
                f'dual synchronization fault: normalized spread '
                f'{spread:.4f} > 0.0500')
    def _dispatch_dual_sync_targets(self, targets):
        if self.mock_mode:
            for dxl_id, tick in targets.items():
                self._tool_samples[dxl_id]['position'] = int(tick)
            return
        try:
            with self._bus_lock:
                self.group_sync_write.clearParam()
                for dxl_id in (3, 4):
                    if (self.read_hardware_error(dxl_id) != 0
                            or self.read_torque(dxl_id) != 1):
                        raise RuntimeError(
                            f'ID{dxl_id} HW/torque preflight failed')
                    data = self.int_to_little_endian_4bytes(
                        int(targets[dxl_id]))
                    if not self.group_sync_write.addParam(dxl_id, data):
                        raise RuntimeError(
                            f'ID{dxl_id} sync target staging failed')
                result = self.group_sync_write.txPacket()
                self.group_sync_write.clearParam()
                if result != 0:
                    raise RuntimeError(f'dual GroupSyncWrite failed: {result}')
        except Exception:
            self._stop_tool('dual FSM motion fault; recalibration required')
            raise
        finally:
            self.group_sync_write.clearParam()

    def _dual_normalized_spread(self, positions=None):
        """Return fresh dual normalized progress and its strict spread limit.

        Motor endpoint signs/directions are encoded in the validated profile,
        so this works for mirrored pinions without assuming tick ordering.
        """
        if self.tool_type != 'dual_motor_gripper' or self.tool_ids != [3, 4]:
            raise RuntimeError('dual normalized spread requested outside IDs [3, 4]')
        endpoints = self.tool_profile.get('motor_endpoints') or {}
        if positions is None:
            # Goal callbacks run independently of the feedback timer.  Keep
            # the three direct reads per motor serialized with SyncRead so a
            # spread safety check cannot manufacture a serial collision.
            with self._bus_lock:
                positions = {
                    dxl_id: self._read_tool_state(dxl_id)[0]
                    for dxl_id in self.tool_ids}
        fractions = {}
        for dxl_id in self.tool_ids:
            endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
            if not endpoint:
                raise RuntimeError(f'missing dual endpoint for ID {dxl_id}')
            span = int(endpoint['open']) - int(endpoint['close'])
            if span == 0:
                raise RuntimeError(f'zero dual endpoint span for ID {dxl_id}')
            fractions[dxl_id] = (
                (float(positions[dxl_id]) - int(endpoint['close'])) / span)
        return fractions, max(fractions.values()) - min(fractions.values())

    def _hold_tool_position(self):
        with self._bus_lock:
            positions = {
                dxl_id: self._read_tool_state(dxl_id)[0]
                for dxl_id in self.tool_ids}
            for dxl_id, position in positions.items():
                comm, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION,
                    position & 0xffffffff)
                if comm != 0 or error != 0:
                    raise RuntimeError(f'hold write failed for id {dxl_id}')

    # ------------------------------------------------------------------ arm
    def teleop_goal_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn("Teleop goal must be [motor_id, goal_tick]")
            return

        dxl_id, goal_tick = (int(msg.data[0]), int(msg.data[1]))
        if dxl_id not in ARM_IDS:
            self.get_logger().warn(f"Unknown arm motor ID from teleop: {dxl_id}")
            return
        if (self.gripper_only_mode
                or getattr(self, "integrated_test_mode", False)):
            self.get_logger().error(
                f"Diagnostic mode: rejecting arm teleop command id={dxl_id}")
            return
        if self.read_only:
            self.get_logger().warn("Read-only mode: ignoring teleop goal")
            return
        if dxl_id not in self.active_ids:
            self.get_logger().error(f"Inactive arm motor ID from teleop: {dxl_id}")
            return

        # 다회전(Extended Position) 축을 0~4095 로 clamp 하면 감속기 축이 한 바퀴
        # 넘는 순간 명령이 끝단에 박혀 더는 안 움직인다 — 축별 범위로 clamp 한다.
        if any(c["id"] == dxl_id and c["extended"] for c in JOINT_CONFIG.values()):
            goal_tick = max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, goal_tick))
        else:
            goal_tick = max(DXL_MINIMUM_POSITION_VALUE,
                            min(DXL_MAXIMUM_POSITION_VALUE, goal_tick))
        self.group_sync_write.clearParam()
        if not self.group_sync_write.addParam(
                dxl_id, self.int_to_little_endian_4bytes(goal_tick)):
            self.get_logger().warn(f"Failed to add teleop sync write param: id={dxl_id}")
            return
        result = self.group_sync_write.txPacket()
        self.group_sync_write.clearParam()
        if result != 0:
            self.get_logger().warn(f"Teleop GroupSyncWrite failed: result={result}")
            return
        self.get_logger().info(f"teleop -> id {dxl_id}: tick {goal_tick}")

    def execute_follow_joint_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        self.get_logger().info(
            f"Executing FollowJointTrajectory with {len(trajectory.points)} points"
        )

        self.trajectory_callback(trajectory)

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory sent to Dynamixel motors"
        return result

    def trajectory_callback(self, msg):
        if self.control_scope == 'END_EFFECTOR_ONLY':
            self.get_logger().warn(
                'Ignoring arm trajectory: control_scope=END_EFFECTOR_ONLY')
            return
        if (self.read_only or not self.tool_selection
                or not self.tool_selection.valid
                or (not self.mock_mode and not self.tool_motion_allowed)):
            self.get_logger().warn(
                "Ignoring arm trajectory: tool interlock not ready")
            return
        if not msg.points:
            return
        self._warn_if_torque_off()

        point = msg.points[-1]

        if len(msg.joint_names) != len(point.positions):
            self.get_logger().warn("JointTrajectory names/positions length mismatch")
            return

        if self.mock_mode:
            for name, rad in zip(msg.joint_names, point.positions):
                if name in self._mock_arm_positions:
                    self._mock_arm_positions[name] = self.tick_to_rad(
                        name, self.rad_to_tick(name, rad))
            return

        self.group_sync_write.clearParam()
        added_any = False

        for joint_name, rad in zip(msg.joint_names, point.positions):
            if joint_name not in JOINT_CONFIG:
                self.get_logger().warn(f"Unknown joint from MoveIt: {joint_name}")
                continue

            dxl_id = JOINT_CONFIG[joint_name]["id"]
            if dxl_id not in self.active_ids:
                self.get_logger().error(
                    f"Inactive arm motor from trajectory: {joint_name}, id={dxl_id}"
                )
                continue
            goal_tick = self.rad_to_tick(joint_name, rad)
            param_goal_position = self.int_to_little_endian_4bytes(goal_tick)

            ok = self.group_sync_write.addParam(dxl_id, param_goal_position)
            if not ok:
                self.get_logger().warn(f"Failed to add sync write param: id={dxl_id}")
                continue
            added_any = True

            self.get_logger().info(
                f"{joint_name} -> id {dxl_id}: {rad:.3f} rad -> {goal_tick}"
            )

        if not added_any:
            self.get_logger().error("Trajectory contains no active arm motors")
            self.group_sync_write.clearParam()
            return

        result = self.group_sync_write.txPacket()
        if result != 0:
            self.get_logger().warn(f"GroupSyncWrite failed: result={result}")

        self.group_sync_write.clearParam()

    # ------------------------------------------------------------------ feedback
    def _mark_tool_feedback_offline(self):
        """Fail closed after a transport-level feedback failure.

        Keep the process and ROS graph alive so an operator can reconnect the
        adapter or remove a competing serial client.  Stale ``online=True``
        samples would otherwise leave the GUI offering a torque command after
        a failed transaction.
        """
        for dxl_id in self.tool_ids:
            sample = dict(self._tool_samples.get(dxl_id, {}))
            sample.update({
                'id': dxl_id,
                'online': False,
                'position': None,
                'effort': None,
                'hardware_error': None,
                'torque_state': 'UNKNOWN',
            })
            self._tool_samples[dxl_id] = sample

    def publish_joint_states(self):
        if self.mock_mode:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            tool_msg = JointState()
            tool_msg.header.stamp = msg.header.stamp
            if self.control_scope == 'FULL_ROBOT':
                msg.name = list(self._mock_arm_positions)
                msg.position = list(self._mock_arm_positions.values())
                msg.effort = [0.0] * len(msg.name)
            for joint in self.tool_profile.get('joint_names', []):
                sample = next(iter(self._tool_samples.values()), {})
                tool_msg.name.append(joint)
                tool_msg.position.append(
                    float(sample.get('position', 0)) * 2 * math.pi / 4096)
                tool_msg.effort.append(float(sample.get('effort', 0)))
            self.joint_state_pub.publish(msg)
            self.tool_joint_state_pub.publish(tool_msg)
            self.fault_pub.publish(Bool(data=False))
            return
        if not self.port_connected:
            self.joint_state_pub.publish(JointState())
            self.tool_joint_state_pub.publish(JointState())
            self.fault_pub.publish(Bool(data=True))
            return
        try:
            with self._bus_lock:
                self.group_sync_read.txRxPacket()
        except Exception as exc:
            # This timer runs in the executor.  Do not allow a transient USB
            # read failure to terminate the bridge and make the GUI report a
            # misleading "control node disconnected" state.
            self.get_logger().warn(f'joint feedback transport failed: {exc}')
            self._mark_tool_feedback_offline()
            self.joint_state_pub.publish(JointState())
            self.tool_joint_state_pub.publish(JointState())
            self.fault_pub.publish(Bool(data=True))
            return
        # 일부 ID가 버스에 없어도 응답받은 ID만 처리 (result 무시)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        tool_msg = JointState()
        tool_msg.header.stamp = msg.header.stamp

        # controller fault 집계 — SyncRead 에 등록된(토크 ON 성공) ID 중 하나라도
        # Hardware Error Status != 0 이거나 이번 tick 응답이 없으면 fault=True.
        # 응답 없음도 fault 로 보는 이유: 활성 등록된 서보가 갑자기 무응답이면 버스/전원
        # 이상일 수 있어 "정상"으로 오인하면 안 됨(안전 측 기본값).
        fault = False if self.gripper_only_mode else not ARM_IDS.issubset(
            self.active_ids)

        # 팔 관절: position(rad) + address-126 feedback(raw signed).
        # 주소 126의 의미는 실제 장착 모터 control table로 확인해야 한다.
        for joint_name, config in JOINT_CONFIG.items():
            dxl_id = config["id"]
            if dxl_id not in self.active_ids:
                continue
            sample = self._read_sample(dxl_id)
            if sample is None:
                fault = True
                continue
            feedback_raw, tick, hw_error = sample
            if hw_error != 0:
                fault = True
            msg.name.append(joint_name)
            msg.position.append(self.tick_to_rad(joint_name, tick))
            msg.effort.append(float(feedback_raw))

        if (self.tool_type == 'cleaner' and self.cleaning_configured
                and self.cleaning_actuator_id in self.active_ids):
            sample = self._read_sample(self.cleaning_actuator_id)
            if sample is None:
                # Some Protocol 2.0 tools answer individual register reads
                # but not the shared 70–135 SyncRead block.  The cleaner is a
                # single isolated ID, so a direct read fallback preserves the
                # same online/HW fail-closed semantics without polling arm IDs.
                sample = self._read_cleaner_sample(self.cleaning_actuator_id)
            if sample is None:
                fault = True
                self._tool_samples[self.cleaning_actuator_id] = {
                    'id': self.cleaning_actuator_id,
                    'joint': self.cleaning_actuator_joint, 'position': None,
                    'effort': None, 'online': False}
            else:
                load_raw, tick, hw_error = sample
                tick = self._tool_position_tick(self.cleaning_actuator_id, tick)
                fault = fault or hw_error != 0
                self._tool_samples[self.cleaning_actuator_id] = {
                    'id': self.cleaning_actuator_id,
                    'joint': self.cleaning_actuator_joint,
                    'position': int(tick), 'effort': float(abs(load_raw)),
                    'online': hw_error == 0}
                tool_msg.name.append(self.cleaning_actuator_joint)
                tool_msg.position.append(
                    float(to_signed(tick, LEN_PRESENT_POSITION)))
                tool_msg.effort.append(float(load_raw))

        # The spur tool has exactly one feedback topology.  Do not fall through
        # to the legacy rack/pinion aggregation below: that path assumes the
        # ID3/ID4 pair and manufactures a controller fault when those IDs are
        # intentionally absent from an ID5-only process.
        if self.tool_ids == [5]:
            joint_names = self.tool_profile.get('joint_names', [])
            dxl_id = 5
            if dxl_id not in self.active_ids:
                fault = True
                self._tool_samples[dxl_id] = {
                    'id': dxl_id, 'joint': joint_names[0] if joint_names else '',
                    'position': None, 'effort': None, 'online': False,
                    'hardware_error': None, 'torque_state': 'UNKNOWN',
                    'operating_mode': None, 'profile_velocity': None,
                    'profile_acceleration': None}
            else:
                sample = self._read_sample(dxl_id)
                if sample is None:
                    # A GroupSyncRead parameter can remain stale across a
                    # cleaner→spur profile swap even though ID5 answers direct
                    # packets.  Do not present that reachable actuator as
                    # offline and permanently block the explicit torque gate.
                    sample = self._read_spur_sample(dxl_id)
                if sample is None:
                    fault = True
                    self._tool_samples[dxl_id] = {
                        'id': dxl_id, 'joint': joint_names[0] if joint_names else '',
                        'position': None, 'effort': None, 'online': False,
                        'hardware_error': None, 'torque_state': 'UNKNOWN',
                        'operating_mode': None, 'profile_velocity': None,
                        'profile_acceleration': None}
                else:
                    load_raw, tick, hw_error = sample
                    control = self._read_tool_control_state(dxl_id)
                    fault = fault or hw_error != 0
                    self._tool_samples[dxl_id] = {
                        'id': dxl_id,
                        'joint': joint_names[0] if joint_names else '',
                        'position': self._tool_position_tick(dxl_id, tick),
                        'effort': float(load_raw), 'online': hw_error == 0,
                        'hardware_error': int(hw_error),
                        # Model identity is obtained by the startup ping and
                        # remains valid across periodic position reads.
                        'model': self._tool_samples.get(dxl_id, {}).get('model'),
                        **control}
                    if joint_names:
                        tool_msg.name.append(joint_names[0])
                        tool_msg.position.append(
                            float(self._tool_samples[dxl_id]['position'])
                            * 2 * math.pi / 4096)
                        tool_msg.effort.append(float(load_raw))

        # Legacy dual-gripper feedback is deliberately isolated from ID5.  It
        # retains the existing ID3/ID4 tuple aggregation for the dual profile.
        # XL430-W250 그리퍼: 주소 126은 signed Present Load(0.1% 추정 부하다).
        # 랙피니언 2모터(ID 3,4)를 함께 읽어 하나의 논리 조인트(gripper_left_pinion_joint)로
        # 보고한다 — position(rad)=대표(첫 응답) 모터 tick, effort=가장 큰 abs(load).
        # 한 모터라도 부하가 크면 파지로 보는 보수적(안전 측) 집계이며, FSM 이 이 effort 로
        # 파지/DROP 을 판정한다.
        if self.tool_type == 'dual_motor_gripper':
            gripper_samples = []
            for gid in self.gripper_ids:
                if gid not in self.active_ids:
                    fault = True
                    self._mark_tool_feedback_offline()
                    continue
                sample = self._read_sample(gid)
                if sample is None:
                    # A runtime tool swap can leave GroupSyncRead with stale
                    # parameters for a cycle (or a tool can decline the
                    # 70--135 block while still answering individual reads).
                    # Treat ID3/ID4 exactly like the isolated ID5 path: a
                    # successful direct observation restores feedback; a
                    # failed one still leaves the torque gate closed.
                    sample = self._read_spur_sample(gid)
                if sample is None:
                    fault = True
                    # Never retain the last valid ID3/ID4 sample after the
                    # TTL line is removed.  The automatic detector treats a
                    # live feedback sample as presence evidence, so stale
                    # `online=True` here would hide a physical detach.
                    self._mark_tool_feedback_offline()
                    continue
                load_raw, tick, hw_error = sample
                velocity_raw = 0
                if hw_error != 0:
                    fault = True
                # Overload(0x20) 는 파지 중 가장 흔한 트립이고, 나면 REBOOT 전까지 서보가
                # 죽어 있다. 자동 복구를 켜 뒀으면 여기서 되살린다.
                if (hw_error & HWERR_OVERLOAD) and self.gripper_overload_reboot \
                        and not self._gripper_recovering and not self.read_only:
                    self._recover_gripper_overload(gid)
                control = self._read_tool_control_state(gid)
                # Temperature lies just beyond the shared 70–135 SyncRead
                # block.  It is diagnostic-only, so a failed read is shown as
                # unavailable rather than making healthy position feedback
                # fail closed.
                try:
                    temperature = self._read_register(
                        gid, ADDR_PRESENT_TEMPERATURE, 1,
                        'tool present temperature')
                except RuntimeError:
                    temperature = None
                self._tool_samples[gid] = {
                    'id': gid,
                    'joint': self.gripper_joints[0] if self.gripper_joints else '',
                    'position': self._tool_position_tick(gid, tick),
                    'effort': float(load_raw), 'online': hw_error == 0,
                    'hardware_error': int(hw_error),
                    'temperature_c': temperature, **control}
                gripper_samples.append(
                    (load_raw, to_signed(tick, LEN_PRESENT_POSITION), velocity_raw))

            dual_driving = set(self.gripper_ids).issubset(
                self.torque_enabled_ids)
            if dual_driving:
                sync_fault = None
                if len(gripper_samples) != len(self.gripper_ids):
                    sync_fault = 'dual feedback lost while torque enabled'
                else:
                    positions = {
                        dxl_id: self._tool_samples[dxl_id]['position']
                        for dxl_id in self.gripper_ids}
                    try:
                        _fractions, spread = self._dual_normalized_spread(
                            positions)
                        if spread > 0.05:
                            sync_fault = (
                                f'normalized spread {spread:.4f} > 0.0500; '
                                'recalibration required')
                    except RuntimeError as exc:
                        sync_fault = str(exc)
                if sync_fault:
                    self._stop_tool(sync_fault)
                    if self.tool_fsm is not None:
                        self.tool_fsm.fault_reason = str(sync_fault)
                        self.tool_fsm.state = ToolState.STOPPED

            if len(gripper_samples) == len(self.gripper_ids) and gripper_samples:
                representative_tick = gripper_samples[0][1]
                max_abs_load = max(abs(sample[0]) for sample in gripper_samples)
                # Dual geometry remains profile-driven.  ID3 is the legacy
                # representative feedback motor; do not reuse any spur-ID5
                # conversion or manufacture a calibration endpoint here.
                endpoints = self.tool_profile.get('motor_endpoints') or {}
                endpoint = endpoints.get(3, endpoints.get('3'))
                if not endpoint or endpoint['open'] == endpoint['close']:
                    raise RuntimeError('dual gripper representative endpoint missing')
                close_position = float(self.tool_profile['close_position'])
                open_position = float(self.tool_profile['open_position'])
                fraction = ((representative_tick - endpoint['close']) /
                            (endpoint['open'] - endpoint['close']))
                finger_rad = close_position + fraction * (
                    open_position - close_position)
                # PRESENT_VELOCITY is not currently extracted from the
                # SyncRead tuple, so publish an explicit zero rather than
                # calling the removed legacy conversion helper.
                finger_vel = 0.0
                for jn in self.gripper_joints:
                    tool_msg.name.append(jn)
                    tool_msg.position.append(finger_rad)
                    tool_msg.velocity.append(finger_vel)
                    tool_msg.effort.append(float(max_abs_load))

        self.joint_state_pub.publish(msg)
        self.tool_joint_state_pub.publish(tool_msg)
        self.fault_pub.publish(Bool(data=fault))

    def _read_sample(self, dxl_id):
        """SyncRead 블록에서 (signed address-126 feedback, position, hw error) 추출.

        PRESENT_VELOCITY(128,4)는 SyncRead 범위(70~135) 안에 이미 포함돼 있어 별도 버스
        요청 없이 같은 블록에서 꺼낸다. 미수신 시 None.
        """
        with self._bus_lock:
            if not self.group_sync_read.isAvailable(
                    dxl_id, ADDR_HARDWARE_ERROR_STATUS,
                    LEN_HARDWARE_ERROR_STATUS):
                return None
            if not self.group_sync_read.isAvailable(
                    dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD):
                return None
            if not self.group_sync_read.isAvailable(
                    dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                return None
            try:
                hw_error = self.group_sync_read.getData(
                    dxl_id, ADDR_HARDWARE_ERROR_STATUS,
                    LEN_HARDWARE_ERROR_STATUS)
                feedback_raw = to_signed(
                    self.group_sync_read.getData(
                        dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD),
                    LEN_PRESENT_LOAD,
                )
                tick = self.group_sync_read.getData(
                    dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            except (IndexError, KeyError):
                return None
        return feedback_raw, tick, hw_error

    def _read_cleaner_sample(self, dxl_id):
        """Read one velocity-mode cleaner when its SyncRead block is absent."""
        try:
            with self._bus_lock:
                hw_error = self._read_register(
                    dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                    'cleaner hardware error')
                feedback_raw = self._read_register(
                    dxl_id, ADDR_PRESENT_LOAD, 2,
                    'cleaner present load', signed=True)
                tick = self._read_register(
                    dxl_id, ADDR_PRESENT_POSITION, 4,
                    'cleaner present position')
            return feedback_raw, tick, hw_error
        except Exception as exc:
            self.get_logger().warn(
                f'cleaner direct feedback unavailable: id={dxl_id}: {exc}')
            return None

    def _read_spur_sample(self, dxl_id):
        """Read ID5 directly when SyncRead was not rebuilt after a tool swap."""
        try:
            with self._bus_lock:
                hw_error = self._read_register(
                    dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                    'spur hardware error')
                feedback_raw = self._read_register(
                    dxl_id, ADDR_PRESENT_LOAD, 2,
                    'spur present load', signed=True)
                tick = self._read_register(
                    dxl_id, ADDR_PRESENT_POSITION, 4,
                    'spur present position')
            return feedback_raw, tick, hw_error
        except Exception as exc:
            self.get_logger().warn(
                f'spur direct feedback unavailable: id={dxl_id}: {exc}')
            return None

    def _read_tool_control_state(self, dxl_id):
        """Read-only control-table observation for the selected tool only."""
        if dxl_id not in self.tool_ids:
            return {'torque_state': 'UNKNOWN', 'operating_mode': None,
                    'profile_velocity': None, 'profile_acceleration': None}
        with self._bus_lock:
            torque, tr, te = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE)
            if dxl_id != 5:
                return {
                    'torque_state': ('ON' if torque == 1 else 'OFF')
                    if tr == 0 and te == 0 else 'UNKNOWN',
                    'operating_mode': None, 'profile_velocity': None,
                    'profile_acceleration': None}
            mode, mr, me = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE)
            accel, ar, ae = self.packet_handler.read4ByteTxRx(
                self.port_handler, dxl_id, ADDR_PROFILE_ACCELERATION)
            velocity, vr, ve = self.packet_handler.read4ByteTxRx(
                self.port_handler, dxl_id, ADDR_PROFILE_VELOCITY)
        return {
            'torque_state': ('ON' if torque == 1 else 'OFF')
                if tr == 0 and te == 0 else 'UNKNOWN',
            'operating_mode': int(mode) if mr == 0 and me == 0 else None,
            'profile_acceleration': int(accel) if ar == 0 and ae == 0 else None,
            'profile_velocity': int(velocity) if vr == 0 and ve == 0 else None,
        }

    def destroy_node(self):
        # A calibration-monitor window must not change the pre-existing torque
        # state merely because the GUI is closed.  Explicit STOP/DISABLE and
        # E-stop still call _stop_tool while it is running.
        if (not self.read_only and not self.mock_mode
                and not self.calibration_jog_enabled
                and self.tool_profile.get('backend') != 'gripper'):
            self._stop_tool('node shutdown')
            if self.cleaning_configured:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, self.cleaning_actuator_id, ADDR_GOAL_VELOCITY, 0)
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, self.cleaning_actuator_id,
                    ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if self.control_scope == 'FULL_ROBOT':
                for config in JOINT_CONFIG.values():
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, config["id"],
                        ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if not self.mock_mode and self.port_connected:
            self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItDynamixelBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    # position_node 와 같은 이유로 traceback 대신 한 줄로 죽는다(그쪽 main 참고).
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
