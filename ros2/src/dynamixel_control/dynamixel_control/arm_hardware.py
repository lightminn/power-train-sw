"""구동되는 팔 관절 4개의 공용 설정.

``arm_joint_1``은 동력이 없는 차체-팔 yaw 연결부다. 로봇 모델에는 고정 변환으로
남아 있지만 이 설정에서는 의도적으로 제외한다.
"""

import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET


ARM_JOINT_NAMES = [f"arm_joint_{index}" for index in range(2, 6)]

# 검증된 4축 버스 매핑. 이 순서는 브리지 및 텔레옵 스택과 맞춰 유지한다.
# arm_joint_1에는 액추에이터가 없다.
ARM_MOTOR_IDS = [14, 13, 12, 16]

# Upstream PR #40 실측값(2026-08-07). 물리 관절 제한이 아직 잠정값이므로
# 이 값만으로 일반 동작을 허용하지 않는다.
ARM_CENTERS = [1627, 4281, 2563, 949]
ARM_DIRECTIONS = [-1, 1, 1, 1]
ARM_COMMAND_CALIBRATED = False

# arm_joint_1은 차체에 물리적으로 고정되어 있다. 현재 URDF는 CAD 영점 변환만
# 보존하며, 장착된 차체의 yaw는 아직 측정하지 않았다.
ARM_FIXED_YAW_CALIBRATED = False

ARM_JOINT_CONFIG = {
    name: {"id": ARM_MOTOR_IDS[index],
           "center": ARM_CENTERS[index],
           "direction": ARM_DIRECTIONS[index],
           "gear_ratio": [9.034, 4.040, 1.0, 1.0][index],
           "extended": [True, True, False, False][index]}
    for index, name in enumerate(ARM_JOINT_NAMES)
}

# TODO(하드웨어 캘리브레이션): 검증된 물리 안전 제한이 아닌 URDF/CAD 제한이다.
# 중심, 방향, 제한을 측정할 때까지 소프트웨어 제한을 비활성으로 유지한다.
# 하드웨어 명령 기동은 ARM_COMMAND_CALIBRATED로 차단한다.
ARM_LIMIT_ENABLED = [False] * 4
ARM_MIN_RADS = [0.0, 0.0, -0.610865, -math.pi]
ARM_MAX_RADS = [math.pi, math.pi, 0.698132, math.pi]


def load_srdf_group_state(group_name, state_name):
    """설치된 MoveIt SRDF에서 이름이 지정된 완전한 관절 상태를 불러온다."""
    candidates = [
        Path(prefix) / "share/robot_arm_moveit_config/config/robot_arm.srdf"
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
        if prefix
    ]
    path = next((candidate for candidate in candidates if candidate.is_file()), None)
    if path is None:
        raise RuntimeError(
            "robot_arm_moveit_config/config/robot_arm.srdf is not installed "
            "in AMENT_PREFIX_PATH")
    root = ET.parse(path).getroot()
    state = root.find(
        f"./group_state[@name='{state_name}'][@group='{group_name}']")
    if state is None:
        raise RuntimeError(
            f"SRDF group_state {group_name}/{state_name} was not found")
    values = {
        joint.get("name"): float(joint.get("value"))
        for joint in state.findall("joint")
    }
    missing = [name for name in ARM_JOINT_NAMES if name not in values]
    if missing:
        raise RuntimeError(
            f"SRDF group_state {group_name}/{state_name} misses {missing}")
    return [values[name] for name in ARM_JOINT_NAMES]
