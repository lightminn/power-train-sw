from __future__ import annotations

import math
from abc import ABC, abstractmethod


class TransportError(Exception):
    """전송 계층 연결/IO 실패."""


def validate_gear_ratio(value: float) -> float:
    """모터 회전수 / 바퀴 회전수 비율을 정규화하고 검증한다."""
    ratio = float(value)
    if not math.isfinite(ratio) or ratio <= 0.0:
        raise ValueError("gear_ratio must be finite and positive")
    return ratio


GEAR_SCALED_TUNABLES = frozenset({
    "vel_limit",
    "trap_vel_limit",
    "trap_accel_limit",
    "trap_decel_limit",
})


def motor_to_wheel_tunables(values: dict, gear_ratio: float) -> dict:
    """모터축 기준 속도계 기본값을 GUI 바퀴축 단위로 변환한다."""
    ratio = validate_gear_ratio(gear_ratio)
    return {
        key: (float(value) / ratio if key in GEAR_SCALED_TUNABLES else value)
        for key, value in values.items()
    }


class Transport(ABC):
    """모든 전송(USB/CAN/Fake)의 공통 계약.

    sample()/apply()/capabilities() 는 **JSON-직렬화 가능한 dict 만** 주고받는다
    (웹 레이어와의 seam — 향후 프로세스 격리 시 그대로 IPC 경계가 됨).
    """

    name: str = "base"

    @abstractmethod
    def connect(self) -> None:
        """장치 연결. 실패 시 TransportError."""

    @abstractmethod
    def sample(self) -> dict:
        """텔레메트리 1프레임. 항상 't_mono' 포함, 키는 '<device>.<signal>'."""

    @abstractmethod
    def apply(self, cmd: dict) -> dict:
        """정규화된 command envelope 적용. ack dict 반환."""

    @abstractmethod
    def capabilities(self) -> dict:
        """이 트랙이 노출하는 devices/signals/commands/limits/notes."""

    @abstractmethod
    def close(self) -> None:
        """안전 정지 + 자원 해제."""

    def read_tunables(self) -> dict:
        """현재 튜닝 파라미터 값 {key: value}. 미지원/미연결 시 빈 dict.
        connect() 후 호출됨 (worker.start 에서 1회)."""
        return {}

    def device_ids(self) -> dict:
        """선택 가능한 CAN ID 스펙 {device: {id,min,max,label}}. CAN 트랙만 노출,
        그 외(USB/Fake)는 빈 dict."""
        return {}

    def set_device_ids(self, mapping: dict) -> None:
        """{device: new_id} 적용 (다음 connect/reconnect 에서 반영). 기본 no-op."""
        return None


# ── 프론트 데이터-드리븐 UI 메타 (capabilities 에 포함) ────────────────

# 신호 → 범례 라벨/단위
SIGNAL_META = {
    "odrive.pos": {"label": "위치", "unit": "turn"},
    "odrive.pos_setpoint": {"label": "위치 명령", "unit": "turn"},
    "odrive.vel": {"label": "속도", "unit": "wheel rev/s"},
    "odrive.vel_setpoint": {"label": "속도 명령", "unit": "wheel rev/s"},
    "odrive.iq_meas": {"label": "Iq 측정 (토크축)", "unit": "A"},
    "odrive.iq_set": {"label": "Iq 목표", "unit": "A"},
    "odrive.id_meas": {"label": "Id 측정 (자속축)", "unit": "A"},
    "odrive.id_set": {"label": "Id 목표", "unit": "A"},
    "odrive.torque_est": {"label": "추정 토크 (Iq×Kt)", "unit": "Nm"},
    "odrive.temp_fet": {"label": "FET 온도", "unit": "℃"},
    "odrive.vbus": {"label": "버스 전압", "unit": "V"},
    "odrive.ibus": {"label": "버스 전류", "unit": "A"},
    "odrive.state": {"label": "상태(8=폐루프)", "unit": ""},
    "odrive.vel_integrator": {"label": "속도 적분 토크", "unit": ""},
    "odrive.axis_err": {"label": "axis 에러", "unit": "hex"},
    "odrive.motor_err": {"label": "motor 에러", "unit": "hex"},
    "odrive.enc_err": {"label": "encoder 에러", "unit": "hex"},
    "odrive.ctrl_err": {"label": "controller 에러", "unit": "hex"},
    "ak.pos_deg": {"label": "AK 위치", "unit": "°"},
    "ak.speed": {"label": "AK 속도(출력축)", "unit": "RPM"},
    "ak.current": {"label": "AK 전류 (≈토크)", "unit": "A"},
    "ak.temp": {"label": "AK 온도", "unit": "℃"},
    "ak.fault": {"label": "AK fault", "unit": ""},
    "ak.tripped": {"label": "AK 과전류정지", "unit": ""},
    "ak.pos_cmd": {"label": "AK 위치 명령", "unit": "°"},
    "ak.speed_cmd": {"label": "AK 속도 명령", "unit": "RPM"},
}

# ODrive 제어 모드 → 목표값 입력 필드
ODRIVE_CONTROL_MODES = ["position", "position_traj", "velocity", "torque"]
ODRIVE_INPUTS = {
    "position": {"key": "pos", "label": "목표 위치", "unit": "turn",
                 "help": "목표 위치(turn)로 이동 (입력 필터)."},
    "position_traj": {"key": "pos", "label": "목표 위치(사다리꼴)", "unit": "turn",
                      "help": "가속한계 지켜 목표 위치로 이동 (사다리꼴 프로파일)."},
    "velocity": {"key": "vel", "label": "목표 속도(바퀴)", "unit": "rev/s",
                 "help": "바퀴 목표 속도(rev/s) 유지."},
    "torque": {"key": "torque", "label": "목표 토크", "unit": "Nm",
               "help": "목표 토크(Nm) 인가. 속도는 vel_limit 로 제한."},
}

# 튜닝 가능 파라미터 (ODrive 캐스케이드 컨트롤러: 위치 P / 속도 PI + 한계)
ODRIVE_TUNABLES_USB = [
    {"op": "set_gain", "key": "pos_gain", "label": "위치 게인 P"},
    {"op": "set_gain", "key": "vel_gain", "label": "속도 게인 P"},
    {"op": "set_gain", "key": "vel_integrator_gain", "label": "속도 적분 게인 I"},
    {"op": "set_gain", "key": "input_filter_bandwidth", "label": "입력 필터 BW [Hz]"},
    {"op": "set_gain", "key": "trap_vel_limit", "label": "TRAP 바퀴 속도한계 [rev/s]"},
    {"op": "set_gain", "key": "trap_accel_limit", "label": "TRAP 바퀴 가속한계 [rev/s²]"},
    {"op": "set_gain", "key": "trap_decel_limit", "label": "TRAP 바퀴 감속한계 [rev/s²]"},
    {"op": "set_limit", "key": "vel_limit", "label": "바퀴 속도 한계 [rev/s]"},
    {"op": "set_limit", "key": "current_lim", "label": "전류 한계 [A]"},
]
# CAN 은 input_filter_bandwidth 미지원 (CANSimple 명령 없음)
ODRIVE_TUNABLES_CAN = [t for t in ODRIVE_TUNABLES_USB
                       if t["key"] != "input_filter_bandwidth"]

# 명시적 사용자 선택 전에는 어느 프로파일도 하드웨어에 쓰지 않는다.
TUNABLE_PROFILES = {
    # X2212-13 + TLE5012B 실측 게인 스윕 최적값
    # (docs/motor-gui-tuning-guide.md 참고). vel_gain>0.05 면 트립,
    # vel_integrator_gain 은 한계진동(움찔) 유발 → 0. 다른 모터에는 재튜닝 필수.
    "x2212": {
        "label": "X2212-13 + TLE5012B",
        "values": {
            "pos_gain": 8.0,
            "vel_gain": 0.015,
            "vel_integrator_gain": 0.0,
            "input_filter_bandwidth": 50.0,
            "vel_limit": 50.0,
            "current_lim": 10.0,
            "trap_vel_limit": 20.0,
            "trap_accel_limit": 15.0,
            "trap_decel_limit": 20.0,
        },
    },
    "bl70200": {
        "label": "BL70200",
        "values": {
            "pos_gain": 2.0,
            "vel_gain": 0.12,
            "vel_integrator_gain": 0.2,
            "current_lim": 9.0,
        },
    },
}

# 기존 import 호환성. 이 값은 X2212 프로파일일 뿐 자동 적용 baseline 이 아니다.
DEFAULT_TUNABLES = dict(TUNABLE_PROFILES["x2212"]["values"])
