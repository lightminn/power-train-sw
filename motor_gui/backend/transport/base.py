from __future__ import annotations

from abc import ABC, abstractmethod


class TransportError(Exception):
    """전송 계층 연결/IO 실패."""


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


# ── 프론트 데이터-드리븐 UI 메타 (capabilities 에 포함) ────────────────

# 신호 → 범례 라벨/단위
SIGNAL_META = {
    "odrive.pos": {"label": "위치", "unit": "turn"},
    "odrive.pos_setpoint": {"label": "위치 명령", "unit": "turn"},
    "odrive.vel": {"label": "속도", "unit": "rev/s"},
    "odrive.vel_setpoint": {"label": "속도 명령", "unit": "rev/s"},
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
}

# ODrive 제어 모드 → 목표값 입력 필드
ODRIVE_CONTROL_MODES = ["position", "position_traj", "velocity", "torque"]
ODRIVE_INPUTS = {
    "position": {"key": "pos", "label": "목표 위치", "unit": "turn"},
    "position_traj": {"key": "pos", "label": "목표 위치(사다리꼴)", "unit": "turn"},
    "velocity": {"key": "vel", "label": "목표 속도", "unit": "rev/s"},
    "torque": {"key": "torque", "label": "목표 토크", "unit": "Nm"},
}

# 튜닝 가능 파라미터 (ODrive 캐스케이드 컨트롤러: 위치 P / 속도 PI + 한계)
ODRIVE_TUNABLES_USB = [
    {"op": "set_gain", "key": "pos_gain", "label": "위치 게인 P"},
    {"op": "set_gain", "key": "vel_gain", "label": "속도 게인 P"},
    {"op": "set_gain", "key": "vel_integrator_gain", "label": "속도 적분 게인 I"},
    {"op": "set_gain", "key": "input_filter_bandwidth", "label": "입력 필터 BW [Hz]"},
    {"op": "set_gain", "key": "trap_vel_limit", "label": "TRAP 속도한계 [rev/s]"},
    {"op": "set_gain", "key": "trap_accel_limit", "label": "TRAP 가속한계 [rev/s²]"},
    {"op": "set_gain", "key": "trap_decel_limit", "label": "TRAP 감속한계 [rev/s²]"},
    {"op": "set_limit", "key": "vel_limit", "label": "속도 한계 [rev/s]"},
    {"op": "set_limit", "key": "current_lim", "label": "전류 한계 [A]"},
]
# CAN 은 input_filter_bandwidth 미지원 (CANSimple 명령 없음)
ODRIVE_TUNABLES_CAN = [t for t in ODRIVE_TUNABLES_USB
                       if t["key"] != "input_filter_bandwidth"]

# X2212-13 + TLE5012B 실측 게인 스윕 최적값 (docs/motor-gui-tuning-guide.md 참고).
# vel_gain>0.05 면 트립, vel_integrator_gain 은 한계진동(움찔) 유발 → 0.
# 잔차(~0.08turn)는 코깅 — fw0.5.1 anticogging 이 불안정(모션 brick)해 미사용, 실용 한계로 수용.
# 다른 모터로 교체 시 docs/motor-gui-tuning-guide.md 절차대로 재튜닝.
DEFAULT_TUNABLES = {
    "pos_gain": 8.0,
    "vel_gain": 0.015,
    "vel_integrator_gain": 0.0,
    "input_filter_bandwidth": 50.0,
    "vel_limit": 5.0,
    "current_lim": 10.0,
    "trap_vel_limit": 20.0,
    "trap_accel_limit": 15.0,
    "trap_decel_limit": 20.0,
}
