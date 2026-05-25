"""ODrive 3.6(USB) 백엔드 DriveActuator. axis1, VELOCITY_CONTROL + PASSTHROUGH.

폐루프 진입 시 input_vel=0 으로 점프를 방지한다. vel_limit/current_lim 은
NVM 에 저장된 설정값을 그대로 사용(init_odrive.py 로 1회 셋업 가정).

Enum 경로 근거
--------------
HIL(Jetson, odrive 라이브러리) 검증 결과, 클래스 enum 객체
(`ControlMode.VELOCITY_CONTROL` 등)를 config 에 대입하면 int 로 변환되지
않아 `TypeError: int() argument ... not 'ControlMode'` 가 발생한다.
따라서 **플랫 int 상수**(`CONTROL_MODE_*`, `INPUT_MODE_*`, `AXIS_STATE_*`)
를 사용한다 — `init_odrive.py` / `odrive_can_setup.py` 와 동일 방식이며
이 펌웨어에서 동작 검증됨.

텔레메트리 경로 (axis1 직접 접근):
    encoder.vel_estimate           — `axis1.encoder.vel_estimate`
    motor.current_control.Iq_measured — odrive 공식 구조체 경로(검증됨)

state() 키는 CornerModule 계약: {"target_vel", "actual_vel", "cur_a"}
"""
import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)

from corner_module.actuator import DriveActuator


class DriveOdriveUsb(DriveActuator):
    """ODrive 3.6 USB 드라이버 — axis1, velocity control.

    Parameters
    ----------
    find_timeout:
        ``odrive.find_any()`` 탐색 타임아웃(초). 기본 10 s.
    """

    def __init__(self, find_timeout: float = 10.0) -> None:
        self._find_timeout = find_timeout
        self._odrv = None
        self._axis = None
        self._target_vel: float = 0.0

    # ------------------------------------------------------------------
    # Actuator interface
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """USB 로 ODrive 를 탐색하고 axis1 핸들을 캐시한다."""
        self._odrv = odrive.find_any(timeout=self._find_timeout)
        if self._odrv is None:
            raise RuntimeError("ODrive USB 미발견. 케이블/전원 확인.")
        self._axis = self._odrv.axis1

    def arm(self) -> None:
        """velocity-control + passthrough 모드로 폐루프 진입.

        input_vel 을 0 으로 고정한 뒤 CLOSED_LOOP_CONTROL 을 요청해
        점프(jump) 를 방지한다 — odrive_dualsense_vel_test.py 동일 패턴.
        """
        ax = self._axis
        ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        ax.controller.input_vel = 0.0          # 점프 방지 (dualsense 스크립트 동일)
        self._target_vel = 0.0
        ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def disarm(self) -> None:
        """속도를 0 으로 내리고 IDLE 전환."""
        self._axis.controller.input_vel = 0.0
        self._axis.requested_state = AXIS_STATE_IDLE

    def set_velocity(self, turns_per_s: float) -> None:
        """다음 tick() 에 적용할 목표 속도를 설정한다(turns/s)."""
        self._target_vel = turns_per_s

    def tick(self) -> None:
        """제어 루프마다 호출 — 목표 속도를 ODrive 로 전송한다."""
        self._axis.controller.input_vel = self._target_vel

    def state(self) -> dict:
        """정규화 텔레메트리.

        Returns
        -------
        dict with keys:
            target_vel  — 현재 set_velocity 값 (turns/s)
            actual_vel  — 엔코더 속도 추정 (turns/s)
            cur_a       — 모터 q 축 전류 (A)
        """
        if self._axis is None:
            return {"target_vel": self._target_vel, "actual_vel": 0.0, "cur_a": 0.0}
        return {
            "target_vel": self._target_vel,
            "actual_vel": self._axis.encoder.vel_estimate,
            "cur_a": self._axis.motor.current_control.Iq_measured,
        }

    def estop(self) -> None:
        """비상 정지 — 즉시 input_vel=0 후 IDLE 전환."""
        if self._axis is not None:
            self._axis.controller.input_vel = 0.0
            self._axis.requested_state = AXIS_STATE_IDLE
        self._target_vel = 0.0

    def close(self) -> None:
        """연결 해제 전 IDLE 로 내린다."""
        if self._axis is not None:
            self._axis.requested_state = AXIS_STATE_IDLE
