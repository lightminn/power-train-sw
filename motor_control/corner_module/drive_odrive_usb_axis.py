"""ODrive 3.6(USB) **다보드** 구동 드라이버 — `DriveOdriveCan` 과 동일 계약.

`drive_odrive_usb.py` 의 `DriveOdriveUsb` 는 `find_any()` 로 보드 1장의 axis1
하나만 잡는 레거시 단축 경로다(아직 `corner_module/teleop_dualsense.py` 가
쓰므로 건드리지 않는다). 이 모듈은 **3보드 6축**을 시리얼+축 인덱스로 주소지정
하고, 감속비·미러 반전·건강 키를 붙여 `ChassisManager` 가 CAN 과 동일하게 쓸 수
있게 한다.

연결·캘리·arm 시퀀스는 `motor_control/drive/bl70200/dualsense_usb_teleop.py`
에서 이식했다 — 벤치에서 3보드 6축 실회전이 확인된 코드다.

**평면 int 상수를 쓰는 이유** (`drive_odrive_usb.py:6-13`): 이 펌웨어의 odrive
라이브러리는 클래스 enum 객체를 config 에 대입하면 int 변환이 안 돼
``TypeError`` 가 난다. 부수 효과로 이 모듈은 `odrive` 없이 import 되므로
무하드웨어 pytest 가 그대로 돈다.

**지연 예산.** USB 는 속성 하나가 왕복 1회다. 50 Hz = 20 ms 안에 6축 쓰기 +
텔레메트리 읽기가 들어가야 하므로, **쓰기는 매 tick, 읽기는 축별 라운드로빈**
으로 나눈다. `state()` 는 캐시만 반환하며 USB 를 절대 건드리지 않는다 —
`CornerModule.tick()` 이 매 tick 호출하기 때문이다.
"""
import math
import time

from corner_module.actuator import DriveActuator

ODRIVE_VID = 0x1209
ODRIVE_PID = 0x0D32

# odrive.enums 의 평면 상수와 같은 값 (위 docstring 참조)
_AXIS_IDLE = 1                      # AXIS_STATE_IDLE
_AXIS_FULL_CALIBRATION = 3          # AXIS_STATE_FULL_CALIBRATION_SEQUENCE
_AXIS_CLOSED_LOOP = 8               # AXIS_STATE_CLOSED_LOOP_CONTROL
_CTRL_VELOCITY = 2                  # CONTROL_MODE_VELOCITY_CONTROL
_INPUT_PASSTHROUGH = 1              # INPUT_MODE_PASSTHROUGH

#: 캘리 스캔 각속도. ⚠️ 기본값 12.566 은 이 모터에서 offset 캘리가 깨진다
#: (2026-07-04 벤치 실측). 바꾸지 말 것.
CALIB_SCAN_OMEGA = 6.0
CALIB_SCAN_DISTANCE = 150
CALIB_RANGE = 0.05
CALIB_CURRENT_A = 8.0
CALIB_HEADROOM_CURRENT_A = 20.0


class UsbBoardPool:
    """USB ODrive 핸들 풀 — 보드 1장당 `find_any()` 를 1회만 돈다.

    같은 보드의 두 축이 하나의 핸들을 공유하므로, 6축 = 3회 탐색이다.

    Parameters
    ----------
    finder:
        테스트 주입용 ``serial -> board_handle``. 없으면 `odrive.find_any` 사용.
    enumerator:
        테스트 주입용 ``() -> [serial]``. 없으면 pyusb 로 VID/PID 열거.
    """

    def __init__(self, find_timeout: float = 20.0, finder=None, enumerator=None):
        self._find_timeout = find_timeout
        self._finder = finder
        self._enumerator = enumerator
        self._boards = {}

    def discover_serials(self) -> list:
        """USB 에 붙은 ODrive 시리얼 목록(정렬·중복제거). 실패하면 빈 리스트."""
        if self._enumerator is not None:
            return sorted({str(s) for s in self._enumerator()})
        try:
            import usb.core
            import usb.util
        except Exception:
            return []
        serials = []
        try:
            found = usb.core.find(find_all=True, idVendor=ODRIVE_VID,
                                  idProduct=ODRIVE_PID)
            for dev in found:
                try:
                    serial = usb.util.get_string(dev, dev.iSerialNumber)
                except Exception:
                    serial = None
                if serial:
                    serials.append(serial.strip())
        except Exception:
            return []
        return sorted(set(serials))

    def board(self, serial: str):
        if serial not in self._boards:
            self._boards[serial] = self._find(serial)
        return self._boards[serial]

    def _find(self, serial: str):
        if self._finder is not None:
            return self._finder(serial)
        import odrive
        handle = odrive.find_any(serial_number=serial, timeout=self._find_timeout)
        if handle is None:
            raise RuntimeError(
                "ODrive %s USB 미발견 — 케이블·전원·권한(udev) 확인." % serial)
        return handle

    def axis(self, serial: str, axis_index: int):
        if axis_index not in (0, 1):
            raise ValueError("axis_index must be 0 or 1, got %r" % (axis_index,))
        return getattr(self.board(serial), "axis%d" % axis_index)

    def close(self) -> None:
        self._boards.clear()


def calibrate_axis(axis, label: str, current_lim_a: float = 9.0,
                   timeout_s: float = 120.0, clock=None, sleep=None) -> bool:
    """축 1개 풀캘리 (~55 s 회전, 출력축이 자유로워야 한다).

    ⚠️ 캘리 결과는 **RAM-only** 라 전원 사이클마다 다시 해야 한다.
    """
    clock = clock or time.monotonic
    sleep = sleep or time.sleep
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0
    axis.motor.config.calibration_current = CALIB_CURRENT_A
    axis.motor.config.current_lim = CALIB_HEADROOM_CURRENT_A
    axis.encoder.config.calib_scan_omega = CALIB_SCAN_OMEGA
    axis.encoder.config.calib_scan_distance = CALIB_SCAN_DISTANCE
    axis.encoder.config.calib_range = CALIB_RANGE
    axis.requested_state = _AXIS_FULL_CALIBRATION
    started = clock()
    while axis.current_state != _AXIS_IDLE:
        if clock() - started > timeout_s:
            break
        sleep(0.5)
    axis.motor.config.current_lim = current_lim_a
    return bool(axis.motor.is_calibrated and axis.encoder.is_ready)


class DriveOdriveUsbAxis(DriveActuator):
    """ODrive 3.6 USB 축 1개 — velocity control, 바퀴 프레임 입출력.

    Parameters
    ----------
    pool, serial, axis_index:
        어느 보드의 어느 축인지. `axis_index` 1 = 각 보드의 M1 = 로봇 우측.
    node_id:
        같은 축의 CAN node 번호. 텔레메트리 라벨 통일용이며 통신에는 안 쓴다.
    gear_ratio:
        모터 회전수 / 바퀴 회전수 (기본 5.0). 양수·유한이어야 한다.
    invert:
        우측 바퀴의 물리적 미러 장착. 반전은 **USB 경계에서만** 하고 드라이버
        바깥은 전부 바퀴 프레임이다.
    stale_ms:
        마지막 성공 폴링 후 이 시간을 넘으면 ``state()["stale"]=True``.
        라운드로빈 주기(`poll_period_ticks / loop_hz`)의 3~4배로 둔다.
    poll_slot, poll_period_ticks:
        ``tick_index % poll_period_ticks == poll_slot`` 인 tick 에만 읽는다.
        6축이면 슬롯 0~5 · 주기 6 → 축당 갱신 주기 = 6 / loop_hz.
    """

    def __init__(self, pool, serial, axis_index, *, node_id=None,
                 gear_ratio: float = 5.0, invert: bool = False,
                 current_lim_a: float = 9.0, stale_ms: float = 500.0,
                 poll_slot: int = 0, poll_period_ticks: int = 6, clock=None):
        gear_ratio = float(gear_ratio)
        if not math.isfinite(gear_ratio) or gear_ratio <= 0.0:
            raise ValueError("gear_ratio must be finite and positive")
        if int(poll_period_ticks) < 1:
            raise ValueError("poll_period_ticks must be >= 1")
        self._pool = pool
        self._serial = str(serial)
        self._axis_index = int(axis_index)
        self._node_id = node_id
        self._gear_ratio = gear_ratio
        self._invert = bool(invert)
        self._sign = -1.0 if self._invert else 1.0
        self._current_lim_a = float(current_lim_a)
        self._stale_ms = float(stale_ms)
        self._poll_period_ticks = int(poll_period_ticks)
        self._poll_slot = int(poll_slot) % self._poll_period_ticks
        self._now = time.monotonic if clock is None else clock
        self._axis = None
        self._target_vel = 0.0
        self._actual_vel = 0.0
        self._cur_a = 0.0
        self._axis_error = 0
        self._axis_state = 0
        self._last_rx_ms = None
        self._tick_index = -1
        self._rx_polls = 0
        self._error_count = 0

    @property
    def invert(self) -> bool:
        return self._invert

    @property
    def label(self) -> str:
        return "%s/ax%d" % (self._serial[-6:], self._axis_index)

    def _now_ms(self) -> float:
        return self._now() * 1000.0

    # ------------------------------------------------------------------
    # Actuator 인터페이스
    # ------------------------------------------------------------------

    def connect(self) -> None:
        self._axis = self._pool.axis(self._serial, self._axis_index)

    def arm(self) -> None:
        """velocity-control + passthrough 로 폐루프 진입 (input_vel=0 점프 방지)."""
        axis = self._axis
        if not axis.motor.is_calibrated or not axis.encoder.is_ready:
            raise RuntimeError(
                "%s 미캘리 — 캘리는 RAM-only 라 전원 사이클마다 다시 해야 한다."
                % self.label)
        axis.error = 0
        axis.motor.error = 0
        axis.encoder.error = 0
        axis.controller.error = 0
        axis.motor.config.current_lim = self._current_lim_a
        axis.encoder.config.ignore_illegal_hall_state = True
        axis.controller.config.control_mode = _CTRL_VELOCITY
        axis.controller.config.input_mode = _INPUT_PASSTHROUGH
        axis.controller.input_vel = 0.0
        self._target_vel = 0.0
        axis.requested_state = _AXIS_CLOSED_LOOP
        self._poll_now()      # arm 직후 stale 오판 방지 — last_rx 시드
                              # (steer_ak40 이 이걸 빼서 첫 tick estop 이 났었다)

    def disarm(self) -> None:
        self._target_vel = 0.0
        self._safe(lambda axis: setattr(axis.controller, "input_vel", 0.0))
        self._safe(lambda axis: setattr(axis, "requested_state", _AXIS_IDLE))

    def set_velocity(self, turns_per_s: float) -> None:
        """다음 tick() 에 전송할 **바퀴** 목표 속도(turns/s)."""
        self._target_vel = turns_per_s

    def tick(self) -> None:
        """쓰기는 매 tick, 읽기는 자기 슬롯 차례에만."""
        self._tick_index += 1
        motor_tps = self._target_vel * self._gear_ratio * self._sign
        self._safe(lambda axis: setattr(axis.controller, "input_vel", motor_tps))
        if self._tick_index % self._poll_period_ticks == self._poll_slot:
            self._poll_now()

    def state(self) -> dict:
        """**캐시만** 반환한다 — USB 를 건드리지 않는다."""
        age_ms = (
            None if self._last_rx_ms is None
            else max(0.0, self._now_ms() - self._last_rx_ms)
        )
        return {
            "node_id": self._node_id,
            "serial": self._serial,
            "axis": self._axis_index,
            "target_vel": self._target_vel,
            "actual_vel": self._actual_vel * self._sign / self._gear_ratio,
            "cur_a": self._cur_a,
            "axis_error": self._axis_error,
            "axis_state": self._axis_state,
            "stale": age_ms is None or age_ms > self._stale_ms,
            "last_rx_age_ms": age_ms,
            "rx_polls": self._rx_polls,
            "error_count": self._error_count,
        }

    def estop(self) -> None:
        self._target_vel = 0.0
        self._safe(lambda axis: setattr(axis.controller, "input_vel", 0.0))
        self._safe(lambda axis: setattr(axis, "requested_state", _AXIS_IDLE))

    def close(self) -> None:
        self._safe(lambda axis: setattr(axis, "requested_state", _AXIS_IDLE))

    # ------------------------------------------------------------------
    # 내부 — 모든 USB 접근은 여기를 지난다
    # ------------------------------------------------------------------

    def _safe(self, action) -> bool:
        """USB 접근 1회. 예외는 흡수하고 error_count 만 올린다(제어 루프 보호)."""
        if self._axis is None:
            return False
        try:
            action(self._axis)
        except Exception:
            self._error_count += 1
            return False
        return True

    def _poll_now(self) -> None:
        """텔레메트리 4종을 한 번에 읽는다. 하나라도 실패하면 캐시를 갱신하지 않아
        `stale` 로 드러난다."""
        if self._axis is None:
            return
        try:
            axis = self._axis
            actual_vel = axis.encoder.vel_estimate
            cur_a = axis.motor.current_control.Iq_measured
            axis_error = axis.error
            axis_state = axis.current_state
        except Exception:
            self._error_count += 1
            return
        self._actual_vel = actual_vel
        self._cur_a = cur_a
        self._axis_error = axis_error
        self._axis_state = axis_state
        self._last_rx_ms = self._now_ms()
        self._rx_polls += 1
