"""USB 다보드 구동 드라이버 — odrive 없이 fake 핸들 트리로 검증한다.

fake 는 우리 ABC 가 아니라 **odrive 라이브러리의 객체 트리**를 흉내낸다
(axis.controller.input_vel, axis.motor.current_control.Iq_measured 등).
그래서 corner_module/fake.py 가 아니라 이 테스트 파일에 둔다.
"""
import pytest

from corner_module.drive_odrive_usb_axis import (
    DriveOdriveUsbAxis, UsbBoardPool, calibrate_axis,
)

_AXIS_IDLE = 1
_AXIS_CLOSED_LOOP = 8


class _Ns:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class FakeAxis:
    """odrive axis 객체 트리 흉내. fail=True 면 모든 속성 접근이 터진다."""

    def __init__(self, calibrated=True, fail=False):
        self.fail = fail
        self.error = 0
        self.requested_state = _AXIS_IDLE
        self.current_state = _AXIS_IDLE
        self.motor = _Ns(
            error=0,
            is_calibrated=calibrated,
            config=_Ns(current_lim=0.0, calibration_current=0.0),
            current_control=_Ns(Iq_measured=1.25),
        )
        self.encoder = _Ns(
            error=0,
            is_ready=calibrated,
            vel_estimate=0.0,
            config=_Ns(ignore_illegal_hall_state=False, calib_scan_omega=0.0,
                       calib_scan_distance=0, calib_range=0.0),
        )
        self.controller = _Ns(
            error=0,
            input_vel=0.0,
            config=_Ns(control_mode=0, input_mode=0),
        )

    def __getattribute__(self, name):
        if name != "fail" and object.__getattribute__(self, "fail"):
            raise OSError("USB gone")
        return object.__getattribute__(self, name)


class FakeBoard:
    def __init__(self, **axes):
        for name, axis in axes.items():
            setattr(self, name, axis)


def make_driver(axis=None, **kw):
    axis = axis or FakeAxis()
    pool = UsbBoardPool(finder=lambda serial: FakeBoard(axis0=axis, axis1=axis))
    kw.setdefault("clock", lambda: make_driver.now)
    driver = DriveOdriveUsbAxis(pool, "SN1", 0, **kw)
    driver.connect()
    return driver, axis


make_driver.now = 0.0


# ── 프레임 변환 ──────────────────────────────────────────────────────────


def test_wheel_command_is_scaled_by_the_gear_ratio():
    driver, axis = make_driver(gear_ratio=5.0)
    driver.arm()

    driver.set_velocity(1.0)          # 바퀴 1 rev/s
    driver.tick()

    assert axis.controller.input_vel == pytest.approx(5.0)   # 모터 5 turns/s


def test_invert_flips_only_at_the_driver_boundary():
    driver, axis = make_driver(gear_ratio=5.0, invert=True)
    driver.arm()

    driver.set_velocity(1.0)
    driver.tick()

    assert axis.controller.input_vel == pytest.approx(-5.0)


def test_measured_velocity_is_reported_in_the_wheel_frame():
    driver, axis = make_driver(gear_ratio=5.0, invert=True, poll_period_ticks=1)
    driver.arm()
    axis.encoder.vel_estimate = -5.0        # 모터 프레임
    driver.tick()

    assert driver.state()["actual_vel"] == pytest.approx(1.0)   # 바퀴 프레임


# ── 폴링 스케줄 ──────────────────────────────────────────────────────────


def test_telemetry_is_polled_only_on_the_assigned_slot():
    driver, axis = make_driver(poll_slot=2, poll_period_ticks=3)
    driver.arm()
    before = driver.state()["rx_polls"]

    for _ in range(3):
        driver.tick()

    assert driver.state()["rx_polls"] == before + 1


def test_command_is_written_every_tick_regardless_of_slot():
    driver, axis = make_driver(poll_slot=0, poll_period_ticks=6)
    driver.arm()

    for expected in (0.2, 0.4, 0.6):
        driver.set_velocity(expected)
        driver.tick()
        assert axis.controller.input_vel == pytest.approx(expected * 5.0)


def test_state_never_touches_usb():
    """CornerModule.tick() 이 매 tick 부르므로 여기서 I/O 하면 예산이 터진다."""
    driver, axis = make_driver(poll_period_ticks=1)
    driver.arm()
    driver.tick()
    axis.fail = True                       # 이제 어떤 속성 접근도 터진다

    snapshot = driver.state()              # 예외가 나면 안 된다

    assert snapshot["target_vel"] == 0.0


# ── 건강 · 예외 흡수 ─────────────────────────────────────────────────────


def test_arm_seeds_the_receive_timestamp():
    """arm 직후 stale 오판이 나면 첫 tick 에 estop 이 걸린다 (steer_ak40 실사고)."""
    make_driver.now = 100.0
    driver, _axis = make_driver(stale_ms=500.0)
    driver.arm()

    assert driver.state()["stale"] is False


def test_stale_turns_true_when_polls_stop_landing():
    make_driver.now = 100.0
    driver, axis = make_driver(stale_ms=500.0, poll_period_ticks=1)
    driver.arm()
    driver.tick()
    assert driver.state()["stale"] is False

    axis.fail = True
    make_driver.now = 100.9                # 900 ms 경과
    with pytest.raises(RuntimeError, match="USB velocity write failed"):
        driver.tick()

    assert driver.state()["stale"] is True


def test_usb_control_failure_is_propagated_and_counted():
    driver, axis = make_driver(poll_period_ticks=1)
    driver.arm()
    axis.fail = True

    driver.set_velocity(0.5)
    with pytest.raises(RuntimeError, match="USB velocity write failed"):
        driver.tick()                      # 차체에서 실패를 latch하고 나머지 축을 정지

    assert driver.state()["error_count"] >= 1


def test_axis_error_is_surfaced_for_the_corner_fault_check():
    driver, axis = make_driver(poll_period_ticks=1)
    driver.arm()
    axis.error = 0x40
    driver.tick()

    assert driver.state()["axis_error"] == 0x40


# ── 상태 전이 ────────────────────────────────────────────────────────────


def test_arm_refuses_an_uncalibrated_axis():
    axis = FakeAxis(calibrated=False)
    driver, _ = make_driver(axis=axis)

    with pytest.raises(RuntimeError, match="캘리"):
        driver.arm()


def test_arm_zeroes_the_command_before_closing_the_loop():
    driver, axis = make_driver()
    driver.set_velocity(9.0)

    driver.arm()

    assert axis.controller.input_vel == 0.0
    assert axis.requested_state == _AXIS_CLOSED_LOOP


def test_estop_zeroes_and_idles():
    driver, axis = make_driver()
    driver.arm()
    driver.set_velocity(1.0)

    driver.estop()

    assert axis.controller.input_vel == 0.0
    assert axis.requested_state == _AXIS_IDLE
    assert driver.state()["target_vel"] == 0.0


def test_estop_reports_a_dead_link_after_attempting_both_writes():
    driver, axis = make_driver()
    driver.arm()
    axis.fail = True

    before = driver.state()["error_count"]
    with pytest.raises(RuntimeError, match="USB stop write failed"):
        driver.estop()
    assert driver.state()["error_count"] == before + 1
    # FakeAxis blocks reads but still accepts the independent IDLE setattr.
    assert object.__getattribute__(axis, "requested_state") == _AXIS_IDLE

    assert driver.state()["target_vel"] == 0.0


# ── 보드 풀 ──────────────────────────────────────────────────────────────


def test_board_pool_finds_each_board_once():
    calls = []

    def finder(serial):
        calls.append(serial)
        return FakeBoard(axis0=FakeAxis(), axis1=FakeAxis())

    pool = UsbBoardPool(finder=finder)
    pool.axis("SN1", 0)
    pool.axis("SN1", 1)
    pool.axis("SN2", 0)

    assert calls == ["SN1", "SN2"]


def test_board_pool_enumerator_is_sorted_and_deduped():
    pool = UsbBoardPool(enumerator=lambda: ["B", "A", "B"])

    assert pool.discover_serials() == ["A", "B"]


# ── 캘리브레이션 ─────────────────────────────────────────────────────────


def test_calibrate_sets_the_verified_scan_omega():
    """0.5 세션 실측: 기본 12.566 이면 offset 캘리가 깨진다."""
    axis = FakeAxis(calibrated=False)
    ticks = iter([0.0, 1.0, 2.0])

    def clock():
        return next(ticks, 3.0)

    def sleep(_seconds):
        axis.current_state = _AXIS_IDLE

    calibrate_axis(axis, "test", current_lim_a=9.0, clock=clock, sleep=sleep)

    assert axis.encoder.config.calib_scan_omega == pytest.approx(6.0)
    assert axis.motor.config.current_lim == pytest.approx(9.0)
