"""끝점 캡처와 부하 캘리브레이션 전달 과정의 통합 테스트."""

from types import SimpleNamespace

import pytest

from dynamixel_control import gripper_calibration as endpoint_tool
from dynamixel_control import gripper_load_calibration as load_tool


class FakeEndpointBus:
    """직렬 하드웨어에 접근하지 않고 생명주기 호출을 기록한다."""

    instances = []

    def __init__(self):
        self.opened = False
        self.closed = False
        self.operating_modes = {3: 4, 4: 4}
        self.__class__.instances.append(self)

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True


class FakeMotionBus:
    """임시 상대/절대 동작 테스트용 단일 ID 가짜 구현."""

    instances = []

    def __init__(self):
        self.closed = False
        self.writes = []
        self.profile = (5, 20)
        self.snapshots = iter((
            {"position": 2100, "velocity": 1, "current": 8,
             "moving_status": 50, "hardware_error": 0},
            {"position": 2300, "velocity": 0, "current": 4,
             "moving_status": 49, "hardware_error": 0},
        ))
        self.__class__.instances.append(self)

    def open(self):
        pass

    def close(self):
        self.closed = True

    def snapshot(self, dxl_id):
        assert dxl_id == 5
        return {
            "model": 1060, "operating_mode": 3, "torque": 0,
            "hardware_error": 0, "position": 2000, "load": 0,
        }

    def write_goal(self, dxl_id, goal):
        self.writes.append(("goal", dxl_id, goal))

    def set_profile(self, dxl_id, acceleration, velocity):
        self.profile = (acceleration, velocity)
        self.writes.append(("profile", dxl_id, acceleration, velocity))

    def set_torque(self, dxl_id, enabled):
        self.writes.append(("torque", dxl_id, enabled))

    def read4(self, dxl_id, address, _label):
        assert dxl_id == 5
        if address == endpoint_tool.ADDR_PROFILE_ACCELERATION:
            return self.profile[0]
        if address == endpoint_tool.ADDR_PROFILE_VELOCITY:
            return self.profile[1]
        raise AssertionError(f"unexpected read4 address {address}")

    def read1(self, dxl_id, address, _label):
        assert dxl_id == 5
        assert address == endpoint_tool.ADDR_TORQUE_ENABLE
        return 0

    def motion_snapshot(self, dxl_id):
        assert dxl_id == 5
        return next(self.snapshots)


def motion_args(stage="move-relative", execute=False, **overrides):
    values = {
        "stage": stage,
        "id": 5,
        "execute": execute,
        "delta_ticks": 300,
        "goal_ticks": 2300,
        "profile_acceleration": 5,
        "profile_velocity": 20,
        "max_abs_current": 100,
        "stall_timeout": 2.0,
        "timeout": 10.0,
        "sample_hz": 1000.0,
        "goal_tolerance": 2,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_endpoint_capture_file_is_consumed_without_schema_translation(
        tmp_path, monkeypatch):
    """생성 파일과 부하 캘리브레이션 파서를 함께 검증한다."""
    output = tmp_path / "gripper_endpoints.json"
    captures = iter(({3: 739, 4: 2106}, {3: 1100, 4: 1700}))
    monkeypatch.setattr(endpoint_tool, "Bus", FakeEndpointBus)
    monkeypatch.setattr(
        endpoint_tool, "require_safe_read_state", lambda *args, **kwargs: {})
    monkeypatch.setattr(
        endpoint_tool, "capture_endpoint", lambda *args: next(captures))

    endpoint_tool.stage_endpoints(SimpleNamespace(
        samples=7, min_span_ticks=50, output=output))

    data = endpoint_tool.load_endpoints(output)
    assert data["id3_open_tick"] == 739
    assert data["id3_close_tick"] == 1100
    assert data["id4_open_tick"] == 2106
    assert data["id4_close_tick"] == 1700
    assert data["operating_modes"] == {"3": 4, "4": 4}
    endpoints = load_tool.load_endpoints(output)
    assert endpoints == {
        3: {"open": 739, "close": 1100},
        4: {"open": 2106, "close": 1700},
    }
    assert endpoints.operating_modes == {3: 4, 4: 4}
    assert load_tool.goals_for_ratio(0.0, endpoints) == {3: 739, 4: 2106}
    assert load_tool.goals_for_ratio(1.0, endpoints) == {3: 1100, 4: 1700}
    assert load_tool.ratio_for_position(3, 1100, endpoints) == 1.0
    assert load_tool.ratio_for_position(4, 1700, endpoints) == 1.0
    assert FakeEndpointBus.instances[-1].closed


@pytest.mark.parametrize("samples,min_span", [(0, 50), (7, 0)])
def test_invalid_capture_arguments_fail_before_opening_bus(
        tmp_path, samples, min_span):
    """직렬 포트를 획득하기 전에 사용할 수 없는 캡처를 거부한다."""
    FakeEndpointBus.instances.clear()
    with pytest.raises(endpoint_tool.CalibrationError):
        endpoint_tool.stage_endpoints(SimpleNamespace(
            samples=samples, min_span_ticks=min_span,
            output=tmp_path / "endpoints.json"))
    assert not FakeEndpointBus.instances


def test_relative_motion_dry_run_performs_no_writes(monkeypatch):
    FakeMotionBus.instances.clear()
    monkeypatch.setattr(endpoint_tool, "Bus", FakeMotionBus)
    endpoint_tool.stage_motion(motion_args(delta_ticks=-300))
    bus = FakeMotionBus.instances[-1]
    assert bus.writes == []
    assert bus.closed


def test_relative_motion_uses_only_id5_and_always_disables_torque(monkeypatch):
    FakeMotionBus.instances.clear()
    monkeypatch.setattr(endpoint_tool, "Bus", FakeMotionBus)
    endpoint_tool.stage_motion(motion_args(execute=True))
    writes = FakeMotionBus.instances[-1].writes
    assert writes == [
        ("goal", 5, 2000),
        ("profile", 5, 5, 20),
        ("torque", 5, True),
        ("goal", 5, 2300),
        ("torque", 5, False),
    ]


def test_absolute_motion_goal_and_arm_ids_are_guarded():
    args = motion_args(stage="move-absolute", goal_ticks=1234)
    assert endpoint_tool.motion_goal(args, 2000) == (2000, 1234)
    with pytest.raises(endpoint_tool.CalibrationError, match="arm motor"):
        endpoint_tool.validate_motion_args(motion_args(id=14))
    with pytest.raises(endpoint_tool.CalibrationError, match="only ID 5"):
        endpoint_tool.validate_motion_args(motion_args(id=6))


def test_out_of_range_relative_goal_is_rejected():
    with pytest.raises(endpoint_tool.CalibrationError, match="outside"):
        endpoint_tool.motion_goal(motion_args(delta_ticks=-300), 100)


def test_current_abort_still_disables_torque(monkeypatch):
    bus = FakeMotionBus()
    bus.snapshots = iter((
        {"position": 2001, "velocity": 0, "current": 100,
         "moving_status": 50, "hardware_error": 0},
    ))
    monkeypatch.setattr(endpoint_tool, "Bus", lambda: bus)
    with pytest.raises(endpoint_tool.CalibrationError, match="current"):
        endpoint_tool.stage_motion(motion_args(execute=True))
    assert bus.writes[-1] == ("torque", 5, False)
    assert bus.closed
