"""Unit tests for guarded two-motor gripper calibration."""

from types import SimpleNamespace
import sys

import pytest

from dynamixel_control import gripper_load_calibration as load_calibration
from dynamixel_control.gripper_load_calibration import (
    CalibrationError,
    CalibrationSession,
    goal_for_mode,
    goals_for_ratio,
    load_endpoints,
    ratio_for_position,
    unwrap_position,
    update_asymmetry_count,
    validate_target_ratio,
)


ENDPOINTS = {
    3: {"open": 1180, "close": -164},
    4: {"open": 2510, "close": 1212},
}


def test_endpoint_mapping_and_distinct_goals():
    assert goals_for_ratio(0.0, ENDPOINTS) == {3: 1180, 4: 2510}
    assert goals_for_ratio(0.5, ENDPOINTS) == {3: 508, 4: 1861}
    assert goals_for_ratio(1.0, ENDPOINTS) == {3: -164, 4: 1212}
    assert all(goals_for_ratio(r, ENDPOINTS)[3] !=
               goals_for_ratio(r, ENDPOINTS)[4]
               for r in (0.0, 0.5, 0.7, 1.0))


def test_wrap_and_unwrap_are_continuous():
    assert unwrap_position(5, 4092) == -4
    assert unwrap_position(-4, 4080) == -16
    assert ratio_for_position(3, -164, ENDPOINTS) == pytest.approx(1.0)
    assert goal_for_mode(-164, 4) == -164
    assert goal_for_mode(4200, 4) == 4200
    assert goal_for_mode(-164, 3) == 3932


def test_ratio_range_guard():
    validate_target_ratio(0.0)
    validate_target_ratio(0.70)
    validate_target_ratio(1.10)
    with pytest.raises(CalibrationError):
        validate_target_ratio(1.101)
    with pytest.raises(ValueError):
        goals_for_ratio(-0.1, ENDPOINTS)


def test_missing_endpoint_file_has_actionable_error(tmp_path):
    missing = tmp_path / "missing.json"
    with pytest.raises(
            CalibrationError, match="run gripper_calibration endpoints first"):
        load_endpoints(missing)


def test_main_returns_failure_without_opening_port_when_endpoints_missing(
        tmp_path, monkeypatch, capsys):
    missing = tmp_path / "missing.json"

    def unexpected_bus(*args, **kwargs):
        pytest.fail("serial bus must not be constructed without endpoints")

    monkeypatch.setattr(load_calibration, "DynamixelBus", unexpected_bus)
    monkeypatch.setattr(sys, "argv", [
        "gripper_load_calibration", "--read-only", "--endpoints",
        str(missing),
    ])
    with pytest.raises(SystemExit) as caught:
        load_calibration.main()
    assert caught.value.code == 1
    assert "run gripper_calibration endpoints first" in capsys.readouterr().out


def test_endpoint_file_drives_ratio_and_goals(tmp_path):
    path = tmp_path / "endpoints.json"
    path.write_text(
        '{"id3_open_tick": 700, "id3_close_tick": 900, '
        '"id4_open_tick": 2200, "id4_close_tick": 1800}')
    endpoints = load_endpoints(path)
    assert ratio_for_position(3, 750, endpoints) == pytest.approx(0.25)
    assert ratio_for_position(4, 2100, endpoints) == pytest.approx(0.25)
    assert goals_for_ratio(0.5, endpoints) == {3: 800, 4: 2000}


def test_endpoint_file_preserves_continuous_ticks_and_mode_metadata(tmp_path):
    path = tmp_path / "endpoints.json"
    path.write_text(
        '{"operating_modes":{"3":4,"4":4},'
        '"id3_open_tick":1180,"id3_close_tick":-164,'
        '"id4_open_tick":4200,"id4_close_tick":5100}')
    endpoints = load_endpoints(path)
    assert endpoints[3] == {"open": 1180, "close": -164}
    assert endpoints[4] == {"open": 4200, "close": 5100}
    assert endpoints.operating_modes == {3: 4, 4: 4}


def test_asymmetry_ignores_quantization_and_requires_consecutive_samples():
    assert update_asymmetry_count([0, 2], 3, 0, 2) == 0
    assert update_asymmetry_count([0, 3], 3, 0, 2) == 1
    with pytest.raises(CalibrationError, match="sustained asymmetric"):
        update_asymmetry_count([0, 4], 3, 1, 2)


def test_asymmetry_counter_resets_when_both_stall_or_move():
    assert update_asymmetry_count([0, 0], 3, 1, 2) == 0
    assert update_asymmetry_count([3, 5], 3, 1, 2) == 0


class FakeBus:
    def __init__(self):
        self.device = "/dev/fake"
        self.disabled = False
        self.closed = False
        self.opened = True
        self.endpoints = ENDPOINTS

    def set_torque(self, enabled):
        assert enabled is False
        self.disabled = True

    def read_states(self):
        return {
            i: {"torque": 0, "hardware_error": 0}
            for i in (3, 4)
        }

    def close(self):
        self.closed = True
        self.opened = False


def test_emergency_stop_and_cleanup_disable_both(tmp_path):
    bus = FakeBus()
    args = SimpleNamespace(armed=False, output_dir=tmp_path,
                           load_stop_threshold=300, max_ratio=0.7)
    session = CalibrationSession(bus, args)
    session.emergency_stop()
    assert bus.disabled
    session.cleanup()
    assert bus.disabled and bus.closed


def test_cleanup_closes_port_even_if_saving_fails(tmp_path, monkeypatch):
    bus = FakeBus()
    args = SimpleNamespace(armed=False, output_dir=tmp_path,
                           load_stop_threshold=300, max_ratio=0.7)
    session = CalibrationSession(bus, args)

    def fail_save():
        raise OSError("disk failure")

    monkeypatch.setattr(session, "save", fail_save)
    with pytest.raises(OSError, match="disk failure"):
        session.cleanup()
    assert bus.disabled and bus.closed


def test_out_of_range_diagnostic_includes_actual_ratio_and_range(tmp_path):
    bus = FakeBus()
    args = SimpleNamespace(
        armed=False, output_dir=tmp_path, load_stop_threshold=300,
        max_ratio=0.7, max_ratio_difference=0.05,
        path_ratio_tolerance=0.08)
    session = CalibrationSession(bus, args)
    states = {
        3: {"position_unwrapped": 739, "ratio": ratio_for_position(
                3, 739, ENDPOINTS), "hardware_error": 0,
            "profile_acceleration": 25, "profile_velocity": 80, "load": 0},
        4: {"position_unwrapped": 2106, "ratio": ratio_for_position(
                4, 2106, ENDPOINTS), "hardware_error": 0,
            "profile_acceleration": 25, "profile_velocity": 80, "load": 0},
    }
    states[3]["position_unwrapped"] = 1300
    states[3]["ratio"] = ratio_for_position(3, 1300, ENDPOINTS)
    with pytest.raises(CalibrationError) as caught:
        session.validate_state(states)
    message = str(caught.value)
    assert "actual=1300" in message
    assert "ratio=-0.089" in message
    assert "allowed ticks=-164..1180" in message
    assert "ratio=0.000..1.000" in message
