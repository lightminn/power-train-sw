"""Pure tests for witnessed dual endpoint calibration; no ROS or serial."""

from copy import deepcopy

import pytest
import yaml

from dynamixel_control.dual_calibration_session import (
    DualCalibrationError, DualCalibrationSession)


PROFILE = {
    'backend': 'gripper', 'calibrated': True, 'actuator_ids': [3, 4],
    'open_tick': 100, 'close_tick': 0, 'safe_min_tick': 0,
    'safe_max_tick': 100, 'direction': 1,
    'motor_endpoints': {3: {'open': 100, 'close': 0},
                        4: {'open': 200, 'close': 50}},
    'required_operating_modes': {3: 4, 4: 3},
    'profile_velocity': 80, 'profile_acceleration': 25,
    'goal_pwm': 280, 'no_load_effort': 10, 'grasp_effort': 40,
    'grasp_threshold': 30, 'release_drop_threshold': 20, 'action_time': 2.5,
}


class Bridge:
    def __init__(self):
        self.state = {
            3: {'position': 10, 'torque': 1, 'hardware_error': 0, 'model': 1060},
            4: {'position': 20, 'torque': 1, 'hardware_error': 0, 'model': 1060},
        }
        self.jogs = []
        self.holds = []
        self.pair_jogs = []

    def read_dual_calibration_state(self):
        return deepcopy(self.state)

    def dual_calibration_jog(self, dxl_id, delta_deg):
        self.jogs.append((dxl_id, delta_deg))
        return self.state[dxl_id]['position']

    def dual_calibration_hold(self):
        positions = {dxl_id: sample['position']
                     for dxl_id, sample in self.state.items()}
        self.holds.append(positions)
        return positions

    def dual_calibration_pair_jog(self, delta_deg):
        self.pair_jogs.append(delta_deg)
        return {3: 10, 4: 20}


def capture_pair(session, bridge, label, id3, id4):
    bridge.state[3]['position'] = id3
    bridge.state[4]['position'] = id4
    return session.capture_open() if label == 'open' else session.capture_close()


def test_existing_endpoint_profile_requires_explicit_dual_recalibration():
    session = DualCalibrationSession(Bridge(), PROFILE)
    assert session.state == 'RECALIBRATION_REQUIRED'
    assert not session.is_ready


def test_capture_is_read_only_and_preserves_independent_per_motor_pairs():
    bridge = Bridge()
    session = DualCalibrationSession(bridge, PROFILE)
    session.start()
    assert capture_pair(session, bridge, 'open', 1000, 2000) == {3: 1000, 4: 2000}
    assert session.state == 'OPEN_CAPTURED_WAITING_FOR_CLOSE'
    assert capture_pair(session, bridge, 'close', 100, 300) == {3: 100, 4: 300}
    assert bridge.jogs == []
    candidate = session.get_candidate()
    assert candidate['motor_endpoints'] == {
        3: {'open': 1000, 'close': 100}, 4: {'open': 2000, 'close': 300}}
    assert session.normalized_progress({3: 1000, 4: 2000}) == {3: 1.0, 4: 1.0}
    assert session.normalized_progress({3: 100, 4: 300}) == {3: 0.0, 4: 0.0}


def test_save_is_explicit_atomic_and_reload_makes_session_ready(tmp_path):
    bridge = Bridge()
    session = DualCalibrationSession(bridge, PROFILE)
    session.start()
    capture_pair(session, bridge, 'open', 1000, 2000)
    capture_pair(session, bridge, 'close', 100, 300)
    session.validate()
    profile_path = tmp_path / 'tool_profiles.yaml'
    profile_path.write_text(yaml.safe_dump({'tool_profiles': {
        'dual_motor_gripper': PROFILE, 'spur_1motor_gripper': {'unchanged': True}}}))
    session.save(profile_path)
    saved = yaml.safe_load(profile_path.read_text())['tool_profiles']
    assert saved['spur_1motor_gripper'] == {'unchanged': True}
    dual = saved['dual_motor_gripper']
    assert dual['open_ticks'] == {3: 1000, 4: 2000}
    assert dual['close_ticks'] == {3: 100, 4: 300}
    assert dual['endpoint_calibration_verified'] is True
    session.reload(dual)
    assert session.state == 'READY'


def test_capture_allows_torque_off_but_jog_requires_torque_on():
    bridge = Bridge()
    session = DualCalibrationSession(bridge, PROFILE)
    with pytest.raises(DualCalibrationError):
        session.capture_open()
    session.start()
    bridge.state[3]['torque'] = 0
    assert session.capture_open() == {3: 10, 4: 20}
    assert bridge.jogs == []
    with pytest.raises(DualCalibrationError, match='Torque Enable is OFF'):
        session.jog_motor_degrees(3, 0.5)
    bridge.state[3]['torque'] = 1
    bridge.state[4]['hardware_error'] = 1
    with pytest.raises(DualCalibrationError, match='hardware error'):
        session.jog_motor_degrees(4, 0.5)


def test_only_configured_one_click_steps_are_forwarded_to_selected_motor():
    bridge = Bridge()
    session = DualCalibrationSession(bridge, PROFILE)
    session.start()
    session.jog_motor_degrees(3, 5.0)
    assert bridge.jogs == [(3, 5.0)]
    with pytest.raises(DualCalibrationError):
        session.jog_motor_degrees(4, 0.25)


def test_pair_jog_uses_one_guarded_bridge_operation():
    bridge = Bridge()
    session = DualCalibrationSession(bridge, PROFILE)
    session.start()
    assert session.jog_pair_degrees(-0.5) == {3: 10, 4: 20}
    assert bridge.pair_jogs == [-0.5]


def test_hold_requires_active_healthy_session_and_holds_both_positions():
    bridge = Bridge()
    session = DualCalibrationSession(bridge, PROFILE)
    with pytest.raises(DualCalibrationError):
        session.hold()
    session.start()
    assert session.hold() == {3: 10, 4: 20}
    assert bridge.holds == [{3: 10, 4: 20}]
