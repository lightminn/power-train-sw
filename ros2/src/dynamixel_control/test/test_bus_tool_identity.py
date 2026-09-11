"""물리 ID 조합 감지와 탈거 후 자동 전환의 fail-closed 규칙."""

from types import SimpleNamespace

from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge
from dynamixel_control.tool_manager import BusToolIdentityProvider


PROFILES = {
    'dual_motor_gripper': {'actuator_ids': [3, 4]},
    'spur_1motor_gripper': {'actuator_ids': [5]},
    'cleaner': {'actuator_ids': [2]},
}


def test_bus_identity_requires_one_exact_profile_signature():
    present = {3, 4}
    provider = BusToolIdentityProvider(
        PROFILES, lambda actuator_id: actuator_id in present)
    assert provider.detected_tool_type() == 'dual_motor_gripper'
    present.clear()
    assert provider.detected_tool_type() is None
    assert provider.last_reason == 'no supported tool actuator detected'
    present.update({3, 5})
    assert provider.detected_tool_type() is None
    assert provider.last_present_ids == {3, 5}


def test_bus_identity_excludes_arm_id_profiles():
    profiles = dict(PROFILES, unsafe={'actuator_ids': [11]})
    provider = BusToolIdentityProvider(
        profiles, lambda actuator_id: actuator_id == 11, excluded_ids={11})
    assert 11 not in provider.candidate_ids
    assert provider.detected_tool_type() is None


def test_explicit_rescan_keeps_observed_signature_for_diagnostics():
    bridge = object.__new__(MoveItDynamixelBridge)
    provider = BusToolIdentityProvider(
        PROFILES, lambda actuator_id: actuator_id == 5)
    bridge._bus_tool_identity = provider
    bridge.mock_mode = False
    bridge.port_connected = True
    bridge._tool_detection_reason = ''
    assert bridge._rescan_physical_tool() == 'spur_1motor_gripper'
    assert provider.last_present_ids == {5}
    assert bridge._tool_detection_reason == ''


def test_profile_discovery_uses_the_shared_bus_lock():
    """The profile ping path must not race the periodic signature probe."""
    source = (__import__('pathlib').Path(__file__).parents[1] /
              'dynamixel_control/moveit_dynamixel_bridge.py').read_text()
    start = source.index('    def _probe_tool_id')
    end = source.index('    def _rescan_physical_tool', start)
    assert 'with self._bus_lock:' in source[start:end]
    assert 'ADDR_HARDWARE_ERROR_STATUS' in source[start:end]


def test_stable_active_tool_uses_only_its_own_fast_probe():
    """A live cleaner must not probe absent gripper IDs every poll period."""
    bridge = object.__new__(MoveItDynamixelBridge)
    provider = BusToolIdentityProvider(PROFILES, lambda _actuator_id: False)
    bridge._bus_tool_identity = provider
    bridge.tool_type = 'cleaner'
    bridge.tool_ids = [2]
    bridge._tool_samples = {}
    bridge._tool_detection_reason = 'old state'
    probed = []
    bridge._probe_tool_id = lambda actuator_id: (probed.append(actuator_id) or True)
    assert bridge._observe_active_tool_signature() == 'cleaner'
    assert probed == [2]
    assert provider.last_present_ids == {2}
    assert provider.last_reason == ''


def test_fresh_dual_feedback_prevents_false_detach_when_ping_is_lost():
    bridge = object.__new__(MoveItDynamixelBridge)
    provider = BusToolIdentityProvider(PROFILES, lambda _actuator_id: False)
    bridge._bus_tool_identity = provider
    bridge.tool_type = 'dual_motor_gripper'
    bridge.tool_ids = [3, 4]
    bridge._tool_detection_reason = 'old state'
    bridge._tool_samples = {
        3: {'online': True, 'hardware_error': 0, 'position': -690},
        4: {'online': True, 'hardware_error': 0, 'position': 2545},
    }
    bridge._probe_tool_id = lambda _actuator_id: (_ for _ in ()).throw(
        AssertionError('fresh feedback must avoid a redundant ping'))
    assert bridge._observe_active_tool_signature() == 'dual_motor_gripper'
    assert provider.last_present_ids == {3, 4}
    assert provider.last_reason == ''


def _bridge(observations):
    bridge = object.__new__(MoveItDynamixelBridge)
    bridge._bus_tool_identity = SimpleNamespace(
        detected_tool_type=lambda: observations.pop(0),
        supported_tool_types={
            'cleaner', 'dual_motor_gripper', 'spur_1motor_gripper'},
        last_reason='no supported tool actuator detected')
    bridge.mock_mode = False
    bridge.port_connected = True
    bridge.tool_detection_confirmations = 2
    bridge._tool_detection_observation = None
    bridge._tool_detection_count = 0
    bridge._physical_tool_detached = False
    bridge._tool_detection_reason = ''
    bridge.tool_type = 'dual_motor_gripper'
    bridge.control_scope = 'END_EFFECTOR_ONLY'
    bridge._arm_fsm_state = None
    bridge._gripper_goal_active = False
    bridge.tool_discovered = True
    bridge.tool_detached = False
    bridge.emergency_stop_active = False
    bridge._tool_change_lock = __import__('threading').Lock()
    bridge._tool_change_pending = None
    bridge._tool_change_error = ''
    bridge.get_logger = lambda: SimpleNamespace(error=lambda message: None)
    bridge._stops = []
    bridge._stop_tool = lambda reason: bridge._stops.append(reason)
    bridge._switches = []
    bridge._switch_tool_runtime = lambda tool: (
        bridge._switches.append(tool), setattr(bridge, 'tool_type', tool))
    return bridge


def test_auto_switch_requires_confirmed_detach_then_confirmed_new_tool():
    bridge = _bridge([
        None, None, 'spur_1motor_gripper', 'spur_1motor_gripper'])
    for _ in range(4):
        bridge._poll_physical_tool()
    assert len(bridge._stops) == 1
    assert bridge._switches == ['spur_1motor_gripper']
    assert bridge.tool_type == 'spur_1motor_gripper'
    assert not bridge._physical_tool_detached


def test_id2_cleaner_signature_switches_after_confirmed_removal():
    bridge = _bridge([None, None, 'cleaner', 'cleaner'])
    for _ in range(4):
        bridge._poll_physical_tool()
    assert bridge._switches == ['cleaner']
    assert bridge.tool_type == 'cleaner'


def test_different_signature_first_latches_stop_before_switching():
    bridge = _bridge(['spur_1motor_gripper', 'spur_1motor_gripper'])
    bridge._poll_physical_tool()
    bridge._poll_physical_tool()
    assert len(bridge._stops) == 1
    assert bridge._switches == []
    assert bridge._physical_tool_detached


def test_confirmed_replacement_clears_manual_detach_latch_but_not_estop():
    bridge = _bridge([
        None, None, 'spur_1motor_gripper', 'spur_1motor_gripper'])
    bridge.tool_detached = True
    for _ in range(4):
        bridge._poll_physical_tool()
    assert bridge._switches == ['spur_1motor_gripper']
    assert not bridge.tool_detached
    bridge = _bridge([None, None, 'spur_1motor_gripper', 'spur_1motor_gripper'])
    bridge.emergency_stop_active = True
    for _ in range(4):
        bridge._poll_physical_tool()
    assert bridge._switches == []
    assert 'emergency stop' in bridge._tool_detection_reason


def test_same_tool_reattachment_is_revalidated_without_runtime_switch():
    bridge = _bridge([None, None, 'dual_motor_gripper',
                      'dual_motor_gripper'])
    bridge.tool_ids = [3, 4]
    bridge.tool_selection = SimpleNamespace(valid=True)
    bridge.group_sync_read = SimpleNamespace(addParam=lambda actuator_id: True)
    bridge.active_ids = set()
    bridge._discover_tool_ids = lambda: True
    bridge.tool_fsm = SimpleNamespace(
        startup=lambda: SimpleNamespace(name='READY'), fault_reason='')
    for _ in range(4):
        bridge._poll_physical_tool()
    assert bridge._switches == []
    assert bridge.tool_discovered
    assert bridge.active_ids == {3, 4}
    assert not bridge._physical_tool_detached


def test_cleaner_id2_signature_is_polled_as_a_physical_tool():
    bridge = _bridge(['cleaner'])
    bridge.tool_type = 'cleaner'
    bridge._poll_physical_tool()
    assert bridge._stops == []
    assert not bridge._physical_tool_detached


def test_full_robot_waits_for_safe_arm_state_before_switch():
    bridge = _bridge([
        None, None, 'spur_1motor_gripper', 'spur_1motor_gripper',
        'spur_1motor_gripper'])
    bridge.control_scope = 'FULL_ROBOT'
    bridge._arm_fsm_state = 'EXECUTE'
    for _ in range(4):
        bridge._poll_physical_tool()
    assert bridge._switches == []
    assert 'waiting for safe arm state' in bridge._tool_detection_reason
    bridge._arm_fsm_state = 'STOWED_LOCKED'
    bridge._poll_physical_tool()
    assert bridge._switches == ['spur_1motor_gripper']
