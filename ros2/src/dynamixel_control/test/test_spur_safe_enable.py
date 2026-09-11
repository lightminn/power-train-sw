"""ID5 torque-on must configure and park the motor before enabling it."""

import threading

import pytest

from dynamixel_control.moveit_dynamixel_bridge import (
    ADDR_GOAL_POSITION, ADDR_HARDWARE_ERROR_STATUS, ADDR_OPERATING_MODE,
    ADDR_PRESENT_POSITION, ADDR_PROFILE_ACCELERATION, ADDR_PROFILE_VELOCITY,
    ADDR_TORQUE_ENABLE, MoveItDynamixelBridge)


class Packet:
    def __init__(self, mode=3, position=3064):
        self.registers = {
            ADDR_HARDWARE_ERROR_STATUS: 0,
            ADDR_TORQUE_ENABLE: 0,
            ADDR_OPERATING_MODE: mode,
            ADDR_PRESENT_POSITION: position,
            ADDR_PROFILE_ACCELERATION: 0,
            ADDR_PROFILE_VELOCITY: 0,
            ADDR_GOAL_POSITION: 999,
        }
        self.writes = []

    def _read(self, address):
        return self.registers[address], 0, 0

    def read1ByteTxRx(self, _port, _id, address):
        return self._read(address)

    def read2ByteTxRx(self, _port, _id, address):
        return self._read(address)

    def read4ByteTxRx(self, _port, _id, address):
        return self._read(address)

    def write1ByteTxRx(self, _port, _id, address, value):
        self.writes.append((address, value))
        self.registers[address] = value
        return 0, 0

    def write2ByteTxRx(self, _port, _id, address, value):
        self.writes.append((address, value))
        self.registers[address] = value
        return 0, 0

    def write4ByteTxRx(self, _port, _id, address, value):
        self.writes.append((address, value))
        self.registers[address] = value
        return 0, 0


def _bridge(packet):
    bridge = object.__new__(MoveItDynamixelBridge)
    bridge.mock_mode = False
    bridge.read_only = False
    bridge.tool_type = 'spur_1motor_gripper'
    bridge.tool_ids = [5]
    bridge.tool_profile = {
        'required_operating_modes': {5: 3},
        'profile_acceleration': 5,
        'profile_velocity': 20,
        'safe_min_tick': 2945,
        'safe_max_tick': 3752,
        'motor_endpoints': {},
    }
    bridge._fsm_allowlist = {5}
    bridge.packet_handler = packet
    bridge.port_handler = object()
    bridge._bus_lock = threading.RLock()
    bridge.torque_enabled_ids = set()
    return bridge


def test_spur_enable_profiles_and_parks_before_torque_on():
    packet = Packet()
    bridge = _bridge(packet)

    bridge.set_torque(5, True)

    assert packet.writes == [
        (ADDR_PROFILE_ACCELERATION, 5),
        (ADDR_PROFILE_VELOCITY, 20),
        (ADDR_GOAL_POSITION, 3064),
        (ADDR_TORQUE_ENABLE, 1),
    ]
    assert bridge.torque_enabled_ids == {5}


def test_spur_enable_rejects_wrong_mode_without_any_write():
    packet = Packet(mode=4)
    bridge = _bridge(packet)

    with pytest.raises(RuntimeError, match='does not match required'):
        bridge.set_torque(5, True)

    assert packet.writes == []
    assert bridge.torque_enabled_ids == set()


def test_spur_enable_rejects_position_outside_calibrated_range():
    packet = Packet(position=4000)
    bridge = _bridge(packet)

    with pytest.raises(RuntimeError, match='outside safe range'):
        bridge.set_torque(5, True)

    assert packet.writes == []
