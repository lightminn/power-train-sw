"""ROS import 없이 브리지 기동 안전 경계를 고정하는 회귀 테스트."""

import ast
from pathlib import Path
import threading
from types import SimpleNamespace


SOURCE = (Path(__file__).parents[1]
          / 'dynamixel_control/moveit_dynamixel_bridge.py')

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
TORQUE_DISABLE = 0
TORQUE_ENABLE = 1


class Logger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)

    def info(self, _message):
        pass


def _method(name):
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    methods = [node for node in ast.walk(tree)
               if isinstance(node, ast.FunctionDef) and node.name == name]
    assert len(methods) == 1, f'{name} must have exactly one definition'
    namespace = {
        'ADDR_TORQUE_ENABLE': ADDR_TORQUE_ENABLE,
        'ADDR_OPERATING_MODE': ADDR_OPERATING_MODE,
        'ADDR_GOAL_POSITION': ADDR_GOAL_POSITION,
        'ADDR_PRESENT_POSITION': ADDR_PRESENT_POSITION,
        'TORQUE_DISABLE': TORQUE_DISABLE,
        'TORQUE_ENABLE': TORQUE_ENABLE,
    }
    exec(compile(ast.Module(body=methods, type_ignores=[]),
                 str(SOURCE), 'exec'), namespace)
    return namespace[name]


def _bridge(read_register):
    logger = Logger()
    writes = []
    profiles = []
    bridge = SimpleNamespace(
        _bus_lock=threading.Lock(),
        _read_register=read_register,
        _write_register=lambda _id, address, _size, value, label: writes.append(
            (address, value, label)),
        _write_motion_profile=lambda _id, _label, velocity=None: profiles.append(
            velocity),
        get_logger=lambda: logger,
    )
    return bridge, logger, writes, profiles


def test_torque_enable_synchronizes_present_goal_before_enable():
    registers = {
        ADDR_TORQUE_ENABLE: TORQUE_DISABLE,
        ADDR_OPERATING_MODE: 3,
        ADDR_PRESENT_POSITION: 1234,
        ADDR_GOAL_POSITION: 1234,
    }
    bridge, _logger, writes, profiles = _bridge(
        lambda _id, address, _size, _label, signed=False: registers[address])
    bridge._write_register = (
        lambda _id, address, _size, value, label:
        (writes.append((address, value, label)), registers.__setitem__(address, value)))

    assert _method('_enable_torque')(bridge, 5, 'spur', 3, 20)
    assert writes == [
        (ADDR_GOAL_POSITION, 1234, 'startup synchronize goal'),
        (ADDR_TORQUE_ENABLE, TORQUE_ENABLE, 'startup torque enable'),
    ]
    assert profiles == [20]


def test_torque_enable_fails_closed_on_goal_readback_mismatch():
    def read_register(_id, address, _size, _label, signed=False):
        return {
            ADDR_TORQUE_ENABLE: TORQUE_DISABLE,
            ADDR_PRESENT_POSITION: 1234,
            ADDR_GOAL_POSITION: 1235,
        }[address]

    bridge, logger, writes, profiles = _bridge(read_register)

    assert not _method('_enable_torque')(bridge, 14, 'arm_joint_2')
    assert profiles == []
    assert not any(address == ADDR_TORQUE_ENABLE and value == TORQUE_ENABLE
                   for address, value, _label in writes)
    assert logger.errors


def test_torque_enable_fails_closed_on_operating_mode_mismatch():
    def read_register(_id, address, _size, _label, signed=False):
        return {
            ADDR_TORQUE_ENABLE: TORQUE_DISABLE,
            ADDR_OPERATING_MODE: 4,
        }[address]

    bridge, logger, writes, profiles = _bridge(read_register)

    assert not _method('_enable_torque')(bridge, 5, 'spur', 3, 20)
    assert writes == []
    assert profiles == []
    assert 'operating mode mismatch' in logger.errors[0]
