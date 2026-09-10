#!/usr/bin/env python3
"""Guarded calibration workflow for the one-motor spur-gear gripper."""

import argparse
from pathlib import Path
import statistics
import sys
import time

import yaml
from dynamixel_sdk import PortHandler, PacketHandler

from dynamixel_control.tool_profiles import validate_profile


ADDR_OPERATING_MODE = 11
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146
ADDR_GOAL_POSITION = 116
PROTOCOL_VERSION = 2.0


class CalibrationError(RuntimeError):
    pass


def signed16(value):
    return value - 65536 if value >= 32768 else value


def build_profile(actuator_id, open_tick, close_tick, safe_margin,
                  profile_velocity, profile_acceleration, samples):
    """Build a strict profile from captured endpoints and load samples."""
    no_load = int(round(statistics.median(samples['no_load'])))
    grasp = int(round(statistics.median(samples['grasp'])))
    released = int(round(statistics.median(samples['release'])))
    if grasp <= no_load:
        raise CalibrationError('grasp load must be greater than no-load')
    grasp_threshold = int(round((no_load + grasp) / 2.0))
    drop_threshold = max(no_load + 1, int(round((released + grasp) / 2.0)))
    low = min(open_tick, close_tick)
    high = max(open_tick, close_tick)
    if high - low <= 2 * safe_margin:
        raise CalibrationError('mechanical stroke is smaller than safety margins')
    if close_tick > open_tick:
        open_command, close_command = open_tick + safe_margin, close_tick - safe_margin
    else:
        open_command, close_command = open_tick - safe_margin, close_tick + safe_margin
    profile = {
        'backend': 'gripper', 'calibrated': True,
        'actuator_ids': [actuator_id],
        'joint_names': ['gripper_drive_joint'],
        'direction': 1 if close_tick > open_tick else -1,
        'open_tick': open_command, 'close_tick': close_command,
        'safe_min_tick': low, 'safe_max_tick': high,
        'profile_velocity': profile_velocity,
        'profile_acceleration': profile_acceleration,
        'no_load_effort': no_load, 'grasp_effort': grasp,
        'grasp_threshold': grasp_threshold,
        'release_drop_threshold': drop_threshold,
        'max_abs_effort': int(round(grasp * 1.5)),
        'action_time': 2.0,
        'required_operating_modes': {actuator_id: 3},
        'calibration_samples': samples,
    }
    errors = validate_profile('spur_1motor_gripper', profile)
    if errors:
        raise CalibrationError('; '.join(errors))
    return profile


class Bus:
    def __init__(self, device, baudrate, actuator_id):
        self.port = PortHandler(device)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.baudrate = baudrate
        self.actuator_id = actuator_id

    def open(self):
        if not self.port.openPort() or not self.port.setBaudRate(self.baudrate):
            raise CalibrationError('cannot open/configure Dynamixel port')
        _model, result, error = self.packet.ping(self.port, self.actuator_id)
        if result != 0 or error != 0:
            raise CalibrationError(f'actuator ID {self.actuator_id} not discovered')

    def read(self, address, size):
        method = {1: self.packet.read1ByteTxRx,
                  2: self.packet.read2ByteTxRx,
                  4: self.packet.read4ByteTxRx}[size]
        value, result, error = method(self.port, self.actuator_id, address)
        if result != 0 or error != 0:
            raise CalibrationError(
                f'read failed id={self.actuator_id} address={address}')
        return value

    def snapshot(self):
        return {
            'position': self.read(ADDR_PRESENT_POSITION, 4),
            'load': abs(signed16(self.read(ADDR_PRESENT_LOAD, 2))),
            'hardware_error': self.read(ADDR_HARDWARE_ERROR_STATUS, 1),
            'torque': self.read(ADDR_TORQUE_ENABLE, 1),
            'operating_mode': self.read(ADDR_OPERATING_MODE, 1),
            'input_voltage_raw': self.read(ADDR_PRESENT_INPUT_VOLTAGE, 2),
            'temperature_c': self.read(ADDR_PRESENT_TEMPERATURE, 1),
        }

    def disable(self):
        self.packet.write1ByteTxRx(
            self.port, self.actuator_id, ADDR_TORQUE_ENABLE, 0)

    def configure_position_mode(self, velocity, acceleration):
        self.disable()
        writes = (
            (ADDR_OPERATING_MODE, 1, 3),
            (ADDR_PROFILE_ACCELERATION, 4, acceleration),
            (ADDR_PROFILE_VELOCITY, 4, velocity),
        )
        for address, size, value in writes:
            method = {1: self.packet.write1ByteTxRx,
                      4: self.packet.write4ByteTxRx}[size]
            result, error = method(
                self.port, self.actuator_id, address, int(value))
            if result != 0 or error != 0:
                raise CalibrationError(f'configuration write failed at {address}')

    def move(self, tick):
        result, error = self.packet.write4ByteTxRx(
            self.port, self.actuator_id, ADDR_GOAL_POSITION, tick & 0xffffffff)
        if result != 0 or error != 0:
            raise CalibrationError('goal position write failed')
        result, error = self.packet.write1ByteTxRx(
            self.port, self.actuator_id, ADDR_TORQUE_ENABLE, 1)
        if result != 0 or error != 0:
            raise CalibrationError('torque enable failed')

    def close(self, disable_torque=False):
        if disable_torque:
            self.disable()
        self.port.closePort()


def capture_load(bus, label, count):
    input(f'Arrange {label}, then press Enter to sample: ')
    values = []
    for _ in range(count):
        snap = bus.snapshot()
        if snap['hardware_error']:
            raise CalibrationError(
                f'hardware error 0x{snap["hardware_error"]:02x}')
        values.append(snap['load'])
        time.sleep(0.05)
    print(f'{label}: {values}')
    return values


def run(args):
    bus = Bus(args.device, args.baudrate, args.actuator_id)
    try:
        bus.open()
        initial = bus.snapshot()
        print(f'diagnostic: id={args.actuator_id} {initial}')
        if args.read_only:
            if initial['torque'] != 0:
                raise CalibrationError(
                    'read-only check found torque enabled; do not proceed')
            if initial['hardware_error'] != 0:
                raise CalibrationError(
                    f'hardware error 0x{initial["hardware_error"]:02x}')
            return 0
        if not args.armed:
            raise CalibrationError(
                'powered load calibration requires explicit --armed approval')
        bus.disable()
        input('Torque is OFF. Move fully OPEN and press Enter: ')
        open_tick = bus.snapshot()['position']
        input('Move fully CLOSED and press Enter: ')
        close_tick = bus.snapshot()['position']
        if open_tick == close_tick:
            raise CalibrationError('open and close ticks are identical')
        bus.configure_position_mode(
            args.profile_velocity, args.profile_acceleration)
        input('Clear the gripper. Press Enter to power it toward OPEN: ')
        command_open = open_tick + args.safe_margin \
            if close_tick > open_tick else open_tick - args.safe_margin
        command_close = close_tick - args.safe_margin \
            if close_tick > open_tick else close_tick + args.safe_margin
        bus.move(command_open)
        time.sleep(args.settle_time)
        samples = {'no_load': capture_load(
            bus, 'NO-LOAD open steady state', args.samples)}
        input('Insert the test object and keep clear. Press Enter to CLOSE: ')
        bus.move(command_close)
        time.sleep(args.settle_time)
        samples['grasp'] = capture_load(
            bus, 'secure GRASP steady state', args.samples)
        input('Support the object. Press Enter to OPEN/RELEASE: ')
        bus.move(command_open)
        time.sleep(args.settle_time)
        samples['release'] = capture_load(
            bus, 'RELEASE/DROP steady state', args.samples)
        profile = build_profile(
            args.actuator_id, open_tick, close_tick, args.safe_margin,
            args.profile_velocity, args.profile_acceleration, samples)
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('w', encoding='utf-8') as stream:
            yaml.safe_dump({'tool_profiles': {'spur_1motor_gripper': profile}},
                           stream, sort_keys=False)
        print(f'validated profile written to {output}')
        return 0
    finally:
        try:
            bus.close(disable_torque=not args.read_only)
        except Exception as exc:
            print(f'WARNING: final torque-off failed: {exc}', file=sys.stderr)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=1000000)
    parser.add_argument('--actuator-id', type=int, required=True)
    parser.add_argument('--profile-velocity', type=int, default=20)
    parser.add_argument('--profile-acceleration', type=int, default=5)
    parser.add_argument('--safe-margin', type=int, default=10)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--settle-time', type=float, default=2.0)
    parser.add_argument('--output', default='/tmp/spur_1motor_gripper.yaml')
    parser.add_argument('--read-only', action='store_true')
    parser.add_argument('--armed', action='store_true',
                        help='explicitly approve powered endpoint motion')
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (CalibrationError, KeyboardInterrupt) as exc:
        print(f'CALIBRATION ABORTED: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
