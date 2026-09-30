#!/usr/bin/env python3
"""XL430 그리퍼 끝점, 파지 및 임시 동작을 안전하게 캘리브레이션한다."""

import argparse
import json
import statistics
import time
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, GroupSyncWrite, PacketHandler, PortHandler

DEVICE = "/dev/ttyUSB0"
BAUD_RATE = 1_000_000
PROTOCOL_VERSION = 2.0
DXL_IDS = (3, 4)
CALIBRATION_ID = 5
ARM_IDS = {12, 13, 14, 16}
EXPECTED_MODEL = 1060

ADDR_MODEL_NUMBER = 0
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_MOVING_STATUS = 123
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

TORQUE_OFF = 0
TORQUE_ON = 1
POSITION_MODE = 3
EXTENDED_POSITION_MODE = 4
SUPPORTED_POSITION_MODES = {POSITION_MODE, EXTENDED_POSITION_MODE}
ENDPOINT_FILE = Path("/tmp/gripper_endpoints.json")
RESULT_FILE = Path("/tmp/gripper_grasp_result.json")


class CalibrationError(RuntimeError):
    pass


def signed(value, bits):
    sign_bit = 1 << (bits - 1)
    return value - (1 << bits) if value & sign_bit else value


def le32(value):
    value &= 0xFFFFFFFF
    return [(value >> shift) & 0xFF for shift in (0, 8, 16, 24)]


def goal_for_mode(goal, operating_mode):
    """모드 3에서 래핑이 필요할 때만 연속 tick 목표를 변환한다."""
    return goal % 4096 if operating_mode == POSITION_MODE else goal


class Bus:
    def __init__(self):
        self.port = PortHandler(DEVICE)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.sync_goal = GroupSyncWrite(
            self.port, self.packet, ADDR_GOAL_POSITION, 4)
        self.operating_modes = {}

    def open(self):
        if not self.port.openPort():
            raise CalibrationError(f"cannot open {DEVICE}")
        if not self.port.setBaudRate(BAUD_RATE):
            self.port.closePort()
            raise CalibrationError(f"cannot set baud rate {BAUD_RATE}")

    def close(self):
        self.port.closePort()

    def _read(self, dxl_id, address, size, label):
        reader = {
            1: self.packet.read1ByteTxRx,
            2: self.packet.read2ByteTxRx,
            4: self.packet.read4ByteTxRx,
        }[size]
        value, result, error = reader(self.port, dxl_id, address)
        if result != COMM_SUCCESS:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getTxRxResult(result)}")
        if error:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getRxPacketError(error)}")
        return value

    def read1(self, dxl_id, address, label):
        return self._read(dxl_id, address, 1, label)

    def read2(self, dxl_id, address, label):
        return self._read(dxl_id, address, 2, label)

    def read4(self, dxl_id, address, label):
        return self._read(dxl_id, address, 4, label)

    def _write(self, dxl_id, address, size, value, label):
        writer = {
            1: self.packet.write1ByteTxRx,
            4: self.packet.write4ByteTxRx,
        }[size]
        result, error = writer(self.port, dxl_id, address, value)
        if result != COMM_SUCCESS:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getTxRxResult(result)}")
        if error:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getRxPacketError(error)}")

    def snapshot(self, dxl_id):
        return {
            "model": self.read2(dxl_id, ADDR_MODEL_NUMBER, "model"),
            "operating_mode": self.read1(
                dxl_id, ADDR_OPERATING_MODE, "operating mode"),
            "torque": self.read1(dxl_id, ADDR_TORQUE_ENABLE, "torque"),
            "hardware_error": self.read1(
                dxl_id, ADDR_HARDWARE_ERROR_STATUS, "hardware error"),
            "position": signed(self.read4(
                dxl_id, ADDR_PRESENT_POSITION, "position"), 32),
            "load": signed(self.read2(
                dxl_id, ADDR_PRESENT_LOAD, "present load"), 16),
        }

    def set_profile(self, dxl_id, acceleration, velocity):
        self._write(dxl_id, ADDR_PROFILE_ACCELERATION, 4,
                    acceleration, "profile acceleration")
        self._write(dxl_id, ADDR_PROFILE_VELOCITY, 4,
                    velocity, "profile velocity")

    def set_torque(self, dxl_id, enabled):
        self._write(dxl_id, ADDR_TORQUE_ENABLE, 1,
                    TORQUE_ON if enabled else TORQUE_OFF, "torque")

    def write_goals(self, goals):
        self.sync_goal.clearParam()
        for dxl_id in DXL_IDS:
            goal = goal_for_mode(
                goals[dxl_id], self.operating_modes.get(dxl_id))
            if not self.sync_goal.addParam(dxl_id, le32(goal)):
                raise CalibrationError(f"cannot queue goal for ID {dxl_id}")
        result = self.sync_goal.txPacket()
        self.sync_goal.clearParam()
        if result != COMM_SUCCESS:
            raise CalibrationError(
                f"sync goal write: {self.packet.getTxRxResult(result)}")

    def write_goal(self, dxl_id, goal):
        """2모터 경로를 거치지 않고 위치 목표 하나를 쓴다."""
        self._write(dxl_id, ADDR_GOAL_POSITION, 4, goal, "goal position")

    def motion_snapshot(self, dxl_id):
        """임시 저수준 동작 캘리브레이션에 사용할 피드백을 읽는다."""
        return {
            "position": signed(self.read4(
                dxl_id, ADDR_PRESENT_POSITION, "position"), 32),
            "velocity": signed(self.read4(
                dxl_id, ADDR_PRESENT_VELOCITY, "present velocity"), 32),
            "current": signed(self.read2(
                dxl_id, ADDR_PRESENT_LOAD, "present current"), 16),
            "moving_status": self.read1(
                dxl_id, ADDR_MOVING_STATUS, "moving status"),
            "hardware_error": self.read1(
                dxl_id, ADDR_HARDWARE_ERROR_STATUS, "hardware error"),
        }


def require_safe_read_state(bus, require_position_mode=False):
    snapshots = {dxl_id: bus.snapshot(dxl_id) for dxl_id in DXL_IDS}
    modes = {dxl_id: state["operating_mode"]
             for dxl_id, state in snapshots.items()}
    bus.operating_modes = modes
    print("Operating Mode: " + ", ".join(
        f"ID {dxl_id}={modes[dxl_id]}" for dxl_id in DXL_IDS))
    for dxl_id, state in snapshots.items():
        if state["model"] != EXPECTED_MODEL:
            raise CalibrationError(
                f"ID {dxl_id}: model {state['model']} != {EXPECTED_MODEL}")
        if state["hardware_error"] != 0:
            raise CalibrationError(
                f"ID {dxl_id}: hardware error 0x{state['hardware_error']:02X}")
        if state["torque"] != TORQUE_OFF:
            raise CalibrationError(
                f"ID {dxl_id}: torque is enabled; disable it before calibration")
        if (require_position_mode and
                state["operating_mode"] not in SUPPORTED_POSITION_MODES):
            raise CalibrationError(
                f"ID {dxl_id}: operating mode {state['operating_mode']} is "
                "not Position (3) or Extended Position (4)")
    if require_position_mode and len(set(modes.values())) != 1:
        raise CalibrationError(
            "ID 3 and ID 4 must use the same operating mode; no EEPROM "
            "register was changed")
    return snapshots


def capture_endpoint(bus, label, sample_count):
    input(f"Move the torque-free gripper fully {label}, then press Enter: ")
    positions = {dxl_id: [] for dxl_id in DXL_IDS}
    for _ in range(sample_count):
        states = require_safe_read_state(bus, require_position_mode=True)
        for dxl_id in DXL_IDS:
            positions[dxl_id].append(states[dxl_id]["position"])
        time.sleep(0.1)
    result = {
        dxl_id: int(statistics.median(values))
        for dxl_id, values in positions.items()
    }
    print(f"{label}: ID3={result[3]}, ID4={result[4]}")
    return result


def stage_endpoints(args):
    if args.samples < 1:
        raise CalibrationError("endpoint sample count must be positive")
    if args.min_span_ticks < 1:
        raise CalibrationError("minimum endpoint span must be positive")
    bus = Bus()
    bus.open()
    try:
        require_safe_read_state(bus, require_position_mode=True)
        opened = capture_endpoint(bus, "open", args.samples)
        closed = capture_endpoint(bus, "closed", args.samples)
        for dxl_id in DXL_IDS:
            if abs(closed[dxl_id] - opened[dxl_id]) < args.min_span_ticks:
                raise CalibrationError(
                    f"ID {dxl_id}: endpoint span is too small")
        data = {
            "device": DEVICE,
            "baud_rate": BAUD_RATE,
            "protocol": PROTOCOL_VERSION,
            "model": EXPECTED_MODEL,
            "operating_modes": {
                str(dxl_id): bus.operating_modes[dxl_id]
                for dxl_id in DXL_IDS
            },
            "id3_open_tick": opened[3],
            "id3_close_tick": closed[3],
            "id4_open_tick": opened[4],
            "id4_close_tick": closed[4],
            "approved": False,
        }
        args.output.write_text(json.dumps(data, indent=2) + "\n")
        print(json.dumps(data, indent=2))
        print(f"Saved unapproved endpoints to {args.output}")
    finally:
        bus.close()


def stage_configure_profile(args):
    """휘발성 모션 프로파일 레지스터를 명시적으로 설정하고 검증한다."""
    if not 1 <= args.profile_acceleration <= 32767:
        raise CalibrationError("profile acceleration must be in [1, 32767]")
    if not 1 <= args.profile_velocity <= 32767:
        raise CalibrationError("profile velocity must be in [1, 32767]")
    bus = Bus()
    bus.open()
    try:
        require_safe_read_state(bus, require_position_mode=True)
        for dxl_id in DXL_IDS:
            bus.set_profile(
                dxl_id, args.profile_acceleration, args.profile_velocity)
        verified = {}
        for dxl_id in DXL_IDS:
            verified[dxl_id] = {
                "profile_acceleration": bus.read4(
                    dxl_id, ADDR_PROFILE_ACCELERATION,
                    "profile acceleration"),
                "profile_velocity": bus.read4(
                    dxl_id, ADDR_PROFILE_VELOCITY, "profile velocity"),
            }
        print("RAM profile verification:")
        for dxl_id in DXL_IDS:
            values = verified[dxl_id]
            print(
                f"ID {dxl_id}: acceleration="
                f"{values['profile_acceleration']}, "
                f"velocity={values['profile_velocity']}")
            if values != {
                    "profile_acceleration": args.profile_acceleration,
                    "profile_velocity": args.profile_velocity}:
                raise CalibrationError(
                    f"ID {dxl_id}: RAM profile verification failed")
        print("Only volatile Profile Acceleration/Velocity were written; "
              "EEPROM was not changed.")
    finally:
        bus.close()


def require_single_motor_safe_state(bus, dxl_id):
    """식별된 ID 5 XL430이 위치 동작에 안전한지 확인한다."""
    if dxl_id in ARM_IDS:
        raise CalibrationError(f"ID {dxl_id} is an arm motor and is forbidden")
    if dxl_id != CALIBRATION_ID:
        raise CalibrationError(
            f"temporary motion calibration permits only ID {CALIBRATION_ID}")
    state = bus.snapshot(dxl_id)
    if state["model"] != EXPECTED_MODEL:
        raise CalibrationError(
            f"ID {dxl_id}: model {state['model']} != {EXPECTED_MODEL}")
    if state["operating_mode"] != POSITION_MODE:
        raise CalibrationError(
            f"ID {dxl_id}: Operating Mode must be 3, got "
            f"{state['operating_mode']}")
    if state["torque"] != TORQUE_OFF:
        raise CalibrationError(
            f"ID {dxl_id}: Torque must be OFF before calibration")
    if state["hardware_error"] != 0:
        raise CalibrationError(
            f"ID {dxl_id}: hardware error 0x{state['hardware_error']:02X}")
    return state


def validate_motion_args(args):
    if args.id in ARM_IDS:
        raise CalibrationError(f"ID {args.id} is an arm motor and is forbidden")
    if args.id != CALIBRATION_ID:
        raise CalibrationError(
            f"temporary motion calibration permits only ID {CALIBRATION_ID}")
    if not 1 <= args.profile_acceleration <= 25:
        raise CalibrationError("profile acceleration must be in [1, 25]")
    if not 1 <= args.profile_velocity <= 80:
        raise CalibrationError("profile velocity must be in [1, 80]")
    if args.max_abs_current < 1:
        raise CalibrationError("current limit must be positive")
    if args.stall_timeout <= 0 or args.timeout <= 0 or args.sample_hz <= 0:
        raise CalibrationError("timeouts and sample rate must be positive")
    if args.goal_tolerance < 0:
        raise CalibrationError("goal tolerance must be non-negative")


def motion_goal(args, present_position):
    """현재 단일 회전 위치를 사용해 모드 3 목표를 계산한다."""
    present_modulo = present_position % 4096
    if args.stage == "move-relative":
        goal = present_modulo + args.delta_ticks
    else:
        goal = args.goal_ticks
    if not 0 <= goal <= 4095:
        raise CalibrationError(
            f"goal {goal} is outside Position Mode limits [0, 4095]")
    return present_modulo, goal


def stage_motion(args):
    """명시적으로 승인된 저수준 ID 5 캘리브레이션 이동을 한 번 실행한다."""
    validate_motion_args(args)
    bus = Bus()
    bus.open()
    execute = bool(args.execute)
    try:
        initial = require_single_motor_safe_state(bus, args.id)
        present_modulo, goal = motion_goal(args, initial["position"])
        print(
            f"ID {args.id}: present_signed={initial['position']} "
            f"present_modulo={present_modulo} goal={goal} "
            f"delta={goal - present_modulo}")
        if not execute:
            print("DRY RUN: no profile, Goal Position, or Torque register was written")
            return

        bus.write_goal(args.id, present_modulo)
        bus.set_profile(
            args.id, args.profile_acceleration, args.profile_velocity)
        profile_acceleration = bus.read4(
            args.id, ADDR_PROFILE_ACCELERATION, "profile acceleration")
        profile_velocity = bus.read4(
            args.id, ADDR_PROFILE_VELOCITY, "profile velocity")
        if (profile_acceleration != args.profile_acceleration or
                profile_velocity != args.profile_velocity):
            raise CalibrationError("profile readback verification failed")
        print(
            f"Profile readback: acceleration={profile_acceleration}, "
            f"velocity={profile_velocity}")

        bus.set_torque(args.id, True)
        bus.write_goal(args.id, goal)
        deadline = time.monotonic() + args.timeout
        last_progress_time = time.monotonic()
        last_progress_position = present_modulo
        max_abs_current = 0
        final = None
        while time.monotonic() < deadline:
            state = bus.motion_snapshot(args.id)
            position_modulo = state["position"] % 4096
            max_abs_current = max(
                max_abs_current, abs(state["current"]))
            print(
                f"goal={goal} present_signed={state['position']} "
                f"present_modulo={position_modulo} "
                f"velocity={state['velocity']} current={state['current']} "
                f"moving_status={state['moving_status']} "
                f"hardware_error={state['hardware_error']}")
            if state["hardware_error"]:
                raise CalibrationError(
                    f"hardware error 0x{state['hardware_error']:02X}")
            if abs(state["current"]) >= args.max_abs_current:
                raise CalibrationError(
                    f"abs(current) {abs(state['current'])} reached limit "
                    f"{args.max_abs_current}")
            if abs(position_modulo - last_progress_position) >= 2:
                last_progress_position = position_modulo
                last_progress_time = time.monotonic()
            elif (time.monotonic() - last_progress_time
                  >= args.stall_timeout and
                  abs(position_modulo - goal) > args.goal_tolerance):
                raise CalibrationError(
                    f"position stalled for {args.stall_timeout:.1f}s")
            final = state
            if (abs(position_modulo - goal) <= args.goal_tolerance and
                    state["velocity"] == 0):
                print(
                    f"Reached goal: actual_delta="
                    f"{position_modulo - present_modulo}, "
                    f"max_abs_current={max_abs_current}")
                return
            time.sleep(1.0 / args.sample_hz)
        final_position = None if final is None else final["position"] % 4096
        raise CalibrationError(
            f"motion timeout after {args.timeout:.1f}s; "
            f"last_position={final_position}")
    finally:
        if execute:
            try:
                bus.set_torque(args.id, False)
                torque = bus.read1(
                    args.id, ADDR_TORQUE_ENABLE, "torque readback")
                print(f"Final Torque Enable readback: {torque}")
                if torque != TORQUE_OFF:
                    raise CalibrationError(
                        f"Torque OFF readback verification failed: {torque}")
            except Exception as exc:
                print(f"ERROR: failed to disable/read back torque: {exc}")
                raise CalibrationError(
                    "final Torque OFF could not be verified") from exc
        bus.close()
        print("Serial port closed")


def load_endpoints(path):
    data = json.loads(path.read_text())
    required = (
        "id3_open_tick", "id3_close_tick",
        "id4_open_tick", "id4_close_tick",
    )
    if any(key not in data for key in required):
        raise CalibrationError("endpoint file is incomplete")
    return data


def interpolate(data, ratio):
    return {
        3: round(data["id3_open_tick"] + ratio * (
            data["id3_close_tick"] - data["id3_open_tick"])),
        4: round(data["id4_open_tick"] + ratio * (
            data["id4_close_tick"] - data["id4_open_tick"])),
    }


def observed_ratio(position, opened, closed):
    return (position - opened) / (closed - opened)


def check_live_safety(states, endpoints, args, previous_positions=None):
    for dxl_id, state in states.items():
        if state["hardware_error"]:
            raise CalibrationError(
                f"ID {dxl_id}: hardware error 0x{state['hardware_error']:02X}")
        if abs(state["load"]) >= args.max_abs_load:
            raise CalibrationError(
                f"ID {dxl_id}: abs(load) {abs(state['load'])} reached safety limit")
    ratios = {
        3: observed_ratio(states[3]["position"], endpoints["id3_open_tick"],
                          endpoints["id3_close_tick"]),
        4: observed_ratio(states[4]["position"], endpoints["id4_open_tick"],
                          endpoints["id4_close_tick"]),
    }
    if abs(ratios[3] - ratios[4]) > args.max_ratio_deviation:
        raise CalibrationError(
            f"coupling mismatch: ID3 ratio={ratios[3]:.3f}, ID4 ratio={ratios[4]:.3f}")
    movement = None
    if previous_positions is not None:
        movement = max(abs(states[i]["position"] - previous_positions[i])
                       for i in DXL_IDS)
    return ratios, movement


def stage_grasp(args):
    if not args.approve_endpoints:
        raise CalibrationError("Stage 2 requires --approve-endpoints")
    if not 0.0 < args.ratio_step <= 0.01:
        raise CalibrationError("ratio step must be in (0, 0.01]")
    if not 0.0 < args.max_close_ratio < 1.0:
        raise CalibrationError("max close ratio must be in (0, 1)")
    if not 0.0 <= args.extra_close_ratio <= 0.02:
        raise CalibrationError("extra close ratio must be in [0, 0.02]")
    if not 0 < args.contact_load < args.max_abs_load:
        raise CalibrationError("contact load must be below the absolute load limit")
    if not 1 <= args.profile_acceleration <= 25:
        raise CalibrationError("profile acceleration must be in [1, 25]")
    if not 1 <= args.profile_velocity <= 80:
        raise CalibrationError("profile velocity must be in [1, 80]")
    endpoints = load_endpoints(args.endpoints)
    print(json.dumps(endpoints, indent=2))
    confirmation = input(
        "Type APPROVE POWERED GRASP to confirm these endpoints: ")
    if confirmation != "APPROVE POWERED GRASP":
        raise CalibrationError("powered calibration not approved")

    bus = Bus()
    profiles = {}
    torque_attempted = False
    max_load = {3: 0, 4: 0}
    result = {"contact": False}
    bus.open()
    try:
        initial = require_safe_read_state(bus, require_position_mode=True)
        recorded_modes = endpoints.get("operating_modes")
        if recorded_modes is not None:
            expected_modes = {
                dxl_id: int(recorded_modes[str(dxl_id)])
                for dxl_id in DXL_IDS
            }
            if bus.operating_modes != expected_modes:
                raise CalibrationError(
                    f"live operating modes {bus.operating_modes} differ "
                    f"from endpoint capture modes {expected_modes}")
        for dxl_id in DXL_IDS:
            open_tick = endpoints[f"id{dxl_id}_open_tick"]
            if abs(initial[dxl_id]["position"] - open_tick) > args.open_tolerance:
                raise CalibrationError(
                    f"ID {dxl_id}: manually place at open endpoint before Stage 2")
            profiles[dxl_id] = {
                "acceleration": bus.read4(
                    dxl_id, ADDR_PROFILE_ACCELERATION, "profile acceleration"),
                "velocity": bus.read4(
                    dxl_id, ADDR_PROFILE_VELOCITY, "profile velocity"),
            }
            bus.set_profile(dxl_id, args.profile_acceleration,
                            args.profile_velocity)

        for dxl_id in DXL_IDS:
            bus.set_torque(dxl_id, True)
            torque_attempted = True
        bus.write_goals(interpolate(endpoints, 0.0))
        time.sleep(args.initial_settle_seconds)

        contact_count = 0
        previous_positions = None
        ratio = 0.0
        while ratio < args.max_close_ratio:
            ratio = min(args.max_close_ratio, ratio + args.ratio_step)
            goals = interpolate(endpoints, ratio)
            bus.write_goals(goals)
            deadline = time.monotonic() + args.step_timeout
            while time.monotonic() < deadline:
                states = {i: bus.snapshot(i) for i in DXL_IDS}
                _, movement = check_live_safety(
                    states, endpoints, args, previous_positions)
                previous_positions = {
                    i: states[i]["position"] for i in DXL_IDS}
                for i in DXL_IDS:
                    max_load[i] = max(max_load[i], abs(states[i]["load"]))
                load_contact = max(abs(states[i]["load"]) for i in DXL_IDS) \
                    >= args.contact_load
                still = movement is not None and movement <= args.max_contact_movement
                contact_count = contact_count + 1 if load_contact and still else 0
                print(
                    f"ratio={ratio:.3f} goals={goals} "
                    f"pos={{3:{states[3]['position']},4:{states[4]['position']}}} "
                    f"load={{3:{states[3]['load']},4:{states[4]['load']}}}")
                if contact_count >= args.contact_samples:
                    contact_ratio = ratio
                    contact_states = states
                    extra_ratio = min(
                        args.max_close_ratio, ratio + args.extra_close_ratio)
                    bus.write_goals(interpolate(endpoints, extra_ratio))
                    time.sleep(args.extra_settle_seconds)
                    final = {i: bus.snapshot(i) for i in DXL_IDS}
                    check_live_safety(final, endpoints, args)
                    result = {
                        "contact": True,
                        "contact_ratio": contact_ratio,
                        "final_ratio": extra_ratio,
                        "contact_before_fully_closed": contact_ratio < 1.0,
                        "id3_contact_tick": contact_states[3]["position"],
                        "id4_contact_tick": contact_states[4]["position"],
                        "id3_present_load": contact_states[3]["load"],
                        "id4_present_load": contact_states[4]["load"],
                        "id3_final_tick": final[3]["position"],
                        "id4_final_tick": final[4]["position"],
                        "maximum_observed_load": max(max_load.values()),
                        "maximum_observed_load_by_id": max_load,
                    }
                    args.output.write_text(json.dumps(result, indent=2) + "\n")
                    print(json.dumps(result, indent=2))
                    return
                if all(abs(states[i]["position"] - goals[i]) <= args.goal_tolerance
                       for i in DXL_IDS):
                    break
                time.sleep(1.0 / args.sample_hz)
        result.update({
            "maximum_observed_load": max(max_load.values()),
            "maximum_observed_load_by_id": max_load,
        })
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(
            "No contact detected before the configured maximum close ratio; "
            "the calibrated mechanical endpoint was not commanded.")
    finally:
        if torque_attempted:
            for dxl_id in DXL_IDS:
                try:
                    bus.set_torque(dxl_id, False)
                except Exception as exc:
                    print(f"WARNING: failed to disable torque on ID {dxl_id}: {exc}")
        for dxl_id, profile in profiles.items():
            try:
                bus.set_profile(dxl_id, profile["acceleration"], profile["velocity"])
            except Exception as exc:
                print(f"WARNING: failed to restore profile on ID {dxl_id}: {exc}")
        bus.close()
        print("Torque-disable attempted for both motors; serial port closed.")


def parser():
    root = argparse.ArgumentParser()
    sub = root.add_subparsers(dest="stage", required=True)
    endpoints = sub.add_parser("endpoints", help="read-only endpoint capture")
    endpoints.add_argument("--samples", type=int, default=7)
    endpoints.add_argument("--min-span-ticks", type=int, default=50)
    endpoints.add_argument("--output", type=Path, default=ENDPOINT_FILE)
    endpoints.set_defaults(func=stage_endpoints)

    profile = sub.add_parser(
        "configure-profile",
        help="explicitly write volatile profile registers with torque off")
    profile.add_argument("--profile-acceleration", type=int, default=25)
    profile.add_argument("--profile-velocity", type=int, default=80)
    profile.set_defaults(func=stage_configure_profile)

    grasp = sub.add_parser("grasp", help="approved powered gradual grasp")
    grasp.add_argument("--endpoints", type=Path, default=ENDPOINT_FILE)
    grasp.add_argument("--output", type=Path, default=RESULT_FILE)
    grasp.add_argument("--approve-endpoints", action="store_true")
    grasp.add_argument("--ratio-step", type=float, default=0.005)
    grasp.add_argument("--profile-velocity", type=int, default=20)
    grasp.add_argument("--profile-acceleration", type=int, default=5)
    grasp.add_argument("--max-abs-load", type=int, default=300)
    grasp.add_argument("--contact-load", type=int, default=80)
    grasp.add_argument("--contact-samples", type=int, default=3)
    grasp.add_argument("--max-contact-movement", type=int, default=3)
    grasp.add_argument("--max-ratio-deviation", type=float, default=0.05)
    grasp.add_argument("--goal-tolerance", type=int, default=30)
    grasp.add_argument("--open-tolerance", type=int, default=30)
    grasp.add_argument("--extra-close-ratio", type=float, default=0.01)
    grasp.add_argument("--max-close-ratio", type=float, default=0.95)
    grasp.add_argument("--sample-hz", type=float, default=10.0)
    grasp.add_argument("--step-timeout", type=float, default=1.5)
    grasp.add_argument("--initial-settle-seconds", type=float, default=1.0)
    grasp.add_argument("--extra-settle-seconds", type=float, default=0.5)
    grasp.set_defaults(func=stage_grasp)

    def add_motion_arguments(command):
        command.add_argument("--id", type=int, default=CALIBRATION_ID)
        command.add_argument("--execute", action="store_true")
        command.add_argument("--profile-acceleration", type=int, default=5)
        command.add_argument("--profile-velocity", type=int, default=20)
        command.add_argument("--max-abs-current", type=int, default=100)
        command.add_argument("--stall-timeout", type=float, default=2.0)
        command.add_argument("--timeout", type=float, default=10.0)
        command.add_argument("--sample-hz", type=float, default=10.0)
        command.add_argument("--goal-tolerance", type=int, default=2)
        command.set_defaults(func=stage_motion)

    relative = sub.add_parser(
        "move-relative", help="temporary guarded ID 5 relative tick motion")
    relative.add_argument("--delta-ticks", type=int, required=True)
    add_motion_arguments(relative)

    absolute = sub.add_parser(
        "move-absolute", help="temporary guarded ID 5 absolute tick motion")
    absolute.add_argument("--goal-ticks", type=int, required=True)
    add_motion_arguments(absolute)
    return root


def main():
    args = parser().parse_args()
    try:
        args.func(args)
    except (CalibrationError, KeyboardInterrupt) as exc:
        print(f"STOPPED: {exc}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
