"""Bench tool change inside the bridge: slot teaching, vertical moves, coupling.

Driven by ``/arm/local_tool_change`` JSON commands (CAPTURE / START / STOP) in
MANUAL FULL_ROBOT bench mode.  The bridge stays the single bus owner; the step
order lives in bench_tool_change.BenchToolChange.
"""

import json
import math
import time

from dynamixel_control import calib_math
from dynamixel_control.bench_tool_change import (
    BenchToolChange, ToolSlotStore, plan, slot_complete)

# Same window the coupling lock uses to call the arm stationary at its goal.
ARRIVAL_TICKS = 12
# One Profile Velocity unit (0.229 rev/min) in ticks per second.
TICKS_PER_PROFILE_UNIT_S = 0.229 * 4096 / 60


class LocalToolChangeBridge:

    def _init_local_tool_change(self):
        from std_msgs.msg import String
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        self.declare_parameter('local_tool_slots_file', '')
        self.declare_parameter('local_tool_change_rise_m', 0.05)
        self.declare_parameter('local_tool_change_step_m', 0.005)
        self.declare_parameter('local_tool_change_speed_mps', 0.01)
        self._tc_configure(
            str(self.get_parameter('local_tool_slots_file').value), list(JOINT_CONFIG),
            float(self.get_parameter('local_tool_change_rise_m').value),
            float(self.get_parameter('local_tool_change_step_m').value),
            float(self.get_parameter('local_tool_change_speed_mps').value))
        self.create_subscription(String, '/arm/local_tool_change', self._tc_command, 10,
                                 callback_group=self._command_group)
        self.create_timer(0.05, self._tc_tick, callback_group=self._command_group)

    def _tc_configure(self, slots_file, joint_names, rise_m, step_m, speed_mps,
                      clock=time.monotonic):
        """ROS-free state setup, shared by the node and the tests."""
        if not (0 < step_m <= rise_m and speed_mps > 0):
            raise ValueError('local tool change rise/step/speed must be positive')
        self._tc_rise_m, self._tc_step_m = rise_m, step_m
        self._tc_segment_s = step_m / speed_mps
        self._tc_clock = clock
        self._tc_store = ToolSlotStore(slots_file, joint_names)
        try:
            self._tc_store.load()
        except Exception as exc:
            self.get_logger().error(f'tool slot file load failed: {exc}')
        self._tc_stream = None
        self._tc_goals = {}
        self._tc_lock_desired = None
        self._tc_error = ''
        self._tc_was_running = False
        self._tool_change = BenchToolChange(
            move=self._tc_move, arrived=self._tc_arrived, lock=self._tc_lock,
            lock_state=self._tc_lock_state, attached_tool=self._tc_attached_tool,
            hold=self._tc_hold, stop_lock=self._stop_toolchanger,
            permission=self._tc_permission, clock=clock)

    def _tc_running(self):
        change = getattr(self, '_tool_change', None)
        return change is not None and change.running

    def _tc_snapshot(self):
        change = self._tool_change
        return dict(state=change.state, slot=change.slot,
                    step=max(change.index, 0), steps=len(change.steps),
                    error=change.error or self._tc_error,
                    slots={key: slot['tool_type']
                           for key, slot in self._tc_store.data['slots'].items()},
                    incomplete=sorted(key for key, slot in self._tc_store.data['slots'].items()
                                      if not slot_complete(slot)),
                    attached_tool=self._tc_attached_tool())

    # --- commands ----------------------------------------------------------
    def _tc_command(self, msg):
        with self._bus_lock:
            try:
                request = json.loads(msg.data)
                command = request.get('command')
                if command == 'STOP':
                    if self._tc_running():
                        self._tool_change.abort('operator stop')
                    return
                self._tc_permission()
                if self._tc_running():
                    raise RuntimeError('a tool change is already running')
                # Same bench switches as the keyboard jog: absent means ON.
                limits = request.get('limits') is not False
                if command == 'CAPTURE':
                    seat = self._tc_stationary_pose()
                    self._tc_store.capture(str(request.get('slot', '')),
                                           request.get('tool_type'), seat,
                                           request.get('pose', 'seat'), limits=limits)
                elif command == 'START':
                    self._tc_start(request.get('tool_type'), request.get('dry_run') is True,
                                   limits=limits, trip=request.get('trip') is not False)
                else:
                    raise ValueError(f'unknown tool change command {command!r}')
                self._tc_error = ''
                self.get_logger().info(f'local tool change {command} accepted')
            except Exception as exc:
                self._tc_error = str(exc)
                self.get_logger().warn(f'local tool change rejected: {exc}')

    def _tc_start(self, tool_type, dry_run=False, limits=True, trip=True):
        if not dry_run and getattr(self, 'toolchanger', None) is None:
            raise RuntimeError('coupling lock is not configured')
        if self._pose_move is not None or self._wrist_jog.deadline is not None:
            raise RuntimeError('wait for the current arm motion to finish')
        home = self._local_poses.get('home')
        if not home:
            raise RuntimeError("save a pose named 'home' first")
        self._tc_stationary_pose()
        steps = plan(self._tc_store.data, self._tc_attached_tool(), tool_type, home,
                     rise_m=self._tc_rise_m, step_m=self._tc_step_m, dry_run=dry_run,
                     limits=limits)
        self._tc_trip = trip
        self._tool_change.start(steps)
        self._tc_was_running = True

    def _tc_permission(self):
        if (self.read_only or self.mock_mode or self.emergency_stop_active
                or self.control_mode != 'MANUAL' or self.control_scope != 'FULL_ROBOT'
                or not self.manual_coupling_lock):
            raise RuntimeError('tool change needs writable MANUAL FULL_ROBOT without E-stop')

    def _tc_stationary_pose(self):
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        pose = {}
        for name, config in JOINT_CONFIG.items():
            if config['id'] not in self.active_ids:
                raise RuntimeError(f'{name} is not connected; all five arm joints are required')
            if self._read_register(config['id'], 128, 4, 'tool change velocity', signed=True):
                raise RuntimeError('arm must be stationary')
            pose[name] = self.tick_to_rad(name, self._read_register(
                config['id'], 132, 4, 'tool change position', signed=True))
        return pose

    # --- runner callbacks ----------------------------------------------------
    def _tc_goal_tick(self, joint, rad):
        from dynamixel_control.moveit_dynamixel_bridge import (
            DXL_EXTENDED_MAX_TICK, DXL_EXTENDED_MIN_TICK, DXL_MAXIMUM_POSITION_VALUE,
            DXL_MINIMUM_POSITION_VALUE, JOINT_CONFIG)
        config = JOINT_CONFIG[joint]
        tick = int(round(calib_math.rad_to_tick(
            self._joint_center(joint), config['direction'],
            self._joint_gear_ratio(joint), rad)))
        if config['extended']:
            return max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def _tc_targets(self, pose):
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        targets = {}
        for joint, rad in pose.items():
            config = JOINT_CONFIG[joint]
            if config['id'] not in self.active_ids:
                raise RuntimeError(f'{joint} is not connected')
            targets[joint] = (config['id'], self._tc_goal_tick(joint, rad))
        return targets

    def _tc_move(self, poses, vertical):
        self._tc_stream = None
        if not vertical:
            for joint, (dxl_id, tick) in self._tc_targets(poses[-1]).items():
                self._write_register(dxl_id, 112, 4, self._pose_profile(joint, dxl_id),
                                     'tool change profile')
                self._write_register(dxl_id, 116, 4, tick & 0xffffffff, 'tool change goal')
                self._tc_goals[dxl_id] = tick
            return
        # Vertical moves stream short waypoints; each joint's speed is sized so
        # all joints finish a segment together, which keeps the wrist on a line.
        self._tc_stream = dict(poses=list(poses), index=0, next_at=self._tc_clock())

    def _tc_write_waypoint(self, pose):
        for joint, (dxl_id, tick) in self._tc_targets(pose).items():
            previous = self._tc_goals.get(dxl_id)
            if previous is None:
                previous = self._read_register(dxl_id, 116, 4, 'tool change goal', signed=True)
            units = abs(tick - previous) / self._tc_segment_s / TICKS_PER_PROFILE_UNIT_S
            self._write_register(dxl_id, 112, 4, max(1, math.ceil(units)),
                                 'tool change vertical profile')
            self._write_register(dxl_id, 116, 4, tick & 0xffffffff, 'tool change waypoint')
            self._tc_goals[dxl_id] = tick

    def _tc_arrived(self, pose):
        if self._tc_stream is not None:
            return False
        for dxl_id, tick in self._tc_targets(pose).values():
            if (self._read_register(dxl_id, 128, 4, 'tool change velocity', signed=True)
                    or abs(self._read_register(dxl_id, 132, 4, 'tool change position',
                                               signed=True) - tick) > ARRIVAL_TICKS):
                return False
        return True

    def _tc_lock(self, desired):
        self._require_manual_lock_arm_stopped()
        self._prepare_tool_for_lock()
        self._tc_lock_desired = desired
        self.toolchanger.move(desired, 1)

    def _tc_lock_state(self):
        lock = self.toolchanger
        if lock.state == 'FAULT':
            return lock.error or 'coupling lock fault'
        if lock.target is not None:
            return 'MOVING'
        if lock.state == self._tc_lock_desired:
            return 'DONE'
        return f'coupling stopped at {lock.state}'

    def _tc_attached_tool(self):
        if (self.tool_discovered and not self._physical_tool_detached
                and not self.tool_detached):
            return self.tool_type
        return None

    def _tc_hold(self):
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        self._tc_stream = None
        for config in JOINT_CONFIG.values():
            if config['id'] in self.active_ids:
                position = self._read_register(config['id'], 132, 4,
                                               'tool change hold', signed=True)
                self._write_register(config['id'], 116, 4, position & 0xffffffff,
                                     'tool change hold')

    def _tc_tick(self):
        with self._bus_lock:
            try:
                stream = self._tc_stream
                if stream is not None and self._tc_clock() >= stream['next_at']:
                    self._tc_write_waypoint(stream['poses'][stream['index']])
                    stream['index'] += 1
                    stream['next_at'] += self._tc_segment_s
                    if stream['index'] == len(stream['poses']):
                        self._tc_stream = None
                if self._tc_running():
                    self._tc_check_current()
                    self._tool_change.poll()
            except Exception as exc:
                self._tc_stream = None
                if self._tc_running():
                    self._tool_change.abort(str(exc))
            if self._tc_was_running and not self._tc_running():
                self._tc_was_running = False
                self._tc_goals.clear()
                self._restore_jog_profile({'ids': [
                    i for i in self.active_ids if i in self._arm_ids()]})
                self.get_logger().info(
                    f'local tool change {self._tool_change.state} {self._tool_change.error}')

    def _tc_check_current(self):
        """Same Present Current trip the bench pose move uses; skipped when trip is OFF."""
        if not getattr(self, '_tc_trip', True):
            return
        for dxl_id in self._arm_ids() & set(self.active_ids):
            current = self._read_register(dxl_id, 126, 2, 'tool change current', signed=True)
            if abs(current) > self.local_jog_trip_threshold:
                raise RuntimeError(f'id {dxl_id} current trip |{current}| > '
                                   f'{self.local_jog_trip_threshold}')

    def _arm_ids(self):
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        return {config['id'] for config in JOINT_CONFIG.values()}
