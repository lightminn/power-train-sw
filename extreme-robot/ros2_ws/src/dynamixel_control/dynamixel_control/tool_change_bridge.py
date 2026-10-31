"""교체 전용 ROS 입력. 일반 팔 명령과 교체 세션은 같은 입력을 공유하지 않는다."""

import json
import math
from pathlib import Path

from robot_arm_msgs.msg import ChassisMode
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dynamixel_control.dock_poses import (
    DockPoseStore, load_docks, tool_slots, validate_exchange)
from dynamixel_control.auto_tool_change import AutoToolChange
from dynamixel_control.qos_profiles import HEARTBEAT_QOS
from dynamixel_control.tool_change_interlock import ToolChangeDenied, ToolChangeInterlock


class ToolChangeBridge:
    """직렬 버스의 단일 소유자인 브리지 안에서 교체 권한을 관리한다."""

    def _init_tool_change(self):
        self._change_gate = ToolChangeInterlock()
        self._dock_table = None
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        self._dock_store = DockPoseStore(
            self.get_parameter('toolchanger_dock_file').value, JOINT_CONFIG)
        self._dock_status_pub = self.create_publisher(String, '/toolchanger/dock_status', 10)
        self.create_subscription(String, '/toolchanger/dock_command', self._dock_command,
                                 10, callback_group=self._command_group)
        if self._dock_store.filename and Path(self._dock_store.filename).is_file():
            try:
                self._dock_store.load()
                if self._dock_store.calibrated:
                    self._dock_table = self._dock_store.data
            except Exception as exc:
                self.get_logger().warn(f'dock table starts uncalibrated: {exc}')
        self._attached_tool = self.tool_type
        self._auto_mode = False
        self._auto_candidate_ids = set()
        self._auto_move_count = 0
        self._change_last_stamp = None
        self._auto_target_stamp = None
        self._change_arm_fault = False
        self._auto_verified_ready = False
        self._auto_change = (AutoToolChange(
            self.lock_sequence, self._auto_move, self._change_hold_arm, self._change_require,
            after_unlock=self._auto_release_tool, before_lock=self._auto_prepare_pickup,
            verify_tool=self._auto_verify_tool,
            before_unlock=self._auto_forget_attachment) if self.lock_sequence is not None else None)
        self.create_subscription(JointState, '/joint_states', self._change_joint_states,
                                 10, callback_group=self._command_group)
        self.create_subscription(String, '/toolchanger/change_tool', self._start_auto_change,
                                 10, callback_group=self._command_group)
        self._change_status = 'IDLE'
        self._change_error = ''
        self._change_status_pub = self.create_publisher(String, '/toolchanger/change_status', 10)
        self.create_subscription(ChassisMode, '/chassis_mode', self._change_chassis,
                                 HEARTBEAT_QOS, callback_group=self._command_group)
        self.create_subscription(String, '/toolchanger/session_command', self._change_session,
                                 10, callback_group=self._command_group)
        self.create_subscription(JointTrajectory, '/toolchanger/arm_trajectory',
                                 self._change_trajectory, 10,
                                 callback_group=self._command_group)
        self.create_timer(0.05, self._change_watchdog, callback_group=self._command_group)

    def _change_conditions(self):
        return dict(emergency=self.emergency_stop_active,
                    docks_valid=self._dock_table is not None,
                    now_ros=self.get_clock().now().nanoseconds * 1e-9)

    def _change_require(self):
        if self.control_scope != 'TOOL_CHANGE' or self.read_only or self.toolchanger is None:
            raise ToolChangeDenied('writable TOOL_CHANGE with coupling lock is required')
        self._change_gate.require_active(**self._change_conditions())
        if self._change_arm_fault:
            raise ToolChangeDenied('arm hardware feedback fault')

    def _change_chassis(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._change_gate.observe(msg.mode, stamp,
                                  self.get_clock().now().nanoseconds * 1e-9)
        self._change_watchdog()

    def _change_session(self, msg):
        try:
            request = json.loads(msg.data)
            command = request['command']
            if self.control_scope != 'TOOL_CHANGE':
                raise ToolChangeDenied('session commands require TOOL_CHANGE')
            if command == 'FINISH' and self._auto_mode:
                raise ToolChangeDenied('automatic change completes only after verified return')
            if command in ('ABORT', 'FINISH'):
                self._change_exit('ABORTED' if command == 'ABORT' else 'DONE', '')
            elif command == 'ARM':
                if self.read_only or self.toolchanger is None:
                    raise ToolChangeDenied('writable bridge and coupling lock are required')
                from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
                self._dock_table = None
                self._dock_table = load_docks(
                    self.get_parameter('toolchanger_dock_file').value, JOINT_CONFIG)
                self._change_gate.arm(**self._change_conditions())
                try:
                    self._stop_tool('tool change arm')
                    self._change_require()
                    if self.mock_mode:
                        self.torque_enabled_ids.update(c['id'] for c in JOINT_CONFIG.values())
                    for name, config in JOINT_CONFIG.items():
                        if not self.mock_mode and not self._enable_torque(config['id'], name):
                            raise ToolChangeDenied(f'arm torque enable failed: {name}')
                except Exception:
                    self._change_exit('ABORTED', 'tool change arm failed')
                    raise
                self._change_status, self._change_error = 'ARMED', ''
            else:
                raise ToolChangeDenied('unknown session command')
        except Exception as exc:
            self._change_error = str(exc)
        self._change_publish()

    def _change_trajectory(self, msg):
        try:
            self._change_require()
            if self._auto_mode:
                raise ToolChangeDenied('automatic sequence owns the dedicated input')
            self._change_send_target(msg)
            self._change_status = 'MOVING'
        except Exception as exc:
            self._change_error = str(exc)
            if self._change_gate.active:
                self._change_exit('ABORTED', str(exc))
        self._change_publish()

    def _change_send_target(self, msg):
        self._change_require()
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        from dynamixel_control.joint_limits import JOINT_LIMITS
        if len(msg.points) != 1 or msg.joint_names != list(JOINT_CONFIG):
            raise ToolChangeDenied('one complete arm target is required')
        positions = msg.points[0].positions
        if len(positions) != len(JOINT_CONFIG):
            raise ToolChangeDenied('arm target length mismatch')
        for name, value in zip(msg.joint_names, positions):
            limits = JOINT_LIMITS[name]
            if not math.isfinite(value) or not limits['lower'] <= value <= limits['upper']:
                raise ToolChangeDenied(f'arm target outside limit: {name}')
        with self._bus_lock:
            if self._auto_mode:
                self._auto_target_stamp = self.get_clock().now().nanoseconds * 1e-9
            self._dispatch_arm_trajectory(msg)

    def _change_watchdog(self):
        if self._change_gate.active:
            try:
                self._change_require()
            except Exception as exc:
                self._change_exit('ABORTED', str(exc))
        if self._auto_mode and self._change_gate.active:
            self._auto_change.poll()
            self._change_status = self._auto_change.state
            if self._auto_change.state == 'ABORTED':
                self._change_exit('ABORTED', self._auto_change.error)
            elif self._auto_change.state == 'DONE':
                confirm = self._auto_change.take_confirm()
                self._change_exit('DONE', '')
                if self._change_status == 'DONE' and confirm is not None:
                    self.toolchanger_confirm_pub.publish(String(data=json.dumps(confirm)))
        self._change_publish()

    def _change_exit(self, state, reason):
        self._change_gate.exit(reason)
        if self._auto_change is not None and self._auto_change.running:
            self._auto_change.abort(reason or 'session ended')
        self._auto_mode = False
        self._change_status, self._change_error = state, reason
        # 현재 위치를 목표로 잡고 팔 토크를 유지한다.
        try:
            self._change_hold_arm()
        except Exception as exc:
            self._change_status = 'ABORTED'
            self._change_error += f'; arm hold failed: {exc}'
        self._stop_tool('tool change exit')
        pending_error = self._auto_stop_unselected_tool()
        self._auto_candidate_ids.clear()
        if pending_error:
            self._change_status = 'ABORTED'
            self._change_error += '; ' + pending_error
        self._stop_toolchanger()
        if self.control_scope == 'TOOL_CHANGE' and not self.read_only:
            if self.tool_ids and self._current_tool_torque_state() != 'OFF':
                self._change_status = 'ABORTED'
                self._change_error += '; tool torque OFF is not confirmed'
            if self.toolchanger is not None:
                self.toolchanger.observe()
                if not self.toolchanger.sample or self.toolchanger.sample['torque'] != 0:
                    self._change_status = 'ABORTED'
                    self._change_error += '; lock torque OFF is not confirmed'

    def _change_hold_arm(self):
        if self.mock_mode or self.read_only:
            return
        from dynamixel_control.moveit_dynamixel_bridge import (
            JOINT_CONFIG, ADDR_PRESENT_POSITION, ADDR_GOAL_POSITION)
        with self._bus_lock:
            for config in JOINT_CONFIG.values():
                motor = config['id']
                position = self._read_register(motor, ADDR_PRESENT_POSITION, 4,
                                               'change hold position', signed=True)
                self._write_register(motor, ADDR_GOAL_POSITION, 4, position,
                                     'change hold goal')

    def _change_publish(self):
        self._change_status_pub.publish(String(data=json.dumps(dict(
            state=self._change_status, armed=self._change_gate.active,
            error=self._change_error, docks_valid=self._dock_table is not None,
            attached_tool=self._attached_tool, moves_sent=self._auto_move_count,
            step_index=(self._auto_change.index if self._auto_change else -1),
            verified_ready=self._auto_verified_ready))))

    def _dock_command(self, msg):
        error = ''
        try:
            if (self.control_scope != 'TOOL_CHANGE' or self.read_only
                    or self._change_gate.active):
                raise ToolChangeDenied('teach requires writable, disarmed TOOL_CHANGE')
            request = json.loads(msg.data)
            if request['command'] == 'LOAD':
                self._dock_store.load()
            elif request['command'] == 'CONFIGURE':
                self._dock_store.configure_exchange(request['exchange'])
            elif request['command'] == 'CAPTURE':
                self._dock_store.capture(
                    request['slot_id'], request['pose'], request['tool_type'],
                    self._dock_arm_samples(), request['tolerance_rad'],
                    request['timeout_s'], self.get_clock().now().nanoseconds * 1e-9)
            else:
                raise ValueError('dock command must be LOAD, CONFIGURE or CAPTURE')
            self._dock_table = self._dock_store.data if self._dock_store.calibrated else None
        except Exception as exc:
            error = str(exc)
            self._dock_table = None
        self._dock_status_pub.publish(String(data=json.dumps(dict(
            calibrated=self._dock_table is not None, error=error,
            slots=list(self._dock_store.data['slots'])))))

    def _dock_arm_samples(self):
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        if self.mock_mode:
            return {name: dict(position=self._mock_arm_positions[name], velocity=0,
                               torque=int(config['id'] in self.torque_enabled_ids))
                    for name, config in JOINT_CONFIG.items()}
        samples = {}
        with self._bus_lock:
            for name, config in JOINT_CONFIG.items():
                motor = config['id']
                samples[name] = dict(
                    position=self.tick_to_rad(name, self._read_register(
                        motor, 132, 4, 'teach position', signed=True)),
                    velocity=self._read_register(motor, 128, 4, 'teach velocity', signed=True),
                    torque=self._read_register(motor, 64, 1, 'teach torque'))
        return samples

    def _start_auto_change(self, msg):
        try:
            self._change_require()
            if self._auto_mode:
                raise ToolChangeDenied('a tool change is already running')
            request = json.loads(msg.data) if msg.data.lstrip().startswith('{') else {
                'tool_type': msg.data.strip()}
            desired = request['tool_type']
            release, pickup = tool_slots(self._dock_table, self._attached_tool, desired)
            settings = validate_exchange(self._dock_table.get('exchange'))
            if self.lock_sequence.load_threshold != 0:
                raise ToolChangeDenied('automatic change requires disabled load proxy')
            if self.lock_sequence.state in ('MOVING', 'TOOL_STOP'):
                raise ToolChangeDenied('coupling lock is already moving')
            self.toolchanger.enable()
            if not self.toolchanger.ready:
                raise ToolChangeDenied('known locked coupling is required before change')
            if (self._change_last_stamp is None or not 0 <=
                    self.get_clock().now().nanoseconds * 1e-9 - self._change_last_stamp
                    <= settings['feedback_timeout_s']):
                raise ToolChangeDenied('fresh complete joint_states are required')
            self.lock_sequence.take_confirm()
            self._auto_move_count = 0
            self._auto_verified_ready = False
            self._auto_change.start(self._dock_table, release, pickup, desired, **settings)
            self._auto_mode = True
            self._change_status, self._change_error = 'ARMED', ''
        except Exception as exc:
            self._change_error = str(exc)
            if self._change_gate.active:
                self._change_exit('ABORTED', str(exc))
        self._change_publish()

    def _change_joint_states(self, msg):
        if self.control_scope != 'TOOL_CHANGE' or self._auto_change is None:
            return
        from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
        names = list(JOINT_CONFIG)
        if not set(msg.name).intersection(names):
            return  # ID1 잠금각 메시지는 팔 피드백이 아니다.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if (set(msg.name) != set(names) or len(msg.name) != len(names)
                or len(msg.position) != len(names) or len(msg.velocity) != len(names)
                or stamp <= 0 or stamp > now
                or (self._change_last_stamp is not None and stamp <= self._change_last_stamp)):
            self._auto_change.observe({}, {})
            if self._auto_mode:
                self._change_exit('ABORTED', 'invalid or out-of-order joint_states')
            return
        self._change_last_stamp = stamp
        if self._auto_mode:
            if now - stamp > self._auto_change.feedback_timeout_s:
                self._change_exit('ABORTED', 'joint_states source timestamp is stale')
                return
            if self._auto_target_stamp is not None and stamp <= self._auto_target_stamp:
                return  # 지령 전에 생성되어 큐에 남아 있던 샘플은 도착 근거가 아니다.
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        # 표의 JOINT_CONFIG 순서로 정렬한다. DDS 배열 순서는 권한 조건이 아니다.
        self._auto_change.observe({n: positions[n] for n in names},
                                  {n: velocities[n] for n in names})

    def _auto_move(self, names, values):
        msg = JointTrajectory(joint_names=names,
                              points=[JointTrajectoryPoint(positions=values)])
        self._change_send_target(msg)
        self._auto_move_count += 1

    def _auto_release_tool(self):
        with self._bus_lock:
            # 도구 정지·해제를 확인한 뒤 옛 ID를 버스 읽기 목록에서도 퇴역시킨다.
            self._unregister_tool_feedback_ids(list(self.tool_ids))
            self._attached_tool = None
            self.tool_ids = []
            self.gripper_ids = []
            self._tool_samples = {}
            self._fsm_allowlist = set()
            self.tool_fsm = None
            self.tool_discovered = False
            self.tool_motion_allowed = False

    def _auto_prepare_pickup(self):
        # 안착 상태에서 새 도구를 토크OFF로 준비한 뒤 기존 잠금 코어에 넘긴다.
        self._change_require()
        with self._bus_lock:
            self._switch_tool_runtime(self._auto_change.tool_type)
        if (not self.tool_discovered or self._tool_preparation_state != 'READY'
                or self._current_tool_torque_state() != 'OFF'):
            raise ToolChangeDenied('pickup tool is not discovered, prepared and torque OFF')

    def _auto_verify_tool(self):
        self._attached_tool = self._auto_change.tool_type
        if not self.toolchanger.ready:
            raise ToolChangeDenied('new coupling lock is not ready')
        if self.lock_sequence.coupling_proxy != 'DISABLED':
            raise ToolChangeDenied('unexpected automatic coupling proxy result')
        # 기존 /tool/change와 같은 재준비 경로를 결합 직후 한 번 더 실행한다.
        with self._bus_lock:
            self._switch_tool_runtime(self._attached_tool)
        if (not self.tool_discovered or self._tool_preparation_state != 'READY'
                or self.tool_fsm is None or self.tool_fsm.state.name != 'READY'
                or self._current_tool_torque_state() != 'OFF'):
            raise ToolChangeDenied('new tool discovery/READY/torque-OFF verification failed')
        self._auto_verified_ready = True

    def _auto_forget_attachment(self):
        # 해제 시도가 중간에 실패하면 장착 상태를 추측해 다음 교체를 시작하지 않는다.
        self._attached_tool = None

    def _change_preparation_reason(self):
        """교체 중 토크OFF 준비의 게이트. 토크ON 허가와 별개이며 잠금을 요구하지 않는다."""
        if (self.control_scope != 'TOOL_CHANGE' or self.read_only
                or self.emergency_stop_active or self.tool_detached):
            return 'writable TOOL_CHANGE without emergency/detached latch is required'
        if not self.tool_selection or not self.tool_selection.valid or not self.tool_profile.get('calibrated'):
            return 'valid calibrated tool profile is required'
        if not self.tool_discovered or not self._tool_actuators_online():
            return 'every selected tool actuator must be discovered and online'
        if any(sample.get('hardware_error') != 0 or sample.get('torque_state') != 'OFF'
               for sample in self._tool_samples.values()):
            return 'healthy torque-OFF tool feedback is required'
        return ''

    def _auto_stop_unselected_tool(self):
        """프로필 확인 중 실패해 런타임에 등록되지 못한 새 도구도 정지를 시도한다."""
        errors = []
        for motor in self._auto_candidate_ids - set(self.tool_ids):
            if self.mock_mode:
                sample = self._tool_samples.get(motor)
                if sample is not None:
                    sample['torque_state'] = 'OFF'
                continue
            try:
                with self._bus_lock:
                    if not self._probe_tool_id(motor):
                        raise RuntimeError('unreachable; torque OFF is unconfirmed')
                    self._write_register(motor, 64, 1, 0, 'candidate tool stop')
                    if self._read_register(motor, 64, 1, 'candidate tool stop readback') != 0:
                        raise RuntimeError('torque OFF readback failed')
            except Exception as exc:
                errors.append(f'candidate ID{motor}: {exc}')
        return '; '.join(errors)
