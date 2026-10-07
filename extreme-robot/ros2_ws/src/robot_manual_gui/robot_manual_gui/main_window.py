"""PyQt5 widgets for manual robot validation."""

import time
import json

from PyQt5.QtCore import QEvent, QLibraryInfo, QProcess, QTimer, QTranslator, Qt
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QApplication, QComboBox, QDoubleSpinBox, QFormLayout,
    QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QLineEdit, QScrollArea, QTableWidget, QTableWidgetItem, QTextEdit,
    QSizePolicy, QVBoxLayout, QWidget)

from robot_manual_gui.ros_interface import ARM_JOINTS
from robot_manual_gui.korean_text import ko
from robot_manual_gui.qt_lifecycle import restart_parent_launch
from dynamixel_control.tool_manager import ToolManager


TRUE_STYLE = 'color: #0b7a25; font-weight: bold;'
FALSE_STYLE = 'color: #b00020; font-weight: bold;'
ESTOP_STYLE = 'background: #b00020; color: white; font-size: 20px; font-weight: bold;'

# `/tool/status` older than this is not evidence of anything current.
STATUS_FRESH_S = 1.5
# The bridge answers a `/tool/change` inside one status period; this is the
# GUI's own deadline for giving up on an answer.  It only releases the request
# latch — every motion gate is recomputed from live status afterwards.
TOOL_CHANGE_TIMEOUT_S = 3.0
# A request published while `/tool/status` is already stale has no listener we
# can observe.  Report the lost link instead of making the operator wait out
# the full deadline, but allow one grace window for a late status sample.
TOOL_CHANGE_DISCONNECT_GRACE_S = 1.0
# Every request ends in exactly one of these; `PENDING` is the only state that
# latches the tool panel.
TOOL_CHANGE_STATES = (
    'IDLE', 'PENDING', 'DONE', 'REJECTED', 'TIMEOUT', 'DISCONNECTED', 'CANCELED')


class ManualMainWindow(QMainWindow):
    """Hardware-test dashboard backed exclusively by ROS interfaces."""

    def __init__(self, ros_node, signals, profile, mock_mode=False):
        super().__init__()
        app = QApplication.instance()
        # A translator belongs to QApplication, not to each hot-swapped
        # window.  Reinstalling/removing a window child left deferred
        # LanguageChange work for the offscreen backend during shutdown.
        translator = getattr(app, '_robot_manual_gui_korean_translator', None)
        if translator is None:
            translator = QTranslator(app)
            translator.load(
                'qtbase_ko', QLibraryInfo.location(QLibraryInfo.TranslationsPath))
            app.installTranslator(translator)
            app._robot_manual_gui_korean_translator = translator
        self._qt_korean = translator
        self.node = ros_node
        self.signals = signals
        self.profile = profile
        self.mock_mode = mock_mode
        self.tool_status = {}
        # Injectable so a test can drive the request deadline without sleeping.
        self._clock = time.monotonic
        self.pending_tool_change = None
        self.pending_tool_change_started = None
        self.pending_tool_change_link_ok = False
        self.tool_change_requested = None
        self.tool_change_state = 'IDLE'
        self.tool_change_detail = ''
        self.tool_change_timeout_s = TOOL_CHANGE_TIMEOUT_S
        self.tool_change_disconnect_grace_s = TOOL_CHANGE_DISCONNECT_GRACE_S
        self.spur_hold_command = None
        self.spur_hold_context = None
        self.spur_hold_timer = QTimer(self)
        self.spur_hold_timer.setInterval(100)
        self.spur_hold_timer.timeout.connect(self._repeat_spur_hold)
        self.fsm_state = 'UNKNOWN'
        self.arm_fsm_state = 'UNKNOWN'
        self.control_mode = 'FSM'
        self.last_status_time = 0.0
        self.processes = []
        self.joint_rows = {}
        self.seen_arm_joints = set()
        self.arm_widgets = {}
        self.gripper_busy = False
        self.dual_hold_jog_active = False
        self.dual_hold_jog_direction = None
        self.dual_hold_context = None
        self.gripper_target_ticks = {}
        self.spur_torque_enabled = False
        self.spur_torque_state = 'UNKNOWN'
        self.spur_endpoints = {}
        self.spur_zero_tick = None
        self.dual_calibration_buttons = []
        self.dual_calibration_step = None
        self.dual_calibration_state = None
        self.dual_start_calibration = None
        self.dual_capture_open = None
        self.dual_capture_close = None
        self.dual_validate_calibration = None
        self.dual_save_calibration = None
        self.dual_capture_label = None
        self._runtime_context_generation = None
        # QApplication owns neither an installed translator nor an event
        # filter.  Both must be unregistered before this window (and its
        # translator child) is deleted; otherwise a later offscreen event can
        # dereference a QObject that Qt has already destroyed.
        self._qt_resources_released = False
        # External spur gears reverse rotation.  This is deliberately shown in
        # the GUI instead of being hidden in a raw-tick jog control.
        self.spur_output_direction = -1
        self.spur_gear_ratio = 1.0
        self.temporary_jog_safe_min = getattr(
            self.node, 'temporary_jog_safe_min', 2867)
        self.temporary_jog_safe_max = getattr(
            self.node, 'temporary_jog_safe_max', 3807)
        get_param = getattr(self.node, 'get_parameter', None)
        self.temporary_jog_mechanical_open = (
            get_param('temporary_jog_mechanical_open_tick').value
            if get_param else 2817)
        self.temporary_jog_mechanical_close = (
            get_param('temporary_jog_mechanical_close_tick').value
            if get_param else 3857)
        self.setWindowTitle(ko('Extreme Robot Manual Hardware Validation'))
        self.resize(1180, 850)
        self._build_ui()
        self._connect_signals()
        # Child widgets normally consume arrow keys for focus navigation.
        # Observe them before dispatch so hold-to-run works anywhere in this
        # window, while preserving arrow editing in input widgets.
        QApplication.instance().installEventFilter(self)
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._refresh_connection)
        self.watchdog.start(500)
        self.dual_key_jog_timer = QTimer(self)
        self.dual_key_jog_timer.setInterval(100)
        self.dual_key_jog_timer.timeout.connect(self._dual_key_jog_tick)
        self.dual_key_jog_direction = 0

    def _build_ui(self):
        root = QWidget()
        outer = QVBoxLayout(root)

        self.scope_banner = QLabel(ko(f'CONTROL / TEST SCOPE: {self.node.control_scope}'))
        self.scope_banner.setAlignment(Qt.AlignCenter)
        self.scope_banner.setStyleSheet(
            'font-size: 22px; font-weight: bold; padding: 8px; '
            'background: #ffe08a; color: #202020;')
        outer.addWidget(self.scope_banner)

        safety = QHBoxLayout()
        self.estop = QPushButton(ko('EMERGENCY STOP'))
        self.estop.setMinimumHeight(62)
        self.estop.setStyleSheet(ESTOP_STYLE)
        self.estop.clicked.connect(self._estop)
        self.detach = QPushButton(ko('TOOL DETACHED'))
        self.detach.clicked.connect(self._detach)
        self.restart_program = QPushButton(ko('프로그램 재구동'))
        self.restart_program.setEnabled(False)
        self.restart_program.clicked.connect(self._restart_program)
        self.estop_state = QLabel(ko('E-STOP: FALSE'))
        self.estop_state.setStyleSheet(TRUE_STYLE)
        safety.addWidget(self.estop, 3)
        safety.addWidget(self.detach)
        safety.addWidget(self.restart_program)
        safety.addWidget(self.estop_state)
        outer.addLayout(safety)

        columns = QHBoxLayout()
        columns.setSpacing(20)
        left_content = QWidget()
        left = QVBoxLayout(left_content)
        self.details_layout = left
        right_content = QWidget()
        right_content.setFixedWidth(380)
        right = QVBoxLayout(right_content)
        right.setAlignment(Qt.AlignTop)
        self.right_layout = right
        left.addWidget(self._status_group())
        left.addWidget(self._tool_selection_group())
        left.addWidget(self._arm_group())
        self.tool_control_box = self._tool_control_group()
        right.addWidget(self.tool_control_box)
        left.addWidget(self.tool_details_box)

        self.diag = QTableWidget(0, 5)
        self.diag.setHorizontalHeaderLabels(
            [ko('ID'), ko('Joint'), ko('Position'), ko('Current/Load'), ko('Online')])
        self.diag.setMinimumHeight(140)
        left.addWidget(self.diag)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        left.addWidget(self.log)
        # Independent scrolling keeps changing telemetry/calibration text
        # from moving the remote's buttons under the operator's pointer.
        status_scroll = QScrollArea()
        status_scroll.setWidgetResizable(True)
        status_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        status_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        status_scroll.setWidget(left_content)
        remote_scroll = QScrollArea()
        remote_scroll.setWidgetResizable(True)
        remote_scroll.setFixedWidth(404)
        remote_scroll.setWidget(right_content)
        columns.addWidget(status_scroll, 1)
        columns.addWidget(remote_scroll)
        outer.addLayout(columns, 1)
        self.content_widget = root
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scroll_area.setWidget(root)
        self.setCentralWidget(self.scroll_area)

    def _status_group(self):
        box = QGroupBox(ko('Connection / Status'))
        form = QFormLayout(box)
        self.status_labels = {}
        self.status_titles = {}
        for key, title in (
                ('connection', 'Bridge connection'),
                ('u2d2', 'U2D2 / serial'), ('tool_type', 'Tool type'),
                ('tool_change', 'Tool change request'),
                ('tool_block', 'Blocked reason'),
                ('profile_valid', 'Profile valid'),
                ('calibration_valid', 'Calibration / endpoints valid'),
                ('actuators_discovered', 'Actuators discovered'),
                ('motion_allowed', 'Motion allowed'), ('fsm', 'FSM state'),
                ('preparation', '도구 준비'),
                ('motor_health', '모터 / 토크 / 하드웨어 오류'),
                ('arm_fsm', '팔 FSM'), ('arm_status', 'Arm contract state'), ('mode', 'Control mode'),
                ('dual_online', 'ID3 / ID4 online'),
                ('dual_positions', 'ID3 / ID4 positions'),
                ('dual_torque', 'ID3 / ID4 torque'),
                ('dual_hw_error', 'ID3 / ID4 hardware error'),
                ('dual_sync', 'Dual synchronization'),
                ('spur_online', 'ID5 연결'),
                ('spur_position', 'ID5 위치 (틱)'),
                ('spur_torque', 'ID5 토크'),
                ('spur_hw_error', 'ID5 하드웨어 오류'),
                ('spur_operating_mode', 'ID5 운전 모드'),
                ('contact', 'Contact sensor')):
            label = QLabel(ko('UNKNOWN'))
            label.setWordWrap(True)
            label.setMinimumWidth(0)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            self.status_labels[key] = label
            title_label = QLabel(ko(title))
            self.status_titles[key] = title_label
            form.addRow(title_label, label)
        self._refresh_tool_change_labels()
        self._update_status_panel({})
        return box

    def _update_status_panel(self, status):
        """Display-only selection; never publish a motor or ownership command."""
        tool = self.node.selected_tool
        for key, label in self.status_labels.items():
            visible = (tool == 'dual_motor_gripper' if key.startswith('dual_')
                       else tool == 'spur_1motor_gripper' if key.startswith('spur_')
                       else True)
            label.setVisible(visible)
            self.status_titles[key].setVisible(visible)
        if hasattr(self, 'diag'):
            self.diag.setVisible(True)
        self.status_labels['preparation'].setText(
            ko(status.get('tool_preparation_state', 'UNKNOWN')))
        self.status_labels['motor_health'].setText('\n'.join(
            ko(f'ID {item.get("id", "?")} · online={item.get("online", "UNKNOWN")} '
               f'· torque {item.get("torque_state", "UNKNOWN")} '
               f'· hardware_error {item.get("hardware_error", "UNKNOWN")}')
            for item in status.get('actuators', [])) or ko('UNKNOWN'))
        if hasattr(self, 'remote_tool_name'):
            names = {'dual_motor_gripper': '2모터 그리퍼',
                     'spur_1motor_gripper': '1모터 스퍼 그리퍼',
                     'cleaner': '클리너'}
            reported = status.get('tool_type')
            self.remote_tool_name.setText(names.get(reported, '도구 확인 중'))
            self.remote_state.setText(status.get('fsm_state') or 'UNKNOWN')
            self.remote_torque.setText(
                'Torque: ' + status.get('tool_torque_state', 'UNKNOWN'))
            self.remote_ids.setText('ID ' + (', '.join(
                str(item.get('id', '?')) for item in status.get('actuators', [])) or '—'))
        sample = next((item for item in status.get('actuators', [])
                       if item.get('id') == 5), {}) if tool == 'spur_1motor_gripper' else {}
        for key, field in (('spur_online', 'online'), ('spur_position', 'position'),
                           ('spur_torque', 'torque_state'), ('spur_hw_error', 'hardware_error'),
                           ('spur_operating_mode', 'operating_mode')):
            value = sample.get(field)
            if value is None:
                text = 'UNKNOWN'
            elif field == 'online':
                text = '연결됨' if value else '연결 끊김'
            elif field == 'operating_mode':
                text = {1: '1 (속도 제어)', 3: '3 (위치 제어)',
                        4: '4 (확장 위치 제어)', 5: '5 (전류 기반 위치 제어)'}.get(value, str(value))
            else:
                text = str(value)
            self.status_labels[key].setText(ko(text))

    def _arm_group(self):
        box = QGroupBox(ko('Arm Manual Control'))
        layout = QGridLayout(box)
        layout.addWidget(QLabel(ko('Joint')), 0, 0)
        layout.addWidget(QLabel(ko('Current rad')), 0, 1)
        layout.addWidget(QLabel(ko('Jog')), 0, 2, 1, 2)
        layout.addWidget(QLabel(ko('Target rad')), 0, 4)
        self.arm_buttons = []
        self.arm_position_labels = {}
        self.arm_targets = {}
        for row, joint in enumerate(ARM_JOINTS, 1):
            label = QLabel(ko('0.0000'))
            minus = QPushButton(ko('−'))
            plus = QPushButton(ko('+'))
            target = QDoubleSpinBox()
            target.setRange(-6.283, 6.283)
            target.setDecimals(4)
            send = QPushButton(ko('GO'))
            minus.clicked.connect(
                lambda _checked=False, name=joint: self._jog(name, -1))
            plus.clicked.connect(
                lambda _checked=False, name=joint: self._jog(name, 1))
            send.clicked.connect(
                lambda _checked=False, name=joint: self._arm_target(name))
            layout.addWidget(QLabel(ko(joint)), row, 0)
            layout.addWidget(label, row, 1)
            layout.addWidget(minus, row, 2)
            layout.addWidget(plus, row, 3)
            layout.addWidget(target, row, 4)
            layout.addWidget(send, row, 5)
            self.arm_position_labels[joint] = label
            self.arm_targets[joint] = target
            self.arm_buttons.extend([minus, plus, target, send])
            self.arm_widgets[joint] = [minus, plus, target, send]
        self.jog_step = QComboBox()
        self.jog_step.addItems(['0.5', '1.0', '5.0'])
        layout.addWidget(QLabel(ko('Jog step (deg)')), 6, 0)
        layout.addWidget(self.jog_step, 6, 1)
        return box

    def _tool_selection_group(self):
        box = QGroupBox(ko('Tool Selection / Ownership'))
        form = QFormLayout(box)
        self.tool_combo = QComboBox()
        for tool in ('dual_motor_gripper', 'spur_1motor_gripper', 'cleaner'):
            self.tool_combo.addItem(ko(tool), tool)
        self.tool_combo.setCurrentIndex(self.tool_combo.findData(self.node.selected_tool))
        self.request_tool_change = QPushButton(ko('REQUEST TOOL CHANGE'))
        self.request_tool_change.clicked.connect(self._request_tool_change)
        # Deliberately outside `tool_control_box`: this is the operator's way
        # out of a pending request, so it must stay clickable while the tool
        # panel is latched.
        self.cancel_tool_change = QPushButton(ko('CANCEL TOOL CHANGE'))
        self.cancel_tool_change.clicked.connect(self._cancel_tool_change)
        self.cancel_tool_change.setEnabled(False)
        self.mode_combo = QComboBox()
        for mode in ('FSM', 'MANUAL'):
            self.mode_combo.addItem(ko(mode), mode)
        self.request_mode = QPushButton(ko('REQUEST MODE'))
        self.request_mode.clicked.connect(self._request_mode)
        form.addRow(ko('Selected tool'), self.tool_combo)
        form.addRow(ko(''), self.request_tool_change)
        form.addRow(ko(''), self.cancel_tool_change)
        form.addRow(ko('Ownership'), self.mode_combo)
        form.addRow(ko(''), self.request_mode)
        return box

    def _clear_tool_specific_widget_refs(self):
        """Drop Python references to widgets owned by the old tool panel."""
        self.dual_recovery_buttons = []
        self.dual_calibration_buttons = []
        for name in (
                'dual_start_calibration', 'dual_calibration_state',
                'dual_calibration_step', 'dual_capture_open',
                'dual_capture_close', 'dual_capture_label',
                'dual_validate_calibration', 'dual_save_calibration',
                'spur_actual_state', 'capture_open', 'capture_close',
                'captured_endpoints_label', 'validate_calibration',
                'save_calibration', 'spur_mapping', 'spur_minus_5',
                'spur_zero', 'spur_plus_5', 'motor_minus_half', 'motor_plus_half',
                'motor_minus_one', 'motor_plus_one', 'developer_direct_box',
                'developer_enable', 'developer_disable', 'developer_minus',
                'developer_plus', 'developer_minus_five',
                'developer_plus_five', 'developer_hold',
                'spur_calibration_box', 'spur_calibration_state',
                'spur_calibration_start', 'spur_calibration_enable',
                'spur_calibration_disable', 'spur_calibration_minus',
                'spur_calibration_plus', 'spur_calibration_minus_five',
                'spur_calibration_plus_five', 'spur_calibration_capture_open',
                'spur_calibration_capture_close',
                'spur_calibration_validate', 'spur_calibration_save',
                'clean_start', 'clean_stop', 'read_diag', 'start_cal',
                'jog_close', 'jog_open', 'gripper_jog_step',
                'gripper_busy_label', 'gripper_position_label',
                'gripper_feedback_label'):
            setattr(self, name, None)

    def _tool_control_group(self):
        # This panel is rebuilt when the active runtime tool changes. Clear
        # references to widgets belonging to the previous tool first.
        self._clear_tool_specific_widget_refs()
        box = QGroupBox('엔드이펙터 리모컨')
        box.setObjectName('toolRemote')
        box.setStyleSheet('''
            QGroupBox#toolRemote { background: #f6f8fc; color: #203047;
                border: 1px solid #d5deea; border-radius: 16px;
                margin-top: 12px; padding: 18px 12px 12px; }
            QGroupBox#toolRemote::title { subcontrol-origin: margin; left: 18px; }
            QGroupBox#toolRemote QLabel { color: #203047; background: transparent; }
            QGroupBox#toolRemote QPushButton { background: white; color: #203047;
                border: 1px solid #c9d5e4; border-radius: 12px;
                font-size: 17px; font-weight: bold; padding: 6px; }
            QGroupBox#toolRemote QPushButton:hover { background: #eaf0fa; }
            QGroupBox#toolRemote QPushButton:pressed { background: #d6e3f7; }
            QGroupBox#toolRemote QPushButton#remoteEnable {
                background: #2563eb; color: white; border-color: #2563eb; }
            QGroupBox#toolRemote QPushButton#remoteEnable:hover { background: #1d4ed8; }
            QGroupBox#toolRemote QPushButton#remoteEnable:pressed { background: #1e40af; }
            QGroupBox#toolRemote QPushButton#remoteStop {
                background: #c6283e; color: white; border-color: #c6283e; }
            QGroupBox#toolRemote QPushButton#remoteStop:hover { background: #ae2034; }
            QGroupBox#toolRemote QPushButton#remoteStop:pressed { background: #8d192a; }
            QGroupBox#toolRemote QPushButton:disabled,
            QGroupBox#toolRemote QPushButton#remoteEnable:disabled,
            QGroupBox#toolRemote QPushButton#remoteStop:disabled {
                background: #e8edf3; color: #8793a4; border-color: #d9e0e9; }
        ''')
        remote = QVBoxLayout(box)
        remote.setSpacing(14)
        remote.addWidget(QLabel('현재 도구'))
        self.remote_tool_name = QLabel('도구 확인 중')
        self.remote_tool_name.setStyleSheet('font-size: 24px; font-weight: bold;')
        self.remote_state = QLabel('UNKNOWN')
        self.remote_ids = QLabel('ID —')
        self.remote_torque = QLabel('Torque: UNKNOWN')
        self.remote_torque.setStyleSheet('font-size: 18px; font-weight: bold;')
        remote.addWidget(self.remote_tool_name)
        state_row = QHBoxLayout()
        state_row.addWidget(self.remote_state)
        state_row.addStretch()
        state_row.addWidget(self.remote_ids)
        remote.addLayout(state_row)
        remote.addWidget(self.remote_torque)
        remote.addSpacing(8)
        self.tool_details_box = QGroupBox('도구 상세 / 캘리브레이션')
        layout = QVBoxLayout(self.tool_details_box)
        self.profile_text = QLabel(ko(self._profile_summary()))
        self.profile_text.setWordWrap(True)
        layout.addWidget(self.profile_text)
        if (getattr(self.node, 'developer_direct_mode', False)
                and self.node.selected_tool == 'spur_1motor_gripper'
                and self.node.control_scope == 'END_EFFECTOR_ONLY'):
            self.developer_direct_box = QGroupBox(ko('개발자 직접 구동 · ID 5 전용'))
            self.developer_direct_box.setStyleSheet(
                'QGroupBox { font-weight: bold; color: #7a4100; }')
            developer = QGridLayout(self.developer_direct_box)
            notice = QLabel(ko(
                '수동 권한·FSM·보정 절차 없이 즉시 시험합니다. '
                '비상 정지, 오류, 오프라인, 안전 범위는 계속 차단됩니다.'))
            notice.setWordWrap(True)
            developer.addWidget(notice, 0, 0, 1, 3)
            self.developer_enable = QPushButton(ko('토크 켜기'))
            self.developer_disable = QPushButton(ko('토크 끄기'))
            self.developer_hold = QPushButton(ko('현재 위치 정지'))
            self.developer_minus = QPushButton(ko('−0.5°'))
            self.developer_plus = QPushButton(ko('+0.5°'))
            self.developer_minus_five = QPushButton(ko('−5° 크게 이동'))
            self.developer_plus_five = QPushButton(ko('+5° 크게 이동'))
            self.developer_enable.clicked.connect(
                lambda: self._developer_spur_command('manual_enable'))
            self.developer_disable.clicked.connect(
                lambda: self._developer_spur_command('manual_disable'))
            self.developer_hold.clicked.connect(
                lambda: self._developer_spur_command('manual_hold'))
            self.developer_minus.clicked.connect(
                lambda: self._developer_spur_command('manual_step', -0.5))
            self.developer_plus.clicked.connect(
                lambda: self._developer_spur_command('manual_step', 0.5))
            self.developer_minus_five.clicked.connect(
                lambda: self._developer_spur_command('manual_step', -5.0))
            self.developer_plus_five.clicked.connect(
                lambda: self._developer_spur_command('manual_step', 5.0))
            developer.addWidget(self.developer_enable, 1, 0)
            developer.addWidget(self.developer_disable, 1, 1)
            developer.addWidget(self.developer_hold, 1, 2)
            developer.addWidget(self.developer_minus, 2, 0, 1, 2)
            developer.addWidget(self.developer_plus, 2, 2)
            developer.addWidget(self.developer_minus_five, 3, 0, 1, 2)
            developer.addWidget(self.developer_plus_five, 3, 2)
            layout.addWidget(self.developer_direct_box)
        if self.node.selected_tool == 'spur_1motor_gripper':
            # Reuse the existing bridge-owned CalibrationSession rather than
            # creating a second calibration protocol in the GUI.  This gives
            # the operator one compact, ordered endpoint workflow.
            self.spur_calibration_box = QGroupBox(ko('그리퍼 끝점 캘리브레이션 · ID 5'))
            calibration = QGridLayout(self.spur_calibration_box)
            guide = QLabel(ko(
                '1. 시작  2. 토크 켜기  3. ±5°로 근접 후 ±0.5°로 미세 조정  '
                '4. 열림/닫힘 현재 위치 기록  5. 검증  6. 저장'))
            guide.setWordWrap(True)
            calibration.addWidget(guide, 0, 0, 1, 4)
            self.spur_calibration_state = QLabel(ko('시작 전'))
            calibration.addWidget(self.spur_calibration_state, 1, 0, 1, 4)
            self.spur_calibration_start = QPushButton(ko('캘리브레이션 시작'))
            self.spur_calibration_enable = QPushButton(ko('토크 켜기'))
            self.spur_calibration_disable = QPushButton(ko('토크 끄기'))
            self.spur_calibration_start.clicked.connect(self._start_calibration)
            self.spur_calibration_enable.clicked.connect(self._enable_spur_motor)
            self.spur_calibration_disable.clicked.connect(self._disable_spur_motor)
            calibration.addWidget(self.spur_calibration_start, 2, 0, 1, 2)
            calibration.addWidget(self.spur_calibration_enable, 2, 2)
            calibration.addWidget(self.spur_calibration_disable, 2, 3)
            self.spur_calibration_minus = QPushButton(ko('−0.5° 이동'))
            self.spur_calibration_plus = QPushButton(ko('+0.5° 이동'))
            self.spur_calibration_minus.clicked.connect(
                lambda: self._calibration_jog(-0.5))
            self.spur_calibration_plus.clicked.connect(
                lambda: self._calibration_jog(0.5))
            calibration.addWidget(self.spur_calibration_minus, 3, 0, 1, 2)
            calibration.addWidget(self.spur_calibration_plus, 3, 2, 1, 2)
            self.spur_calibration_minus_five = QPushButton(ko('−5° 크게 이동'))
            self.spur_calibration_plus_five = QPushButton(ko('+5° 크게 이동'))
            self.spur_calibration_minus_five.clicked.connect(
                lambda: self._calibration_jog(-5.0))
            self.spur_calibration_plus_five.clicked.connect(
                lambda: self._calibration_jog(5.0))
            calibration.addWidget(self.spur_calibration_minus_five, 4, 0, 1, 2)
            calibration.addWidget(self.spur_calibration_plus_five, 4, 2, 1, 2)
            self.spur_calibration_capture_open = QPushButton(ko('현재 위치를 열림으로 기록'))
            self.spur_calibration_capture_close = QPushButton(ko('현재 위치를 닫힘으로 기록'))
            self.spur_calibration_capture_open.clicked.connect(
                lambda: self._capture_spur_endpoint('open'))
            self.spur_calibration_capture_close.clicked.connect(
                lambda: self._capture_spur_endpoint('close'))
            calibration.addWidget(self.spur_calibration_capture_open, 5, 0, 1, 2)
            calibration.addWidget(self.spur_calibration_capture_close, 5, 2, 1, 2)
            self.spur_calibration_validate = QPushButton(ko('기록값 검증'))
            self.spur_calibration_save = QPushButton(ko('프로파일 저장'))
            self.spur_calibration_validate.clicked.connect(self._validate_spur_calibration)
            self.spur_calibration_save.clicked.connect(self._save_spur_calibration)
            calibration.addWidget(self.spur_calibration_validate, 6, 0, 1, 2)
            calibration.addWidget(self.spur_calibration_save, 6, 2, 1, 2)
            layout.addWidget(self.spur_calibration_box)
        if not hasattr(self, 'common_enable'):
            self.open_button = QPushButton(ko('OPEN'))
            self.close_button = QPushButton(ko('CLOSE'))
            self.tool_stop = QPushButton(ko('STOP'))
            self.hold_open_button = QPushButton(ko('HOLD TO OPEN'))
            self.hold_close_button = QPushButton(ko('HOLD TO CLOSE'))
            self.common_enable = QPushButton('활성화')
            self.common_disable = QPushButton('비활성화')
            # Cleaner direction must leave the GUI on mouse/key press, not on
            # Qt's clicked signal (which fires only after release).  The same
            # shared buttons retain release-click behaviour for grippers.
            self.open_button.pressed.connect(
                lambda: self._common_pressed(2))
            self.close_button.pressed.connect(
                lambda: self._common_pressed(1))
            self.open_button.clicked.connect(
                lambda: self._common_clicked(2))
            self.close_button.clicked.connect(
                lambda: self._common_clicked(1))
            self.tool_stop.pressed.connect(self._tool_stop_pressed)
            self.tool_stop.clicked.connect(self._tool_stop_clicked)
            for button, enabled in ((self.common_enable, True),
                                    (self.common_disable, False)):
                button.pressed.connect(
                    lambda value=enabled: self._trace_torque_button('pressed', value))
                button.released.connect(
                    lambda value=enabled: self._trace_torque_button('released', value))
                button.clicked.connect(
                    lambda _checked=False, value=enabled:
                    self._trace_torque_button('clicked', value))
            self.common_enable.clicked.connect(lambda: self._common_torque(True))
            self.common_disable.clicked.connect(lambda: self._common_torque(False))
            self.hold_open_button.pressed.connect(lambda: self._common_hold('OPEN'))
            self.hold_close_button.pressed.connect(lambda: self._common_hold('CLOSE'))
            self.hold_open_button.released.connect(self._common_release)
            self.hold_close_button.released.connect(self._common_release)
            self.spur_enable = self.dual_enable = self.common_enable
            self.spur_disable = self.dual_disable = self.common_disable
        # Only geometry/text changes: the persistent buttons above keep all
        # original pressed/released/clicked connections and dispatchers.
        for button in self._common_buttons():
            button.setMinimumHeight(56)
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.tool_stop.setObjectName('remoteStop')
        self.tool_stop.setText('■ 정지')
        self.common_enable.setObjectName('remoteEnable')
        self.hold_open_button.setText('누르는 동안 열기')
        self.hold_close_button.setText('누르는 동안 닫기')
        remote.addWidget(self.tool_stop)
        actions = QGridLayout()
        actions.setSpacing(12)
        actions.setColumnStretch(0, 1)
        actions.setColumnStretch(1, 1)
        actions.addWidget(self.close_button, 0, 0)
        actions.addWidget(self.open_button, 0, 1)
        actions.addWidget(self.common_disable, 1, 0)
        actions.addWidget(self.common_enable, 1, 1)
        remote.addLayout(actions)
        remote.addSpacing(6)
        remote.addWidget(self.hold_open_button)
        remote.addWidget(self.hold_close_button)
        self.dual_recovery_buttons = []
        if self.node.selected_tool == 'dual_motor_gripper':
            recovery = QGroupBox(ko('MANUAL DUAL MOTOR RECOVERY (one click only)'))
            recovery_layout = QGridLayout(recovery)
            for row, dxl_id in enumerate((3, 4)):
                recovery_layout.addWidget(QLabel(ko(f'ID{dxl_id}')), row, 0)
                for column, delta in enumerate((-0.5, 0.5), 1):
                    button = QPushButton(ko(f'ID{dxl_id} {delta:+.1f}°'))
                    button.setAutoRepeat(False)
                    button.clicked.connect(
                        lambda _checked=False, motor=dxl_id, step=delta:
                        self._manual_dual_recovery_jog(motor, step))
                    recovery_layout.addWidget(button, row, column)
                    self.dual_recovery_buttons.append((dxl_id, button))
            layout.addWidget(recovery)
            calibration = QGroupBox(ko('DUAL ENDPOINT CALIBRATION'))
            calibration_layout = QGridLayout(calibration)
            self.dual_start_calibration = QPushButton(ko('START DUAL CALIBRATION'))
            self.dual_start_calibration.clicked.connect(self._start_dual_calibration)
            self.dual_calibration_state = QLabel(ko('RECALIBRATION_REQUIRED'))
            calibration_layout.addWidget(self.dual_start_calibration, 0, 0, 1, 2)
            calibration_layout.addWidget(self.dual_calibration_state, 0, 2, 1, 2)
            calibration_layout.addWidget(QLabel(ko('Calibration step (motor degree)')), 1, 0, 1, 2)
            self.dual_calibration_step = QComboBox()
            self.dual_calibration_step.addItems(['0.5', '1', '2', '5'])
            calibration_layout.addWidget(self.dual_calibration_step, 1, 2, 1, 2)
            for row, dxl_id in enumerate((3, 4), 2):
                calibration_layout.addWidget(QLabel(ko(f'ID{dxl_id}')), row, 0)
                for column, direction in enumerate((-1.0, 1.0), 1):
                    button = QPushButton(ko(f'ID{dxl_id} {"−" if direction < 0 else "+"} step'))
                    button.setAutoRepeat(False)
                    button.clicked.connect(
                        lambda _checked=False, motor=dxl_id, sign=direction:
                        self._jog_dual_calibration_motor(motor, sign))
                    calibration_layout.addWidget(button, row, column)
                    self.dual_calibration_buttons.append((dxl_id, button))
            self.dual_capture_open = QPushButton(ko('CAPTURE OPEN'))
            self.dual_capture_close = QPushButton(ko('CAPTURE CLOSE'))
            self.dual_capture_open.clicked.connect(
                lambda: self._command_dual_calibration('capture_open'))
            self.dual_capture_close.clicked.connect(
                lambda: self._command_dual_calibration('capture_close'))
            calibration_layout.addWidget(self.dual_capture_open, 4, 0, 1, 2)
            calibration_layout.addWidget(self.dual_capture_close, 4, 2, 1, 2)
            self.dual_capture_label = QLabel(ko('Captured OPEN: — | CLOSE: —'))
            calibration_layout.addWidget(self.dual_capture_label, 5, 0, 1, 4)
            self.dual_validate_calibration = QPushButton(ko('VALIDATE DUAL CALIBRATION'))
            self.dual_save_calibration = QPushButton(ko('SAVE DUAL CALIBRATION'))
            self.dual_validate_calibration.clicked.connect(
                lambda: self._command_dual_calibration('validate'))
            self.dual_save_calibration.clicked.connect(
                lambda: self._command_dual_calibration('save'))
            calibration_layout.addWidget(self.dual_validate_calibration, 6, 0, 1, 2)
            calibration_layout.addWidget(self.dual_save_calibration, 6, 2, 1, 2)
            calibration_layout.addWidget(QLabel(
                ko('Captured endpoint pairs become the only OPEN/CLOSE targets. '
                'Capture itself performs reads only.')), 7, 0, 1, 4)
            bypass = QLabel(
                ko('CALIBRATION JOG: spread protection bypassed\n'
                'Normal OPEN/CLOSE and legacy gripper JOG remain blocked until READY.'))
            bypass.setStyleSheet(FALSE_STYLE)
            bypass.setWordWrap(True)
            calibration_layout.addWidget(bypass, 8, 0, 1, 4)
            layout.addWidget(calibration)
        jog = QGroupBox(ko('GRIPPER JOG'))
        jog_layout = QGridLayout(jog)
        self.spur_minus_5 = self.spur_zero = self.spur_plus_5 = None
        if self.node.selected_tool == 'spur_1motor_gripper':
            left_label, right_label = '누르는 동안 열기 (열림 끝점 방향)', '누르는 동안 닫기 (닫힘 끝점 방향)'
        else:
            left_label, right_label = 'LEFT / +  (OPEN)', 'RIGHT / −  (CLOSE)'
        self.jog_close = QPushButton(ko(left_label))
        self.jog_open = QPushButton(ko(right_label))
        self.gripper_jog_step = QComboBox()
        self.gripper_jog_step.addItems(['5', '10', '25', '50'])
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.gripper_jog_step.clear()
            self.gripper_jog_step.addItem('0.5°')
        self.gripper_busy_label = QLabel(ko('READY'))
        self.gripper_position_label = QLabel(ko('Gripper position: UNKNOWN'))
        self.gripper_feedback_label = QLabel(ko('ID3: UNKNOWN\nID4: UNKNOWN'))
        self.gripper_feedback_label.setWordWrap(True)
        shortcut = QLabel(
            ko('Shortcuts: Left=OPEN jog, Right=CLOSE jog, Space=STOP\n'
            '(disabled while editing a field; key auto-repeat ignored)'))
        if self.node.selected_tool == 'spur_1motor_gripper':
            shortcut.setText(ko('Left: ID5 −0.5° / Right: ID5 +0.5° / Space: STOP'))
        shortcut.setWordWrap(True)
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.jog_close.pressed.connect(lambda: self._start_spur_hold('manual_open'))
            self.jog_open.pressed.connect(lambda: self._start_spur_hold('manual_close'))
            self.jog_close.released.connect(self._release_spur_hold)
            self.jog_open.released.connect(self._release_spur_hold)
        else:
            self.jog_close.clicked.connect(lambda: self._jog_gripper(-1))
            self.jog_open.clicked.connect(lambda: self._jog_gripper(1))
        jog_layout.addWidget(self.jog_close, 0, 0)
        jog_layout.addWidget(self.jog_open, 0, 1)
        step_label = ('Motor step' if self.node.selected_tool == 'spur_1motor_gripper'
                      else 'Step (tick equivalent)')
        jog_layout.addWidget(QLabel(ko(step_label)), 1, 0)
        jog_layout.addWidget(self.gripper_jog_step, 1, 1)
        jog_layout.addWidget(self.gripper_busy_label, 2, 0, 1, 2)
        jog_layout.addWidget(self.gripper_position_label, 3, 0, 1, 2)
        jog_layout.addWidget(self.gripper_feedback_label, 4, 0, 1, 2)
        shortcut_row = 5
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.spur_actual_state = QLabel(ko('ID5: position=UNKNOWN torque=UNKNOWN load=UNKNOWN'))
            jog_layout.addWidget(self.spur_actual_state, 5, 0, 1, 2)
            self.capture_open = QPushButton(ko('SET CURRENT AS OPEN'))
            self.capture_close = QPushButton(ko('SET CURRENT AS CLOSE'))
            self.capture_open.clicked.connect(lambda: self._capture_spur_endpoint('open'))
            self.capture_close.clicked.connect(lambda: self._capture_spur_endpoint('close'))
            jog_layout.addWidget(self.capture_open, 6, 0)
            jog_layout.addWidget(self.capture_close, 6, 1)
            self.captured_endpoints_label = QLabel(ko('Captured OPEN: — | CLOSE: —'))
            jog_layout.addWidget(self.captured_endpoints_label, 7, 0, 1, 2)
            self.validate_calibration = QPushButton(ko('VALIDATE CALIBRATION'))
            self.save_calibration = QPushButton(ko('SAVE CALIBRATION'))
            self.validate_calibration.clicked.connect(self._validate_spur_calibration)
            self.save_calibration.clicked.connect(self._save_spur_calibration)
            jog_layout.addWidget(self.validate_calibration, 8, 0)
            jog_layout.addWidget(self.save_calibration, 8, 1)
            self.motor_minus_half = QPushButton(ko('MOTOR −0.5°'))
            self.motor_plus_half = QPushButton(ko('MOTOR +0.5°'))
            self.motor_minus_one = QPushButton(ko('MOTOR −1°'))
            self.motor_plus_one = QPushButton(ko('MOTOR +1°'))
            for button, degrees in ((self.motor_minus_half, -0.5),
                                    (self.motor_plus_half, 0.5),
                                    (self.motor_minus_one, -1.0),
                                    (self.motor_plus_one, 1.0)):
                button.clicked.connect(
                    lambda _checked=False, delta=degrees: self._jog_spur_motor(delta))
            jog_layout.addWidget(self.motor_minus_half, 9, 0)
            jog_layout.addWidget(self.motor_plus_half, 9, 1)
            jog_layout.addWidget(self.motor_minus_one, 10, 0)
            jog_layout.addWidget(self.motor_plus_one, 10, 1)
            jog_layout.addWidget(QLabel(
                ko('Safety policy: captured OPEN/CLOSE are the command limits; '
                'no hidden endpoint or temporary range is used.')), 11, 0, 1, 2)
            self.spur_mapping = QLabel(ko('Output mapping: waiting for ID5 feedback'))
            self.spur_mapping.setWordWrap(True)
            jog_layout.addWidget(self.spur_mapping, 12, 0, 1, 2)
            self.spur_minus_5 = QPushButton(ko('OUTPUT −5°'))
            self.spur_zero = QPushButton(ko('OUTPUT 0°'))
            self.spur_plus_5 = QPushButton(ko('OUTPUT +5°'))
            self.spur_minus_5.clicked.connect(lambda: self._command_spur_output_deg(-5.0))
            self.spur_zero.clicked.connect(lambda: self._command_spur_output_deg(0.0))
            self.spur_plus_5.clicked.connect(lambda: self._command_spur_output_deg(5.0))
            jog_layout.addWidget(self.spur_minus_5, 13, 0)
            jog_layout.addWidget(self.spur_zero, 13, 1)
            jog_layout.addWidget(self.spur_plus_5, 14, 0, 1, 2)
            shortcut_row = 15
        jog_layout.addWidget(shortcut, shortcut_row, 0, 1, 2)
        layout.addWidget(jog)
        cleaner = QHBoxLayout()
        self.clean_start = QPushButton(ko('CLEANER START'))
        self.clean_stop = QPushButton(ko('CLEANER STOP'))
        self.clean_start.clicked.connect(lambda: self.node.command_cleaner(True))
        self.clean_stop.clicked.connect(lambda: self.node.command_cleaner(False))
        cleaner.addWidget(self.clean_start)
        cleaner.addWidget(self.clean_stop)
        layout.addLayout(cleaner)
        calibration = QHBoxLayout()
        self.read_diag = QPushButton(ko('READ ONLY DIAGNOSTIC'))
        self.start_cal = QPushButton(ko('START CALIBRATION'))
        self.read_diag.clicked.connect(self._read_only_diagnostic)
        self.start_cal.clicked.connect(self._start_calibration)
        calibration.addWidget(self.read_diag)
        calibration.addWidget(self.start_cal)
        layout.addLayout(calibration)
        dual = self.node.selected_tool == 'dual_motor_gripper'
        spur = self.node.selected_tool == 'spur_1motor_gripper'
        cleaner_tool = self.node.selected_tool == 'cleaner'
        for widget in (self.hold_open_button, self.hold_close_button,
                       self.dual_enable, self.dual_disable):
            widget.setVisible(dual)
        for widget in (self.spur_enable, self.spur_disable, self.read_diag, self.start_cal):
            widget.setVisible(spur)
        for widget in (self.open_button, self.close_button, self.tool_stop, jog):
            widget.setVisible(dual or spur or cleaner_tool)
        self.common_enable.setVisible(dual or spur or cleaner_tool)
        self.common_disable.setVisible(dual or spur or cleaner_tool)
        self.clean_start.setVisible(False)
        self.clean_stop.setVisible(False)
        return box

    def _common_buttons(self):
        return (self.open_button, self.close_button, self.tool_stop,
                self.common_enable, self.common_disable,
                self.hold_open_button, self.hold_close_button)

    def _common_torque(self, enabled):
        context = self._reported_tool_context()
        self._append_log(
            f'COMMON TORQUE CLICK: enabled={enabled}, '
            f'CURRENT TOOL={context[0]} FSM={context[1]} IDS={list(context[2])} '
            f'generation={context[3]}')
        logger = getattr(self.node, 'get_logger', None)
        if logger is not None:
            logger().info(
                f'COMMON TORQUE CLICK enabled={enabled} '
                f'tool={self.node.selected_tool}')

        setter = getattr(self.node, 'set_current_tool_enabled', None)
        if setter is not None:
            result = setter(enabled)
            self._append_log(
                f'COMMON TORQUE RESULT: {result}')
            return
        if self.node.selected_tool == 'dual_motor_gripper':
            (self._enable_dual_motors if enabled else self._disable_dual_motors)()
        elif self.node.selected_tool == 'spur_1motor_gripper':
            self.node.command_calibration('manual_enable' if enabled else 'manual_disable')

    def _trace_torque_button(self, event, enabled):
        logger = getattr(self.node, 'get_logger', None)
        if logger is not None:
            button = self.common_enable if enabled else self.common_disable
            logger().info(
                f'COMMON_{"ENABLE" if enabled else "DISABLE"}.{event} '
                f'tool={self.node.selected_tool} enabled={button.isEnabled()} '
                f'down={button.isDown()}')

    def _developer_spur_ready(self):
        sample = self._gripper_samples().get(5, {})
        return bool(
            getattr(self.node, 'developer_direct_mode', False)
            and self.node.selected_tool == 'spur_1motor_gripper'
            and self.node.control_scope == 'END_EFFECTOR_ONLY'
            and self.tool_status.get('tool_type') == 'spur_1motor_gripper'
            and self._status_fresh()
            and not self.tool_status.get('read_only')
            and not getattr(self.node, 'read_only', False)
            and not self.tool_status.get('emergency_stop')
            and not self.tool_status.get('tool_detached')
            and sample.get('online') and sample.get('hardware_error') == 0
            and isinstance(sample.get('position'), int))

    def _developer_spur_command(self, command, delta_deg=0.0):
        if not self._developer_spur_ready():
            self._append_log('개발자 직접 구동 차단: ID5 실시간 안전 상태를 확인하세요')
            return
        if self.node.command_calibration(command, delta_deg=float(delta_deg)):
            detail = f' {delta_deg:+.1f}°' if command == 'manual_step' else ''
            self._append_log(f'개발자 직접 구동 요청: {command}{detail}')

    def _common_hold(self, direction):
        if self.tool_status.get('tool_type') == 'dual_motor_gripper':
            self.dual_hold_context = self._reported_tool_context()
            self._start_dual_hold_jog(direction)
        elif self.tool_status.get('tool_type') == 'spur_1motor_gripper':
            self.spur_hold_context = self._reported_tool_context()
            self._start_spur_hold('manual_open' if direction == 'OPEN' else 'manual_close')

    def _common_release(self):
        if self.tool_status.get('tool_type') == 'dual_motor_gripper':
            self._release_dual_hold_jog()
        elif self.tool_status.get('tool_type') == 'spur_1motor_gripper':
            self._release_spur_hold()

    def _spur_enable_ready(self):
        status = self.tool_status
        sample = self._gripper_samples().get(5, {})
        return bool(self.node.selected_tool == 'spur_1motor_gripper'
                    and status.get('tool_type') == 'spur_1motor_gripper'
                    and self.pending_tool_change is None
                    and self.control_mode == 'MANUAL'
                    and self.node.control_scope in ('END_EFFECTOR_ONLY', 'FULL_ROBOT')
                    and self._status_fresh()
                    and not status.get('read_only') and not getattr(self.node, 'read_only', False)
                    and not status.get('emergency_stop') and not status.get('tool_detached')
                    and sample.get('online') and sample.get('hardware_error') == 0
                    and self.fsm_state in ('STOPPED', 'READY', 'OPEN', 'CLOSED'))

    def _common_action(self, number):
        tool = self.tool_status.get('tool_type')
        command = (('LEFT', 'RIGHT') if tool == 'cleaner'
                   else ('CLOSE', 'OPEN'))[number - 1]
        self.command_current_tool(command)

    def _common_pressed(self, number):
        """Send cleaner direction at physical button/key depression."""
        if self.node.selected_tool == 'cleaner':
            self._common_action(number)

    def _common_clicked(self, number):
        """Keep the gripper buttons' historical release-click behaviour."""
        if self.node.selected_tool != 'cleaner':
            self._common_action(number)

    def _tool_stop_pressed(self):
        if self.tool_status.get('tool_type') == 'cleaner':
            self._stop_tool()

    def _tool_stop_clicked(self):
        if self.tool_status.get('tool_type') != 'cleaner':
            self._stop_tool()

    def _reported_tool_context(self):
        """Return the bridge-reported identity; never consult the ComboBox."""
        status = self.tool_status or {}
        profile = status.get('tool_profile') or {}
        return (status.get('tool_type'), status.get('fsm_class'),
                tuple(int(i) for i in profile.get('actuator_ids', [])),
                status.get('tool_context_generation'))

    def command_current_tool(self, command, *, expected_context=None, **values):
        """Single GUI command gate, bound to the bridge's current generation."""
        command = str(command).strip().upper()
        status = self.tool_status or {}
        tool, fsm_class, ids, generation = self._reported_tool_context()
        self._append_log(
            f'GUI COMMAND CLICK command={command} CURRENT TOOL={tool} '
            f'CURRENT FSM={fsm_class} CURRENT IDS={list(ids)} generation={generation}')
        reason = ''
        expected_fsms = {
            'dual_motor_gripper': 'DualMotorGripperFSM',
            'spur_1motor_gripper': 'SingleMotorGripperFSM',
            'cleaner': 'CleanerFSM'}
        supported = {
            'dual_motor_gripper': {'OPEN', 'CLOSE', 'STOP', 'HOLD',
                                   'JOG_OPEN', 'JOG_CLOSE', 'JOG_RELATIVE'},
            'spur_1motor_gripper': {'OPEN', 'CLOSE', 'STOP', 'HOLD'},
            'cleaner': {'LEFT', 'RIGHT', 'STOP'},
        }
        if self.pending_tool_change is not None:
            reason = 'tool context switch is pending'
        elif tool not in expected_fsms:
            reason = 'bridge has not reported a supported runtime tool'
        elif (self.node.control_scope == 'END_EFFECTOR_ONLY'
              and (tool != getattr(self.node, 'runtime_tool_type', None)
                   or generation is None
                   or generation != getattr(self.node, 'runtime_tool_generation', None)
                   or fsm_class != expected_fsms[tool]
                   or tuple(self.node.actuator_ids) != ids)):
            reason = 'reported runtime context, FSM, IDs, or generation do not match'
        elif (self.node.control_scope == 'END_EFFECTOR_ONLY'
              and status.get('tool_preparation_state') != 'READY'):
            reason = 'runtime tool preparation is not complete'
        elif (self.node.control_scope == 'END_EFFECTOR_ONLY'
              and status.get('tool_preparation_generation', generation)
              != generation):
            reason = 'runtime tool preparation belongs to another generation'
        elif expected_context is not None and tuple(expected_context) != (
                tool, fsm_class, ids, generation):
            reason = 'command originated from a stale tool generation'
        elif command not in supported[tool]:
            reason = f'{command} is unsupported by {tool}'
        elif command != 'STOP' and self.node.control_scope == 'END_EFFECTOR_ONLY':
            samples = {int(s['id']): s for s in status.get('actuators', [])
                       if isinstance(s, dict) and s.get('id') is not None}
            if (status.get('tool_preparation_state') != 'READY'
                    or status.get('control_mode') != 'MANUAL'
                    or status.get('fsm_class') != expected_fsms[tool]
                    or not status.get('tool_enable_allowed')
                    or not status.get('motion_allowed')
                    or status.get('tool_torque_state') != 'ON'
                    or status.get('fsm_state') not in (
                        'READY', 'OPEN', 'CLOSED', 'CLEANING')
                    or set(samples) != set(ids)
                    or any(not samples[i].get('online')
                           or samples[i].get('hardware_error') != 0
                           or samples[i].get('torque_state') != 'ON'
                           for i in ids)):
                reason = 'runtime FSM/motion/actuator safety state is not ready'
        if reason:
            self._append_log(f'COMMAND RESULT=REJECTED reason={reason}')
            logger = getattr(self.node, 'get_logger', None)
            if logger is not None:
                logger().warn(f'GUI COMMAND REJECTED command={command}: {reason}')
            return False
        result = bool(self.node.command_tool_fsm(command, **values))
        detail = (f'COMMAND ROUTED TO {tool}/{fsm_class} IDs={list(ids)} '
                  f'generation={generation} result={"PUBLISHED" if result else "REJECTED"}')
        self._append_log(detail)
        logger = getattr(self.node, 'get_logger', None)
        if logger is not None:
            logger().info(f'GUI {detail} command={command}')
        return result

    def _refresh_common_buttons(self):
        self._refresh_legacy_common_buttons()
        cleaner = self.node.selected_tool == 'cleaner'
        self.close_button.setText('좌회전' if cleaner else '닫기')
        self.open_button.setText('우회전' if cleaner else '열기')
        self.close_button.setVisible(True)
        self.open_button.setVisible(True)
        self.tool_stop.setVisible(True)
        self.common_enable.setVisible(True)
        self.common_disable.setVisible(True)
        self.hold_open_button.setVisible(not cleaner and self.hold_open_button.isVisible())
        self.hold_close_button.setVisible(not cleaner and self.hold_close_button.isVisible())
        self.clean_start.hide()
        self.clean_stop.hide()
        if cleaner:
            direct = bool(
                getattr(self.node, 'developer_direct_mode', False)
                and self.node.control_scope == 'END_EFFECTOR_ONLY'
                and not self.tool_status.get('read_only')
                and not self.tool_status.get('emergency_stop')
                and not self.tool_status.get('tool_detached'))
            ready = (self.control_mode == 'MANUAL'
                     and self._tool_motion_ready()
                     and bool(self.profile.get('actuator_ids'))
                     and bool(self.tool_status.get('actuators_discovered'))
                     and not self.pending_tool_change)
            self.close_button.setEnabled(ready)
            self.open_button.setEnabled(ready)
            self.tool_stop.setEnabled(
                direct or not self.tool_status.get('read_only'))
        # The integrated operator panel exposes exactly two motion buttons.
        # Bench calibration/recovery widgets remain available in bench scope.
        if self.node.control_scope == 'FULL_ROBOT':
            allowed = {self.close_button, self.open_button, self.tool_stop,
                       self.common_enable, self.common_disable, self.read_diag}
            for button in (self.tool_control_box.findChildren(QPushButton)
                           + self.tool_details_box.findChildren(QPushButton)):
                button.setVisible(button in allowed)

    def _refresh_legacy_common_buttons(self):
        tool = self.node.selected_tool
        for widget in self._common_buttons():
            widget.setVisible(tool in (
                'dual_motor_gripper', 'spur_1motor_gripper', 'cleaner'))
        self.hold_open_button.setVisible(tool == 'dual_motor_gripper')
        self.hold_close_button.setVisible(tool == 'dual_motor_gripper')
        if tool != 'spur_1motor_gripper':
            # The existing dual gates above are authoritative, unchanged.
            return
        sample = self._gripper_samples().get(5, {})
        ready = self._spur_enable_ready()
        torque_on = sample.get('torque_state') == 'ON'
        # Torque-off readiness must never depend on motion_allowed or calibration-session state.
        if self.node.control_scope != 'END_EFFECTOR_ONLY':
            self.common_enable.setEnabled(ready and not torque_on)
        motion = (ready and torque_on
                  and self.fsm_state in ('READY', 'OPEN', 'CLOSED'))
        if self.node.control_scope == 'END_EFFECTOR_ONLY':
            motion = motion and self._tool_motion_ready()
        for widget in (self.open_button, self.close_button,
                       self.hold_open_button, self.hold_close_button):
            widget.setEnabled(motion)
        writable = (not self.tool_status.get('read_only')
                    and not getattr(self.node, 'read_only', False))
        if self.node.control_scope != 'END_EFFECTOR_ONLY':
            self.common_disable.setEnabled(bool(sample.get('online') and writable and torque_on))
        self.tool_stop.setEnabled(bool(sample.get('online') and writable))
        self._log_spur_enable_diagnostics()

    def _log_spur_enable_diagnostics(self):
        status = self.tool_status
        sample = self._gripper_samples().get(5, {})
        gates = {
            'selected_spur': self.node.selected_tool == 'spur_1motor_gripper',
            'reported_spur': status.get('tool_type') == 'spur_1motor_gripper',
            'no_pending_change': self.pending_tool_change is None,
            'manual': self.control_mode == 'MANUAL',
            'scope': self.node.control_scope == 'END_EFFECTOR_ONLY',
            'fresh': self._status_fresh(),
            'bridge_writable': not bool(status.get('read_only')),
            'gui_writable': not getattr(self.node, 'read_only', False),
            'no_estop': not bool(status.get('emergency_stop')),
            'attached': not bool(status.get('tool_detached')),
            'online': bool(sample.get('online')),
            'hw_error_zero': sample.get('hardware_error') == 0,
            'fsm_ready': self.fsm_state in ('READY', 'OPEN', 'CLOSED'),
            'torque_not_on': sample.get('torque_state') != 'ON',
        }
        parents = []
        parent = self.common_enable.parentWidget()
        while parent is not None:
            parents.append({'class': type(parent).__name__, 'enabled': parent.isEnabled()})
            parent = parent.parentWidget()
        record = {
            'selected_tool': self.node.selected_tool, 'control_mode': self.control_mode,
            'node_control_mode': getattr(self.node, 'control_mode', None), 'fsm_state': self.fsm_state,
            'id5': sample, 'mode_combo': self.mode_combo.currentData(),
            'tool_status': {k: status.get(k) for k in ('tool_type', 'control_mode',
                'fsm_state', 'tool_enable_allowed', 'motion_allowed', 'profile_valid', 'calibrated')},
            'enable_conditions': gates, 'setEnabled_argument': all(gates.values()),
            'button_isEnabled': self.common_enable.isEnabled(), 'parents': parents,
        }
        encoded = json.dumps(record, sort_keys=True, ensure_ascii=False)
        if encoded != getattr(self, '_last_enable_diagnostics', None):
            self._last_enable_diagnostics = encoded
            logger = getattr(self.node, 'get_logger', None)
            if logger:
                logger().info('SPUR_ENABLE_DIAGNOSTICS ' + encoded)

    def _profile_summary(self):
        keys = ('calibrated', 'actuator_ids', 'safe_min_tick', 'safe_max_tick',
                'open_tick', 'close_tick', 'profile_velocity',
                'profile_acceleration')
        return '\n'.join(f'{key}: {self.profile.get(key)}' for key in keys)

    def _rebuild_tool_control_group(self):
        old = getattr(self, 'tool_control_box', None)
        # Clear references to tool-specific widgets before deleting the old panel.
        # Qt deletes the underlying C++ objects with the panel, so stale Python
        # references must not survive a runtime tool switch.
        self._clear_tool_specific_widget_refs()
        if old is not None:
            for widget in self._common_buttons():
                widget.setParent(None)
            self.right_layout.removeWidget(old)
            old.setEnabled(False)
            old.hide()
            old.deleteLater()
            details = self.tool_details_box
            self.details_layout.removeWidget(details)
            details.hide()
            details.deleteLater()
        self.tool_control_box = self._tool_control_group()
        self.right_layout.addWidget(self.tool_control_box)
        self.details_layout.insertWidget(3, self.tool_details_box)

    def _connect_signals(self):
        self.signals.joint_states.connect(self._update_joints)
        self.signals.tool_status.connect(self._update_tool_status)
        self.signals.fsm_state.connect(self._update_mission_fsm)
        self.signals.control_mode.connect(self._update_mode)
        self.signals.arm_status.connect(
            lambda value: self.status_labels['arm_status'].setText(ko(value)))
        self.signals.contact_status.connect(
            lambda value: self._set_bool(self.status_labels['contact'], value))
        self.signals.log.connect(self._append_log)
        self.signals.gripper_state.connect(self._update_gripper_state)

    def _set_bool(self, label, value):
        label.setText(ko('TRUE' if value else 'FALSE'))
        label.setStyleSheet(TRUE_STYLE if value else FALSE_STYLE)

    def _status_fresh(self):
        """True only while `/tool/status` is recent enough to act on."""
        return self._clock() - self.last_status_time < STATUS_FRESH_S

    def _refresh_connection(self):
        connected = self._status_fresh()
        self._set_bool(self.status_labels['connection'], connected)
        if not connected:
            self._set_bool(self.status_labels['motion_allowed'], False)
        self._expire_tool_change(connected)
        self._refresh_buttons()

    def _expire_tool_change(self, connected):
        """Release a request the bridge never answered.

        This clears the GUI-side latch only.  Nothing here grants motion: the
        buttons are recomputed from live connection/ready/`motion_allowed`
        state right after, exactly as they are while no request is pending.
        """
        if self.pending_tool_change is None:
            return
        started = self.pending_tool_change_started
        if started is None:
            return
        elapsed = self._clock() - started
        # A live bridge stops publishing status while it performs the switch
        # (both run on its single executor), so silence right after a healthy
        # request is not proof of a lost link — only the full deadline is.
        # Silence on a link that was already dead when the operator clicked is
        # reported at once instead of making them wait it out.
        if (not connected and not self.pending_tool_change_link_ok
                and elapsed >= self.tool_change_disconnect_grace_s):
            self._finish_tool_change(
                'DISCONNECTED',
                f'{elapsed:.1f}초 동안 제어 노드 상태를 받지 못했습니다')
        elif elapsed >= self.tool_change_timeout_s:
            self._finish_tool_change(
                'TIMEOUT', f'{self.tool_change_timeout_s:.1f}초 안에 응답이 없습니다')

    def _finish_tool_change(self, state, detail=''):
        """Single exit for every request outcome (success, error, give-up)."""
        if state not in TOOL_CHANGE_STATES:
            raise ValueError(f'unknown tool change state: {state}')
        requested = self.pending_tool_change
        self.pending_tool_change = None
        self.pending_tool_change_started = None
        self.tool_change_state = state
        self.tool_change_detail = detail
        if requested is not None and state != 'DONE':
            # Never leave the combo advertising a tool the bridge is not on.
            active = self.tool_status.get('tool_type', self.node.selected_tool)
            index = self.tool_combo.findData(active)
            if index >= 0:
                self.tool_combo.setCurrentIndex(index)
        if requested is not None:
            self._append_log(ko(self._tool_change_message()))
        # Buttons are deliberately left to the caller: on the success path the
        # tool panel is rebuilt right after this, and refreshing the old panel
        # for the new tool touches widgets that do not exist yet.

    def _tool_change_message(self):
        tool = ko(str(self.tool_change_requested))
        detail = self.tool_change_detail
        if self.tool_change_state == 'PENDING':
            return f'도구 준비 중: {tool}'
        if self.tool_change_state == 'DONE':
            status = self.tool_status
            return (f'도구 준비 완료: {tool} · FSM {status.get("fsm_state", "READY")} '
                    f'· Torque {status.get("tool_torque_state", "OFF")} '
                    '· 활성화 가능')
        if self.tool_change_state == 'REJECTED':
            return f'도구 변경 거부: {tool} — {detail}'
        if self.tool_change_state == 'TIMEOUT':
            return f'도구 변경 응답 없음: {tool} — {detail}. 요청을 해제합니다'
        if self.tool_change_state == 'DISCONNECTED':
            return (f'도구 변경 중단: {tool} — {detail}. 제어 노드 연결을 확인하세요')
        if self.tool_change_state == 'CANCELED':
            return (f'도구 변경 요청 취소: {tool} — 화면 대기만 취소했고 '
                    '제어 노드에 이미 전달된 요청은 되돌리지 않습니다')
        return '도구 변경 요청 없음'

    def _refresh_tool_change_labels(self):
        """Keep requested/active/blocked visible; display only, no commands."""
        text = self._tool_change_message()
        if self.pending_tool_change is not None:
            elapsed = self._clock() - (self.pending_tool_change_started or self._clock())
            text = f'{text} {elapsed:.1f}초'
        label = self.status_labels.get('tool_change')
        if label is not None:
            label.setText(ko(text))
            label.setStyleSheet(
                FALSE_STYLE if self.tool_change_state in (
                    'REJECTED', 'TIMEOUT', 'DISCONNECTED') else '')
        blocked = self.status_labels.get('tool_block')
        if blocked is not None:
            reason = self._tool_block_reason()
            blocked.setText(ko(reason or '없음'))
            blocked.setStyleSheet(FALSE_STYLE if reason else TRUE_STYLE)
        cancel = getattr(self, 'cancel_tool_change', None)
        if cancel is not None:
            cancel.setEnabled(self.pending_tool_change is not None)
        request = getattr(self, 'request_tool_change', None)
        if request is not None:
            waiting = self.pending_tool_change is not None
            request.setText(ko('처리 중…') if waiting else ko('REQUEST TOOL CHANGE'))
            request.setEnabled(not waiting)

    def _tool_block_reason(self):
        """Why tool motion is unavailable right now, in the operator's terms.

        Written from GUI-visible state only so an untranslated bridge string is
        never rendered into this Korean panel.
        """
        if not self._status_fresh():
            return '제어 노드 상태 수신 끊김'
        if self.pending_tool_change is not None:
            return '도구 준비 중…'
        status = self.tool_status
        preparation = status.get('tool_preparation_state')
        if preparation == 'PREPARING':
            return '도구 준비 중…'
        if preparation == 'FAILED':
            detail = (status.get('tool_preparation_error')
                      or status.get('tool_enable_reason') or '안전 조건 미충족')
            return f'도구 준비 실패: {detail}'
        if status.get('emergency_stop'):
            return '비상 정지 래치'
        if status.get('tool_detached'):
            return '도구 분리 래치'
        if status.get('read_only') or getattr(self.node, 'read_only', False):
            return '읽기 전용 모드'
        if not status.get('profile_valid'):
            return '도구 설정이 유효하지 않음'
        if not status.get('actuators_discovered'):
            detected = (status.get('automatic_detection') or {}).get(
                'present_ids') or []
            expected = (status.get('tool_profile') or {}).get(
                'actuator_ids') or []
            if detected and expected:
                return ('선택 도구와 연결 모터 번호가 다름 '
                        f'(감지: {", ".join(map(str, detected))}, '
                        f'기대: {", ".join(map(str, expected))})')
            return '도구 모터 미감지'
        if not status.get('calibrated'):
            return '보정 필요'
        if self.control_mode != 'MANUAL':
            return '수동 제어 권한 없음'
        if not status.get('tool_enable_allowed'):
            return (status.get('tool_enable_reason')
                    or '활성화 준비 조건을 만족하지 못했습니다')
        if not status.get('motion_allowed'):
            if status.get('tool_torque_state') == 'OFF':
                return '토크 꺼짐 · 활성화 버튼 대기'
            return '토크 미인가 또는 도구 준비 안 됨'
        return ''

    @staticmethod
    def _runtime_tool_prepared(status):
        """Accept a tool-change response only after a real OFF/READY readback."""
        if (status.get('tool_preparation_state') != 'READY'
                or status.get('tool_preparation_generation',
                              status.get('tool_context_generation'))
                != status.get('tool_context_generation')
                or not status.get('profile_valid')
                or not status.get('tool_enable_allowed')
                or status.get('tool_torque_state') != 'OFF'
                or status.get('fsm_state') != 'READY'
                or status.get('control_mode') != 'MANUAL'
                or status.get('emergency_stop')
                or status.get('tool_detached')
                or status.get('physical_tool_detached')):
            return False
        profile = status.get('tool_profile') or {}
        ids = [int(value) for value in profile.get('actuator_ids', [])]
        samples = {int(sample.get('id', -1)): sample
                   for sample in status.get('actuators', [])
                   if isinstance(sample, dict) and sample.get('id') is not None}
        if not ids or set(samples) != set(ids):
            return False
        required_modes = profile.get('required_operating_modes') or {}
        expected_models = profile.get('endpoint_calibration_models') or {}
        for dxl_id in ids:
            sample = samples[dxl_id]
            if (not sample.get('online') or sample.get('hardware_error') != 0
                    or sample.get('torque_state') != 'OFF'
                    or sample.get('position') is None):
                return False
            required_mode = required_modes.get(
                dxl_id, required_modes.get(str(dxl_id)))
            if required_mode is None and profile.get('backend') == 'cleaner':
                required_mode = 1
            if (required_mode is not None
                    and sample.get('operating_mode') != required_mode):
                return False
            expected_model = expected_models.get(
                dxl_id, expected_models.get(str(dxl_id)))
            if expected_model is not None and sample.get('model') != expected_model:
                return False
            if (profile.get('backend') == 'cleaner'
                    and sample.get('goal_velocity') != 0):
                return False
        return True

    def _cancel_tool_change(self):
        if self.pending_tool_change is None:
            return
        self._finish_tool_change('CANCELED')
        self._refresh_buttons()

    def _reset_runtime_tool_gui_state(self):
        """Release old-tool GUI state before binding the new runtime context."""
        self.spur_hold_timer.stop()
        self.spur_hold_command = None
        self.spur_hold_context = None
        self.dual_key_jog_timer.stop()
        self.dual_key_jog_direction = 0
        self.dual_hold_jog_active = False
        self.dual_hold_jog_direction = None
        self.dual_hold_context = None
        self.gripper_busy = self.node.gripper_busy = False
        self.node.last_gripper_goal = None
        self.node.tool_context_generation = (
            getattr(self.node, 'tool_context_generation', 0) + 1)
        self.gripper_target_ticks = {}
        self.spur_endpoints = {}
        self.spur_zero_tick = None
        self.spur_torque_enabled = False
        self.spur_torque_state = 'UNKNOWN'
        for button in self._common_buttons():
            button.setEnabled(False)
        for key in ('dual_online', 'dual_positions', 'dual_torque',
                    'dual_hw_error', 'dual_sync'):
            self.status_labels[key].setText('—')

    def _adopt_runtime_tool_context(self, status):
        """Bind GUI routing/profile/widgets to the bridge's active tool status."""
        previous_tool = self.node.selected_tool
        reported_tool = status.get('tool_type')
        runtime_profile = status.get('tool_profile')
        if isinstance(runtime_profile, dict):
            self.profile = runtime_profile
        valid_tool = reported_tool in (
            'spur_1motor_gripper', 'dual_motor_gripper', 'cleaner')
        generation = status.get('tool_context_generation')
        context_changed = (generation is not None
                           and generation != self._runtime_context_generation)
        if self.pending_tool_change:
            if (reported_tool == self.pending_tool_change
                    and (generation is None or generation != getattr(
                        self, '_pending_tool_generation', None))):
                self.node.selected_tool = reported_tool
                if self._runtime_tool_prepared(status):
                    self._finish_tool_change('DONE')
                elif (status.get('tool_preparation_state') == 'FAILED'
                      or (status.get('tool_change') or {}).get('error')):
                    detail = (status.get('tool_preparation_error')
                              or (status.get('tool_change') or {}).get('error')
                              or status.get('tool_enable_reason')
                              or 'Torque-OFF READY 검증 실패')
                    self._finish_tool_change('REJECTED', str(detail))
            elif ((status.get('tool_change') or {}).get('error')
                  or status.get('tool_preparation_state') == 'FAILED'):
                self._finish_tool_change(
                    'REJECTED', str(
                        status.get('tool_preparation_error')
                        or (status.get('tool_change') or {}).get('error')
                        or status.get('tool_enable_reason')
                        or '도구 준비 실패'))
        if valid_tool and reported_tool != self.node.selected_tool:
            self.node.selected_tool = reported_tool

        self.node.tool_profile = self.profile
        self.node.actuator_ids = list(self.profile.get('actuator_ids', []))
        tool_changed = self.node.selected_tool != previous_tool
        self.node.runtime_tool_generation = generation
        self.node.runtime_tool_type = reported_tool if valid_tool else None
        self.node.runtime_fsm_class = status.get('fsm_class') if valid_tool else None
        self.node.runtime_preparation_state = status.get(
            'tool_preparation_state', 'UNKNOWN')
        self.node.runtime_preparation_generation = status.get(
            'tool_preparation_generation', generation)
        self._runtime_context_generation = generation
        if tool_changed or context_changed:
            self._reset_runtime_tool_gui_state()
            index = self.tool_combo.findData(self.node.selected_tool)
            if index >= 0:
                self.tool_combo.setCurrentIndex(index)
            self._rebuild_tool_control_group()
            logger = getattr(self.node, 'get_logger', None)
            if logger:
                logger().info(
                    f'GUI TOOL CONTEXT tool={self.node.selected_tool} '
                    f'ids={self.node.actuator_ids} generation={generation} '
                    f'fsm={status.get("fsm_class")}')
        return tool_changed

    def _update_tool_status(self, status):
        self.tool_status = status
        self.last_status_time = self._clock()
        previous_tool = self.node.selected_tool
        self._adopt_runtime_tool_context(status)
        # Mode-status is event-only; recover the current mode from periodic
        # runtime tool context for every hot-swappable end effector.
        mode = status.get('control_mode')
        if mode in ('MANUAL', 'FSM'):
            self.control_mode = self.node.control_mode = mode
            self.status_labels['mode'].setText(ko(mode))
        self._update_status_panel(status)
        self.profile_text.setText(ko(self._profile_summary()))
        self.status_labels['tool_type'].setText(ko(status.get('tool_type', 'UNKNOWN')))
        if (self.node.selected_tool != 'cleaner' or previous_tool != 'cleaner'
                or status.get('fsm_state') in ('READY', 'CLEANING', 'STOPPED',
                                               'CALIBRATION_REQUIRED')):
            self._update_fsm(status.get('fsm_state') or 'UNKNOWN')
        self._set_bool(
            self.status_labels['u2d2'], bool(status.get('u2d2_connected')))
        for key in ('profile_valid', 'actuators_discovered', 'motion_allowed'):
            self._set_bool(self.status_labels[key], bool(status.get(key)))
        self._set_bool(
            self.status_labels['calibration_valid'],
            bool(status.get('calibrated')) and (
                self.node.selected_tool != 'dual_motor_gripper'
                or bool(status.get('endpoint_calibration_verified'))))
        estop = bool(status.get('emergency_stop'))
        self.estop_state.setText(ko(f'E-STOP: {str(estop).upper()}'))
        self.estop_state.setStyleSheet(FALSE_STYLE if estop else TRUE_STYLE)
        self.restart_program.setEnabled(estop)
        self._rebuild_diagnostics(status.get('actuators', []))
        self._update_gripper_feedback()
        self._refresh_buttons()
        samples = self._gripper_samples()
        if self.node.selected_tool == 'dual_motor_gripper':
            self.status_labels['dual_online'].setText(
                ko(' / '.join(f'ID{i}={bool(samples.get(i, {}).get("online"))}'
                           for i in (3, 4))))
            self.status_labels['dual_positions'].setText(
                ko(' / '.join(f'ID{i}={samples.get(i, {}).get("position")}'
                           for i in (3, 4))))
            self.status_labels['dual_torque'].setText(
                ko(' / '.join(f'ID{i}={samples.get(i, {}).get("torque_state", "UNKNOWN")}'
                           for i in (3, 4))))
            self.status_labels['dual_hw_error'].setText(
                ko(' / '.join(f'ID{i}={samples.get(i, {}).get("hardware_error")}'
                           for i in (3, 4))))
            sync = status.get('synchronization') or {}
            self.status_labels['dual_sync'].setText(
                ko(f'{sync.get("state", "UNKNOWN")} spread={sync.get("spread")} '
                f'(limit={sync.get("limit", 0.05)})'))

    def _update_joints(self, values):
        for joint, sample in values.items():
            if joint in self.arm_position_labels and sample['position'] is not None:
                self.seen_arm_joints.add(joint)
                self.arm_position_labels[joint].setText(ko(f'{sample["position"]:.4f}'))
                self.arm_targets[joint].setValue(float(sample['position']))
        self._refresh_buttons()
        self._rebuild_diagnostics(self.tool_status.get('actuators', []), values)

    def _update_mission_fsm(self, state):
        self.arm_fsm_state = state
        self.status_labels['arm_fsm'].setText(ko(state))

    def _update_fsm(self, state):
        self.fsm_state = state
        self.status_labels['fsm'].setText(ko(state))

    def _update_mode(self, mode):
        self.control_mode = mode
        self.status_labels['mode'].setText(ko(mode))
        self._refresh_buttons()

    def _refresh_buttons(self):
        self._refresh_tool_change_labels()
        self.tool_control_box.setEnabled(self.pending_tool_change is None)
        # The same existing parent gate also covers the relocated widgets.
        self.tool_details_box.setEnabled(self.tool_control_box.isEnabled())
        manual = self.control_mode == 'MANUAL'
        end_effector_only = self.node.control_scope == 'END_EFFECTOR_ONLY'
        for widget in self.arm_buttons:
            widget.setEnabled(manual and not end_effector_only)
        if not self.mock_mode:
            for joint, widgets in self.arm_widgets.items():
                for widget in widgets:
                    widget.setEnabled(
                        manual and not end_effector_only
                        and joint in self.seen_arm_joints)
        profile_ok = bool(self.tool_status.get('profile_valid'))
        motion = self._tool_motion_ready()
        gripper = self.node.selected_tool.endswith('gripper')
        spur = self.node.selected_tool == 'spur_1motor_gripper'
        dual = self.node.selected_tool == 'dual_motor_gripper'
        calibrated = bool(self.tool_status.get('calibrated')) or self.mock_mode
        captured = (set(self.spur_endpoints) == {'open', 'close'}
                    and self.spur_endpoints['open'] != self.spur_endpoints['close'])
        dual_calibration = self.tool_status.get('dual_calibration') or {}
        dual_ready = dual_calibration.get('state') == 'READY'
        fsm_commandable = self.fsm_state in ('READY', 'OPEN', 'CLOSED')
        preset_ready = (manual and gripper and profile_ok and motion
                        and calibrated and not self.gripper_busy
                        and (not spur or fsm_commandable)
                        and (not dual or (dual_ready and fsm_commandable)))
        # Captures are only a candidate.  They never silently turn an
        # uncalibrated live profile into a normal-motion profile.
        self.open_button.setEnabled(preset_ready)
        self.close_button.setEnabled(preset_ready)
        calibration = self.tool_status.get('calibration') or {}
        # spur_enable/dual_enable alias the common buttons. Only the active
        # tool may update them: a transient disable cancels a pressed click.
        dual_samples = self._gripper_samples()
        dual_online = all(dual_samples.get(dxl_id, {}).get('online')
                          for dxl_id in (3, 4))
        dual_healthy = all(dual_samples.get(dxl_id, {}).get('hardware_error') == 0
                           for dxl_id in (3, 4))
        dual_torque_on = all(dual_samples.get(dxl_id, {}).get('torque_state') == 'ON'
                             for dxl_id in (3, 4))
        dual_profile_verified = bool(
            self.tool_status.get('endpoint_calibration_verified'))
        dual_synchronized = ((self.tool_status.get('synchronization') or {}).get(
            'state') == 'SYNCHRONIZED')
        if dual:
            self.open_button.setEnabled(
                preset_ready and dual_online and dual_healthy
                and dual_torque_on and dual_profile_verified
                and dual_synchronized)
            self.close_button.setEnabled(self.open_button.isEnabled())
        hold_jog_ready = dual and self.open_button.isEnabled()
        self.hold_open_button.setVisible(dual)
        self.hold_close_button.setVisible(dual)
        self.hold_open_button.setEnabled(
            (self.dual_hold_jog_active
             and self.dual_hold_jog_direction == 'OPEN')
            or (hold_jog_ready and not self.dual_hold_jog_active))
        self.hold_close_button.setEnabled(
            (self.dual_hold_jog_active
             and self.dual_hold_jog_direction == 'CLOSE')
            or (hold_jog_ready and not self.dual_hold_jog_active))
        self.tool_stop.setEnabled(
            (spur and not bool(self.tool_status.get('read_only'))
             and bool(self.tool_status.get('online')))
            or (dual and not bool(self.tool_status.get('read_only'))))
        if dual and not end_effector_only:
            self.dual_enable.setEnabled(
                manual and dual_online and dual_healthy
                and not dual_torque_on and not bool(self.tool_status.get('read_only')))
            self.dual_disable.setEnabled(
                dual_online and not bool(self.tool_status.get('read_only')))
        recovery_base = (
            dual and manual and self.node.control_scope == 'END_EFFECTOR_ONLY'
            and not bool(self.tool_status.get('read_only'))
            and not bool(self.tool_status.get('emergency_stop'))
            and not bool(self.tool_status.get('tool_detached')))
        for dxl_id, button in self.dual_recovery_buttons:
            sample = dual_samples.get(dxl_id, {})
            button.setEnabled(
                recovery_base and bool(sample.get('online'))
                and sample.get('hardware_error') == 0
                and sample.get('torque_state') == 'ON')
        dual_calibration_active = bool(dual_calibration.get('active'))
        dual_calibration_capture_ready = (
            dual and manual and end_effector_only
            and not bool(self.tool_status.get('read_only'))
            and not bool(self.tool_status.get('emergency_stop'))
            and not bool(self.tool_status.get('tool_detached'))
            and dual_online and dual_healthy)
        dual_calibration_jog_ready = (
            dual_calibration_capture_ready and dual_torque_on)
        if self.dual_calibration_state is not None:
            self.dual_calibration_state.setText(
                ko(dual_calibration.get('state', 'RECALIBRATION_REQUIRED')))
        if self.dual_capture_label is not None:
            captures = dual_calibration.get('captures') or {}
            self.dual_capture_label.setText(
                ko(f'Captured OPEN: {captures.get("open", "—")} | '
                f'CLOSE: {captures.get("close", "—")}'))
        if self.dual_start_calibration is not None:
            self.dual_start_calibration.setEnabled(
                dual and manual and end_effector_only
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached'))
                and not dual_calibration_active)
        for _dxl_id, button in self.dual_calibration_buttons:
            button.setEnabled(dual_calibration_active and dual_calibration_jog_ready)
        if self.dual_calibration_step is not None:
            self.dual_calibration_step.setEnabled(
                dual_calibration_active and dual_calibration_jog_ready)
        if self.dual_capture_open is not None:
            self.dual_capture_open.setEnabled(
                dual_calibration_active and dual_calibration_capture_ready)
            self.dual_capture_close.setEnabled(
                dual_calibration_active and dual_calibration_capture_ready)
            captured_pairs = dual_calibration.get('captures') or {}
            both_pairs = set(captured_pairs) == {'open', 'close'}
            self.dual_validate_calibration.setEnabled(
                dual_calibration_active and dual_calibration_capture_ready and both_pairs)
            self.dual_save_calibration.setEnabled(
                dual_calibration_active and bool(dual_calibration.get('validated')))
        jog_ready = (manual and not self.gripper_busy
                     and self.node.control_scope == 'END_EFFECTOR_ONLY'
                     and self.node.selected_tool in (
                         'dual_motor_gripper', 'spur_1motor_gripper')
                     and self._tool_motion_ready()
                     and self._gripper_positions_synchronized()
                     and (not dual or dual_ready))
        # When the measured position is outside the temporary range, expose
        # only the inward recovery direction.  This prevents a disabled
        # direction from being retried by either a click or a key shortcut.
        spur_open_allowed = True   # LEFT / '-' decreases ticks (opens)
        spur_close_allowed = True  # RIGHT / '+' increases ticks (closes)
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            current = sample.get('position')
            if current is not None:
                if current > self.temporary_jog_safe_max:
                    spur_close_allowed = False
                elif current < self.temporary_jog_safe_min:
                    spur_open_allowed = False
        self.jog_close.setEnabled(jog_ready and spur_open_allowed)
        self.jog_open.setEnabled(jog_ready and spur_close_allowed)
        self.gripper_jog_step.setEnabled(not self.gripper_busy)
        cleaner = self.node.selected_tool == 'cleaner'
        configured = bool(self.tool_status.get('actuators_discovered'))
        current_tool_torque_on = self.tool_status.get('tool_torque_state') == 'ON'
        current_tool_enable_ready = bool(
            self.tool_status.get('tool_enable_allowed')
            and self._status_fresh()
            and self.control_mode == 'MANUAL'
            and not bool(self.tool_status.get('read_only'))
            and not getattr(self.node, 'read_only', False)
            and not bool(self.tool_status.get('emergency_stop'))
            and not bool(self.tool_status.get('tool_detached'))
            and self.tool_status.get('tool_type') == self.node.selected_tool)
        cleaner_direct = bool(
            cleaner and getattr(self.node, 'developer_direct_mode', False)
            and self.node.control_scope == 'END_EFFECTOR_ONLY'
            and not bool(self.tool_status.get('read_only'))
            and not bool(self.tool_status.get('emergency_stop'))
            and not bool(self.tool_status.get('tool_detached'))
            and configured)
        self.clean_start.setEnabled(
            cleaner_direct or (manual and cleaner and profile_ok
                               and motion and configured))
        self.clean_stop.setEnabled(
            cleaner_direct or (manual and cleaner and profile_ok and motion))
        if cleaner and not end_effector_only:
            self.common_enable.setEnabled(
                current_tool_enable_ready and not current_tool_torque_on)
            self.common_disable.setEnabled(
                current_tool_enable_ready and current_tool_torque_on)
        for widget in (self.spur_minus_5, self.spur_zero, self.spur_plus_5):
            if widget is not None:
                widget.setEnabled(False)
        if spur:
            calibration_ready = (manual and calibration.get('active', False)
                                 and self.spur_torque_state == 'ON'
                                 and calibration.get('enabled', False)
                                 and bool(self.tool_status.get('calibration_jog_enabled'))
                                 and self._gripper_positions_synchronized())
            for widget in (self.motor_minus_half, self.motor_plus_half,
                           self.motor_minus_one, self.motor_plus_one,
                           self.capture_open, self.capture_close):
                widget.setEnabled(calibration_ready)
            manual_ready = self._spur_manual_ready()
            opening = self.profile.get('open_tick')
            closing = self.profile.get('close_tick')
            if isinstance(opening, (int, float)) and isinstance(closing, (int, float)):
                self.jog_close.setText(f'누르는 동안 열기 (틱 {"감소" if opening < closing else "증가"} → {opening})')
                self.jog_open.setText(f'누르는 동안 닫기 (틱 {"증가" if opening < closing else "감소"} → {closing})')
            self.open_button.setEnabled(manual_ready)
            self.close_button.setEnabled(manual_ready)
            self.jog_close.setEnabled(manual_ready)
            self.jog_open.setEnabled(manual_ready)
            for widget in (self.motor_minus_half, self.motor_plus_half,
                           self.motor_minus_one, self.motor_plus_one):
                widget.setEnabled(manual_ready)
            self.gripper_jog_step.setEnabled(False)
            captures = calibration.get('captures', {})
            both_captured = (set(captures) == {'open', 'close'}
                             and captures['open'] != captures['close'])
            self.validate_calibration.setEnabled(
                manual and calibration.get('active', False) and both_captured)
            self.save_calibration.setEnabled(
                manual and calibration.get('active', False)
                and calibration.get('validated', False))
        self.read_diag.setEnabled(spur and not self.mock_mode)
        self.start_cal.setEnabled(
            spur and manual and bool(self.tool_status.get('calibration_jog_enabled'))
            and not calibration.get('active', False))
        if self.spur_calibration_state is not None:
            active = bool(calibration.get('active'))
            enabled = bool(calibration.get('enabled'))
            captures = calibration.get('captures', {})
            both_captured = (set(captures) == {'open', 'close'}
                             and captures['open'] != captures['close'])
            if not active:
                state = '시작 전'
            elif not enabled:
                state = '토크 켜기 필요'
            elif calibration.get('validated'):
                state = '검증 완료 · 저장 가능'
            elif both_captured:
                state = '열림/닫힘 기록 완료 · 검증 필요'
            else:
                state = '±0.5° 이동 후 열림/닫힘 위치를 각각 기록하세요'
            self.spur_calibration_state.setText(ko(state))
            ready = (spur and manual and self._status_fresh()
                     and not bool(self.tool_status.get('read_only'))
                     and not bool(self.tool_status.get('emergency_stop'))
                     and not bool(self.tool_status.get('tool_detached')))
            healthy = (self._gripper_samples().get(5, {}).get('online')
                       and self._gripper_samples().get(5, {}).get(
                           'hardware_error') == 0)
            self.spur_calibration_start.setEnabled(ready and healthy and not active)
            self.spur_calibration_enable.setEnabled(
                ready and active and healthy and not enabled)
            self.spur_calibration_disable.setEnabled(active and enabled)
            for button in (self.spur_calibration_minus,
                           self.spur_calibration_plus,
                           self.spur_calibration_minus_five,
                           self.spur_calibration_plus_five,
                           self.spur_calibration_capture_open,
                           self.spur_calibration_capture_close):
                button.setEnabled(ready and active and enabled and healthy)
            self.spur_calibration_validate.setEnabled(
                ready and active and both_captured)
            self.spur_calibration_save.setEnabled(
                ready and active and bool(calibration.get('validated')))
        self._refresh_common_buttons()
        if end_effector_only:
            self._refresh_end_effector_torque_buttons()
            if self.tool_status.get('tool_preparation_state') != 'READY':
                for button in (self.open_button, self.close_button,
                               self.tool_stop, self.hold_open_button,
                               self.hold_close_button, self.clean_start,
                               self.clean_stop):
                    button.setEnabled(False)
        self._refresh_developer_direct_buttons()

    def _refresh_end_effector_torque_buttons(self):
        status = self.tool_status
        writable = not (status.get('read_only') or self.node.read_only)
        current = (self._status_fresh() and status.get('bridge_connected')
                   and status.get('tool_type') == self.node.selected_tool
                   and not self.pending_tool_change)
        prepared = (status.get('tool_preparation_state') == 'READY'
                    and status.get('tool_preparation_generation',
                                   status.get('tool_context_generation'))
                    == status.get('tool_context_generation'))
        self.common_enable.setEnabled(bool(
            current and writable and self.control_mode == 'MANUAL'
            and prepared
            and status.get('tool_enable_allowed')
            and not status.get('emergency_stop')
            and not status.get('tool_detached')
            and not status.get('physical_tool_detached')
            and status.get('tool_torque_state') != 'ON'))
        # Disable remains available for partial torque, faults and STOPPED.
        self.common_disable.setEnabled(bool(current and writable and any(
            sample.get('torque_state') != 'OFF'
            for sample in status.get('actuators', []))))

    def _refresh_developer_direct_buttons(self):
        """Keep the optional bench panel independent of FSM/manual ownership."""
        if not getattr(self, 'developer_direct_box', None):
            return
        sample = self._gripper_samples().get(5, {})
        ready = self._developer_spur_ready()
        torque_on = sample.get('torque_state') == 'ON'
        self.developer_enable.setEnabled(ready and not torque_on)
        self.developer_disable.setEnabled(ready and torque_on)
        self.developer_hold.setEnabled(ready and torque_on)
        self.developer_minus.setEnabled(ready and torque_on)
        self.developer_plus.setEnabled(ready and torque_on)
        self.developer_minus_five.setEnabled(ready and torque_on)
        self.developer_plus_five.setEnabled(ready and torque_on)

    def _spur_manual_ready(self):
        return (self._spur_enable_ready()
                and self.fsm_state in ('READY', 'OPEN', 'CLOSED')
                and (self.node.control_scope != 'END_EFFECTOR_ONLY'
                     or self._tool_motion_ready())
                and self._gripper_samples().get(5, {}).get('torque_state') == 'ON')

    def _common_motion_ready(self):
        status = self.tool_status
        ids = self.profile.get('actuator_ids', [])
        samples = self._gripper_samples()
        return bool(self.node.selected_tool in ('dual_motor_gripper', 'spur_1motor_gripper')
                    and self.pending_tool_change is None
                    and self.control_mode == 'MANUAL'
                    and self.node.control_scope == 'END_EFFECTOR_ONLY'
                    and self._status_fresh()
                    and status.get('tool_type') == self.node.selected_tool
                    and status.get('profile_valid') and status.get('calibrated')
                    and not status.get('read_only') and not getattr(self.node, 'read_only', False)
                    and not status.get('emergency_stop') and not status.get('tool_detached')
                    and ids and all(samples.get(i, {}).get('online')
                        and samples[i].get('hardware_error') == 0
                        and samples[i].get('torque_state') == 'ON' for i in ids)
                    and self.fsm_state in ('READY', 'OPEN', 'CLOSED'))

    def _start_spur_hold(self, command):
        if not self._spur_manual_ready():
            return
        self.spur_hold_command = command
        self.spur_hold_context = self._reported_tool_context()
        self._repeat_spur_hold()
        self.spur_hold_timer.start()

    def _repeat_spur_hold(self):
        if not self._spur_manual_ready():
            self._release_spur_hold()
            return
        if self.spur_hold_command:
            self.node.command_calibration(
                self.spur_hold_command,
                expected_context=self.spur_hold_context)

    def _release_spur_hold(self):
        self.spur_hold_timer.stop()
        active = self.spur_hold_command is not None
        self.spur_hold_command = None
        context = self.spur_hold_context
        self.spur_hold_context = None
        if active and self.tool_status.get('tool_type') == 'spur_1motor_gripper':
            self.node.command_calibration('manual_hold', expected_context=context)

    def _tool_motion_ready(self):
        fresh = self._status_fresh()
        scope_ok = self.tool_status.get('control_scope') == self.node.control_scope
        tool_type_ok = self.tool_status.get('tool_type') == self.node.selected_tool
        expected_ids = set(self.profile.get('actuator_ids', []))
        samples = self.tool_status.get('actuators', [])
        online_ids = {sample.get('id') for sample in samples
                      if sample.get('online')}
        actuators_ok = bool(expected_ids) and online_ids == expected_ids
        if self.node.selected_tool == 'cleaner' and self.mock_mode:
            actuators_ok = online_ids == expected_ids
        profile_ready = bool(self.tool_status.get('profile_valid')) \
            and (bool(self.tool_status.get('calibrated')) or self.mock_mode)
        temporary_ready = bool(self.tool_status.get('temporary_jog_ready')) \
            and self.node.temporary_jog_mode
        return (fresh and bool(self.tool_status.get('bridge_connected'))
                and bool(self.tool_status.get('motion_allowed')) and scope_ok
                and tool_type_ok
                and (self.node.control_scope != 'END_EFFECTOR_ONLY'
                     or (self.tool_status.get('tool_preparation_state') == 'READY'
                         and self.tool_status.get('fsm_class') == {
                             'dual_motor_gripper': 'DualMotorGripperFSM',
                             'spur_1motor_gripper': 'SingleMotorGripperFSM',
                             'cleaner': 'CleanerFSM'}.get(
                                 self.tool_status.get('tool_type'))
                         and getattr(self.node, 'runtime_tool_type', None)
                         == self.tool_status.get('tool_type')
                         and getattr(self.node, 'runtime_tool_generation', None)
                         == self.tool_status.get('tool_context_generation')
                         and self.tool_status.get(
                             'tool_preparation_generation',
                             self.tool_status.get('tool_context_generation'))
                         == self.tool_status.get('tool_context_generation')))
                and actuators_ok and (profile_ready or temporary_ready)
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached')))

    def _tool_enable_ready(self):
        """Readiness before torque is enabled; used only by ENABLE ID5."""
        fresh = self._status_fresh()
        return (fresh and bool(self.tool_status.get('bridge_connected'))
                and bool(self.tool_status.get('online'))
                and self.tool_status.get('position') is not None
                and self.tool_status.get('hardware_error') == 0
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached')))

    def _update_gripper_state(self, busy, state):
        self.gripper_busy = bool(busy)
        self.gripper_busy_label.setText(
            ko(f'BUSY: {state}' if busy else f'READY: {state}'))
        self.gripper_busy_label.setStyleSheet(
            FALSE_STYLE if busy else TRUE_STYLE)
        self._refresh_buttons()

    def _motor_endpoints(self):
        endpoints = self.profile.get('motor_endpoints', {})
        return {
            dxl_id: endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
            for dxl_id in self.profile.get('actuator_ids', [])}

    def _gripper_samples(self):
        return {sample.get('id'): sample
                for sample in self.tool_status.get('actuators', [])}

    def _normalized_positions(self):
        samples = self._gripper_samples()
        fractions = {}
        for dxl_id, endpoint in self._motor_endpoints().items():
            sample = samples.get(dxl_id)
            if not endpoint or not sample or sample.get('position') is None:
                return {}
            span = endpoint['open'] - endpoint['close']
            if span == 0:
                return {}
            fractions[dxl_id] = (
                (float(sample['position']) - endpoint['close']) / span)
        return fractions

    def _gripper_positions_synchronized(self):
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            return sample.get('position') is not None and bool(sample.get('online'))
        fractions = self._normalized_positions()
        return (len(fractions) == len(self.profile.get('actuator_ids', []))
                and max(fractions.values()) - min(fractions.values()) <= 0.05)

    def _update_gripper_feedback(self):
        if self.node.selected_tool == 'cleaner':
            return
        samples = self._gripper_samples()
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = samples.get(5, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(5)
            error = None if current is None or target is None else target - current
            self.gripper_position_label.setText(
                ko(f'Spur Gripper | Current: {current} | Target: {target} '
                f'| Error: {error}'))
            self.gripper_feedback_label.setText(
                ko(f'ID5: current={current}, target={target}, error={error}, '
                f'current/load={sample.get("effort")}, '
                f'online={sample.get("online", False)}\n'
                f'Safe range: {self.temporary_jog_safe_min} ~ '
                f'{self.temporary_jog_safe_max}\n'
                f'Mechanical range: {self.temporary_jog_mechanical_open} ~ '
                f'{self.temporary_jog_mechanical_close}'))
            self.spur_torque_state = self.tool_status.get(
                'tool_torque_state', 'UNKNOWN')
            self.spur_torque_enabled = self.spur_torque_state == 'ON'
            self.spur_endpoints = dict((self.tool_status.get('calibration') or {}).get(
                'captures', self.spur_endpoints))
            self.captured_endpoints_label.setText(
                ko(f'Captured OPEN: {self.spur_endpoints.get("open", "—")} | '
                f'CLOSE: {self.spur_endpoints.get("close", "—")}'))
            self.spur_actual_state.setText(
                ko(f'ID5: position={current} torque={self.spur_torque_state} '
                f'load={sample.get("effort")} mode={sample.get("operating_mode")} '
                f'velocity={sample.get("profile_velocity")} '
                f'acceleration={sample.get("profile_acceleration")} '
                f'hardware_error={self.tool_status.get("hardware_error")} '
                f'model={self.tool_status.get("model")} '
                f'fsm={self.fsm_state} calibrated={self.tool_status.get("calibrated")}'))
            zero = 'UNSET' if self.spur_zero_tick is None else str(self.spur_zero_tick)
            self.spur_mapping.setText(
                ko('Output mapping: zero offset/reference tick=' + zero + '\n'
                'GUI output angle → motor tick: motor_deg = '
                f'output_deg × {self.spur_gear_ratio:.3f} / '
                f'({self.spur_output_direction:+d}); '
                'tick = zero + motor_deg × 4096 / 360.\n'
                'External spur pair: output direction is the inverse of motor direction.'))
            return
        fractions = self._normalized_positions()
        if fractions:
            normalized = sum(fractions.values()) / len(fractions)
            spread = max(fractions.values()) - min(fractions.values())
            self.gripper_position_label.setText(
                ko(f'Gripper position: {normalized:.4f} '
                f'(0.0=closed, 1.0=open, motor spread={spread:.4f})'))
            if not self.gripper_busy and spread > 0.05:
                self.gripper_busy_label.setText(
                    ko(f'BLOCKED: motor normalized spread {spread:.4f} > 0.0500'))
                self.gripper_busy_label.setStyleSheet(FALSE_STYLE)
        else:
            self.gripper_position_label.setText(ko('Gripper position: UNKNOWN'))
        lines = []
        for dxl_id in self.profile.get('actuator_ids', []):
            sample = samples.get(dxl_id, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(dxl_id)
            error = None if current is None or target is None else target - current
            lines.append(
                f'ID{dxl_id}: current={current}, target={target}, '
                f'error={error}, current/load={sample.get("effort")}, '
                f'online={sample.get("online", False)}, '
                f'actual torque={sample.get("torque_state", "UNKNOWN")}, '
                f'hardware error={sample.get("hardware_error")}')
        self.gripper_feedback_label.setText(ko('\n'.join(lines) or 'No actuator data'))

    def _jog_gripper(self, direction):
        if self.pending_tool_change:
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            button = self.motor_minus_half if direction < 0 else self.motor_plus_half
            if button.isEnabled():
                self._jog_spur_motor(direction * 0.5)
            return
        reason = self._gripper_jog_block_reason()
        if reason:
            self._append_log(f'Gripper jog blocked: {reason}')
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            self._jog_spur(direction)
            return
        endpoints = self._motor_endpoints()
        samples = self._gripper_samples()
        step_ticks = max(1, round(4096 * 0.5 / 360.0))
        current = {dxl_id: samples[dxl_id]['position'] for dxl_id in endpoints}
        targets = {}
        for dxl_id, endpoint in endpoints.items():
            sign_to_open = 1 if endpoint['open'] > endpoint['close'] else -1
            raw = current[dxl_id] + direction * sign_to_open * step_ticks
            low, high = sorted((endpoint['open'], endpoint['close']))
            targets[dxl_id] = min(high, max(low, raw))
        if targets == current:
            self._append_log('Dual relative JOG no-op: already at requested endpoint')
            return
        label = 'OPEN' if direction > 0 else 'CLOSE'
        self._append_log(
            f'Dual relative {label} JOG: current={current}, target={targets}, '
            f'clamped_target={targets}, step=0.5°/{step_ticks} ticks')
        if self.command_current_tool(
                'JOG_RELATIVE', direction=int(direction), step_ticks=step_ticks):
            self.gripper_target_ticks = targets
            self._update_gripper_feedback()

    def _gripper_jog_block_reason(self):
        if (self.node.selected_tool == 'dual_motor_gripper'
                and (self.tool_status.get('dual_calibration') or {}).get('state')
                != 'READY'):
            return 'dual endpoint recalibration is required'
        if self.node.control_scope != 'END_EFFECTOR_ONLY':
            return 'control scope is not END_EFFECTOR_ONLY'
        if self.node.selected_tool not in (
                'dual_motor_gripper', 'spur_1motor_gripper'):
            return 'selected tool is not a supported gripper'
        if self.control_mode != 'MANUAL':
            return 'ownership is not MANUAL'
        if self.gripper_busy or self.node.gripper_busy:
            return 'BUSY'
        if not self._tool_motion_ready():
            return 'bridge/tool safety status is not ready or fresh'
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            if sample.get('position') is None or not sample.get('online'):
                return 'ID5 position/online feedback unavailable'
            return ''
        if not self._normalized_positions():
            return 'current actuator positions are unavailable'
        if not self._gripper_positions_synchronized():
            return 'motor normalized positions are not synchronized'
        return ''

    def _jog_spur(self, direction):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        step = int(self.gripper_jog_step.currentText())
        # Spur mapping: decreasing ticks opens, increasing ticks closes.
        target = int(current) + direction * step
        in_safe = self.temporary_jog_safe_min <= target <= self.temporary_jog_safe_max
        recovery = current < self.temporary_jog_safe_min or current > self.temporary_jog_safe_max
        inward = ((current > self.temporary_jog_safe_max and direction < 0)
                  or (current < self.temporary_jog_safe_min and direction > 0))
        if (not in_safe and not (recovery and inward)):
            self._append_log(
                f'Spur jog blocked: target={target} outside safe range '
                f'[{self.temporary_jog_safe_min}, {self.temporary_jog_safe_max}]')
            return
        if self.node.command_gripper(target):
            self.gripper_target_ticks = {5: target}
            self._update_gripper_feedback()

    def _jog_spur_motor(self, degrees):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        if current is None or self.spur_torque_state != 'ON':
            self._append_log('Motor jog blocked: ID5 position/actual torque unavailable')
            return
        if (self._spur_manual_ready()
                and self.node.command_calibration(
                    'manual_step', delta_deg=float(degrees),
                    expected_context=self._reported_tool_context())):
            self._append_log(f'ID5 CalibrationSession jog {degrees:+.1f}° requested')

    def _calibration_jog(self, degrees):
        """Compact panel adapter for the existing ID5 CalibrationSession."""
        if self.node.command_calibration('jog_motor_degrees', delta_deg=float(degrees)):
            self._append_log(f'캘리브레이션 ID5 이동 요청: {degrees:+.1f}°')

    def _capture_spur_endpoint(self, label):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        if current is None:
            self._append_log(f'Capture {label} blocked: ID5 position unavailable')
            return
        if self.node.command_calibration(f'capture_{label}'):
            self._append_log(f'CalibrationSession capture {label.upper()} requested (read only)')
            self._refresh_buttons()

    def _validate_spur_calibration(self):
        if self.node.command_calibration('validate'):
            self._append_log('CalibrationSession validation requested (no motor write)')

    def _save_spur_calibration(self):
        if self.node.command_calibration('save'):
            self._append_log(
                'Calibration save requested; bridge will atomically reload and require READY')

    def _command_tool(self, command):
        return self.command_current_tool(command)

    def _start_dual_hold_jog(self, direction):
        if self.tool_status.get('tool_type') != 'dual_motor_gripper':
            return
        if not self.open_button.isEnabled():
            self._append_log('Hold-to-run jog blocked by dual safety gate')
            return
        self.dual_hold_jog_active = True
        self.dual_hold_jog_direction = direction
        self.dual_hold_context = self._reported_tool_context()
        self._start_dual_key_jog(self._dual_jog_sign(direction))
        self._append_log(
            f'Dual hold-to-run {direction}: endpoint-ratio jog while held')
        self._refresh_buttons()

    def _release_dual_hold_jog(self):
        if not self.dual_hold_jog_active:
            return
        self._stop_dual_key_jog()
        self.dual_hold_jog_active = False
        self.dual_hold_jog_direction = None
        self._append_log('Dual hold-to-run released: current-position HOLD requested')
        self._refresh_buttons()

    def _stop_tool(self):
        if self.dual_key_jog_timer.isActive():
            self._stop_dual_key_jog()
        self.command_current_tool('STOP')

    def _enable_spur_motor(self):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        if current is None:
            self._append_log('ID5 Enable blocked: current tick feedback unavailable')
            return
        self.node.command_calibration('enable')
        self._append_log('CalibrationSession ENABLE ID5 requested')

    def _disable_spur_motor(self):
        self.node.command_calibration('disable')
        self._append_log('CalibrationSession DISABLE ID5 requested')

    def _enable_dual_motors(self):
        if self.node.set_dual_motor_enabled(True, self.profile.get('actuator_ids', [])):
            self._append_log('Operator requested dual torque enable for IDs [3, 4]')

    def _disable_dual_motors(self):
        if self.node.set_dual_motor_enabled(False, self.profile.get('actuator_ids', [])):
            self._append_log('Operator requested dual torque disable for IDs [3, 4]')

    def _manual_dual_recovery_jog(self, actuator_id, delta_deg):
        if self.node.manual_dual_recovery_jog(actuator_id, delta_deg):
            self._append_log(
                f'Operator requested one-click recovery jog: ID{actuator_id} '
                f'{delta_deg:+.1f}° (bridge re-reads actual position)')

    def _start_dual_calibration(self):
        if self.node.command_dual_calibration('start'):
            self._append_log('Dual endpoint calibration started (no motor write)')

    def _jog_dual_calibration_motor(self, actuator_id, direction):
        if self.dual_calibration_step is None:
            return
        degrees = float(self.dual_calibration_step.currentText()) * float(direction)
        if self.node.command_dual_calibration(
                'jog_motor_degrees', actuator_id=int(actuator_id),
                delta_deg=degrees):
            self._append_log(
                f'Dual calibration one-click jog requested: ID{actuator_id} '
                f'{degrees:+.1f}°')

    def _command_dual_calibration(self, command):
        if self.node.command_dual_calibration(command):
            self._append_log(f'Dual calibration command requested: {command}')

    def _command_spur_output_deg(self, output_deg):
        self._append_log('Output-angle command is unavailable during ID5 calibration')

    def keyPressEvent(self, event):
        if not self._keyboard_shortcuts_enabled():
            super().keyPressEvent(event)
            return
        if event.isAutoRepeat():
            event.ignore()
            return
        editing = self._keyboard_focus_is_editing()
        enabled = (self.pending_tool_change is None
                   and self.node.control_scope == 'END_EFFECTOR_ONLY'
                   and self.control_mode == 'MANUAL')
        if enabled and event.key() == Qt.Key_Space:
            self._stop_tool()
            event.accept()
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            if enabled and not editing and event.key() in (Qt.Key_Left, Qt.Key_Right):
                self._jog_gripper(-1 if event.key() == Qt.Key_Left else 1)
                event.accept()
                return
            event.ignore()
            return
        if (enabled and not editing and self.node.selected_tool == 'cleaner'
                and event.key() in (Qt.Key_Left, Qt.Key_Right)):
            self._common_action(1 if event.key() == Qt.Key_Left else 2)
            event.accept()
            return
        if (enabled and not editing
                and self.node.selected_tool == 'dual_motor_gripper'
                and event.key() in (Qt.Key_Left, Qt.Key_Right)):
            direction = (self._dual_jog_sign('OPEN') if event.key() == Qt.Key_Left
                         else self._dual_jog_sign('CLOSE'))
            self._start_dual_key_jog(direction)
            event.accept()
            return
        super().keyPressEvent(event)

    def eventFilter(self, watched, event):
        if event.type() == QEvent.WindowDeactivate and watched is self:
            self._release_spur_hold()
        if (event.type() == QEvent.WindowDeactivate
                and watched is self and self.dual_key_jog_timer.isActive()):
            self._stop_dual_key_jog()
        if event.type() not in (QEvent.KeyPress, QEvent.KeyRelease):
            return super().eventFilter(watched, event)
        if not self._keyboard_shortcuts_enabled():
            return super().eventFilter(watched, event)
        if self._keyboard_focus_is_editing():
            return super().eventFilter(watched, event)
        if event.key() not in (Qt.Key_Left, Qt.Key_Right):
            return super().eventFilter(watched, event)
        if not self.isVisible():
            return super().eventFilter(watched, event)
        if (self.node.selected_tool != 'dual_motor_gripper'
                or self.node.control_scope != 'END_EFFECTOR_ONLY'
                or self.control_mode != 'MANUAL'):
            return super().eventFilter(watched, event)
        if event.isAutoRepeat():
            return True
        if event.type() == QEvent.KeyPress:
            direction = (self._dual_jog_sign('OPEN') if event.key() == Qt.Key_Left
                         else self._dual_jog_sign('CLOSE'))
            self._log_key_trace(
                f'arrow keyPress: key={event.key()} direction={direction}')
            self._start_dual_key_jog(direction)
        elif self.dual_key_jog_timer.isActive():
            self._log_key_trace(
                f'arrow keyRelease: key={event.key()} -> HOLD')
            self._stop_dual_key_jog()
        return True

    def keyReleaseEvent(self, event):
        if not self._keyboard_shortcuts_enabled():
            super().keyReleaseEvent(event)
            return
        if (not event.isAutoRepeat()
                and event.key() in (Qt.Key_Left, Qt.Key_Right)
                and self.dual_key_jog_timer.isActive()):
            self._stop_dual_key_jog()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def _keyboard_shortcuts_enabled(self):
        return bool(getattr(self.node, 'keyboard_shortcuts_enabled', True))

    def _keyboard_focus_is_editing(self):
        return isinstance(
            self.focusWidget(),
            (QAbstractSpinBox, QLineEdit, QTextEdit, QComboBox))

    def _start_dual_key_jog(self, direction):
        if (self.pending_tool_change
                or self.tool_status.get('tool_type') != 'dual_motor_gripper'):
            return
        if not self.open_button.isEnabled():
            return
        if self.dual_key_jog_timer.isActive():
            return
        if self.dual_hold_context is None:
            self.dual_hold_context = self._reported_tool_context()
        self.dual_key_jog_direction = int(direction)
        self.dual_key_jog_timer.start()
        self._dual_key_jog_tick()

    @staticmethod
    def _dual_jog_sign(direction):
        # Shared physical convention for the hold buttons and arrow keys.
        return {'OPEN': 1, 'CLOSE': -1}[direction]

    def _dual_key_jog_tick(self):
        if self.dual_key_jog_direction not in (-1, 1):
            return
        if (self.control_mode != 'MANUAL'
                or self.tool_status.get('tool_type') != 'dual_motor_gripper'
                or self.fsm_state in ('FAULT', 'STOPPED')
                or self.tool_status.get('emergency_stop')
                or self.tool_status.get('tool_detached')):
            self._stop_dual_key_jog()
            return
        command = 'JOG_OPEN' if self.dual_key_jog_direction > 0 else 'JOG_CLOSE'
        if not self.command_current_tool(
                command, expected_context=self.dual_hold_context):
            self._stop_dual_key_jog()

    def _stop_dual_key_jog(self):
        self.dual_key_jog_timer.stop()
        self.dual_key_jog_direction = 0
        context = self.dual_hold_context
        self.dual_hold_context = None
        if context is not None:
            self.command_current_tool('HOLD', expected_context=context)
        self.dual_hold_jog_active = False
        self.dual_hold_jog_direction = None
        self._append_log('Dual jog released: current-position HOLD requested')

    def closeEvent(self, event):
        self._release_spur_hold()
        if self.dual_key_jog_timer.isActive():
            self._stop_dual_key_jog()
        self._release_qt_resources()
        # A repeating timer keeps the window alive for the event loop; stop it
        # so shutdown does not depend on Qt garbage-collection order.
        self.watchdog.stop()
        super().closeEvent(event)

    def _release_qt_resources(self):
        """Unregister application-owned Qt hooks before deleting the window.

        This is deliberately idempotent: test fixtures can call ``close()``,
        then ``deleteLater()``, and Qt may send a second close event during
        application shutdown.  The Korean translator is QApplication-owned
        and remains installed until QApplication shuts down.
        """
        if self._qt_resources_released:
            return
        self._qt_resources_released = True
        app = QApplication.instance()
        if app is not None:
            app.removeEventFilter(self)
        for timer in (self.spur_hold_timer, self.watchdog,
                      self.dual_key_jog_timer):
            timer.stop()

    def _log_key_trace(self, message):
        get_logger = getattr(self.node, 'get_logger', None)
        if get_logger is not None:
            get_logger().info(message)

    def _jog(self, joint, sign):
        self.node.jog_arm(joint, sign * float(self.jog_step.currentText()))

    def _arm_target(self, joint):
        self.node.command_arm(joint, self.arm_targets[joint].value())

    def _request_mode(self):
        requested = self.mode_combo.currentData()
        self._append_log(
            f'Mode request clicked: requested={requested}, '
            f'approved={self.control_mode}')
        if (not self.mock_mode and requested == 'MANUAL'
                and (self.arm_fsm_state if self.node.control_scope == 'FULL_ROBOT'
                     else self.fsm_state) not in ToolManager.SAFE_CHANGE_STATES
                and not (self.node.selected_tool == 'spur_1motor_gripper'
                         and self.fsm_state in ('CALIBRATION_REQUIRED', 'STOPPED', 'READY'))
                and not (self.node.selected_tool == 'dual_motor_gripper'
                         and self.node.control_scope == 'END_EFFECTOR_ONLY')
                and not (self.node.selected_tool == 'cleaner'
                         and self.node.control_scope == 'END_EFFECTOR_ONLY')):
            QMessageBox.warning(
                self, ko('Ownership denied'),
                ko(f'MANUAL is allowed only in IDLE/STOWED; current={self.fsm_state}'))
            return
        self.node.request_mode(requested)

    def _request_tool_change(self):
        requested = self.tool_combo.currentData()
        current = self.tool_status.get('tool_type', self.node.selected_tool)
        if requested == current and self.node.control_scope != 'END_EFFECTOR_ONLY':
            self._append_log(f'{requested} is already selected')
            return
        if requested not in ('spur_1motor_gripper', 'dual_motor_gripper', 'cleaner'):
            QMessageBox.warning(self, ko('도구 변경 거부'),
                                ko('지원하지 않는 도구입니다.'))
            self.tool_combo.setCurrentIndex(self.tool_combo.findData(current))
            return
        if self.dual_key_jog_timer.isActive():
            self._stop_dual_key_jog()
        self.pending_tool_change = requested
        self._pending_tool_generation = self._runtime_context_generation
        self.tool_change_requested = requested
        self.pending_tool_change_started = self._clock()
        self.pending_tool_change_link_ok = self._status_fresh()
        self.tool_change_state = 'PENDING'
        self.tool_change_detail = ''
        if not self.node.request_tool_change(requested):
            self._finish_tool_change(
                'REJECTED', '요청을 발행하지 못했습니다 (GUI 안전 게이트)')
            self._refresh_buttons()
            return
        self._refresh_buttons()
        self._append_log(ko(self._tool_change_message()))

    def _estop(self):
        self.node.emergency_stop()
        self.estop_state.setText(ko('E-STOP: REQUESTED'))
        self.estop_state.setStyleSheet(FALSE_STYLE)
        self.restart_program.setEnabled(False)

    def _restart_program(self):
        """Restart the whole launch after the bridge has latched an E-stop."""
        if not self.tool_status.get('emergency_stop'):
            return
        answer = QMessageBox.question(
            self, ko('프로그램 재구동'),
            ko('비상 정지 상태입니다. 현재 프로그램을 종료하고 다시 시작할까요?'))
        if answer != QMessageBox.Yes:
            return
        self.restart_program.setEnabled(False)
        try:
            restart_parent_launch()
        except Exception as exc:
            self.restart_program.setEnabled(True)
            self._append_log(f'프로그램 재구동 실패: {exc}')
            return
        self._append_log('비상 정지 프로그램을 재구동합니다')
        QApplication.instance().quit()

    def _detach(self):
        answer = QMessageBox.question(
            self, ko('Confirm detach'), ko('Mark the current tool as DETACHED and stop it?'))
        if answer == QMessageBox.Yes:
            self.node.tool_detached()

    def _run_process(self, program, args):
        process = QProcess(self)
        process.setProgram(program)
        process.setArguments(args)
        process.readyReadStandardOutput.connect(
            lambda: self._append_log(bytes(
                process.readAllStandardOutput()).decode(errors='replace')))
        process.readyReadStandardError.connect(
            lambda: self._append_log(bytes(
                process.readAllStandardError()).decode(errors='replace')))
        process.finished.connect(lambda: self._append_log('Diagnostic process finished'))
        self.processes.append(process)
        process.start()

    def _read_only_diagnostic(self):
        if self._status_fresh():
            self._append_log(
                'Bridge already owns the serial bus; using /tool/status read-only '
                f'diagnostics: {self.tool_status}')
            return
        ids = self.profile.get('actuator_ids', [5])
        self._run_process('ros2', [
            'run', 'dynamixel_control', 'spur_gripper_calibration',
            '--actuator-id', str(ids[0]), '--read-only'])

    def _start_calibration(self):
        if self.node.command_calibration('start'):
            self._append_log('CalibrationSession started (no register write)')

    def _rebuild_diagnostics(self, actuators, joint_values=None):
        joint_values = joint_values or {}
        rows = []
        for index, joint in enumerate(ARM_JOINTS):
            sample = joint_values.get(joint, {})
            position = sample.get('position', self.node.positions.get(joint))
            effort = sample.get('effort', self.node.efforts.get(joint))
            rows.append((index, joint, position, effort, position is not None))
        for sample in actuators:
            rows.append((sample.get('id'), sample.get('joint'),
                         sample.get('position'), sample.get('effort'),
                         sample.get('online', False)))
        self.diag.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for column, value in enumerate(values):
                text = '—' if value is None else str(value)
                item = QTableWidgetItem(ko(text))
                if column == 4:
                    item.setForeground(Qt.darkGreen if value else Qt.red)
                self.diag.setItem(row, column, item)

    def _append_log(self, text):
        self.log.append(ko(str(text).strip()))
