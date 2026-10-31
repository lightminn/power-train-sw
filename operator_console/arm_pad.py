"""Pad input to the existing authenticated arm command session.

Face buttons drive the attached tool; bumpers select an arm axis and the D-pad
jogs it (up/down) or changes jog speed (left/right).  Arm jogs need no tool.
"""

ARM_AXES = 5


class ArmPadControl:
    def __init__(self, session, axis_view=None):
        self.session = session
        # axis_view (the keyboard-teleop panel) shares one selected axis with the pad.
        self._axis_view = axis_view
        self._axis = 1
        self._identity = None
        self._needs_release = True
        self._target = None
        self._previous = set()
        self._arm_axis = None
        self._arm_wait_release = False
        self.detail = '패드 조작 꺼짐'

    @property
    def axis(self):
        if self._axis_view is not None:
            return self._axis_view.selected_axis or 1
        return self._axis

    @axis.setter
    def axis(self, index):
        self._axis = index
        if self._axis_view is not None:
            self._axis_view.select_axis(index)

    def cancel(self):
        if self._target is not None:
            self.session.release_tool_jog(self._target)
        self._target = None
        self._release_arm()
        self._needs_release = True
        self._previous.clear()

    def _release_arm(self):
        if self._arm_axis is not None:
            self.session.release_axis_jog(self._arm_axis)
        self._arm_axis = None

    def update(self, status, *, enabled, focused):
        context = self.session.context
        identity = context.tool_id, context.tool_generation
        if identity != self._identity:
            self.cancel()
            self._identity = identity
        if (not enabled or not focused or not status.get('pad_connected')
                or status.get('age_s', 9) > .6 or not status.get('arm_control')
                or not context.link_live or not context.manual_granted):
            self.cancel()
            self.detail = ('수동 제어권·패드 연결·창 포커스를 확인하세요' if enabled else '패드 조작 꺼짐')
            return
        if status.get('tool_mapping') != 'known':
            self.cancel()
            self.detail = '지원 패드: Xbox / DualSense · 패드 이름을 확인하세요'
            return
        buttons = set(status.get('tool_buttons', []))
        if self._needs_release:
            self._needs_release = bool(buttons)
            self._previous = buttons
            self.detail = '도구 교체·연결 후 버튼을 모두 놓으세요' if buttons else '패드 조작 준비'
            return
        pressed = buttons - self._previous
        arm_detail = self._update_arm(buttons, pressed)
        target = 'both' if context.tool_kind == 'dual_gripper' else 'single'
        direction = 1 if 'open' in buttons else -1 if 'close' in buttons else 0
        if 'stop' in buttons or 'disable' in buttons or {'open', 'close'} <= buttons:
            direction = 0
        if not direction and self._target is not None:
            self.session.release_tool_jog(self._target)
            self._target = None
        if 'disable' in pressed:
            self.session.set_tool_enabled(False)
        elif 'stop' in pressed:
            self.session.soft_stop_tool(target)
        elif direction:
            result = self.session.press_tool_jog(target, direction)
            if result:
                self._target = target
            self.detail = '도구 누름 조작 중' if result else result.korean
        else:
            self.detail = arm_detail
        self._previous = buttons

    def _update_arm(self, buttons, pressed):
        if 'axis_next' in pressed:
            self.axis = self.axis % ARM_AXES + 1
        if 'axis_prev' in pressed:
            self.axis = (self.axis - 2) % ARM_AXES + 1
        if 'speed_up' in pressed:
            self.session.change_speed(+1)
        if 'speed_down' in pressed:
            self.session.change_speed(-1)
        direction = 1 if 'jog_plus' in buttons else -1 if 'jog_minus' in buttons else 0
        if {'jog_plus', 'jog_minus'} <= buttons:
            direction = 0
        if not direction:
            self._arm_wait_release = False
        if self._arm_axis is not None and (not direction or self._arm_axis != self.axis):
            # An axis change mid-jog stops the old axis; the new one needs a fresh press.
            self._arm_wait_release = bool(direction)
            self._release_arm()
        status = f'팔 {self.axis}축 · 속도 {self.session.speed_level}단'
        if direction and not self._arm_wait_release:
            result = self.session.press_axis_jog(self.axis, direction)
            if result:
                self._arm_axis = self.axis
                return f'{status} 조그 중'
            return result.korean
        if self._arm_wait_release:
            return f'{status} · 십자키를 놓았다가 다시 누르세요'
        return (f'{status} · LB/RB 축 선택 · 십자키 ↑↓ 조그 ←→ 속도 · '
                'X/□ 열기 · B/○ 닫기 · A/× 정지 · Y/△ 토크 해제')
