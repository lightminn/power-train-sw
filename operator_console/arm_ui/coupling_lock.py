"""Existing manual coupling-lock controls, rendered in the console card style."""
from __future__ import annotations

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from . import contracts as C
from .widgets import Card, Metric, SignalTracker, button, label, metric_grid, reflow_row


LOCK_LABELS = {
    'LOCKED': '결합 위치', 'UNLOCKED': '분리 위치', 'RELEASED': '분리 완료',
    'UNCALIBRATED': '영점 미설정', 'MOVING': '이동 중', 'IDLE': '대기',
    'CONFIRMED': '절차 완료', 'PREPARING': '도구 정지 확인',
    'DONE': '완료', 'ABORTED': '중단', 'ENGAGED': '결합 추정',
    'NOT_ENGAGED': '결합 미확인', 'DISABLED': '사용 안 함',
    'UNKNOWN': '확인 불가', 'FAULT': '오류',
}


def display(value):
    return '미수신' if value is None else LOCK_LABELS.get(value, str(value))


class CouplingLockPanel:
    def __init__(self, callbacks: C.ArmUiCallbacks):
        self._callback = callbacks.coupling_lock_command
        self._state = C.default_state()
        self.tracker = SignalTracker()
        card = Card('도구 결합 잠금',
                    '도구를 받친 상태에서 잠금 장치를 조작합니다. 팔 이동은 실행하지 않습니다.',
                    accent='tool')
        self.box = card.box
        self.badge = card.badge
        self.metrics = {key: Metric(title, width_chars=12) for key, title in (
            ('state', '잠금 위치'), ('zero', '영점 기준'), ('position', '모터 위치 · tick'),
            ('sequence', '결합·해제 진행'), ('coupling', '결합 추정'), ('torque', '잠금 모터'))}
        self.reason = label('', 'arm-note', wrap=True)
        row = Gtk.Box(spacing=8)
        row.pack_start(label('회전 방향', 'arm-metric-label'), False, False, 0)
        self.direction = Gtk.ComboBoxText()
        self.direction.append('1', '정방향 · 22.5°')
        self.direction.append('-1', '역방향 · 22.5°')
        self.direction.set_active_id('1')
        row.pack_start(self.direction, False, False, 0)
        self.buttons = {}
        definitions = (
            ('zero_locked', '현재 위치를 결합 영점으로', 'SET_ZERO', {'state': 'LOCKED'}),
            ('zero_unlocked', '현재 위치를 분리 영점으로', 'SET_ZERO', {'state': 'UNLOCKED'}),
            ('enable', '잠금 모터 활성화', 'ENABLE', {}),
            ('stop', '잠금 정지', 'STOP', {}),
            ('attach', '도구 결합', 'CHANGE', {'to': 'LOCKED'}),
            ('detach', '도구 해제', 'CHANGE', {'to': 'UNLOCKED'}),
        )
        for key, title, command, options in definitions:
            widget = button(title, 'arm-danger' if key == 'stop' else 'arm-primary'
                            if key in ('enable', 'attach') else '')
            self.buttons[key] = widget
            self.tracker.connect(widget, 'clicked',
                                 lambda _w, k=key, c=command, o=options: self._send(k, c, o))
        card.pack(reflow_row([self.buttons[k] for k in ('attach', 'detach', 'stop')], columns=3))
        card.pack(metric_grid(tuple(self.metrics.values()), columns=3))
        card.pack(label('결합 센서가 없습니다. 위치 도달과 잠금 절차만으로 판단하며, '
                        '도구가 빠지거나 떨어져도 감지하지 못합니다.', 'arm-reason', wrap=True))
        card.pack(self.reason)
        advanced = Gtk.Expander(label='영점 · 회전 방향 · 잠금 모터 준비')
        setup = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        setup.pack_start(row, False, False, 0)
        setup.pack_start(reflow_row([self.buttons[k] for k in
                                    ('zero_locked', 'zero_unlocked', 'enable')], columns=3),
                         False, False, 0)
        advanced.add(setup)
        card.pack(advanced)
        self.error = label('', 'arm-note', wrap=True)
        card.pack(self.error)
        self.update(self._state)

    def _allowed(self, key):
        state, lock = self._state, self._state.coupling_lock
        if self._callback is None:
            return False, '미연결 · 잠금 명령 경로 없음'
        if key == 'stop':
            # The authenticated backend still enforces read-only mode. A stale
            # display or lost MANUAL grant must not prevent asking it to stop.
            return True, ''
        allowed, reason = C.gate(state, self._callback, C.CAP_COUPLING_LOCK)
        if not allowed:
            return allowed, reason
        if not lock:
            return False, '잠금 상태 미수신 또는 갱신 지연'
        if key.startswith('zero_') or key == 'enable':
            if lock.get('torque') != 0:
                return False, '잠금 모터 토크OFF 필요'
            if key == 'enable' and lock.get('zero_tick') is None:
                return False, '영점 설정 필요'
        elif (lock.get('torque') != 1
              or lock.get('state') not in ('LOCKED', 'UNLOCKED')):
            return False, '잠금 모터 활성화·위치 확정 필요'
        return True, ''

    def _send(self, key, command, options):
        if not self._allowed(key)[0]:
            return
        payload = dict(options)
        if command == 'CHANGE':
            payload['direction'] = int(self.direction.get_active_id())
        self._callback(command, payload)

    def update(self, state):
        self._state = state
        lock = state.coupling_lock
        for key, value in (
                ('state', display(lock.get('state'))),
                ('zero', '미설정' if lock.get('zero_tick') is None else
                 f"{lock['zero_tick']} · {display(lock.get('zero_state'))}"),
                ('position', display(lock.get('position'))),
                ('sequence', display(lock.get('sequence_state'))),
                ('coupling', display(lock.get('coupling_proxy'))),
                ('torque', {0: '토크OFF', 1: '토크ON'}.get(lock.get('torque'), '미수신'))):
            self.metrics[key].set(value)
        enabled, reason = self._allowed('attach')
        self.reason.set_text('수동 잠금 조작 준비됨' if enabled else reason)
        self.badge.set(display(lock.get('state')), 'status-live' if enabled else 'status-muted')
        for key, widget in self.buttons.items():
            allowed, hint = self._allowed(key)
            widget.set_sensitive(allowed)
            widget.set_tooltip_text('잠금 명령을 요청합니다. 결과는 수신 상태로 확인합니다.'
                                    if allowed else hint)
        self.error.set_text(lock.get('command_error') or lock.get('sequence_error')
                            or lock.get('error') or '')

    def dispose(self):
        self.tracker.dispose()
