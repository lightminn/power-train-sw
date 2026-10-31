"""Headless arm client: existing authenticated ops and arm telemetry only.

Run ``python -m operator_console.arm_terminal --help``. No GTK, ROS or serial
driver is imported here. The existing ROS bridge remains the motor owner.
"""
from __future__ import annotations

import argparse
import curses
from dataclasses import replace
import os
from pathlib import Path
import queue
import shlex
import signal
import sys
import time

from .arm_ops import ArmCommandSession, ContractGatedTransport
from .arm_ops import contract as K
from .arm_ops.adapter import context_from_state
from .arm_telemetry import LatestArmTelemetryReceiver
from .arm_ui_binding import ArmUiTelemetryBinding
from .ops_client import ConsoleOpsClient


HOLD_TIMEOUT_S = .15  # A terminal sends repeats, not key-release events.
KEY_HELP = (
    '1..5 팔/6 도구 · ↑/w + · ↓/s - · [/] 속도 · Space 정지 · t 5축 복귀',
    'a MANUAL 요청 · h home · l 제한 · y 트립 · : 명령 · q 종료 · Ctrl+X 항상 정지',
    '명령: save/move/delete 이름 | freedrive/release | lock/unlock [1|-1] | lock-stop',
    'zero locked/unlocked | lock-enable/lock-disable | tool-enable/tool-disable',
    'tool open/close/stop/left/right [single/left/right/both] | target 대상',
    'select-tool single/dual/cleaner | capture 슬롯 도구 seat/above',
    'exchange 도구 [dry-run] | exchange-stop | manual | authority-release',
    '정지: 1~4축 이동 홀드, 5축·볼락 토크OFF. 전체 팔 하드웨어 E-stop 아님.',
    'freedrive는 1~4축 토크OFF. 팔을 받친 상태에서 사용하세요.',
)
TOOLS = {'single': 'spur_1motor_gripper', 'dual': 'dual_motor_gripper',
         'cleaner': 'cleaner'}


class ArmTerminalClient:
    """Python API; call tick regularly and repeat jog while the key is held.

    Commands report submission; jog reports an accepted hold, sent by tick.
    Neither means physically completed. Inspect state and last_ack for feedback.
    Construction never requests motion.
    """

    def __init__(self, host, port, token_file, *, telemetry_port=15707):
        token = Path(token_file).read_text().strip()
        if not token:
            raise ValueError('ops token file is empty')
        self._callbacks = queue.SimpleQueue()
        self._pending = {}
        self._stop_ids = set()
        self._failed_stop = False
        self.last_ack = ''
        self.last_outcome = ''
        self.snapshot = None
        self._binding = ArmUiTelemetryBinding()
        self.state = self._binding.state(None)
        self._closed = False
        self._receiver = LatestArmTelemetryReceiver(telemetry_port, exclusive=True)
        try:
            self._ops = ConsoleOpsClient(host, port, token,
                submit_sink=self._ack, state_sink=lambda _state: None,
                schedule=self._callbacks.put)
        except BaseException:
            self._receiver.close()
            raise
        self.session = ArmCommandSession(clock=time.monotonic,
            transport=ContractGatedTransport(self._submit,
                frozenset(K.ARM_ACTIONS)))
        self.axis = 5
        self._target = None
        self._jog_axis = None
        self._jog_at = None
        self._jog_direction = None
        self._jog_refresh_at = 0.
        self._needs_release = False
        self._exchange_available = False

    @property
    def target(self):
        return self._target or ('both' if self.session.context.tool_kind == 'dual_gripper' else 'single')

    def _submit(self, action, params, revision):
        request_id = self._ops.submit(action, params, revision)
        self._pending[request_id] = action
        if action in K.STOP_ACTIONS or params.get('command') == 'stop':
            self._stop_ids.add(request_id)
        return request_id

    def wait_until(self, predicate, timeout=5.):
        """Pump received state until a caller's feedback condition is true."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.tick()
            if predicate(self.state):
                return self.state
            time.sleep(.02)
        raise TimeoutError(f'팔 피드백 대기 시간 초과: {self.last_ack}')

    def _ack(self, response):
        request_id = response.get('request_id', '')
        action = self._pending.get(request_id)
        if action is None:
            return
        status = response.get('status', '')
        self.session.on_ack(K.ArmOpsAck(request_id=request_id,
            action=action, status=status, detail=response.get('detail', '')))
        self.last_ack = f'{action}: {status} {response.get("detail", "")}'
        if status.startswith('FINAL_') or status == 'OUTCOME_UNKNOWN':
            if request_id in self._stop_ids and status != 'FINAL_SUCCESS':
                self._failed_stop = True
            self._stop_ids.discard(request_id)
            self._pending.pop(request_id, None)

    def _outcome(self, result):
        self.last_outcome = (f'전송 대기: {result.action}' if result else
                             f'거부: {result.korean or result.reason}')
        return result

    def tick(self):
        for _ in range(256):
            try:
                self._callbacks.get_nowait()()
            except queue.Empty:
                break
        self.snapshot = self._receiver.latest()
        ops = self._ops.latest_state() or {}
        ops_ready = self._ops.connected and bool(ops)
        self.state = self._binding.state(self.snapshot,
            ops_link_ready=ops_ready)
        if self.state.link.state == 'LIVE':
            self._exchange_available = ('teleop_pose' in self.state.capabilities
                                        or bool(self.state.tool_exchange))
        context = context_from_state(self.state, state_revision=ops.get('revision'))
        if not ops_ready:
            context = replace(context, link_live=False)
        ended = self.session.update_context(context)
        if ended and self._jog_axis is not None:
            self.release_jog()
            self._needs_release = True
        if (self._jog_at is not None
                and time.monotonic() - self._jog_at >= HOLD_TIMEOUT_S):
            self.release_jog()
            self._needs_release = False
        if (self._jog_axis is not None
                and time.monotonic() - self._jog_refresh_at >= .2):
            self._press_jog()
        for outcome in self.session.pump():
            if not outcome:
                self._outcome(outcome)
        return self.state

    def jog(self, axis, direction):
        if type(axis) is not int or axis not in range(1, 7) or type(direction) is not int or direction not in (-1, 1):
            raise ValueError('jog requires axis 1..6 and direction -1/+1')
        self._jog_at = time.monotonic()
        if self._needs_release:
            self.last_outcome = '조그를 놓은 뒤 다시 누르세요'
            return None
        if (axis, direction) != (self._jog_axis, self._jog_direction):
            self.release_jog()
            self.axis = self._jog_axis = axis
            self._jog_direction = direction
            self._jog_at = time.monotonic()
            return self._press_jog()
        return None  # Key-repeat rate must not become motor-command rate.

    def _press_jog(self):
        result = (self.session.press_tool_jog(self.target, self._jog_direction)
                  if self._jog_axis == 6 else
                  self.session.press_axis_jog(self._jog_axis, self._jog_direction))
        self._jog_refresh_at = time.monotonic()
        if not result:
            self.release_jog()
        return self._outcome(result)

    def release_jog(self):
        if self._jog_axis is not None:
            if self._jog_axis == 6:
                self.session.release_tool_jog(self.target)
            else:
                self.session.release_axis_jog(self._jog_axis)
        self._jog_axis = self._jog_direction = None

    def stop(self):
        self.release_jog()
        self._needs_release = True
        self._jog_at = time.monotonic()
        if self._exchange_available:
            self.session.stop_tool_exchange()
        self.session.coupling_lock_command('STOP', {})
        if self.session.context.tool_id:
            self.session.soft_stop_tool(self.target)
        return self._outcome(self.session.stop_motion())

    def command(self, text):
        """Execute the same named commands used by the terminal's ':' prompt."""
        self.release_jog()
        args = shlex.split(text)
        if not args:
            return None
        name, *values = args
        s = self.session
        simple = {
            'manual': lambda: s.request_mode('MANUAL'),
            'authority-release': s.release_mode, 'stop': self.stop,
            'resume': s.resume_hold, 'home': s.go_home,
            'freedrive': s.freedrive, 'release': s.freedrive_release,
            'lock': lambda: s.coupling_lock_command('CHANGE', {'to': 'LOCKED', 'direction': 1}),
            'unlock': lambda: s.coupling_lock_command('CHANGE', {'to': 'UNLOCKED', 'direction': -1}),
            'lock-stop': lambda: s.coupling_lock_command('STOP', {}),
            'lock-enable': lambda: s.coupling_lock_command('ENABLE', {}),
            'lock-disable': lambda: s.coupling_lock_command('STOP', {}),
            'tool-enable': lambda: s.set_tool_enabled(True),
            'tool-disable': lambda: s.set_tool_enabled(False),
            'exchange-stop': s.stop_tool_exchange,
        }
        if name in simple and not values:
            return self._outcome(simple[name]())
        if name in ('lock', 'unlock') and len(values) == 1 and values[0] in ('1', '-1'):
            return self._outcome(s.coupling_lock_command('CHANGE',
                {'to': 'LOCKED' if name == 'lock' else 'UNLOCKED', 'direction': int(values[0])}))
        if name in ('limits', 'trip') and values in (['on'], ['off']):
            K.BENCH_JOG_FLAGS[name] = values[0] == 'on'
            self.last_outcome = f'{name}: {values[0]} (다음 명령부터 적용)'
            return None
        if name == 'target' and len(values) == 1 and values[0] in K.TOOL_TARGETS:
            self._target = values[0]
            return None
        if name in ('save', 'move', 'delete') and len(values) == 1:
            method = {'save': s.save_pose, 'move': s.move_to_pose,
                      'delete': s.delete_pose}[name]
            return self._outcome(method(values[0]))
        if name == 'zero' and values in (['locked'], ['unlocked']):
            return self._outcome(s.coupling_lock_command('SET_ZERO',
                {'state': values[0].upper()}))
        if name == 'select-tool' and len(values) == 1 and values[0] in TOOLS:
            return self._outcome(s.request_tool_change(TOOLS[values[0]]))
        if name == 'tool' and 1 <= len(values) <= 2:
            target = values[1] if len(values) == 2 else self.target
            return self._outcome(s.tool_command(target, values[0]))
        if (name == 'capture' and len(values) == 3 and values[1] in TOOLS):
            return self._outcome(s.capture_tool_slot(values[0], TOOLS[values[1]], values[2]))
        if (name == 'exchange' and len(values) in (1, 2) and values[0] in TOOLS
                and (len(values) == 1 or values[1] == 'dry-run')):
            return self._outcome(s.start_tool_exchange(TOOLS[values[0]], len(values) == 2))
        raise ValueError('알 수 없는 명령/인수, 도움말의 형식을 확인하세요')

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            result = self.stop()
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                self.tick()
                if not self._pending and self.session.pending_stop_count == 0:
                    break
                time.sleep(.02)
            if not result or self._failed_stop or self._pending or self.session.pending_stop_count:
                print('정지 ACK 미확인. 팔 상태를 현장에서 확인하세요.', file=sys.stderr)
        finally:
            self._ops.close()
            self._receiver.close()

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


def terminal(screen, client, stopping):
    curses.curs_set(0)
    screen.nodelay(True)
    screen.keypad(True)
    prompt = None
    while not stopping[0]:
        client.tick()
        try:
            character = screen.get_wch()
            key = ord(character) if isinstance(character, str) else character
        except curses.error:
            character, key = None, -1
        try:
            if key == 24:
                client.stop()
                prompt = None
            elif prompt is not None:
                if key == 27:
                    prompt = None
                elif key in (10, 13):
                    client.command(prompt)
                    prompt = None
                elif key in (curses.KEY_BACKSPACE, 127, 8):
                    prompt = prompt[:-1]
                elif isinstance(character, str) and character.isprintable() and len(prompt) < 180:
                    prompt += character
            elif key in (ord('q'), 3):
                return
            elif key == ord(':'):
                client.release_jog()
                prompt = ''
            elif key in range(ord('1'), ord('7')):
                client.release_jog()
                client.axis = key - ord('0')
            elif key in (curses.KEY_UP, ord('w'), curses.KEY_DOWN, ord('s')):
                client.jog(client.axis, 1 if key in (curses.KEY_UP, ord('w')) else -1)
            elif key == ord(' '):
                client.stop()
            elif key == ord('t'):
                client.command('resume')
            elif key == ord('a'):
                client.command('manual')
            elif key == ord('h'):
                client.command('home')
            elif key in (ord('['), ord(']')):
                client.release_jog()
                client.session.change_speed(1 if key == ord(']') else -1)
            elif key in (ord('l'), ord('y')):
                flag = 'limits' if key == ord('l') else 'trip'
                client.command(f'{flag} {"off" if K.BENCH_JOG_FLAGS.get(flag, True) else "on"}')
        except (ValueError, K.ArmParamsError) as exc:
            client.last_outcome = str(exc)
        state = client.state
        lock = state.coupling_lock
        exchange = state.tool_exchange
        lines = [f'로봇팔 터미널 | 팔 {state.link.state} | OPS={client._ops.connected} | MANUAL={client.session.context.manual_granted}',
            f'축 {client.axis} | 속도 {client.session.speed_level}/10 | 제한 {K.BENCH_JOG_FLAGS.get("limits", True)} | 트립 {K.BENCH_JOG_FLAGS.get("trip", True)}',
            f'도구 {state.detected_tool.tool_id or "미수신"} | 대상 {client.target}',
            f'볼락 {lock.get("state", "미수신")} | 위치 {lock.get("position")} | 토크 {lock.get("torque")} | 결합점 {lock.get("locked_tick")} | 해제점 {lock.get("unlocked_tick")}',
            f'볼락 오류: {lock.get("command_error") or lock.get("error") or lock.get("sequence_error") or "없음"}',
            f'저장 자세: {", ".join(state.teleop.poses) or "없음/미수신"}',
            f'교체 {exchange.get("state", "미수신")} | 슬롯 {exchange.get("slot")} | {exchange.get("step")}/{exchange.get("steps")} | 오류 {exchange.get("error", "")}',
            f'슬롯 목록: {exchange.get("slots", {})}',
            '각도(°): ' + ' | '.join(f'{a.index}: {a.angle_deg:.2f}'
                if a.angle_deg is not None else f'{a.index}: 미수신' for a in state.teleop.axes),
            client.last_outcome, client.last_ack, '', *KEY_HELP,
            ':' + prompt if prompt is not None else '']
        screen.erase()
        height, width = screen.getmaxyx()
        for row, line in enumerate(lines[:max(0, height - 1)]):
            try:
                screen.addnstr(row, 0, line, max(0, width - 1))
            except curses.error:
                pass  # A terminal resize can race this draw.
        screen.refresh()
        time.sleep(.02)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=29001)
    parser.add_argument('--token-file', type=Path, required=True)
    parser.add_argument('--telemetry-port', type=int, default=15707)
    parser.add_argument('--bench', action='store_true', help='기존 벤치 1~5축 기능 사용')
    parser.add_argument('--ignore-lock', action='store_true', help='명시적 벤치 조그의 볼락 UI 게이트 해제; 브리지 설정도 필요')
    args = parser.parse_args(argv)
    if not sys.stdin.isatty():
        parser.error('실제 터미널에서 실행하세요')
    if args.ignore_lock and not args.bench:
        parser.error('--ignore-lock requires --bench')
    if args.bench:
        os.environ['ARM_LOCAL_ALL_AXES'] = '1'
    if args.ignore_lock:
        os.environ['ARM_LOCAL_WRIST_IGNORE_LOCK'] = '1'
    stopping = [False]
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[sig] = signal.signal(sig, lambda *_: stopping.__setitem__(0, True))
    try:
        with ArmTerminalClient(args.host, args.port, args.token_file,
                               telemetry_port=args.telemetry_port) as client:
            curses.wrapper(terminal, client, stopping)
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
