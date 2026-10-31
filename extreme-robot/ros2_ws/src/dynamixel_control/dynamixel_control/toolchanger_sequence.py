"""자동 도구교체 시퀀스. 결합 판정은 ID1 부하 프록시이며 결합 센서가 아니다."""

import time


class SequenceError(RuntimeError):
    """하드웨어 쓰기를 시도하기 전에 거부된 시퀀스 요청."""


class ToolchangerSequence:
    """ToolchangerLock 위에 올린 한 번의 교체 시퀀스.

    레퍼런스 OSrobotToolChanger의 changeStatus 흐름을 단계별로 분리한다:
    도구 정지 확인 → 잠금 이동 → 도달 검증 → 결합 프록시 평가 → confirm 발행.
    이동이 시작된 뒤의 실패는 모두 ID1 토크를 끄고 confirm을 발행하지 않는다.

    레퍼런스는 근접센서로 장착을 직접 확인하지만 이 기구에는 결합 센서가 없다.
    대신 잠금 스트로크 동안의 ID1 Present Load 최대값을 프록시로 쓴다.
    프록시는 결합 센서 확인을 대신하지 않으므로 mechanically_confirmed는 계속 false다.
    """

    def __init__(self, lock, prepare_tool, load_threshold=0, clock=time.monotonic):
        """load_threshold가 0 이하면 결합 부하 프록시를 쓰지 않는다."""
        self.lock = lock
        self.prepare_tool = prepare_tool
        self.load_threshold = int(load_threshold)
        self.clock = clock
        self.state = 'IDLE'
        self.desired = None
        self.direction = None
        self.peak_load = None
        self.coupling_proxy = 'UNKNOWN'
        self.error = ''
        self._confirm = None

    def start(self, desired, direction=1):
        """도구 정지 확인 뒤 한 단계 잠금 이동을 시작한다.

        이동을 걸기 전의 거부는 ID1에 아무 쓰기도 하지 않고 잠금 상태를 유지한다.
        """
        if self.state in ('TOOL_STOP', 'MOVING'):
            raise SequenceError('a tool change is already running')
        if desired not in ('LOCKED', 'UNLOCKED'):
            raise SequenceError('CHANGE requires to=LOCKED or UNLOCKED')
        self._confirm = None
        self.peak_load = None
        self.coupling_proxy = 'UNKNOWN'
        self.state = 'TOOL_STOP'
        try:
            self.prepare_tool()
            self.lock.move(desired, direction)
        except Exception as exc:
            self.state = 'IDLE'
            self.desired = self.direction = None
            self.error = str(exc)
            raise SequenceError(str(exc)) from exc
        self.desired = desired
        self.direction = direction
        self.error = ''
        if self.lock.target is None:
            # 이미 원하는 상태였다. lock.move는 이동하지 않았으므로 confirm도 없다.
            self.coupling_proxy = 'UNCHANGED'
            self.state = 'UNCHANGED'
            self.desired = self.direction = None
            return self.state
        self.state = 'MOVING'
        return self.state

    def poll(self):
        """잠금 observe 직후에 호출한다. 이동 완료/실패를 판정한다."""
        if self.state != 'MOVING':
            return self.state
        if self.lock.sample is None or self.lock.state == 'FAULT':
            return self._abort(f'lock feedback fault during change: {self.lock.error}')
        load = self.lock.sample.get('load')
        if load is not None:
            self.peak_load = max(self.peak_load or 0, abs(int(load)))
        if self.lock.state == 'MOVING':
            return self.state
        if self.lock.state != self.desired:
            return self._abort(
                f'lock settled in {self.lock.state} instead of {self.desired}')
        return self._finish()

    def _finish(self):
        if self.load_threshold <= 0:
            self.coupling_proxy = 'DISABLED'
        elif self.peak_load is None:
            self.coupling_proxy = 'UNAVAILABLE'
        elif self.desired == 'UNLOCKED':
            # 분리는 자유 회전으로도 끝날 수 있어 부하로 통과/실패를 가르지 않는다.
            self.coupling_proxy = 'RELEASED'
        elif self.peak_load >= self.load_threshold:
            self.coupling_proxy = 'ENGAGED'
        else:
            self.coupling_proxy = 'NOT_ENGAGED'
        if self.desired == 'LOCKED' and self.coupling_proxy in (
                'NOT_ENGAGED', 'UNAVAILABLE'):
            return self._abort(
                f'coupling load proxy {self.coupling_proxy}: peak {self.peak_load} '
                f'below threshold {self.load_threshold}')
        self.state = 'CONFIRMED'
        self.error = ''
        self._confirm = {'state': self.desired, 'direction': self.direction,
                         'coupling_proxy': self.coupling_proxy,
                         'peak_load': self.peak_load,
                         'load_threshold': self.load_threshold,
                         'mechanically_confirmed': False,
                         'at': self.clock()}
        return self.state

    def _abort(self, reason):
        self.error = str(reason)
        self._confirm = None
        self.state = 'ABORTED'
        self.desired = self.direction = None
        try:
            self.lock.stop()
        except Exception as exc:
            self.error += f'; lock stop failed: {exc}'
        return self.state

    def abort(self, reason):
        """비상정지·도구분리·종료에서 진행 중인 시퀀스를 닫는다."""
        if self.state in ('TOOL_STOP', 'MOVING'):
            return self._abort(reason)
        return self.state

    def take_confirm(self):
        """성공 confirm을 1회만 반환한다. 레퍼런스의 confirm 릴레이에 대응한다."""
        payload = self._confirm
        self._confirm = None
        return payload

    def snapshot(self):
        """상태 발행용 시퀀스 요약. 잠금 snapshot과 합쳐서 쓴다."""
        return {'sequence_state': self.state, 'sequence_desired': self.desired,
                'sequence_direction': self.direction,
                'coupling_proxy': self.coupling_proxy,
                'coupling_peak_load': self.peak_load,
                'coupling_load_threshold': self.load_threshold,
                'sequence_error': self.error}
