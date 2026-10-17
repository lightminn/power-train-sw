"""Bounded local wrist jog lease. Host and container share a monotonic clock."""
import math


class WristJogLease:
    def __init__(self):
        self.sequence = -1
        self.deadline = None
        self._last_deadline = float('-inf')

    def accept(self, request, now, axes=(5,)):
        if request.get('axis') not in axes or type(request.get('axis')) is not int:
            raise ValueError(f'local jog supports only axes {axes}')
        sequence = request.get('sequence')
        if type(sequence) is not int or sequence < 0:
            raise ValueError('invalid wrist sequence')
        if request.get('command') == 'STOP':
            self.sequence = max(self.sequence, sequence)
            self.deadline = None
            return 0
        if request.get('command') != 'JOG':
            raise ValueError('unknown wrist command')
        deadline = request.get('expires_at_s')
        if (type(deadline) not in (int, float) or not math.isfinite(deadline)
                or not 0 < deadline - now <= 1.0):
            raise ValueError('expired or foreign-clock local wrist jog')
        direction = request.get('direction')
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError('invalid wrist direction')
        if (sequence < self.sequence or (sequence == self.sequence
                and deadline <= self._last_deadline)):
            raise ValueError('out-of-order wrist jog')
        self.sequence, self.deadline = sequence, deadline
        self._last_deadline = deadline
        return direction

    def expired(self, now):
        return self.deadline is not None and now >= self.deadline
