"""단일 목표 teach/replay 교체 코어. ROS와 모터 I/O는 호출자가 소유한다."""

from copy import deepcopy
import math
import time

from dynamixel_control.dock_poses import positive, validate_docks


class AutoToolChange:
    """새 관절 피드백으로 도달·정지를 확인한 다음에만 다음 단계를 발행한다."""

    def __init__(self, lock_sequence, move, hold, permission,
                 after_unlock=lambda: None, verify_tool=lambda: None, before_lock=lambda: None,
                 before_unlock=lambda: None,
                 clock=time.monotonic):
        self.lock_sequence = lock_sequence
        self.move = move
        self.hold = hold
        self.permission = permission
        self.after_unlock = after_unlock
        self.verify_tool = verify_tool
        self.before_lock = before_lock
        self.before_unlock = before_unlock
        self.clock = clock
        self.state = 'IDLE'
        self.error = ''
        self.steps = []
        self.index = -1
        self.feedback = None
        self.feedback_at = None
        self.feedback_generation = 0
        self._confirm = None
        self._lock_confirm = None
        self._lock_started = False

    @property
    def running(self):
        return self.state not in ('IDLE', 'DONE', 'ABORTED')

    def observe(self, positions, velocities):
        """호출자는 새 샘플만 전달한다. 불완전하거나 비유한 피드백은 거부한다."""
        if (not positions or set(positions) != set(velocities)
                or any(not math.isfinite(v) for v in (*positions.values(), *velocities.values()))):
            self.feedback = self.feedback_at = None
            if self.running:
                self.abort('invalid arm feedback')
            return
        self.feedback = dict(positions=positions.copy(), velocities=velocities.copy())
        self.feedback_at = self.clock()
        self.feedback_generation += 1

    def start(self, docks, release_slot, pickup_slot, tool_type, *, feedback_timeout_s,
              home_timeout_s, home_tolerance_rad, unlock_direction, lock_direction):
        if self.running:
            raise ValueError('a tool change is already running')
        self.permission()
        self._confirm = self._lock_confirm = None
        positive(feedback_timeout_s, 'feedback timeout')
        positive(home_timeout_s, 'home timeout')
        positive(home_tolerance_rad, 'home tolerance')
        if (type(unlock_direction) is not int or unlock_direction not in (-1, 1)
                or type(lock_direction) is not int or lock_direction not in (-1, 1)):
            raise ValueError('lock directions must be +1 or -1')
        self.feedback_timeout_s = feedback_timeout_s
        if self.feedback is None:
            raise ValueError('fresh complete arm feedback is required')
        self._check_feedback()
        names = list(self.feedback['positions'])
        validate_docks(docks, names)
        if release_slot == pickup_slot:
            raise ValueError('release and pickup slots must differ')
        release = deepcopy(docks['slots'][release_slot])
        pickup = deepcopy(docks['slots'][pickup_slot])
        if pickup['tool_type'] != tool_type:
            raise ValueError('pickup slot tool type mismatch')
        self.tool_type = tool_type
        self.release_slot, self.pickup_slot = release_slot, pickup_slot
        self.names = names
        self.directions = dict(UNLOCK=unlock_direction, LOCK=lock_direction)
        home = dict(positions=[self.feedback['positions'][n] for n in names],
                    timeout_s=home_timeout_s, tolerance_rad=home_tolerance_rad)
        self.steps = [
            ('TO_RELEASE_APPROACH', release['poses']['approach']),
            ('TO_RELEASE_SEAT', release['poses']['seat']),
            ('SEAT_CHECK', release['poses']['seat']),
            ('UNLOCK', None),
            ('RETREAT', release['poses']['retreat']),
            ('TO_PICKUP_APPROACH', pickup['poses']['approach']),
            ('TO_PICKUP_SEAT', pickup['poses']['seat']),
            ('SEAT_CHECK', pickup['poses']['seat']),
            ('LOCK', None),
            ('RETREAT', pickup['poses']['retreat']),
            ('TO_HOME', home),
        ]
        self.index = -1
        self.state, self.error = 'ARMED', ''

    def _check_feedback(self):
        if (self.feedback_at is None
                or not 0 <= self.clock() - self.feedback_at <= self.feedback_timeout_s):
            raise RuntimeError('arm feedback is stale')
        if hasattr(self, 'names') and set(self.feedback['positions']) != set(self.names):
            raise RuntimeError('incomplete arm feedback')

    def poll(self):
        if not self.running:
            return self.state
        try:
            self.permission()
            self._check_feedback()
            if self.state == 'ARMED':
                self._advance()
            elif self.state in ('UNLOCK', 'LOCK'):
                self._poll_lock()
            elif self.state == 'SEAT_CHECK':
                # 안착 재확인에 실패하면 잠금을 발행하지 않는다.
                if not self._arrived(require_new=False):
                    raise RuntimeError('seat target is no longer reached and stationary')
                self._advance()
            else:
                if self.clock() >= self.deadline:
                    raise RuntimeError(f'{self.state} move timed out')
                if self._arrived():
                    self._advance()
        except Exception as exc:
            self.abort(str(exc))
        return self.state

    def _advance(self):
        self.index += 1
        if self.index == len(self.steps):
            self.state = 'DONE'
            self._confirm = self._lock_confirm
            return
        self.state, self.target = self.steps[self.index]
        self._lock_started = False
        if self.state in ('UNLOCK', 'LOCK'):
            # 안착 확인과 다른 tick에서 시작해 중간 정지 요청을 받을 수 있게 한다.
            return
        self.deadline = self.clock() + self.target['timeout_s']
        if self.state != 'SEAT_CHECK':
            self.target_generation = self.feedback_generation
            self.move(self.names, list(self.target['positions']))

    def _arrived(self, require_new=True):
        if require_new and self.feedback_generation <= self.target_generation:
            return False
        return all(
            abs(self.feedback['positions'][name] - target) <= self.target['tolerance_rad']
            and self.feedback['velocities'][name] == 0
            for name, target in zip(self.names, self.target['positions']))

    def _poll_lock(self):
        if not self._lock_started:
            # 안착 확인 후 자세가 변했는지 잠금을 움직이기 직전에 재확인한다.
            seat = self.steps[self.index - 1][1]
            self.target = seat
            if not self._arrived(require_new=False):
                raise RuntimeError('seat target lost before lock command')
            if self.state == 'UNLOCK':
                self.before_unlock()
            if self.state == 'LOCK':
                self.before_lock()
                self.permission()
                self._check_feedback()
            desired = 'UNLOCKED' if self.state == 'UNLOCK' else 'LOCKED'
            self.lock_sequence.start(desired, self.directions[self.state])
            self._lock_started = True
        state = self.lock_sequence.poll()
        if state == 'ABORTED':
            raise RuntimeError(self.lock_sequence.error)
        if state == 'UNCHANGED':
            raise RuntimeError('lock did not perform the requested transition')
        if state != 'CONFIRMED':
            return
        confirm = self.lock_sequence.take_confirm()
        if confirm is None:
            raise RuntimeError('lock confirmation is missing')
        if self.state == 'UNLOCK':
            self.after_unlock()
        else:
            self.verify_tool()
            self._lock_confirm = confirm
        # I/O 검사 중에도 허가와 피드백이 노후화될 수 있다.
        self.permission()
        self._check_feedback()
        self._advance()

    def abort(self, reason):
        self._confirm = self._lock_confirm = None
        self.state, self.error = 'ABORTED', str(reason)
        try:
            self.lock_sequence.abort(reason)
            self.lock_sequence.take_confirm()
            self.lock_sequence.lock.stop()
        except Exception as exc:
            self.error += f'; lock stop failed: {exc}'
        try:
            self.hold()
        except Exception as exc:
            self.error += f'; arm hold failed: {exc}'
        return self.state

    def take_confirm(self):
        confirm, self._confirm = self._confirm, None
        return confirm
