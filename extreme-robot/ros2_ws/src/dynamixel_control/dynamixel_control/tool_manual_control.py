"""Profile-bound manual tool commands with a receiver-owned jog lease."""
import math
import time


class ToolManualControl:
    def __init__(self, bridge, *, clock=time.monotonic, message_clock=None):
        self.bridge = bridge
        self.clock = clock
        self.message_clock = message_clock or clock
        self.generation = None
        self.leases = {}
        self.sequences = {}

    def _dispatch(self, command, target):
        b = self.bridge
        if command == 'HOLD' and b.tool_type != 'cleaner' and hasattr(b, 'hold_tool_position'):
            ids = {'left': (3,), 'right': (4,), 'both': (3, 4), 'single': (5,)}[target]
            if b.hold_tool_position(ids):
                b.tool_fsm.state = type(b.tool_fsm.state).READY
            return
        if b.tool_type == 'dual_motor_gripper':
            return b.tool_fsm.command(command, target=target)
        if b.tool_type == 'cleaner':
            command = {'JOG_OPEN': 'LEFT', 'JOG_CLOSE': 'RIGHT', 'HOLD': 'STOP',
                       'DISABLE': 'STOP'}.get(command, command)
        return b.tool_fsm.command(command)

    def handle(self, request):
        b = self.bridge
        if not isinstance(request, dict):
            raise ValueError('tool request must be an object')
        generation = request.get('tool_context_generation')
        if (type(generation) is not int or generation != b._tool_context_generation
                or request.get('tool_type') != b.tool_type):
            raise ValueError('replaced tool context')
        if generation != self.generation:
            self.generation = generation
            self.leases.clear()
            self.sequences.clear()
        target = request.get('target')
        targets = {'dual_motor_gripper': ('left', 'right', 'both'),
                   'spur_1motor_gripper': ('single',), 'cleaner': ('single',)}
        if target not in targets.get(b.tool_type, ()):
            raise ValueError('target does not belong to active tool')
        stamp = request.get('sent_at_s')
        if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                or not -0.1 <= self.message_clock() - stamp <= 1.0):
            raise ValueError('stale tool request')
        command = request.get('command')
        if command not in ('JOG', 'HOLD', 'OPEN', 'CLOSE', 'STOP', 'DISABLE', 'LEFT', 'RIGHT'):
            raise ValueError('unknown manual tool command')
        if b.read_only or b.tool_fsm is None:
            raise RuntimeError('tool is read-only or unavailable')
        if command not in ('HOLD', 'STOP', 'DISABLE') and (
                b.control_scope not in ('FULL_ROBOT', 'END_EFFECTOR_ONLY')
                or b.control_mode != 'MANUAL' or b.emergency_stop_active
                or b.tool_detached or not b.tool_discovered):
            raise RuntimeError('manual tool control is not available')
        if command == 'JOG':
            direction, ttl, sequence = (request.get(k) for k in ('direction', 'ttl_s', 'sequence'))
            if (type(direction) is not int or direction not in (-1, 1)
                    or type(ttl) not in (int, float) or not math.isfinite(ttl)
                    or not 0 < ttl <= 0.6 or type(sequence) is not int or sequence < 0):
                raise ValueError('invalid tool jog lease')
            valid_until = request.get('valid_until_s')
            if type(valid_until) not in (int, float) or not math.isfinite(valid_until):
                raise ValueError('missing ROS tool deadline')
            ttl = min(ttl, valid_until - self.message_clock())
            if ttl <= 0:
                raise ValueError('tool jog expired in ROS queue')
            deadline = self.clock() + ttl
            source = request.get('source_id', 'legacy')
            if not isinstance(source, str) or not 1 <= len(source) <= 160:
                raise ValueError('invalid command source')
            previous = self.sequences.get(target, (-1, float('-inf'), None))
            if (stamp <= previous[1] or source == previous[2] and sequence < previous[0]):
                raise ValueError('out-of-order tool jog')
            # A side takes over from a pair, or a pair takes over from both sides.
            for other in tuple(self.leases):
                if other != target and (other == 'both' or target == 'both'):
                    self._dispatch('HOLD', other)
                    del self.leases[other]
            self._dispatch('JOG_OPEN' if direction > 0 else 'JOG_CLOSE', target)
            self.sequences[target] = sequence, stamp, source
            self.leases[target] = deadline
        else:
            sequence = request.get('sequence')
            if command == 'HOLD' and type(sequence) is int:
                source = request.get('source_id', 'legacy')
                previous = self.sequences.get(target, (-1, float('-inf'), None))
                if stamp < previous[1] or source == previous[2] and sequence < previous[0]:
                    return
                self.sequences[target] = sequence, stamp, source
            self._dispatch(command, target)
            for other in tuple(self.leases):
                if other == target or other == 'both' or target == 'both':
                    del self.leases[other]

    def poll(self):
        b = self.bridge
        if self.generation != b._tool_context_generation:
            self.leases.clear()
            return
        for target, deadline in tuple(self.leases.items()):
            if (self.clock() >= deadline or b.control_mode != 'MANUAL'
                    or b.emergency_stop_active or b.tool_detached or not b.tool_discovered):
                del self.leases[target]
                self._dispatch('HOLD', target)
