"""교체 세션의 명시적 무장과 차체 정지 허가. ROS·모터 쓰기는 어댑터 소유다."""

import math
import time

from dynamixel_control.contract import MODE_MISSION_STOP


class ToolChangeDenied(RuntimeError):
    """교체 권한을 얻지 못한 요청."""


class ToolChangeInterlock:
    """무장은 한 세션에만 유효하며 허가를 잃으면 재무장을 요구한다."""

    def __init__(self, timeout=1.0, clock=time.monotonic):
        # 기존 arm_fsm의 chassis_mode_timeout과 같은 1초 계약이다.
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('chassis timeout must be finite and positive')
        self.timeout = timeout
        self.clock = clock
        self.active = False
        self.mode = None
        self.stamp = None
        self.received = None
        self.error = ''

    def observe(self, mode, stamp, now_ros):
        valid = (math.isfinite(stamp) and stamp > 0
                 and 0 <= now_ros - stamp <= self.timeout
                 and (self.stamp is None or stamp > self.stamp))
        if not valid:
            self.mode = None
            return False
        self.mode, self.stamp, self.received = mode, stamp, self.clock()
        return True

    def check(self, emergency, docks_valid, now_ros):
        if emergency:
            raise ToolChangeDenied('emergency stop is active')
        if not docks_valid:
            raise ToolChangeDenied('dock poses are not validated')
        if (self.mode != MODE_MISSION_STOP or self.received is None
                or not 0 <= self.clock() - self.received <= self.timeout
                or not 0 <= now_ros - self.stamp <= self.timeout):
            raise ToolChangeDenied('fresh MISSION_STOP is required')

    def arm(self, **conditions):
        if self.active:
            raise ToolChangeDenied('tool change session already armed')
        self.check(**conditions)
        self.active = True
        self.error = ''

    def require_active(self, **conditions):
        if not self.active:
            raise ToolChangeDenied('explicit operator arm is required')
        self.check(**conditions)

    def exit(self, reason=''):
        self.active = False
        self.error = reason
