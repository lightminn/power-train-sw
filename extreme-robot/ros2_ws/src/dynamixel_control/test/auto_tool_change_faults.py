"""설치된 ros2 run 프로세스에 실패만 주입한다. 생산 런타임에서 import하지 않는다."""

from std_msgs.msg import Bool
from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge as Bridge


def install(case):
    original_watchdog = Bridge._change_watchdog
    def watchdog(self):
        core = self._auto_change
        if self._auto_mode and core is not None:
            if case.startswith('abort:') and core.index == int(case.split(':')[1]):
                self._change_exit('ABORTED', f'injected stage abort {core.index}')
            elif case == 'permission' and core.index == 5:
                self._change_gate.mode = 'DRIVING'
            elif case == 'emergency' and core.index == 6:
                self._on_emergency_stop(Bool(data=True))
        return original_watchdog(self)
    Bridge._change_watchdog = watchdog

    original_dispatch = Bridge._dispatch_arm_trajectory
    def dispatch(self, msg):
        if case == 'undershoot' and self._auto_mode and self._auto_change.index == 1:
            return
        return original_dispatch(self, msg)
    Bridge._dispatch_arm_trajectory = dispatch

    original_feedback = Bridge._change_joint_states
    def feedback(self, msg):
        if case == 'stale' and self._auto_mode and self._auto_change.index >= 4:
            return
        return original_feedback(self, msg)
    Bridge._change_joint_states = feedback

    original_switch = Bridge._switch_tool_runtime
    def switch(self, tool_type):
        if self._auto_mode and case == 'absent':
            raise RuntimeError('injected pickup tool absent')
        result = original_switch(self, tool_type)
        if (self._auto_mode and case == 'ready' and self._auto_change.state == 'LOCK'
                and self.lock_sequence.state == 'CONFIRMED'):
            self._tool_preparation_state = 'FAILED'
        return result
    Bridge._switch_tool_runtime = switch
