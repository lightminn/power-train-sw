"""Preserve DDS receipt metadata for motion subscriptions on ROS 2 Humble.

Humble rclpy 3.3.x discards take_message()[1] in its default executor. Keep that
RMW metadata only for explicitly tagged commands; all other callbacks retain
normal executor handling. Missing receipt metadata never refreshes a command.
"""
from rclpy.executors import SingleThreadedExecutor, await_or_execute


class ReceiptTimeCallback:
    def __init__(self, callback):
        self.callback = callback

    def __call__(self, message, info=None):
        # A default executor or incomplete middleware may omit info. Fail closed.
        if info is not None:
            return self.callback(message, info)


class ReceiptTimeExecutor(SingleThreadedExecutor):
    def _take_subscription(self, sub):
        if not isinstance(sub.callback, ReceiptTimeCallback):
            return super()._take_subscription(sub)
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, message):
        if not isinstance(sub.callback, ReceiptTimeCallback):
            return await super()._execute_subscription(sub, message)
        if message is not None:
            await await_or_execute(sub.callback, message[0], message[1])
