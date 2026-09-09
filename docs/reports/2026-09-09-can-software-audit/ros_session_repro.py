"""Host-only ROS/session audit probes against the current source tree.

No ROS node, socket, device, or subprocess is started. The ROS methods are
extracted from the current production source AST, because rclpy is unavailable
on the audit host. SimpleNamespace objects are injected fixtures, not actual
DDS messages. The accept error is injected by a fake listener.

Run from the repository root with:
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:motor_control:ros2/src/powertrain_ros \
      python docs/reports/2026-09-09-can-software-audit/ros_session_repro.py

These probes reproduce narrow source behavior. They do not establish that a
fake publisher, delayed DDS message, or ECONNABORTED occurred on the Jetson,
and none explains persistent physical ODrive heartbeat loss after shutdown.
"""

import ast
import errno
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS


def method(path, cls_name, meth_name, env):
    tree = ast.parse(Path(path).read_text())
    cls = next(
        n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls_name
    )
    fn = next(
        n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == meth_name
    )
    # Remove ROS type annotations only; the method body remains unchanged.
    fn.returns = None
    for arg in fn.args.args:
        arg.annotation = None
    module = ast.fix_missing_locations(ast.Module(body=[fn], type_ignores=[]))
    exec(compile(module, path, "exec"), env)
    return env[meth_name]


def main():
    base = "ros2/src/powertrain_ros/powertrain_ros/"
    on_wheels = method(
        base + "ops_broker_node.py", "OpsBrokerNode", "_on_wheels",
        dict(math=math, time=time, WHEEL_STOP_TURNS=.1),
    )
    node = NS(_state_lock=threading.Lock(), _fields={}, _stamps={})
    fake = NS(
        chassis_mode="FAKE", healthy=True,
        wheels=[
            NS(name=n, drive_turns_per_s=0., drive_stale=False, drive_axis_error=0)
            for n in ["FL", "FR", "ML", "MR", "RL", "RR"]
        ],
    )
    on_wheels(node, fake)
    assert node._fields['wheels_stopped'] is True
    print("Non-owner FAKE wheel snapshot accepted as wheels_stopped:",
          node._fields["wheels_stopped"])

    captured = []
    on_command = method(
        base + "chassis_node.py", "ChassisNode", "_on_authority_cmd", {},
    )
    node = NS(
        _authority=NS(submit=lambda *args: captured.append(args)),
        _now_ms=lambda: 10500.,
    )
    # A Twist has no source timestamp. This invocation simulates delivery after
    # a stall; no real ROS queue or executor is executed by this probe.
    on_command(node, "manual", NS(linear=NS(x=.4), angular=NS(z=0.)))
    assert captured == [('manual', .4, 0., 10.5)]
    print("Twist delivered after executor stall gets current timestamp:", captured[0])

    from powertrain_runtime.session import SessionServer

    class TransientListener:
        calls = 0

        def accept(self):
            self.calls += 1
            raise OSError(errno.ECONNABORTED, "transient aborted connection")

    listener = TransientListener()
    SessionServer._accept(NS(_done=threading.Event()), listener, "input")
    assert listener.calls == 1
    print("Session listener transient OSError returned permanently; accept calls:",
          listener.calls)


if __name__ == "__main__":
    main()
