"""Executable headless Gateway: ``python3 -m l515_dashboard.gateway_main``."""

import signal
import time

from .config import DashboardConfig
from .control_server import UnixControlServer
from .gateway import Gateway, GatewayState
from .gateway_ros import GatewayRosPublisher
from .gateway_source import L515GatewaySource
from .resource_guard import ResourceGuard
from .streamer import SrtStreamer


class RosRuntime:
    def __init__(self):
        import rclpy
        self._rclpy=rclpy; self.node=None; self.publisher=None

    def start(self):
        self._rclpy.init(args=None)
        self.node=self._rclpy.create_node("l515_gateway")
        self.publisher=GatewayRosPublisher(self.node)

    def publish(self, frames):
        published=self.publisher.publish(frames)
        self._rclpy.spin_once(self.node, timeout_sec=0.0)
        return published

    def publish_counts(self):
        return self.publisher.publish_counts() if self.publisher else {}

    def stop(self):
        if self.node is not None: self.node.destroy_node(); self.node=None
        if self._rclpy.ok(): self._rclpy.shutdown()


def build_gateway(config=None):
    import pyrealsense2 as rs
    config=config or DashboardConfig()
    guard=ResourceGuard(config.lock_path, config.socket_path)
    source=L515GatewaySource(rs, config)
    ros=RosRuntime()
    factory=lambda: SrtStreamer(config)
    gateway=Gateway(guard=guard, source=source, ros=ros, streamer=factory(), streamer_factory=factory)
    gateway.server=UnixControlServer(config.socket_path, gateway.handle_request,
        max_message_bytes=config.max_message_bytes,
        on_disconnect=gateway.client_disconnected).require_owner(guard)
    return gateway


def main():
    gateway=build_gateway()
    stopping=False
    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping=True
    old={sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        gateway.start()
        while not stopping and gateway.state not in (GatewayState.STOPPED, GatewayState.FAULT):
            gateway.run_once(); time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        gateway.last_error=str(exc)
        gateway.ros_fatal(exc)
        raise
    finally:
        gateway.shutdown()
        for sig, handler in old.items(): signal.signal(sig, handler)
    return 1 if gateway.fatal_error else 0


if __name__ == "__main__":
    raise SystemExit(main())
