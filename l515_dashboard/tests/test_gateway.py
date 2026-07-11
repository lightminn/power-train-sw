import threading
from types import SimpleNamespace

from l515_dashboard.gateway import Gateway, GatewayState


class Part:
    def __init__(self, *, fail=False): self.started=0; self.stopped=0; self.fail=fail
    def start(self):
        self.started += 1
        if self.fail: raise RuntimeError("start failed")
    def stop(self): self.stopped += 1


class Source(Part):
    state = SimpleNamespace(value="streaming")
    def poll_latest(self): return SimpleNamespace(empty=True)


class Streamer(Part):
    def __init__(self): super().__init__(); self.mode=None; self.running=True
    def set_mode(self, mode): self.mode=mode
    def snapshot(self): return SimpleNamespace(running=self.running, mode=self.mode, sent=0, dropped=0, last_error=None)


def make_gateway(**overrides):
    parts = dict(guard=Part(), source=Source(), ros=Part(), streamer=Streamer(), server=Part())
    parts.update(overrides)
    return Gateway(**parts), parts


def test_lifecycle_and_idempotent_cleanup_order():
    order=[]
    class Ordered(Part):
        def __init__(self, name): super().__init__(); self.name=name
        def stop(self): super().stop(); order.append(self.name)
    parts={name: Ordered(name) for name in ("source","streamer","ros","server","guard")}
    gateway=Gateway(**parts); gateway.start(); gateway.shutdown(); gateway.shutdown()
    assert gateway.state is GatewayState.STOPPED
    assert order == ["streamer", "source", "ros", "server", "guard"]
    assert all(part.stopped == 1 for part in parts.values())


def test_partial_start_is_cleaned_and_faulted():
    gateway, parts = make_gateway(ros=Part(fail=True))
    try: gateway.start()
    except RuntimeError: pass
    assert gateway.state is GatewayState.FAULT
    assert parts["source"].stopped == parts["guard"].stopped == 1


def test_optional_streamer_start_failure_is_degraded_not_fatal():
    gateway, parts = make_gateway(streamer=Part(fail=True))
    gateway.start()
    assert gateway.state is GatewayState.DEGRADED
    assert parts["server"].started == 1
    gateway.shutdown()


def test_commands_are_serialized_and_dashboard_disconnect_is_noop():
    gateway, parts = make_gateway(); gateway.start()
    threads=[threading.Thread(target=gateway.handle_request, args=({"type":"set_video_mode", "payload":{"mode":"depth"}},)) for _ in range(8)]
    for thread in threads: thread.start()
    for thread in threads: thread.join()
    assert parts["streamer"].mode.value == "depth"
    gateway.client_disconnected(); assert gateway.state is GatewayState.RUNNING
    gateway.shutdown()


def test_source_loss_and_streamer_failure_degrade_but_ros_fatal_faults():
    gateway, parts = make_gateway(); gateway.start()
    gateway.observe(); parts["source"].state=SimpleNamespace(value="disconnected"); gateway.observe()
    assert gateway.state is GatewayState.DEGRADED
    parts["source"].state=SimpleNamespace(value="streaming"); parts["streamer"].running=False; gateway.observe()
    assert gateway.state is GatewayState.DEGRADED
    gateway.ros_fatal(RuntimeError("ROS died"))
    assert gateway.state is GatewayState.FAULT
    assert all(part.stopped == 1 for part in parts.values())
