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
    def __init__(self): super().__init__(); self.state=SimpleNamespace(value="streaming")
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


def test_component_that_fails_inside_start_is_rolled_back():
    failed=Part(fail=True); gateway, _=make_gateway(source=failed)
    try: gateway.start()
    except RuntimeError: pass
    assert failed.started == failed.stopped == 1


def test_crashed_streamer_is_reaped_before_replacement_and_at_cleanup():
    old=Streamer(); old.running=False
    new=Streamer(); gateway, _=make_gateway(streamer=old)
    gateway._streamer_factory=lambda: new
    gateway.start(); gateway.observe(); gateway._set_streaming(True)
    assert old.stopped >= 1 and new.started == 1
    gateway.shutdown()
    assert new.stopped == 1


def test_run_once_cannot_overlap_cleanup():
    entered=threading.Event(); release=threading.Event()
    class BlockingSource(Source):
        def poll_latest(self): entered.set(); release.wait(); return SimpleNamespace(empty=True)
    gateway, parts=make_gateway(source=BlockingSource()); gateway.start()
    runner=threading.Thread(target=gateway.run_once); runner.start(); assert entered.wait(1)
    stopper=threading.Thread(target=gateway.shutdown); stopper.start()
    assert parts["source"].stopped == 0
    release.set(); runner.join(1); stopper.join(1)
    assert parts["source"].stopped == 1


def test_connecting_is_starting_and_status_contract_is_complete():
    source=Source(); source.state=SimpleNamespace(value="connecting")
    source.config=SimpleNamespace(color_width=1280,color_height=720,
                                  depth_width=640,depth_height=480,fps=30)
    metric=SimpleNamespace(fps=30.0,age_s=.01,max_gap_s=.04,nonincreasing_count=0)
    diagnostics=SimpleNamespace(snapshot=lambda _: SimpleNamespace(topics={"color":metric}))
    gateway, _=make_gateway(source=source)
    gateway._diagnostics=diagnostics; gateway._system_collector=lambda: {"cpu":1,"ram":2}
    gateway.start(); assert gateway.state is GatewayState.STARTING
    status=gateway.status_snapshot()
    assert status["sdk"]["serial"] == "00000000F0271544"
    assert status["sdk"]["profile"]["color"] == [1280,720,30]
    assert set(status) == {"state","sdk","diagnostics","ros_publish_counts","srt","system","last_error"}
    assert status["diagnostics"]["color"]["fps"] == 30.0
    assert status["system"] == {"cpu":1,"ram":2}
    gateway.shutdown()
