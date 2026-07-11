import threading
from types import SimpleNamespace

from l515_dashboard.gateway import Gateway, GatewayState, SystemCollector


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
    assert status["sdk"]["serial"] is None
    assert status["sdk"]["expected_serial"] == "00000000F0271544"
    assert status["sdk"]["profile"] is None
    assert set(status) == {"state","sdk","diagnostics","ros_publish_counts","srt","system","last_error"}
    assert status["diagnostics"]["color"]["fps"] == 30.0
    assert status["system"] == {"cpu":1,"ram":2}
    gateway.shutdown()


def test_streamer_submit_and_snapshot_exceptions_are_isolated():
    class BadStreamer(Streamer):
        def submit_color(self, _): raise RuntimeError("submit broke")
    class Frame:
        empty=False; raw_depth=aligned_depth=gyro=accel=None
        raw_color=SimpleNamespace(get_data=lambda: __import__("numpy").zeros((1,1,3),dtype="uint8"), get_timestamp=lambda:1)
    source=Source(); source.poll_latest=lambda: Frame()
    ros=Part(); ros.publish=lambda _: ("/l515/color/image_raw",)
    gateway, parts=make_gateway(source=source,ros=ros,streamer=BadStreamer()); gateway.start(); gateway.run_once()
    assert gateway.state is GatewayState.DEGRADED and parts["source"].stopped == 0
    assert "submit broke" in gateway.last_error
    gateway.shutdown()

    bad=Streamer(); bad.snapshot=lambda: (_ for _ in ()).throw(RuntimeError("snapshot broke"))
    gateway, parts=make_gateway(streamer=bad); gateway.start()
    assert gateway.state is GatewayState.DEGRADED
    assert "snapshot broke" in gateway.last_error and parts["source"].stopped == 0
    gateway.shutdown()


def test_successful_stream_restart_clears_recoverable_error_not_fatal():
    old=Streamer(); old.running=False
    old.snapshot=lambda: SimpleNamespace(running=False,mode=None,sent=0,dropped=0,last_error="gst died")
    new=Streamer(); gateway,_=make_gateway(streamer=old); gateway._streamer_factory=lambda:new
    gateway.start(); gateway.observe(); assert gateway.last_error == "gst died"
    gateway._set_streaming(True)
    assert gateway.last_error is None and gateway.fatal_error is None
    gateway.shutdown()


def test_system_collector_reports_current_cpu_and_rss():
    times=iter([0.0,1.0]); cpus=iter([0.0,.25])
    collector=SystemCollector(monotonic=lambda:next(times), process_time=lambda:next(cpus))
    snapshot=collector()
    assert snapshot["cpu_percent"] == 25.0
    assert snapshot["current_rss_bytes"] is None or snapshot["current_rss_bytes"] > 0


def test_restart_source_stop_failure_faults_and_cleans_every_resource():
    class BadStopSource(Source):
        def stop(self): self.stopped += 1; raise RuntimeError("source stop failed")
    source=BadStopSource(); gateway,parts=make_gateway(source=source)
    gateway.start()
    try: gateway.restart_components()
    except RuntimeError: pass
    assert gateway.state is GatewayState.FAULT
    assert gateway.fatal_error == "source stop failed"
    assert source.stopped >= 2
    assert parts["streamer"].stopped >= 1
    assert parts["ros"].stopped == parts["server"].stopped == parts["guard"].stopped == 1


def test_cleanup_does_not_hold_lifecycle_lock_while_server_stop_blocks():
    entered=threading.Event(); release=threading.Event()
    class BlockingServer(Part):
        def stop(self):
            self.stopped += 1; entered.set(); release.wait()
    gateway, _=make_gateway(server=BlockingServer()); gateway.start()
    stopper=threading.Thread(target=gateway.shutdown); stopper.start(); assert entered.wait(1)
    acquired=gateway._lock.acquire(timeout=.2)
    assert acquired
    gateway._lock.release(); release.set(); stopper.join(1)
    assert not stopper.is_alive() and gateway.state is GatewayState.STOPPED


def test_cleanup_during_restart_teardown_prevents_every_later_start():
    entered=threading.Event(); release=threading.Event()
    class BarrierSource(Source):
        def stop(self):
            self.stopped += 1
            if self.stopped == 1:
                entered.set(); release.wait()
    source=BarrierSource(); gateway,parts=make_gateway(source=source)
    gateway._streamer_factory=lambda: Streamer()
    gateway.start()
    restart=threading.Thread(target=gateway.restart_components); restart.start()
    assert entered.wait(1)
    cleanup=threading.Thread(target=gateway.shutdown); cleanup.start()
    for _ in range(100):
        if gateway.shutdown_requested: break
        threading.Event().wait(.002)
    release.set(); restart.join(1); cleanup.join(1)
    assert not restart.is_alive() and not cleanup.is_alive()
    assert source.started == 1 and parts["ros"].started == 1
    assert gateway.state is GatewayState.STOPPED
    assert gateway._owned == []
    assert all(part.stopped >= 1 for part in parts.values())


def test_fatal_request_during_normal_cleanup_dominates_stopped():
    entered=threading.Event(); release=threading.Event()
    class BarrierStreamer(Streamer):
        def stop(self): self.stopped += 1; entered.set(); release.wait()
    gateway,parts=make_gateway(streamer=BarrierStreamer()); gateway.start()
    normal=threading.Thread(target=gateway.shutdown); normal.start(); assert entered.wait(1)
    fatal=threading.Thread(target=gateway.ros_fatal,args=(RuntimeError("late fatal"),))
    fatal.start()
    for _ in range(100):
        if gateway.fatal_error: break
        threading.Event().wait(.002)
    release.set(); normal.join(1); fatal.join(1)
    assert not normal.is_alive() and not fatal.is_alive()
    assert gateway.state is GatewayState.FAULT
    assert gateway.fatal_error == "late fatal"
    assert gateway._owned == []
    assert parts["source"].stopped == parts["ros"].stopped == parts["server"].stopped == parts["guard"].stopped == 1
