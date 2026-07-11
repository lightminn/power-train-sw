from types import SimpleNamespace
import threading
import time
import sys

from l515_dashboard.gateway_source import (
    EXPECTED_L515_SERIAL, L515GatewaySource, GatewayFrames,
)
from l515_dashboard.config import DashboardConfig


class Sample:
    def __init__(self, name, ts): self.name, self.ts = name, ts
    def get_timestamp(self): return self.ts

class Frames:
    def __init__(self, tag, ts):
        self.color=Sample("color-"+tag,ts); self.depth=Sample("raw-"+tag,ts)
        self.accel=Sample("accel-"+tag,ts); self.gyro=Sample("gyro-"+tag,ts)
    def get_color_frame(self): return self.color
    def get_depth_frame(self): return self.depth
    def first_or_default(self, stream): return getattr(self, stream)

class Aligned:
    def __init__(self, original): self.depth=Sample("aligned-"+original.color.name, original.depth.ts)
    def get_depth_frame(self): return self.depth

class Pipeline:
    def __init__(self, rs): self.rs=rs; self.stopped=False
    def start(self, config): self.rs.started.append(config)
    def wait_for_frames(self):
        item=self.rs.results.pop(0)
        if isinstance(item, BaseException): raise item
        return item
    def stop(self): self.stopped=True

class Config:
    def __init__(self): self.serial=None; self.streams=[]
    def enable_device(self,s): self.serial=s
    def enable_stream(self,*args): self.streams.append(args)

class RS:
    stream=SimpleNamespace(color="color",depth="depth",accel="accel",gyro="gyro")
    format=SimpleNamespace(bgr8="bgr8",z16="z16")
    camera_info=SimpleNamespace(serial_number="serial")
    config=Config
    def __init__(self, results): self.results=list(results); self.started=[]; self.pipelines=[]; self.align_targets=[]
    def context(self):
        dev=SimpleNamespace(get_info=lambda _: EXPECTED_L515_SERIAL)
        return SimpleNamespace(query_devices=lambda:[dev])
    def pipeline(self): p=Pipeline(self); self.pipelines.append(p); return p
    def align(self,target):
        self.align_targets.append(target)
        return SimpleNamespace(process=lambda frames: Aligned(frames))

def test_gateway_frames_latest_slot_and_empty():
    slot=GatewayFrames(); slot.put(raw_color="old"); slot.put(raw_color="new", raw_depth="d")
    assert slot.drain().raw_color == "new" and slot.drain().empty

def test_exact_profiles_alignment_raw_separation_imu_and_mapper():
    rs=RS([Frames("one",1)])
    source=L515GatewaySource(rs, wait_fn=lambda _:False, mapper_factory=object)
    original=source._latest.put
    def put_then_stop(**payload):
        original(**payload); source._stop_event.set()
    source._latest.put=put_then_stop
    source._run()
    assert rs.started[0].streams == [("color",1280,720,"bgr8",30),("depth",640,480,"z16",30),("accel",),("gyro",)]
    assert rs.align_targets == ["color"]
    out=source.poll_latest()
    assert out.raw_color.name == "color-one" and out.raw_depth.name == "raw-one"
    assert out.aligned_depth.name == "aligned-color-one"
    assert out.accel.name == "accel-one" and out.gyro.name == "gyro-one" and out.mapper is not None


def test_source_consumes_dashboard_config_profiles_and_reconnect_interval():
    config = DashboardConfig(reconnect_interval_s=0.125)
    waits=[]; rs=RS([RuntimeError("drop")])
    source=L515GatewaySource(rs, config=config, wait_fn=lambda value: waits.append(value) or False, mapper_factory=object)
    source._run()
    assert waits == [0.125]
    assert rs.started[0].streams[:2] == [("color",1280,720,"bgr8",30),("depth",640,480,"z16",30)]


def test_default_mapper_is_real_timestamp_mapper(monkeypatch):
    sentinel=object()
    monkeypatch.setitem(
        sys.modules, "powertrain_ros.l515_adapter",
        SimpleNamespace(TimestampMapper=lambda: sentinel),
    )
    from l515_dashboard.gateway_source import _new_timestamp_mapper
    assert _new_timestamp_mapper() is sentinel

def test_dedup_and_reconnect_reset_allow_same_timestamps_in_new_session():
    rs=RS([Frames("old",4), Frames("duplicate",4), RuntimeError("drop"), Frames("new",4)])
    source=L515GatewaySource(rs, wait_fn=lambda _:True, mapper_factory=object)
    original=source._latest.put; commits=[]
    def put(**payload):
        commits.append(payload); original(**payload)
        if len(commits)==3: source._stop_event.set()
    source._latest.put=put
    source._run()
    assert commits[1]["raw_color"] is None and commits[1]["aligned_depth"] is None
    assert commits[2]["raw_color"].name == "color-new"
    assert source.poll_latest().raw_color.name == "color-new"

def test_stop_is_bounded_when_sdk_stop_blocks():
    rs=RS([]); source=L515GatewaySource(rs, stop_timeout=0.01)
    blocker=SimpleNamespace(stop=lambda: __import__("time").sleep(1))
    source._pipeline=blocker
    start=__import__("time").monotonic(); source.stop()
    assert __import__("time").monotonic()-start < .1


def test_late_worker_cannot_enqueue_or_regress_state_after_stop():
    ready, release = threading.Event(), threading.Event()
    class LatePipeline(Pipeline):
        def wait_for_frames(self): ready.set(); release.wait(); return Frames("late",1)
    rs=RS([]); rs.pipeline=lambda: LatePipeline(rs)
    source=L515GatewaySource(rs, stop_timeout=.01, mapper_factory=object)
    source.start(); assert ready.wait(.2); source.stop(); worker=source._thread
    release.set(); worker.join(.2)
    assert source.state.value == "stopped" and source.poll_latest().empty


def test_stop_immediately_before_native_start_prevents_sdk_start():
    ready, release=threading.Event(), threading.Event(); rs=RS([])
    source=L515GatewaySource(rs, stop_timeout=.01, mapper_factory=object)
    source._before_pipeline_start=lambda: (ready.set(), release.wait())
    source.start(); assert ready.wait(.2); source.stop(); worker=source._thread
    release.set(); worker.join(.2)
    assert rs.started == [] and source.state.value == "stopped"


def test_stop_after_prestart_validation_cleans_late_start_without_stop_race():
    validated, release, started = threading.Event(), threading.Event(), threading.Event()
    class ActivePipeline(Pipeline):
        def __init__(self,rs): super().__init__(rs); self.active=False; self.stop_calls=0
        def start(self,c): self.rs.started.append(c); self.active=True; started.set()
        def stop(self): self.stop_calls += 1; assert self.active; self.active=False
    rs=RS([]); pipeline=ActivePipeline(rs); rs.pipeline=lambda:pipeline
    source=L515GatewaySource(rs, stop_timeout=.01, mapper_factory=object)
    source._after_pipeline_start_validation=lambda: (validated.set(), release.wait())
    source.start(); assert validated.wait(.2)
    source.stop(); assert pipeline.stop_calls == 0
    worker=source._thread; release.set(); worker.join(.2)
    assert started.is_set() and pipeline.stop_calls == 1 and not pipeline.active
    assert source.state.value == "stopped"


def test_concurrent_start_waits_for_in_progress_stop():
    join_entered, release_join=threading.Event(), threading.Event()
    class Worker:
        alive=True
        def is_alive(self): return self.alive
        def join(self,_): join_entered.set(); release_join.wait(); self.alive=False
    source=L515GatewaySource(RS([]),stop_timeout=.2); source._thread=Worker()
    stopping=threading.Thread(target=source.stop); restarting=threading.Thread(target=source.start)
    stopping.start(); assert join_entered.wait(.2); restarting.start()
    time.sleep(.02); assert restarting.is_alive()
    release_join.set(); stopping.join(.2); restarting.join(.2); source.stop()
    assert not stopping.is_alive() and not restarting.is_alive()


def test_stop_during_device_query_prevents_pipeline_creation():
    entered, release = threading.Event(), threading.Event()
    rs=RS([])
    def context():
        def query(): entered.set(); release.wait(); return [SimpleNamespace(get_info=lambda _:EXPECTED_L515_SERIAL)]
        return SimpleNamespace(query_devices=query)
    rs.context=context
    source=L515GatewaySource(rs,stop_timeout=.01,mapper_factory=object)
    source.start(); assert entered.wait(.2); source.stop(); worker=source._thread
    release.set(); worker.join(.2)
    assert rs.pipelines == [] and rs.started == [] and source.state.value == "stopped"


def test_stop_after_generation_check_prevents_frame_commit():
    entered, release = threading.Event(), threading.Event(); rs=RS([Frames("race",1)])
    source=L515GatewaySource(rs,stop_timeout=.01,mapper_factory=object)
    source._before_frame_commit=lambda: (entered.set(),release.wait())
    source.start(); assert entered.wait(.2); source.stop(); worker=source._thread
    release.set(); worker.join(.2)
    assert source.poll_latest().empty and source.state.value == "stopped"


def test_cancelled_native_start_uses_one_worker_stop_attempt():
    entered, release, stop_entered, release_stop = (threading.Event() for _ in range(4))
    class BlockingCleanup(Pipeline):
        def __init__(self,rs): super().__init__(rs); self.active=False; self.calls=0
        def start(self,c): self.rs.started.append(c); self.active=True
        def stop(self):
            assert self.active; self.calls += 1; stop_entered.set(); release_stop.wait()
    rs=RS([]); pipeline=BlockingCleanup(rs); rs.pipeline=lambda:pipeline
    source=L515GatewaySource(rs,stop_timeout=.01,mapper_factory=object)
    source._after_pipeline_start_validation=lambda: (entered.set(),release.wait())
    source.start(); assert entered.wait(.2); source.stop(); assert pipeline.calls == 0
    worker=source._thread; release.set(); worker.join(.2)
    assert stop_entered.wait(.2) and pipeline.calls == 1 and not worker.is_alive()
    assert source.state.value == "stopped"
    release_stop.set()
