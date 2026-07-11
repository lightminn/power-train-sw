from types import SimpleNamespace

from l515_dashboard.gateway_source import (
    EXPECTED_L515_SERIAL, L515GatewaySource, GatewayFrames,
)


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
