"""Single-owner, reconnecting RealSense source for the L515 Gateway."""

from dataclasses import dataclass, field
from enum import Enum
import threading
import time

EXPECTED_L515_SERIAL = "00000000F0271544"


def _canonical_serial(value):
    normalized = str(value).casefold().lstrip("0")
    return normalized or "0"


@dataclass(frozen=True)
class GatewaySourceConfig:
    serial: str = EXPECTED_L515_SERIAL
    color_width: int = 1280
    color_height: int = 720
    depth_width: int = 640
    depth_height: int = 480
    fps: int = 30
    reconnect_interval: float = 2.0

    def __post_init__(self):
        if self.serial != EXPECTED_L515_SERIAL:
            raise ValueError("serial must identify the powertrain L515")


class GatewaySourceState(Enum):
    STOPPED="stopped"; CONNECTING="connecting"; STREAMING="streaming"; DISCONNECTED="disconnected"


@dataclass
class GatewayFrames:
    raw_color: object = None
    raw_depth: object = None
    aligned_depth: object = None
    accel: object = None
    gyro: object = None
    mapper: object = None
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False, compare=False)

    @property
    def empty(self):
        with self._lock:
            return all(getattr(self, n) is None for n in ("raw_color","raw_depth","aligned_depth","accel","gyro"))

    def put(self, **payload):
        with self._lock:
            for name, value in payload.items():
                if value is not None:
                    setattr(self, name, value)

    def clear(self):
        with self._lock:
            for name in ("raw_color","raw_depth","aligned_depth","accel","gyro","mapper"):
                setattr(self, name, None)

    def drain(self):
        with self._lock:
            result = GatewayFrames(**{n:getattr(self,n) for n in ("raw_color","raw_depth","aligned_depth","accel","gyro","mapper")})
            for name in ("raw_color","raw_depth","aligned_depth","accel","gyro","mapper"):
                setattr(self, name, None)
            return result


class L515GatewaySource:
    def __init__(self, rs_module, config=None, *, clock=time.monotonic, wait_fn=None,
                 mapper_factory=object, stop_timeout=1.0):
        self._rs=rs_module; self.config=config or GatewaySourceConfig(); self._clock=clock
        self._stop_event=threading.Event(); self._wait_fn=wait_fn or self._interruptible_wait
        self._mapper_factory=mapper_factory; self._stop_timeout=float(stop_timeout)
        self._latest=GatewayFrames(); self._thread=None; self._pipeline=None; self._starting=None
        self._public_lock=threading.Lock(); self._lifecycle_lock=threading.Lock(); self._generation=0
        self.state=GatewaySourceState.STOPPED; self.state_changed_at=clock()

    def _set_state(self,state): self.state=state; self.state_changed_at=self._clock()
    def _interruptible_wait(self,seconds): return not self._stop_event.wait(seconds)
    def _is_current(self,g): return not self._stop_event.is_set() and g == self._generation

    def start(self):
        with self._public_lock:
            with self._lifecycle_lock:
                if self._thread is not None and self._thread.is_alive(): return
                self._generation += 1; generation=self._generation; self._stop_event.clear()
                self._thread=threading.Thread(target=self._run,args=(generation,),name="l515-gateway-source",daemon=True)
                thread=self._thread
            thread.start()

    @staticmethod
    def _stop_pipeline(pipeline):
        try: pipeline.stop()
        except Exception: pass

    def _stop_pipeline_bounded(self,pipeline):
        thread=threading.Thread(target=self._stop_pipeline,args=(pipeline,),daemon=True)
        thread.start(); thread.join(self._stop_timeout)

    def stop(self):
        with self._public_lock:
            with self._lifecycle_lock:
                self._stop_event.set(); self._generation += 1
                if self._starting is not None: self._starting["cancel_requested"]=True
                pipeline=self._pipeline; thread=self._thread
            if pipeline is not None: self._stop_pipeline_bounded(pipeline)
            if thread is not None and thread.is_alive(): thread.join(self._stop_timeout)
            if thread is None or not thread.is_alive():
                with self._lifecycle_lock:
                    if self._thread is thread: self._thread=None; self._pipeline=None; self._starting=None
                    self._latest.clear(); self._set_state(GatewaySourceState.STOPPED)

    def poll_latest(self): return self._latest.drain()

    def _matching_serial(self):
        expected=_canonical_serial(self.config.serial)
        matches=[d.get_info(self._rs.camera_info.serial_number) for d in self._rs.context().query_devices()
                 if _canonical_serial(d.get_info(self._rs.camera_info.serial_number)) == expected]
        return matches[0] if len(matches)==1 else None

    def _sdk_config(self,serial):
        c=self._rs.config(); c.enable_device(serial)
        c.enable_stream(self._rs.stream.color,self.config.color_width,self.config.color_height,self._rs.format.bgr8,self.config.fps)
        c.enable_stream(self._rs.stream.depth,self.config.depth_width,self.config.depth_height,self._rs.format.z16,self.config.fps)
        c.enable_stream(self._rs.stream.accel); c.enable_stream(self._rs.stream.gyro); return c

    @staticmethod
    def _dedup(payload,last):
        for name in ("raw_color","raw_depth","aligned_depth","accel","gyro"):
            sample=payload[name]; getter=getattr(sample,"get_timestamp",None)
            if sample is None or not callable(getter): continue
            stamp=float(getter())
            if last.get(name)==stamp: payload[name]=None
            else: last[name]=stamp

    def _run(self,generation=None):
        generation=self._generation if generation is None else generation
        while self._is_current(generation):
            pipeline=None
            with self._lifecycle_lock:
                if not self._is_current(generation): break
                self._set_state(GatewaySourceState.CONNECTING)
            try:
                serial=self._matching_serial()
                if serial is None: raise RuntimeError("expected L515 serial is not present")
                pipeline=self._rs.pipeline()
                with self._lifecycle_lock:
                    if not self._is_current(generation): break
                    self._pipeline=pipeline; self._starting={"generation":generation,"pipeline":pipeline,"cancel_requested":False}
                pipeline.start(self._sdk_config(serial))
                with self._lifecycle_lock:
                    starting=self._starting
                    cancelled=not self._is_current(generation) or starting["cancel_requested"]
                    if not cancelled: self._starting=None
                if cancelled: break
                align=self._rs.align(self._rs.stream.color); mapper=self._mapper_factory(); last={}
                self._latest.clear()
                with self._lifecycle_lock:
                    if not self._is_current(generation): break
                    self._set_state(GatewaySourceState.STREAMING)
                while self._is_current(generation):
                    frames=pipeline.wait_for_frames(); aligned=align.process(frames)
                    payload={"raw_color":frames.get_color_frame(),"raw_depth":frames.get_depth_frame(),
                             "aligned_depth":aligned.get_depth_frame(),"accel":frames.first_or_default(self._rs.stream.accel),
                             "gyro":frames.first_or_default(self._rs.stream.gyro),"mapper":mapper}
                    self._dedup(payload,last)
                    with self._lifecycle_lock:
                        if not self._is_current(generation): break
                        self._latest.put(**payload)
            except Exception:
                if self._is_current(generation): self._latest.clear(); self._set_state(GatewaySourceState.DISCONNECTED)
            finally:
                if pipeline is not None: self._stop_pipeline_bounded(pipeline)
                with self._lifecycle_lock:
                    if self._pipeline is pipeline: self._pipeline=None
                    if self._starting is not None and self._starting.get("pipeline") is pipeline: self._starting=None
            if not self._is_current(generation) or not self._wait_fn(self.config.reconnect_interval): break

