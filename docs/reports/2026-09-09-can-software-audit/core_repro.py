"""Host-only failure probes. All bus/reset operations use injected fakes.

Run from the repository root with PYTHONPATH=.:motor_control:ros2/src/powertrain_ros.
Assertions document defects in the audited snapshot, not desired behavior.
"""
import collections,json,struct,threading,time
import can
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.corner_module import CornerModule
from corner_module.config import CornerConfig
from corner_module.null_steer import NullSteer
from corner_module.can_watchdog import CanWatchdog
from chassis.chassis_manager import ChassisManager,build_corners

class Bus:
    def __init__(self,frames=(),fail=False):self.frames=collections.deque(frames);self.sent=[];self.fail=fail
    def send(self,msg,timeout=None):
        self.sent.append(msg)
        if self.fail:raise can.CanOperationError('injected ENOBUFS')
    def recv(self,timeout=0):return self.frames.popleft() if self.frames else None
    def shutdown(self):pass

def report(case,**kw):print(json.dumps({'case':case,**kw}))
# Existing production arm path; all actual transmit attempts fail.
buses={}
def factory(n):
    buses[n]=Bus(fail=True)
    return DriveOdriveCan(n,bus=buses[n])
cm=ChassisManager(build_corners(lambda n:NullSteer(),factory))
cm.connect();armed=cm.arm()
assert armed is True and cm.mode=='ARMED'
assert sum(len(b.sent) for b in buses.values())==24
report('send_errors_swallowed',arm_return=armed,chassis_mode=cm.mode,failed_send_attempts=24)
cm.tick()
assert cm.mode=='ESTOP'
report('arm_next_tick_no_feedback_guard',chassis_mode=cm.mode)
# A live heartbeat does not prove that failed TX commands reached the axis.
buses={}
def factory(n):
    buses[n]=Bus(fail=True)
    return DriveOdriveCan(n,bus=buses[n])
cm=ChassisManager(build_corners(lambda n:NullSteer(),factory))
cm.connect();cm.arm()
for _ in range(5):
    for n,b in buses.items():
        b.frames.extend([
            can.Message(arbitration_id=(n<<5)|1,is_extended_id=False,
                        data=struct.pack('<I',0)+bytes([1,0,0,0])),
            can.Message(arbitration_id=(n<<5)|9,is_extended_id=False,
                        data=struct.pack('<ff',0,0)),
        ])
    cm.tick()
assert cm.mode=='ARMED'
states=[c.drive.health_state()['axis_state'] for c in cm.corners.values()]
assert states==[1]*6
report('failed_tx_with_fresh_idle_feedback',chassis_mode=cm.mode,axis_states=states,
       feedback_ticks=5,all_send_attempts_failed=True)
# Delayed old receive is timestamped at processing time.
old=can.Message(arbitration_id=(11<<5)|9,is_extended_id=False,timestamp=time.time()-10,data=struct.pack('<ff',0,5))
d=DriveOdriveCan(11,bus=Bus([old]),clock=lambda:100.0)
h=d.state()
assert h['last_encoder_age_ms']==0 and h['encoder_stale'] is False
report('old_frame_marked_fresh',injected_frame_age_s=10,reported_encoder_age_ms=h['last_encoder_age_ms'],actual_vel=h['actual_vel'])
# Heartbeat remains fresh while encoder telemetry never arrives.
hb=can.Message(arbitration_id=(11<<5)|1,is_extended_id=False,data=struct.pack('<I',0)+bytes([8,0,0,0]))
d=DriveOdriveCan(11,bus=Bus([hb]),clock=lambda:100.0)
c=CornerModule(NullSteer(),d,CornerConfig(),clock=lambda:100.0)
c.mode='ARMED';c.tick();h=d.health_state()
assert c.mode=='ARMED' and h['stale'] is False and h['encoder_stale'] is True
report('encoder_loss_not_in_drive_tick_gate',corner_mode=c.mode,stale=h['stale'],encoder_stale=h['encoder_stale'])
# Count real driver query emissions in stationary IDLE, with no CAN socket.
buses={}
def factory(n):
    buses[n]=Bus();return DriveOdriveCan(n,bus=buses[n])
cm=ChassisManager(build_corners(lambda n:NullSteer(),factory));cm.connect()
for _ in range(50):cm.tick()
frames=[m for b in buses.values() for m in b.sent]
assert len(frames)==600 and all(m.is_remote_frame for m in frames)
report('idle_poll_50_ticks',total_tx_frames=len(frames),command_ids=dict(collections.Counter(m.arbitration_id&31 for m in frames)),all_rtr=True)
# Independent watchdogs can both decide to reset on the same old observation.
barrier=threading.Barrier(2);events=[];resets=[]
watchdogs=[CanWatchdog(),CanWatchdog()]
for i,w in enumerate(watchdogs):
    w._fails=1;w._last_tx=10
    w._interface_is_up=lambda:True
    w._tx_packets=lambda:10
    w._probe_ok=lambda:False
    w._reopen_probe_socket=lambda:None
    def reset(i=i):barrier.wait(timeout=2);resets.append(i)
    w._reset_interface=reset
threads=[threading.Thread(target=lambda w=w:events.append(w._step())) for w in watchdogs]
for t in threads:t.start()
for t in threads:t.join(3)
assert len(resets)==2 and events==['reset','reset']
report('watchdogs_reset_same_stall',reset_calls=len(resets),events=events)
