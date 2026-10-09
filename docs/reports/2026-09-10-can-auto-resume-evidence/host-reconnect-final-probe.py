"""Actual gateway event drain and chassis callbacks; no ROS import or network."""
import ast
from collections import deque
import importlib.util
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

from chassis.authority import CommandAuthority, MANUAL_SOURCE, TELEOP
from powertrain_ros.remote_input import RemoteInputDecoder
from powertrain_ros.remote_input_gateway import RemoteInputGateway
from motor_control.laptop.remote_operation_client import ClientInput, encode_frame

repo = Path('/home/light/ZETIN/robotics/power-train-sw')
src = repo / 'ros2/src/powertrain_ros/powertrain_ros'
spec = importlib.util.spec_from_file_location('rig', repo / 'motor_control/chassis/tests/test_can_auto_recovery.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
rig = mod.Rig()
rig.buses[13].recover()
rig.cm.tick()
rig.wait_rearmed()

namespace = dict(json=json, math=math, threading=threading, time=time, String=SimpleNamespace,
                 ManualDriveCommand=SimpleNamespace, MAX_EVENTS_PER_TICK=256)
def method(file, cls, name):
    tree = ast.parse((src / file).read_text())
    item = next(x for c in tree.body if isinstance(c, ast.ClassDef) and c.name == cls
                for x in c.body if isinstance(x, ast.FunctionDef) and x.name == name)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[item], type_ignores=[])), str(src / file), 'exec'), namespace)
    return namespace[name]

def frame(session_id, stamp, active=False):
    decoder = RemoteInputDecoder()
    decoder.start_connection()
    sample = ClientInput(deadman=active, right_trigger=.4 if active else 0)
    return decoder.feed(encode_frame(sample,session_id='e628ef42-8120-4d81-9555-d7e7244b5dc0',sequence=0,client_monotonic_ns=1),
                        receive_monotonic_s=stamp)[0].frame

gateway = RemoteInputGateway()
old = 'server-1'
new = 'server-2'
gateway.begin_connection(connection_session_id=old)
gateway.submit(frame(old,rig.now-.03))
gateway.tick(rig.now-.03)
gateway.submit(frame(old,rig.now-.02,True))
assert gateway.tick(rig.now-.02).state == 'DRIVE'

# TCP worker accepts a replacement connection within the 30 Hz gateway period.
rig.now += .02
harness = SimpleNamespace(_events_lock=threading.Lock(), _gateway=gateway,
    _lifecycle_events=deque([('disconnect',old),('connect',new)]),
    _motion_frame=frame(new,rig.now), _violation_events=deque())
count = method('teleop_command_node.py','TeleopCommandNode','_drain_events')(harness,now_s=rig.now)
output = gateway.tick(rig.now)

au = CommandAuthority()
au.set_mode(TELEOP)
au.submit(MANUAL_SOURCE,0,0,rig.now-.1,source_received_s=rig.now-.1)
au.select(rig.now-.1)
node = SimpleNamespace(_authority=au, cm=rig.cm, _gateway_connection_session_id=old, _command_received_s=lambda info:rig.now,
    pub_authority_state=SimpleNamespace(publish=lambda msg:None))
state_callback=method('chassis_node.py','ChassisNode','_on_gateway_state')
command_callback=method('chassis_node.py','ChassisNode','_on_manual_drive_command')
tick_authority=method('chassis_node.py','ChassisNode','_tick_authority')
def deliver(output):
    state_callback(node,SimpleNamespace(data=json.dumps({'state':output.state,'input_fresh':output.input_fresh,'connection_session_id':output.drive.connection_session_id})),{})
    command_callback(node,SimpleNamespace(speed_mps=output.drive.linear,steering=output.drive.steering,
        source_received_s=output.drive.source_received_s or 0,connection_session_id=output.drive.connection_session_id),{})
    tick_authority(node,rig.now)
    for bus in rig.buses.values(): bus.emit()
    rig.cm.tick()

deliver(output)
first=rig.cm.can_recovery_state().copy()
rig.now+=.02
gateway.submit(frame(new,rig.now,True))
deliver(gateway.tick(rig.now))
print(json.dumps({'drained_events':count,'published_gateway_state':output.state,
    'after_reconnect_neutral':first,'after_next_drive':rig.cm.can_recovery_state(),
    'mode':rig.cm.mode,'raw_drive_velocities':{n:b.velocity for n,b in rig.buses.items()}}))

assert rig.cm.mode == 'IDLE'
assert rig.cm.can_recovery_state()['state'] == 'CANCELLED'
assert rig.cm.can_recovery_state()['count'] == 0
assert all(bus.velocity == 0 for bus in rig.buses.values())
print(json.dumps({'case':'P1-3','same_client_session_id':True,'result':'PASS'}))
