import json,sys
from pathlib import Path
sys.path.insert(0,'/evidence')
import can
from probe_steering_mount import Probe
from chassis.runtime_lock import RealCanSession
count=Path('/sys/class/net/can0/statistics/tx_packets')
before=int(count.read_text())
with RealCanSession(channel='can0',owner='steering-mount-preflight-readonly'):
 bus=can.interface.Bus(channel='can0',interface='socketcan')
 try:
  p=Probe(bus,1); p.receive(1.5); p.check()
  print(json.dumps({'steer':{n:{'raw_deg':v[1],'fault':v[2]} for n,v in p.steer.items()},'drive':{n:{'error':v[1],'state':v[2]} for n,v in p.drive.items()}},sort_keys=True))
 finally: bus.shutdown()
after=int(count.read_text())
print('CAN tx packets',before,after)
assert before==after, 'unexpected CAN writer during read-only preflight'
