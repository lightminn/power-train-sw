"""Authenticated manual Ackermann acceptance on isolated installed ROS/vcan77."""
import json,pathlib,socket,time,uuid,sys,os,math
from powertrain_runtime.client import SessionClient
from motor_control.laptop.remote_operation_client import ClientInput,encode_frame
base=pathlib.Path(os.environ.get('LOOP_EVIDENCE_DIR','/tmp/can-remediation-20260909/loop'));base.mkdir(parents=True,exist_ok=True)
token=base/'loop.token';token.write_text('isolated-loop-test-token');token.chmod(0o600)
config=dict(robot_id='isolated-loop-test',hosts=[os.environ.get('LOOP_HOST','jetson-orin.local')],session_port=29002,token_file=str(token))
c=SessionClient(config);raw=None;state={};last_packet=None;packets=[];seq=0;sid=str(uuid.uuid4());sample=ClientInput();last_heartbeat=0.
negative='--negative-neutral' in sys.argv
negative_fake='--negative-fake' in sys.argv
if negative:sample=ClientInput(deadman=True,right_trigger=.4)
udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind(('0.0.0.0',15005));udp.setblocking(False)
def pump(predicate,timeout=6,label='condition'):
 global state,seq,last_packet,last_heartbeat
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  now=time.monotonic()
  if raw is not None:
   raw.sendall(encode_frame(sample,session_id=sid,sequence=seq,client_monotonic_ns=time.monotonic_ns()));seq+=1
   try:
    reply=raw.recv(8192)
    if not reply:raise RuntimeError('input channel ended')
   except BlockingIOError:pass
  if now-last_heartbeat>.12:
   state=c.heartbeat();last_heartbeat=now
  while True:
   try:data,_=udp.recvfrom(8192)
   except BlockingIOError:break
   last_packet=json.loads(data);packets.append(last_packet)
  if predicate():return
  time.sleep(.025)
 raise AssertionError(label+' timed out: '+json.dumps({k:v for k,v in state.items() if k not in ('ticket','lease_id','boot_id')},ensure_ascii=False))
def start():
 c.request('start_begin')
 pump(lambda:False,timeout=1.6,label='hold')

def assert_case(raw_state, *, label, steering_sign, drive_sign):
 """Judge received CAN commands, using independently extracted CAD centers.

 Front/rear feedback must keep its physical angle when speed is reversed.
 The intersection check uses measured raw CAN angles and the reviewed URDF
 snapshot, never solve_steering() or its curvature-bound implementation.
 """
 assert set(raw_state['drive'])=={'11','12','13','14','15','16'},(label,raw_state)
 assert set(raw_state['steer'])=={'1','2','3','4'},(label,raw_state)
 for n in ('11','13','15'):
  value=raw_state['drive'][n]
  assert value==0. if drive_sign==0 else value*drive_sign<0,(label,raw_state)
 for n in ('12','14','16'):
  value=raw_state['drive'][n]
  assert value==0. if drive_sign==0 else value*drive_sign>0,(label,raw_state)
 for n in ('1','2'):assert raw_state['steer'][n]*steering_sign<-1,(label,raw_state)
 for n in ('3','4'):assert raw_state['steer'][n]*steering_sign>1,(label,raw_state)
 # All AK axes invert raw-CW into wheel-frame positive-left. CAD geometry
 # source: ../2026-09-09-ackermann-urdf-evidence.json (URDF SHA29581f39...).
 coordinates={'1':(.437747327500397,.2725),'2':(.437747327500397,-.2725),
              '3':(-.437747327500397,.2125),'4':(-.437747327500397,-.2125)}
 fixed_axle_x=-.0603355674998952
 intersections=[]
 for n,(x,y) in coordinates.items():
  angle=math.radians(-raw_state['steer'][n])
  intersections.append(y+(x-fixed_axle_x)/math.tan(angle))
 assert max(intersections)-min(intersections)<.002,(label,'inconsistent ICR',intersections)
 assert all(radius*steering_sign>0 for radius in intersections),(label,intersections)
 return intersections

try:
 until=time.monotonic()+25
 while True:
  try:state=c.connect();break
  except OSError:
   if time.monotonic()>until:raise
   time.sleep(.2)
 pump(lambda:(state.get('ops_state') or {}).get('chassis_mode')=='IDLE',label='fresh emulated hardware chassis IDLE')
 if not negative_fake:
  pump(lambda:(state.get('ops_state') or {}).get('wheels_stopped') is True,timeout=8,label='fresh hardware stop proof')
 try:
  raw=c.open_channel('input');raw.setblocking(False)
 except OSError:
  if not negative_fake:raise
 if negative_fake:
  assert raw is None, 'fake hardware proof admitted an input channel'
  assert (state.get('ops_state') or {}).get('wheels_stopped') is False
 if negative or negative_fake:
  settle=time.monotonic()+2
  pump(lambda:time.monotonic()>=settle,timeout=3,label='negative fixture settle')
  assert state.get('start_blocker') is not None, 'unsafe fixture became start-ready'
  c.request('start_begin')
  held=time.monotonic()+1.6
  pump(lambda:time.monotonic()>=held,timeout=2.2,label='negative held intent')
  response=c.request('start')
  if response.get('status')!='REJECTED':
   pump(lambda:state.get('action')=='start' and state.get('status')=='REJECTED',timeout=6,label='unsafe start rejected')
  assert (state.get('ops_state') or {}).get('chassis_mode')=='IDLE'
  assert all(all(abs(w.get('drive_turns_per_s',0.))<.001 for w in packet.get('wheel_statuses',[])) for packet in packets)
  label='fake hardware stop proof' if negative_fake else 'non-neutral pad'
  (base/'negative-result.json').write_text(json.dumps({'result':'PASS','rejected':label,'packets':len(packets)},indent=2))
  print('NEGATIVE PASS: '+label+' cannot arm despite held start',flush=True)
  raise SystemExit(0)
 pump(lambda:state.get('start_blocker','waiting') is None,timeout=4 if negative else 8,label='neutral gateway ready')
 reply=c.request('start')
 assert reply.get('status')=='REJECTED','unheld Start accepted'
 c.request('start_begin')
 hold_until=time.monotonic()+1.6
 pump(lambda:time.monotonic()>=hold_until,timeout=2.2,label='held intent')
 reply=c.request('start')
 pump(lambda:state.get('action')=='start' and state.get('status') in ('SUCCEEDED','REJECTED','OUTCOME_UNKNOWN'),timeout=8,label='manual arm result')
 assert state['status']=='SUCCEEDED',state.get('detail')
 assert state['ops_state']['chassis_mode']=='ARMED'
 print('HELD START PASS: installed ops -> manual authority -> arm',flush=True)
 sample=ClientInput(deadman=True,right_trigger=.4)
 pump(lambda:last_packet is not None and any(abs(w.get('drive_turns_per_s',0.))>.1 for w in last_packet.get('wheel_statuses',[])),timeout=6,label='nonzero wheel feedback over real UDP')
 print('MOTION LOOP PASS: encoded input -> Jetson ROS -> virtual CAN wheel feedback -> laptop UDP',flush=True)
 # RT/LT/zero change speed only; the same stick keeps the same wheel angles.
 cases=[]
 for label,lx,rt,lt,front_sign,rear_sign,drive_sign in [
  ('RT-left',-.25,.5,0,1,-1,1),('RT-right',.25,.5,0,-1,1,1),
  ('LT-left',-.25,0,.5,1,-1,-1),('LT-right',.25,0,.5,-1,1,-1),
  ('STOP-left',-.25,0,0,1,-1,0),('STOP-right',.25,0,0,-1,1,0)]:
  sample=ClientInput(deadman=True,left_x=lx,right_trigger=rt,left_trigger=lt)
  def correct():
   if not last_packet:return False
   rows=last_packet.get('wheel_statuses',[])
   if len(rows)!=6:return False
   for row in rows:
    name=row['name']
    drive=row.get('drive_turns_per_s',0)
    if drive_sign==0:
     if drive!=0.:return False
    elif drive*drive_sign<=.1:return False
    sign=front_sign if name.startswith('front') else rear_sign if name.startswith('rear') else 0
    if sign and row.get('steer_deg',0)*sign<=1:return False
   return True
  pump(correct,timeout=6,label=label+' physical-frame feedback')
  settle=time.monotonic()+.4
  pump(lambda:time.monotonic()>=settle,timeout=2,label=label+' settle')
  assert (state.get('ops_state') or {}).get('chassis_mode')=='ARMED',(label,state.get('ops_state'))
  raw_state=json.loads((base/'raw-observation.json').read_text())
  intersections=assert_case(raw_state,label=label,steering_sign=front_sign,drive_sign=drive_sign)
  cases.append({'case':label,'raw':raw_state,'wheels':last_packet['wheel_statuses'],
                'icr_y_from_raw_steering_m':intersections})
  print(label+' MANUAL ACKERMANN PASS',flush=True)
 for side in ('left','right'):
  by_label={case['case']:case for case in cases}
  reference=by_label['RT-'+side]['raw']['steer']
  for prefix in ('LT','STOP'):
   actual=by_label[prefix+'-'+side]['raw']['steer']
   assert all(abs(actual[n]-reference[n])<=1e-3 for n in ('1','2','3','4')),(side,prefix,reference,actual)
 (base/'manual-ackermann-cases.json').write_text(json.dumps(cases,indent=2))
 print('ANGLE INDEPENDENCE PASS: same stick at RT/LT/zero has identical CAN angles within .001 degree',flush=True)
 # Disconnect while demanding motion. Session must stop and never replay.
 sample=ClientInput(deadman=True,left_x=-.25,right_trigger=.5)
 pump(lambda:last_packet is not None and len(last_packet.get('wheel_statuses',[]))==6
      and all(w.get('drive_turns_per_s',0.)>.1 for w in last_packet['wheel_statuses']),
      timeout=6,label='restore positive motion before disconnect')
 raw.close();raw=None
 try:c.heartbeat()
 except OSError:pass
 c.close();c=SessionClient(config);last_heartbeat=0.
 until=time.monotonic()+8
 while True:
  try:state=c.connect();break
  except OSError:
   if time.monotonic()>until:raise
   time.sleep(.2)
 pump(lambda:(state.get('ops_state') or {}).get('chassis_mode')=='IDLE' and state['ops_state'].get('wheels_stopped') is True,timeout=6,label='input-loss stop')
 generation_end=time.monotonic()+1
 pump(lambda:time.monotonic()>=generation_end,timeout=2,label='reconnect stability')
 assert state['ops_state']['chassis_mode']=='IDLE'
 assert state['ops_state']['authority_mode']=='IDLE'
 assert state['ops_state']['estop_latched'] is False
 assert last_packet and all(abs(w.get('drive_turns_per_s',0.))<.001 for w in last_packet.get('wheel_statuses',[]))
 print('INPUT LOSS / RECONNECT PASS: fresh IDLE, zero wheel feedback, no re-arm',flush=True)
 (base/'loop-result.json').write_text(json.dumps({'result':'PASS','manual_ackermann_cases':len(cases),
   'stationary_cases':2,'same_angle_tolerance_deg':1e-3,'packets':len(packets),
   'last_telemetry':last_packet,'ops_state':state['ops_state']},indent=2))
finally:
 if raw is not None:raw.close()
 c.close();udp.close()
