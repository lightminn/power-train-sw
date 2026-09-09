"""Read-only CAN queries on USB-recoverable nodes 13/14; no axis commands."""
import collections,datetime,json,pathlib,threading,time
import can,odrive
from chassis.usb_session import motor_session,communication_snapshot
from corner_module.drive_odrive_can import DriveOdriveCan
out=pathlib.Path('/evidence'); serial='336A33523235';done=threading.Event();phase=['baseline']
counts={k:collections.Counter() for k in ('baseline','encoder','iq','combined','after')}
last_hb={};tx_commands=collections.Counter();result={'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'serial':serial,'phases':[],'lost_nodes':[]}
with motor_session('mks_can_bounded_query_diagnosis'):
 board=odrive.find_any(serial_number=serial,timeout=12)
 result['configuration_before']=communication_snapshot(board,serial=serial,strict=True)
 result['uptime_before_ms']=board.system_stats.uptime
 observer=can.Bus(interface='socketcan',channel='can0',receive_own_messages=False)
 buses={n:can.Bus(interface='socketcan',channel='can0',can_filters=[{'can_id':n<<5,'can_mask':0x7e0,'extended':False}]) for n in (13,14)}
 drivers={n:DriveOdriveCan(n,bus=bus) for n,bus in buses.items()}
 def receive():
  with (out/'usb-can-query-frames.jsonl').open('w') as f:
   while not done.is_set():
    m=observer.recv(.02)
    if m is None:continue
    p=phase[0]; now=time.monotonic()
    f.write(json.dumps({'monotonic_s':now,'phase':p,'timestamp':m.timestamp,'id':m.arbitration_id,'extended':m.is_extended_id,'rtr':m.is_remote_frame,'error':m.is_error_frame,'rx':m.is_rx,'dlc':m.dlc,'data':bytes(m.data).hex()})+'\n')
    if m.is_remote_frame and not m.is_extended_id:tx_commands[f'{m.arbitration_id>>5}:0x{m.arbitration_id&31:02x}']+=1
    if m.is_rx and not m.is_remote_frame and not m.is_error_frame and not m.is_extended_id and m.arbitration_id&31==1:
     node=m.arbitration_id>>5
     if node in range(11,17):counts[p][str(node)]+=1;last_hb[node]=now
 thread=threading.Thread(target=receive);thread.start()
 try:
  for name,duration in [('baseline',6),('encoder',15),('iq',15),('combined',30),('after',6)]:
   phase[0]=name;start=time.monotonic();deadline=start+duration;next_query=start
   print('PHASE '+name,flush=True)
   while time.monotonic()<deadline:
    now=time.monotonic()
    if now-start>.5:
     missing=[n for n in (13,14) if now-last_hb.get(n,0)>.4]
     if missing:result['lost_nodes']=missing;break
    if name in ('encoder','iq') and now>=next_query:
     command=9 if name=='encoder' else 20
     for n,bus in buses.items():bus.send(can.Message(arbitration_id=(n<<5)|command,is_extended_id=False,is_remote_frame=True))
     next_query=now+(.05 if name=='encoder' else .2)
    elif name=='combined' and now>=next_query:
     for driver in drivers.values():driver.poll_feedback();driver.state()
     next_query=now+.02
    time.sleep(.002)
   result['phases'].append({'phase':name,'elapsed_s':time.monotonic()-start,'lost_nodes':list(result['lost_nodes'])})
   if result['lost_nodes']:break
  result['configuration_after']=communication_snapshot(board,serial=serial,strict=True)
  result['uptime_after_ms']=board.system_stats.uptime
  result['can_error_after']=board.can.error
  result['axis_errors_after']={name:getattr(board,name).error for name in ('axis0','axis1')}
  assert result['configuration_after']==result['configuration_before']
 finally:
  done.set();thread.join(2)
  observer.shutdown()
  for bus in buses.values():bus.shutdown()
  result['heartbeat_counts']={p:dict(c) for p,c in counts.items()};result['queries']=dict(tx_commands)
  result['finished_at']=datetime.datetime.now(datetime.timezone.utc).isoformat()
  (out/'usb-can-query-result.json').write_text(json.dumps(result,indent=2)+'\n')
  print(json.dumps(result,indent=2),flush=True)
