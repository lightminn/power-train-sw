import collections,datetime,json,pathlib,subprocess,threading,time
import can,odrive
from chassis.usb_session import motor_session,communication_snapshot,read_path
out=pathlib.Path('/evidence'); serial='336A33523235'
result={'method':'same-baud USB CAN peripheral reinit; no board reboot/NVM save/axis command','serial':serial,'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat()}
phase=['before']; counts={p:collections.Counter() for p in ('before','reset','after')};done=threading.Event()
def link():
 root=pathlib.Path('/sys/class/net/can0/statistics')
 return {'source':'sysfs counters', 'stats64':{side:{key:int((root/(side+'_'+key)).read_text()) for key in ('packets','errors','dropped')} for side in ('tx','rx')}}
with motor_session('mks_usb_can_peripheral_diagnosis'):
 board=odrive.find_any(serial_number=serial,timeout=12)
 before=communication_snapshot(board,serial=serial,strict=True)
 assert before['axis0']['node']==13 and before['axis1']['node']==14 and before['baud']==500000
 result['communication_before']=before
 result['uptime_before_ms']=board.system_stats.uptime
 result['link_before']=link()
 bus=can.Bus(interface='socketcan',channel='can0',receive_own_messages=False)
 def receive():
  with (out/'usb-can-peripheral-frames.jsonl').open('w') as f:
   while not done.is_set():
    m=bus.recv(.05)
    if m is None:continue
    p=phase[0]
    f.write(json.dumps({'monotonic_s':time.monotonic(),'phase':p,'timestamp':m.timestamp,'id':m.arbitration_id,'extended':m.is_extended_id,'rtr':m.is_remote_frame,'error':m.is_error_frame,'rx':m.is_rx,'dlc':m.dlc,'data':bytes(m.data).hex()})+'\n')
    if m.is_rx and not m.is_remote_frame and not m.is_error_frame:
     if not m.is_extended_id and m.arbitration_id&31==1 and m.arbitration_id>>5 in range(11,17):counts[p]['ODrive'+str(m.arbitration_id>>5)]+=1
     elif m.is_extended_id and m.arbitration_id>>8==41 and m.arbitration_id&255 in range(1,5):counts[p]['AK'+str(m.arbitration_id&255)]+=1
 thread=threading.Thread(target=receive);thread.start()
 try:
  time.sleep(3)
  phase[0]='reset';result['reset_at']=datetime.datetime.now(datetime.timezone.utc).isoformat()
  board.can.set_baud_rate(500000)
  phase[0]='after';time.sleep(8)
  result['communication_after']=communication_snapshot(board,serial=serial,strict=True)
  result['uptime_after_ms']=board.system_stats.uptime
  result['errors_after']={name:{p:read_path(getattr(board,name),p) for p in ('current_state','error','motor.error','encoder.error','controller.error')} for name in ('axis0','axis1')}
  result['can_error_after']=board.can.error
  result['link_after']=link()
  assert result['communication_before']==result['communication_after']
  assert result['uptime_after_ms']>result['uptime_before_ms']
 finally:
  done.set();thread.join(2);bus.shutdown()
  result['counts']={p:dict(c) for p,c in counts.items()}
  (out/'usb-can-peripheral-reset.json').write_text(json.dumps(result,indent=2)+'\n')
  print(json.dumps(result,indent=2),flush=True)
