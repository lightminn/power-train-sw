"""Actual driver feedback-only test. No arm/state/speed/close() writes."""
import collections,datetime,gzip,hashlib,json,pathlib,threading,time
import can
from chassis.runtime_lock import RealCanSession
from corner_module.drive_odrive_can import DriveOdriveCan, CanFeedbackScheduler
import corner_module.drive_odrive_can as module
out=pathlib.Path('/evidence/mks-reliability/final-queries'); out.mkdir(exist_ok=True); done=threading.Event(); capture_errors=[]
result={'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),'driver_sha256':hashlib.sha256(pathlib.Path(module.__file__).read_bytes()).hexdigest(),'scheduler':'shared logical cycle', 'duration_requested_s':600,'phases':[],'errors':[]}
phase=['baseline']; counts=collections.defaultdict(collections.Counter); last_hb={}; maxgap=collections.Counter(); tx=collections.Counter(); last_state={}; rx_positions={}; stales=collections.Counter()
def counters():
 return {k:int(pathlib.Path('/sys/class/net/can0/statistics/'+k).read_text()) for k in ('rx_packets','tx_packets','rx_errors','tx_errors','rx_dropped','tx_dropped')}
with RealCanSession(owner='mks_six_axis_feedback_only'):
 observer=can.Bus(interface='socketcan',channel='can0')
 buses={n:can.Bus(interface='socketcan',channel='can0',can_filters=[{'can_id':n<<5,'can_mask':0x7e0,'extended':False}]) for n in range(11,17)}
 scheduler=CanFeedbackScheduler()
 drivers={n:DriveOdriveCan(n,bus=b,feedback_scheduler=scheduler) for n,b in buses.items()}
 def receive():
  try:
   with gzip.open(out/'six-axis-queries-frames.jsonl.gz','wt',compresslevel=1) as f:
    while not done.is_set():
     m=observer.recv(.02)
     if m is None:continue
     now=time.monotonic();p=phase[0];f.write(json.dumps({'t':now,'timestamp':m.timestamp,'phase':p,'id':m.arbitration_id,'ext':m.is_extended_id,'rtr':m.is_remote_frame,'rx':m.is_rx,'error':m.is_error_frame,'dlc':m.dlc,'data':bytes(m.data).hex()})+'\n')
     if m.is_error_frame: capture_errors.append('CAN error frame')
     if not m.is_rx:
      tx[f'{m.arbitration_id>>5}:{m.arbitration_id&31}:{m.is_remote_frame}']+=1
      if m.is_extended_id or not m.is_remote_frame or m.arbitration_id>>5 not in drivers or m.arbitration_id&31 not in (9,20):capture_errors.append('unexpected TX frame')
     if not m.is_rx or m.is_remote_frame or m.is_error_frame:continue
     key=None
     if not m.is_extended_id and m.arbitration_id&31==1 and m.arbitration_id>>5 in drivers and len(m.data)>=5:
      n=m.arbitration_id>>5;key=str(n);last_state[n]={'state':m.data[4],'error':int.from_bytes(m.data[:4],'little')}
     elif m.is_extended_id and m.arbitration_id>>8==41 and m.arbitration_id&255 in (1,2,3,4):key='ak'+str(m.arbitration_id&255)
     if key:
      if key in last_hb:maxgap[key]=max(maxgap[key],now-last_hb[key])
      last_hb[key]=now;counts[p][key]+=1
  except Exception as e:capture_errors.append(repr(e))
 thread=threading.Thread(target=receive);thread.start();result['counters_before']=counters()
 try:
  for name,duration in [('baseline',6),('queries',600),('after',6)]:
   phase[0]=name;start=time.monotonic();next_tick=start;next_report=start+30
   while time.monotonic()-start<duration:
    now=time.monotonic()
    if capture_errors:raise RuntimeError(capture_errors)
    if now-start>1:
     missing=[k for k in [str(n) for n in drivers]+['ak1','ak2','ak3','ak4'] if now-last_hb.get(k,0)>.4]
     if missing:raise RuntimeError('heartbeat missing '+repr(missing))
     if any(s['state']!=1 or s['error']!=0 for s in last_state.values()):raise RuntimeError('non-idle or error '+repr(last_state))
    if name=='queries':
     scheduler.begin_cycle()
     for n,d in drivers.items():
      d.poll_feedback();state=d.state()
      if now-start>1 and state['encoder_stale']:stales[str(n)]+=1
    if now>=next_report:
     print(json.dumps({'phase':name,'elapsed_s':round(now-start,1),'heartbeats':dict(counts[name]),'stale_samples':dict(stales),'max_gap_ms':{k:round(v*1000,2) for k,v in maxgap.items()}}),flush=True);next_report+=30
    next_tick+=.02
    time.sleep(max(0,next_tick-time.monotonic()))
    if time.monotonic()-next_tick>.02:next_tick=time.monotonic()
   result['phases'].append({'name':name,'duration_s':time.monotonic()-start,'counts':dict(counts[name])})
 except Exception as e:result['errors'].append(repr(e))
 finally:
  result['counters_after']=counters();result['last_state']=last_state;result['max_heartbeat_gap_ms']={k:v*1000 for k,v in maxgap.items()};result['stale_samples']=dict(stales);result['tx_commands']=dict(tx)
  done.set();thread.join(3)
  for b in buses.values():b.shutdown()
  observer.shutdown()
 result['finished_at']=datetime.datetime.now(datetime.timezone.utc).isoformat();(out/'six-axis-queries-result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
 if result['errors'] or stales:raise SystemExit(1)
