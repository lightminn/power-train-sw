"""Independent closure checks for the three previously reproduced P1 findings."""
import importlib.util
import json
from pathlib import Path
import struct

repo=Path('/home/light/ZETIN/robotics/power-train-sw')
spec=importlib.util.spec_from_file_location('fixture',repo/'motor_control/chassis/tests/test_can_auto_recovery.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
Rig=module.Rig

def nonzero_since(rig,baseline):
    return {str(n):[struct.unpack('<ff',m.data)[0] for m in bus.sent[baseline[n]:]
                   if m.arbitration_id & 31 == 13 and struct.unpack('<ff',m.data)[0] != 0]
            for n,bus in rig.buses.items()}

for new_packet_immediately in [False,True]:
    rig=Rig();rig.buses[13].recover();rig.cm.tick();rig.wait_rearmed()
    baseline={n:len(b.sent) for n,b in rig.buses.items()}
    rig.cm.corners['front_left'].steer.stale_flag=True
    if not new_packet_immediately:rig.step(received_s=rig.now)
    rig.step()
    emitted=nonzero_since(rig,baseline)
    assert all(not values for values in emitted.values()), emitted
    assert rig.cm.mode=='ESTOP' and rig.cm.can_recovery_state()['count']==0
    print(json.dumps({'case':'P1-1','new_packet_immediately':new_packet_immediately,
                      'mode':rig.cm.mode,'nonzero_sends':emitted,'result':'PASS'}))

rig=Rig()
for n in [13,14,15,16]:rig.buses[n].online=False
rig.step(.21)
baseline={n:len(b.sent) for n,b in rig.buses.items()}
for n in [13,14]:rig.buses[n].recover()
for n in [15,16]:rig.buses[n].recover(error=0,flags=0,generation=0)
for _ in range(28):rig.step(.19)
assert all(bus.state==1 and bus.velocity==0 for bus in rig.buses.values())
assert rig.cm.can_recovery_state()['state']=='CANCELLED' and rig.cm.can_recovery_state()['count']==0
assert all(not v for v in nonzero_since(rig,baseline).values())
print(json.dumps({'case':'P1-2','board13_14':'eligible','board15_16':'reboot_error0',
                  'state':rig.cm.can_recovery_state(),'result':'PASS'}))

# Preserve the old reproduction file. Update only metadata plumbing in a new
# copy so it exercises the final server epoch and unchanged lifecycle ordering.
old=Path('/tmp/mks-can-auto-resume-20260910/host-reconnect-probe.py').read_text()
old=old.replace("old = 'e628ef42-8120-4d81-9555-d7e7244b5dc0'", "old = 'server-1'")
old=old.replace("new = 'f628ef42-8120-4d81-9555-d7e7244b5dc0'", "new = 'server-2'")
old=old.replace("encode_frame(sample,session_id=session_id,", "encode_frame(sample,session_id='e628ef42-8120-4d81-9555-d7e7244b5dc0',")
old=old.replace('gateway.begin_connection()', 'gateway.begin_connection(connection_session_id=old)')
old=old.replace('node = SimpleNamespace(_authority=au, cm=rig.cm,',
                'node = SimpleNamespace(_authority=au, cm=rig.cm, _gateway_connection_session_id=old,')
old=old.replace("'input_fresh':output.input_fresh}","'input_fresh':output.input_fresh,'connection_session_id':output.drive.connection_session_id}")
old=old.replace('source_received_s=output.drive.source_received_s or 0),{})',
                'source_received_s=output.drive.source_received_s or 0,connection_session_id=output.drive.connection_session_id),{})')
old += "\nassert rig.cm.mode == 'IDLE'\nassert rig.cm.can_recovery_state()['state'] == 'CANCELLED'\nassert rig.cm.can_recovery_state()['count'] == 0\nassert all(bus.velocity == 0 for bus in rig.buses.values())\nprint(json.dumps({'case':'P1-3','same_client_session_id':True,'result':'PASS'}))\n"
new=Path('/tmp/mks-can-auto-resume-20260910/host-reconnect-final-probe.py');new.write_text(old)
exec(compile(old,str(new),'exec'),{})
