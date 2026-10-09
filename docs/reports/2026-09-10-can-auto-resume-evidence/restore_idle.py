"""Restore the user's absent-component mask through ops, never arm."""
import json,time
from pathlib import Path
from laptop.ops_channel_client import OpsChannelClient
client=OpsChannelClient('127.0.0.1',19001,Path('/etc/powertrain/ops_console.token').read_text().strip())
result={'actions':[]}
def state_wait(predicate,seconds=8):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        client.pump(); state=client.latest_ops_state()
        if state and predicate(state): return state
        time.sleep(.03)
    raise RuntimeError('ops state condition timed out')
def action(name,params=None):
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        req=client.submit(name,params=params)
        while time.monotonic()<deadline:
            replies=client.pump()
            reply=next((r for r in replies if r.get('request_id')==req and r.get('status')!='PENDING'),None)
            if reply:
                if reply['status']=='FINAL_SUCCESS':
                    result['actions'].append({'action':name,'status':reply['status'],'detail':reply.get('detail')}); return
                if reply['status']=='FINAL_REJECTED' and 'busy:mutation_inflight' in reply.get('detail',''):
                    time.sleep(.15); break
                raise RuntimeError(str(reply))
            time.sleep(.03)
    raise RuntimeError('ops action timed out: '+name)
try:
    result['before']=state_wait(lambda s:s.get('chassis_mode') in ('IDLE','ESTOP') and s.get('wheels_stopped'))
    for component in ('us100','robot_arm'):
        if client.latest_ops_state()['component_mask'][component]:action(component+'_enable',{'data':False})
    current=state_wait(lambda s: not s['component_mask']['us100'] and not s['component_mask']['robot_arm'])
    if current['estop_latched']:
        assert not current['active_estop_sources'],current
        action('estop_reset')
    result['after']=state_wait(lambda s:s.get('chassis_mode')=='IDLE' and not s['estop_latched'] and s.get('wheels_stopped'))
    assert result['after']['component_mask']=={'drive':True,'steer':True,'us100':False,'robot_arm':False}
    result['result']='PASS'
finally:
    client.close(); print(json.dumps(result,indent=2))
