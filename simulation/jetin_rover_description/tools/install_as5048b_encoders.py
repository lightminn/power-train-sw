"""Isaac Sim Script Editor: run, then Play. Reads only; no drives changed."""
import sys,builtins
from pathlib import Path
import omni.usd,omni.physx,omni.timeline
stage=omni.usd.get_context().get_stage()
candidates=[]
for layer in stage.GetUsedLayers():
    if layer.realPath:
        folder=Path(layer.realPath).parent
        candidates.extend((folder,folder.parent))
if '__file__' in globals():candidates.insert(0,Path(__file__).resolve().parents[1])
package=next((p for p in candidates if (p/'config/as5048b_encoders.json').is_file()),None)
assert package is not None,'Keep config and tools beside the final JETIN USD'
sys.path.insert(0,str(package/'tools'))
from as5048b_encoders import SuspensionEncoderRig
old=getattr(builtins,'JETIN_AS5048_ENCODERS',None)
if old is not None:
    subscription=old.get('subscription')
    if subscription is not None:subscription.unsubscribe()
    old['timeline_subscription']=None
state={'rig':None,'samples':{},'time_s':0.,'error':None}
def timeline_event(event):
    if event.type==int(omni.timeline.TimelineEventType.STOP):
        state.update(rig=None,samples={},time_s=0.,error=None)
state['timeline_subscription']=omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(timeline_event)
def read_encoders(dt):
    try:
        if state['rig'] is None:state['rig']=SuspensionEncoderRig()
        state['time_s']+=float(dt)
        state['samples']=state['rig'].sample(state['time_s'])
        state['error']=None
    except Exception as error:
        if state['error']!=str(error):print('AS5048B readback:',str(error))
        state['error']=str(error)
state['subscription']=omni.physx.get_physx_interface().subscribe_physics_step_events(read_encoders)
JETIN_AS5048_ENCODERS=state;builtins.JETIN_AS5048_ENCODERS=state
print('AS5048B four-axis readback installed. Press Play; read JETIN_AS5048_ENCODERS["samples"].')
