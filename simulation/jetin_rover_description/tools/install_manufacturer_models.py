"""Isaac Sim Script Editor: run once, then Play. No automatic travel command.

Run install_virtual_sensors.py separately when rendered RGB/depth is wanted.
Runtime settings can be edited in config/manufacturer_models.json.
"""
import sys,builtins,runpy,math
from pathlib import Path
import omni.usd,omni.physx,omni.timeline
from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
from isaacsim.core.simulation_manager import SimulationManager
stage=omni.usd.get_context().get_stage()
candidates=[Path(l.realPath).parent for l in stage.GetUsedLayers() if l.realPath]
if '__file__' in globals():candidates.insert(0,Path(__file__).resolve().parents[1])
package=next((p for x in candidates for p in [x,x.parent] if (p/'config/manufacturer_models.json').is_file()),None)
assert package,'Keep the final USD next to its config/ and tools/ folders'
if str(package/'tools') not in sys.path:sys.path.insert(0,str(package/'tools'))
from manufacturer_models import IsaacManufacturerActuators
from bl70200_controller import IsaacWheelController
old=getattr(builtins,'JETIN_MANUFACTURER_MODELS',None)
if old:
 if old.get('subscription'):old['subscription'].unsubscribe()
 old['timeline_subscription']=None
state=dict(rig=None,wheels=None,wheel_speed_command_m_s=None,actuator_samples={},error=None,time_s=0.)
def on_timeline(e):
 if e.type==int(omni.timeline.TimelineEventType.STOP):state.update(rig=None,wheels=None,actuator_samples={},error=None,time_s=0.)
def on_step(dt):
 try:
  if state['rig'] is None:
   if SimulationManager.get_physics_sim_view() is None:return
   state['rig']=IsaacManufacturerActuators(stage)
  state['actuator_samples']=state['rig'].step(float(dt));state['servo_samples']=state['rig'].servo_last;state['time_s']+=float(dt)
  if state['wheel_speed_command_m_s'] is not None:
   if state['wheels'] is None:state['wheels']=IsaacWheelController(stage)
   state['wheel_samples']=state['wheels'].step(float(state['wheel_speed_command_m_s']),float(dt))
  state['error']=None
 except Exception as e:
  if state['error']!=str(e):print('JETIN public actuator model:',str(e))
  state['error']=str(e)
state['timeline_subscription']=omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(on_timeline)
state['subscription']=omni.physx.get_physx_interface().subscribe_physics_step_events(on_step)
JETIN_MANUFACTURER_MODELS=state;builtins.JETIN_MANUFACTURER_MODELS=state
runpy.run_path(str(package/'tools/install_motion_feedback.py'))
runpy.run_path(str(package/'tools/install_as5048b_encoders.py'))
print('JETIN public actuator/sensor models installed. Press Play. Set steering angular targets in degrees; wheel command is optional.')
