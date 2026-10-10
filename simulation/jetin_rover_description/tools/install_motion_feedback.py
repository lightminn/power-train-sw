"""Script Editor: run before Play. Read-only feedback, no motor commands."""
import sys
import builtins
from pathlib import Path
import omni.usd
import omni.physx
import omni.timeline
from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
from isaacsim.core.simulation_manager import SimulationManager

stage = omni.usd.get_context().get_stage()
candidates = []
for layer in stage.GetUsedLayers():
    if layer.realPath:
        folder = Path(layer.realPath).parent
        candidates.extend((folder, folder.parent))
if '__file__' in globals():
    candidates.insert(0, Path(__file__).resolve().parents[1])
package = next((p for p in candidates if (p/'config/motion_feedback.json').is_file()), None)
assert package is not None, 'Open a scene referencing the updated JETIN package'
if str(package/'tools') not in sys.path:
    sys.path.insert(0, str(package/'tools'))
from motion_feedback import MotionFeedbackRig
old = getattr(builtins, 'JETIN_MOTION_FEEDBACK', None)
if old:
    if old.get('subscription') is not None:
        old['subscription'].unsubscribe()
    old['timeline_subscription'] = None
state = dict(rig=None, samples={}, time_s=0., error=None)
core_nodes = _isaacsim_core_nodes.acquire_interface()

def on_timeline(event):
    if event.type == int(omni.timeline.TimelineEventType.STOP):
        state.update(rig=None, samples={}, time_s=0., error=None)

def on_step(dt):
    try:
        if state['rig'] is None:
            if SimulationManager.get_physics_sim_view() is None:
                return
            state['rig'] = MotionFeedbackRig()
        state['time_s'] = float(core_nodes.get_sim_time())
        state['samples'] = state['rig'].sample(state['time_s'])
        state['error'] = None
    except Exception as error:
        if state['error'] != str(error):
            print('JETIN motion feedback:', str(error))
        state['error'] = str(error)

state['timeline_subscription'] = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(on_timeline)
state['subscription'] = omni.physx.get_physx_interface().subscribe_physics_step_events(on_step)
JETIN_MOTION_FEEDBACK = state
builtins.JETIN_MOTION_FEEDBACK = state
print('L515 IMU + 6 wheel speeds + 4 steering angles installed. Press Play; JETIN_MOTION_FEEDBACK["samples"].')
