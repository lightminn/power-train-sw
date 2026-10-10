"""Script Editor: unified Play/Stop, bounded motor controls and capture UI.

Run after opening a final starter/scenario. No command is sent until a control
is used. Sensor frames, masses, collision shapes and limits are not changed.
"""
from pathlib import Path
import sys,json,runpy,builtins,datetime,subprocess,shutil
import omni.ui as ui,omni.usd,omni.timeline,omni.kit.app
from pxr import UsdPhysics
P=Path(__file__).resolve().parents[1] if '__file__' in globals() else next(p for l in omni.usd.get_context().get_stage().GetUsedLayers() if l.realPath for p in [Path(l.realPath).parent,Path(l.realPath).parent.parent] if (p/'config/manufacturer_models.json').is_file())
sys.path.insert(0,str(P/'tools'))
old=getattr(builtins,'JETIN_RUNTIME_UI',None)
if old:
    if old['recording']:
        previous=old['recording'];previous['pipe'].stdin.close();previous['pipe'].wait(timeout=30)
        previous['ann'].detach([previous['product']]);previous['product'].destroy()
        (previous['folder']/'frame_times.json').write_text(json.dumps(previous['times']),encoding='utf8')
    if old['sensors']:old['sensors'].close()
    old['subscription']=None;old['window'].destroy()
state=dict(window=None,subscription=None,sensors=None,recording=None,status='Ready',last_record_time=-1.,frames=0,control_models={})
models=runpy.run_path(str(P/'tools/install_manufacturer_models.py'))['JETIN_MANUFACTURER_MODELS']
timeline=omni.timeline.get_timeline_interface()
cfg=json.loads((P/'config/manufacturer_models.json').read_text(encoding='utf8'))
def play():timeline.play()
def stop():
    models['wheel_speed_command_m_s']=None
    if state['recording']:record()
    if state['sensors']:state['sensors'].close()
    timeline.stop();state['sensors']=None;state['status']='Stopped'
def move_joint(name,m):
    p=omni.usd.get_context().get_stage().GetPrimAtPath('/JETIN/Joints/'+name);drive=UsdPhysics.DriveAPI.Get(p,'angular');value=float(m.as_float)
    lo=p.GetAttribute('physics:lowerLimit').Get();hi=p.GetAttribute('physics:upperLimit').Get()
    if lo is not None:value=max(lo,value)
    if hi is not None:value=min(hi,value)
    drive.GetTargetPositionAttr().Set(value)
def sensors():
    if state['sensors'] is None:
        from virtual_sensors import VirtualSensorRig
        state['sensors']=VirtualSensorRig()
    return state['sensors']
def snapshot():
    if not timeline.is_playing():state['status']='Press Play before capture';return
    try:
        folder=P/'results'/('snapshot_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));sensors().save(folder)
        motion=getattr(builtins,'JETIN_MOTION_FEEDBACK',{}).get('samples',{});enc=getattr(builtins,'JETIN_AS5048_ENCODERS',{}).get('samples',{})
        (folder/'feedback.json').write_text(json.dumps(dict(motion=motion,encoders=enc),indent=2),encoding='utf8');state['status']='Captured: '+str(folder)
    except Exception as e:state['status']='Capture: '+str(e)
def record():
    if state['recording']:
        r=state['recording'];r['pipe'].stdin.close();r['pipe'].wait(timeout=30);r['ann'].detach([r['product']]);r['product'].destroy()
        (r['folder']/'frame_times.json').write_text(json.dumps(r['times']),encoding='utf8');state['recording']=None;state['status']='Recorded '+str(state['frames'])+' actual frames';return
    if not timeline.is_playing():state['status']='Press Play before recording';return
    if not shutil.which('ffmpeg'):state['status']='ffmpeg is needed for MP4; Snapshot remains available';return
    import omni.replicator.core as rep
    folder=P/'results'/('record_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));folder.mkdir(parents=True,exist_ok=True)
    stage=omni.usd.get_context().get_stage();path=next((p for p in ['/Task/Wide','/Task/Side','/Environment/Overview'] if stage.GetPrimAtPath(p)),None)
    product=rep.create.render_product(path,(960,540));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach([product])
    pipe=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','960x540','-r','20','-i','-','-an','-c:v','libx264','-threads','1','-pix_fmt','yuv420p',str(folder/'overview.mp4')],stdin=subprocess.PIPE)
    state.update(recording=dict(folder=folder,pipe=pipe,ann=ann,product=product,times=[]),last_record_time=-1.,frames=0,status='Recording at up to 20 samples per simulation second')
window=ui.Window('JETIN Rover controls',width=470,height=600);state['window']=window
with window.frame:
    with ui.VStack(spacing=8):
        ui.Label('Open a starter or scene, then Play. Angles are degrees.',height=26)
        with ui.HStack(height=28):ui.Button('Play',clicked_fn=play);ui.Button('Stop',clicked_fn=stop);ui.Button('Snapshot',clicked_fn=snapshot);ui.Button('Record / finish',clicked_fn=record)
        with ui.HStack(height=25):
            ui.Label('Wheel speed (m/s)',width=170);slider=ui.FloatSlider(min=-.3,max=.3);slider.model.add_value_changed_fn(lambda m:models.update(wheel_speed_command_m_s=float(m.as_float)))
            state['control_models']['wheel_speed']=slider.model
        for n in [*cfg['confirmed_servo_joints'],*['fl_steer_joint','fr_steer_joint','rl_steer_joint','rr_steer_joint']]:
            if n=='pinion_B_rotation':continue
            p=omni.usd.get_context().get_stage().GetPrimAtPath('/JETIN/Joints/'+n)
            if not p:continue
            lo=p.GetAttribute('physics:lowerLimit').Get();hi=p.GetAttribute('physics:upperLimit').Get();d=UsdPhysics.DriveAPI.Get(p,'angular')
            with ui.HStack(height=25):
                ui.Label(n,width=230);slider=ui.FloatSlider(min=-180 if lo is None else lo,max=180 if hi is None else hi);slider.model.set_value(float(d.GetTargetPositionAttr().Get() or 0.));slider.model.add_value_changed_fn(lambda m,name=n:move_joint(name,m));state['control_models'][n]=slider.model
        status=ui.Label('Ready',word_wrap=True,height=80)
        ui.Label('Passive rocker/bogie/connector joints are read only.\nCapture files are saved in results/.',word_wrap=True)
def update(_):
    state['status']=models['error'] or state['status'];status.text=state['status']
    r=state['recording']
    if r and timeline.is_playing():
        import numpy as np
        t=float(timeline.get_current_time())
        if t-state['last_record_time']>=.05-1e-6:
            a=np.asarray(r['ann'].get_data())
            if a.shape[:2]==(540,960):r['pipe'].stdin.write(a[:,:,:3].copy().tobytes());r['times'].append(t);state['last_record_time']=t;state['frames']+=1
state['subscription']=omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(update)
JETIN_RUNTIME_UI=state;builtins.JETIN_RUNTIME_UI=state
print('JETIN unified controls ready; no automatic motion command.')
