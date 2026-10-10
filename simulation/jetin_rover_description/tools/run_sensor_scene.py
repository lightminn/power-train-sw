"""Flat/obstacle sensor scene with real rendered outputs and optional ROS2."""
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
from pathlib import Path
import argparse,sys,json,runpy,traceback,hashlib,subprocess
import numpy as np
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P/'tools'))
a=argparse.ArgumentParser();a.add_argument('--scenario',choices=['flat','obstacles'],default='flat');a.add_argument('--seconds',type=float,default=10);a.add_argument('--film',action='store_true');a.add_argument('--gui',action='store_true');a.add_argument('--ros2',action='store_true');a.add_argument('--check-controls',action='store_true');args=a.parse_args()
if args.ros2:
    from ros2_bridge import configure_environment
    configure_environment()
OUT=P/'results'/('sensors_'+args.scenario+'_20261010');OUT.mkdir(parents=True,exist_ok=True)
r=dict(passed=False,status='starting',job=int(os.environ.get('SLURM_JOB_ID',0)),scenario=args.scenario,automatic_motion=False,asset_sha256=hashlib.sha256((P/'JETIN_Rover_Final_20261009.usdc').read_bytes()).hexdigest())
def save():(OUT/'report.json').write_text(json.dumps(r,indent=2,default=str),encoding='utf8')
save();pipe=None;bridge=None
from isaacsim import SimulationApp
gpu=int(os.environ.get('SLURM_JOB_GPUS',os.environ.get('CUDA_VISIBLE_DEVICES','0')).split(',')[0])
app=SimulationApp(dict(headless=not args.gui,multi_gpu=False,active_gpu=gpu,limit_cpu_threads=2,renderer='RaytracedLighting',extra_args=['--portable-root',os.environ.get('JETIN_KIT_RUNTIME',str(P/'kit-runtime')),'--/plugins/carb.tasking.plugin/threadCount=2','--/plugins/omni.tbb.globalcontrol/maxThreadCount=2']))
try:
    import omni.usd,carb,omni.replicator.core as rep,builtins
    from isaacsim.core.api import World
    from virtual_sensors import VirtualSensorRig
    from integrated_scenes import make_obstacles
    scene=P/'JETIN_Rover_Start_20261009.usda' if args.scenario=='flat' else make_obstacles()[0]
    assert omni.usd.get_context().open_stage(str(scene));stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
    carb.settings.get_settings().set('/physics/updateToUsd',True);carb.settings.get_settings().set('/physics/suppressReadback',False)
    world=World(physics_dt=1/120,rendering_dt=1/120,physics_prim_path='/PhysicsScene',sim_params=dict(use_fabric=False,use_gpu_pipeline=False),backend='numpy',device='cpu');world.get_physics_context().enable_fabric(False);world.reset()
    state=runpy.run_path(str(P/'tools/install_manufacturer_models.py'))['JETIN_MANUFACTURER_MODELS']
    for _ in range(12):world.step(render=False)
    if args.gui or args.check_controls:
        controls=runpy.run_path(str(P/'tools/install_jetin_runtime.py'));state=builtins.JETIN_MANUFACTURER_MODELS
        if args.check_controls:
            assert state['wheel_speed_command_m_s'] is None
            assert 'arm4_rotation' in controls['state']['control_models'] and 'ml_rocker_joint' not in controls['state']['control_models']
    rig=VirtualSensorRig()
    if args.ros2:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.ros2.bridge')
        for _ in range(20):app.update()
        from ros2_bridge import JetinROS2
        bridge=JetinROS2(stage,state['rig'].robot,state,rig)
    ann=None;frames=0
    if args.film:
        product=rep.create.render_product('/Environment/Overview',(960,540));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach([product])
        pipe=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','960x540','-r','20','-i','-','-an','-c:v','libx264','-threads','1','-crf','21','-pix_fmt','yuv420p',str(OUT/'overview.mp4')],stdin=subprocess.PIPE)
    for _ in range(24):world.render()
    samples=[]
    for i in range(round(args.seconds*120)):
        t=(i+1)/120;world.step(render=i%4==0);assert state['error'] is None,state['error']
        data=rig.collect() if i%12==0 else None
        if bridge:bridge.step(world.current_time,data)
        if i%12==0:samples.append(dict(time_s=t,feedback=getattr(builtins,'JETIN_MOTION_FEEDBACK',{}).get('samples',{}),encoders=getattr(builtins,'JETIN_AS5048_ENCODERS',{}).get('samples',{})))
        if pipe and i%6==0:pipe.stdin.write(np.asarray(ann.get_data())[:,:,:3].copy().tobytes());frames+=1
    metadata=rig.save(OUT,prefix='final');assert len(metadata)==5 and all(v.get('point_count',1)>0 for n,v in metadata.items() if n.endswith('depth'))
    (OUT/'feedback.json').write_text(json.dumps(samples,indent=2,default=str),encoding='utf8')
    r.update(passed=True,status='completed',streams=metadata,seconds=args.seconds,frames=frames,controls_created=bool(args.gui or args.check_controls),ros2_topics=list(bridge.publishers) if bridge else []);save();print('SENSOR_SCENE_RESULT',json.dumps(r),flush=True)
    if args.gui:
        while app.is_running():
            world.step(render=True)
            if bridge and world.is_playing():bridge.step(world.current_time,rig.collect())
    rig.close();world.stop()
except Exception:
    r.update(status='failed',error=traceback.format_exc());save();print(r['error'],flush=True);raise
finally:
    if pipe:pipe.stdin.close();pipe.wait(timeout=60)
    if bridge:bridge.close()
    save();app.close()
