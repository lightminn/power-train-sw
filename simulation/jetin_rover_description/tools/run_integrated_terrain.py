"""Actual uneven-terrain traversal with four read-only AS5048B encoders."""
import os
for key in ['OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','OMP_NUM_THREADS']:os.environ[key]='1'
from pathlib import Path
import json,sys,argparse,traceback,csv,math,subprocess,time,runpy,hashlib,numpy as np
PACKAGE=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--height',type=float,default=.12);p.add_argument('--wavelength',type=float,default=.9);p.add_argument('--seconds',type=float,default=48);p.add_argument('--speed',type=float,default=.09);p.add_argument('--probe',action='store_true');p.add_argument('--gui',action='store_true');args=p.parse_args()
OUT=PACKAGE/'results/integrated_terrain_20261010';OUT.mkdir(parents=True,exist_ok=True)
report={'passed':False,'status':'starting','Isaac_job':int(os.environ.get('SLURM_JOB_ID',0)),'terrain':{'height_m':args.height,'wavelength_m':args.wavelength,'start_x_m':0.,'end_x_m':3.,'left_right_phase_deg':180.,'envelope_length_m':.4},'speed_m_s':args.speed,'seconds':args.seconds,'hardware_calibrated':False,'joint_limits_and_geometry_unchanged':True}
def save():(OUT/'report.json').write_text(json.dumps(report,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else str(x)),encoding='utf8')
save();encoders=[];stream=None
from isaacsim import SimulationApp
cpu_threads=int(os.environ.get('SLURM_CPUS_PER_TASK','4'))
app=SimulationApp({'headless':not args.gui,'width':960,'height':540,'renderer':'RaytracedLighting','multi_gpu':False,'active_gpu':int(os.environ.get('SLURM_JOB_GPUS',os.environ.get('CUDA_VISIBLE_DEVICES','0')).split(',')[0]),'physics_gpu':0,'limit_cpu_threads':cpu_threads,'extra_args':['--portable-root',os.environ.get('JETIN_KIT_RUNTIME',str(PACKAGE/'kit-runtime')),f'--/plugins/carb.tasking.plugin/threadCount={cpu_threads}',f'--/plugins/omni.tbb.globalcontrol/maxThreadCount={cpu_threads}','--/rtx/hydra/mdlMaterialWarmup=false']})
try:
 import omni.usd,carb
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf,Sdf,UsdShade
 from isaacsim.core.api import World
 sys.path.insert(0,str(PACKAGE/'tools'));from integrated_scenes import make_wave
 scene,generation=make_wave(args.height,args.wavelength)
 assert omni.usd.get_context().open_stage(str(scene))
 stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
 PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath('/JETIN')).CreateSolverPositionIterationCountAttr(128)
 report.update(terrain=generation,asset_sha256=hashlib.sha256((PACKAGE/'JETIN_Rover_Final_20261009.usdc').read_bytes()).hexdigest(),solver_position_iterations=128,actuator_models_enabled=True)
 def camera(path,eye,target,up=(0,0,1)):
  c=UsdGeom.Camera.Define(stage,path);c.CreateFocalLengthAttr(24.);c.CreateClippingRangeAttr(Gf.Vec2f(.01,100));xf=UsdGeom.Xformable(c)
  if not xf.GetOrderedXformOps():xf.AddTransformOp()
  xf.GetOrderedXformOps()[0].Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(*up)).GetInverse())
 def views(x):
  camera('/Task/Side',(x+.12,2.3,1.0),(x+.12,0,.45))
  camera('/Task/Top',(x+.12,-.01,3.6),(x+.12,0,.2),(1,0,0))
 views(0.)
 stage.GetSessionLayer().Export(str(OUT/'scene_session.usda'))
 settings=carb.settings.get_settings();settings.set('/physics/updateToUsd',True);settings.set('/physics/updateVelocitiesToUsd',True);settings.set('/physics/suppressReadback',False)
 world=World(physics_dt=1/120,rendering_dt=1/120,stage_units_in_meters=1,physics_prim_path='/PhysicsScene',sim_params={'use_fabric':False,'use_gpu_pipeline':False},backend='numpy',device='cpu');world.get_physics_context().enable_fabric(False);world.reset()
 state=runpy.run_path(str(PACKAGE/'tools/install_manufacturer_models.py'))['JETIN_MANUFACTURER_MODELS']
 for _ in range(12):world.step(render=False)
 assert state['rig'] is not None and state['error'] is None
 sys.path.insert(0,str(PACKAGE/'tools'));from bl70200_controller import IsaacWheelController
 from as5048b_encoders import SuspensionEncoderRig,wrap
 motor=IsaacWheelController(stage);rig=SuspensionEncoderRig(motor.robot)
 wheels=['fl','fr','cl','cr','rl','rr'];ball_names=[s+'_rod_end_'+e+'_'+a for s in ['ml','mr'] for e in ['A','B'] for a in ['rx','ry','rz']]
 balls=[UsdPhysics.RevoluteJoint(stage.GetPrimAtPath('/JETIN/Joints/'+n)) for n in ball_names]
 limits=[(float(j.GetLowerLimitAttr().Get()),float(j.GetUpperLimitAttr().Get())) for j in balls]
 def ball_angles(cache):
  angles=[]
  for j in balls:
   frames=[]
   for rel,rot in [(j.GetBody0Rel(),j.GetLocalRot0Attr()),(j.GetBody1Rel(),j.GetLocalRot1Attr())]:
    body=stage.GetPrimAtPath(rel.GetTargets()[0]);local=rot.Get()
    frames.append(cache.GetLocalToWorldTransform(body).ExtractRotationQuat()*Gf.Quatd(local.GetReal(),Gf.Vec3d(local.GetImaginary())))
   delta=frames[0].GetInverse()*frames[1];axis={'X':0,'Y':1,'Z':2}[j.GetAxisAttr().Get()]
   angles.append((math.degrees(2*math.atan2(delta.GetImaginary()[axis],delta.GetReal()))+180)%360-180)
  return angles
 anchors=[]
 for side in ['ml','mr']:
  first=UsdPhysics.Joint(stage.GetPrimAtPath('/JETIN/Joints/'+side+'_rod_end_B_rz'))
  anchors.append((stage.GetPrimAtPath(first.GetBody0Rel().GetTargets()[0]),stage.GetPrimAtPath(first.GetBody1Rel().GetTargets()[0]),Gf.Vec3d(first.GetLocalPos0Attr().Get()),Gf.Vec3d(first.GetLocalPos1Attr().Get())))
 annotators=[]
 if not args.probe:
  import omni.replicator.core as rep
  for path,name in [('/Task/Side','terrain_side.mp4'),('/Task/Top','terrain_top.mp4')]:
   product=rep.create.render_product(path,(960,540));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach([product]);annotators.append(ann)
   encoders.append(subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','960x540','-r','30','-i','-','-an','-c:v','libx264','-threads','1','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(OUT/name)],stdin=subprocess.PIPE))
  for _ in range(12):world.render()
 rows=[];motion=[];max_error=0.;max_excess=0.;frames=0;start=time.monotonic()
 stream=(OUT/'encoder_samples.csv').open('w',newline='');keys=['sensor','sample_time_s','raw_count','absolute_angle_deg','joint_angle_deg','reference_joint_angle_rad','joint_velocity_rad_s','error_deg'];writer=csv.DictWriter(stream,keys,extrasaction='ignore');writer.writeheader()
 report['status']='simulating';save()
 for i in range(round(args.seconds*120)):
  t=(i+1)/120;motor.step(0. if t<2 else args.speed,1/120)
  if not args.probe and i%4==0:
   m=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/JETIN/base_link'));views(float(m.ExtractTranslation()[0]))
  world.step(render=not args.probe and i%4==0)
  assert state['error'] is None,state['error']
  data=rig.sample(t);truth=motor.robot.get_joint_positions(joint_indices=rig.indices)
  for spec,actual in zip(rig.config['sensors'],truth):
   r=data.get(spec['name'])
   if r is not None and r['new_sample']:
    row=dict(r,sensor=spec['name'],reference_joint_angle_rad=float(actual),error_deg=float(np.degrees(wrap(r['joint_angle_rad']-float(actual)))));rows.append(row);writer.writerow(row)
  if i%12==0:
   cache=UsdGeom.XformCache();base=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/JETIN/base_link')).ExtractTranslation());assert np.isfinite(base).all() and .15<base[2]<1.5,'Invalid robot pose'
   closure=max((cache.GetLocalToWorldTransform(a).Transform(pa)-cache.GetLocalToWorldTransform(b).Transform(pb)).GetLength() for a,b,pa,pb in anchors);max_error=max(max_error,closure)
   bq=ball_angles(cache);excess=max(max(lo-v,v-hi,0.) for v,(lo,hi) in zip(bq,limits));max_excess=max(max_excess,excess)
   centers={n:list(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/JETIN/'+n+'_bl70200s')).ExtractTranslation()) for n in wheels}
   motion.append({'time_s':t,'base_xyz_m':base.tolist(),'closure_error_m':closure,'ball_limit_excess_deg':float(excess),'wheel_centers':centers})
   assert closure<.01,'Closed loop lost connection'
  if not args.probe and i%4==0:
   for ann,encoder in zip(annotators,encoders):
    frame=np.asarray(ann.get_data());assert frame.shape[:2]==(540,960);encoder.stdin.write(frame[:,:,:3].copy().tobytes())
   frames+=1
   if frames in [1,450,750,1050,1500,1850]:
    from PIL import Image
    for ann,name in zip(annotators,['side','top']):Image.fromarray(np.asarray(ann.get_data())[:,:,:3]).save(OUT/f'{name}_{frames}.png')
  if i%1200==0:
   stream.flush();report.update(time_s=t,frames=frames,forward_x_m=motion[-1]['base_xyz_m'][0],max_closure_error_m=max_error);save();print('INTEGRATED_TERRAIN_PROGRESS',t,report['forward_x_m'],max_error,flush=True)
 metrics={}
 for spec in rig.config['sensors']:
  r=[x for x in rows if x['sensor']==spec['name']];a=np.array([x['joint_angle_deg'] for x in r]);times=np.array([x['sample_time_s'] for x in r]);err=np.array([x['error_deg'] for x in r])
  metrics[spec['name']]={'samples':len(r),'minimum_deg':float(a.min()),'maximum_deg':float(a.max()),'range_deg':float(np.ptp(a)),'max_model_readback_error_deg':float(np.max(np.abs(err))),'sample_interval_max_error_s':float(np.max(np.abs(np.diff(times)-1/60)))}
 all_cleared=all(v[0]-.11>generation['end_x_m'] and v[2]<.16 for v in motion[-1]['wheel_centers'].values())
 large=all(m['range_deg']>=(8 if name.startswith('rocker') else 10) for name,m in metrics.items())
 fidelity=all(m['max_model_readback_error_deg']<1.25 and m['sample_interval_max_error_s']<1e-7 for m in metrics.values())
 report['encoder_acceptance']='Uncalibrated public-spec noise/INL plus sampled latency; absolute model/readback error under 1.25deg,not measured hardware accuracy'
 report.update(status='completed',time_s=args.seconds,frames=frames,metrics=metrics,all_wheels_cleared=bool(all_cleared),large_motion_all_four=bool(large),encoder_readback_passed=bool(fidelity),max_closure_error_m=max_error,max_ball_limit_excess_deg=float(max_excess),motion=motion,wall_seconds=time.monotonic()-start,
  passed=bool(all_cleared and large and fidelity and max_error<.001 and max_excess<.5));save();print('INTEGRATED_TERRAIN_RESULT',json.dumps({k:v for k,v in report.items() if k not in ['motion','terrain_profiles']}),flush=True);world.stop()
except Exception:
 report.update(status='failed',error=traceback.format_exc());save();print(report['error'],flush=True)
finally:
 if stream:stream.close()
 for encoder in encoders:encoder.stdin.close();encoder.wait(timeout=60)
 app.close()
