"""Real rendered OV5640/D435i observations command force-driven five-axis pick.

Object ground truth is used only by the evaluator,not perception or IK goals.
No attachment constraint,kinematic object or joint-position teleportation.
"""
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
from pathlib import Path
import sys,json,math,traceback,argparse,runpy,subprocess,time,hashlib
import numpy as np
P=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--seed',type=int,default=20261010);parser.add_argument('--perception-only',action='store_true');parser.add_argument('--film',action='store_true');parser.add_argument('--gui',action='store_true');parser.add_argument('--diagnostic-seconds',type=int);args=parser.parse_args()
OUT=P/'results'/('vision_'+str(args.seed)+('_perception' if args.perception_only else ''));OUT.mkdir(parents=True,exist_ok=True)
report=dict(passed=False,status='starting',job=int(os.environ.get('SLURM_JOB_ID',0)),seed=args.seed,
    asset_sha256=hashlib.sha256((P/'JETIN_Rover_Final_20261009.usdc').read_bytes()).hexdigest(),
    object_kinematic=False,object_attached_by_constraint=False,controller_uses_object_ground_truth=False,
    detector='RGB color segmentation,not a general unknown-object model',hardware_calibrated=False,samples=[],contacts=[],arm_contacts=[],observations=[])
def save():(OUT/'report.json').write_text(json.dumps(report,indent=2,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else float(x) if isinstance(x,np.generic) else str(x)),encoding='utf8')
save();pipes=[]
from isaacsim import SimulationApp
gpu=int(os.environ.get('SLURM_JOB_GPUS',os.environ.get('CUDA_VISIBLE_DEVICES','0')).split(',')[0]);threads=int(os.environ.get('SLURM_CPUS_PER_TASK',2))
app=SimulationApp(dict(headless=not args.gui,multi_gpu=False,active_gpu=gpu,physics_gpu=0,limit_cpu_threads=threads,width=960,height=540,renderer='RaytracedLighting',
    extra_args=['--portable-root',os.environ.get('JETIN_KIT_RUNTIME',str(P/'kit-runtime')),f'--/plugins/carb.tasking.plugin/threadCount={threads}',f'--/plugins/omni.tbb.globalcontrol/maxThreadCount={threads}','--/rtx/hydra/mdlMaterialWarmup=false']))
try:
    import omni.usd,omni.physx,carb
    from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,PhysicsSchemaTools
    from isaacsim.core.api import World
    from scipy.optimize import least_squares
    from PIL import Image
    sys.path.insert(0,str(P/'tools'))
    from integrated_scenes import make_vision
    from virtual_sensors import VirtualSensorRig
    from vision_perception import estimate_rgbd,wrist_correction,detect_orange
    from grasp_kinematics import forward,ARM_NAMES,TCP_ZERO
    scene,generation=make_vision(args.seed);report['generation_for_evaluator_only']=generation
    assert omni.usd.get_context().open_stage(str(scene));stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
    carb.settings.get_settings().set('/physics/updateToUsd',True);carb.settings.get_settings().set('/physics/suppressReadback',False)
    world=World(physics_dt=1/120,rendering_dt=1/120,stage_units_in_meters=1,physics_prim_path='/PhysicsScene',sim_params=dict(use_fabric=False,use_gpu_pipeline=False),backend='numpy',device='cpu');world.get_physics_context().enable_fabric(False);world.reset()
    state=runpy.run_path(str(P/'tools/install_manufacturer_models.py'))['JETIN_MANUFACTURER_MODELS']
    for _ in range(12):world.step(render=False)
    assert state['rig'] is not None and state['error'] is None
    robot=state['rig'].robot
    from bl70200_controller import IsaacWheelController
    wheels=IsaacWheelController(stage)
    plan=json.loads((P/'config/export_plan.json').read_text(encoding='utf8'))
    armidx=np.array([robot.get_dof_index(n) for n in ARM_NAMES]);drives=[UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/JETIN/Joints/'+n),'angular') for n in ARM_NAMES]
    pin=UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/JETIN/Joints/pinion_A_rotation'),'angular')
    pinidx=np.array([robot.get_dof_index(n) for n in ['pinion_A_rotation','pinion_B_rotation','finger_lower_slide','finger_upper_slide']])
    camera=VirtualSensorRig(names=['ov5640_rgb','d435i_rgb','d435i_depth'])
    for _ in range(18):world.render()
    import omni.replicator.core as rep
    product=rep.create.render_product('/Task/Wide',(960,540));wide=rep.AnnotatorRegistry.get_annotator('rgb');wide.attach([product])
    if args.film:
        for name,size in [('wide',(960,540)),('ov5640',(640,480)),('d435i_rgb',(1280,720)),('d435i_depth',(848,480))]:
            pipe=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{size[0]}x{size[1]}','-r','20','-i','-','-an','-c:v','libx264','-threads','1','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(OUT/(name+'.mp4'))],stdin=subprocess.PIPE);pipes.append(pipe)
        for _ in range(12):world.render()
    clock=[0.];force={'lower':0.,'upper':0.,'other':0.}
    def on_contact(headers,data):
        for h in headers:
            a=str(PhysicsSchemaTools.intToSdfPath(h.actor0));b=str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            f=sum(float(np.linalg.norm(np.asarray(data[k].impulse))) for k in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))*120
            if f>.2 and any(n in a+b for n in ['arm_link','arm_tool','finger_','pinion_','toolchanger_']) and len(report['arm_contacts'])<20000:
                report['arm_contacts'].append(dict(time_s=clock[0],a=a,b=b,collider0=str(PhysicsSchemaTools.intToSdfPath(h.collider0)),collider1=str(PhysicsSchemaTools.intToSdfPath(h.collider1)),force_N=f,
                    points=[dict(position_m=list(data[k].position),separation_m=float(data[k].separation)) for k in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]))
            if '/Task/Object' not in a+b:continue
            key='lower' if 'finger_lower' in a+b else 'upper' if 'finger_upper' in a+b else 'other';force[key]+=f
            if f>.05 and len(report['contacts'])<15000:report['contacts'].append(dict(time_s=clock[0],a=a,b=b,force_N=f))
    subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
    low=np.array([-80.,-179.9,-179.9,-99.9,-179.9]);high=np.array([80.,-.000001,-.000001,99.9,179.9])
    def base_matrix():return np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/JETIN/base_link'))).T
    def solve(goal,q0):
        B=base_matrix();R=B[:3,:3];pos=B[:3,3]
        def residual(q):
            p,A=forward(plan,q);return np.r_[(R@p+pos-goal)*8,(R@A)[2,0]*.4,(R@A)[2,1]*.4]
        sol=least_squares(residual,np.clip(q0,low+1e-7,high-1e-7),bounds=(low,high),max_nfev=60,ftol=1e-7,xtol=1e-7,gtol=1e-7)
        return sol.x,float(np.linalg.norm(residual(sol.x)[:3])/8)
    smooth=lambda u:(max(0.,min(1.,u))**3)*(10-15*max(0.,min(1.,u))+6*max(0.,min(1.,u))**2)
    def blend(a,b,t,start,end):return np.asarray(a)+(np.asarray(b)-a)*smooth((t-start)/(end-start))
    observations=[];estimate=None;preq=None;pickq=None;liftq=None;fine_done=False;frames=0;initial_z=None;started=time.monotonic();sensor_data={};world_goal=None;pre_world=None;home_world=None;closed_q=None
    gap0=.061;open_grip=40.;close_grip=None;phase='settle';seconds=6 if args.perception_only else (args.diagnostic_seconds or 92)
    report['status']='simulating';save()
    for i in range(seconds*120):
        t=(i+1)/120;clock[0]=t
        if t<5:
            q=np.zeros(5);grip=open_grip*smooth(t/2);phase='rgbd_detect'
        else:
            if estimate is None or preq is None:raise RuntimeError('RGB-D object detection failed;no ground-truth fallback')
            if t<25:q=blend(np.zeros(5),preq,t,5,25);grip=open_grip;phase='approach_observation_pose'
            elif t<28:q=preq;grip=open_grip;phase='ov5640_refine'
            elif t<33:q=preq;grip=open_grip;phase='raise_clearance'
            elif t<39:q=pickq;grip=open_grip;phase='approach_above_support'
            elif t<44:q=pickq;grip=open_grip;phase='descend'
            elif t<50:q=pickq;grip=float(blend(open_grip,close_grip,t,44,50));phase='close'
            elif t<60:q=blend(pickq,liftq,t,50,60);grip=close_grip;phase='lift'
            elif t<66:q=liftq;grip=close_grip;phase='hold'
            elif t<74:q=blend(liftq,pickq,t,66,74);grip=close_grip;phase='replace'
            elif t<78:q=pickq;grip=float(blend(close_grip,open_grip,t,74,78));phase='release'
            else:q=preq;grip=open_grip;phase='retreat'
        # Re-solve against actual floating rover pose. World camera target is
        # retained when the arm occludes the front RGB-D view.
        if t>=5 and not args.perception_only:
            if home_world is None:
                B=base_matrix();hp,_=forward(plan,np.rad2deg(robot.get_joint_positions(joint_indices=armidx)));home_world=B[:3,:3]@hp+B[:3,3]
            if t<25:cart=blend(home_world,pre_world,t,5,25)
            elif t<28:cart=pre_world
            elif t<33:cart=blend(pre_world,np.r_[pre_world[:2],world_goal[2]+.08],t,28,33)
            elif t<39:cart=blend(np.r_[pre_world[:2],world_goal[2]+.08],world_goal+np.array([0,0,.08]),t,33,39)
            elif t<44:cart=world_goal+np.array([0,0,.08*(1-smooth((t-39)/5))])
            elif t<50:cart=world_goal
            elif t<60:cart=world_goal+np.array([0,0,.10*smooth((t-50)/10)])
            elif t<66:cart=world_goal+np.array([0,0,.10])
            elif t<74:cart=world_goal+np.array([0,0,.10*(1-smooth((t-66)/8))])
            elif t<78:cart=world_goal
            elif t<84:cart=world_goal+np.array([0,0,.08*smooth((t-78)/6)])
            else:cart=blend(world_goal+np.array([0,0,.08]),np.r_[pre_world[:2],world_goal[2]+.08],t,84,92)
            if i%12==0 or closed_q is None:
                closed_q,residual=solve(cart,q if closed_q is None else closed_q)
                if t>24:assert residual<.015,('Current world target unreachable',t,residual)
            q=closed_q
        for d,v in zip(drives,q):d.GetTargetPositionAttr().Set(float(v))
        pin.GetTargetPositionAttr().Set(float(grip))
        for k in force:force[k]=0.
        wheels.step(0.,1/120);world.step(render=i%4==0)
        assert state['error'] is None and np.isfinite(robot.get_joint_positions()).all()
        if i%12==0:
            sensor_data=camera.collect()
            if len(sensor_data)==3 and 2<t<5:
                measured=estimate_rgbd(sensor_data['d435i_rgb'],sensor_data['d435i_depth'])
                if measured:
                    observations.append(measured);report['observations'].append(dict(time_s=t,**measured))
                    if len(observations)>=3:
                        estimate=dict(measured);estimate['center_world_m']=np.median([x['center_world_m'] for x in observations[-8:]],axis=0)
                        pre=estimate['center_world_m']-np.array([.12,0,0]);pre_world=pre.copy();preq,err=solve(pre,[-4,-110,-25,-80,0])
                        assert err<.01,('Perceived pregrasp unreachable',err)
                        close_grip=math.degrees((estimate['width_m']-.002-gap0)/(2*.017999994345347013))
            if len(sensor_data)==3 and 25.5<t<27.5 and not fine_done:
                update=estimate_rgbd(sensor_data['d435i_rgb'],sensor_data['d435i_depth'])
                if update:
                    # Front view can be partly occluded by the approaching arm;
                    # do not infer a new center from a cropped silhouette.
                    reliable=update if update['height_m']>.75*estimate['height_m'] else estimate
                    wrist=wrist_correction(sensor_data['ov5640_rgb'],reliable)
                    if wrist and wrist['correction_m']<.03:
                        estimate=reliable;center_goal=.5*np.asarray(reliable['center_world_m'])+.5*wrist['center_world_m']
                        # Grasp upper part of the observed upright object so
                        # the physical finger's 77mm tall plate clears support.
                        grasp_offset=min(.025,.3*reliable['height_m']);goal=center_goal+np.array([0,0,grasp_offset]);world_goal=goal.copy()
                        pickq,err=solve(goal,preq);liftq,lifterr=solve(goal+np.array([0,0,.10]),pickq)
                        assert max(err,lifterr)<.01,('Visual target unreachable',err,lifterr)
                        report['visual_target']=dict(rgbd=reliable,occluded_current_rgbd=update,ov5640=wrist,fused_center_m=center_goal,fused_goal_m=goal,grasp_above_center_m=grasp_offset,IK_error_m=err,pre_joint_deg=preq,pick_joint_deg=pickq,lift_joint_deg=liftq)
                        camera.save(OUT,prefix='visual_refine');fine_done=True
            if t>=28 and not fine_done and not args.perception_only:raise RuntimeError('OV5640 target not visible at pregrasp;no ground-truth fallback')
            # Evaluator only: controller above never reads /Task/Object pose.
            cache=UsdGeom.XformCache();opos=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/Task/Object')).ExtractTranslation())
            link=cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/JETIN/arm_link5'))
            tcp=np.asarray(link.Transform(Gf.Vec3d(*(TCP_ZERO-np.array([.23571,.00025,.32006856815031626])))))
            if t>2 and initial_z is None:initial_z=float(opos[2])
            actual=np.rad2deg(robot.get_joint_positions(joint_indices=armidx));g=robot.get_joint_positions(joint_indices=pinidx)
            B=base_matrix();fp,_=forward(plan,actual);model_tcp=B[:3,:3]@fp+B[:3,3]
            report['samples'].append(dict(time_s=t,phase=phase,command_arm_deg=q,actual_arm_deg=actual,
                tcp_m=tcp,model_tcp_m=model_tcp,model_geometry_error_m=float(np.linalg.norm(model_tcp-tcp)),base_matrix=B,object_center_m=opos,lift_m=float(opos[2]-(initial_z or opos[2])),object_tcp_error_m=float(np.linalg.norm(opos-tcp)),
                contacts_N=force.copy(),rack_error_m=float(max(abs(g[2]+.017999994345347013*g[0]),abs(g[3]-.017999994345347013*g[0]))),
                tracking_error_deg=float(max(abs(actual-q)))))
        if args.film and i%6==0:
            sensor_data=camera.collect()
            if len(sensor_data)!=3:raise RuntimeError('Camera output missing while filming')
            import cv2
            depth=sensor_data['d435i_depth']['depth_m'];vis=cv2.applyColorMap(np.uint8(np.clip(depth/2,0,1)*255),cv2.COLORMAP_TURBO)[:,:,::-1].copy();vis[depth<=0]=0
            data=[np.asarray(wide.get_data())[:,:,:3],sensor_data['ov5640_rgb']['rgb'],sensor_data['d435i_rgb']['rgb'],vis]
            for pipe,frame in zip(pipes,data):pipe.stdin.write(frame.copy().tobytes())
            frames+=1
        if i%600==0:
            report.update(time_s=t,phase=phase,frames=frames);save();print('VISION_PROGRESS',t,phase,'detection',len(observations),'q',np.rad2deg(robot.get_joint_positions(joint_indices=armidx)).tolist(),flush=True)
            image_data=np.asarray(wide.get_data())
            if image_data.shape[:2]==(540,960):Image.fromarray(image_data[:,:,:3]).save(OUT/('wide_'+str(round(t))+'.png'))
    if args.diagnostic_seconds:
        report.update(passed=False,status='diagnostic_completed',full_grasp_completed=False,diagnostic_seconds=seconds)
    elif args.perception_only:
        assert estimate is not None and len(observations)>=3
        camera.save(OUT,prefix='perception')
        last=report['samples'][-1];error=float(np.linalg.norm(estimate['center_world_m']-np.asarray(last['object_center_m'])))
        report.update(passed=error<.025,status='perception_completed',perceived=estimate,evaluator_position_error_m=error,full_grasp_completed=False)
    else:
        held=[r for r in report['samples'] if 61<r['time_s']<65];released=[r for r in report['samples'] if 80<r['time_s']<82]
        metrics=dict(held_min_lift_m=min(r['lift_m'] for r in held),held_max_object_tcp_error_m=max(r['object_tcp_error_m'] for r in held),
            held_both_contact_fraction=sum(r['contacts_N']['lower']>.05 and r['contacts_N']['upper']>.05 for r in held)/len(held),
            release_height_error_m=max(abs(r['lift_m']) for r in released),rack_max_error_m=max(r['rack_error_m'] for r in report['samples']),
            max_tracking_error_deg=max(r['tracking_error_deg'] for r in report['samples']),
            evaluator_vision_position_error_m=float(np.linalg.norm(np.asarray(report['visual_target']['fused_center_m'])-np.asarray(report['samples'][30]['object_center_m']))),
            max_kinematic_geometry_error_m=max(r['model_geometry_error_m'] for r in report['samples']))
        report.update(metrics=metrics,full_sequence_executed=True,status='completed',frames=frames,wall_seconds=time.monotonic()-started)
        report['passed']=bool(fine_done and metrics['held_min_lift_m']>.065 and metrics['held_max_object_tcp_error_m']<.04 and metrics['held_both_contact_fraction']>.7 and metrics['release_height_error_m']<.02 and metrics['rack_max_error_m']<.0003 and metrics['max_kinematic_geometry_error_m']<.001)
        report['full_grasp_completed']=report['passed']
    save();print('VISION_RESULT',json.dumps({k:v for k,v in report.items() if k not in ['samples','contacts','arm_contacts','observations']},default=lambda x:x.tolist() if isinstance(x,np.ndarray) else str(x)),flush=True)
    camera.close();world.stop()
except Exception as exc:
    report.update(passed=False,status='failed',error=str(exc),traceback=traceback.format_exc());save();print(report['traceback'],flush=True);raise
finally:
    for pipe in pipes:
        pipe.stdin.close();pipe.wait(timeout=60)
    save();app.close()
