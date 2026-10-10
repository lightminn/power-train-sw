"""Actual isolated-package J1..J4 output-side motor/transmission check."""
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
from pathlib import Path
import json,sys,runpy,traceback,math,csv,hashlib,builtins
import numpy as np
P=Path(__file__).resolve().parents[1];OUT=P/'arm_validation_output';OUT.mkdir(exist_ok=True)
report=dict(passed=False,status='starting',job=int(os.environ.get('SLURM_JOB_ID',0)),hardware_calibrated=False,
            asset_sha256=hashlib.sha256((P/'JETIN_Rover_Final_20261009.usdc').read_bytes()).hexdigest())
rows=[]
def save():
    (OUT/'report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    if rows:
        with (OUT/'arm_samples.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,list(rows[0]));w.writeheader();w.writerows(rows)
save()
from isaacsim import SimulationApp
gpu=int(os.environ.get('SLURM_JOB_GPUS',os.environ.get('CUDA_VISIBLE_DEVICES','0')).split(',')[0])
app=SimulationApp(dict(headless=True,multi_gpu=False,active_gpu=gpu,physics_gpu=0,limit_cpu_threads=2,
    extra_args=['--portable-root',os.environ['JETIN_KIT_RUNTIME'],'--/plugins/carb.tasking.plugin/threadCount=2','--/plugins/omni.tbb.globalcontrol/maxThreadCount=2','--/rtx/hydra/mdlMaterialWarmup=false']))
try:
    import omni.usd,carb
    from pxr import Usd,UsdUtils,UsdPhysics,UsdGeom,Gf
    from isaacsim.core.api import World
    deps=UsdUtils.ComputeAllDependencies(str(P/'JETIN_Rover_Start_20261009.usda'))
    assert len(deps[0])==2 and not deps[1] and not deps[2]
    assert all(Path(x.realPath).is_relative_to(P) for x in deps[0])
    assert omni.usd.get_context().open_stage(str(P/'JETIN_Rover_Start_20261009.usda'))
    stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
    if os.environ.get('JETIN_TOOL_MIMIC_HARD')=='1':
        for i in range(1,9):
            p=stage.GetPrimAtPath('/JETIN/Joints/lock_ball_'+str(i))
            p.GetAttribute('physxMimicJoint:rotX:naturalFrequency').Set(0.)
            p.GetAttribute('physxMimicJoint:rotX:dampingRatio').Set(0.)
        report['tool_mimic_candidate']='hard analytical CAD motion-link;geometry and collisions retained'
    carb.settings.get_settings().set('/physics/updateToUsd',True)
    carb.settings.get_settings().set('/physics/suppressReadback',False)
    world=World(physics_dt=1/120,rendering_dt=1/120,stage_units_in_meters=1,physics_prim_path='/PhysicsScene',sim_params=dict(use_fabric=False,use_gpu_pipeline=False),backend='numpy',device='cpu')
    world.get_physics_context().enable_fabric(False);world.reset()
    state=runpy.run_path(str(P/'tools/install_manufacturer_models.py'))['JETIN_MANUFACTURER_MODELS']
    for _ in range(12):world.step(render=False)
    assert state['rig'] is not None and state['error'] is None
    rig=state['rig'];robot=rig.robot
    from manufacturer_models import config,servo_output_spec
    cfg=config();names=['arm_J1_yaw','arm_J2_shoulder','arm_J3_elbow','arm4_rotation']
    ratios=[1.,10.,10.,1.];indices=np.array([robot.get_dof_index(n) for n in names])
    assert set(names)<=set(n for n,i,d,s in rig.caps)
    assert Path(sys.modules['manufacturer_models'].__file__).is_relative_to(P)
    drives=[UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/JETIN/Joints/'+n),'angular') for n in names]
    specs=[servo_output_spec(cfg,n) for n in names]
    assert [s['external_ratio'] for s in specs]==ratios
    for spec in specs:
        assert spec['active_torque_cap_Nm']<=spec['stall_torque_Nm']
    anchors=[]
    for side in ['ml','mr']:
        j=UsdPhysics.Joint(stage.GetPrimAtPath('/JETIN/Joints/'+side+'_rod_end_B_rz'))
        anchors.append((stage.GetPrimAtPath(j.GetBody0Rel().GetTargets()[0]),stage.GetPrimAtPath(j.GetBody1Rel().GetTargets()[0]),Gf.Vec3d(j.GetLocalPos0Attr().Get()),Gf.Vec3d(j.GetLocalPos1Attr().Get())))
    max_closure=0.;errors={n:[] for n in names};limits={n:[] for n in names}
    report['status']='simulating';save()
    for i in range(1920):
        t=(i+1)/120
        ramp=lambda start,end,a,b: a+(b-a)*max(0.,min(1.,(t-start)/(end-start)))
        # Output-side targets; actual movement uses force drives and gravity,
        # never joint-position teleportation or a larger test-local torque cap.
        target=[ramp(2,5,0,10),ramp(2,5,0,-9),ramp(3,5,0,-6),ramp(6,8,0,12)]
        if t>=9:
            target=[ramp(9,13,10,-10),ramp(9,13,-9,-3),ramp(9,13,-6,-12),ramp(9,13,12,-12)]
        for d,v in zip(drives,target):d.GetTargetPositionAttr().Set(float(v))
        world.step(render=False)
        assert state['error'] is None and state['wheel_speed_command_m_s'] is None
        q=np.rad2deg(robot.get_joint_positions(joint_indices=indices));dq=robot.get_joint_velocities(joint_indices=indices)
        assert np.isfinite(q).all() and np.isfinite(dq).all()
        for n,d,s,v,x,goal in zip(names,drives,specs,dq,q,target):
            speedlimit=s['no_load_speed_rpm']*2*math.pi/60
            cap=float(d.GetMaxForceAttr().Get())
            assert 0<=cap<=s['active_torque_cap_Nm']+1e-5
            assert abs(v)<=speedlimit+.025,('output speed exceeds cap',n,v,speedlimit)
            limits[n].append(cap)
            if i%6==0:rows.append(dict(time_s=t,joint=n,target_deg=goal,actual_deg=float(x),velocity_rad_s=float(v),drive_torque_cap_Nm=cap,external_ratio=s['external_ratio']))
            if 8.5<t<9 or t>15:errors[n].append(abs(float(x)-goal))
        if i%12==0:
            cache=UsdGeom.XformCache();closure=max((cache.GetLocalToWorldTransform(a).Transform(pa)-cache.GetLocalToWorldTransform(b).Transform(pb)).GetLength() for a,b,pa,pb in anchors)
            max_closure=max(max_closure,closure);assert closure<.001,'differential closure >1mm'
        if i%600==0:print('ARM_PROGRESS',t,q.tolist(),flush=True)
    summary={}
    for n,s in zip(names,specs):
        r=[x for x in rows if x['joint']==n]
        span=max(x['actual_deg'] for x in r)-min(x['actual_deg'] for x in r)
        tracking=max(errors[n]);assert span>5.,('axis failed to move',n,span)
        assert tracking<3.,('settled target error >3deg',n,tracking)
        summary[n]=dict(external_ratio=s['external_ratio'],span_deg=span,max_settled_target_error_deg=tracking,
            max_actual_speed_rad_s=max(abs(x['velocity_rad_s']) for x in r),speed_cap_rad_s=s['no_load_speed_rpm']*2*math.pi/60,
            active_output_torque_cap_Nm=s['active_torque_cap_Nm'],min_live_drive_cap_Nm=min(limits[n]))
    # Independent gripper actuation must not turn the fifth arm motor.
    assert set(cfg['confirmed_servo_joints'])==set(n for n,i,d,s in rig.caps)
    extra=['axis5_rotation','toolchanger_lock_wheel','pinion_A_rotation','pinion_B_rotation']
    eidx=np.array([robot.get_dof_index(n) for n in extra])
    extra_drives=[UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/JETIN/Joints/'+n),'angular') for n in extra]
    fidx=np.array([robot.get_dof_index(n) for n in ['finger_lower_slide','finger_upper_slide']])
    ballidx=np.array([robot.get_dof_index('lock_ball_'+str(i)) for i in range(1,9)])
    coupling_rows=[];rack=.017999994345347013
    for i in range(960):
        t=i/120
        goal5=0. if t<4 else 15*min(1.,(t-4)/2)
        grip=30*math.sin(t*math.pi/2) if t<4 else 0.
        lock=0. if t<6 else 15*min(1.,t-6)
        for d,v in zip(extra_drives,[goal5,lock,grip,grip]):d.GetTargetPositionAttr().Set(float(v))
        world.step(render=False)
        q=robot.get_joint_positions(joint_indices=eidx);f=robot.get_joint_positions(joint_indices=fidx)
        balls=robot.get_joint_positions(joint_indices=ballidx)
        assert state['error'] is None and np.isfinite(q).all()
        if i%12==0:coupling_rows.append(dict(time_s=t,axis5_deg=float(math.degrees(q[0])),axis5_goal_deg=goal5,
            lock_deg=float(math.degrees(q[1])),pinion_A_deg=float(math.degrees(q[2])),pinion_B_deg=float(math.degrees(q[3])),
            rack_error_m=float(max(abs(f[0]+rack*q[2]),abs(f[1]-rack*q[2]))),lock_balls_max_m=float(np.max(balls)),
            lock_balls_m=balls.tolist(),lock_ball_error_m=float(np.max(abs(balls-math.degrees(q[1])*.00259/22.5)))))
    rack_error=max(r['rack_error_m'] for r in coupling_rows)
    pinion_error=max(abs(r['pinion_A_deg']-r['pinion_B_deg']) for r in coupling_rows)
    isolated5=max(abs(r['axis5_deg']) for r in coupling_rows if 1<r['time_s']<4)
    assert rack_error<.0003 and pinion_error<.5 and isolated5<.5,(rack_error,pinion_error,isolated5)
    ball_error=max(r['lock_ball_error_m'] for r in coupling_rows)
    ball_ranges=np.ptp(np.array([r['lock_balls_m'] for r in coupling_rows]),axis=0)
    assert abs(coupling_rows[-1]['axis5_deg']-15)<1.
    assert ball_error<.0003 and ball_ranges.min()>.001,('tool ball coupling failed',ball_error,ball_ranges.tolist())
    report['eight_motor_coupling']=dict(passed=True,rack_error_m=rack_error,pinion_sync_error_deg=pinion_error,
        axis5_drift_during_gripper_deg=isolated5,axis5_final_deg=coupling_rows[-1]['axis5_deg'],toolchanger_ball_stroke_m=coupling_rows[-1]['lock_balls_max_m'],toolchanger_max_ball_error_m=ball_error,toolchanger_ball_ranges_m=ball_ranges.tolist(),samples=coupling_rows)
    for d in extra_drives:d.GetTargetPositionAttr().Set(0.)
    for _ in range(120):world.step(render=False)
    assert builtins.JETIN_MOTION_FEEDBACK['samples']['imu']['valid'] and len(builtins.JETIN_AS5048_ENCODERS['samples'])==4
    # Five real camera renders from this independent updated-body package.
    runpy.run_path(str(P/'tools/install_virtual_sensors.py'))
    for _ in range(24):world.step(render=True)
    data=builtins.JETIN_SENSOR_RIG.collect();assert len(data)==5
    for name,row in data.items():
        assert row['rgb'].size and np.isfinite(row['rgb']).all()
        if 'depth_m' in row:assert len(row['points_optical_m'])>100
    builtins.JETIN_SENSOR_RIG.save(OUT,prefix='arm_revision');builtins.JETIN_SENSOR_RIG.close()
    world.stop();assert state['rig'] is None
    world.play()
    for _ in range(48):world.step(render=False)
    assert state['error'] is None and state['rig'] is not None
    assert set(names)<=set(n for n,i,d,s in state['rig'].caps)
    report.update(passed=True,status='completed',revision=cfg['revision'],seconds=16.,independent_dependencies=2,
        output_limits_and_movement=summary,max_closure_error_m=max_closure,arm_samples=len(rows),
        sensor_streams=list(data),point_counts={n:len(r['points_optical_m']) for n,r in data.items() if 'points_optical_m' in r},
        imu_and_four_encoders_valid=True,stop_play_passed=True,automatic_wheel_command=False,
        caveat='Torque caps/gains/efficiency are editable theoretical assumptions,not continuous hardware ratings; no full grasp/terrain test in this run.')
    save();print('ARM_PASS',json.dumps(report),flush=True)
except Exception as exc:
    report.update(passed=False,status='failed',error=str(exc),traceback=traceback.format_exc());save();print(report['traceback'],flush=True);raise
finally:app.close()
