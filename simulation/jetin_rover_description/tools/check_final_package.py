"""Headless independent final-package check. Run with Isaac Sim Python."""
import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
from pathlib import Path
import json, sys, runpy, traceback, builtins,hashlib
import numpy as np
P=Path(__file__).resolve().parents[1];OUT=P/'validation_output';OUT.mkdir(exist_ok=True)
report=dict(passed=False,status='starting',job=int(os.environ.get('SLURM_JOB_ID',0)),package=str(P))
def save():(OUT/'portable_report.json').write_text(json.dumps(report,indent=2),encoding='utf8')
save()
from isaacsim import SimulationApp
gpu=int(os.environ.get('SLURM_JOB_GPUS',os.environ.get('CUDA_VISIBLE_DEVICES','0')).split(',')[0])
warm=os.environ.get('JETIN_KIT_RUNTIME',str(P/'kit-runtime'))
app=SimulationApp(dict(headless=True,multi_gpu=False,active_gpu=gpu,physics_gpu=0,limit_cpu_threads=2,
    extra_args=['--portable-root',warm,'--/plugins/carb.tasking.plugin/threadCount=2','--/plugins/omni.tbb.globalcontrol/maxThreadCount=2','--/rtx/hydra/mdlMaterialWarmup=false']))
try:
    import omni.usd,carb
    from pxr import Usd, UsdUtils
    from audit_integrated_asset import audit
    static=audit();assert static['passed_static']
    from isaacsim.core.api import World
    deps=UsdUtils.ComputeAllDependencies(str(P/'JETIN_Rover_Start_20261009.usda'))
    assert len(deps[0])==2 and not deps[1] and not deps[2]
    assert all(Path(x.realPath).is_relative_to(P) for x in deps[0])
    assert omni.usd.get_context().open_stage(str(P/'JETIN_Rover_Start_20261009.usda'))
    stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
    settings=carb.settings.get_settings();settings.set('/physics/updateToUsd',True);settings.set('/physics/suppressReadback',False)
    world=World(physics_dt=1/120,rendering_dt=1/120,stage_units_in_meters=1,physics_prim_path='/PhysicsScene',sim_params=dict(use_fabric=False,use_gpu_pipeline=False),backend='numpy',device='cpu')
    world.get_physics_context().enable_fabric(False);world.reset()
    state=runpy.run_path(str(P/'tools/install_manufacturer_models.py'))['JETIN_MANUFACTURER_MODELS']
    for _ in range(240):world.step(render=False)
    assert state['error'] is None and state['rig'] is not None and state['wheel_speed_command_m_s'] is None
    assert builtins.JETIN_MOTION_FEEDBACK['error'] is None and builtins.JETIN_MOTION_FEEDBACK['samples']['imu']['valid']
    assert builtins.JETIN_AS5048_ENCODERS['error'] is None and len(builtins.JETIN_AS5048_ENCODERS['samples'])==4
    assert Path(sys.modules['manufacturer_models'].__file__).is_relative_to(P)
    assert len(state['servo_samples'])==8
    ui=runpy.run_path(str(P/'tools/install_jetin_runtime.py'))['JETIN_RUNTIME_UI']
    state=builtins.JETIN_MANUFACTURER_MODELS
    assert 'arm4_rotation' in ui['control_models'] and 'ml_rocker_joint' not in ui['control_models']
    runpy.run_path(str(P/'tools/install_virtual_sensors.py'))
    for _ in range(24):world.step(render=True)
    data=builtins.JETIN_SENSOR_RIG.collect();assert len(data)==5
    for name,row in data.items():
        assert row['rgb'].size>0 and np.isfinite(row['rgb']).all()
        if 'depth_m' in row:assert len(row['points_optical_m'])>100
    builtins.JETIN_SENSOR_RIG.save(OUT,prefix='portable');builtins.JETIN_SENSOR_RIG.close()
    world.stop()
    assert state['rig'] is None and builtins.JETIN_AS5048_ENCODERS['rig'] is None and builtins.JETIN_MOTION_FEEDBACK['rig'] is None
    world.play()
    for _ in range(48):world.step(render=False)
    assert state['error'] is None and state['rig'] is not None
    assert builtins.JETIN_MOTION_FEEDBACK['samples']['imu']['valid'] and len(builtins.JETIN_AS5048_ENCODERS['samples'])==4
    report.update(passed=True,status='completed',independent_dependencies=2,no_external_geometry=True,automatic_wheel_command=False,
                  sensor_streams=list(data),point_counts={n:len(r['points_optical_m']) for n,r in data.items() if 'points_optical_m' in r},stop_play_passed=True,controls_created=True,eight_servo_feedback_channels=len(state['servo_samples']),static_audit=static,asset_sha256=hashlib.sha256((P/'JETIN_Rover_Final_20261009.usdc').read_bytes()).hexdigest())
    save();print('PORTABLE_PASS',json.dumps(report),flush=True)
except Exception as exc:
    report.update(passed=False,status='failed',error=str(exc),traceback=traceback.format_exc());save();print(report['traceback'],flush=True);raise
finally:app.close()
