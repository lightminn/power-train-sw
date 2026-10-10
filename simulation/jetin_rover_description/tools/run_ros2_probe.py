"""Actual bundled ROS2 message/command loopback in an isolated local domain."""
import os
for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
from pathlib import Path
import sys,json,runpy,traceback,hashlib,math
P=Path(__file__).resolve().parents[1];sys.path.insert(0,str(P/'tools'))
from ros2_bridge import configure_environment
if os.environ.get('SLURM_JOB_ID'):
    os.environ.setdefault('ROS_DOMAIN_ID',str(130+int(os.environ['SLURM_JOB_ID'])%80))
configure_environment()
profile='ros2_frames_20261010' if '--frame-check' in sys.argv else 'ros2_controls_20261010' if '--check-controls' in sys.argv else 'ros2_20261010'
OUT=P/'results'/profile;OUT.mkdir(parents=True,exist_ok=True)
r=dict(passed=False,status='starting',job=int(os.environ.get('SLURM_JOB_ID',0)),domain_id=os.environ['ROS_DOMAIN_ID'],localhost_only=True,asset_sha256=hashlib.sha256((P/'JETIN_Rover_Final_20261009.usdc').read_bytes()).hexdigest())
def save():(OUT/'report.json').write_text(json.dumps(r,indent=2),encoding='utf8')
save()
from isaacsim import SimulationApp
gpu=int(os.environ.get('SLURM_JOB_GPUS',os.environ.get('CUDA_VISIBLE_DEVICES','0')).split(',')[0])
app=SimulationApp(dict(headless=True,multi_gpu=False,active_gpu=gpu,limit_cpu_threads=2,renderer='RaytracedLighting',extra_args=['--portable-root',os.environ['JETIN_KIT_RUNTIME'],'--/plugins/carb.tasking.plugin/threadCount=2','--/plugins/omni.tbb.globalcontrol/maxThreadCount=2']))
try:
    import omni.usd,carb
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension('isaacsim.ros2.bridge')
    for _ in range(20):app.update()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState,Imu,Image,CameraInfo,PointCloud2
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from isaacsim.core.api import World
    from ros2_bridge import JetinROS2
    from virtual_sensors import VirtualSensorRig
    assert omni.usd.get_context().open_stage(str(P/'JETIN_Rover_Start_20261009.usda'))
    stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
    carb.settings.get_settings().set('/physics/updateToUsd',True);carb.settings.get_settings().set('/physics/suppressReadback',False)
    world=World(physics_dt=1/120,rendering_dt=1/120,physics_prim_path='/PhysicsScene',sim_params=dict(use_fabric=False,use_gpu_pipeline=False),backend='numpy',device='cpu');world.get_physics_context().enable_fabric(False);world.reset()
    state=runpy.run_path(str(P/'tools/install_manufacturer_models.py'))['JETIN_MANUFACTURER_MODELS']
    for _ in range(12):world.step(render=False)
    if '--check-controls' in sys.argv:
        import builtins
        controls=runpy.run_path(str(P/'tools/install_jetin_runtime.py'))['JETIN_RUNTIME_UI']
        state=builtins.JETIN_MANUFACTURER_MODELS
        for _ in range(12):world.step(render=False)
        assert 'arm4_rotation' in controls['control_models']
        r['controls_and_ROS_same_runtime']=True
    camera=VirtualSensorRig();bridge=JetinROS2(stage,state['rig'].robot,state,camera)
    recv=rclpy.create_node('jetin_validation_receiver');counts={};last={};subs=[]
    topics=[('/clock',Clock),('/jetin/joint_states',JointState),('/jetin/encoders',JointState),('/jetin/wheel_steering_feedback',JointState),('/jetin/l515/imu',Imu),('/tf',TFMessage)]
    for name,spec in camera.specs.items():
        topics.extend([('/jetin/'+name+'/image',Image),('/jetin/'+name+'/camera_info',CameraInfo)])
        if spec['depth_enabled']:topics.extend([('/jetin/'+name+'/depth',Image),('/jetin/'+name+'/depth_camera_info',CameraInfo),('/jetin/'+name+'/points',PointCloud2)])
    def callback(topic):
        def f(msg):counts[topic]=counts.get(topic,0)+1;last[topic]=msg
        return f
    for topic,typ in topics:subs.append(recv.create_subscription(typ,topic,callback(topic),10 if typ in [Clock,TFMessage] else qos_profile_sensor_data))
    pub=recv.create_publisher(JointState,'/jetin/arm_steering_command',10)
    for _ in range(20):world.render()
    for i in range(120 if '--frame-check' in sys.argv else 7*120):
        t=(i+1)/120;world.step(render=i%4==0)
        data=camera.collect() if i%12==0 else None
        bridge.step(world.current_time,data)
        for _ in range(30):rclpy.spin_once(recv,timeout_sec=0)
        if i in [240,360]:
            m=JointState();m.name=['arm_J1_yaw','fl_steer_joint'];m.position=[-.08726646259971647,.05235987755982989];pub.publish(m)
        if i==600:
            m=JointState();m.name=['rocker_left_joint'];m.position=[1.];pub.publish(m)
    for _ in range(100):rclpy.spin_once(recv,timeout_sec=0)
    missing=[topic for topic,_ in topics if counts.get(topic,0)==0]
    assert not missing,missing
    if '--frame-check' in sys.argv:
        frames={f.child_frame_id for f in last['/tf'].transforms}
        required={last['/jetin/l515/imu'].header.frame_id,*[last['/jetin/'+n+'/image'].header.frame_id for n in camera.specs]}
        assert required<=frames,(required-frames)
        r.update(passed=True,status='frame_check_completed',received_counts=counts,all_published_sensor_frames_in_TF=True,required_sensor_frames=sorted(required),tf_frames=sorted(frames),seconds=1)
        save();print('ROS2_FRAME_RESULT',json.dumps(r),flush=True);recv.destroy_node();bridge.close();camera.close();world.stop();raise SystemExit(0)
    assert len(last['/jetin/encoders'].name)==4 and len(last['/jetin/wheel_steering_feedback'].name)==10
    assert last['/jetin/l515/imu'].header.frame_id
    assert last['/jetin/d435i_depth/depth'].encoding=='32FC1'
    assert last['/jetin/ov5640_rgb/image'].encoding=='rgb8'
    assert bridge.command_count>=2 and bridge.rejected
    j=last['/jetin/joint_states'];actual=dict(zip(j.name,j.position));r.update(observed_commanded_joint_rad={n:actual[n] for n in ['arm_J1_yaw','fl_steer_joint']},received_counts=counts);save()
    assert abs(actual['arm_J1_yaw']+.08726646)<.01
    assert abs(actual['fl_steer_joint']-.05235988)<.015,'PD steady-state error under tire/ground friction plus0.1deg deadband;command-tracking tolerance0.86deg'
    assert counts['/jetin/l515/imu']>=650 and counts['/jetin/encoders']>=400 and counts['/jetin/joint_states']>=340
    now=last['/clock'].clock.sec+last['/clock'].clock.nanosec*1e-9
    for topic in ['/jetin/l515/imu','/jetin/d435i_depth/depth','/jetin/ov5640_rgb/image']:
        stamp=last[topic].header.stamp;assert stamp.sec+stamp.nanosec*1e-9<=now+.02,(topic,'future sensor stamp')
    for name,row in state['servo_samples'].items():
        assert abs(row['joint_position_rad']-row['simulation_diagnostic']['true_position_rad'])<=math.pi/4096/row['external_ratio']+1e-7
    if '--check-controls' in sys.argv:
        old_rig=state['rig'];prior=counts.copy();world.stop();assert state['rig'] is None
        world.play()
        for i in range(48):
            world.step(render=i%4==0);bridge.step(world.current_time,camera.collect() if i%12==0 else None)
            for _ in range(30):rclpy.spin_once(recv,timeout_sec=0)
        assert state['rig'] is not old_rig and bridge.robot is state['rig'].robot
        for topic in ['/jetin/encoders','/jetin/l515/imu','/jetin/joint_states','/jetin/ov5640_rgb/image']:
            assert counts[topic]>prior[topic],(topic,'no publication after Stop/Play')
        r['controls_ROS_stop_play_passed']=True;r['restart_seconds']=.4
    r.update(passed=True,status='completed',received_counts=counts,bounded_commands_accepted=bridge.command_count,passive_command_rejected=bridge.rejected,actual_commanded_joint_rad={n:actual[n] for n in ['arm_J1_yaw','fl_steer_joint']},encoders=last['/jetin/encoders'].name,feedback_channels=last['/jetin/wheel_steering_feedback'].name,tf_transforms=len(last['/tf'].transforms),seconds=7)
    save();print('ROS2_RESULT',json.dumps(r),flush=True);recv.destroy_node();bridge.close();camera.close();world.stop()
except Exception:
    r.update(passed=False,status='failed',error=traceback.format_exc());save();print(r['error'],flush=True);raise
finally:save();app.close()
