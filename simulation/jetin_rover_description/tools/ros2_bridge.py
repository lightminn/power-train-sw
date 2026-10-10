"""ROS2 transport for actual simulated sensors and bounded motor commands.

Uses Isaac Sim's bundled Python3.11 ROS2 libraries. Never publishes object
ground truth into the perception topics. Passive suspension is read only.
"""
import os,json,math,builtins,sys
from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parents[1]

def configure_environment(domain=171,localhost=True):
    os.environ.setdefault('ROS_DISTRO','humble')
    os.environ.setdefault('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    os.environ.setdefault('ROS_DOMAIN_ID',str(domain))
    if localhost:os.environ.setdefault('ROS_LOCALHOST_ONLY','1')
    import importlib.util
    root=Path(importlib.util.find_spec('isaacsim').origin).parent/'exts/isaacsim.ros2.bridge'
    lib=root/os.environ['ROS_DISTRO']/'lib'
    existing=os.environ.get('LD_LIBRARY_PATH','').split(':')
    if str(lib) not in existing:os.environ['LD_LIBRARY_PATH']=':'.join([*filter(None,existing),str(lib)])
    # glibc's loader reads LD_LIBRARY_PATH at process startup,so the command
    # line runner restarts once before SimulationApp loads any ROS libraries.
    if os.environ.get('JETIN_ROS2_BOOTSTRAPPED')!='1':
        os.environ['JETIN_ROS2_BOOTSTRAPPED']='1'
        os.execv(sys.executable,[sys.executable,*sys.argv])
    return root

class JetinROS2:
    def __init__(self,stage,robot,state,cameras=None):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState,Imu,Image,CameraInfo,PointCloud2,PointField
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        from std_msgs.msg import Float64
        self.rclpy=rclpy;self.owned=not rclpy.ok()
        if self.owned:rclpy.init()
        self.node=rclpy.create_node('jetin_simulation');self.stage=stage;self.robot=robot;self.state=state;self.cameras=cameras
        self.JointState=JointState;self.Imu=Imu;self.Image=Image;self.CameraInfo=CameraInfo;self.PointCloud2=PointCloud2;self.PointField=PointField;self.Clock=Clock;self.TFMessage=TFMessage
        self.publishers={};self.command_count=0;self.rejected=[];self.last={};self.t=0.;self.next_joint_time=0.
        for topic,typ in [('/clock',Clock),('/jetin/joint_states',JointState),('/jetin/encoders',JointState),('/jetin/wheel_steering_feedback',JointState),('/jetin/l515/imu',Imu),('/tf',TFMessage)]:
            self.publishers[topic]=self.node.create_publisher(typ,topic,10 if typ in [Clock,TFMessage] else qos_profile_sensor_data)
        cfg=json.loads((P/'config/manufacturer_models.json').read_text(encoding='utf8'))
        self.allowed=set(cfg['confirmed_servo_joints'])|{s+'_'+j for s in ['fl','fr','rl','rr'] for j in ['steer_joint']}
        self.node.create_subscription(JointState,'/jetin/arm_steering_command',self.joint_command,10)
        self.node.create_subscription(Float64,'/jetin/wheel_speed_command',self.wheel_command,10)
        self.node.create_subscription(Float64,'/jetin/gripper_command',self.gripper_command,10)
        self.feedback=json.loads((P/'config/motion_feedback.json').read_text(encoding='utf8'))
        self.enc=json.loads((P/'config/as5048b_encoders.json').read_text(encoding='utf8'))
    def stamp(self,t):
        from builtin_interfaces.msg import Time
        s=Time();s.sec=int(t);s.nanosec=round((t-int(t))*1e9)
        if s.nanosec>=1000000000:s.sec+=1;s.nanosec-=1000000000
        return s
    def wheel_command(self,m):
        if not math.isfinite(m.data) or abs(m.data)>.3:self.rejected.append('wheel speed outside +/-0.3m/s');return
        self.state['wheel_speed_command_m_s']=float(m.data);self.command_count+=1
    def gripper_command(self,m):
        x=self.JointState();x.name=['pinion_A_rotation'];x.position=[float(m.data)];self.joint_command(x)
    def joint_command(self,m):
        from pxr import UsdPhysics
        if len(m.name)!=len(m.position):self.rejected.append('joint command length');return
        updates=[]
        for name,q in zip(m.name,m.position):
            if name not in self.allowed or not math.isfinite(q):self.rejected.append('unknown/passive/nonfinite joint: '+name);return
            p=self.stage.GetPrimAtPath('/JETIN/Joints/'+name);d=UsdPhysics.DriveAPI.Get(p,'angular')
            if not d:self.rejected.append('missing drive: '+name);return
            lo=p.GetAttribute('physics:lowerLimit').Get();hi=p.GetAttribute('physics:upperLimit').Get();deg=math.degrees(q)
            if (lo is not None and deg<lo) or (hi is not None and deg>hi):self.rejected.append('CAD angle limit: '+name);return
            updates.append((d,deg))
        for d,q in updates:d.GetTargetPositionAttr().Set(float(q))
        self.command_count+=1
    def publish(self,topic,m):self.publishers[topic].publish(m);self.last[topic]=m
    def step(self,t,rendered=None):
        if self.state['rig'] is None:return
        if self.state['rig'].robot is not self.robot or t<self.t-1e-9:
            self.last={};self.next_joint_time=float(t)
        self.robot=self.state['rig'].robot
        self.t=float(t);self.rclpy.spin_once(self.node,timeout_sec=0)
        c=self.Clock();c.clock=self.stamp(t);self.publish('/clock',c)
        if rendered:self.publish_cameras(rendered)
        self.publish_imu(t)
        self.publish_encoders(t)
        if t+1e-9<self.next_joint_time:return
        while self.next_joint_time<=t+1e-9:self.next_joint_time+=1/50
        q=np.asarray(self.robot.get_joint_positions());dq=np.asarray(self.robot.get_joint_velocities())
        j=self.JointState();j.header.stamp=self.stamp(t);j.header.frame_id='base_link';j.name=list(self.robot.dof_names);j.position=q.astype(float).tolist();j.velocity=dq.astype(float).tolist();self.publish('/jetin/joint_states',j)
        samples=getattr(builtins,'JETIN_MOTION_FEEDBACK',{}).get('samples',{})
        f=self.JointState();f.header.stamp=self.stamp(t);f.header.frame_id='base_link'
        for spec in self.feedback['joints']:
            row=samples.get('wheels' if spec['kind']=='wheel' else 'steering',{}).get(spec['name'])
            if row:f.name.append(spec['joint']);f.position.append(float(row['joint_position_rad']));f.velocity.append(float(row['joint_velocity_rad_s']))
        self.publish('/jetin/wheel_steering_feedback',f)
        self.publish_tf(t)
    def publish_encoders(self,t):
        enc=getattr(builtins,'JETIN_AS5048_ENCODERS',{}).get('samples',{})
        if not enc:return
        stamp=max(r['sample_time_s'] for r in enc.values())
        if stamp<=self.last.get('_encoder_stamp',-1):return
        self.last['_encoder_stamp']=stamp
        e=self.JointState();e.header.stamp=self.stamp(stamp);e.header.frame_id='base_link'
        for spec in self.enc['sensors']:
            row=enc.get(spec['name'])
            if row:e.name.append(spec['joint']);e.position.append(float(row['joint_angle_rad']));e.velocity.append(float(row['joint_velocity_rad_s'] or 0.))
        self.publish('/jetin/encoders',e)
    def publish_imu(self,t):
        row=getattr(builtins,'JETIN_MOTION_FEEDBACK',{}).get('samples',{}).get('imu',{})
        if row.get('valid') and row['sample_time_s']>self.last.get('_imu_stamp',-1):
            self.last['_imu_stamp']=row['sample_time_s']
            imu=self.Imu();imu.header.stamp=self.stamp(row['sample_time_s']);imu.header.frame_id=row['accel_frame_id'];imu.orientation_covariance[0]=-1.
            for dest,key in [(imu.linear_acceleration,'acceleration_sensor_m_s2'),(imu.angular_velocity,'angular_velocity_sensor_rad_s')]:dest.x,dest.y,dest.z=map(float,row[key])
            # Covariances unknown; zero means unknown in sensor_msgs/Imu.
            self.publish('/jetin/l515/imu',imu)
    def publish_tf(self,t):
        from pxr import UsdGeom,UsdPhysics
        from geometry_msgs.msg import TransformStamped
        from scipy.spatial.transform import Rotation
        cache=UsdGeom.XformCache();msg=self.TFMessage()
        paths=[p for p in self.stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith('/JETIN/')]
        if self.cameras:paths.extend(self.stage.GetPrimAtPath(s['frame_path']) for s in self.cameras.specs.values())
        paths.extend(p for p in self.stage.Traverse() if p.GetName() in [self.feedback['imu']['accel_frame_id'],self.feedback['imu']['gyro_frame_id']])
        seen=set()
        for p in paths:
            name=p.GetName()
            if name in seen:continue
            seen.add(name);M=np.asarray(cache.GetLocalToWorldTransform(p)).T
            tr=TransformStamped();tr.header.stamp=self.stamp(t);tr.header.frame_id='world';tr.child_frame_id=name;tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=map(float,M[:3,3]);quat=Rotation.from_matrix(M[:3,:3]).as_quat();tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=map(float,quat);msg.transforms.append(tr)
        self.publish('/tf',msg)
    def publish_cameras(self,samples):
        from rclpy.qos import qos_profile_sensor_data
        for name,data in samples.items():
            frame=Path(self.cameras.specs[name]['frame_path']).name;stamp=self.stamp(data['rendering_time']);prefix='/jetin/'+name
            arrays=[('image',data['rgb'],'rgb8')]
            if 'depth_m' in data:arrays.append(('depth',data['depth_m'],'32FC1'))
            for suffix,a,encoding in arrays:
                topic=prefix+'/'+suffix
                if topic not in self.publishers:self.publishers[topic]=self.node.create_publisher(self.Image,topic,qos_profile_sensor_data)
                m=self.Image();m.header.stamp=stamp;m.header.frame_id=frame;m.height,m.width=a.shape[:2];m.encoding=encoding;m.is_bigendian=0;m.step=int(a.strides[0]);m.data=np.ascontiguousarray(a).tobytes();self.publish(topic,m)
            topic=prefix+'/camera_info'
            if topic not in self.publishers:self.publishers[topic]=self.node.create_publisher(self.CameraInfo,topic,qos_profile_sensor_data)
            m=self.CameraInfo();m.header.stamp=stamp;m.header.frame_id=frame;m.width,m.height=self.cameras.specs[name]['resolution'];m.distortion_model='plumb_bob';m.k=data['K'].reshape(-1).tolist();m.d=self.cameras.specs[name]['distortion_coefficients'];m.r=np.eye(3).reshape(-1).tolist();m.p=np.c_[data['K'],np.zeros(3)].reshape(-1).tolist();self.publish(topic,m)
            if 'depth_K' in data:
                # Image-plane depth follows the USD pinhole projection. Its
                # intrinsics must not be replaced with the RGB lens matrix.
                topic=prefix+'/depth_camera_info'
                if topic not in self.publishers:self.publishers[topic]=self.node.create_publisher(self.CameraInfo,topic,qos_profile_sensor_data)
                m=self.CameraInfo();m.header.stamp=stamp;m.header.frame_id=frame;m.width,m.height=self.cameras.specs[name]['resolution'];m.distortion_model='plumb_bob';m.k=data['depth_K'].reshape(-1).tolist();m.d=[0.]*5;m.r=np.eye(3).reshape(-1).tolist();m.p=np.c_[data['depth_K'],np.zeros(3)].reshape(-1).tolist();self.publish(topic,m)
            if 'points_optical_m' in data:
                topic=prefix+'/points'
                if topic not in self.publishers:self.publishers[topic]=self.node.create_publisher(self.PointCloud2,topic,qos_profile_sensor_data)
                a=np.ascontiguousarray(data['points_optical_m'],dtype='<f4');m=self.PointCloud2();m.header.stamp=stamp;m.header.frame_id=frame;m.height=1;m.width=len(a);m.is_dense=True;m.is_bigendian=False;m.point_step=12;m.row_step=12*len(a)
                m.fields=[self.PointField(name=n,offset=4*i,datatype=self.PointField.FLOAT32,count=1) for i,n in enumerate('xyz')];m.data=a.tobytes();self.publish(topic,m)
    def close(self):
        self.node.destroy_node()
        if self.owned and self.rclpy.ok():self.rclpy.shutdown()
