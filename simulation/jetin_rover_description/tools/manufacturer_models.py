"""Public-specification approximations. Thermal/CAN/ISP hardware is not emulated.

All physical values and assumptions are in config/manufacturer_models.json.
Pure sensor/actuator models are also usable without Isaac Sim.
"""
from pathlib import Path
import json, math
import numpy as np

PACKAGE=Path(__file__).resolve().parents[1]
def config():return json.loads((PACKAGE/'config/manufacturer_models.json').read_text(encoding='utf8'))
def servo_output_spec(cfg,joint):
    """Manufacturer servo OUTPUT plus one external transmission, SI torque/rpm.

    The internal XM540 reduction is already included in its public endpoints.
    Gearbox variant/efficiency/continuous rating are not hardware calibration.
    """
    model=cfg['confirmed_servo_joints'][joint]
    spec=dict(cfg['xm540' if model=='XM540' else 'xl430'])
    tr=cfg.get('servo_transmissions',{}).get(joint,{})
    ratio=float(tr.get('external_ratio',1.));eta=float(tr.get('efficiency',1.))
    assert ratio>=1 and 0<eta<=1
    spec['stall_torque_Nm']*=ratio*eta
    spec['no_load_speed_rpm']/=ratio
    spec['active_torque_cap_Nm']=min(spec['active_torque_cap_Nm']*ratio*eta,float(tr.get('output_torque_cap_Nm',math.inf)))
    spec['external_ratio']=ratio;spec['efficiency']=eta
    return spec

def clip(x,b):return max(-b,min(b,x))
def quantize(x,step):return float(np.round(x/step)*step)

class HallPLL:
    """ODrive fw-v0.5.1 PLL core; interpolated physical angle at nominal8kHz.

    60 Hall edges/motor turn, project reduction5, bandwidth30 1/s.
    Interpolation between 120Hz PhysX samples is a simulation assumption.
    """
    def __init__(self,cpr=60,ratio=5,bandwidth=30):
        self.cpr=cpr;self.ratio=ratio;self.kp=2*bandwidth;self.ki=bandwidth**2;self.reset()
    def reset(self):self.previous=None;self.last_time=None;self.pos=0.;self.vel=0.;self.raw_count=0
    def update(self,wheel_angle,time_s):
        phase=wheel_angle*self.ratio*self.cpr/(2*math.pi)
        if self.last_time is None or time_s<self.last_time:
            self.pos=phase;self.vel=0.;self.previous=phase;self.last_time=time_s
        dt=time_s-self.last_time
        if dt>0:
            count=max(1,int(math.ceil(dt*8000)));h=dt/count
            for i in range(count):
                measured=self.previous+(phase-self.previous)*(i+1)/count
                self.raw_count=math.floor(measured)
                self.pos+=h*self.vel
                error=self.raw_count-math.floor(self.pos)
                self.pos+=h*self.kp*error;self.vel+=h*self.ki*error
                if abs(self.vel)<.5*h*self.ki:self.vel=0.
        self.previous=phase;self.last_time=time_s
        return dict(angle_rad=self.pos/self.cpr/self.ratio*2*math.pi,
                    velocity_rad_s=self.vel/self.cpr/self.ratio*2*math.pi,
                    hall_count=self.raw_count,model='Hall60CPR_motor/ODrive_PLL_bw30/ratio5')

class BMI085Model:
    def __init__(self,spec=None):
        self.spec=spec or config()['l515_imu'];self.reset()
    def reset(self):
        s=self.spec;self.rng=np.random.default_rng(s['seed']);self.last_time=None;self.output=None
        self.acc_bias=self.rng.uniform(-s['accel_bias_bound_g'],s['accel_bias_bound_g'],3)*9.80665
        self.gyro_bias=np.deg2rad(self.rng.uniform(-s['gyro_bias_bound_deg_s'],s['gyro_bias_bound_deg_s'],3))
    def update(self,acc,gyro,time_s):
        if self.last_time is not None and time_s<self.last_time:self.reset()
        if self.last_time is not None and time_s<=self.last_time+1e-9:return self.output
        s=self.spec;T=s['temperature_C']-25.
        an=np.asarray(s['accel_noise_density_g_sqrtHz'])*9.80665*math.sqrt(s['accel_bandwidth_hz'])
        gn=math.radians(s['gyro_noise_density_deg_s_sqrtHz'])*math.sqrt(s['gyro_bandwidth_hz'])
        a=np.asarray(acc)+self.acc_bias+self.rng.normal(0,an,3)+T*s['accel_temp_bias_g_K']*9.80665
        g=np.asarray(gyro)+self.gyro_bias+self.rng.normal(0,gn,3)+T*math.radians(s['gyro_temp_bias_deg_s_K'])
        a=np.round(np.clip(a,-s['accel_range_g']*9.80665,s['accel_range_g']*9.80665)/s['accel_lsb_m_s2'])*s['accel_lsb_m_s2']
        g=np.round(np.clip(g,-math.radians(s['gyro_range_deg_s']),math.radians(s['gyro_range_deg_s']))/s['gyro_lsb_rad_s'])*s['gyro_lsb_rad_s']
        self.output=(a.astype(np.float32),g.astype(np.float32));self.last_time=time_s
        return self.output

class L515DepthModel:
    def __init__(self,spec=None):self.spec=spec or config()['l515_depth'];self.rng=np.random.default_rng(self.spec['seed'])
    def update(self,depth,rays):
        s=self.spec;r=np.linalg.norm(rays,axis=-1);distance=depth*r
        limit=s['valid_range_m'][1] if s['reflectivity_percent']>=95 else s['low_reflectivity_range_m'][1]
        valid=np.isfinite(distance)&(distance>=s['valid_range_m'][0])&(distance<=limit)
        sigma=np.interp(distance,s['sigma_anchor_distance_m'],s['sigma_anchor_m'])
        measured=distance+s['mean_bias_m']+self.rng.normal(size=depth.shape)*sigma
        valid &= (measured>=s['valid_range_m'][0])&(measured<=limit)
        return np.where(valid,measured/r,0).astype(np.float32),valid

class AKServo:
    def __init__(self,spec=None):self.s=spec or config()['ak45'];self.reset()
    def reset(self):self.reference=None;self.reference_velocity=0.;self.last={}
    def envelope(self,velocity):
        s=self.s;rpm=abs(velocity)*60/(2*math.pi)
        return min(s['active_continuous_torque_Nm'],float(np.interp(rpm,s['curve_rpm'],s['curve_torque_Nm'])))
    def update(self,target,q,dq,dt):
        if dt<=0 or not np.isfinite([target,q,dq,dt]).all():raise ValueError('Invalid AK input')
        s=self.s
        if self.reference is None:self.reference=q
        step=math.radians(s['command_velocity_limit_deg_s'])*dt
        desired=clip((target-self.reference)/dt,step/dt)
        acceleration=math.radians(s['command_acceleration_deg_s2'])*dt
        self.reference_velocity+=clip(desired-self.reference_velocity,acceleration)
        advance=self.reference_velocity*dt
        if (target-self.reference)*advance>0 and abs(advance)>abs(target-self.reference):advance=target-self.reference;self.reference_velocity=0
        self.reference+=advance
        half=math.radians(s['backlash_deg'])/2;error=self.reference-q
        effective=math.copysign(max(0,abs(error)-half),error)
        # Curve is net available output torque; do not double-subtract gearbox
        # losses while powered. Backdrive drag is for explicitly disabled mode.
        torque=clip(s['position_kp_Nm_rad']*effective+s['velocity_kd_Nm_s_rad']*(self.reference_velocity-dq),self.envelope(dq))
        self.last=dict(torque_Nm=torque,torque_limit_Nm=self.envelope(dq),reference_rad=self.reference,reference_velocity_rad_s=self.reference_velocity,
                       measured_position_deg=quantize(math.degrees(q),s['feedback_position_step_deg']),
                       measured_velocity_erpm=quantize(dq*60/(2*math.pi)*36*14,s['feedback_speed_step_erpm']))
        return torque
    def unpowered_drag(self,dq):return -self.s['backdrive_torque_Nm']*math.tanh(dq/.01)

class IsaacManufacturerActuators:
    """Four AK effort servos and dynamic caps on existing confirmed servo drives.

    Steering and confirmed arm target attributes stay editable; external reductions included.
    No robot reset, pose teleportation, grasp mimic rewrite or wheel command here.
    """
    def __init__(self,stage,robot=None):
        from pxr import UsdPhysics
        from isaacsim.core.prims import SingleArticulation
        self.stage=stage;self.cfg=config();self.robot=robot or SingleArticulation(prim_path='/JETIN',name='jetin_public_actuators')
        if robot is None:self.robot.initialize()
        self.ak_names=self.cfg['steering_joints'];self.indices=np.array([self.robot.get_dof_index(n) for n in self.ak_names])
        self.models=[AKServo(self.cfg['ak45']) for _ in self.indices];self.drives=[];self.powered=True
        ctrl=self.robot.get_articulation_controller()
        for name,idx in zip(self.ak_names,self.indices):
            d=UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath('/JETIN/Joints/'+name),'angular');assert d
            self.drives.append(d);d.CreateStiffnessAttr(0.);d.CreateDampingAttr(0.)
            ctrl.switch_dof_control_mode(int(idx),'effort')
        self.caps=[]
        for name,model in self.cfg['confirmed_servo_joints'].items():
            p=stage.GetPrimAtPath('/JETIN/Joints/'+name);assert p and p.GetTypeName()=='PhysicsRevoluteJoint',name
            d=UsdPhysics.DriveAPI.Get(p,'angular')
            if d:self.caps.append((name,self.robot.get_dof_index(name),d,servo_output_spec(self.cfg,name)))
        self.last={};self.servo_last={}
    def step(self,dt):
        q=self.robot.get_joint_positions(joint_indices=self.indices);dq=self.robot.get_joint_velocities(joint_indices=self.indices)
        efforts=[]
        for n,d,m,x,v in zip(self.ak_names,self.drives,self.models,q,dq):
            goal=math.radians(d.GetTargetPositionAttr().Get() or 0.)
            torque=m.update(goal,float(x),float(v),dt) if self.powered else m.unpowered_drag(float(v))
            efforts.append(torque);self.last[n]=dict(m.last,powered=self.powered)
        # Isaac5.1 subset set_joint_efforts zeros EVERY unspecified DOF.
        # Preserve other controllers' forces and submit one complete vector.
        full=np.asarray(self.robot.get_applied_joint_efforts()).copy()
        full[self.indices]=efforts;self.robot.set_joint_efforts(full)
        for name,idx,d,spec in self.caps:
            source=self.cfg.get('dependent_servo_targets',{}).get(name)
            if source:
                from pxr import UsdPhysics
                source_drive=UsdPhysics.DriveAPI.Get(self.stage.GetPrimAtPath('/JETIN/Joints/'+source),'angular')
                d.GetTargetPositionAttr().Set(source_drive.GetTargetPositionAttr().Get() or 0.)
            signed_speed=float(self.robot.get_joint_velocities(joint_indices=np.array([idx]))[0]);speed=abs(signed_speed)
            capacity=max(0.,spec['stall_torque_Nm']*(1-speed/(spec['no_load_speed_rpm']*2*math.pi/60)))
            limit=min(capacity,spec['active_torque_cap_Nm']);d.GetMaxForceAttr().Set(float(limit))
            actual_q=float(self.robot.get_joint_positions(joint_indices=np.array([idx]))[0]);ratio=spec['external_ratio'];step=2*math.pi/spec.get('position_counts',4096)
            count=int(np.round(actual_q*ratio/step));measured_q=count*step/ratio
            measured_speed=quantize(signed_speed*ratio*60/(2*math.pi),spec.get('feedback_velocity_step_rpm',.229))*2*math.pi/60/ratio
            self.servo_last[name]=dict(joint_position_rad=measured_q,joint_speed_rad_s=measured_speed,relative_encoder_count=count,output_torque_cap_Nm=limit,external_ratio=ratio,model=spec['model'],
                feedback_basis='public4096counts/.229rpm quantization before external gearbox;relative zero only;bus/noise/current not emulated',simulation_diagnostic=dict(true_position_rad=actual_q,true_speed_rad_s=signed_speed))
        return self.last
