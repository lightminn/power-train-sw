"""Read-only AS5048B single-turn virtual encoders on passive suspension axes.

I2C model identity is metadata; no electrical bus/register emulation. Angle
zero, polarity, sampling, noise and latency are explicit simulation settings.
"""
from pathlib import Path
import json,math,collections
import numpy as np
PACKAGE=Path(__file__).resolve().parents[1]
def wrap(value):return (value+math.pi)%(2*math.pi)-math.pi

class AS5048BEncoder:
    def __init__(self,spec,seed=0):
        self.spec=spec;self.N=1<<spec['resolution_bits'];self.lsb=2*math.pi/self.N
        self.sign=spec['joint_to_encoder_sign'];self.zero=math.radians(spec['zero_offset_deg'])
        self.period=1/spec['sample_hz'];self.rng=np.random.default_rng(seed);self.reset()
    def reset(self):
        self.next_sample=None;self.last=None;self.queue=collections.deque();self.output=None;self.last_time=None;self.start_time=None
    def update(self,angle_rad,time_s):
        if not math.isfinite(angle_rad) or not math.isfinite(time_s):raise ValueError('Nonfinite encoder input')
        if self.last_time is not None and time_s<self.last_time-1e-9:self.reset()
        self.last_time=time_s;new=False
        if self.start_time is None:self.start_time=time_s
        if time_s-self.start_time<self.spec.get('startup_time_s',0):return None
        if self.next_sample is None:self.next_sample=time_s
        if time_s+1e-9>=self.next_sample:
            phase=self.sign*angle_rad+self.zero
            phase+=math.radians(self.spec.get('inl_amplitude_deg',0))*math.sin(2*phase+self.spec.get('inl_phase_rad',0))
            phase+=math.radians(float(self.rng.normal(0,self.spec['noise_std_deg'])))
            count=int(math.floor((phase%(2*math.pi))/self.lsb+.5))%self.N
            absolute=count*self.lsb;velocity=None
            if self.last is not None:
                dt=time_s-self.last['sample_time_s']
                if dt>0:velocity=self.sign*wrap(absolute-self.last['absolute_angle_rad'])/dt
            row={'raw_count':count,'absolute_angle_rad':absolute,'absolute_angle_deg':math.degrees(absolute),
                 'joint_angle_rad':self.sign*wrap(absolute-self.zero),'joint_angle_deg':math.degrees(self.sign*wrap(absolute-self.zero)),
                 'joint_velocity_rad_s':velocity,'sample_time_s':time_s,'hardware_calibrated':False,
                 'measurement_model':'AS5048B_14bit_noise_INL_propagation_approximation'}
            self.last=row;self.queue.append((time_s+self.spec['latency_s'],row))
            while self.next_sample<=time_s+1e-9:self.next_sample+=self.period
        while self.queue and self.queue[0][0]<=time_s+1e-9:
            _,self.output=self.queue.popleft();new=True
        return dict(self.output,new_sample=new) if self.output is not None else None

class SuspensionEncoderRig:
    def __init__(self,robot=None):
        from isaacsim.core.prims import SingleArticulation
        self.config=json.loads((PACKAGE/'config/as5048b_encoders.json').read_text(encoding='utf8'))
        self.robot=robot
        if self.robot is None:
            self.robot=SingleArticulation(prim_path='/JETIN',name='jetin_suspension_encoder_readback');self.robot.initialize()
        self.indices=np.array([self.robot.get_dof_index(s['joint']) for s in self.config['sensors']])
        self.models={s['name']:AS5048BEncoder(s,i) for i,s in enumerate(self.config['sensors'])}
        self.latest={}
    def sample(self,time_s):
        angles=self.robot.get_joint_positions(joint_indices=self.indices)
        if angles is None:raise RuntimeError('Encoder articulation readback unavailable; press Play first')
        self.latest={}
        for s,angle in zip(self.config['sensors'],angles):
            value=self.models[s['name']].update(float(angle),float(time_s))
            if value is not None:self.latest[s['name']]=value
        return self.latest
    def reset(self):
        for model in self.models.values():model.reset()
        self.latest={}
