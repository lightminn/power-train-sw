"""L515 inertial output and sampled read-only drive feedback, Isaac Sim 5.1.

The articulation supplies ideal joint measurements; this is not Hall/CAN
firmware emulation. IMU orientation is not published as a hardware measurement.
"""
from pathlib import Path
import collections
import json
import math
import numpy as np
from manufacturer_models import HallPLL, BMI085Model, quantize, config as manufacturer_config

PACKAGE = Path(__file__).resolve().parents[1]

class JointFeedbackChannel:
    def __init__(self, spec):
        self.spec = spec
        self.period = 1.0 / spec['sample_hz']
        if self.period <= 0 or spec['latency_s'] < 0:
            raise ValueError('Invalid feedback period or latency')
        self.reset()

    def reset(self):
        self.next_sample = None
        self.last_time = None
        self.pending = collections.deque()
        self.output = None
        self.hall = HallPLL() if self.spec.get('model')=='hall_pll' else None

    def sample(self, position, velocity, time_s):
        if not all(map(math.isfinite, [position, velocity, time_s])):
            raise ValueError('Nonfinite joint feedback')
        if self.last_time is not None and time_s < self.last_time - 1e-9:
            self.reset()
        self.last_time = time_s
        modeled=self.hall.update(position,time_s) if self.hall else None
        fresh = False
        if self.next_sample is None:
            self.next_sample = time_s
        if time_s + 1e-9 >= self.next_sample:
            sign = self.spec['joint_sign']
            q = sign * position - math.radians(self.spec['zero_offset_deg'])
            dq = sign * velocity
            truth_q,truth_dq=q,dq
            model='ideal_articulation_sample'
            if modeled is not None:
                q=sign*modeled['angle_rad']-math.radians(self.spec['zero_offset_deg']);dq=sign*modeled['velocity_rad_s'];model=modeled['model']
            elif self.spec.get('model')=='ak_servo_packet':
                q=math.radians(quantize(math.degrees(q),.1))
                erpm=quantize(dq*60/(2*math.pi)*36*14,10);dq=erpm/(36*14)*2*math.pi/60;model='AK_SERVO_STATUS_1_quantized_50Hz'
            row = dict(joint=self.spec['joint'], sample_time_s=time_s,
                       joint_position_rad=q, joint_velocity_rad_s=dq,
                       hardware_calibrated=False, measurement_model=model,
                       simulation_diagnostic=dict(true_position_rad=truth_q,true_velocity_rad_s=truth_dq))
            if modeled is not None:row['hall_count']=modeled['hall_count']
            if self.spec['kind'] == 'wheel':
                row.update(wheel_speed_rad_s=dq, wheel_speed_rpm=dq*60/(2*math.pi),
                           peripheral_speed_m_s=dq*self.spec['wheel_radius_m'],
                           motor_velocity_turn_s=dq*self.spec['gear_ratio_motor_to_wheel']/(2*math.pi))
            else:
                row.update(steering_angle_rad=q, steering_angle_deg=math.degrees(q),
                           steering_velocity_rad_s=dq)
            self.pending.append((time_s + self.spec['latency_s'], row))
            while self.next_sample <= time_s + 1e-9:
                self.next_sample += self.period
        while self.pending and self.pending[0][0] <= time_s + 1e-9:
            _, self.output = self.pending.popleft()
            fresh = True
        if self.output is None:
            return None
        age = max(0.0, time_s-self.output['sample_time_s'])
        return dict(self.output, new_sample=fresh, sample_age_s=age,
                    valid=age <= self.spec['stale_after_s'])

class MotionFeedbackRig:
    def __init__(self, robot=None):
        import omni.usd
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.sensors.physics import _sensor
        from pxr import UsdGeom, UsdPhysics
        self.stage = omni.usd.get_context().get_stage()
        self.config = json.loads((PACKAGE/'config/motion_feedback.json').read_text(encoding='utf8'))
        self.robot = robot
        if self.robot is None:
            self.robot = SingleArticulation(prim_path=self.config['robot_path'], name='jetin_motion_feedback')
            self.robot.initialize()
        self.specs = self.config['joints']
        self.indices = np.array([self.robot.get_dof_index(s['joint']) for s in self.specs], dtype=int)
        self.channels = {s['joint']: JointFeedbackChannel(s) for s in self.specs}
        self.imu = self.config['imu']
        prim = self.stage.GetPrimAtPath(self.imu['prim_path'])
        if not prim or prim.GetTypeName() != 'IsaacImuSensor':
            raise RuntimeError('Open the updated USD with the L515 ImuSensor prim')
        base = self.stage.GetPrimAtPath(self.config['robot_path']+'/base_link')
        if not base.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError('L515 IMU must inherit the base_link rigid body')
        cache = UsdGeom.XformCache()
        def rotation(path):
            return np.asarray(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(path))).T[:3,:3]
        self.sensor_to_base = rotation(str(base.GetPath())).T @ rotation(self.imu['frame_path'])
        assert np.allclose(self.sensor_to_base.T@self.sensor_to_base, np.eye(3), atol=1e-6)
        self.interface = _sensor.acquire_imu_sensor_interface()
        self.last_imu_time = None
        self.imu_error_model=BMI085Model()
        self.latest = {}

    def sample(self, time_s):
        if self.latest and time_s < self.latest['time_s']-1e-9:
            self.reset()
        positions = self.robot.get_joint_positions(joint_indices=self.indices)
        velocities = self.robot.get_joint_velocities(joint_indices=self.indices)
        if positions is None or velocities is None:
            raise RuntimeError('Articulation feedback unavailable; press Play first')
        wheel, steering = {}, {}
        for spec, q, dq in zip(self.specs, positions, velocities):
            row = self.channels[spec['joint']].sample(float(q), float(dq), float(time_s))
            if row is not None:
                (wheel if spec['kind']=='wheel' else steering)[spec['name']] = row
        reading = self.interface.get_sensor_reading(self.imu['prim_path'], use_latest_data=False, read_gravity=True)
        imu = dict(valid=False, new_sample=False)
        if reading.is_valid:
            stamp = float(reading.time)
            acc = np.array([reading.lin_acc_x, reading.lin_acc_y, reading.lin_acc_z], dtype=float)
            gyro = np.array([reading.ang_vel_x, reading.ang_vel_y, reading.ang_vel_z], dtype=float)
            assert np.isfinite(acc).all() and np.isfinite(gyro).all()
            truth_acc,truth_gyro=acc.copy(),gyro.copy()
            acc,gyro=self.imu_error_model.update(acc,gyro,stamp)
            imu = dict(valid=True, new_sample=self.last_imu_time is None or stamp>self.last_imu_time+1e-9,
                       sample_time_s=stamp, sample_age_s=max(0.,time_s-stamp),
                       accel_frame_id=self.imu['accel_frame_id'], gyro_frame_id=self.imu['gyro_frame_id'],
                       acceleration_sensor_m_s2=acc.tolist(), angular_velocity_sensor_rad_s=gyro.tolist(),
                       acceleration_base_m_s2=(self.sensor_to_base@acc).tolist(),
                       angular_velocity_base_rad_s=(self.sensor_to_base@gyro).tolist(),
                       base_frame_id='base_link', includes_gravity=True,
                       measurement_model='IsaacSim_native_IMU_plus_BMI085_public_spec_approximation', hardware_calibrated=False,
                       simulation_diagnostic=dict(true_acceleration_sensor_m_s2=truth_acc.tolist(),true_angular_velocity_sensor_rad_s=truth_gyro.tolist()))
            self.last_imu_time = stamp
        self.latest = dict(time_s=float(time_s), imu=imu, wheels=wheel, steering=steering)
        return self.latest

    def reset(self):
        for channel in self.channels.values():
            channel.reset()
        self.last_imu_time = None
        self.imu_error_model.reset()
        self.latest = {}

    def save(self, path):
        Path(path).write_text(json.dumps(self.latest, ensure_ascii=False, indent=2), encoding='utf8')
