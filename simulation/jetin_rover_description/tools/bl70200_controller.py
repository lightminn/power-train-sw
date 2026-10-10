"""Recorded motor-side PI parameters, transformed to wheel-side torque.

Pure math core has no Isaac dependencies. Runtime adapter uses articulation
efforts, with wheel USD velocity drives disabled only in the session layer.
Gearbox efficiency is a scalar torque approximation. Hall/PLL feedback uses
the published ODrive core with interpolated physical angles. Backlash,
compliance and firmware current-loop dynamics are not emulated.
"""
import json
import math
from pathlib import Path
from manufacturer_models import HallPLL

WHEELS = ['fl', 'fr', 'cl', 'cr', 'rl', 'rr']
DEFAULT_SETTINGS = Path(__file__).resolve().parents[1] / 'config/bl70200_recorded_settings.json'

class MotorPI:
    def __init__(self, settings):
        self.s = settings
        self.reset()

    def reset(self):
        self.reference_motor_tps = 0.0
        self.integral_motor_Nm = 0.0
        self.last = {}

    def update(self, command_wheel_rad_s, measured_wheel_rad_s, dt):
        if dt <= 0 or not all(map(math.isfinite, [command_wheel_rad_s, measured_wheel_rad_s, dt])):
            raise ValueError('Invalid motor feedback/time step')
        s = self.s
        ratio = s['gear_ratio_motor_to_wheel']
        command = max(-s['motor_velocity_limit_turn_s'], min(s['motor_velocity_limit_turn_s'], command_wheel_rad_s * ratio / (2 * math.pi)))
        step = s['motor_velocity_ramp_turn_s2'] * dt
        self.reference_motor_tps += max(-step, min(step, command - self.reference_motor_tps))
        measured = measured_wheel_rad_s * ratio / (2 * math.pi)
        error = self.reference_motor_tps - measured
        limit = s['torque_constant_motor_Nm_per_A'] * s['motor_current_limit_A']
        p_torque = s['motor_velocity_gain_Nm_per_turn_s'] * error
        raw = p_torque + self.integral_motor_Nm
        motor_torque = max(-limit, min(limit, raw))
        saturated = abs(raw) > limit
        # Conditional integration prevents windup; firmware's 8kHz decay is
        # deliberately not imitated as a 120Hz per-step decay.
        if not saturated or raw * error < 0:
            self.integral_motor_Nm = max(-limit, min(limit, self.integral_motor_Nm + s['motor_velocity_integrator_gain_Nm_per_turn'] * error * dt))
        self.last = dict(reference_motor_tps=self.reference_motor_tps, measured_motor_tps=measured,
                         error_motor_tps=error, commanded_motor_torque_Nm=motor_torque,
                         estimated_command_current_A=motor_torque/s['torque_constant_motor_Nm_per_A'],
                         integral_motor_Nm=self.integral_motor_Nm, saturated=saturated)
        return motor_torque * ratio * s['gearbox_efficiency_used']

class IsaacWheelController:
    def __init__(self, stage, settings_path=DEFAULT_SETTINGS, root='/JETIN'):
        from pxr import UsdPhysics
        from isaacsim.core.prims import SingleArticulation
        self.s = json.loads(Path(settings_path).read_text(encoding='utf-8'))
        self.stage = stage
        self.drives = []
        for name in WHEELS:
            drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(root+'/Joints/'+name+'_wheel_joint'), 'angular')
            drive.CreateStiffnessAttr(0)
            drive.CreateDampingAttr(0)
            drive.CreateTargetVelocityAttr(0)
            self.drives.append(drive)
        self.robot = SingleArticulation(prim_path=root, name='jetin_recorded_drive')
        self.robot.initialize()
        self.indices = [self.robot.get_dof_index(n+'_wheel_joint') for n in WHEELS]
        controller = self.robot.get_articulation_controller()
        for index in self.indices:
            controller.switch_dof_control_mode(index, 'effort')
        kp, kd = controller.get_gains()
        assert all(abs(float(kp[i])) < 1e-9 and abs(float(kd[i])) < 1e-9 for i in self.indices), 'Wheel drive gains must be zero for effort PI'
        self.models = [MotorPI(self.s) for _ in WHEELS]
        self.hall_models=[HallPLL(cpr=60,ratio=self.s['gear_ratio_motor_to_wheel'],bandwidth=30) for _ in WHEELS]
        self.time_s=0.
        self.last = {}

    def reset(self):
        for model in self.models:
            model.reset()
        for model in self.hall_models:model.reset()
        self.time_s=0.

    def step(self, speed_m_s, dt):
        import numpy as np
        velocity = self.robot.get_joint_velocities(joint_indices=np.array(self.indices))
        if velocity is None:
            raise RuntimeError('Articulation wheel velocity feedback unavailable')
        command = speed_m_s / self.s['wheel_radius_command_conversion_m']
        position=self.robot.get_joint_positions(joint_indices=np.array(self.indices));self.time_s+=dt
        feedback=[m.update(float(q),self.time_s) for m,q in zip(self.hall_models,position)]
        measured=np.array([v['velocity_rad_s'] for v in feedback])
        efforts = [model.update(command, float(v), dt) for model, v in zip(self.models, measured)]
        # Isaac5.1 subset effort API otherwise clears steering/arm commands.
        full=np.asarray(self.robot.get_applied_joint_efforts()).copy()
        full[np.asarray(self.indices)]=efforts;self.robot.set_joint_efforts(full)
        self.last = {n: dict(m.last, commanded_wheel_torque_Nm=t, measured_wheel_rad_s=float(v),simulation_true_wheel_rad_s=float(truth),feedback_model=f['model'])
                     for n, m, t, v,truth,f in zip(WHEELS, self.models, efforts, measured,velocity,feedback)}
        return self.last
