"""Read-only inverse-geometry and lifecycle audit; never opens hardware."""
from chassis.kinematics import default_geometry, solve_steering
from chassis.odometry import WheelObservation, solve_twist
from chassis.tests.test_chassis_manager import _armed_manager, FakeClock
from chassis.chassis_manager import STEERING_SKID, STEERING_ACKERMANN
import math

geom=default_geometry()
mid=next(w.x for w in geom.wheels if w.name=='mid_left')
count=0
residual=0
for speed in [-100,-1.5,-.4,-1e-9,0,1e-9,.4,1.5,100]:
 for step in range(-100,101):
  steering=step/100
  for yawlimit in [.05,1.2,20]:
   out=solve_steering(geom,speed,steering,max_omega_rad_s=yawlimit)
   vx=(out.wheels['mid_left'].drive_mps+out.wheels['mid_right'].drive_mps)/2
   assert abs(out.omega_applied)<=yawlimit+1e-12
   for w in geom.wheels:
    cmd=out.wheels[w.name]
    angle=math.radians(cmd.steer_deg)
    assert abs(cmd.steer_deg)<=geom.steer_limit_deg+1e-12
    assert abs(cmd.drive_mps)<=geom.drive_limit_mps+1e-12
    assert cmd.drive_mps*speed>=0
    assert abs(cmd.drive_mps*math.sin(angle)-out.omega_applied*(w.x-mid))<1e-12
    assert abs(cmd.drive_mps*math.cos(angle)-(vx-out.omega_applied*w.y))<1e-12
   est=solve_twist(geom,[WheelObservation(c.name,c.drive_mps,c.steer_deg) for c in out.wheels.values()])
   residual=max(residual,est.residual_mps)
   assert abs(est.vx-vx)<1e-10
   assert abs(est.vy+out.omega_applied*mid)<1e-10
   assert abs(est.omega-out.omega_applied)<1e-10
   count+=1
print('PASS manual numeric sweep',count,'cases; max odometry residual',residual)
clock=FakeClock()
m=_armed_manager(clock=clock)
m.set(.4,0,steering=.8)
m.tick()
m.request_steering_mode(STEERING_SKID)
for i in range(80):
 clock.advance(.001)
 m.set(.4,0,steering=.8)
 m.tick()
 if m.steering_mode==STEERING_SKID:break
assert m.steering_mode==STEERING_SKID
assert m.state()['steering'] is None
assert all(c.drive.state()['target_vel']==0 for c in m.corners.values())
m.set(0,0,steering=.8)
m.tick()
assert any(c.drive.state()['target_vel']!=0 for c in m.corners.values())
m.request_steering_mode(STEERING_ACKERMANN)
m.tick()
assert m.state()['steering'] is None
assert all(c.drive.state()['target_vel']==0 for c in m.corners.values())
m.set(0,0,steering=.8)
m.tick()
assert all(c.drive.state()['target_vel']==0 for c in m.corners.values())
print('PASS manual->skid->manual clears old intent; explicit skid stationary turn, Ackermann stationary drive0')
