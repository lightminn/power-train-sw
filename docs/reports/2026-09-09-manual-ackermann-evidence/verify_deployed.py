from pathlib import Path
import json
import powertrain_ros
from powertrain_msgs.msg import ManualDriveCommand
from chassis.kinematics import default_geometry, solve_steering
src=Path('/workspace/ros2/src/powertrain_ros/powertrain_ros')
dst=Path(powertrain_ros.__file__).parent
assert str(dst).startswith('/workspace/ros2/install/'),dst
files=list(src.glob('*.py'))
assert all(p.read_bytes()==(dst/p.name).read_bytes() for p in files)
for name in ('control.launch.py', 'wp5_control.launch.py'):
 p=Path('/workspace/ros2/src/powertrain_ros/launch')/name
 assert p.read_bytes()==(Path('/workspace/ros2/install/powertrain_ros/share/powertrain_ros/launch')/p.name).read_bytes(),p
message=ManualDriveCommand(speed_mps=0.,steering=.5)
assert message.get_fields_and_field_types()=={'speed_mps':'double','steering':'double'}
geom=default_geometry()
assert abs(geom.wheels[0].x-.437747327500)<1e-12
assert all(w.drive_mps==0 for w in solve_steering(geom,0,.5).wheels.values())
print(json.dumps({'installed_source_files_equal':len(files),'changed_launch_files_equal':2,'message_type':str(type(message)),'module':str(dst),'front_x_m':geom.wheels[0].x}))
