import json,time
import rclpy
from powertrain_msgs.msg import WheelStates
rclpy.init();node=rclpy.create_node('manual_deploy_readonly_check');seen=[]
node.create_subscription(WheelStates,'/wheel_states',lambda m:seen.append(m),1)
end=time.monotonic()+6
while time.monotonic()<end and len(seen)<3:rclpy.spin_once(node,timeout_sec=.2)
assert seen,'no wheel feedback'
m=seen[-1]
rows=[dict(name=w.name,drive=w.drive_turns_per_s,drive_command=w.command_turns_per_s,steer=w.steer_deg,drive_stale=w.drive_stale,steer_stale=w.steer_stale,drive_error=w.drive_axis_error,steer_fault=w.steer_fault) for w in m.wheels]
print(json.dumps({'chassis_mode':m.chassis_mode,'stop_state':m.stop_state,'wheels':rows}))
assert len(rows)==6
assert all(not w['drive_stale'] and not w['steer_stale'] and w['drive_error']==0 and w['steer_fault']==0 for w in rows)
assert all(w['drive_command']==0 for w in rows)
node.destroy_node();rclpy.shutdown()
