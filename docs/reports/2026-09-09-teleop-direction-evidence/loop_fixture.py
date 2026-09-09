"""Jetson verification: installed ROS nodes and CAN drivers on vcan77 only."""
import json,os,pathlib,signal,subprocess,time,threading,struct
import can
assert os.environ['ROS_DOMAIN_ID']=='77'
import rclpy
from powertrain_msgs.msg import SafetyVerdict,WheelStates
from robot_arm_msgs.msg import ArmStatus
from powertrain_ros import contract
from geometry_msgs.msg import Twist
from powertrain_ros.command_receipt import ReceiptTimeCallback, ReceiptTimeExecutor
import powertrain_ros.chassis_node as installed_chassis
assert '/evidence/install/' in installed_chassis.__file__, installed_chassis.__file__
base=pathlib.Path('/tmp/integrated-loop'); base.mkdir(exist_ok=True)
out=pathlib.Path('/evidence/loop'); out.mkdir(exist_ok=True)
token=base/'ops_console.token';token.write_text('isolated-loop-test-token');token.chmod(0o600)
config=dict(robot_id='isolated-loop-test',token_file=str(token),host='0.0.0.0',session_port=29002,input_port=29000,ops_port=29001,input_target_port=39000,ops_target_port=39001,destination_file=str(base/'session.json'),lease_timeout_s=2.)
config_path=base/'robot.json';config_path.write_text(json.dumps(config))
rclpy.init();node=rclpy.create_node('isolated_loop_sensor_fixture');executor=ReceiptTimeExecutor();executor.add_node(node)
safety=node.create_publisher(SafetyVerdict,'/safety_verdict',10)
arm=node.create_publisher(ArmStatus,contract.TOPIC_ARM_STATUS,10)
summary={'virtual_can_actuators':True,'domain':77,'max_feedback':0.,'last_mode':None,'last_feedback':None,'wheel_messages':0,'stopped_after_motion':False}
def wheels(message):
    velocity=max((abs(w.drive_turns_per_s) for w in message.wheels),default=0.)
    summary.update(last_mode=message.chassis_mode,last_feedback=velocity,wheel_messages=summary['wheel_messages']+1)
    summary['max_feedback']=max(summary['max_feedback'],velocity)
    if summary['max_feedback']>.1 and velocity<.001 and message.chassis_mode=='IDLE':summary['stopped_after_motion']=True
node.create_subscription(WheelStates,'/wheel_states',wheels,10)
summary['installed_chassis_module']=installed_chassis.__file__
summary['dds_receipt_samples']=0
summary['dds_receipt_invalid']=0
def received_twist(message, info):
    summary['dds_receipt_samples']+=1
    age=(time.time_ns()-info['received_timestamp'])/1e9
    if info['received_timestamp']<=0 or not 0<=age<.3:summary['dds_receipt_invalid']+=1
node.create_subscription(Twist,'/teleop/cmd_vel',ReceiptTimeCallback(received_twist),1)
commands=[
 ['ros2','launch','powertrain_ros','control.launch.py','input_host:=127.0.0.1','input_port:=39000','ops_host:=127.0.0.1','ops_port:=39001','ops_token_dir:='+str(base)],
 ['ros2','run','powertrain_ros','chassis','--ros-args','-p','fake:='+('true' if os.environ.get('NEGATIVE_FAKE_ONLY')=='1' else 'false'),'-p','channel:=vcan77','-p','authority_enabled:=true','-p','console_estop_latch_path:='+str(base/'latch.json'),'-p','mission_id_path:='+str(base/'mission-id'),'-p','safety_startup_timeout:=5.0'],
 ['ros2','run','powertrain_ros','chassis_telemetry','--ros-args','-p','operator_host:=127.0.0.1','-p','operator_port:=15005','-p','publish_hz:=10.0'],
 ['python3','-m','powertrain_runtime.robot_service','--config',str(config_path)],
]
running=True
for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:globals().__setitem__('running',False))

# Protocol emulator opens only the dedicated virtual bus. It never accesses can0.
assert not pathlib.Path('/sys/class/net/can0').exists()
assert pathlib.Path('/sys/class/net/vcan77').exists(), 'create isolated vcan77 first'
emulator_done=threading.Event()
raw_observation={'drive': {}, 'steer': {}}
def emulate_can():
 bus=can.interface.Bus(channel='vcan77',interface='socketcan')
 states={n:1 for n in range(11,17)};velocities={n:0. for n in states};positions={n:0. for n in states}
 steering={n:0. for n in range(1,5)}
 previous=time.monotonic();next_status=previous
 def send(n,c,data):
  bus.send(can.Message(arbitration_id=(n<<5)|c,is_extended_id=False,data=data),timeout=.05)
 try:
  while not emulator_done.is_set():
   now=time.monotonic();dt=now-previous;previous=now
   for n in states:positions[n]+=velocities[n]*dt
   if now>=next_status:
    for n in states:send(n,1,struct.pack('<IBBBB',0,states[n],0,0,0))
    for n in range(1,5):
     bus.send(can.Message(arbitration_id=(41<<8)|n,is_extended_id=True,
                          data=struct.pack('>hhhbb',int(steering[n]*10),0,0,25,0)),timeout=.05)
    next_status=now+.02
   msg=bus.recv(.002)
   if msg is None or msg.is_error_frame:continue
   if msg.is_extended_id:
    if msg.arbitration_id>>8==6:
     n=msg.arbitration_id&255
     steering[n]=struct.unpack('>i',msg.data[:4])[0]/10000.
     raw_observation['steer'][str(n)]=steering[n]
    continue
   n,c=msg.arbitration_id>>5,msg.arbitration_id&31
   if n not in states:continue
   if msg.is_remote_frame:
    if c==9:send(n,c,struct.pack('<ff',positions[n],velocities[n]))
    elif c==20:send(n,c,struct.pack('<ff',0.,0.))
   elif c==7 and len(msg.data)>=4:
    states[n]=struct.unpack('<I',msg.data[:4])[0]
    if states[n]!=8:velocities[n]=0.
    send(n,1,struct.pack('<IBBBB',0,states[n],0,0,0))
   elif c==13 and len(msg.data)==8:
    velocities[n]=struct.unpack('<ff',msg.data)[0] if states[n]==8 else 0.
    raw_observation['drive'][str(n)]=velocities[n]
 finally:bus.shutdown()
emulator=threading.Thread(target=emulate_can,daemon=True)
emulator.start()

processes=[]; logs=[]
try:
 for index,command in enumerate(commands):
  log=(out/f'node-{index}.log').open('w');logs.append(log)
  env=dict(os.environ,POWERTRAIN_OPERATOR_SESSION_FILE=str(base/'session.json'))
  processes.append(subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
 until=time.monotonic()+240
 while running and time.monotonic()<until:
  for p in processes:
   if p.poll() is not None:raise RuntimeError(f'installed node exited: {p.args}, rc={p.returncode}')
  v=SafetyVerdict();v.status=SafetyVerdict.VALID;v.distance_mm=1000.;v.estop_required=False;v.header.stamp=node.get_clock().now().to_msg();v.detail='isolated synthetic sensor fixture'
  safety.publish(v)
  a=ArmStatus();a.status=contract.ARM_STOWED_LOCKED
  stamp=time.monotonic_ns();a.header.stamp.sec=stamp//1_000_000_000;a.header.stamp.nanosec=stamp%1_000_000_000
  arm.publish(a)
  executor.spin_once(timeout_sec=.03)
  (out/'wheel-observation.json').write_text(json.dumps(summary))
  (out/'raw-observation.tmp').write_text(json.dumps(raw_observation))
  (out/'raw-observation.tmp').replace(out/'raw-observation.json')
finally:
 # Stop session first, keeping the actual ROS services alive for disarm.
 for p in reversed(processes):
  if p.poll() is None:
   if p.args[:2]==['ros2','launch']:p.send_signal(signal.SIGINT)
   else:os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for log in logs:log.close()
 emulator_done.set();emulator.join(1)
 executor.shutdown();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 print('Fixture cleanup complete',flush=True)
