import json, math, os, signal, socket, subprocess, time, uuid
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from powertrain_ros import remote_input_gateway

assert os.environ['ROS_DOMAIN_ID'] == '77'
assert '/evidence/install/' in remote_input_gateway.__file__, remote_input_gateway.__file__
assert not Path('/sys/class/net/can0').exists()
print('installed module', remote_input_gateway.__file__, flush=True)
log=open('/evidence/entrypoint.log','w')
child=subprocess.Popen(['ros2','run','powertrain_ros','teleop_command','--ros-args','-p','host:=127.0.0.1','-p','port:=19000'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init(); node=rclpy.create_node('direction_smoke')
received=[]
sub=node.create_subscription(Twist,'/teleop/cmd_vel',lambda m:received.append((m.linear.x,m.angular.z)),10)
sock=None
seq=0; sid=str(uuid.uuid4())
def send(deadman=False,lx=0,rt=0,lt=0):
    global seq
    record={'schema_version':2,'session_id':sid,'sequence':seq,'client_monotonic_ns':time.monotonic_ns(),'mode':'DRIVE','deadman':deadman,'axes':{'left_x':lx,'right_y':0.0,'left_trigger':lt,'right_trigger':rt},'dpad':{'x':0,'y':0},'mode_chord':False,'estop_edge':False,'assist_bypass':False}
    seq+=1; sock.sendall((json.dumps(record)+'\n').encode())
def run_case(name,deadman,lx,rt,lt,expected,duration=.7):
    received.clear(); deadline=time.monotonic()+duration
    while time.monotonic()<deadline:
        assert child.poll() is None, 'entrypoint exited'
        send(deadman,lx,rt,lt); rclpy.spin_once(node,timeout_sec=.03)
        try: sock.recv(8192)
        except BlockingIOError: pass
        time.sleep(.01)
    assert received, (name,'no ROS output')
    matching=[v for v in received if all(math.isclose(a,b,abs_tol=1e-8) for a,b in zip(v,expected))]
    assert matching and all(math.isclose(a,b,abs_tol=1e-8) for a,b in zip(received[-1],expected)), (name,received[-5:])
    print(name,'PASS',received[-1],flush=True)
try:
    deadline=time.monotonic()+12
    while time.monotonic()<deadline:
        try: sock=socket.create_connection(('127.0.0.1',19000),timeout=.3); break
        except OSError: time.sleep(.1)
    assert sock is not None, 'TCP did not open'
    sock.setblocking(False)
    discovery_deadline=time.monotonic()+10
    while node.count_publishers('/teleop/cmd_vel')==0:
        assert time.monotonic()<discovery_deadline, 'DDS publisher discovery timeout'
        rclpy.spin_once(node,timeout_sec=.05)
    run_case('neutral',False,0,0,0,(0,0),1.0)
    for name,lx,rt,lt,expected in [('RT-right',.25,.5,0,(.75,-.3)),('RT-left',-.25,.5,0,(.75,.3)),('LT-right',.25,0,.5,(-.75,-.3)),('LT-left',-.25,0,.5,(-.75,.3))]:
        run_case(name,True,lx,rt,lt,expected)
    run_case('deadman-release',False,.25,.5,0,(0,0))
    sock.close(); sock=None
    received.clear(); deadline=time.monotonic()+.7
    while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
    assert received and received[-1]==(0.0,0.0),received[-5:]
    print('disconnect PASS',flush=True)
finally:
    if sock: sock.close()
    if child.poll() is None:
        os.killpg(child.pid,signal.SIGINT)
        try: child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGKILL); child.wait(); raise
    log.close(); node.destroy_node(); rclpy.shutdown()
    assert 'Traceback' not in Path('/evidence/entrypoint.log').read_text()
