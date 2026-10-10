"""One entry point for portable JETIN scenarios,run with Isaac Python."""
from pathlib import Path
import argparse,sys,runpy
P=Path(__file__).resolve().parents[1]
a=argparse.ArgumentParser(description='JETIN integrated robot simulation')
a.add_argument('--scenario',choices=['flat','obstacles','terrain','vision','ros2-check'],default='flat')
a.add_argument('--film',action='store_true');a.add_argument('--gui',action='store_true');a.add_argument('--ros2',action='store_true');a.add_argument('--seed',type=int,default=20261010);a.add_argument('--seconds',type=float,default=10)
args=a.parse_args();extra=[]
if args.scenario=='vision':
    target='run_vision_pick.py';extra=['--seed',str(args.seed)]+(['--film'] if args.film else [])
elif args.scenario=='terrain':
    target='run_integrated_terrain.py';extra=[] if args.film else ['--probe']
elif args.scenario=='ros2-check':target='run_ros2_probe.py'
else:
    target='run_sensor_scene.py';extra=['--scenario',args.scenario,'--seconds',str(args.seconds)]+(['--film'] if args.film else [])+(['--ros2'] if args.ros2 else [])
if args.gui:
    if args.scenario=='ros2-check':raise SystemExit('ros2-check is an automated headless validation')
    extra+=['--gui']
if args.ros2 and args.scenario in ['vision','terrain']:raise SystemExit('Use flat/obstacles --ros2 for external control; autonomous vision/terrain owns its actuator targets.')
sys.argv=[str(P/'tools'/target),*extra];runpy.run_path(sys.argv[0],run_name='__main__')
