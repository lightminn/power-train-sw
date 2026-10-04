"""One AK axis: current+delta (max 30 degrees), hold, return. No ODrive/NVM/link writes.

Default only prints the plan. Execution requires --execute after the normal
CAN owner is stopped and an observer is ready. Uses the production owner lock.
"""
import argparse
import json
import math
import signal
import struct
import sys
import time

NODES=tuple(range(11,17))
class Probe:
    def __init__(self,bus,node,delta_deg=5.,hold_s=1.):
        if not math.isfinite(delta_deg) or not 0<delta_deg<=30:
            raise ValueError('delta must be within (0, 30] degrees')
        if not math.isfinite(hold_s) or not .5<=hold_s<=5:
            raise ValueError('hold must be within [0.5, 5] seconds')
        self.bus,self.node=bus,node
        self.delta_deg,self.hold_s=delta_deg,hold_s
        self.drive={}; self.steer={}; self.baseline=None
    def ingest(self,m):
        if m.is_error_frame or m.is_remote_frame: return
        now=time.monotonic()
        if m.is_extended_id and m.arbitration_id>>8==41 and len(m.data)==8:
            mid=m.arbitration_id&255
            pos,spd,cur,temp,fault=struct.unpack('>hhhbb',m.data)
            self.steer[mid]=(now,pos/10.,fault)
        elif not m.is_extended_id and m.arbitration_id&31==1 and len(m.data)>=5:
            mid=m.arbitration_id>>5
            if mid in NODES:
                self.drive[mid]=(now,struct.unpack('<I',m.data[:4])[0],m.data[4])
    def receive(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            m=self.bus.recv(timeout=min(.01,max(0.,end-time.monotonic())))
            if m is not None: self.ingest(m)
    def check(self):
        now=time.monotonic()
        for mid in NODES:
            value=self.drive.get(mid)
            if value is None or now-value[0]>.5 or value[1]!=0 or value[2]!=1:
                raise RuntimeError('ODrive %s must have fresh error-free IDLE heartbeat: %r'%(mid,value))
        for mid in (1,2,3,4):
            value=self.steer.get(mid)
            if value is None or now-value[0]>.5 or value[2]!=0:
                raise RuntimeError('AK %s status unavailable/faulted: %r'%(mid,value))
        pos=self.steer[self.node][1]
        if not math.isfinite(pos) or abs(pos)>40:
            raise RuntimeError('AK position outside probe range: %r'%pos)
        if self.baseline is not None and abs(pos-self.baseline)>self.delta_deg+2:
            raise RuntimeError('AK displacement exceeded permitted delta plus 2 degrees')
        return pos
    def send(self,packet,payload):
        import can
        if packet not in (3,6): raise ValueError('probe permits only AK position and zero RPM')
        if packet==3 and payload!=bytes(4): raise ValueError('only zero RPM is allowed')
        self.bus.send(can.Message(arbitration_id=(packet<<8)|self.node,data=payload,is_extended_id=True),timeout=.01)
    def stage(self,target,label):
        if self.baseline is None or not self.baseline<=target<=self.baseline+self.delta_deg:
            raise ValueError('target outside baseline..baseline+delta')
        began=time.monotonic(); reached=None
        while time.monotonic()-began<self.delta_deg/6.+self.hold_s+3.:
            pos=self.check()
            # 500 ERPM / 10 in the speed field: about 6 output degrees/s.
            self.send(6,struct.pack('>ihh',round(target*10000),50,200))
            if abs(pos-target)<.6:
                if reached is None:
                    reached=time.monotonic()
                    print(json.dumps({'event':label,'node':self.node,'target':target,'actual':pos}),flush=True)
                elif time.monotonic()-reached>=self.hold_s: return
            else: reached=None
            self.receive(.04)
        raise RuntimeError('AK did not settle at '+label)
    def stop(self):
        failures=[]
        for _ in range(5):
            try: self.send(3,bytes(4))
            except Exception as exc: failures.append(str(exc))
            time.sleep(.05)
        if failures: raise RuntimeError('AK zero RPM send errors: '+repr(failures))
    def run(self):
        self.receive(1.5)
        self.baseline=self.check()
        if abs(self.baseline)>30: raise RuntimeError('baseline must be within +/-30 degrees')
        if abs(self.baseline+self.delta_deg)>35:
            raise RuntimeError('target must stay within +/-35 degrees')
        print(json.dumps({'event':'BASELINE','node':self.node,'deg':self.baseline,'drive_states':{n:v[2] for n,v in self.drive.items()}}),flush=True)
        try:
            self.stage(self.baseline+self.delta_deg,'PLUS_%g'%self.delta_deg)
            self.stage(self.baseline,'RETURNED')
        finally: self.stop()
        print('PROBE_COMPLETE; chosen AK zero RPM; no drive/NVM/link writes',flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--node',type=int,choices=(1,2,3,4),required=True)
    p.add_argument('--delta-deg',type=float,default=5.)
    p.add_argument('--hold-s',type=float,default=1.)
    p.add_argument('--execute',action='store_true')
    args=p.parse_args()
    if not args.execute:
        Probe(None,args.node,delta_deg=args.delta_deg,hold_s=args.hold_s)
        print(json.dumps({'node':args.node,'target':'current + %g output degrees'%args.delta_deg,'hold_s':args.hold_s,'then':'return to current','odrive':'all six must remain IDLE; no writes','needs':'normal CAN owner stopped and physical observer ready'}))
        return
    import can
    from chassis.runtime_lock import RealCanSession
    def interrupted(*_): raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    with RealCanSession(channel='can0',owner='steering-mount-probe'):
        bus=can.interface.Bus(channel='can0',interface='socketcan')
        try: Probe(bus,args.node,delta_deg=args.delta_deg,hold_s=args.hold_s).run()
        finally: bus.shutdown()
if __name__=='__main__': main()
