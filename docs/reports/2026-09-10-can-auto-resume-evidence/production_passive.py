"""Read-only production CAN observation; no transmitted frames."""
import can,collections,json,time
bus=can.Bus(interface='socketcan',channel='can0'); start=time.monotonic(); last={}; gaps=collections.defaultdict(float); counts=collections.Counter(); states={}; raw={}; errors=[]
try:
    while time.monotonic()-start<30:
        m=bus.recv(.05); now=time.monotonic()
        if m is None:continue
        if m.is_error_frame:errors.append(str(m));continue
        if m.is_remote_frame:continue
        node=None
        if not m.is_extended_id and m.arbitration_id&31==1 and m.arbitration_id>>5 in range(11,17):
            node=str(m.arbitration_id>>5); value=(int.from_bytes(m.data[:4],'little'),m.data[4]); states[node]=value;raw[node]=bytes(m.data).hex()
            if value!=(0,1):errors.append({'node':node,'state':value})
        elif m.is_extended_id and m.arbitration_id>>8==41 and m.arbitration_id&255 in range(1,5):node='ak'+str(m.arbitration_id&255)
        if node is not None:
            counts[node]+=1;gaps[node]=max(gaps[node],now-last.get(node,start));last[node]=now
    end=time.monotonic()
    for node,t in last.items():gaps[node]=max(gaps[node],end-t)
    result={'seconds':end-start,'counts':dict(counts),'maximum_gap_s':dict(gaps),'drive_states':states,'raw_heartbeat':raw,'errors':errors,'transmitted_frames':0}
    assert len(counts)==10 and min(counts.values())>=1200,result
    assert not errors and max(gaps.values())<.2,result
    assert all(bytes.fromhex(raw[str(n)])[5]==0xA3 for n in range(13,17)),result
    result['result']='PASS'
finally:
    bus.shutdown()
    print(json.dumps(result,indent=2))
