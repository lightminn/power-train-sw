import math
import numpy as np

ARM_NAMES=['arm_J1_yaw','arm_J2_shoulder','arm_J3_elbow','arm4_rotation','axis5_rotation']
TCP_ZERO=np.array([.39176,.00028311367,.32006856815])

def forward(plan,q,point=TCP_ZERO):
    js={j['name']:j for j in plan['tree_joints']};ls={l['link']:l for l in plan['links']}
    T=np.eye(4);parent=np.array(ls['base_link']['origin_world_m']);i=0
    for name in ['arm_J1_yaw','arm_J2_shoulder','arm_J3_elbow','arm4_rotation','arm_tool_fixed','axis5_rotation']:
        j=js[name];origin=np.array(j['origin_world_m']);tr=np.eye(4);tr[:3,3]=origin-parent;T=T@tr;parent=origin
        if name!='arm_tool_fixed':
            a=np.array(j['axis_world'],float);a/=np.linalg.norm(a);v=math.radians(q[i]);i+=1
            K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
            rot=np.eye(4);rot[:3,:3]=np.eye(3)+math.sin(v)*K+(1-math.cos(v))*K@K;T=T@rot
    return (T@np.r_[np.array(point)-parent,1])[:3],T[:3,:3]
