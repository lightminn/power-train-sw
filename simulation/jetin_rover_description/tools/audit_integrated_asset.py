"""Portable structural audit,not a claim of measured hardware fidelity."""
from pathlib import Path
import json,math,hashlib,collections,xml.etree.ElementTree as ET
import numpy as np
from pxr import Usd,UsdPhysics,UsdUtils
P=Path(__file__).resolve().parents[1]
def audit():
    s=Usd.Stage.Open(str(P/'JETIN_Rover_Final_20261009.usdc'));cfg=json.loads((P/'config/manufacturer_models.json').read_text(encoding='utf8'))
    embedded=json.loads(s.GetPrimAtPath('/JETIN/ManufacturerSpecificationRegistry').GetAttribute('jetin:configurationJson').Get());assert embedded==cfg
    bodies=[p for p in s.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)];joints=[p for p in s.Traverse() if p.IsA(UsdPhysics.Joint)];colliders=[p for p in s.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    for p in bodies:
        m=UsdPhysics.MassAPI(p);assert m.GetMassAttr().Get()>0,str(p.GetPath());assert min(m.GetDiagonalInertiaAttr().Get())>0,str(p.GetPath())
    assert s.GetPrimAtPath('/JETIN').GetAttribute('physxArticulation:enabledSelfCollisions').Get() is True
    filters={str(p.GetPath()):[str(t) for t in p.GetRelationship('physics:filteredPairs').GetTargets()] for p in s.Traverse() if p.GetRelationship('physics:filteredPairs').GetTargets()}
    approximation=collections.Counter(p.GetAttribute('physics:approximation').Get() for p in colliders)
    assert len(bodies)==47 and len(joints)==48 and len(colliders)==125
    drives={}
    for n in cfg['confirmed_servo_joints']:
        p=s.GetPrimAtPath('/JETIN/Joints/'+n);d=UsdPhysics.DriveAPI.Get(p,'angular');assert d and d.GetMaxForceAttr().Get()>0
        drives[n]=dict(torque_cap_Nm=d.GetMaxForceAttr().Get(),speed_cap_deg_s=p.GetAttribute('physxJoint:maxJointVelocity').Get())
    for i in range(1,9):assert s.GetPrimAtPath('/JETIN/Joints/lock_ball_'+str(i)).GetAttribute('physxMimicJoint:rotX:naturalFrequency').Get()==0
    deps={}
    for path in [P/'JETIN_Rover_Start_20261009.usda',*(P/'scenes').glob('*.usda')]:
        layers,assets,missing=UsdUtils.ComputeAllDependencies(str(path));assert not missing,(path,missing)
        assert not assets,(path,assets);deps[str(path.relative_to(P))]=[str(Path(l.realPath).relative_to(P)) for l in layers]
    urdf=P/'jetin_rover_isaac.urdf';tree=ET.parse(urdf);mesh=[]
    for m in tree.findall('.//mesh'):
        n=m.get('filename');assert n.startswith(('package://jetin_rover_description/','meshes/')),n
        f=P/n.removeprefix('package://jetin_rover_description/');assert f.is_file(),n;mesh.append(str(f.relative_to(P)))
    return dict(passed_static=True,hardware_equivalence_verified=False,revision=cfg['revision'],asset_sha256=hashlib.sha256((P/'JETIN_Rover_Final_20261009.usdc').read_bytes()).hexdigest(),
        body_count=len(bodies),joint_count=len(joints),collider_count=len(colliders),total_mass_kg=sum(UsdPhysics.MassAPI(p).GetMassAttr().Get() for p in bodies),all_positive_mass_and_inertia=True,self_collision_enabled=True,collision_approximations=dict(approximation),
        collision_filters=filters,eight_drives=drives,dependencies=deps,URDF='jetin_rover_isaac.urdf',URDF_meshes=sorted(set(mesh)),
        collision_scope='Detailed CAD SDF approximation256,1mm contact offsets; shaft fits,analytical gear/cam pairs filtered. Swept CAD clearance of every possible joint combination not certified.',
        real_world_gaps=cfg['fidelity_limits'])
if __name__=='__main__':
    result=audit();(P/'config/integrated_static_audit_20261010.json').write_text(json.dumps(result,indent=2),encoding='utf8');print(json.dumps({k:v for k,v in result.items() if k not in ['collision_filters','dependencies','URDF_meshes']},indent=2))
