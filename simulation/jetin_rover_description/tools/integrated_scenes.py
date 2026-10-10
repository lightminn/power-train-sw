"""Portable scenes sharing exactly one final robot asset."""
from pathlib import Path
import json,math
import numpy as np
from pxr import Usd,Sdf,UsdGeom,UsdPhysics,UsdShade,Gf
P=Path(__file__).resolve().parents[1]

def cube(stage,path,pos,size,color,yaw=0):
    c=UsdGeom.Cube.Define(stage,path);c.CreateSizeAttr(1.)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));c.AddRotateZOp().Set(float(yaw));c.AddScaleOp().Set(Gf.Vec3d(*map(float,size)))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)]);UsdPhysics.CollisionAPI.Apply(c.GetPrim());return c

def camera(stage,path,eye,target,up=(0,0,1)):
    c=UsdGeom.Camera.Define(stage,path);c.CreateFocalLengthAttr(30.);c.CreateClippingRangeAttr(Gf.Vec2f(.01,100))
    x=UsdGeom.Xformable(c);x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(*up)).GetInverse())

def new_scene(name):
    folder=P/'scenes';folder.mkdir(exist_ok=True);path=folder/name
    if path.exists():stage=Usd.Stage.Open(str(path));stage.GetRootLayer().Clear()
    else:stage=Usd.Stage.CreateNew(str(path))
    stage.GetRootLayer().subLayerPaths=['../JETIN_Rover_Start_20261009.usda'];stage.SetDefaultPrim(stage.GetPrimAtPath('/JETIN'))
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
    return stage,path

def make_vision(seed=20261010):
    stage,path=new_scene('JETIN_VisionPick_20261010.usda');rng=np.random.default_rng(seed)
    size=np.array([rng.uniform(.035,.042),rng.uniform(.029,.034),rng.uniform(.075,.09)])
    center=np.array([rng.uniform(.71,.745),rng.uniform(.015,.045),rng.uniform(.643,.661)])
    yaw=float(rng.uniform(-10,10));top=center[2]-size[2]/2
    cube(stage,'/Task/Pedestal',[center[0],center[1],(top-.025)/2],[.06,.06,top-.025],[.23,.28,.33])
    cube(stage,'/Task/Table',[center[0],center[1],top-.0125],[.12,.12,.025],[.45,.49,.54])
    obj=cube(stage,'/Task/Object',center,size,[1,.22,.02],yaw)
    UsdPhysics.RigidBodyAPI.Apply(obj.GetPrim()).CreateKinematicEnabledAttr(False)
    UsdPhysics.MassAPI.Apply(obj.GetPrim()).CreateMassAttr(.08)
    mat=UsdShade.Material.Define(stage,'/Task/ObjectContact');m=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    m.CreateStaticFrictionAttr(.6);m.CreateDynamicFrictionAttr(.5);m.CreateRestitutionAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat,materialPurpose='physics')
    obj.GetPrim().SetCustomDataByKey('jetin:generatedTestObject',True)
    obj.GetPrim().SetCustomDataByKey('jetin:seed',int(seed))
    obj.GetPrim().SetCustomDataByKey('jetin:materialAssumption','80g generic plastic test object,friction .6/.5')
    camera(stage,'/Task/Wide',(1.65,1.65,1.35),(.45,.02,.65))
    camera(stage,'/Task/Detail',(1.05,.63,.94),tuple(center))
    stage.GetRootLayer().Save()
    return path,dict(seed=seed,size_m=size.tolist(),center_m=center.tolist(),yaw_deg=yaw,mass_kg=.08)

def make_wave(height=.12,wavelength=.9):
    if not 0<height<1 or not wavelength>0:raise ValueError('Positive terrain height and wavelength required; height must be below1m')
    stage,path=new_scene('JETIN_WaveTerrain_20261010.usda')
    start,end=.6,2.4
    for side,y0,y1,phase,color in [('Left',.16,.58,0,(.22,.5,.7)),('Right',-.58,-.16,math.pi/2,(.82,.48,.22))]:
        xs=np.linspace(start,end,241);points=[];indices=[]
        for x in xs:
            e=math.sin(math.pi/2*max(0.,min(1.,(x-start)/.35,(end-x)/.35)))**2
            z=height*e*math.sin(math.pi*(x-start)/wavelength+phase)**2
            points.extend([Gf.Vec3f(x,y0,z),Gf.Vec3f(x,y1,z)])
        for i in range(len(xs)-1):a=2*i;indices.extend([a,a+2,a+3,a,a+3,a+1])
        mesh=UsdGeom.Mesh.Define(stage,'/Task/'+side+'Track');mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([3]*(len(indices)//3));mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True);mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath('/Environment/GroundMaterial')),materialPurpose='physics')
    camera(stage,'/Task/Side',(.5,2.8,1.25),(.5,0,.5));camera(stage,'/Task/Top',(.5,0,3.8),(.5,0,.25),(1,0,0))
    stage.GetRootLayer().Save();return path,dict(start_x_m=start,end_x_m=end,height_m=height,wavelength_m=wavelength,left_right_phase_rad=math.pi/2)

def make_obstacles(seed=515):
    stage,path=new_scene('JETIN_FrontObstacles_20261010.usda');rng=np.random.default_rng(seed);objects=[]
    for i,(x,y) in enumerate([(1.1,-.32),(1.5,.12),(2.,.48),(2.5,-.4)]):
        size=np.array([rng.uniform(.1,.25),rng.uniform(.1,.22),rng.uniform(.15,.4)])
        pos=np.array([x,y,size[2]/2]);cube(stage,'/Task/Obstacle'+str(i),pos,size,[.7,.25+.1*i,.15])
        objects.append(dict(position_m=pos.tolist(),size_m=size.tolist()))
    stage.GetRootLayer().Save();return path,dict(seed=seed,objects=objects)

if __name__=='__main__':
    outputs={}
    for mode,f in [('vision',make_vision),('terrain',make_wave),('obstacles',make_obstacles)]:
        path,record=f();outputs[mode]=dict(scene=str(path.relative_to(P)),generation=record)
    (P/'config/integrated_scenes.json').write_text(json.dumps(outputs,indent=2),encoding='utf8')
    print(json.dumps(outputs))
