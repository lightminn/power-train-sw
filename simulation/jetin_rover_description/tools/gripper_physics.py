"""Analytical rack/pinion transmission; preserve all external and silicone-pad contacts."""
import math,json
from pathlib import Path

SETTINGS={'rack_pitch_radius_m':.017999994345347013,'mimic_natural_frequency_hz':0.,'mimic_damping_ratio':0.,'motor_max_torque_Nm':.3,'motor_stiffness_Nm_deg':2.,'motor_damping_Nm_s_deg':.05,'pad_static_friction':1.,'pad_dynamic_friction':.8,'pad_contact_stiffness_N_m':20000.,'pad_contact_damping_N_s_m':50.,'hardware_calibrated':False}
SETTINGS_PATH=Path(__file__).resolve().parents[1]/'config/gripper_physics.json'
if SETTINGS_PATH.is_file():SETTINGS.update(json.loads(SETTINGS_PATH.read_text(encoding='utf-8')))

def apply(stage):
    from pxr import UsdPhysics,PhysxSchema,UsdShade,Sdf
    for pinion in ['pinion_A','pinion_B']:
        body=stage.GetPrimAtPath('/JETIN/'+pinion)
        UsdPhysics.FilteredPairsAPI.Apply(body).CreateFilteredPairsRel().SetTargets([])
        mesh=stage.GetPrimAtPath('/JETIN/'+pinion+'/Colliders/b_00')
        rel=UsdPhysics.FilteredPairsAPI.Apply(mesh).CreateFilteredPairsRel()
        # Only the rack/gear teeth are represented by mimic constraints. Silicone pads remain collidable.
        rel.SetTargets([Sdf.Path('/JETIN/'+finger+'/Colliders/b_00') for finger in ['finger_lower','finger_upper']])
    for name,multiplier in [('pinion_B_rotation',1.),('finger_lower_slide',-SETTINGS['rack_pitch_radius_m']),('finger_upper_slide',SETTINGS['rack_pitch_radius_m'])]:
        p=stage.GetPrimAtPath('/JETIN/Joints/'+name)
        api=PhysxSchema.PhysxMimicJointAPI.Apply(p,'rotX')
        api.CreateReferenceJointRel().SetTargets([Sdf.Path('/JETIN/Joints/pinion_A_rotation')])
        api.CreateReferenceJointAxisAttr('rotX')
        api.CreateGearingAttr(-multiplier if name=='pinion_B_rotation' else -multiplier*math.pi/180.)
        api.CreateOffsetAttr(0.)
        p.CreateAttribute('physxMimicJoint:rotX:naturalFrequency',Sdf.ValueTypeNames.Float).Set(SETTINGS['mimic_natural_frequency_hz'])
        p.CreateAttribute('physxMimicJoint:rotX:dampingRatio',Sdf.ValueTypeNames.Float).Set(SETTINGS['mimic_damping_ratio'])
    for name in ['pinion_A_rotation','pinion_B_rotation']:
        drive=UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath('/JETIN/Joints/'+name),'angular')
        drive.CreateTypeAttr('force');drive.CreateMaxForceAttr(SETTINGS['motor_max_torque_Nm'])
        drive.CreateStiffnessAttr(SETTINGS['motor_stiffness_Nm_deg']);drive.CreateDampingAttr(SETTINGS['motor_damping_Nm_s_deg'])
        drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
    material=UsdShade.Material.Define(stage,'/JETIN/PhysicsMaterials/GripperSilicone')
    m=UsdPhysics.MaterialAPI.Apply(material.GetPrim());m.CreateStaticFrictionAttr(SETTINGS['pad_static_friction']);m.CreateDynamicFrictionAttr(SETTINGS['pad_dynamic_friction']);m.CreateRestitutionAttr(0.)
    mat=PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    mat.CreateCompliantContactStiffnessAttr(SETTINGS['pad_contact_stiffness_N_m']);mat.CreateCompliantContactDampingAttr(SETTINGS['pad_contact_damping_N_s_m']);mat.CreateCompliantContactAccelerationSpringAttr(False)
    for finger in ['finger_lower','finger_upper']:
        pad=stage.GetPrimAtPath('/JETIN/'+finger+'/Colliders/b_01')
        UsdShade.MaterialBindingAPI.Apply(pad).Bind(material,materialPurpose='physics')
    return SETTINGS.copy()
