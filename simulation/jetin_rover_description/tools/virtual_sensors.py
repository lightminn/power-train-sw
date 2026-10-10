"""Isaac Sim 5.1 camera rig at the existing CAD optical frames.

Nominal intrinsics are separate from physical sensor calibration. L515
range/noise uses an explicit public-specification approximation. MEMS scan
physics and D435i stereo-processing errors are not modeled.
"""
from pathlib import Path
import json
import numpy as np
from manufacturer_models import L515DepthModel

PACKAGE = Path(__file__).resolve().parents[1]

class VirtualSensorRig:
    def __init__(self, names=None):
        import omni.usd
        from isaacsim.sensors.camera import Camera
        from pxr import UsdGeom
        self.stage=omni.usd.get_context().get_stage()
        self.config=json.loads((PACKAGE/'config/virtual_sensors.json').read_text(encoding='utf-8'))
        self.cameras={};self.specs={}
        self.l515_depth_model=L515DepthModel()
        for row in self.config['streams']:
            if names is None and not row['enabled']:continue
            if names is not None and row['name'] not in names:continue
            prim=self.stage.GetPrimAtPath(row['prim_path'])
            if not prim or not prim.IsA(UsdGeom.Camera):raise RuntimeError('Missing sensor Camera prim: '+row['prim_path'])
            before=UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            camera=Camera(prim_path=row['prim_path'],resolution=tuple(row['resolution']),frequency=row['frequency_hz'])
            camera.initialize()
            camera.set_clipping_range(.01,row['range_m'][1])
            camera.set_lens_aperture(0.)
            K=np.asarray(row['K'],float)
            camera.set_opencv_pinhole_properties(fx=float(K[0,0]),fy=float(K[1,1]),cx=float(K[0,2]),cy=float(K[1,2]),pinhole=row['distortion_coefficients'])
            if row['depth_enabled']:camera.add_distance_to_image_plane_to_frame()
            after=UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            assert np.max(np.abs(np.asarray(before)-np.asarray(after)))<1e-7, 'Camera initialization changed the mount transform'
            self.cameras[row['name']]=camera;self.specs[row['name']]=row

    def collect(self):
        from pxr import UsdGeom
        cache=UsdGeom.XformCache();result={}
        for name,camera in self.cameras.items():
            row=self.specs[name];frame=camera.get_current_frame(clone=True)
            rgba=np.asarray(camera.get_rgba())
            w,h=row['resolution']
            if rgba.shape[:2]!=(h,w):continue
            optical=cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath(row['frame_path']))
            data={'rgb':rgba[:,:,:3].copy(),'K':np.asarray(row['K'],float),
                  'optical_to_world':np.asarray(optical).T.copy(),
                  'rendering_time':float(frame.get('rendering_time',0.) or 0.),
                  'hardware_calibrated':False}
            if row['depth_enabled']:
                raw=camera.get_depth()
                if raw is None:continue
                depth=np.asarray(raw,dtype=np.float32).reshape(h,w).copy()
                # Image-plane depth annotator uses the underlying USD pinhole
                # projection. Its intrinsics differ from a distorted RGB lens.
                fl=camera.get_focal_length()
                K=np.array([[w*fl/camera.get_horizontal_aperture(),0.,w/2],
                            [0.,h*fl/camera.get_vertical_aperture(),h/2],
                            [0.,0.,1.]],float)
                data['depth_K']=K.copy()
                yy,xx=np.indices((h,w))
                rays=np.stack([(xx-K[0,2])/K[0,0],(yy-K[1,2])/K[1,1],np.ones_like(xx)],axis=-1)
                ranges=depth*np.linalg.norm(rays,axis=-1)
                valid=np.isfinite(depth)&(depth>0)&(ranges>=row['range_m'][0])&(ranges<=row['range_m'][1])
                depth[~valid]=0
                depth_model='ideal_geometry_depth'
                if row['name']=='l515_depth':
                    data['simulation_diagnostic_ideal_depth_m']=depth.copy()
                    depth,valid=self.l515_depth_model.update(depth,rays)
                    depth_model='L515_XGA_range_and_VGA_noise_reference_approximation'
                points=(rays*depth[:,:,None])[valid].astype(np.float32)
                M=data['optical_to_world']
                data.update(depth_m=depth,valid_depth_mask=valid,points_optical_m=points,
                            points_world_m=(points@M[:3,:3].T+M[:3,3]).astype(np.float32),depth_model=depth_model)
            result[name]=data
        return result

    def save(self, directory, prefix='snapshot'):
        from PIL import Image
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        samples=self.collect();metadata={}
        for name,data in samples.items():
            Image.fromarray(data['rgb']).save(directory/(prefix+'_'+name+'.png'))
            arrays={k:v for k,v in data.items() if isinstance(v,np.ndarray) and k!='rgb'}
            np.savez_compressed(directory/(prefix+'_'+name+'.npz'),**arrays)
            metadata[name]={'hardware_calibrated':False,'resolution':self.specs[name]['resolution'],
                            'frame_path':self.specs[name]['frame_path'],'rendering_time':data['rendering_time'],
                            'point_count':len(data.get('points_optical_m',[])),'physical_fidelity':self.specs[name]['physical_fidelity']}
        (directory/(prefix+'_metadata.json')).write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
        return metadata

    def collect_motion(self):
        """Read the latest IMU/wheel/steering packet installed by the sensor installer."""
        import builtins
        state=getattr(builtins,'JETIN_MOTION_FEEDBACK',None)
        if state is None:raise RuntimeError('Run install_motion_feedback.py or install_virtual_sensors.py first')
        if state['error'] is not None:raise RuntimeError(state['error'])
        return state['samples']

    def close(self):
        for camera in self.cameras.values():camera.pause()
