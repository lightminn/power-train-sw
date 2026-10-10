"""Image-only orange-object detection and RGB-D registration.

No USD object pose, semantic annotator or renderer segmentation IDs enter
this module. Camera transforms/intrinsics are the robot's calibration model.
This initial detector is color based,not a general unknown-object recognizer.
"""
import numpy as np
import cv2

def detect_orange(rgb):
    hsv=cv2.cvtColor(np.asarray(rgb,dtype=np.uint8),cv2.COLOR_RGB2HSV)
    mask=cv2.inRange(hsv,np.array([0,100,55],np.uint8),np.array([28,255,255],np.uint8))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    n,labels,stats,centroids=cv2.connectedComponentsWithStats(mask)
    if n<=1:return None
    idx=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA]))
    if stats[idx,cv2.CC_STAT_AREA]<45:return None
    return dict(mask=(labels==idx),pixels=int(stats[idx,cv2.CC_STAT_AREA]),uv=centroids[idx].astype(float),bbox=stats[idx,:4].tolist())

def estimate_rgbd(color,depth):
    """Register measured depth points into RGB,fit visible object surface.

    Cube-like object center behind observed front face is inferred using half
    its observed width. This shape assumption and confidence are reported.
    """
    detection=detect_orange(color['rgb'])
    if detection is None:return None
    points=np.asarray(depth['points_world_m'])[::2]
    if len(points)<30:return None
    M=np.asarray(color['optical_to_world']);local=(points-M[:3,3])@M[:3,:3]
    ok=local[:,2]>.05;points=points[ok];local=local[ok]
    K=np.asarray(color['K']);uv=local[:,:2]/local[:,2:3]
    uv=uv*np.array([K[0,0],K[1,1]])+np.array([K[0,2],K[1,2]])
    uv=np.rint(uv).astype(int);h,w=detection['mask'].shape
    ok=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    points=points[ok];local=local[ok];uv=uv[ok]
    # Interior mask prevents RGB/depth disocclusion at silhouette edges.
    mask=cv2.erode(detection['mask'].astype(np.uint8),np.ones((3,3),np.uint8))
    ok=mask[uv[:,1],uv[:,0]]!=0;points=points[ok];local=local[ok]
    if len(points)<30:return None
    z=np.median(local[:,2]);ok=np.abs(local[:,2]-z)<.05;points=points[ok]
    if len(points)<30:return None
    lo,hi=np.percentile(points,[2,98],axis=0);surface=np.median(points,axis=0)
    surface[2]=(lo[2]+hi[2])/2
    width=float(np.clip(hi[1]-lo[1],.018,.065))
    # Small upright cuboid: use width for unseen depth,not generator size.
    center=surface+M[:3,2]*(width/2)
    return dict(center_world_m=center,width_m=width,height_m=float(hi[2]-lo[2]),surface_world_m=surface,
        pixels=detection['pixels'],depth_points=len(points),rgb_uv=detection['uv'],bbox=detection['bbox'],
        confidence='visible measured surface + cube-like unseen-depth assumption',
        source='RGB color segmentation + registered measured D435i depth only')

def wrist_correction(ov,depth_estimate):
    """OV5640 bearing plus RGB-D distance; no depth assigned to OV5640.

    Intersect OV image bearing with a plane at the estimated object's depth.
    Returns a world point and actual observed image coordinates.
    """
    det=detect_orange(ov['rgb'])
    if det is None:return None
    K=np.asarray(ov['K']);u,v=det['uv'];ray=np.array([(u-K[0,2])/K[0,0],(v-K[1,2])/K[1,1],1.])
    M=np.asarray(ov['optical_to_world']);old=np.asarray(depth_estimate['center_world_m'])
    local=(old-M[:3,3])@M[:3,:3]
    if local[2]<=.02:return None
    measured=M[:3,:3]@(ray*local[2])+M[:3,3]
    # Correct lateral bearing without inventing a monocular depth reading.
    return dict(center_world_m=measured,pixels=det['pixels'],uv=det['uv'],
        source='OV5640 RGB bearing,depth supplied by D435i observation',
        correction_m=float(np.linalg.norm(measured-old)))
