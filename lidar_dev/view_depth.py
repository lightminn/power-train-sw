# view_depth.py   사용법: python view_depth.py ~/Downloads/L515/20260930_212517 5
import sys, os, json
import numpy as np
import cv2
import open3d as o3d

folder = os.path.expanduser(sys.argv[1])
idx = int(sys.argv[2]) if len(sys.argv) > 2 else 0

meta = json.load(open(os.path.join(folder, "meta.json")))
scale = meta.get("depth_scale_m", 0.00025)
di = meta["depth_intrinsics"]
if isinstance(di, dict):
    fx, fy = di["fx"], di["fy"]
    cx, cy = di.get("ppx", di.get("cx")), di.get("ppy", di.get("cy"))
else:
    fx, fy, cx, cy = di[:4]
print(f"scale={scale}  fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

path = os.path.join(folder, "depth", f"{idx:06d}.png")
depth = cv2.imread(path, cv2.IMREAD_UNCHANGED)  # 16비트 그대로 읽기
assert depth is not None and depth.dtype == np.uint16, (
    f"16비트 depth PNG가 아니에요: {path}"
)
h, w = depth.shape
print("depth 크기:", w, "x", h)

z = depth.astype(np.float32) * scale  # 미터, 0이면 측정 없음
valid = z > 0
u, v = np.meshgrid(np.arange(w), np.arange(h))
zv = z[valid]
x = (u[valid] - cx) * zv / fx
y = (v[valid] - cy) * zv / fy
pts = np.stack([x, y, zv], axis=1)

print(
    f"유효 픽셀 {100 * valid.mean():.1f}%  거리 {zv.min():.2f} ~ {zv.max():.2f} m  중앙값 {np.median(zv):.2f} m"
)

# 거리별 색 (가까우면 파랑, 멀면 빨강)
t = ((zv - zv.min()) / (np.ptp(zv) + 1e-6) * 255).astype(np.uint8)
cols = (
    cv2.applyColorMap(t.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)[:, ::-1]
    / 255.0
)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts * np.array([1, -1, -1]))  # 위가 위로 보이게
pcd.colors = o3d.utility.Vector3dVector(cols)
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)  # 카메라 위치 표시
o3d.visualization.draw_geometries([pcd, axes])
