# ground.py   사용법: python ground.py data/flat 5
import sys, os, json
import numpy as np
import cv2
import open3d as o3d

folder = sys.argv[1]
idx = int(sys.argv[2]) if len(sys.argv) > 2 else 0
OBST_H = 0.05  # 바닥 위 5cm 이상이면 장애물 (m)
FLOOR_H = 0.03  # 바닥 위 3cm 이내면 바닥
MAX_DIST = 3.0  # 이 거리 안쪽의 장애물만 본다 (m)

meta = json.load(open(os.path.join(folder, "meta.json")))
scale = meta.get("depth_scale_m", 0.00025)
di = meta["depth_intrinsics"]
if isinstance(di, dict):
    fx, fy = di["fx"], di["fy"]
    cx, cy = di.get("ppx", di.get("cx")), di.get("ppy", di.get("cy"))
else:
    fx, fy, cx, cy = di[:4]

depth = cv2.imread(
    os.path.join(folder, "depth", f"{idx:06d}.png"), cv2.IMREAD_UNCHANGED
)
h, w = depth.shape
z = depth.astype(np.float32) * scale
valid = z > 0
u, v = np.meshgrid(np.arange(w), np.arange(h))
P = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=-1)  # (h, w, 3), 미터

# 1) 바닥 평면: 화면 아래쪽 절반의 점들로 RANSAC
cand = valid & (v > h * 0.5)
pc = o3d.geometry.PointCloud()
pc.points = o3d.utility.Vector3dVector(P[cand])
pc = pc.voxel_down_sample(0.02)
if len(pc.points) < 200:
    print(
        f"status: NO_GROUND (아래쪽 절반의 유효 점 {len(pc.points)}개, 유효 픽셀 {100 * valid.mean():.1f}%)"
    )
    sys.exit(0)
plane, _ = pc.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
n = np.array(plane[:3])
d = plane[3]
nn = np.linalg.norm(n)
n, d = n / nn, d / nn
if d < 0:  # 카메라(원점)가 바닥 위쪽(+)에 오도록 방향 맞춤
    n, d = -n, -d
tilt = np.degrees(np.arccos(abs(n[1])))
if tilt > 40 or not (0.2 < d < 1.5):
    print(
        f"status: NO_GROUND (바닥 같지 않은 평면: 높이 {d * 100:.0f}cm, 기울기 {tilt:.0f}도)"
    )
    sys.exit(0)

# 2) 모든 점의 바닥 기준 높이
H = P @ n + d
dist = np.sqrt(P[..., 0] ** 2 + P[..., 2] ** 2)  # 카메라에서 수평 거리
floor = valid & (np.abs(H) < FLOOR_H)
obst = valid & (H > OBST_H) & (dist < MAX_DIST)
obst = cv2.morphologyEx(
    obst.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)
).astype(bool)  # 점 노이즈 제거

# 3) 결과 출력
print(
    f"카메라 높이(바닥까지): {d * 100:.1f} cm,  카메라 기울기(약): {np.degrees(np.arccos(abs(n[1]))):.1f}도"
)
print(
    f"유효 픽셀 중 바닥 {100 * floor.sum() / valid.sum():.1f}%  장애물 {100 * obst.sum() / valid.sum():.1f}%"
)
for name, cols in (
    ("왼쪽", slice(0, w // 3)),
    ("가운데", slice(w // 3, 2 * w // 3)),
    ("오른쪽", slice(2 * w // 3, w)),
):
    m = obst[:, cols]
    print(
        f"{name}: "
        + (
            f"가장 가까운 장애물 {dist[:, cols][m].min():.2f} m"
            if m.any()
            else "장애물 없음"
        )
    )
    # (선택) 장애물 면의 기울기: 바닥 평면과 이루는 각도
if obst.sum() > 300:
    op = o3d.geometry.PointCloud()
    op.points = o3d.utility.Vector3dVector(P[obst])
    op = op.voxel_down_sample(0.02)
    if len(op.points) >= 30:
        oplane, inl = op.segment_plane(
            distance_threshold=0.015, ransac_n=3, num_iterations=500
        )
        on = np.array(oplane[:3])
        on /= np.linalg.norm(on)
        ang = np.degrees(np.arccos(np.clip(abs(on @ n), 0, 1)))
        frac = len(inl) / len(op.points)
        kind = (
            "완만한 경사"
            if ang < 25
            else ("가파른 경사" if ang < 60 else "수직에 가까운 면(벽/상자)")
        )
        print(
            f"장애물 주 평면: 바닥과 {ang:.0f}도, 장애물 점의 {100 * frac:.0f}%가 이 평면 → {kind}"
        )

# 4) 2D 결과 이미지 저장 (초록=바닥, 빨강=장애물)
gray = (np.clip(z / 5.0, 0, 1) * 255).astype(np.uint8)
img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
img[floor] = (0.5 * img[floor] + 0.5 * np.array([0, 200, 0])).astype(np.uint8)
img[obst] = (0, 0, 255)
out = f"out_{os.path.basename(folder.rstrip('/'))}_{idx}.png"
cv2.imwrite(out, img)
print("저장:", out)

# 5) 3D 보기 (회색=바닥, 빨강=장애물, 파랑=그 외)
col = np.tile(np.array([0.2, 0.5, 1.0]), (h, w, 1))
col[floor] = [0.6, 0.6, 0.6]
col[obst] = [1.0, 0.0, 0.0]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(P[valid] * np.array([1, -1, -1]))
pcd.colors = o3d.utility.Vector3dVector(col[valid])
if os.environ.get("NO3D") != "1":
    o3d.visualization.draw_geometries(
        [pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(0.3)]
    )
