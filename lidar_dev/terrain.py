# terrain.py
import json, os, sys, glob, time
import numpy as np
import cv2
import open3d as o3d


class TerrainDetector:
    def __init__(
        self,
        fx,
        fy,
        cx,
        cy,
        depth_scale=0.00025,
        shape=(480, 640),
        obst_h=0.05,
        floor_h=0.03,
        max_dist=3.0,
        half_width=0.52,
        hold_frames=15,
        max_h=2.0,
        front_offset=0.0,
    ):
        self.scale = depth_scale
        self.obst_h, self.floor_h, self.max_dist = obst_h, floor_h, max_dist
        self.half_width, self.hold_max = half_width, hold_frames
        self.max_h, self.front_offset = max_h, front_offset
        self.h, self.w = shape
        u, v = np.meshgrid(np.arange(self.w), np.arange(self.h))
        self.v = v
        self.xn, self.yn = (u - cx) / fx, (v - cy) / fy  # 정규화 좌표 (미리 계산)
        self.n = self.d = None  # 바닥 평면 (법선, 카메라 높이)
        self.hold = 0
        self.floor_mask = self.obst_mask = None

    def _fit_ground(self, P, valid):
        cand = valid & (self.v > self.h * 0.5)  # 화면 아래쪽 절반만 사용
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(P[cand])
        pc = pc.voxel_down_sample(0.02)
        if len(pc.points) < 200:
            return False
        plane, _ = pc.segment_plane(0.02, 3, 1000)
        n, d = np.array(plane[:3]), plane[3]
        nn = np.linalg.norm(n)
        n, d = n / nn, d / nn
        if d < 0:
            n, d = -n, -d
        tilt = np.degrees(np.arccos(min(1.0, abs(n[1]))))
        if tilt > 40 or not (0.25 < d < 0.80):  # 바닥 같지 않은 평면(벽 등)
            return False
        if self.n is None:
            self.n, self.d = n, d
        else:  # 이전 결과와 섞어 안정화
            nb = 0.7 * self.n + 0.3 * n
            self.n, self.d = nb / np.linalg.norm(nb), 0.7 * self.d + 0.3 * d
        return True

    def process(self, depth, mode="DRIVE"):
        out = {
            "timestamp": round(time.time(), 3),
            "status": "OK",
            "ground_plane_ok": False,
            "nearest_obstacle_m": None,
            "zones": {"left": None, "center": None, "right": None},
            "camera_height_m": None,
            "camera_tilt_deg": None,
        }
        if mode != "DRIVE":  # 팔 작업 중(정지): 판단 보류
            out["status"] = "HOLD"
            return out
        if depth is None or depth.shape != (self.h, self.w):
            out["status"] = "SENSOR_ERROR"
            return out
        z = depth.astype(np.float32) * self.scale
        valid = z > 0
        if valid.mean() < 0.05:  # 거의 다 비어 있음
            out["status"] = "SENSOR_ERROR"
            return out
        P = np.stack([self.xn * z, self.yn * z, z], axis=-1)

        fresh = self._fit_ground(P, valid)
        self.hold = 0 if fresh else self.hold + 1
        if self.n is not None and self.hold > self.hold_max:
            self.n = self.d = None  # 너무 오래 못 찾으면 버림
        if self.n is None:
            out["status"] = "NO_GROUND"
            return out
        out["ground_plane_ok"] = fresh  # False면 직전 평면을 임시 사용 중
        out["camera_height_m"] = round(float(self.d), 3)
        out["camera_tilt_deg"] = round(
            float(np.degrees(np.arccos(min(1.0, abs(self.n[1]))))), 1
        )

        H = P @ self.n + self.d  # 바닥 기준 높이(m)
        zc, xc = np.array([0, 0, 1.0]), np.array([1.0, 0, 0])
        f = zc - (zc @ self.n) * self.n
        f /= np.linalg.norm(f)  # 바닥 위 '앞' 방향
        r = xc - (xc @ self.n) * self.n
        r /= np.linalg.norm(r)  # 바닥 위 '오른쪽' 방향
        fwd, lat = P @ f, P @ r  # 앞쪽 거리, 좌우 위치(m)

        floor = valid & (np.abs(H) < self.floor_h)
        obst = (
            valid
            & (H > self.obst_h)
            & (H < self.max_h)
            & (fwd > 0)
            & (fwd < self.max_dist)
        )
        obst = cv2.morphologyEx(
            obst.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)
        ).astype(bool)
        self.floor_mask, self.obst_mask = floor, obst

        hw = self.half_width
        bands = {"left": lat < -hw, "center": np.abs(lat) <= hw, "right": lat > hw}
        for name, m in bands.items():
            mm = obst & m
            if mm.any():
                out["zones"][name] = round(
                    max(0.0, float(np.percentile(fwd[mm], 2)) - self.front_offset), 2
                )
        vals = [v for v in out["zones"].values() if v is not None]
        out["nearest_obstacle_m"] = min(vals) if vals else None
        return out


def load_detector(folder):
    meta = json.load(open(os.path.join(folder, "meta.json")))
    di = meta["depth_intrinsics"]
    if isinstance(di, dict):
        fx, fy = di["fx"], di["fy"]
        cx, cy = di.get("ppx", di.get("cx")), di.get("ppy", di.get("cy"))
    else:
        fx, fy, cx, cy = di[:4]
    return TerrainDetector(
        fx,
        fy,
        cx,
        cy,
        meta.get("depth_scale_m", 0.00025),
        half_width=0.52,
        max_h=2.0,
        front_offset=0.0,
    )


if __name__ == "__main__":  # 사용법: python terrain.py data/obj_50cm
    folder = sys.argv[1]
    det = load_detector(folder)
    for p in sorted(glob.glob(os.path.join(folder, "depth", "*.png"))):
        res = det.process(cv2.imread(p, cv2.IMREAD_UNCHANGED))
        print(os.path.basename(p), json.dumps(res, ensure_ascii=False))
