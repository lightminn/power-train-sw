# ground_est.py — numpy 전용 바닥·장애물 검출(live_overlay 7판, 10-22 밤): PR #13 terrain.TerrainDetector와 같은 결과 사전·속성
#   (n, d, fwd, lat, floor_mask, obst_mask, half_width, process(depth) → status·zones·camera_height_m·camera_tilt_deg)을 내되 바닥 찾기를 바꿨다.
#   - open3d 없음: Jetson ROS 컨테이너(Python 3.10·opencv-headless·open3d 없음)에서도 돈다
#   - 바닥 후보: 화면 아래 절반 제한 없이 전체(최대 거리 max_dist 안). 위 방향(IMU 중력 → 장착 캘리브레이션 사전 → 직전 평면 → 카메라 −y 순)으로
#     점 높이 히스토그램(1 cm)을 만들고, 카메라 아래 hmin–hmax 안에서 '가장 낮은 큰 면'(최대 봉우리의 25 % 넘는 가장 낮은 칸)을 바닥으로 고른다
#     → 책상 윗면·벽을 바닥으로 잡는 일이 줄고, 먼 바닥 점도 평면 맞춤에 들어간다
#   - 재맞춤: 거리에 따라 넓어지는 허용폭 band(z) = band0 + band_k·z(L515 잡음이 거리에 따라 커짐) 안 점으로 가중 최소제곱(SVD, 가중 1/band²) 3회
#   - 거름: 법선이 IMU 위와 grav_tol° 넘게 다르면, 장착 캘리브레이션 사전과 높이 prior_dh·각 prior_deg 넘게 다르면, 안쪽 점이 min_inliers 미만이면 버림
#   - 시간: 직전 평면과 섞기(0.7/0.3), hold_frames 넘게 못 찾으면 버림
#   - 바닥 칠: |높이| < band(z), 장애물: 높이 > max(obst_h, 2·band(z)) — 먼 거리 잡음이 장애물로 칠해지지 않게
import numpy as np
import cv2


class GroundDetector:
    def __init__(self, fx, fy, cx, cy, depth_scale=0.00025, shape=(480, 640), obst_h=0.05, half_width=0.4445, max_dist=5.0, max_h=2.0,
                 hmin=0.12, hmax=1.6, grav_tol=12.0, band0=0.02, band_k=0.01, min_inliers=300, hold_frames=15, max_points=24000,
                 prior=None, prior_dh=0.10, prior_deg=8.0, front_offset=0.0):
        self.scale, self.obst_h, self.half_width, self.max_dist, self.max_h = depth_scale, obst_h, half_width, max_dist, max_h
        self.hmin, self.hmax, self.grav_tol, self.band0, self.band_k = hmin, hmax, grav_tol, band0, band_k
        self.min_inliers, self.hold_max, self.max_points = min_inliers, hold_frames, max_points
        self.prior, self.prior_dh, self.prior_deg, self.front_offset = prior, prior_dh, prior_deg, front_offset
        self.h, self.w = shape
        u, v = np.meshgrid(np.arange(self.w), np.arange(self.h))
        self.xn, self.yn = ((u - cx) / fx).astype(np.float32), ((v - cy) / fy).astype(np.float32)
        self.n = self.d = None
        self.up = None                 # IMU 위(깊이 광학 좌표, 단위) — 밖에서 매 프레임 넣음
        self.use_grav = True
        self.hold = 0
        self.reject = ""
        self.stats = {}
        self.floor_mask = self.obst_mask = self.fwd = self.lat = None
        self.z = self.valid = self.P = None
        self.rng = np.random.default_rng(20261022)

    def band(self, z):
        return self.band0 + self.band_k * z

    def _up_guess(self):
        if self.use_grav and self.up is not None:
            return self.up, "imu"
        if self.prior is not None:
            return self.prior[0], "prior"
        if self.n is not None:
            return self.n, "prev"
        return np.array([0.0, -1.0, 0.0]), "camera"

    def _fit(self, P, valid, z):
        idx = np.flatnonzero(valid & (z < self.max_dist))            # 번호만 먼저 뽑고 그 점만 가져옴(13만 점 불리언 추출 대신, 7판 끝 속도)
        if len(idx) > self.max_points:
            idx = idx[self.rng.integers(0, len(idx), self.max_points)]
        pts, zz = P.reshape(-1, 3)[idx], z.reshape(-1)[idx]
        if len(pts) < self.min_inliers:
            self.reject = "few points"
            return None
        u, src = self._up_guess()
        u = u / np.linalg.norm(u)
        s = pts @ u                                                    # 위 방향 성분: 바닥이면 −(카메라 높이)
        lo, hi = -self.hmax, -self.hmin
        if self.prior is not None:                                    # 캘리브레이션 높이 근처만
            lo, hi = max(lo, -self.prior[1] - 2 * self.prior_dh), min(hi, -self.prior[1] + 2 * self.prior_dh)
        nb = max(1, int(round((hi - lo) / 0.01)))
        hist, edges = np.histogram(s, bins=nb, range=(lo, hi))
        if hist.max() < self.min_inliers * 0.3:
            self.reject = "no floor peak"
            return None
        sm = np.convolve(hist, np.ones(3), "same")
        cand = np.nonzero(sm >= 0.25 * sm.max())[0]
        k0 = int(cand[0])                                             # 가장 낮은 큰 면
        win = np.arange(max(0, k0 - 2), min(nb, k0 + 3))
        k0 = int(win[np.argmax(sm[win])])
        s0 = 0.5 * (edges[k0] + edges[k0 + 1])
        inl = np.abs(s - s0) < 1.5 * self.band(zz)
        n, d = u, -s0
        for _ in range(3):                                            # 가중 최소제곱 재맞춤(SVD)
            if inl.sum() < self.min_inliers:
                self.reject = f"inliers {int(inl.sum())}"
                return None
            Q, wq = pts[inl], 1.0 / self.band(zz[inl]) ** 2
            c = (Q * wq[:, None]).sum(0) / wq.sum()
            _, _, Vt = np.linalg.svd((Q - c) * np.sqrt(wq)[:, None], full_matrices=False)
            n = Vt[2]
            if n @ u < 0:
                n = -n
            d = -float(n @ c)
            inl = np.abs(pts @ n + d) < self.band(zz)
        r = pts[inl] @ n + d
        rng_in = np.linalg.norm(pts[inl], axis=1)
        st = {"src": src, "inliers": int(inl.sum()), "rms_mm": float(np.sqrt(np.mean(r ** 2)) * 1000), "far_m": float(np.percentile(rng_in, 95)),
              "height_m": d}
        if self.use_grav and self.up is not None:
            ang = float(np.degrees(np.arccos(min(1.0, abs(float(n @ (self.up / np.linalg.norm(self.up))))))))
            st["grav_deg"] = ang
            if ang > self.grav_tol:
                self.reject = f"plane {ang:.0f}deg off gravity"
                return None
        if self.prior is not None:
            pa = float(np.degrees(np.arccos(min(1.0, float(n @ self.prior[0])))))
            if pa > self.prior_deg or abs(d - self.prior[1]) > self.prior_dh:
                self.reject = f"off calibration ({pa:.0f}deg, {abs(d - self.prior[1]) * 100:.0f}cm)"
                return None
        if not (self.hmin < d < self.hmax):
            self.reject = f"height {d:.2f} m"
            return None
        self.reject = ""
        self.stats = st
        return n, d

    def process(self, depth):
        out = {"status": "OK", "ground_plane_ok": False, "nearest_obstacle_m": None, "zones": {"left": None, "center": None, "right": None},
               "camera_height_m": None, "camera_tilt_deg": None}
        if depth is None or depth.shape != (self.h, self.w):
            out["status"] = "SENSOR_ERROR"
            return out
        z = depth.astype(np.float32) * self.scale
        valid = z > 0
        P = np.stack([self.xn * z, self.yn * z, z], axis=-1)
        self.z, self.valid, self.P = z, valid, P                       # 화면 루프가 같은 점을 다시 계산하지 않게
        if valid.mean() < 0.05:
            out["status"] = "SENSOR_ERROR"
            return out
        fit = self._fit(P, valid, z)
        if fit is not None:
            n, d = fit
            if self.n is None:
                self.n, self.d = n, d
            else:
                nb = 0.7 * self.n + 0.3 * n
                self.n, self.d = nb / np.linalg.norm(nb), 0.7 * self.d + 0.3 * d
            self.hold = 0
        else:
            self.hold += 1
            if self.n is not None and self.hold > self.hold_max:
                self.n = self.d = None
        if self.n is None:
            out["status"] = "NO_GROUND"
            return out
        out["ground_plane_ok"] = fit is not None
        out["camera_height_m"] = round(float(self.d), 3)
        out["camera_tilt_deg"] = round(float(np.degrees(np.arccos(min(1.0, abs(self.n[1]))))), 1)
        H = P @ self.n + self.d
        zc, xc = np.array([0, 0, 1.0]), np.array([1.0, 0, 0])
        f = zc - (zc @ self.n) * self.n
        f /= np.linalg.norm(f)
        r = xc - (xc @ self.n) * self.n
        r /= np.linalg.norm(r)
        fwd, lat = P @ f, P @ r
        self.fwd, self.lat = fwd, lat
        b = self.band(z)
        floor = valid & (np.abs(H) < b)
        obst = valid & (H > np.maximum(self.obst_h, 2 * b)) & (H < self.max_h) & (fwd > 0) & (fwd < self.max_dist)
        obst = cv2.morphologyEx(obst.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool)
        self.floor_mask, self.obst_mask = floor, obst
        hw = self.half_width
        for name, m in (("left", lat < -hw), ("center", np.abs(lat) <= hw), ("right", lat > hw)):
            mm = obst & m
            if mm.any():
                out["zones"][name] = round(max(0.0, float(np.percentile(fwd[mm], 2)) - self.front_offset), 2)
        vals = [v for v in out["zones"].values() if v is not None]
        out["nearest_obstacle_m"] = min(vals) if vals else None
        if fit is not None:
            far = fwd[floor & (fwd > 0)]
            self.stats["floor_far_m"] = float(np.percentile(far, 95)) if far.size > 50 else None
        return out
