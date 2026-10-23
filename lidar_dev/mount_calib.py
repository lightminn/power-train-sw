# mount_calib.py — L515 장착 캘리브레이션(live_overlay 7판, 10-22 밤 — 사용자: '로버에 장착하자마자 쓸 수 있게 자체 캘리브레이션')
#   로버를 평평한 바닥에 세우고 키 k → 3 s 가만히: 바닥면(ground_est.GroundDetector, 프레임마다)과 IMU 중력(가속도 보정 적용, 깊이 좌표)을 모아
#   - 장착 숙임·옆(광민 base_to_optical 규약, 바닥 법선 기준 — 지도는 깊이 좌표에서 쓰므로 바닥면이 기준), 카메라 높이(바닥에서 렌즈)
#   - IMU ↔ 바닥 법선 각(바닥이 수평이면 IMU–깊이 축 어긋남·바닥 기울기 합) → IMU 위 방향을 바닥 법선에 맞추는 작은 회전(보정)
#   - 바닥 잔차·안쪽 점 수·흔들림(프레임 사이 높이·각)
#   거부: 자이로가 3 °/s 넘게 돌거나 가속도 흔들림이 크면(움직임), 바닥을 찾은 프레임이 반 미만, 높이 흔들림 > 1 cm, 법선 흔들림 > 1°,
#   IMU–바닥 각 > 5°(바닥이 기울었거나 L515가 장착대에서 들뜸)
#   저장: ~/.config/l515-overlay/mount.json(+ 기록 폴더 사본). 다음 실행에서 자동으로 불러 광민 지도 장착 각·바닥 찾기 사전값(높이 ±prior_dh, 각 ±prior_deg)으로 씀.
#   지금 IMU 자세가 캘리브레이션 때와 10° 넘게 다르면(손에 들었거나 장착대가 바뀜) 사전값을 쓰지 않는다(mount_matches)
import json, math, os, time

import numpy as np

CONFIG = os.environ.get("L515_MOUNT_CONFIG") or os.path.expanduser("~/.config/l515-overlay/mount.json")   # 시험 때 환경 변수로 다른 경로


def rot_between(a, b):
    """단위 벡터 a → b로 돌리는 회전 행렬(로드리게스)."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * (1.0 / (1.0 + c))


def pitch_roll_from_up(up):
    """깊이 광학 좌표의 위 방향 → 광민 base_to_optical(pitch_down, roll, 0)의 (pitch_down, roll) °(tmap_live.mount_from_gravity와 같은 식)."""
    u = np.asarray(up, float) / np.linalg.norm(up)
    return math.degrees(math.asin(max(-1.0, min(1.0, -u[2])))), math.degrees(math.atan2(-u[0], -u[1]))


class MountCalibrator:
    SECONDS = 3.0

    def __init__(self, t0):
        self.t0, self.planes, self.ups, self.gyro_max, self.acc_std, self.n_frames = t0, [], [], 0.0, 0.0, 0

    def add(self, det, up_depth, gyro_rad_s=None):
        self.n_frames += 1
        if det.n is not None and det.reject == "" and det.stats.get("inliers"):
            self.planes.append((det.n.copy(), float(det.d), float(det.stats.get("rms_mm", np.nan)), int(det.stats.get("inliers", 0)),
                                det.stats.get("floor_far_m")))
        if up_depth is not None:
            self.ups.append(np.asarray(up_depth, float))
        if gyro_rad_s is not None:
            self.gyro_max = max(self.gyro_max, float(np.max(np.linalg.norm(np.atleast_2d(gyro_rad_s), axis=1))))

    def done(self, now):
        return now - self.t0 >= self.SECONDS

    def solve(self, serial, cal_name):
        why = []
        if self.gyro_max > math.radians(3.0):
            why.append(f"움직임(자이로 최대 {math.degrees(self.gyro_max):.1f} °/s > 3)")
        if len(self.planes) < max(5, self.n_frames // 2):
            why.append(f"바닥 찾은 프레임 {len(self.planes)}/{self.n_frames}")
        if len(self.ups) < 20:
            why.append("IMU 표본 부족")
        if why:
            return None, why
        N = np.array([p[0] for p in self.planes])
        D = np.array([p[1] for p in self.planes])
        n = N.mean(0)
        n /= np.linalg.norm(n)
        jitter_deg = float(np.degrees(np.arccos(np.clip(N @ n, -1, 1))).std())
        d = float(np.median(D))
        U = np.array(self.ups)
        up = U.mean(0)
        up /= np.linalg.norm(up)
        imu_deg = float(np.degrees(np.arccos(min(1.0, float(up @ n)))))
        if D.std() > 0.01:
            why.append(f"높이 흔들림 {D.std() * 100:.1f} cm > 1")
        if jitter_deg > 1.0:
            why.append(f"법선 흔들림 {jitter_deg:.2f}° > 1")
        if imu_deg > 5.0:
            why.append(f"IMU–바닥 각 {imu_deg:.1f}° > 5(바닥 기울기·L515 들뜸?)")
        p_plane, r_plane = pitch_roll_from_up(n)
        p_imu, r_imu = pitch_roll_from_up(up)
        res = {"serial": serial, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "valid": not why, "reasons": why,
               "plane_normal_depth": [round(float(v), 6) for v in n], "camera_height_m": round(d, 4),
               "pitch_down_deg": round(p_plane, 3), "roll_deg": round(r_plane, 3), "yaw_deg": 0.0,
               "imu_up_depth": [round(float(v), 6) for v in up], "imu_pitch_down_deg": round(p_imu, 3), "imu_roll_deg": round(r_imu, 3),
               "imu_vs_plane_deg": round(imu_deg, 3), "R_imu_correction": np.round(rot_between(up, n), 8).tolist(),
               "height_std_mm": round(float(D.std() * 1000), 2), "normal_jitter_deg": round(jitter_deg, 3),
               "floor_rms_mm": round(float(np.nanmedian([p[2] for p in self.planes])), 2), "inliers_median": int(np.median([p[3] for p in self.planes])),
               "floor_far_m": None if not any(p[4] for p in self.planes) else round(float(np.median([p[4] for p in self.planes if p[4]])), 2),
               "frames": [len(self.planes), self.n_frames], "gyro_max_dps": round(math.degrees(self.gyro_max), 2), "accel_calibration": cal_name,
               "note": "장착 숙임·옆은 바닥 법선 기준(평지 가정), 좌우 돌아감(yaw)은 정지 상태로 못 잼 → 0(CAD). R_imu_correction = IMU 위 → 바닥 법선 회전"}
        return res, why


def save(res, extra_dir=None):
    os.makedirs(os.path.dirname(CONFIG), exist_ok=True)
    json.dump(res, open(CONFIG, "w"), ensure_ascii=False, indent=1)
    paths = [CONFIG]
    if extra_dir:
        p = os.path.join(extra_dir, time.strftime("mount_calib_%Y%m%d_%H%M%S.json"))
        json.dump(res, open(p, "w"), ensure_ascii=False, indent=1)
        paths.append(p)
    return paths


def load(serial=None):
    try:
        res = json.load(open(CONFIG))
    except (OSError, ValueError):
        return None
    if not res.get("valid"):
        return None
    if serial and res.get("serial") not in (None, serial):
        return None
    return res


def mount_matches(res, up_depth, tol_deg=10.0):
    """지금 IMU 위 방향(보정 회전 적용 전)이 캘리브레이션 때 IMU 위와 tol° 안이면 True — 손에 들었거나 장착이 바뀌면 사전값을 쓰지 않음."""
    if res is None or up_depth is None:
        return False
    a = np.asarray(res["imu_up_depth"], float)
    u = np.asarray(up_depth, float) / np.linalg.norm(up_depth)
    return float(np.degrees(np.arccos(min(1.0, float(a @ u))))) <= tol_deg
