# synth_seq.py — 정답을 아는 합성 L515 기록(l515-seq v1) 생성: 바닥·벽·상자 장면을 정해진 궤적으로 레이캐스트.
# map_seq.py(L1)·obstacles_map.py(L2) 시험용. 실제 장치 없이 자세·치수 오차를 숫자로 잴 수 있다.
# 사용: python synth_seq.py <출력 폴더> [프레임 수]   → 폴더/truth.json(카메라 자세·상자 치수)
import json, math, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import seq_format as sf

W, H = 640, 480
K = dict(width=W, height=H, fx=464.5, fy=464.2, ppx=351.9, ppy=245.5, model="none", coeffs=[0.0] * 5)  # 실제 L515 640×480 공장값
SCALE = sf.L515_DEPTH_SCALE
# 장면(월드: z 위, 바닥 z = 0, m). 상자: (x0, x1, y0, y1, 높이)
BOXES = [(1.6, 2.0, -0.5, -0.1, 0.10), (2.2, 2.5, 0.3, 0.7, 0.30), (3.0, 3.6, -0.9, -0.4, 0.06), (2.6, 2.9, -0.2, 0.1, 0.07)]  # 기준값 v0.1: 6 cm 주의, 7·10·30 cm 불가
WALLS_X = (-0.5, 5.0)
WALLS_Y = (-2.0, 2.0)
CEIL = 2.5
R_BASE = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)  # optical(x 오른쪽, y 아래, z 앞) → 월드(x 앞, y 왼쪽, z 위)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def cam_pose(x, y, yaw, h=0.46, pitch_deg=15.0):
    """월드 ← 카메라 4×4. 아래로 pitch_deg 숙인 카메라(L515 실물 장착 15°, CAD의 20°는 구버전)."""
    R = rot_z(yaw) @ R_BASE @ rot_x(-math.radians(pitch_deg))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (x, y, h)
    return T


def texture(p):
    """밝기 무늬(광도 항용): 월드 좌표의 결정적 패턴 0~255."""
    return (128 + 60 * np.sin(p[..., 0] * 9.0) * np.cos(p[..., 1] * 7.0) + 40 * np.sin((p[..., 0] + p[..., 1] + p[..., 2]) * 23.0)).clip(0, 255)


def render(Twc):
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    d_cam = np.stack([(u - K["ppx"]) / K["fx"], (v - K["ppy"]) / K["fy"], np.ones_like(u, float)], -1)  # z = 1
    d = d_cam @ Twc[:3, :3].T
    o = Twc[:3, 3]
    t = np.full((H, W), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for axis, vals in ((2, (0.0, CEIL)), (0, WALLS_X), (1, WALLS_Y)):
            for c in vals:
                tt = (c - o[axis]) / d[..., axis]
                t = np.where((tt > 0) & (tt < t), tt, t)
        for x0, x1, y0, y1, hgt in BOXES:
            lo, hi = np.array([x0, y0, 0.0]), np.array([x1, y1, hgt])
            t1, t2 = (lo - o) / d, (hi - o) / d
            tmin = np.max(np.minimum(t1, t2), -1)
            tmax = np.min(np.maximum(t1, t2), -1)
            hit = (tmax >= tmin) & (tmin > 0) & (tmin < t)
            t = np.where(hit, tmin, t)
    z = t  # d_cam의 z가 1이라 t가 곧 깊이
    p = o + d * t[..., None]
    ok = np.isfinite(z) & (z > 0.25) & (z < 6.0)
    depth = np.where(ok, np.round(z / SCALE), 0).astype(np.uint16)
    ir = np.where(ok, texture(np.where(ok[..., None], p, 0)), 0).astype(np.uint8)
    return depth, ir


def trajectory(n):
    """앞으로 1.2 m 가며 왼쪽으로 20° 돌았다가 출발 자세 근처로 돌아오지 않는 열린 경로(손 기록과 비슷한 속도, 15 Hz)."""
    poses = []
    for i in range(n):
        s = i / max(n - 1, 1)
        poses.append(cam_pose(0.0 + 1.2 * s, 0.15 * math.sin(math.pi * s), math.radians(20.0) * s))
    return poses


def main():
    out = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    os.makedirs(out, exist_ok=True)
    poses = trajectory(n)
    meta = {"format": sf.FORMAT, "format_version": sf.FORMAT_VERSION, "label": "synthetic", "note": "synth_seq.py",
            "device": {"name": "synthetic", "product_line": "L500"}, "depth_scale_m": SCALE, "depth_intrinsics": K,
            "infrared_intrinsics": K, "color_intrinsics": None, "streams": {"depth": {"w": W, "h": H}, "ir": {"w": W, "h": H}, "color": None},
            "imu": {"available": False}, "every": 2, "stats": None}
    sf.write_json_atomic(os.path.join(out, "meta.json"), meta)
    w = sf.SeqWriter(out, ("depth", "ir"), workers=4, queue_size=n + 2)
    for i, T in enumerate(poses):
        depth, ir = render(T)
        ts = f"{1000 + i * 66.67:.3f}"
        w.submit(i, {"depth": depth, "ir": ir}, {"host_mono_s": f"{i / 15:.6f}", "host_wall": "synthetic",
                                               "depth_ts_ms": ts, "depth_ts_domain": "synthetic", "depth_fn": 2 * i,
                                               "ir_ts_ms": ts, "ir_ts_domain": "synthetic", "ir_fn": 2 * i})
    st = w.close()
    meta["stats"] = {"saved": st["saved"], "error": st["error"], "write_errors": st["write_errors"]}
    sf.write_json_atomic(os.path.join(out, "meta.json"), meta)
    json.dump({"T_world_cam": [T.tolist() for T in poses], "boxes": BOXES, "floor_z": 0.0,
               "note": "월드 z 위, 바닥 z = 0. 상자 (x0, x1, y0, y1, 높이)"}, open(os.path.join(out, "truth.json"), "w"), indent=1)
    print(f"합성 기록 {st['saved']}프레임 → {out}")


if __name__ == "__main__":
    main()
