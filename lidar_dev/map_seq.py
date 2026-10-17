# map_seq.py — L1: l515-seq v1 기록 하나로 3D 지도를 만든다(오프라인).
# 프레임 간 자세 = IR+깊이 RGB-D 오도메트리(Hybrid). ICP는 기본으로 맞춤 지표(수용 판정)만 잰다 — 점–평면 ICP 보정은 바닥이
# 화면 대부분인 장면에서 바닥면을 따라 미끄러져(합성 시험 이동 오차 53 mm 대 오도메트리만 2.6 mm) 선택지(--refine icp)로만 둔다.
# 추적 실패 프레임은 넣지 않고 TRACKING_LOST로 남긴다.
# 지도 좌표 = 첫 프레임 바닥 평면 기준(z 위, 바닥 z = 0, 원점은 시작 카메라 바로 아래, x는 시작 시 카메라 앞의 수평 성분).
# 통합 = Open3D 텐서 VoxelBlockGrid TSDF(옛 ScalableTSDFVolume은 0.20 pip판에서 점을 내지 않음).
# 사용: python map_seq.py <기록 폴더> <출력 폴더> [--step N] [--voxel 0.015] [--max-frames N]
# 설계: docs/specs/2026-10-17-l515-seq-recorder-and-mapping-design.md §2
import argparse, csv, json, math, os, sys, time
import numpy as np
import cv2
import open3d as o3d
import open3d.core as o3c

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from seq_format import check_sequence

# 수용 조건(설계 §2.2) — 하나라도 어기면 TRACKING_LOST
ACCEPT = {"icp_fitness_min": 0.6, "icp_rmse_max_m": 0.015, "step_trans_max_m": 0.15, "step_rot_max_deg": 10.0, "gap_max_s": 0.5}
KEY_TRANS_M, KEY_ROT_DEG = 0.05, 5.0
RANGE_M = (0.25, 4.0)
LOST_SPLIT_S = 1.0


def edge_filter(z_u16, ratio=1.12):
    """3×3 최대/최소 깊이 비가 ratio를 넘는 화소를 0으로(떠다니는 점·경계 양쪽 1화소)."""
    z = z_u16.astype(np.float32)
    k = np.ones((3, 3), np.uint8)
    zmax = cv2.dilate(z, k)
    zmin = cv2.erode(np.where(z > 0, z, 1e9).astype(np.float32), k)
    out = z_u16.copy()
    out[(z > 0) & (zmax > ratio * zmin)] = 0
    return out


def rot_deg(T):
    return math.degrees(math.acos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1.0, 1.0)))


def floor_frame(depth_u16, K, scale):
    """첫 프레임 바닥 평면 → 월드 ← 카메라 변환. 반환: (T_world_cam 또는 None, 정보)."""
    h, w = depth_u16.shape
    z = depth_u16.astype(np.float32) * scale
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    sel = (z > RANGE_M[0]) & (z < RANGE_M[1]) & (v > h * 0.5)
    P = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], -1)[sel]
    if len(P) < 500:
        return None, {"reason": "바닥 후보 점 부족"}
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)).voxel_down_sample(0.02)
    plane, inl = pc.segment_plane(0.02, 3, 1000)
    n, d = np.array(plane[:3]), plane[3]
    nn = np.linalg.norm(n)
    n, d = n / nn, d / nn
    if d < 0:
        n, d = -n, -d                       # n은 바닥에서 카메라 쪽(위), d = 카메라 높이
    tilt = math.degrees(math.acos(min(1.0, abs(n[1]))))
    frac = len(inl) / max(len(pc.points), 1)
    info = {"camera_height_m": float(d), "tilt_deg": float(tilt), "inlier_frac": float(frac)}
    if tilt > 40 or not 0.15 < d < 1.6 or frac < 0.3:
        info["reason"] = "바닥 평면 조건 불충족"
        return None, info
    zw = n                                   # 월드 z(위)를 카메라 좌표로
    fwd = np.array([0.0, 0.0, 1.0]) - zw * zw[2]
    xw = fwd / np.linalg.norm(fwd)           # 카메라 앞의 수평 성분
    yw = np.cross(zw, xw)
    R_cw = np.stack([xw, yw, zw], 1)         # 열 = 월드 축의 카메라 좌표 → R_cam_world
    T = np.eye(4)
    T[:3, :3] = R_cw.T                       # R_world_cam
    T[:3, 3] = [0.0, 0.0, d]                 # 카메라는 바닥 위 d, 원점은 바로 아래
    return T, info


class Mapper:
    def __init__(self, meta, voxel=0.015, refine="none"):
        self.refine = refine
        di = meta["depth_intrinsics"]
        self.K = np.array([[di["fx"], 0, di["ppx"]], [0, di["fy"], di["ppy"]], [0, 0, 1]], float)
        self.Ko = o3d.camera.PinholeCameraIntrinsic(int(di["width"]), int(di["height"]), di["fx"], di["fy"], di["ppx"], di["ppy"])
        self.scale = float(meta["depth_scale_m"])
        self.voxel = voxel
        dev = o3c.Device("CPU:0")
        self.dev = dev
        self.vbg = o3d.t.geometry.VoxelBlockGrid(attr_names=("tsdf", "weight", "color"),
                                                 attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
                                                 attr_channels=((1), (1), (3)), voxel_size=voxel, block_resolution=16,
                                                 block_count=100000, device=dev)
        self.Kt = o3c.Tensor(self.K, o3c.float64)
        self.opt = o3d.pipelines.odometry.OdometryOption()
        self.opt.depth_min, self.opt.depth_max, self.opt.depth_diff_max = RANGE_M[0], RANGE_M[1], 0.07

    def rgbd(self, depth_u16, ir_u8):
        return o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(ir_u8), o3d.geometry.Image(depth_u16), depth_scale=1.0 / self.scale,
            depth_trunc=RANGE_M[1], convert_rgb_to_intensity=True)

    def cloud(self, rgbd):
        pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.Ko)
        return pc

    def register(self, cur, key, init):
        """source = 현재, target = 키프레임. 반환: (T_key_cur, 지표 dict)."""
        ok, T, _ = o3d.pipelines.odometry.compute_rgbd_odometry(
            cur["rgbd"], key["rgbd"], self.Ko, init, o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), self.opt)
        m = {"odom_ok": bool(ok)}
        T0 = T if ok else init
        tr = T0
        reg = None
        for vs in (0.04, 0.02, 0.01):
            s = cur["pc"].voxel_down_sample(vs)
            t = key["pc"].voxel_down_sample(vs)
            for c in (s, t):
                c.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vs * 3, max_nn=30))
            if self.refine == "none":   # 오도메트리 결과는 그대로 두고 맞춤 지표만 잰다
                reg = o3d.pipelines.registration.evaluate_registration(s, t, vs * 2, tr)
                continue
            reg = o3d.pipelines.registration.registration_icp(
                s, t, vs * 2, tr, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
            tr = reg.transformation
        info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            cur["pc"].voxel_down_sample(0.02), key["pc"].voxel_down_sample(0.02), 0.04, tr)
        ev = np.linalg.eigvalsh(info)
        m.update(icp_fitness=float(reg.fitness), icp_rmse=float(reg.inlier_rmse), info_min_eig=float(ev[0]),
                 info_cond=float(ev[-1] / max(ev[0], 1e-9)))
        return np.asarray(tr), m

    def integrate(self, depth_u16, ir_u8, T_world_cam):
        dt = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(depth_u16))).to(self.dev)
        ct = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(np.repeat(ir_u8[..., None], 3, 2)))).to(self.dev)
        ext = o3c.Tensor(np.linalg.inv(T_world_cam), o3c.float64)  # 월드 → 카메라
        blk = self.vbg.compute_unique_block_coordinates(dt, self.Kt, ext, 1.0 / self.scale, RANGE_M[1], trunc_voxel_multiplier=3.0)
        self.vbg.integrate(blk, dt, ct, self.Kt, self.Kt, ext, 1.0 / self.scale, RANGE_M[1], trunc_voxel_multiplier=3.0)


def load_frame(folder, i):
    d = cv2.imread(os.path.join(folder, "depth", f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
    ir = cv2.imread(os.path.join(folder, "infrared", f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
    return d, ir


def run(folder, out, step=1, voxel=0.015, max_frames=0, refine="none"):
    E, W, _ = check_sequence(folder)
    if E:
        raise SystemExit(f"기록 점검 실패: {E}")
    meta = json.load(open(os.path.join(folder, "meta.json"), encoding="utf-8"))
    if not (meta.get("streams") or {}).get("ir"):
        raise SystemExit("IR 스트림이 없는 기록 — v1 L1은 IR을 밝기로 쓴다")
    rows = list(csv.DictReader(open(os.path.join(folder, "frames.csv"))))
    idx = list(range(0, len(rows), step))[: max_frames or None]
    os.makedirs(out, exist_ok=True)
    mp = Mapper(meta, voxel, refine)
    t_start = time.time()
    d0, ir0 = load_frame(folder, idx[0])
    d0 = edge_filter(d0)
    T_w_c0, finfo = floor_frame(d0, mp.K, mp.scale)
    gravity = "floor_plane"
    if T_w_c0 is None:
        T_w_c0, gravity = np.eye(4), "none(카메라 좌표 그대로)"
    log, poses = [], {}
    key = None
    T_w_key = None
    T_prev_rel = np.eye(4)   # 직전 프레임의 키프레임 대비 자세(등속 가정 초기값용)
    T_vel = np.eye(4)
    last_ok_t = None
    n_key = n_int = n_lost = 0
    segment = 0
    last_int_t = -1e9
    for i in idx:
        t = float(rows[i]["depth_ts_ms"]) / 1000.0
        d, ir = load_frame(folder, i)
        d = edge_filter(d)
        rg = mp.rgbd(d, ir)
        cur = {"rgbd": rg, "pc": mp.cloud(rg), "t": t, "i": i}
        state, m = "OK", {}
        if key is None:
            key, T_w_key, T_cur_w = cur, T_w_c0, T_w_c0
            T_rel = np.eye(4)
            n_key += 1
            state = "KEY_INIT"
        else:
            init = T_prev_rel @ T_vel
            T_rel, m = mp.register(cur, key, init)
            step_T = np.linalg.inv(T_prev_rel) @ T_rel
            gap = t - (last_ok_t if last_ok_t is not None else t)
            bad = []
            if not m["odom_ok"]:
                bad.append("odom_fail")
            if m["icp_fitness"] < ACCEPT["icp_fitness_min"]:
                bad.append("fitness")
            if m["icp_rmse"] > ACCEPT["icp_rmse_max_m"]:
                bad.append("rmse")
            if np.linalg.norm(step_T[:3, 3]) > ACCEPT["step_trans_max_m"]:
                bad.append("step_trans")
            if rot_deg(step_T) > ACCEPT["step_rot_max_deg"]:
                bad.append("step_rot")
            if gap > ACCEPT["gap_max_s"]:
                bad.append("gap")
            if bad and not (bad == ["odom_fail"] and m["icp_fitness"] >= ACCEPT["icp_fitness_min"] and m["icp_rmse"] <= ACCEPT["icp_rmse_max_m"]):
                state = "TRACKING_LOST:" + "+".join(bad)
                n_lost += 1
                if last_ok_t is not None and t - last_ok_t > LOST_SPLIT_S:
                    segment += 1        # 1 s 넘게 잃으면 지도 구간을 나눈다(지금은 기록만)
                log.append({"idx": i, "t": t, "state": state, "segment": segment, **m})
                continue
            T_cur_w = T_w_key @ T_rel                        # T_world_cur = T_world_key · T_key_cur
            T_vel = step_T
            T_prev_rel = T_rel
            if np.linalg.norm(T_rel[:3, 3]) > KEY_TRANS_M or rot_deg(T_rel) > KEY_ROT_DEG:
                key, T_w_key = cur, T_cur_w
                T_prev_rel = np.eye(4)
                n_key += 1
                state = "KEY"
        last_ok_t = t
        poses[i] = T_cur_w
        if state.startswith("KEY") or t - last_int_t >= 0.5:
            mp.integrate(d, ir, T_cur_w)
            last_int_t = t
            n_int += 1
        log.append({"idx": i, "t": t, "state": state, "segment": segment, **m,
                    **{f"T{r}{c}": float(T_cur_w[r, c]) for r in range(3) for c in range(4)}})
    pc = mp.vbg.extract_point_cloud(weight_threshold=1.0)
    mesh = mp.vbg.extract_triangle_mesh(weight_threshold=1.0)
    o3d.t.io.write_point_cloud(os.path.join(out, "map_points.ply"), pc)
    o3d.t.io.write_triangle_mesh(os.path.join(out, "map_mesh.ply"), mesh)
    cols = sorted({k for r in log for k in r}, key=lambda k: (k not in ("idx", "t", "state", "segment"), k))
    with open(os.path.join(out, "poses.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, cols)
        w.writeheader()
        w.writerows(log)
    ok_idx = [r["idx"] for r in log if not r["state"].startswith("TRACKING_LOST")]
    summary = {"input": os.path.abspath(folder), "frames_used": len(idx), "tracked": len(ok_idx), "lost": n_lost,
               "keyframes": n_key, "integrated": n_int, "segments": segment + 1, "gravity": gravity, "floor": finfo,
               "voxel_m": voxel, "refine": refine, "accept": ACCEPT, "map_points": int(pc.point.positions.shape[0]),
               "start_end_trans_m": float(np.linalg.norm(poses[ok_idx[-1]][:3, 3] - poses[ok_idx[0]][:3, 3])) if ok_idx else None,
               "sec_per_frame": round((time.time() - t_start) / max(len(idx), 1), 3), "check_warnings": W}
    json.dump(summary, open(os.path.join(out, "summary.json"), "w"), ensure_ascii=False, indent=1)
    np.save(os.path.join(out, "poses_T_world_cam.npy"), np.array([poses[i] if i in poses else np.full((4, 4), np.nan) for i in idx]))
    return summary


def main():
    ap = argparse.ArgumentParser(description="L515 기록 → 3D 지도(L1)")
    ap.add_argument("folder")
    ap.add_argument("out")
    ap.add_argument("--step", type=int, default=1, help="N프레임마다 하나 사용")
    ap.add_argument("--voxel", type=float, default=0.015)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--refine", choices=("none", "icp"), default="none",
                    help="none: RGB-D 오도메트리 결과를 쓰고 ICP는 맞춤 지표만(기본) / icp: 점–평면 ICP로 보정(바닥이 많은 장면에서 미끄러짐 — 합성 시험 53 mm 대 2.6 mm)")
    a = ap.parse_args()
    s = run(a.folder, a.out, a.step, a.voxel, a.max_frames, a.refine)
    print(json.dumps(s, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
