# map_seq.py — L1: l515-seq v1 기록 하나로 3D 지도를 만든다(오프라인).
# 프레임 간 자세 = IR+깊이 RGB-D 오도메트리(Hybrid, 현재→키프레임). 오도메트리 실패·수용 조건 위반 프레임은 TRACKING_LOST로 남기고
# 통합하지 않는다. ICP는 기본으로 맞춤 지표만 잰다 — 점–평면 ICP 보정은 바닥이 화면 대부분인 장면에서 바닥면을 따라 미끄러져
# (합성 시험 이동 오차 53 mm 대 오도메트리만 2.6 mm) 선택지(--refine icp)로만 둔다(잠정 선택 — 실기 이동 기록으로 다시 비교).
# 지도 좌표 = 시작 구간 첫 프레임의 **초기 평면 정렬**(z 위, 평면 z = 0, 원점 = 시작 카메라에서 평면에 내린 발, x = 카메라 앞의 수평 성분).
#   평면이 바닥이라는 증거(위 방향·카메라 높이·평면 점 비율·평면 아래 점 비율·지지 폭)가 모자라면 정렬 실패 → 지도는 만들되
#   alignment_valid = false로 남기고 L2(obstacles_map)는 거부한다. 시작은 수평이 확인된 바닥에서 해야 한다.
# 1 s 넘게 추적을 잃으면 새 구간(새 키프레임·새 TSDF·자기 초기 평면)을 연다. 구간 사이 변환은 미확정이다.
# 통합 = Open3D 텐서 VoxelBlockGrid TSDF(옛 ScalableTSDFVolume은 0.20 pip판에서 점을 내지 않음).
# 사용: python map_seq.py <기록 폴더> <출력 폴더> [--step N] [--voxel 0.015] [--max-frames N] [--refine none|icp]
# 설계: docs/specs/2026-10-17-l515-seq-recorder-and-mapping-design.md §2, 검토 반영: docs/plans/2026-10-18-l515-seq-l1-plan-and-log.md
import argparse, csv, json, math, os, sys, time
import numpy as np
import cv2
import open3d as o3d
import open3d.core as o3c

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from seq_format import check_sequence

_odometry = o3d.pipelines.odometry.compute_rgbd_odometry   # 시험에서 실패 주입용으로 바꿔 끼운다

PREPROC_VERSION = "pp1: 범위 0.25–4.0 m 밖 0, 3×3 최대/최소 비 1.12 경계 필터"
RANGE_M = (0.25, 4.0)
# 수용 조건(설계 §2.2). 걸음 제한은 실제 시간 간격에 비례해 늘린다(손실 뒤 복귀용): max(기본, 속도 상한 × dt)
ACCEPT = {"icp_fitness_min": 0.6, "icp_rmse_max_m": 0.015, "step_trans_base_m": 0.15, "step_rot_base_deg": 10.0,
          "v_max_m_s": 1.0, "w_max_deg_s": 60.0}
KEY_TRANS_M, KEY_ROT_DEG = 0.05, 5.0
LOST_SPLIT_S = 1.0            # 이만큼 넘게 잃으면 새 구간
INTEGRATE_EVERY_S = 0.5       # 키프레임 사이에도 이 간격마다 한 장은 통합
WEIGHT_MIN = 0.5              # 표면 추출: weight > 0.5 = 한 번 이상 통합된 칸(Open3D는 '초과' 조건)
VBG_INIT_BLOCKS = 20000       # 초기 용량(16³ 블록). 부족하면 Open3D가 늘린다
VBG_MAX_BLOCKS = 120000       # 예산: 넘으면 그 구간 통합을 멈추고 budget_hit(메모리 보호)
DEGEN_RATIO = 1e-3            # 점–평면 Hessian 최소/최대 고유값 비가 이보다 작으면 기하 퇴화 표시(거부는 안 함 — 광도 항이 잡을 수 있음)


def preprocess(depth_u16, scale):
    """공통 전처리: 범위 밖 0, 경계 필터. 오도메트리·점구름·TSDF가 같은 깊이를 쓴다."""
    z = depth_u16.astype(np.float32) * scale
    d = depth_u16.copy()
    d[(z < RANGE_M[0]) | (z > RANGE_M[1])] = 0
    zf = d.astype(np.float32)
    k = np.ones((3, 3), np.uint8)
    zmax = cv2.dilate(zf, k)
    zmin = cv2.erode(np.where(zf > 0, zf, 1e9).astype(np.float32), k)
    d[(zf > 0) & (zmax > 1.12 * zmin)] = 0
    return d


def rot_deg(T):
    return math.degrees(math.acos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1.0, 1.0)))


def initial_plane(depth_u16, K, scale):
    """초기 평면 정렬: 바닥 평면 → 월드 ← 카메라. 반환: (T 또는 None, 정보). 바닥이라는 증거가 모자라면 None."""
    h, w = depth_u16.shape
    z = depth_u16.astype(np.float32) * scale
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    valid = z > 0
    P_all = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], -1)
    P = P_all[valid & (v > h * 0.5)]
    info = {"valid_px_frac": float(valid.mean())}
    if len(P) < 500:
        return None, {**info, "reason": "하단 유효 점 부족"}
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)).voxel_down_sample(0.02)
    plane, inl = pc.segment_plane(0.02, 3, 1000)
    n, d = np.array(plane[:3]), plane[3]
    nn = np.linalg.norm(n)
    n, d = n / nn, d / nn
    if d < 0:
        n, d = -n, -d                       # n은 평면에서 카메라 쪽, d = 카메라에서 평면까지 수직 거리
    up_dot = float(-n[1])                   # 정상 장착(앞·아래를 봄)에서 위 방향은 optical −y 쪽
    tilt = math.degrees(math.acos(min(1.0, abs(n[1]))))
    Pin = np.asarray(pc.points)[inl]
    frac = len(inl) / max(len(pc.points), 1)
    below = float(np.mean(P_all[valid] @ n + d < -0.03))   # 평면보다 3 cm 넘게 아래인 점 비율 — 상자 윗면이면 크다
    span = float(Pin[:, 0].max() - Pin[:, 0].min()) if len(Pin) else 0.0   # 가로(optical x) 지지 폭
    info.update(camera_height_m=float(d), tilt_deg=float(tilt), up_dot=up_dot, inlier_frac=float(frac),
                below_frac=below, inlier_span_x_m=span)
    bad = []
    if up_dot < math.cos(math.radians(40)):
        bad.append("위 방향 아님(법선이 optical −y와 40° 넘게 다름)")
    if not 0.15 < d < 1.6:
        bad.append("카메라 높이 범위 밖")
    if frac < 0.3:
        bad.append("평면 점 비율 < 0.3")
    if below > 0.05:
        bad.append("평면 아래 점 > 5%(가장 낮은 면이 아님 — 상자 윗면 등)")
    if span < 0.5:
        bad.append("지지 폭 < 0.5 m")
    if bad:
        return None, {**info, "reason": "; ".join(bad)}
    zw = n
    fwd = np.array([0.0, 0.0, 1.0]) - zw * zw[2]
    xw = fwd / np.linalg.norm(fwd)
    yw = np.cross(zw, xw)
    T = np.eye(4)
    T[:3, :3] = np.stack([xw, yw, zw], 1).T  # R_world_cam(열 = 월드 축의 카메라 좌표 → 전치)
    T[:3, 3] = [0.0, 0.0, d]
    return T, info


def p2plane_degeneracy(src, tgt, T, max_dist):
    """점–평면 잔차의 Hessian(법선 포함) 최소/최대 고유값 비. 회전 열은 대표 거리로 정규화한다."""
    s = o3d.geometry.PointCloud(src).transform(T)
    tree = o3d.geometry.KDTreeFlann(tgt)
    S, Tn = np.asarray(s.points), np.asarray(tgt.normals)
    if len(S) == 0 or len(Tn) == 0:
        return 0.0, 0
    J = []
    for p in S[:: max(1, len(S) // 3000)]:
        k, idx, d2 = tree.search_knn_vector_3d(p, 1)
        if k and d2[0] < max_dist ** 2:
            nrm = Tn[idx[0]]
            J.append(np.r_[np.cross(p, nrm), nrm])
    if len(J) < 30:
        return 0.0, len(J)
    J = np.array(J)
    L = float(np.median(np.linalg.norm(S, axis=1))) or 1.0
    J[:, :3] /= L
    ev = np.linalg.eigvalsh(J.T @ J)
    return float(ev[0] / max(ev[-1], 1e-12)), len(J)


class Segment:
    """구간 하나: 자기 TSDF와 초기 정렬."""

    def __init__(self, sid, voxel, dev):
        self.sid = sid
        self.vbg = o3d.t.geometry.VoxelBlockGrid(attr_names=("tsdf", "weight", "color"),
                                                 attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
                                                 attr_channels=((1), (1), (3)), voxel_size=voxel, block_resolution=16,
                                                 block_count=VBG_INIT_BLOCKS, device=dev)
        self.T0, self.align = None, {}
        self.n_int = self.n_key = self.n_ok = 0
        self.budget_hit = False
        self.first_idx = self.last_idx = None
        self.last_int_t = -1e9


class Mapper:
    def __init__(self, meta, voxel=0.015, refine="none"):
        self.refine = refine
        di = meta["depth_intrinsics"]
        self.K = np.array([[di["fx"], 0, di["ppx"]], [0, di["fy"], di["ppy"]], [0, 0, 1]], float)
        self.Ko = o3d.camera.PinholeCameraIntrinsic(int(di["width"]), int(di["height"]), di["fx"], di["fy"], di["ppx"], di["ppy"])
        self.scale = float(meta["depth_scale_m"])
        self.voxel = voxel
        self.dev = o3c.Device("CPU:0")
        self.Kt = o3c.Tensor(self.K, o3c.float64)
        self.opt = o3d.pipelines.odometry.OdometryOption()
        self.opt.depth_min, self.opt.depth_max, self.opt.depth_diff_max = RANGE_M[0], RANGE_M[1], 0.07

    def frame(self, depth_u16, ir_u8, t, i):
        rg = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(ir_u8), o3d.geometry.Image(depth_u16), depth_scale=1.0 / self.scale,
            depth_trunc=RANGE_M[1], convert_rgb_to_intensity=True)
        pc = o3d.geometry.PointCloud.create_from_rgbd_image(rg, self.Ko)
        return {"rgbd": rg, "pc": pc, "t": t, "i": i, "depth": depth_u16, "ir": ir_u8, "n_valid": int((depth_u16 > 0).sum())}

    @staticmethod
    def key_cache(key):
        if "pc02" not in key:
            k = key["pc"].voxel_down_sample(0.02)
            k.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.06, max_nn=30))
            key["pc02"] = k
        return key["pc02"]

    def register(self, cur, key, init):
        """source = 현재, target = 키프레임. 반환: (T_key_cur 또는 None, 지표). 오도메트리 실패는 바로 실패(예측 자세로 덮지 않음)."""
        ok, T, _ = _odometry(
            cur["rgbd"], key["rgbd"], self.Ko, init, o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), self.opt)
        m = {"odom_ok": bool(ok)}
        if not ok:
            return None, m
        tr = np.asarray(T)
        if self.refine == "icp":
            for vs in (0.04, 0.02, 0.01):
                s = cur["pc"].voxel_down_sample(vs)
                t = key["pc"].voxel_down_sample(vs)
                for c in (s, t):
                    c.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=vs * 3, max_nn=30))
                tr = np.asarray(o3d.pipelines.registration.registration_icp(
                    s, t, vs * 2, tr, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)).transformation)
        tgt = self.key_cache(key)
        src = cur["pc"].voxel_down_sample(0.02)
        ev = o3d.pipelines.registration.evaluate_registration(src, tgt, 0.03, tr)
        ratio, n_corr = p2plane_degeneracy(src, tgt, tr, 0.03)
        m.update(icp_fitness=float(ev.fitness), icp_rmse=float(ev.inlier_rmse), p2plane_eig_ratio=ratio, n_corr=n_corr,
                 geom_degenerate=bool(ratio < DEGEN_RATIO))
        return tr, m

    def integrate(self, seg, cur, T_world_cam):
        if seg.budget_hit:
            return False
        dt = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(cur["depth"]))).to(self.dev)
        ct = o3d.t.geometry.Image(o3c.Tensor(np.ascontiguousarray(np.repeat(cur["ir"][..., None], 3, 2)))).to(self.dev)
        ext = o3c.Tensor(np.linalg.inv(T_world_cam), o3c.float64)  # 월드 → 카메라
        blk = seg.vbg.compute_unique_block_coordinates(dt, self.Kt, ext, 1.0 / self.scale, RANGE_M[1], trunc_voxel_multiplier=3.0)
        seg.vbg.integrate(blk, dt, ct, self.Kt, self.Kt, ext, 1.0 / self.scale, RANGE_M[1], trunc_voxel_multiplier=3.0)
        if seg.vbg.hashmap().size() > VBG_MAX_BLOCKS:
            seg.budget_hit = True
        seg.n_int += 1
        seg.last_int_t = cur["t"]
        return True


def load_frame(folder, i):
    d = cv2.imread(os.path.join(folder, "depth", f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
    ir = cv2.imread(os.path.join(folder, "infrared", f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
    return d, ir


def mat_pow(T, n):
    R = np.eye(4)
    for _ in range(max(0, n)):
        R = R @ T
    return R


def run(folder, out, step=1, voxel=0.015, max_frames=0, refine="none"):
    E, W, _ = check_sequence(folder)
    if E:
        raise SystemExit(f"기록 점검 실패: {E}")
    meta = json.load(open(os.path.join(folder, "meta.json"), encoding="utf-8"))
    if not (meta.get("streams") or {}).get("ir"):
        raise SystemExit("IR 스트림이 없는 기록 — v1 L1은 IR을 밝기로 쓴다")
    rows = list(csv.DictReader(open(os.path.join(folder, "frames.csv"))))
    idx = list(range(0, len(rows), step))[: max_frames or None]
    ts_all = np.array([float(rows[i]["depth_ts_ms"]) / 1000.0 for i in idx])
    dt_nom = float(np.median(np.diff(ts_all))) if len(ts_all) > 1 else 1 / 15
    os.makedirs(out, exist_ok=True)
    mp = Mapper(meta, voxel, refine)
    t_start = time.time()
    segs, log, poses = [], [], {}
    seg = key = T_w_key = None
    T_prev_rel, T_vel = np.eye(4), np.eye(4)
    last_ok_t = lost_since = None
    for i, t in zip(idx, ts_all):
        d, ir = load_frame(folder, i)
        cur = mp.frame(preprocess(d, mp.scale), ir, t, i)
        rec = {"idx": i, "t": t}
        # 새 구간 시작(처음 또는 1 s 넘게 잃은 뒤): 이 프레임이 키프레임, 자기 초기 평면
        if seg is None or (lost_since is not None and t - lost_since > LOST_SPLIT_S):
            if cur["n_valid"] < 0.05 * cur["depth"].size:
                log.append({**rec, "state": "SKIP_START:유효 깊이 부족", "segment": seg.sid if seg else -1})
                continue
            seg = Segment(len(segs), voxel, mp.dev)
            segs.append(seg)
            T0, ainfo = initial_plane(cur["depth"], mp.K, mp.scale)
            seg.align = {"alignment_valid": T0 is not None, **ainfo}
            seg.T0 = T0 if T0 is not None else np.eye(4)
            key, T_w_key = cur, seg.T0
            T_prev_rel, T_vel = np.eye(4), np.eye(4)
            last_ok_t, lost_since = t, None
            seg.n_key += 1
            seg.n_ok += 1
            seg.first_idx = seg.last_idx = i
            poses[i] = T_w_key
            mp.integrate(seg, cur, T_w_key)
            log.append({**rec, "state": "SEGMENT_START" if seg.sid else "KEY_INIT", "segment": seg.sid, "integrated": True,
                        **{f"T{r}{c}": float(T_w_key[r, c]) for r in range(3) for c in range(4)}})
            continue
        dt = t - last_ok_t
        n_steps = max(1, int(round(dt / dt_nom)))
        init = T_prev_rel @ mat_pow(T_vel, n_steps)        # 실제 시간 간격만큼 등속 외삽
        T_rel, m = mp.register(cur, key, init)
        bad = []
        if T_rel is None:
            bad.append("odom_fail")
        else:
            step_T = np.linalg.inv(T_prev_rel) @ T_rel
            if m["icp_fitness"] < ACCEPT["icp_fitness_min"]:
                bad.append("fitness")
            if m["icp_rmse"] > ACCEPT["icp_rmse_max_m"]:
                bad.append("rmse")
            if np.linalg.norm(step_T[:3, 3]) > max(ACCEPT["step_trans_base_m"], ACCEPT["v_max_m_s"] * dt):
                bad.append("step_trans")
            if rot_deg(step_T) > max(ACCEPT["step_rot_base_deg"], ACCEPT["w_max_deg_s"] * dt):
                bad.append("step_rot")
        if bad:
            if lost_since is None:
                lost_since = last_ok_t                     # 잃기 시작한 시각 — 구간 전환은 이 사건당 한 번
            log.append({**rec, "state": "TRACKING_LOST:" + "+".join(bad), "segment": seg.sid, "integrated": False, **m})
            continue
        T_cur_w = T_w_key @ T_rel                          # T_world_cur = T_world_key · T_key_cur
        state = "RECOVERED" if lost_since is not None else ("OK_GEOM_DEGENERATE" if m["geom_degenerate"] else "OK")
        lost_since = None
        if n_steps == 1:                                   # 속도는 한 걸음 간격일 때만 갱신
            T_vel = step_T
        T_prev_rel = T_rel
        if np.linalg.norm(T_rel[:3, 3]) > KEY_TRANS_M or rot_deg(T_rel) > KEY_ROT_DEG:
            key, T_w_key, T_prev_rel = cur, T_cur_w, np.eye(4)
            seg.n_key += 1
            state += "+KEY"
        integrated = False
        if "KEY" in state or t - seg.last_int_t >= INTEGRATE_EVERY_S:
            integrated = mp.integrate(seg, cur, T_cur_w)
        last_ok_t = t
        seg.n_ok += 1
        seg.last_idx = i
        poses[i] = T_cur_w
        log.append({**rec, "state": state, "segment": seg.sid, "integrated": integrated, **m,
                    **{f"T{r}{c}": float(T_cur_w[r, c]) for r in range(3) for c in range(4)}})
    # 출력: 구간마다 지도, 주 구간 = 정렬 유효하고 성공 프레임이 가장 많은 구간 → map_points.ply(L2 기본 입력)
    seg_info = []
    for s in segs:
        pc = s.vbg.extract_point_cloud(weight_threshold=WEIGHT_MIN)
        n_pts = int(pc.point.positions.shape[0]) if "positions" in pc.point else 0
        if n_pts:
            o3d.t.io.write_point_cloud(os.path.join(out, f"map_points_seg{s.sid}.ply"), pc)
            o3d.t.io.write_triangle_mesh(os.path.join(out, f"map_mesh_seg{s.sid}.ply"), s.vbg.extract_triangle_mesh(weight_threshold=WEIGHT_MIN))
        seg_info.append({"segment": s.sid, "frames_ok": s.n_ok, "keyframes": s.n_key, "integrated": s.n_int, "map_points": n_pts,
                         "first_idx": s.first_idx, "last_idx": s.last_idx, "budget_hit": s.budget_hit,
                         "active_blocks": int(s.vbg.hashmap().size()), **s.align})
    good = [x for x in seg_info if x["alignment_valid"] and x["map_points"] > 0]
    main = max(good, key=lambda x: x["frames_ok"]) if good else None
    for ext_ in ("points", "mesh"):
        dst = os.path.join(out, f"map_{ext_}.ply")
        if os.path.exists(dst):
            os.remove(dst)
        if main and os.path.exists(os.path.join(out, f"map_{ext_}_seg{main['segment']}.ply")):
            os.link(os.path.join(out, f"map_{ext_}_seg{main['segment']}.ply"), dst)
    cols = sorted({k for r in log for k in r}, key=lambda k: (k not in ("idx", "t", "state", "segment", "integrated"), k))
    with open(os.path.join(out, "poses.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, cols)
        w.writeheader()
        w.writerows(log)
    states = [r["state"] for r in log]
    last_intended = idx[-1] if idx else None
    ok_idx = [r["idx"] for r in log if not r["state"].startswith(("TRACKING_LOST", "SKIP"))]
    summary = {
        "input": os.path.abspath(folder), "preprocess": PREPROC_VERSION, "frames_used": len(idx),
        "tracked": len(ok_idx), "lost": sum(s.startswith("TRACKING_LOST") for s in states),
        "recovered": sum(s.startswith("RECOVERED") for s in states), "geom_degenerate_frames": sum("GEOM_DEGENERATE" in s for s in states),
        "segments": seg_info, "main_segment": main["segment"] if main else None,
        "alignment_valid": bool(main), "frame_id": "map(초기 평면 정렬)" if main else "unaligned",
        "end_frame_tracked": bool(last_intended in poses),
        "voxel_m": voxel, "refine": refine, "accept": ACCEPT, "weight_min": WEIGHT_MIN,
        "map_points": main["map_points"] if main else 0,
        "sec_per_frame": round((time.time() - t_start) / max(len(idx), 1), 3), "check_warnings": W}
    if main and summary["end_frame_tracked"] and segs[main["segment"]].last_idx == last_intended:
        a, b = poses[main["first_idx"]], poses[last_intended]
        summary["start_end_trans_m"] = float(np.linalg.norm(b[:3, 3] - a[:3, 3]))
        summary["start_end_rot_deg"] = rot_deg(np.linalg.inv(a) @ b)
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
                    help="none: RGB-D 오도메트리 결과를 쓰고 ICP는 맞춤 지표만(기본, 잠정) / icp: 점–평면 ICP로 보정")
    a = ap.parse_args()
    s = run(a.folder, a.out, a.step, a.voxel, a.max_frames, a.refine)
    print(json.dumps({k: v for k, v in s.items() if k != "segments"}, ensure_ascii=False, indent=1))
    for x in s["segments"]:
        print("segment", x)


if __name__ == "__main__":
    main()
