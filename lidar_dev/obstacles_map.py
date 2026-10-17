# obstacles_map.py — L2: map_seq(L1) 지도 점구름 → 2.5D 높이 격자 → 기준값(JSON) 분류 → 장애물 목록(terrain_map JSON)·위에서 본 그림.
# 기준값 정본: 시뮬 담당의 obstacle_criteria JSON(v0.1, profile l2-exp-v1). 값을 코드에 옮겨 적지 않고 파일을 읽는다.
# v1 한계: 관측 상태는 '지도 표면 점이 있는 칸 = 관측(OCCUPIED/표면), 없는 칸 = UNKNOWN'이다(광선으로 빈 공간을 지우는 FREE는 아직 없음).
#          통로 폭·머리 위 높이는 2.5D로 표현하지 못해 계산하지 않는다(목록에 미지원으로 남김).
# 사용: python obstacles_map.py <map_seq 출력 폴더> <기준값 json> [--cell 0.05]
import argparse, json, math, os, sys
import numpy as np
import cv2

LEVEL = {"unknown": 0, "pass": 1, "caution": 2, "block": 3}
NAME = {v: k for k, v in LEVEL.items()}
COLOR_BGR = {0: (158, 158, 158), 1: (50, 125, 46), 2: (37, 168, 249), 3: (40, 40, 198)}


def classify(value, rule):
    """기준값 규칙 하나로 값 → 단계(1 pass, 2 caution, 3 block). 반개구간(JSON interval_rule)."""
    if rule["kind"] == "higher_is_worse":
        return np.where(value >= rule["block_from"], 3, np.where(value >= rule["caution_from"], 2, 1))
    return np.where(value < rule["block_below"], 3, np.where(value < rule["pass_from"], 2, 1))


def build_grid(P, cell, min_pts=3):
    """점(N×3, 지도 좌표 z 위) → 칸별 지지면 높이(하위 분위)·윗면(상위 분위)·점 수."""
    x0, y0 = P[:, 0].min() - cell, P[:, 1].min() - cell
    ix = ((P[:, 0] - x0) / cell).astype(int)
    iy = ((P[:, 1] - y0) / cell).astype(int)
    nx, ny = ix.max() + 2, iy.max() + 2
    key = iy * nx + ix
    order = np.argsort(key)
    key, z = key[order], P[order, 2]
    uniq, start, count = np.unique(key, return_index=True, return_counts=True)
    top = np.full(ny * nx, np.nan)
    low = np.full(ny * nx, np.nan)
    cnt = np.zeros(ny * nx, int)
    for k, s, c in zip(uniq, start, count):
        if c < min_pts:
            continue
        zz = z[s:s + c]
        top[k] = np.percentile(zz, 90)   # 강건한 윗면(튀는 점 무시)
        low[k] = np.percentile(zz, 10)
        cnt[k] = c
    return {"x0": x0, "y0": y0, "nx": nx, "ny": ny, "cell": cell, "top": top.reshape(ny, nx), "low": low.reshape(ny, nx),
            "cnt": cnt.reshape(ny, nx)}


def local_plane(h, win):
    """win×win 창 평면 맞춤 → (기울기 °, 잔차 범위 m). 관측 칸이 창의 절반 미만이면 NaN."""
    ny, nx = h.shape
    slope = np.full(h.shape, np.nan)
    rough = np.full(h.shape, np.nan)
    r = win // 2
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    for j in range(r, ny - r):
        for i in range(r, nx - r):
            if np.isnan(h[j, i]):
                continue
            w = h[j - r:j + r + 1, i - r:i + r + 1]
            m = ~np.isnan(w)
            if m.sum() < 0.5 * w.size:
                continue
            A = np.stack([xx[m], yy[m], np.ones(m.sum())], 1)
            coef, *_ = np.linalg.lstsq(A, w[m], rcond=None)
            res = w[m] - A @ coef
            slope[j, i] = math.degrees(math.atan(math.hypot(coef[0], coef[1]) / CELL_FOR_SLOPE[0]))
            rough[j, i] = float(res.max() - res.min())
    return slope, rough


CELL_FOR_SLOPE = [0.05]
R_FLOOR_H = 0.03   # 지지면 구역을 잇는 이웃 높이 차 상한(인수받은 코드 FLOOR_H와 같은 값)
R_WALL_H = 0.5     # 둘레보다 이만큼 솟으면 벽·큰 물체(기준값 문서 표시 구분)


def smooth_patches(h, tol):
    """이웃(4-연결) 높이 차 < tol로 이어진 관측 칸을 구역 번호로. 관측 안 된 칸은 −1."""
    ny, nx = h.shape
    parent = np.arange(ny * nx)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    ok = ~np.isnan(h)
    for dy, dx in ((0, 1), (1, 0)):
        a = ok[: ny - dy, : nx - dx] & ok[dy:, dx:]
        d = np.abs(h[dy:, dx:] - h[: ny - dy, : nx - dx])
        js, is_ = np.nonzero(a & (d < tol))
        for j, i in zip(js, is_):
            ra, rb = find(j * nx + i), find((j + dy) * nx + i + dx)
            if ra != rb:
                parent[ra] = rb
    lab = np.array([find(k) for k in range(ny * nx)]).reshape(ny, nx)
    lab[~ok] = -1
    return lab


def run(map_dir, criteria_path, cell=0.05, max_range_m=4.0, allow_unaligned=False):
    import open3d as o3d
    crit = json.load(open(criteria_path, encoding="utf-8"))
    R = crit["rules"]
    P = np.asarray(o3d.io.read_point_cloud(os.path.join(map_dir, "map_points.ply")).points)
    summ = json.load(open(os.path.join(map_dir, "summary.json"), encoding="utf-8"))
    if not summ.get("alignment_valid") and not allow_unaligned:   # 초기 평면 정렬이 없으면 z가 높이가 아니다
        raise SystemExit(f"지도 좌표 정렬이 유효하지 않음(frame_id={summ.get('frame_id')}) — L2는 높이 지도로 쓸 수 없다")
    poses = np.load(os.path.join(map_dir, "poses_T_world_cam.npy"))
    cams = np.array([T[:3, 3] for T in poses if not np.isnan(T).any()])
    P = P[P[:, 2] < 2.0]                       # 천장 등 로버와 무관한 높은 점은 2.5D 지지면 계산에서 뺀다
    CELL_FOR_SLOPE[0] = cell
    G = build_grid(P, cell)
    top, cnt = G["top"], G["cnt"]
    observed = cnt > 0
    # 카메라 궤적에서 신뢰 범위(4 m) 밖 칸은 미확인
    yy, xx = np.mgrid[0:G["ny"], 0:G["nx"]]
    cx, cy = G["x0"] + (xx + 0.5) * cell, G["y0"] + (yy + 0.5) * cell
    dmin = np.full(top.shape, np.inf)
    for c in cams[:: max(1, len(cams) // 30)]:
        dmin = np.minimum(dmin, np.hypot(cx - c[0], cy - c[1]))
    observed &= dmin <= max_range_m
    h = np.where(observed, top, np.nan)
    low = np.where(observed, G["low"], np.nan)
    rng = h - low
    edge = observed & (rng > R_FLOOR_H)          # 모서리 칸: 한 칸에 바닥·물체 점이 섞임 → 구역 연결에서 뺀다(5 cm 상자가 바닥과 이어지는 것 방지)
    # 지지면 구역: 이웃 높이 차 < 바닥 기준(3 cm)으로 이어진 칸을 한 구역으로. 가장 큰 구역 = 지면
    patch = smooth_patches(np.where(edge, np.nan, h), R_FLOOR_H)
    ids, areas = np.unique(patch[patch >= 0], return_counts=True)
    ground_id = int(ids[np.argmax(areas)]) if len(ids) else -1
    level = np.ones(h.shape, int)
    reason = np.full(h.shape, "", dtype=object)
    step_of = np.full(h.shape, np.nan)
    gmask = patch == ground_id
    # 지면 칸: 경사·거칠기 — 지면 구역 칸만으로 평면 맞춤(물체가 창에 섞여 거칠기로 잘못 걸리는 것 방지)
    win = max(3, int(round(0.30 / cell)) | 1)   # 30 cm 창(홀수 칸)
    slope, rough = local_plane(np.where(gmask, h, np.nan), win)
    lv_slope = np.where(np.isnan(slope), 1, classify(np.nan_to_num(slope), R["slope_deg"]))
    lv_rough = np.where(np.isnan(rough), 1, classify(np.nan_to_num(rough), R["roughness_m"]))
    level[gmask] = np.maximum(lv_slope, lv_rough)[gmask]
    reason[gmask & (lv_slope >= lv_rough) & (lv_slope > 1)] = "slope"
    reason[gmask & (lv_rough > lv_slope)] = "roughness"
    # 지면이 아닌 구역: 둘레(구역 밖 관측 칸)와의 높이 차로 턱·단·내려가기·벽
    kernel = np.ones((3, 3), np.uint8)
    ring_h = np.where(edge, low, h)              # 둘레 비교 높이: 모서리 칸은 아래쪽(바닥 쪽) 점을 쓴다
    for pid in ids:
        if pid == ground_id:
            continue
        m = patch == pid
        ring = (cv2.dilate(m.astype(np.uint8), kernel) > 0) & ~m & ~np.isnan(ring_h)
        if not ring.any():
            continue
        diff = float(np.nanmedian(h[m]) - np.nanmedian(ring_h[ring]))
        if diff >= R_WALL_H:
            lv, why = 3, "wall"
        elif diff > 0:
            lv, why = int(classify(np.array(diff), R["step_up_m"])), "step_up"
        else:
            lv, why = int(classify(np.array(-diff), R["step_down_m"])), "step_down"
        level[m] = lv
        reason[m] = why
        step_of[m] = diff
    # 모서리 칸: 칸 안 높이 범위를 턱 높이로 보고 분류(물체 윤곽이 장애물에 포함되게), 벽 높이면 벽
    lv_edge = np.where(rng >= R_WALL_H, 3, classify(np.nan_to_num(rng), R["step_up_m"]))
    level[edge] = lv_edge[edge]
    reason[edge] = np.where(rng[edge] >= R_WALL_H, "wall", "edge")
    step_of[edge] = rng[edge]
    level = np.where(observed, level, LEVEL["unknown"])
    # 장애물 목록: 주의·불가 칸 8-연결 영역
    obstacles = []
    mask = (level >= LEVEL["caution"]).astype(np.uint8)   # 주의·불가 칸이 이어지면 한 물체(모서리·몸통이 갈라지지 않게), 단계는 가장 나쁜 것
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    for c in range(1, n):
        area_cells = stats[c, cv2.CC_STAT_AREA]
        if area_cells < 2:
            continue
        m = lab == c
        lv = int(level[m].max())
        cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt_ = max(cs, key=cv2.contourArea)
        poly = cv2.approxPolyDP(cnt_, 1.0, True)[:16, 0, :]
        poly_xy = [[round(float(G["x0"] + (px + 0.5) * cell), 3), round(float(G["y0"] + (py + 0.5) * cell), 3)] for px, py in poly]
        rs, rc = np.unique(reason[m].astype(str), return_counts=True)
        body = m & (level == lv)
        obstacles.append({"id": len(obstacles), "level": NAME[lv], "area_m2": round(float(area_cells * cell * cell), 4),
                          "centroid_xy_m": [round(float(cx[m].mean()), 3), round(float(cy[m].mean()), 3)],
                          "step_m": round(float(np.nanmax(step_of[body])), 3) if np.any(~np.isnan(step_of[body])) else None,
                          "top_z_m": round(float(np.nanmedian(h[body])), 3),
                          "reasons": {str(a): int(b) for a, b in zip(rs, rc) if a}, "poly_xy_m": poly_xy})
    counts = {NAME[v]: int((level == v).sum()) for v in LEVEL.values()}
    out = {"type": "terrain_map", "schema_version": 1, "map_version": 1, "criteria_version": crit.get("criteria_version"),
           "profile": crit.get("profile"), "frame": "map(z 위, 바닥 z = 0, 원점 = 시작 카메라 바로 아래, x = 시작 시 카메라 앞)",
           "cell_m": cell, "grid": {"x0": float(G["x0"]), "y0": float(G["y0"]), "nx": int(G["nx"]), "ny": int(G["ny"])}, "cell_counts": counts,
           "observation_model": "v1: 표면 점이 있는 칸 = 관측, 없으면 unknown, 카메라 궤적에서 4 m 밖 unknown(광선 FREE 미구현)",
           "unsupported": ["passage_width_m", "overhead_m"] + crit.get("unsupported_v0", []),
           "source_map": {"tracked": summ.get("tracked"), "lost": summ.get("lost"), "frame_id": summ.get("frame_id"),
                          "main_segment": summ.get("main_segment"), "preprocess": summ.get("preprocess")},
           "obstacles": obstacles}
    json.dump(out, open(os.path.join(map_dir, "terrain_map.json"), "w"), ensure_ascii=False, indent=1)
    np.savez_compressed(os.path.join(map_dir, "terrain_grid.npz"), level=level, top=top, patch=patch, step=step_of, slope=slope, rough=rough,
                        x0=G["x0"], y0=G["y0"], cell=cell)
    img = np.zeros(level.shape + (3,), np.uint8)
    for v, col in COLOR_BGR.items():
        img[level == v] = col
    img = cv2.resize(img[::-1], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)   # 위가 +y(왼쪽)인 위에서 본 그림
    cv2.imwrite(os.path.join(map_dir, "terrain_top.png"), img)
    return out


def main():
    ap = argparse.ArgumentParser(description="L1 지도 → 장애물·지형 분류(L2)")
    ap.add_argument("map_dir")
    ap.add_argument("criteria")
    ap.add_argument("--cell", type=float, default=0.05)
    a = ap.parse_args()
    out = run(a.map_dir, a.criteria, a.cell)
    print(json.dumps({k: out[k] for k in ("cell_counts", "criteria_version")}, ensure_ascii=False))
    for o in out["obstacles"]:
        print(o["id"], o["level"], o["centroid_xy_m"], "단차", o["step_m"], "윗면", o["top_z_m"], o["reasons"], "면적", o["area_m2"])


if __name__ == "__main__":
    main()
