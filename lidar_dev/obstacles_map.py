# obstacles_map.py — L2: map_seq(L1) 지도 점구름 → 2.5D 높이 격자 → 기준값(JSON) 분류 → 장애물 목록(terrain_map JSON)·위에서 본 그림.
# 기준값 정본: 시뮬 담당의 obstacle_criteria JSON(v0.1, profile l2-exp-v1). 값을 코드에 옮겨 적지 않고 파일을 읽는다.
# v2(10-18, 교차검토 4차 반영):
#   - 기준 바닥 = L1이 검증한 초기 평면(z = 0) 근처 칸을 품은 지지면 구역. 면적이 가장 큰 구역을 바닥으로 고르지 않는다.
#     기준 바닥이 모자라면 분류를 보류한다(전부 unknown).
#   - 단차는 맞닿은 경계(직접 이웃 칸, 모서리 칸 양쪽)의 국소 높이 차로 재고 방향을 지킨다: 기준 바닥에서 바깥으로 갈 때 오르기면 step_up,
#     내려가기면 step_down. 구역 단계 = 기준 바닥에서 그 구역까지의 길 가운데 가장 나쁜 경계가 가장 덜 나쁜 길의 단계(병목).
#     기준 바닥과 경계로 이어지지 않는 구역은 unknown.
#   - 경사·거칠기는 모든 구역에서 잰다. 창 전체가 한 평면이면(잔차 < planar_rough) 구역과 무관하게 그 평면으로 잰다(급경사가 연결 규칙에 끊겨 새는 것 방지).
#   - 평가하지 못한 칸(지지 부족·퇴화)은 통과가 아니라 unknown이다. 단차·거칠기에는 측정 여유를 더해 경계 근처 값은 나쁜 쪽으로 보낸다.
#   - 장애물 목록은 단일 칸도 남긴다. 다각형은 칸 모서리의 볼록 껍질이다(위험 영역을 줄이지 않는다).
# 관측 모델 한계: 표면 점이 있는 칸 = 관측, 없는 칸 = unknown(광선으로 빈 공간을 지우는 FREE는 없다). 가림 띠 너머의 낮은 바닥은
#   기준 바닥과 경계를 맺지 못해 unknown으로 남는다. 칸 안 높이 범위가 연결 상한을 넘는 아주 급한 면(약 40° 이상)은 모서리 칸이 되어 unknown/불가다.
#   통로 폭·머리 위 높이·틈 폭은 계산하지 않는다(unsupported). pass는 평가한 요소(단차·경사·거칠기)에 대해서만의 판정이다.
# 사용: python obstacles_map.py <map_seq 출력 폴더> <기준값 json> [--cell 0.05]
import argparse, csv, heapq, itertools, json, math, os
from collections import defaultdict
import numpy as np
import cv2

LEVEL = {"unknown": 0, "pass": 1, "caution": 2, "block": 3}
NAME = {v: k for k, v in LEVEL.items()}
COLOR_BGR = {0: (158, 158, 158), 1: (50, 125, 46), 2: (37, 168, 249), 3: (40, 40, 198)}

# 알고리즘 설정 — 정책 기준값(JSON)과 별개다. 출력 JSON "algorithm"에 그대로 남긴다
ALG = {"min_pts": 6,             # 칸 통계에 쓰는 최소 지도 점 수(TSDF 표면 점: 1.5 cm 복셀이면 수평 칸 하나에 약 11개)
       "link_tol_m": 0.03,       # 같은 지지면으로 잇는 이웃 높이 차 상한. 칸 안 점이 주 지지면에서 이 값의 절반 넘게 벗어나면(상·하위 10%) 모서리 칸
       "wall_m": 0.5,            # 이 이상 오르는 경계는 벽(불가)
       "meas_margin_m": 0.01,    # 단차·거칠기 값에 더하는 측정 여유
       "slope_win_m": 0.30,      # 경사·거칠기 창(기준값: 30 cm 이상 창)
       "planar_rough_m": 0.02,   # 창 전체 평면 잔차 범위가 이보다 작으면 구역과 무관하게 그 평면으로 경사를 잰다
       "min_support_frac": 0.25,  # 창에서 평면 맞춤에 필요한 칸 비율(가로·세로 각 3칸 이상은 별도 검사)
       "edge_win": 2,            # 모서리 칸 양쪽 구역을 찾는 반경(칸)
       "ref_seed_h_m": 0.03,     # 기준 바닥 씨앗: |z| ≤ 이 값인 칸(z = 0은 L1 초기 평면)
       "ref_min_seed": 12,       # 구역이 기준 바닥이 되려면 씨앗 칸이 이만큼 이상이거나 구역의 60% 이상
       "ref_min_cells": 40,      # 기준 바닥 칸 합이 이보다 적으면 분류 보류
       "z_max_m": 2.0,           # 이보다 높은 점(천장 등)은 2.5D 계산에서 뺀다
       "range_m": 4.0}           # 주 구간 카메라 궤적에서 이보다 먼 칸은 unknown


def classify(value, rule):
    """기준값 규칙 하나로 값 → 단계(1 pass, 2 caution, 3 block). 반개구간(JSON interval_rule)."""
    if rule["kind"] == "higher_is_worse":
        return np.where(value >= rule["block_from"], 3, np.where(value >= rule["caution_from"], 2, 1))
    return np.where(value < rule["block_below"], 3, np.where(value < rule["pass_from"], 2, 1))


def combine(a, b):
    """단계 합치기: 불가가 하나라도 있으면 불가, 아니면 미확인이 하나라도 있으면 미확인, 아니면 더 나쁜 쪽."""
    a, b = np.asarray(a), np.asarray(b)
    return np.where((a == 3) | (b == 3), 3, np.where((a == 0) | (b == 0), 0, np.maximum(a, b)))


def build_grid(P, cell, min_pts):
    """점(N×3, 지도 좌표 z 위) → 칸별 윗면(상위 90%)·아랫면(하위 10%)·주 지지면(점이 가장 많은 1 cm 높이 구간의 중앙값)·점 수.
    점이 min_pts 미만인 칸은 높이 NaN. 주 지지면은 수직 면 점이 섞인 칸을 가려내는 데 쓴다(윗면만 보면 중간 높이 칸이 생겨 바닥과 윗면이 이어진다)."""
    x0, y0 = P[:, 0].min() - cell, P[:, 1].min() - cell
    ix = ((P[:, 0] - x0) / cell).astype(int)
    iy = ((P[:, 1] - y0) / cell).astype(int)
    nx, ny = int(ix.max()) + 2, int(iy.max()) + 2
    key = iy * nx + ix
    order = np.argsort(key, kind="stable")
    key, z = key[order], P[order, 2]
    uniq, start, count = np.unique(key, return_index=True, return_counts=True)
    top = np.full(ny * nx, np.nan)
    low = np.full(ny * nx, np.nan)
    sup = np.full(ny * nx, np.nan)
    cnt = np.zeros(ny * nx, int)
    cnt[uniq] = count
    for k, s, c in zip(uniq, start, count):
        if c >= min_pts:
            zz = z[s:s + c]
            top[k], low[k] = np.percentile(zz, 90), np.percentile(zz, 10)
            b = np.floor((zz - zz.min()) / 0.01).astype(int)
            kb = int(np.argmax(np.bincount(b)))
            sup[k] = float(np.median(zz[b == kb]))
    return {"x0": float(x0), "y0": float(y0), "nx": nx, "ny": ny, "cell": cell, "top": top.reshape(ny, nx), "low": low.reshape(ny, nx),
            "sup": sup.reshape(ny, nx), "cnt": cnt.reshape(ny, nx)}


def smooth_patches(h, tol):
    """이웃(4-연결) 높이 차 < tol로 이어진 칸을 구역 번호(0부터)로. 높이 NaN 칸은 −1."""
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
        for j, i in zip(*np.nonzero(a & (d < tol))):
            ra, rb = find(j * nx + i), find((j + dy) * nx + i + dx)
            if ra != rb:
                parent[ra] = rb
    roots = np.array([find(k) for k in range(ny * nx)])
    roots[~ok.ravel()] = -1
    _, lab = np.unique(roots, return_inverse=True)
    lab = lab.reshape(ny, nx) - (1 if (~ok).any() else 0)   # −1(관측 없음)이 가장 작은 값이라 0번이 된다 → 한 칸씩 당긴다
    lab[~ok] = -1
    return lab


def fit_plane(xx, yy, w, cell):
    """창 안 칸(칸 단위 좌표 xx, yy, 높이 w m)에 평면 맞춤 → (기울기 °, 잔차 범위 m). 가로·세로 3칸 미만이면 퇴화 → None."""
    if len(np.unique(xx)) < 3 or len(np.unique(yy)) < 3:
        return None
    A = np.stack([xx, yy, np.ones(len(xx))], 1).astype(float)
    coef, *_ = np.linalg.lstsq(A, w, rcond=None)
    res = w - A @ coef
    return math.degrees(math.atan(math.hypot(coef[0], coef[1]) / cell)), float(res.max() - res.min())


def surface(h, patch, cell, alg, extra=None):
    """칸별 경사(°)·거칠기(m). 창 전체 평면이 맞으면 그것을, 아니면 같은 구역 칸만으로. 지지 부족·퇴화면 NaN(→ unknown).
    extra: 높이가 없는 칸(모서리 칸) 가운데 창 전체 평면만으로 평가할 칸 — 평면 위에 있으면 진짜 단차가 아니다."""
    win = max(3, int(round(alg["slope_win_m"] / cell)) | 1)
    r = win // 2
    need = max(6, int(math.ceil(alg["min_support_frac"] * win * win)))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    hp = np.pad(h, r, constant_values=np.nan)
    pp = np.pad(patch, r, constant_values=-1)
    slope = np.full(h.shape, np.nan)
    rough = np.full(h.shape, np.nan)
    src = np.zeros(h.shape, np.int8)   # 1 = 창 전체 평면, 2 = 같은 구역만
    todo = ~np.isnan(h) | (np.zeros(h.shape, bool) if extra is None else extra)
    for j, i in zip(*np.nonzero(todo)):
        w = hp[j:j + win, i:i + win]
        m = ~np.isnan(w)
        if m.sum() >= need:
            f = fit_plane(xx[m], yy[m], w[m], cell)
            if f is not None and f[1] < alg["planar_rough_m"]:
                slope[j, i], rough[j, i], src[j, i] = f[0], f[1], 1
                continue
        if patch[j, i] < 0:
            continue
        m2 = m & (pp[j:j + win, i:i + win] == patch[j, i])
        if m2.sum() >= need:
            f = fit_plane(xx[m2], yy[m2], w[m2], cell)
            if f is not None:
                slope[j, i], rough[j, i], src[j, i] = f[0], f[1], 2
    return slope, rough, src


def trans_level(d, R, alg):
    """경계 높이 차 d(m, 가는 쪽 − 오는 쪽) → (단계, 이유). 측정 여유를 더한다."""
    if d >= alg["wall_m"]:
        return 3, "wall"
    if d > 0:
        return int(classify(np.array(d + alg["meas_margin_m"]), R["step_up_m"])), "step_up"
    return int(classify(np.array(-d + alg["meas_margin_m"]), R["step_down_m"])), "step_down"


def classify_points(P, crit, cams_xy, cell=0.05, alg=None):
    """핵심: 지도 점(z 위, 바닥 z = 0) → 칸 단계·장애물. cams_xy = 주 구간 카메라 위치(N×2, 관측 범위 판정용)."""
    alg = {**ALG, **(alg or {})}
    R = crit["rules"]
    P = P[P[:, 2] < alg["z_max_m"]]
    G = build_grid(P, cell, alg["min_pts"])
    top, low = G["top"], G["low"]
    yy, xx = np.mgrid[0:G["ny"], 0:G["nx"]]
    cx, cy = G["x0"] + (xx + 0.5) * cell, G["y0"] + (yy + 0.5) * cell
    observed = ~np.isnan(top)
    if len(cams_xy):
        dmin = np.full(top.shape, np.inf)
        for c in cams_xy[:: max(1, len(cams_xy) // 30)]:
            dmin = np.minimum(dmin, np.hypot(cx - c[0], cy - c[1]))
        observed &= dmin <= alg["range_m"]
    else:
        observed[:] = False
    rng = np.where(observed, top - low, np.nan)
    spread = np.where(observed, np.maximum(top - G["sup"], G["sup"] - low), np.nan)
    edge = observed & (spread > alg["link_tol_m"] / 2)   # 모서리 칸: 주 지지면에서 연결 상한의 절반 넘게 벗어난 점이 10% 넘게 섞임(아래·위 면, 수직 면)
    h = np.where(observed & ~edge, top, np.nan)           # 지지면 높이(모서리 칸 제외)
    patch = smooth_patches(h, alg["link_tol_m"])
    npatch = int(patch.max()) + 1
    # 기준 바닥: z = 0 근처 씨앗 칸을 충분히 품은 구역
    seed = ~np.isnan(h) & (np.abs(h) <= alg["ref_seed_h_m"])
    n_cells = np.bincount(patch[patch >= 0], minlength=npatch)
    n_seed = np.bincount(patch[seed], minlength=npatch)
    ref = (n_seed >= alg["ref_min_seed"]) | ((n_cells >= 4) & (n_seed >= 0.6 * n_cells))
    ref_cells = int(n_cells[ref].sum())
    level = np.zeros(top.shape, int)
    reason = np.full(top.shape, "", dtype=object)
    step = np.full(top.shape, np.nan)
    slope, rough, src = surface(h, patch, cell, alg, extra=edge)
    info = {"ref_cells": ref_cells, "ref_patches": int(ref.sum()), "patches": npatch, "edge_cells": int(edge.sum()),
            "observed_cells": int(observed.sum()), "withheld": None}
    if ref_cells < alg["ref_min_cells"]:
        info["withheld"] = f"기준 바닥(z = 0 근처) 칸 {ref_cells} < {alg['ref_min_cells']} — 분류 보류, 전부 unknown"
    else:
        # 경계 표본: (구역 p, 구역 q) → q 높이 − p 높이
        samples = defaultdict(list)

        def add(p, q, d):
            if p < q:
                samples[(p, q)].append(d)
            elif q < p:
                samples[(q, p)].append(-d)
        ny, nx = h.shape
        for dy, dx in ((0, 1), (1, 0)):
            a, b = patch[: ny - dy, : nx - dx], patch[dy:, dx:]
            ok = (a >= 0) & (b >= 0) & (a != b)
            for pa, pb, d in zip(a[ok], b[ok], (h[dy:, dx:] - h[: ny - dy, : nx - dx])[ok]):
                add(int(pa), int(pb), float(d))
        er = alg["edge_win"]
        pp = np.pad(patch, er, constant_values=-1)
        hp = np.pad(h, er, constant_values=np.nan)
        edge_meds = {}
        for j, i in zip(*np.nonzero(edge)):
            w, hw = pp[j:j + 2 * er + 1, i:i + 2 * er + 1], hp[j:j + 2 * er + 1, i:i + 2 * er + 1]
            meds = {int(p): float(np.median(hw[w == p])) for p in np.unique(w[w >= 0])}
            edge_meds[(j, i)] = meds
            for p, q in itertools.combinations(meds, 2):
                add(p, q, meds[q] - meds[p])
        adj = defaultdict(list)
        for (p, q), ds in samples.items():
            d = float(np.median(ds))
            adj[p].append((q, d))
            adj[q].append((p, -d))
        # 병목 전파: 기준 바닥에서 바깥으로
        plev = {int(p): 1 for p in np.nonzero(ref)[0]}
        pwhy = {p: ("ref", np.nan) for p in plev}
        heap = [(1, p) for p in plev]
        heapq.heapify(heap)
        while heap:
            L, p = heapq.heappop(heap)
            if L > plev[p]:
                continue
            for q, d in adj[p]:
                t, why = trans_level(d, R, alg)
                n = max(L, t)
                if n < plev.get(q, 99):
                    plev[q] = n
                    pwhy[q] = (why, d) if t >= L else pwhy[p]
                    heapq.heappush(heap, (n, q))
        # 지지면 칸: 구역 단계 ∧ 표면(경사·거칠기)
        lev_p = np.zeros(top.shape, int)
        for p, L in plev.items():
            m = patch == p
            lev_p[m] = L
            if pwhy[p][0] != "ref":
                reason[m] = pwhy[p][0]
                step[m] = pwhy[p][1]
        lv_slope = np.where(np.isnan(slope), 0, classify(np.nan_to_num(slope), R["slope_deg"]))
        lv_rough = np.where(np.isnan(rough), 0, classify(np.nan_to_num(rough) + alg["meas_margin_m"], R["roughness_m"]))
        surf = np.where(np.isnan(slope), 0, np.maximum(lv_slope, lv_rough))
        sm = patch >= 0
        # 표면을 평가 못 한 칸: 단차로 이미 주의·불가인 구역은 그 단계를 지키고, 통과 구역이면 unknown(통과 확정 보류)
        lv_cell = np.where(surf == 0, np.where(lev_p >= 2, lev_p, 0), combine(lev_p, surf))
        level[sm] = lv_cell[sm]
        worse = sm & (surf > lev_p) & (surf > 1)
        reason[worse & (lv_slope >= lv_rough)] = "slope"
        reason[worse & (lv_rough > lv_slope)] = "roughness"
        reason[sm & (lev_p == 0)] = "no_support_relation"
        reason[sm & (lev_p == 1) & (surf == 0)] = "surface_unevaluated"
        # 모서리 칸
        bf = R["step_up_m"]["block_from"]
        for (j, i), meds in edge_meds.items():
            r_ = float(rng[j, i])
            reached = [p for p in meds if plev.get(p, 0) > 0]
            if len(meds) >= 2 and reached:
                lv = max(plev[p] for p in reached)
                pairs = [(meds[q] - meds[p]) for p, q in itertools.permutations(meds, 2)]
                d, why = max(pairs, key=abs), "edge"
            elif len(meds) == 1 and reached:
                p = reached[0]
                d = r_ if abs(meds[p] - low[j, i]) <= abs(meds[p] - top[j, i]) else -r_   # 구역이 칸 아래쪽이면 오르는 모서리
                t, why = trans_level(d, R, alg)
                lv, why = max(plev[p], t), ("edge_" + why if t >= plev[p] else "edge")
            else:
                d = r_
                lv, why = (3, "edge_tall") if r_ + alg["meas_margin_m"] >= bf else (0, "edge_unrelated")
            if src[j, i] == 1 and lv > 0:      # 주변 창 전체가 한 평면 → 단차가 아니라 그 평면의 경사로
                lv_s = int(max(classify(np.array(slope[j, i]), R["slope_deg"]), classify(np.array(rough[j, i] + alg["meas_margin_m"]), R["roughness_m"])))
                if lv_s >= lv:
                    lv, why = lv_s, "slope"
            level[j, i], reason[j, i], step[j, i] = lv, why, d
    level[~observed] = LEVEL["unknown"]
    reason[~observed] = ""
    # 장애물 목록: 주의·불가 칸 8-연결 영역(단일 칸 포함), 단계는 가장 나쁜 것
    mask = (level >= LEVEL["caution"]).astype(np.uint8)
    n, lab = cv2.connectedComponents(mask, connectivity=8)
    obstacles = []
    obj_id = np.full(top.shape, -1, int)
    for c in range(1, n):
        m = lab == c
        obj_id[m] = len(obstacles)
        lv = int(level[m].max())
        js, is_ = np.nonzero(m)
        corners = np.concatenate([np.stack([G["x0"] + (is_ + a) * cell, G["y0"] + (js + b) * cell], 1)
                                  for a in (0, 1) for b in (0, 1)]).astype(np.float32)
        hull = cv2.convexHull(corners)[:, 0, :]
        body = m & (level == lv)
        sp = body & ~edge & ~np.isnan(step)
        se = body & edge & ~np.isnan(step)
        st = float(np.median(step[sp])) if sp.any() else (float(step[se][np.argmax(np.abs(step[se]))]) if se.any() else None)
        tz = h[body & ~edge]
        rs, rc = np.unique(reason[m].astype(str), return_counts=True)
        obstacles.append({"id": len(obstacles), "level": NAME[lv], "cells": int(m.sum()), "area_m2": round(float(m.sum() * cell * cell), 4),
                          "centroid_xy_m": [round(float(cx[m].mean()), 3), round(float(cy[m].mean()), 3)],
                          "step_m": None if st is None else round(st, 3),
                          "direction": None if st is None else ("up" if st > 0 else "down"),
                          "top_z_m": round(float(np.median(tz)), 3) if tz.size else round(float(np.nanmax(top[body])), 3),
                          "reasons": {str(a): int(b) for a, b in zip(rs, rc) if a},
                          "poly_xy_m": [[round(float(x), 3), round(float(y), 3)] for x, y in hull]})
    grids = {"level": level, "reason": reason, "top": top, "low": low, "sup": G["sup"], "cnt": G["cnt"], "patch": patch, "edge": edge, "observed": observed,
             "step": step, "slope": slope, "rough": rough, "surface_src": src, "obstacle_id": obj_id,
             "ref": np.isin(patch, np.nonzero(ref)[0]) & (patch >= 0)}
    return {"grid": {"x0": G["x0"], "y0": G["y0"], "nx": G["nx"], "ny": G["ny"]}, "info": info, "obstacles": obstacles,
            "cell_counts": {NAME[v]: int((level == v).sum()) for v in LEVEL.values()}, "alg": alg}, grids


def main_segment_cams(map_dir, main_segment):
    """poses.csv에서 주 구간의 수용된 자세(T 열이 있는 행)만 카메라 XY로."""
    out = []
    p = os.path.join(map_dir, "poses.csv")
    if not os.path.exists(p) or main_segment is None:
        return np.zeros((0, 2))
    for r in csv.DictReader(open(p)):
        if str(r.get("segment")) == str(main_segment) and r.get("T03") not in (None, ""):
            out.append([float(r["T03"]), float(r["T13"])])
    return np.array(out).reshape(-1, 2)


def run(map_dir, criteria_path, cell=0.05, allow_unaligned=False, alg=None):
    import open3d as o3d
    crit = json.load(open(criteria_path, encoding="utf-8"))
    summ = json.load(open(os.path.join(map_dir, "summary.json"), encoding="utf-8"))
    aligned = bool(summ.get("alignment_valid"))
    if not aligned and not allow_unaligned:   # 초기 평면 정렬이 없으면 z가 높이가 아니다
        raise SystemExit(f"지도 좌표 정렬이 유효하지 않음(frame_id={summ.get('frame_id')}) — L2는 높이 지도로 쓸 수 없다")
    P = np.asarray(o3d.io.read_point_cloud(os.path.join(map_dir, "map_points.ply")).points)
    cams = main_segment_cams(map_dir, summ.get("main_segment"))
    core, grids = classify_points(P, crit, cams, cell, alg)
    out = {"type": "terrain_map", "schema_version": 2, "map_version": 1, "criteria_version": crit.get("criteria_version"),
           "profile": crit.get("profile"), "alignment_valid": aligned,
           "frame": "map(z 위, 바닥 z = 0, 원점 = 시작 카메라 바로 아래, x = 시작 시 카메라 앞)" if aligned else "unaligned(높이 아님 — 시험용)",
           "cell_m": cell, "grid": core["grid"], "cell_counts": core["cell_counts"], "classification": core["info"],
           "grid_file": {"path": "terrain_grid.npz", "index": "배열[iy, ix], 칸 중심 x = x0 + (ix + 0.5)·cell, y = y0 + (iy + 0.5)·cell",
                         "layers": sorted(grids)},
           "semantics": "pass = 평가한 요소(단차·경사·거칠기)만의 판정. 단차 방향은 기준 바닥에서 바깥으로. unknown은 경로 허가가 아니다",
           "observation_model": "v2: 표면 점 ≥ min_pts인 칸 = 관측, 아니면 unknown. 주 구간 카메라 궤적에서 range_m 밖 unknown(광선 FREE 미구현)",
           "algorithm": core["alg"],
           "unsupported": ["passage_width_m", "overhead_m", "gap_width_m"] + crit.get("unsupported_v0", []),
           "source_map": {"tracked": summ.get("tracked"), "lost": summ.get("lost"), "frame_id": summ.get("frame_id"),
                          "main_segment": summ.get("main_segment"), "preprocess": summ.get("preprocess"), "main_segment_poses": int(len(cams))},
           "obstacles": core["obstacles"]}
    json.dump(out, open(os.path.join(map_dir, "terrain_map.json"), "w"), ensure_ascii=False, indent=1)
    np.savez_compressed(os.path.join(map_dir, "terrain_grid.npz"), x0=core["grid"]["x0"], y0=core["grid"]["y0"], cell=cell,
                        **{k: (v.astype(str) if v.dtype == object else v) for k, v in grids.items()})
    level = grids["level"]
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
    print(json.dumps({k: out[k] for k in ("cell_counts", "criteria_version", "classification")}, ensure_ascii=False))
    for o in out["obstacles"]:
        print(o["id"], o["level"], o["centroid_xy_m"], "단차", o["step_m"], o["direction"], "윗면", o["top_z_m"], o["reasons"], "면적", o["area_m2"])


if __name__ == "__main__":
    main()
