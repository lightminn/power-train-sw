# obstacles_map.py — L2: map_seq(L1) 지도 점구름 → 2.5D 높이 격자 → 기준값(JSON) 분류 → 장애물 목록(terrain_map JSON)·위에서 본 그림.
# 기준값 정본: 시뮬 담당의 obstacle_criteria JSON(v0.1, v0.2부터 프로필 A·B — criteria.load_criteria). 값을 코드에 옮겨 적지 않고 파일을 읽는다.
# v3(10-18, 교차검토 4·5차 반영) — 두 층으로 나눈다:
#   level_local = 그 칸의 국소 위험(방향 무관). 경사·거칠기, 칸 안의 설명되지 않는 돌출·꺼짐, 머리 위 틈, 4-이웃과의 단차
#                 (그 칸이 낮은 쪽이면 오르기 기준 step_up, 높은 쪽이면 내려가기 기준 step_down).
#   level       = 출발 지지면에서의 경로 허가. 출발 시야(시작 카메라 앞 0.25–1.2 m, ±40°)의 z = 0 근처 칸(섞인 칸 제외) 가운데 로버에서
#                 가장 가까운 띠(+15 cm)가 속한 구역의 그 칸들에서 출발해(멀리 있는 큰 구역으로 옮기지 않음 — 바로 앞 상자 뒤 바닥을 출발점으로 잡지 않게)
#                 4-이웃으로 가는 길 가운데 '가장 나쁜 칸·경계'가 가장 덜 나쁜 길의 단계(칸 단위 병목). 경계 단차는 가는 방향으로
#                 (오르기 = 가는 칸 윗면 − 오는 칸 지지면 → step_up, 내려가기 = 오는 칸 지지면 − 가는 칸 아랫면 → step_down;
#                  섞인 칸의 윗면·아랫면 = 상·하위 10% 높이, 아니면 지지면),
#                 칸은 경사·거칠기·머리 위 틈을 포함한다. 관측 안 된 칸과 표면을 평가 못 한 칸은 지나갈 수 없다 → 이어지지 않으면 unknown.
#   - 칸 높이 = 주 지지면(점이 가장 많은 3 cm 높이 창 → 그 안 최빈 1 cm 구간 중앙값 ±1.5 cm 안 점의 평균). 수직 면 점이 칸 높이를 끌어올려 바닥과 윗면이 이어지는 것을 막는다.
#     칸 안 상위 10%가 주 지지면보다 link_tol 넘게 높으면 돌출, 하위 10%가 낮으면 꺼짐(섞인 칸). 국소 층에서는 늘 그 칸의 위험이고
#     (이웃 높이로 지우지 않음 — 우연히 같은 높이 칸이 있으면 돌출이 사라지던 문제), 경로 층에서는 그 칸으로 들어갈 때의 경계 단차로 잡힌다.
#   - 평가하지 못한 칸(지지 부족·퇴화)은 통과가 아니라 unknown이다. 단차·돌출에는 측정 여유를 더해 경계 근처 값은 나쁜 쪽으로 보낸다.
#     거칠기는 잔차의 5–95% 범위이고 측정 여유를 더하지 않는다.
#   - 머리 위 구조: 같은 칸에서 지지면 위로 overhang_gap 넘게 비어 있다가 나타나는 점(책상·선반 아랫면)은 돌출이 아니라 머리 위 틈으로
#     overhead 규칙(1.00 m 이상 통과, 0.90 m 미만 불가)을 적용한다. 아래 지지면이 같은 칸에서 보일 때만 잴 수 있다(부분 지원).
#   - 장애물 목록 = 국소 위험 칸 + 경로 허가가 주의·불가인 솟은/꺼진 면(z가 step_up 주의 이상 높거나 step_down 주의 이상 낮음). 단일 칸도 남기고,
#     다각형은 칸 모서리의 볼록 껍질이다(위험 영역을 줄이지 않는다).
# 관측 모델 한계: 표면 점이 있는 칸 = 관측, 없는 칸 = unknown(광선으로 빈 공간을 지우는 FREE는 없다). 가림 띠 너머의 낮은 바닥은 경로가
#   이어지지 않아 unknown이다. 출발 지지면과 관측된 길로 이어지지 않는 곳(상자에 가로막힌 반대편 바닥 등)은 level이 불가(via_…)지만
#   level_local은 그대로 통과일 수 있다 — 관측된 범위에서는 거기까지 갈 길이 없다는 뜻이다. 통로 폭·머리 위 높이·틈 폭은 계산하지 않는다.
# 사용: python obstacles_map.py <map_seq 출력 폴더> <기준값 json> [--cell 0.05]
import argparse, csv, heapq, json, math, os, sys
import numpy as np
import cv2

if __package__:
    from .criteria import load_criteria
else:
    from criteria import load_criteria

LEVEL = {"unknown": 0, "pass": 1, "caution": 2, "block": 3}
NAME = {v: k for k, v in LEVEL.items()}
COLOR_BGR = {0: (158, 158, 158), 1: (50, 125, 46), 2: (37, 168, 249), 3: (40, 40, 198)}
NB4 = ((0, 1), (1, 0), (0, -1), (-1, 0))

# 알고리즘 설정 — 정책 기준값(JSON)과 별개다. 출력 JSON "algorithm"에 그대로 남긴다
ALG = {"min_pts": 6,             # 칸 통계에 쓰는 최소 지도 점 수(TSDF 표면 점: 1.5 cm 복셀이면 수평 칸 하나에 약 11개)
       "link_tol_m": 0.03,       # 같은 지지면으로 보는 이웃 높이 차 상한, 칸 안 돌출·꺼짐 판정 기준
       "wall_m": 0.5,            # 이 이상 오르는 경계·돌출은 벽(불가)
       "meas_margin_m": 0.01,    # 단차·돌출 값에 더하는 측정 여유(거칠기에는 미적용 — 잔차 범위에 잡음이 이미 들어 있다)
       "slope_win_m": 0.30,      # 경사·거칠기 창(기준값: 30 cm 이상 창)
       "planar_rough_m": 0.02,   # 창 전체 평면 잔차(5–95% 범위)가 이보다 작으면 구역과 무관하게 그 평면으로 경사를 잰다
       "min_support_frac": 0.25,  # 창에서 평면 맞춤에 필요한 칸 비율(가로·세로 각 3칸 이상은 별도 검사)
       "overhang_gap_m": 0.25,   # 같은 칸에서 지지면 위로 이만큼 넘게 비어 있다가 나타나는 점 = 머리 위 구조(overhead 규칙), 돌출 아님
       "ref_seed_h_m": 0.03,     # 출발 지지면 씨앗: |z| ≤ 이 값(z = 0은 L1 초기 평면)
       "start_view_m": [0.25, 1.2], "start_view_deg": 40.0,   # 씨앗을 찾는 출발 시야(시작 카메라 기준 거리·좌우 각) — 로버 바로 앞 바닥
       "ref_band_m": 0.15,       # 출발 지지면 = 시야 안 가장 가까운 씨앗에서 이 거리 안 씨앗이 가장 많이 속한 구역
       "ref_min_seed": 12,       # 그 구역의 씨앗 칸 최소(모자라면 다른 구역으로 옮기지 않고 보류)
       "ref_min_cells": 40,      # 출발 지지면 구역 칸 합이 이보다 적으면 분류 보류
       "z_max_m": 2.0,           # 이보다 높은 점(천장 등)은 2.5D 계산에서 뺀다
       "range_m": 4.0}           # 주 구간 카메라 궤적에서 이보다 먼 칸은 unknown


EPS = 1e-9   # 경계 비교 허용오차: 0.08 + 0.01이 0.09보다 작게 계산돼 보수 판정이 뒤집히지 않게(Codex 9차 #13)


def classify(value, rule):
    """기준값 규칙 하나로 값 → 단계(1 pass, 2 caution, 3 block). 반개구간(JSON interval_rule), 경계는 보수 쪽으로 EPS 허용."""
    if rule["kind"] == "higher_is_worse":
        return np.where(value >= rule["block_from"] - EPS, 3, np.where(value >= rule["caution_from"] - EPS, 2, 1))
    return np.where(value < rule["block_below"] + EPS, 3, np.where(value < rule["pass_from"] + EPS, 2, 1))


def build_grid(P, cell, min_pts, band=0.015, ovh_gap=0.25):
    """점(N×3, 지도 좌표 z 위) → 칸별 윗면(상위 90%)·아랫면(하위 10%)·주 지지면·머리 위 틈·점 수. 점이 min_pts 미만인 칸은 높이 NaN.
    주 지지면: 점이 가장 많은 3 cm 높이 창(1 cm 구간 셋)을 고르고, 그 안에서 점이 가장 많은 1 cm 구간의 중앙값 ±band 안 점들의 평균.
    (최빈 1 cm 구간만 고르면 점이 적은 칸에서 흔들리고, 창 평균을 쓰면 수직 면 점에 끌려 중간 높이가 된다.)
    지지면 위 2·band 넘는 점 가운데 가장 낮은 것이 지지면에서 ovh_gap 넘게 떨어져 있으면 머리 위 구조(책상·선반 아랫면)로 보고
    그 높이 차를 틈(ovh)으로 남긴 뒤 윗면·아랫면 통계에서 뺀다(돌출로 세지 않음)."""
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
    ovh = np.full(ny * nx, np.inf)
    cnt = np.zeros(ny * nx, int)
    cnt[uniq] = count
    for k, s, c in zip(uniq, start, count):
        if c >= min_pts:
            zz = z[s:s + c]
            b = np.floor((zz - zz.min()) / 0.01).astype(int)
            hc = np.bincount(b)
            sm = np.convolve(hc, np.ones(3, int), "same")              # 3 cm 창의 점 수(1 cm 최빈 구간은 점이 적은 칸에서 흔들린다)
            cand = np.nonzero(sm == sm.max())[0]
            kb = int(cand[np.argmin(np.abs(cand - np.median(b)))])      # 동률이면 중앙값에 가까운 쪽
            win = np.arange(max(kb - 1, 0), min(kb + 2, len(hc)))
            kb = int(win[np.argmax(hc[win])])                            # 그 창 안에서 점이 가장 많은 1 cm 구간(창 평균은 수직 면 점에 끌려 중간 높이가 된다)
            m0 = float(np.median(zz[b == kb]))
            sup[k] = float(zz[np.abs(zz - m0) <= band].mean())
            above = zz[zz > sup[k] + 2 * band]
            if above.size and above.min() - sup[k] >= ovh_gap:
                ovh[k] = float(above.min() - sup[k])
                zz = zz[zz <= sup[k] + 2 * band]
            top[k], low[k] = np.percentile(zz, 90), np.percentile(zz, 10)
    return {"x0": float(x0), "y0": float(y0), "nx": nx, "ny": ny, "cell": cell, "top": top.reshape(ny, nx), "low": low.reshape(ny, nx),
            "sup": sup.reshape(ny, nx), "ovh": ovh.reshape(ny, nx), "cnt": cnt.reshape(ny, nx)}


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
    """창 안 칸(칸 단위 좌표 xx, yy, 높이 w m)에 평면 맞춤 → (기울기 °, 잔차 5–95% 범위 m). 가로·세로 3칸 미만이면 퇴화 → None.
    잔차 범위는 최대−최소 대신 5–95%로 잰다(칸 수가 많은 창에서 최대−최소는 잡음만으로 부풀려진다)."""
    if len(np.unique(xx)) < 3 or len(np.unique(yy)) < 3:
        return None
    A = np.stack([xx, yy, np.ones(len(xx))], 1).astype(float)
    coef, *_ = np.linalg.lstsq(A, w, rcond=None)
    res = w - A @ coef
    return math.degrees(math.atan(math.hypot(coef[0], coef[1]) / cell)), float(np.percentile(res, 95) - np.percentile(res, 5))


def surface(h, patch, cell, alg):
    """칸별 경사(°)·거칠기(m). 창 전체 평면이 맞으면 그것을, 아니면 같은 구역 칸만으로. 지지 부족·퇴화면 NaN(→ unknown)."""
    win = max(3, int(round(alg["slope_win_m"] / cell)) | 1)
    r = win // 2
    need = max(6, int(math.ceil(alg["min_support_frac"] * win * win)))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    hp = np.pad(h, r, constant_values=np.nan)
    pp = np.pad(patch, r, constant_values=-1)
    slope = np.full(h.shape, np.nan)
    rough = np.full(h.shape, np.nan)
    src = np.zeros(h.shape, np.int8)   # 1 = 창 전체 평면, 2 = 같은 구역만
    for j, i in zip(*np.nonzero(~np.isnan(h))):
        w = hp[j:j + win, i:i + win]
        m = ~np.isnan(w)
        if m.sum() >= need:
            f = fit_plane(xx[m], yy[m], w[m], cell)
            if f is not None and f[1] < alg["planar_rough_m"]:
                slope[j, i], rough[j, i], src[j, i] = f[0], f[1], 1
                continue
        m2 = m & (pp[j:j + win, i:i + win] == patch[j, i])
        if m2.sum() >= need:
            f = fit_plane(xx[m2], yy[m2], w[m2], cell)
            if f is not None:
                slope[j, i], rough[j, i], src[j, i] = f[0], f[1], 2
    return slope, rough, src


def step_level(up, down, R, alg):
    """경계를 건너는 오르기 up = 가는 칸 윗면 − 오는 칸 지지면, 내려가기 down = 오는 칸 지지면 − 가는 칸 아랫면(m) → (단계, 이유, 부호 단차).
    가는 칸만 높이 범위로 본다(섞인 칸 둘이 맞닿을 때 같은 수직 면을 거짓 오르기로 세지 않게).
    link_tol 미만은 같은 지지면. 측정 여유를 더한다."""
    lu = ld = 1
    if up >= alg["link_tol_m"]:
        lu = 3 if up >= alg["wall_m"] else int(classify(np.array(up + alg["meas_margin_m"]), R["step_up_m"]))
    if down >= alg["link_tol_m"]:
        ld = int(classify(np.array(down + alg["meas_margin_m"]), R["step_down_m"]))
    if lu == 1 and ld == 1:
        return 1, "", 0.0
    if lu >= ld:
        return lu, ("wall" if up >= alg["wall_m"] else "step_up"), up
    return ld, "step_down", -down


def classify_points(P, crit, cams_xy, cell=0.05, alg=None, start=(0.0, 0.0, 0.0)):
    """핵심: 지도 점(z 위, 바닥 z = 0) → 두 층 단계·장애물.
    cams_xy = 주 구간 카메라 위치(N×2, 관측 범위 판정), start = 시작 카메라 (x, y, 앞 방향 yaw rad)."""
    alg = {**ALG, **(alg or {})}
    R = crit["rules"]
    tol, mg = alg["link_tol_m"], alg["meas_margin_m"]
    P = P[P[:, 2] < alg["z_max_m"]]
    G = build_grid(P, cell, alg["min_pts"], tol / 2, alg["overhang_gap_m"])
    top, low, sup = G["top"], G["low"], G["sup"]
    ny, nx = top.shape
    yy, xx = np.mgrid[0:ny, 0:nx]
    cx, cy = G["x0"] + (xx + 0.5) * cell, G["y0"] + (yy + 0.5) * cell
    observed = ~np.isnan(top)
    if len(cams_xy):
        dmin = np.full(top.shape, np.inf)
        for c in cams_xy[:: max(1, len(cams_xy) // 30)]:
            dmin = np.minimum(dmin, np.hypot(cx - c[0], cy - c[1]))
        observed &= dmin <= alg["range_m"]
    else:
        observed[:] = False
    h = np.where(observed, sup, np.nan)
    prot = np.where(observed, top - sup, np.nan)
    dep = np.where(observed, sup - low, np.nan)
    patch = smooth_patches(h, tol)
    slope, rough, src = surface(h, patch, cell, alg)
    # 섞인 칸(칸 안 돌출·꺼짐): 경계를 건널 때 가는 칸의 윗면·아랫면으로 쓴다(수직 면 칸이 중간 높이 디딤돌이 되지 않게)
    mix_up, mix_dn = observed & (prot > tol), observed & (dep > tol)
    hi = np.where(mix_up, top, h)
    lo = np.where(mix_dn, low, h)
    pr = np.nan_to_num(prot)
    lv_prot = np.where(mix_up, np.where(pr >= alg["wall_m"], 3, classify(pr + mg, R["step_up_m"])), 1)   # 국소 층: 돌출은 늘 위험(이웃 높이로 지우지 않음)
    lv_dep = np.where(mix_dn, classify(np.nan_to_num(dep) + mg, R["step_down_m"]), 1)
    lv_slope = np.where(np.isnan(slope), 0, classify(np.nan_to_num(slope), R["slope_deg"]))
    lv_rough = np.where(np.isnan(rough), 0, classify(np.nan_to_num(rough), R["roughness_m"]))   # 잡음이 이미 범위를 키우므로 측정 여유 미적용
    surf = np.where(np.isnan(slope), 0, np.maximum(lv_slope, lv_rough))
    ovh = np.where(observed, G["ovh"], np.inf)
    lv_ovh = np.where(np.isfinite(ovh), classify(np.where(np.isfinite(ovh), ovh, 9.0) - mg, R["overhead_m"]), 1)   # 머리 위 틈(낮을수록 나쁨)
    # ── 국소 층: 4-이웃 단차(낮은 쪽 = 오르기, 높은 쪽 = 내려가기)
    up_n = np.full(top.shape, -np.inf)
    dn_n = np.full(top.shape, -np.inf)
    hip, lop = np.pad(hi, 1, constant_values=np.nan), np.pad(lo, 1, constant_values=np.nan)
    for dy, dx in NB4:
        up_n = np.fmax(up_n, hip[1 + dy:1 + dy + ny, 1 + dx:1 + dx + nx] - h)    # 이 칸(주 지지면)에서 이웃(윗면)으로 오르기
        dn_n = np.fmax(dn_n, h - lop[1 + dy:1 + dy + ny, 1 + dx:1 + dx + nx])    # 이 칸에서 이웃(아랫면)으로 내려가기
    upv, dnv = np.nan_to_num(up_n, neginf=0), np.nan_to_num(dn_n, neginf=0)
    lv_up = np.where(upv >= tol, np.where(upv >= alg["wall_m"], 3, classify(upv + mg, R["step_up_m"])), 1)
    lv_dn = np.where(dnv >= tol, classify(dnv + mg, R["step_down_m"]), 1)
    cand = [("slope", lv_slope, slope), ("roughness", lv_rough, rough), ("protrusion", lv_prot, prot), ("depression", lv_dep, -dep),
            ("step_up", lv_up, upv), ("step_down", lv_dn, -dnv), ("overhead", lv_ovh, np.where(np.isfinite(ovh), ovh, np.nan))]
    haz = np.maximum.reduce([lv_prot, lv_dep, lv_up, lv_dn, lv_ovh])
    local = np.where(surf == 0, np.where(haz >= 2, haz, 0), np.maximum(surf, haz))
    local[~observed] = 0
    reason_local = np.full(top.shape, "", dtype=object)
    step_local = np.full(top.shape, np.nan)
    best = np.ones(top.shape, int)
    for name, lv, val in cand:
        lvz = np.where(np.isnan(lv.astype(float)), 0, lv)
        better = observed & (lvz > best) & (lvz >= 2)
        reason_local[better] = name
        step_local[better] = np.nan_to_num(val)[better] if name not in ("slope", "overhead") else np.nan
        best = np.where(better, lvz, best)
    reason_local[observed & (local == 0)] = "surface_unevaluated"
    # ── 경로 허가 층: 출발 지지면에서 칸 단위 병목
    sx, sy, syaw = start
    dist = np.hypot(cx - sx, cy - sy)
    ang = np.abs((np.arctan2(cy - sy, cx - sx) - syaw + np.pi) % (2 * np.pi) - np.pi)
    near = (dist >= alg["start_view_m"][0]) & (dist <= alg["start_view_m"][1]) & (ang <= math.radians(alg["start_view_deg"]))
    passable = observed & (surf > 0)        # 표면을 평가 못 한 칸은 지나갈 수 없다(뒤쪽도 이어지지 않으면 unknown)
    seed = passable & near & ~mix_up & ~mix_dn & (np.abs(np.nan_to_num(h, nan=9.0)) <= alg["ref_seed_h_m"])
    npatch = int(patch.max()) + 1
    n_seed = np.bincount(patch[seed], minlength=npatch) if npatch > 0 else np.zeros(0, int)
    n_cells = np.bincount(patch[patch >= 0], minlength=npatch) if npatch > 0 else np.zeros(0, int)
    ref = np.zeros(npatch, bool)
    if seed.any():   # 출발 지지면 = 로버에서 가장 가까운 씨앗 띠(최근접 + ref_band)의 구역. 멀리 있는 큰 구역으로 옮기지 않는다
        band = seed & (dist <= dist[seed].min() + alg["ref_band_m"])
        pid = int(np.argmax(np.bincount(patch[band], minlength=npatch)))
        if n_seed[pid] >= alg["ref_min_seed"]:
            ref[pid] = True
    ref_cells = int(n_cells[ref].sum()) if npatch > 0 else 0
    node = np.maximum(surf, lv_ovh)          # 경로 계산용 칸 단계. 돌출·꺼짐은 경계(가는 칸 윗면 − 오는 칸 지지면)에서 잰다
    node_why = np.where((lv_ovh >= surf) & (lv_ovh > 1), "overhead", np.where(lv_slope >= lv_rough, "slope", "roughness"))
    reach = np.full(top.shape, 99, int)
    why = np.full(top.shape, "", dtype=object)
    step = np.full(top.shape, np.nan)
    info = {"ref_cells": ref_cells, "ref_patches": int(ref.sum()) if npatch else 0, "start_seed_cells": int(seed.sum()), "patches": npatch,
            "mixed_cells": int((mix_up | mix_dn).sum()), "unevaluated_cells": int((observed & ~passable).sum()),
            "observed_cells": int(observed.sum()), "withheld": None}
    if ref_cells < alg["ref_min_cells"]:
        info["withheld"] = f"출발 지지면(시작 시야의 z = 0 근처) 칸 {ref_cells} < {alg['ref_min_cells']} — 경로 허가 보류, level 전부 unknown"
    else:
        srcs = seed & np.isin(patch, np.nonzero(ref)[0])
        heap = []
        for j, i in zip(*np.nonzero(srcs)):
            reach[j, i] = node[j, i]
            why[j, i] = node_why[j, i] if node[j, i] > 1 else ""
            heap.append((int(node[j, i]), int(j), int(i)))
        heapq.heapify(heap)
        while heap:
            L, j, i = heapq.heappop(heap)
            if L > reach[j, i]:
                continue
            for dy, dx in NB4:
                b, a = j + dy, i + dx
                if not (0 <= b < ny and 0 <= a < nx) or not passable[b, a]:
                    continue
                e, ewhy, d = step_level(float(hi[b, a] - h[j, i]), float(h[j, i] - lo[b, a]), R, alg)
                n = max(L, e, int(node[b, a]))
                if n < reach[b, a]:
                    reach[b, a] = n
                    if n == e and e > L and e >= node[b, a]:
                        why[b, a], step[b, a] = ewhy, d
                    elif n == node[b, a] and node[b, a] > L:
                        why[b, a], step[b, a] = node_why[b, a], np.nan
                    else:
                        why[b, a] = why[j, i] if why[j, i].startswith("via_") or not why[j, i] else "via_" + why[j, i]
                        step[b, a] = step[j, i]
                    heapq.heappush(heap, (n, b, a))
    level = np.where(reach == 99, 0, reach)
    level[~observed] = 0
    why[(level == 0) & observed] = np.where(passable[(level == 0) & observed], "no_path_from_start", "surface_unevaluated")
    why[~observed] = ""
    # 장애물 목록: 국소 위험 칸 + 경로 허가가 주의·불가인 솟은/꺼진 면
    hz = np.nan_to_num(h)
    island = observed & (level >= 2) & ((hz >= R["step_up_m"]["caution_from"]) | (hz <= -R["step_down_m"]["caution_from"]))
    olev = np.where(local >= 2, local, 0)
    olev = np.where(island, np.maximum(olev, level), olev)
    n, lab = cv2.connectedComponents((olev >= 2).astype(np.uint8), connectivity=8)
    obstacles = []
    obj_id = np.full(top.shape, -1, int)
    for c in range(1, n):
        m = lab == c
        obj_id[m] = len(obstacles)
        lv = int(olev[m].max())
        js, is_ = np.nonzero(m)
        corners = np.concatenate([np.stack([G["x0"] + (is_ + a) * cell, G["y0"] + (js + b) * cell], 1)
                                  for a in (0, 1) for b in (0, 1)]).astype(np.float32)
        hull = cv2.convexHull(corners)[:, 0, :]
        body = m & (olev == lv)
        st_all = np.concatenate([step[body & island & ~np.isnan(step)], step_local[body & ~island & ~np.isnan(step_local)]])
        st = float(np.median(st_all)) if st_all.size else None
        rs = np.where(island[m], why[m], reason_local[m]).astype(str)
        ru, rc = np.unique(rs, return_counts=True)
        obstacles.append({"id": len(obstacles), "level": NAME[lv], "cells": int(m.sum()), "area_m2": round(float(m.sum() * cell * cell), 4),
                          "centroid_xy_m": [round(float(cx[m].mean()), 3), round(float(cy[m].mean()), 3)],
                          "step_m": None if st is None else round(st, 3),
                          "direction": None if st is None else ("up" if st > 0 else "down"),
                          "top_z_m": round(float(np.nanmax(top[body])), 3), "support_z_m": round(float(np.nanmedian(h[body])), 3),
                          "reasons": {str(a): int(b) for a, b in zip(ru, rc) if a},
                          "poly_xy_m": [[round(float(x), 3), round(float(y), 3)] for x, y in hull]})
    grids = {"level": level, "level_local": local, "reason": why, "reason_local": reason_local, "step": step, "step_local": step_local,
             "top": top, "low": low, "sup": sup, "overhead_gap": ovh, "cnt": G["cnt"], "patch": patch, "observed": observed, "slope": slope, "rough": rough,
             "surface_src": src, "mixed_up": mix_up, "mixed_down": mix_dn, "start_seed": seed, "obstacle_id": obj_id}
    counts = {NAME[v]: int((level == v).sum()) for v in LEVEL.values()}
    counts_local = {NAME[v]: int((local == v).sum()) for v in LEVEL.values()}
    return {"grid": {"x0": G["x0"], "y0": G["y0"], "nx": G["nx"], "ny": G["ny"]}, "info": info, "obstacles": obstacles,
            "cell_counts": counts, "cell_counts_local": counts_local, "alg": alg}, grids


def main_segment_poses(map_dir, main_segment):
    """poses.csv에서 주 구간의 수용된 자세(T 열이 있는 행) → 카메라 XY 목록과 시작 자세 (x, y, 앞 방향 yaw)."""
    out, start = [], None
    p = os.path.join(map_dir, "poses.csv")
    if not os.path.exists(p) or main_segment is None:
        return np.zeros((0, 2)), None
    for r in csv.DictReader(open(p)):
        if str(r.get("segment")) == str(main_segment) and r.get("T03") not in (None, ""):
            out.append([float(r["T03"]), float(r["T13"])])
            if start is None:   # 카메라 광축(optical z) = 회전 행렬 셋째 열의 지도 성분
                start = (float(r["T03"]), float(r["T13"]), math.atan2(float(r["T12"]), float(r["T02"])))
    return np.array(out).reshape(-1, 2), start


def run(map_dir, criteria_path, cell=0.05, allow_unaligned=False, alg=None, profile=None):
    import open3d as o3d
    crit = load_criteria(criteria_path, profile)
    summ = json.load(open(os.path.join(map_dir, "summary.json"), encoding="utf-8"))
    aligned = bool(summ.get("alignment_valid"))
    if not aligned and not allow_unaligned:   # 초기 평면 정렬이 없으면 z가 높이가 아니다
        raise SystemExit(f"지도 좌표 정렬이 유효하지 않음(frame_id={summ.get('frame_id')}) — L2는 높이 지도로 쓸 수 없다")
    P = np.asarray(o3d.io.read_point_cloud(os.path.join(map_dir, "map_points.ply")).points)
    cams, start = main_segment_poses(map_dir, summ.get("main_segment"))
    core, grids = classify_points(P, crit, cams, cell, alg, start or (0.0, 0.0, 0.0))
    out = {"type": "terrain_map", "schema_version": 3, "map_version": 1, "criteria_version": crit.get("criteria_version"),
           "profile": crit.get("profile"), "criteria_profile": crit.get("criteria_profile"), "alignment_valid": aligned,
           "frame": "map(z 위, 바닥 z = 0, 원점 = 시작 카메라 바로 아래, x = 시작 시 카메라 앞)" if aligned else "unaligned(높이 아님 — 시험용)",
           "cell_m": cell, "grid": core["grid"], "start_pose": start, "cell_counts": core["cell_counts"], "cell_counts_local": core["cell_counts_local"],
           "classification": core["info"],
           "grid_file": {"path": "terrain_grid.npz", "index": "배열[iy, ix], 칸 중심 x = x0 + (ix + 0.5)·cell, y = y0 + (iy + 0.5)·cell",
                         "layers": sorted(grids)},
           "semantics": {"level": "출발 지지면에서의 경로 허가(칸 단위 병목, 단차는 가는 방향). 이어지지 않으면 unknown, 가로막히면 via_…",
                         "level_local": "그 칸의 국소 위험(방향 무관: 낮은 쪽 칸은 오르기, 높은 쪽 칸은 내려가기 기준)",
                         "pass": "평가한 요소(단차·경사·거칠기·돌출)만의 판정. unknown은 경로 허가가 아니다"},
           "observation_model": "표면 점 ≥ min_pts인 칸 = 관측, 아니면 unknown. 주 구간 카메라 궤적에서 range_m 밖 unknown(광선 FREE 미구현)",
           "algorithm": core["alg"],
           "unsupported": ["passage_width_m", "gap_width_m", "overhead_m(부분 지원: 같은 칸에서 아래 지지면이 보일 때만)"] + crit.get("unsupported_v0", []),
           "source_map": {"tracked": summ.get("tracked"), "lost": summ.get("lost"), "frame_id": summ.get("frame_id"),
                          "main_segment": summ.get("main_segment"), "preprocess": summ.get("preprocess"), "main_segment_poses": int(len(cams))},
           "obstacles": core["obstacles"]}
    json.dump(out, open(os.path.join(map_dir, "terrain_map.json"), "w"), ensure_ascii=False, indent=1)
    np.savez_compressed(os.path.join(map_dir, "terrain_grid.npz"), x0=core["grid"]["x0"], y0=core["grid"]["y0"], cell=cell,
                        **{k: (v.astype(str) if v.dtype == object else v) for k, v in grids.items()})
    for key, name in (("level", "terrain_top.png"), ("level_local", "terrain_local.png")):
        img = np.zeros(grids[key].shape + (3,), np.uint8)
        for v, col in COLOR_BGR.items():
            img[grids[key] == v] = col
        img = cv2.resize(img[::-1], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)   # 위가 +y(왼쪽)인 위에서 본 그림
        cv2.imwrite(os.path.join(map_dir, name), img)
    return out


def main():
    ap = argparse.ArgumentParser(description="L1 지도 → 장애물·지형 분류(L2)")
    ap.add_argument("map_dir")
    ap.add_argument("criteria")
    ap.add_argument("--cell", type=float, default=0.05)
    ap.add_argument("--profile", help="기준값 프로필(v0.2: A_no_stop 기본 · B_stop20). 없으면 파일의 기본")
    a = ap.parse_args()
    out = run(a.map_dir, a.criteria, a.cell, profile=a.profile)
    print(json.dumps({k: out[k] for k in ("cell_counts", "cell_counts_local", "criteria_version", "classification")}, ensure_ascii=False))
    for o in out["obstacles"]:
        print(o["id"], o["level"], o["centroid_xy_m"], "단차", o["step_m"], o["direction"], "윗면", o["top_z_m"], o["reasons"], "면적", o["area_m2"])


if __name__ == "__main__":
    main()
