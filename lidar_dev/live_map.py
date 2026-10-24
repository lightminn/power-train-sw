# live_map.py — 실시간 2.5D 높이 지도(10-22, live_overlay.py 3판): 찾은 바닥 평면 기준으로 점을 칸(기본 2 cm)에 모아
#   칸마다 윗면 높이(상위 90%)를 프레임 사이 지수 평균으로 쌓고, 기준값 JSON(step_up_m·trench_depth_m)으로 통과·주의·불가를 칠한다.
#   - 위치 추정이 없으므로 카메라가 가만히 있을 때만 쌓는다: 자이로가 돌면 쌓기를 멈추고, 바닥 평면이 3°·3 cm 넘게 바뀌거나
#     세로축으로 4° 넘게 돌면 지도를 비운다. keep_s 넘게 안 보인 칸은 모름(가려진 곳은 바닥이 아니라 모름)
#   - 분류는 바닥 평면 기준 높이로만 한다(obstacles_map.py L2처럼 이웃 지지면과의 단차는 아님) — 평평한 바닥 위 시험용
#     올라온 칸: step_up_m(측정 여유 +1 cm, 0.5 m 넘으면 벽), 꺼진 칸: trench_depth_m(내려가기 step_down_m보다 보수)
import math

import cv2
import numpy as np

if __package__:
    from .obstacles_map import classify
else:
    from obstacles_map import classify             # 경계 비교·EPS를 L2와 같게

UP_COL = {1: (50, 125, 46), 2: (37, 168, 249), 3: (40, 40, 198)}     # 통과·주의·불가(BGR, obstacles_map 색)
DIP_COL = {2: (235, 160, 40), 3: (190, 40, 150)}                    # 꺼진 칸 주의·불가(하늘·보라)
UNK = (48, 48, 48)


def classify_heights(Z, known, rules, margin=0.01, wall=0.5):
    """바닥 평면 기준 칸 높이 → (단계 0 모름·1 통과·2 주의·3 불가, 꺼진 칸 여부). 올라온 칸 = step_up_m(측정 여유 + margin, wall 넘으면 벽),
    꺼진 칸 = trench_depth_m. 자체 지도(LiveHeightMap)와 광민 지도(live_overlay --map gwangmin)가 같이 쓴다."""
    z0 = np.nan_to_num(Z, nan=0.0)
    if rules:
        up = classify(np.maximum(z0, 0) + margin, rules["step_up_m"])
        up = np.where(z0 >= wall, 3, up)
        dn = classify(np.maximum(-z0, 0) + margin, rules.get("trench_depth_m") or rules["step_up_m"])   # v0.2.1 전 형식엔 trench 없음 → 다시 올라와야 하니
        #   턱 기준으로 보수 대체(step_down_m 15/30 cm로 보면 6.5 cm 도랑도 통과로 보임 — Codex 15차 C12와 같은 이유)
        lv, dip = np.maximum(up, dn).astype(np.int8), dn > up
    else:
        lv, dip = np.ones(Z.shape, np.int8), np.zeros(Z.shape, bool)
    lv[~known] = 0
    return lv, dip


class LiveHeightMap:
    def __init__(self, cell=0.02, fwd_max=2.5, lat_max=1.25, alpha=0.35, min_pts=3, keep_s=2.0, z_max=1.0,
                 rules=None, margin=0.01, wall=0.5):
        self.cell, self.fwd_max, self.lat_max = cell, fwd_max, lat_max
        self.nf, self.nl = int(round(fwd_max / cell)), int(round(2 * lat_max / cell))
        self.alpha, self.min_pts, self.keep_s, self.z_max = alpha, min_pts, keep_s, z_max
        self.rules, self.margin, self.wall = rules, margin, wall
        self.n_reset = -1
        self.reset("start")

    def reset(self, why):
        self.h = np.full((self.nf, self.nl), np.nan, np.float32)      # [앞 칸, 옆 칸], 앞 0 = 카메라 발밑
        self.seen = np.zeros((self.nf, self.nl), np.int32)
        self.t_last = np.full((self.nf, self.nl), -1e9)
        self.ref, self.yaw, self.why, self.frames = None, 0.0, why, 0
        self.n_reset += 1

    def check_motion(self, n, d):
        """바닥 평면·세로축 회전(self.yaw, 밖에서 자이로로 누적)으로 카메라가 움직였는지 → 움직였으면 지도를 비우고 True."""
        if self.ref is None:
            self.ref = (n.copy(), float(d))
            return False
        ang = math.degrees(math.acos(min(1.0, float(n @ self.ref[0]))))
        dd = abs(float(d) - self.ref[1])
        if ang > 3.0 or dd > 0.03 or abs(self.yaw) > 4.0:
            self.reset(f"moved (plane {ang:.1f}deg {dd * 100:.1f}cm, turn {self.yaw:+.1f}deg)")
            self.ref = (n.copy(), float(d))
            return True
        return False

    def update(self, H, F, L, valid, t, step=2):
        """H 바닥 위 높이, F 앞 거리, L 옆(오른쪽 +) — 이미지 모양 배열. step 픽셀마다 하나."""
        H, F, L, valid = H[::step, ::step], F[::step, ::step], L[::step, ::step], valid[::step, ::step]
        m = valid & (F >= 0) & (F < self.fwd_max) & (np.abs(L) < self.lat_max) & (H > -0.5) & (H < self.z_max)
        if m.sum() < 100:
            return
        i = np.minimum((F[m] / self.cell).astype(np.int64), self.nf - 1)
        j = np.minimum(((L[m] + self.lat_max) / self.cell).astype(np.int64), self.nl - 1)
        hq = np.clip(np.round((H[m] + 0.5) * 1000), 0, 2047).astype(np.int64)     # 1 mm 단위, −0.5…1.5 m
        code = np.sort((i * self.nl + j) * 2048 + hq)                              # 칸 → 높이 순 정렬 한 번
        key, hq = code >> 11, code & 2047
        start = np.r_[0, np.flatnonzero(np.diff(key)) + 1]
        count = np.diff(np.r_[start, len(key)])
        ok = count >= self.min_pts
        uniq, start, count = key[start[ok]], start[ok], count[ok]
        top = hq[start + ((count - 1) * 0.9).astype(np.int64)] / 1000.0 - 0.5    # 칸 윗면 = 상위 90%(튀는 점 몇 개는 무시)
        hf, sf, tf = self.h.ravel(), self.seen.ravel(), self.t_last.ravel()
        cur = hf[uniq]
        fresh = np.isnan(cur) | (t - tf[uniq] > self.keep_s)
        hf[uniq] = np.where(fresh, top, cur + self.alpha * (top - cur))
        sf[uniq] = np.where(fresh, 1, sf[uniq] + 1)
        tf[uniq] = t
        self.frames += 1

    def levels(self, t):
        """→ (Z 높이(모름 NaN), known, 단계 0 모름·1 통과·2 주의·3 불가, 꺼진 칸 여부)"""
        known = (t - self.t_last <= self.keep_s) & ~np.isnan(self.h) & (self.seen >= 2)
        Z = np.where(known, self.h, np.nan)
        lv, dip = classify_heights(Z, known, self.rules, self.margin, self.wall)
        return Z, known, lv, dip

    def cell_xy(self, i, j):
        return (i + 0.5) * self.cell, (j + 0.5) * self.cell - self.lat_max

    def nearest(self, lv, half_width, level):
        return nearest_in_band(lv, self.cell, self.lat_max, half_width, level)


def filter_hazards(lv, dip, codes=None, C=None, min_neighbors=1):
    """튐 거르기(5판, 사용자: '장애물이 없는데 빨간 기둥이 생겼다 사라짐'): 광민 칸 높이는 마지막 한 번 관측이라 한 프레임 튐이 그대로 단계가 된다.
    (1) codes(광민 칸 분류 이력 다수결 — 최근 3개 중 2개)가 있으면 올라온 주의·불가는 코드가 OBSTACLE, 꺼진 주의·불가는 DITCH일 때만 남김
        (광민 코드 문턱을 기준값 측정값 문턱 = 주의 − 측정 여유로 맞춰 두면 두 판정 경계가 같다)
    (2) 주의·불가 칸은 8이웃 가운데 min_neighbors개 이상이 주의·불가일 때만(외톨이 제거, 0 = 끔).
    반환 (lv, dip, 지운 칸 수). 지운 칸은 통과(1)로 — 관측은 됐으므로 모름이 아님."""
    lv = lv.copy()
    n0 = int((lv >= 2).sum())
    if codes is not None and C is not None:
        bad = (lv >= 2) & ((~dip & (codes != C.OBSTACLE)) | (dip & (codes != C.DITCH)))
        lv[bad] = 1
    if min_neighbors > 0:
        h = (lv >= 2).astype(np.uint8)
        nb = cv2.filter2D(h, -1, np.ones((3, 3), np.float32), borderType=cv2.BORDER_CONSTANT).astype(np.int32) - h
        lv[(h == 1) & (nb < min_neighbors)] = 1
    return lv, dip & (lv >= 2), n0 - int((lv >= 2).sum())


def nearest_in_band(lv, cell, lat_max, half_width, level):
    """반폭 띠 안에서 단계 ≥ level인 가장 가까운 칸의 앞 거리(m) — 없으면 None. 격자 [앞 칸, 옆 칸], 옆 0 = −lat_max."""
    i, j = np.nonzero(lv >= level)
    if not len(i):
        return None
    f, l = (i + 0.5) * cell, (j + 0.5) * cell - lat_max
    sel = np.abs(l) <= half_width
    return float(f[sel].min()) if sel.any() else None


def tmap_local_grid(snap, now, cam_xy, fwd_max, lat_max, keep_s, C):
    """광민 지도 스냅숏(콘솔 수신기 MapSnapshot, 세계 고정 칸) → 지금 카메라 기준 '앞이 위' 격자.
    원점 = 카메라를 바닥에 내린 점(base 자세 + 장착 위치 cam_xy), 앞 = 현재 요. 반환 dict(Z m·known·codes·age_s·cell) 또는 None.
    칸 높이는 1 cm 정수(−128 = 관측 없음), 코드 FREE·OBSTACLE·DITCH만 known(관측 부족 INSUFFICIENT는 모름), 나이 > keep_s도 모름."""
    st = snap.status
    if st is None or not snap.tiles:
        return None
    cell, side = float(st.cell_m), int(st.tile_cells)
    tx0, ty0 = min(t.tx for t in snap.tiles), min(t.ty for t in snap.tiles)
    ntx, nty = max(t.tx for t in snap.tiles) - tx0 + 1, max(t.ty for t in snap.tiles) - ty0 + 1
    if ntx * nty > 4096:
        return None
    H = np.full((nty * side, ntx * side), -128, np.int16)
    K = np.full(H.shape, 255, np.uint8)
    A = np.full(H.shape, np.inf, np.float32)
    for t in snap.tiles:
        r0, c0 = (t.ty - ty0) * side, (t.tx - tx0) * side
        H[r0:r0 + side, c0:c0 + side] = t.heights.reshape(side, side)
        K[r0:r0 + side, c0:c0 + side] = t.codes.reshape(side, side)
        A[r0:r0 + side, c0:c0 + side] = t.ages.reshape(side, side).astype(np.float32) * st.age_step_s + np.float32(now - t.received_s)
    px, py, yaw = (st.pose.x_m, st.pose.y_m, st.pose.yaw_rad) if st.pose is not None else (0.0, 0.0, 0.0)
    c, s = np.cos(yaw), np.sin(yaw)
    ox, oy = px + cam_xy[0] * c - cam_xy[1] * s, py + cam_xy[0] * s + cam_xy[1] * c
    nf, nl = int(round(fwd_max / cell)), int(round(2 * lat_max / cell))
    F, L = np.meshgrid((np.arange(nf) + 0.5) * cell, (np.arange(nl) + 0.5) * cell - lat_max, indexing="ij")   # L: 오른쪽 +
    wx, wy = ox + F * c + L * s, oy + F * s - L * c                       # 오른쪽 = (sin, −cos)(세계 y = 왼쪽)
    gx = np.floor(wx / cell).astype(np.int64) - tx0 * side
    gy = np.floor(wy / cell).astype(np.int64) - ty0 * side
    inb = (gx >= 0) & (gx < H.shape[1]) & (gy >= 0) & (gy < H.shape[0])
    hh = np.full((nf, nl), -128, np.int16)
    kk = np.full((nf, nl), 255, np.uint8)
    aa = np.full((nf, nl), np.inf, np.float32)
    hh[inb], kk[inb], aa[inb] = H[gy[inb], gx[inb]], K[gy[inb], gx[inb]], A[gy[inb], gx[inb]]
    known = (hh != -128) & np.isin(kk, (C.FREE, C.OBSTACLE, C.DITCH)) & (aa <= keep_s)
    weak = (hh != -128) & (kk == C.INSUFFICIENT) & (aa <= keep_s)          # 관측 부족(광민: 반투명 회색)
    return {"Z": np.where(known, hh / 100.0, np.nan), "known": known, "weak": weak, "codes": kk, "age_s": aa, "cell": cell, "nf": nf, "nl": nl}


def tidy_low(Z, known, lv, low_m=0.03, min_neighbours=2):
    """광민 콘솔 _tidy와 같은 규칙: |h| < low_m이고 주의·불가가 아닌 '낮은' 칸은 관측된 8이웃이 min_neighbours개 미만이면 숨김(모름으로 그림)."""
    k = known.astype(np.uint8)
    nb = cv2.filter2D(k, -1, np.ones((3, 3), np.float32), borderType=cv2.BORDER_CONSTANT).astype(np.int32) - k
    low = known & (np.abs(np.nan_to_num(Z)) < low_m) & (lv <= 1)
    return known & ~(low & (nb < min_neighbours))


def hillshade(Zf, cell):
    gy, gx = np.gradient(Zf, cell)                       # 행 = 앞, 열 = 오른쪽
    nrm = np.dstack([-gx, -gy, np.ones_like(Zf)])
    nrm /= np.linalg.norm(nrm, axis=2, keepdims=True)
    ld = np.array([-0.45, 0.45, 0.77])
    return np.clip(0.45 + 0.75 * (nrm @ (ld / np.linalg.norm(ld))), 0.25, 1.35)


def cell_colors(Z, known, lv, dip, mode, cell):
    if mode == "class":
        C = np.zeros(Z.shape + (3,), np.float32)
        for k, col in UP_COL.items():
            C[(lv == k) & ~dip] = col
        for k, col in DIP_COL.items():
            C[(lv == k) & dip] = col
    else:                                                # 높이 색: −10 … +20 cm
        zz = np.clip((np.nan_to_num(Z) + 0.10) / 0.30, 0, 1)
        C = cv2.applyColorMap((zz * 255).astype(np.uint8), cv2.COLORMAP_TURBO).astype(np.float32)
    C *= hillshade(np.nan_to_num(Z, nan=0.0), cell)[..., None]
    C[~known] = UNK
    return np.clip(C, 0, 255).astype(np.uint8)


def _put(img, s, org, col=(255, 255, 255), sc=0.42):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, col, 1, cv2.LINE_AA)


def top_image(M, C, size, half_width, mode):
    """위에서 본 지도(앞 = 위). 0.5 m 눈금, 반폭 선, 카메라 표시."""
    W = int(round(size * 2 * M.lat_max / M.fwd_max))
    img = cv2.resize(np.flipud(C), (W, size), interpolation=cv2.INTER_NEAREST)
    pm_x, pm_y = W / (2 * M.lat_max), size / M.fwd_max
    for r in np.arange(0.5, M.fwd_max + 1e-6, 0.5):
        y = int(size - r * pm_y)
        cv2.line(img, (0, y), (W, y), (90, 90, 90), 1)
        _put(img, f"{r:.1f}m", (4, y - 3), (200, 200, 200))
    for lat in np.arange(-M.lat_max + 0.25, M.lat_max, 0.5):
        x = int(W / 2 + lat * pm_x)
        cv2.line(img, (x, 0), (x, size), (70, 70, 70), 1)
    for lat in (-half_width, half_width):
        x = int(W / 2 + lat * pm_x)
        cv2.line(img, (x, 0), (x, size), (60, 200, 220), 1)
    cv2.drawMarker(img, (W // 2, size - 4), (230, 200, 40), cv2.MARKER_TRIANGLE_UP, 14, 2)
    if mode == "class":
        lg = [("pass", UP_COL[1]), ("caution", UP_COL[2]), ("block", UP_COL[3]), ("dip c", DIP_COL[2]), ("dip b", DIP_COL[3]), ("unknown", UNK)]
        for k, (s, col) in enumerate(lg):
            x = 6 + 88 * (k % 3)
            y = 16 + 18 * (k // 3)
            cv2.rectangle(img, (x, y - 10), (x + 12, y + 2), col, -1)
            _put(img, s, (x + 16, y))
    else:
        bar = cv2.applyColorMap(np.linspace(0, 255, 160).astype(np.uint8)[None, :].repeat(10, 0), cv2.COLORMAP_TURBO)
        img[8:18, 8:168] = bar
        _put(img, "-10", (6, 32))
        _put(img, "+20 cm", (130, 32))
    _put(img, "2.5D top (v: class/height, m: reset)", (6, size - 24 if size > 60 else 12), (200, 200, 200))
    return img


def oblique_image(Z, C, known, W=960, Hh=300, fwd_max=2.5, tilt=0.35):
    """비스듬히 본 2.5D: 먼 줄부터 그려 가까운 줄이 덮는다. 올라온 칸은 앞면을 어둡게, 모름은 비움.
    tilt = 내려다보는 정도(0 = 6판 시점 y_far 60·높이 350 px/m, 1 = 깊이에 세로 30 px 더·높이 220 px/m·먼 쪽 좌우 14 % 좁힘). 7판 키 [ ] 로 바꿈.
    7판 첫 시도(다 그린 뒤 warpPerspective로 사다리꼴 휘기 + 높이 180 px/m)는 실제 지도에서 1–2 px 줄이 빠지고 겹쳐 뭉개짐(사용자 '이전 판이 나을 지경')
    → 칸 하나 = 열 하나로 그린 뒤(6판과 같음) 세로는 그대로 두고 화면 줄마다 가로만 사다리꼴 폭으로 늘린다(cv2.remap, 가장 가까운 칸 — 줄이 빠지거나 겹치지 않음)."""
    tilt = float(np.clip(tilt, 0.0, 1.0))
    y_far = int(round(60 - 30 * tilt))
    zpx = 350.0 - 130.0 * tilt
    persp = 0.14 * tilt
    nf, nl = Z.shape
    img = np.full((Hh, nl, 3), 18, np.uint8)
    y_near = Hh - 14
    rowh = (y_near - y_far) / (nf - 1)
    ys = np.arange(Hh, dtype=np.float32)[:, None]
    inset = lambda y: persp / 2 * (1.0 - (y - y_far) / max(1.0, y_near - y_far))   # 그 줄 좌우 줄임(폭 비율, 한쪽)
    marks = []
    for r in np.arange(0.5, fwd_max + 1e-6, 0.5):       # 거리 눈금(지도 밑에 깔림)
        y = int(y_near - (y_near - y_far) * (r / fwd_max))
        img[y] = 55
        marks.append((r, y, inset(y)))
    for i in range(nf - 1, -1, -1):
        ok = known[i]
        if not ok.any():
            continue
        base = y_near - rowh * i
        z = np.where(ok, Z[i], 0.0)
        surf = base - np.clip(z, -0.15, 0.3) * zpx           # 그림만 30 cm에서 자름(높은 가구가 앞을 다 가리지 않게) — 분류와 무관
        col = C[i].astype(np.float32)
        top_y = np.minimum(surf, base)                   # 칸마다 [윗면 … 바닥선 + 줄 두께]를 채운다(잡음으로 줄 사이가 비지 않게)
        bot_y = np.maximum(surf, base) + rowh + 1
        lo = max(0, int(top_y[ok].min()) - 1)
        hi = min(Hh, int(bot_y[ok].max()) + 1)
        if lo >= hi:
            continue
        Y = ys[lo:hi]
        face = np.abs(z) > 0.01                          # 1 cm 넘게 올라오거나 꺼진 칸만 옆면을 어둡게
        sb = np.where(z < 0, bot_y - rowh - 1, surf + rowh + 1)   # 꺼진 칸: 위쪽이 안쪽 벽, 맨 아래가 바닥면
        m_s = ok & (Y >= top_y) & (Y < bot_y) & ~(face & ((z > 0) & (Y >= sb) | (z < 0) & (Y < sb)))
        m_f = ok & face & (Y >= top_y) & (Y < bot_y) & ~m_s
        seg = img[lo:hi]
        if m_f.any():
            seg[m_f] = np.broadcast_to(col * 0.55, m_f.shape + (3,))[m_f].astype(np.uint8)
        seg[m_s] = np.broadcast_to(col, m_s.shape + (3,))[m_s].astype(np.uint8)
    e = inset(np.arange(Hh, dtype=np.float32))[:, None]                  # 화면 줄마다 가로 사다리꼴
    mx = ((np.arange(W, dtype=np.float32)[None, :] + 0.5) / W - e) / (1.0 - 2 * e) * nl - 0.5
    img = cv2.remap(img, mx.astype(np.float32), np.repeat(np.arange(Hh, dtype=np.float32)[:, None], W, 1), cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=(18, 18, 18))
    for r, y, e in marks:
        _put(img, f"{r:.1f}m", (int(e * W) + 4, y - 2), (150, 150, 150))
    _put(img, f"2.5D oblique (view {tilt:.2f} [ ], height x{zpx / 87.5:.1f}, drawn up to 30 cm; gaps = unknown/occluded)", (6, 16), (200, 200, 200))
    return img
