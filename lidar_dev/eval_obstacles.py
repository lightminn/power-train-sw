# eval_obstacles.py — L2 결과(terrain_map.json·terrain_grid.npz)를 줄자 실측표(정답 JSON)와 맞대어 평가한다(10-23 계획 A5, 10-19).
# v2(같은 날, Codex 7차 검토 12개 반영): 매칭을 격자 칸 겹침으로 바꾸고(장애물 하나 = 정답 하나), 단계는 경로 허가 층(level)에서,
#   단차는 부호까지, 평가 못 한 것은 0이 아니라 '평가 불가'로 따로 센다. 사람 좌표 오차 민감도를 같이 낸다.
# 정답 JSON(장면 하나):
#   {"scene": "s2_boxes_a",
#    "objects": [{"id": "box1", "kind": "step_up", "height_m": 0.07, "x_m": 1.5, "y_m": 0.0, "size_m": [0.30, 0.40]}, ...],
#    "eval_region": {"x_m": [0.4, 2.5], "y_m": [-1.0, 1.0]}}
#   좌표는 L2 지도 좌표와 같다: 원점 = 시작 카메라 바로 아래 바닥, x = 시작 시 카메라 앞, y = 왼쪽(+). x_m·y_m = 물체 가운데,
#   size_m = [x 방향 길이, y 방향 폭](없으면 10 cm 네모로 가정하고 누출 검사 생략). height_m은 늘 양수(step_down이면 깊이).
#   kind = step_up(솟은 물체·단) | step_down(꺼진 곳 — 발자국 = 아래 면) | wall(넘지 못하는 높은 것).
#   eval_region = 오검출·바닥 가짜 위험·unknown을 세는 구역(정답에 안 적은 벽·가구는 구역 밖에 두기). 지도 밖 구역 칸은 unknown으로 센다.
# 판정:
#   - 칸 집합: 발자국 F(칸 중심이 발자국 안), 소유 구역 Z = F를 tol(사람 좌표 오차, 기본 10 cm)만큼 넓힌 것, 안쪽 C = F를 core_shrink만큼 줄인 것
#   - 장애물 소유: 장애물 칸(obstacle_id 층)이 Z와 겹치는 칸 수가 가장 많은 정답 하나가 갖는다(같으면 가운데가 가까운 쪽). 둘 이상의 Z와
#     발자국까지 덮으면 '병합'으로 표시 — 갖지 못한 정답은 검출이 아니라 병합으로 센다(소유 구역만 스친 것은 병합 아님). 어떤 Z와도 안 겹치고 구역 안에 칸이 있으면 오검출
#   - 검출 = 기대 주의·불가인 정답이 주의 이상 장애물을 하나 이상 가짐
#   - 단계(got) = F 안 경로 허가 층(level)의 관측된 칸 중 가장 나쁜 단계. F가 전부 unknown이면 '평가 불가'(맞음도 틀림도 아님)
#   - 기대 단계는 같은 기준값 JSON으로 정답 높이에서 계산. 높이가 가장 가까운 경계 ± border 안이면 그 경계 너머 단계도 '허용'
#     (5·7 cm 사이처럼 두 경계가 같은 거리면 나쁜 쪽 경계). 엄격 일치와 허용 일치를 따로 센다
#   - 단차 = 가진 장애물 중 Z와 가장 많이 겹친 것의 step_m, 부호로 방향 확인(step_up +, step_down −). 방향이 틀리면 오차를 내지 않는다
#   - 누출 = 기대 주의·불가인 정답의 C 안 경로 허가 '통과' 칸. C가 없거나(크기 없음·너무 작음) 지도 밖이거나 전부 unknown이면 검사 못 함으로 따로 센다
#   - 바닥 가짜 위험 = 구역 안, 정답 Z + ring 밖, 관측된 칸 중 국소 위험(level_local) 주의 이상
# 사용: python eval_obstacles.py <기준값 json> <지도 폴더 | map.npz> <정답 json> [...] [--out eval.json] [--jitter 0.05,3]
#   map.npz = 광민 실시간 지형 지도 재생 결과(python3 -m powertrain_ros.terrain_map_replay <bag> --out map.npz, 10-20 형식) — TerrainMapNpz
import argparse, itertools, json, math, os, statistics, sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # python -m lidar_dev.… 과 스크립트 실행 둘 다
from criteria import load_criteria  # noqa: E402

LEVEL = {"unknown": 0, "pass": 1, "caution": 2, "block": 3}
NAME = {v: k for k, v in LEVEL.items()}
CAUTION = LEVEL["caution"]


def expected_level(obj, rules, border_m):
    """정답 물체 → (기대 단계 이름, 허용 단계 집합). wall은 늘 불가. 허용은 가장 가까운 경계 하나만(같은 거리면 나쁜 쪽)."""
    if obj["kind"] == "wall":
        return "block", {"block"}
    r = rules["step_up_m" if obj["kind"] == "step_up" else "step_down_m"]
    h = abs(float(obj["height_m"]))
    lv = "block" if h >= r["block_from"] else "caution" if h >= r["caution_from"] else "pass"
    near = [(round(abs(h - thr), 6), -k, lo, hi) for k, (thr, lo, hi) in enumerate(((r["caution_from"], "pass", "caution"), (r["block_from"], "caution", "block")))
            if abs(h - thr) <= border_m]
    ok = {lv}
    if near:
        _, _, lo, hi = min(near)
        ok |= {lo, hi}
    return lv, ok


class Map:
    """terrain_map.json + terrain_grid.npz. 칸 (iy, ix)의 중심 = (x0 + (ix + 0.5)·c, y0 + (iy + 0.5)·c)."""

    def __init__(self, map_dir, crit):
        tm = json.load(open(os.path.join(map_dir, "terrain_map.json"), encoding="utf-8"))
        if tm.get("type") != "terrain_map" or not tm.get("alignment_valid"):
            raise SystemExit(f"{map_dir}: 정렬된 terrain_map이 아님 — 평가하지 않는다")
        if tm.get("criteria_version") != crit.get("criteria_version"):
            raise SystemExit(f"{map_dir}: 지도 기준값 {tm.get('criteria_version')} ≠ 평가 기준값 {crit.get('criteria_version')}")
        if tm.get("criteria_profile") != crit.get("criteria_profile"):   # v0.2부터: A 지도를 B 기준으로 평가하지 않는다(Codex 9차 #2)
            raise SystemExit(f"{map_dir}: 지도 프로필 {tm.get('criteria_profile')} ≠ 평가 프로필 {crit.get('criteria_profile')}")
        g = np.load(os.path.join(map_dir, "terrain_grid.npz"))
        if "obstacle_id" not in g.files:
            raise SystemExit(f"{map_dir}: terrain_grid에 obstacle_id 층이 없음(L2 v3.1 이전 출력) — L2를 다시 돌린다")
        self.dir, self.tm = map_dir, tm
        self.lv, self.loc, self.obs, self.oid = g["level"], g["level_local"], g["observed"].astype(bool), g["obstacle_id"]
        self.x0, self.y0, self.c = float(g["x0"]), float(g["y0"]), float(g["cell"])
        self.ny, self.nx = self.lv.shape
        self.obstacles = {o["id"]: o for o in tm["obstacles"]}

    def cells(self, x_lo, x_hi, y_lo, y_hi, pred=None):
        """격자 칸 중 중심이 사각형(그리고 pred) 안인 것 → (지도 안 마스크, 지도 밖까지 센 전체 칸 수)."""
        c = self.c
        i0, i1 = math.ceil((x_lo - self.x0) / c - 0.5), math.floor((x_hi - self.x0) / c - 0.5)
        j0, j1 = math.ceil((y_lo - self.y0) / c - 0.5), math.floor((y_hi - self.y0) / c - 0.5)
        m = np.zeros((self.ny, self.nx), bool)
        if i1 < i0 or j1 < j0:
            return m, 0
        I, J = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1))
        if pred is not None:
            keep = pred(self.x0 + (I + 0.5) * c, self.y0 + (J + 0.5) * c)
            I, J = I[keep], J[keep]
        inside = (I >= 0) & (I < self.nx) & (J >= 0) & (J < self.ny)
        m[J[inside], I[inside]] = True
        return m, int(np.size(I))


class TerrainMapNpz(Map):
    """광민 실시간 지형 지도의 재생 결과(map.npz, powertrain_ros.terrain_map_replay, 10-20 형식)를 같은 판정으로 평가한다.
    tiles [N,2](tx, ty) · heights_cm [N,256](-128 = 관측 없음) · codes [N,256](0 정상, 1 장애물(턱), 2 도랑·낙차, 4 관측 부족)
    · 타일 = 16×16칸, 칸 0.05 m, i번째 칸 gx = tx·16 + i%16, gy = ty·16 + i//16, 중심 ((gx+0.5)·0.05, (gy+0.5)·0.05),
    원점 = 노드(또는 epoch) 시작 때 로봇 위치, x 앞·y 왼쪽. 지도에 '주의' 단계가 없어 장애물·도랑은 불가로, 관측 부족·관측 없음은 미확인으로 본다.
    장애물 목록은 코드 1·2 칸을 8방향 이웃끼리 묶어 만들고, 단차 = 묶음의 최고 높이(코드 2는 최저, 음수)."""

    CELL = 0.05

    def __init__(self, path, crit=None):
        z = np.load(path)
        need = {"tiles", "heights_cm", "codes"}
        if not need <= set(z.files):
            raise SystemExit(f"{path}: map.npz 형식 아님(필요 {sorted(need)}, 있음 {z.files})")
        T, Hc, C = z["tiles"].astype(np.int64), z["heights_cm"].astype(np.int64), z["codes"].astype(np.int64)
        i = np.arange(256)
        gx = (T[:, :1] * 16 + i % 16).reshape(-1)
        gy = (T[:, 1:2] * 16 + i // 16).reshape(-1)
        h, code = Hc.reshape(-1), C.reshape(-1)
        self.dir, self.tm = path, {"type": "terrain_map_npz", "epoch": int(z["epoch"]) if "epoch" in z.files else None}
        self.c = self.CELL
        if len(gx) == 0:
            raise SystemExit(f"{path}: 타일이 없음")
        gx0, gy0 = int(gx.min()), int(gy.min())
        self.nx, self.ny = int(gx.max()) - gx0 + 1, int(gy.max()) - gy0 + 1
        self.x0, self.y0 = gx0 * self.c, gy0 * self.c
        ix, iy = gx - gx0, gy - gy0
        observed = (h != -128) & (code != 4)
        lv = np.zeros((self.ny, self.nx), np.int64)                      # 지도 밖 칸 = unknown(0)
        hz = np.full((self.ny, self.nx), np.nan)
        cd = np.full((self.ny, self.nx), -1)
        lv[iy, ix] = np.where(~observed, LEVEL["unknown"], np.where((code == 1) | (code == 2), LEVEL["block"], LEVEL["pass"]))
        hz[iy, ix] = np.where(h == -128, np.nan, h / 100.0)
        cd[iy, ix] = np.where(observed, code, -1)
        self.lv, self.loc = lv, lv.copy()
        self.obs = lv != LEVEL["unknown"]
        self.oid = np.full((self.ny, self.nx), -1, np.int64)
        self.obstacles = {}
        for k in (1, 2):
            seen = np.zeros_like(self.obs)
            for j0, i0 in zip(*np.nonzero(cd == k)):
                if seen[j0, i0]:
                    continue
                stack, comp = [(j0, i0)], []
                seen[j0, i0] = True
                while stack:
                    j, ii = stack.pop()
                    comp.append((j, ii))
                    for dj in (-1, 0, 1):
                        for di in (-1, 0, 1):
                            jj, kk = j + dj, ii + di
                            if 0 <= jj < self.ny and 0 <= kk < self.nx and not seen[jj, kk] and cd[jj, kk] == k:
                                seen[jj, kk] = True
                                stack.append((jj, kk))
                oid = len(self.obstacles)
                J, I = np.array(comp).T
                self.oid[J, I] = oid
                hs = hz[J, I]
                step = float(np.nanmax(hs)) if k == 1 else float(np.nanmin(hs))
                self.obstacles[oid] = {"id": oid, "level": "block", "kind": "obstacle" if k == 1 else "ditch", "cells": int(len(comp)),
                                       "step_m": round(step, 3) if np.isfinite(step) else None,
                                       "centroid_xy_m": [round(float(self.x0 + (I.mean() + 0.5) * self.c), 3), round(float(self.y0 + (J.mean() + 0.5) * self.c), 3)]}


def _rect(M, obj, grow):
    sx, sy = obj.get("size_m") or (0.10, 0.10)
    hx, hy = sx / 2 + grow, sy / 2 + grow
    if hx <= 0 or hy <= 0:
        return np.zeros((M.ny, M.nx), bool), 0
    return M.cells(obj["x_m"] - hx, obj["x_m"] + hx, obj["y_m"] - hy, obj["y_m"] + hy)


def eval_scene(M, gt, crit, tol_m=0.10, border_m=0.015, core_shrink_m=0.05, ring_m=0.10):
    if not isinstance(M, Map):
        M = Map(M, crit)
    objs = gt["objects"]
    zones = [_rect(M, o, tol_m)[0] for o in objs]
    feet = [_rect(M, o, 0.0) for o in objs]
    on_foot = [set(np.unique(M.oid[F & (M.oid >= 0)]).tolist()) for F, _ in feet]   # 발자국 칸까지 덮은 장애물(병합 판정용)
    # 장애물 소유: Z와 겹친 칸 수가 가장 많은 정답 하나
    overlap = {}   # 장애물 id → {정답 번호: 칸 수}
    for k, Z in enumerate(zones):
        ids, cnt = np.unique(M.oid[Z & (M.oid >= 0)], return_counts=True)
        for i, n in zip(ids.tolist(), cnt.tolist()):
            overlap.setdefault(i, {})[k] = n
    owner = {}
    for i, d in overlap.items():
        o = M.obstacles.get(i)
        dist = lambda k: math.hypot(o["centroid_xy_m"][0] - objs[k]["x_m"], o["centroid_xy_m"][1] - objs[k]["y_m"]) if o else 0.0
        owner[i] = min(d, key=lambda k: (-d[k], dist(k)))
    rows = []
    for k, obj in enumerate(objs):
        want, ok = expected_level(obj, crit["rules"], border_m)
        mine = sorted((i for i, w in owner.items() if w == k), key=lambda i: -overlap[i][k])
        hazard = [i for i in mine if i in M.obstacles and LEVEL[M.obstacles[i]["level"]] >= CAUTION]
        merged = [i for i in on_foot[k] if owner.get(i, k) != k and i in M.obstacles and LEVEL[M.obstacles[i]["level"]] >= CAUTION]
        F, f_total = feet[k]
        fv = M.lv[F]
        known = fv[fv > 0]
        got = NAME[int(known.max())] if known.size else "unknown"
        r = {"id": obj["id"], "kind": obj["kind"], "height_m": obj["height_m"], "expected": want, "accepted": sorted(ok), "border": len(ok) > 1,
             "got": got, "evaluable": got != "unknown",
             "level_strict": None if got == "unknown" else got == want, "level_ok": None if got == "unknown" else got in ok,
             "foot_cells": f_total, "foot_unknown_frac": round(1 - known.size / f_total, 3) if f_total else None,
             "size_assumed": not obj.get("size_m"),
             "detected": None if want == "pass" else bool(hazard), "merged_into": merged if (want != "pass" and not hazard and merged) else [],
             "pieces": len(mine), "step_m": None, "direction_ok": None}
        main = M.obstacles.get(mine[0]) if mine else None
        if main is not None and main.get("step_m") is not None and obj["kind"] != "wall":
            s, h = float(main["step_m"]), abs(float(obj["height_m"]))
            r["step_m"] = s
            r["direction_ok"] = s > 0 if obj["kind"] == "step_up" else s < 0
            if r["direction_ok"]:
                r["step_err_m"] = round(abs(s) - h, 4)
        if want != "pass":   # 안쪽 칸 누출
            if not obj.get("size_m"):
                r["leak"] = {"status": "no_size"}
            else:
                C, c_total = _rect(M, obj, -core_shrink_m)
                if c_total == 0:
                    r["leak"] = {"status": "too_small"}
                elif not C.any():
                    r["leak"] = {"status": "outside_map", "core_cells": c_total}
                elif not (M.lv[C] > 0).any():
                    r["leak"] = {"status": "unobserved", "core_cells": c_total}
                else:
                    cv = M.lv[C]
                    r["leak"] = {"status": "checked", "core_cells": c_total, "in_map": int(C.sum()),
                                 "pass_cells": int((cv == LEVEL["pass"]).sum()), "unknown_cells": int((cv == 0).sum()) + c_total - int(C.sum())}
        rows.append(r)
    out = {"scene": gt.get("scene", os.path.basename(M.dir.rstrip("/"))), "map_dir": M.dir, "objects": rows,
           "merged_obstacles": sorted(i for i in overlap if sum(i in f for f in on_foot) > 1)}
    reg = gt.get("eval_region")
    if reg:
        R, total = M.cells(reg["x_m"][0], reg["x_m"][1], reg["y_m"][0], reg["y_m"][1])
        outside = total - int(R.sum())
        floor = R.copy()
        for o in objs:
            floor &= ~_rect(M, o, tol_m + ring_m)[0]
        fps = [i for i in np.unique(M.oid[R & (M.oid >= 0)]).tolist() if i not in overlap and i in M.obstacles]
        out["region"] = {"cells": total, "outside_map_cells": outside,
                         "unknown_cells": int((M.lv[R] == 0).sum()) + outside, "unobserved_cells": int((~M.obs[R]).sum()) + outside,
                         "floor_cells": int(floor.sum()), "floor_false_risk_cells": int((floor & M.obs & (M.loc >= CAUTION)).sum()),
                         "false_obstacles": [{"id": i, "level": M.obstacles[i]["level"], "centroid_xy_m": M.obstacles[i]["centroid_xy_m"],
                                              "step_m": M.obstacles[i].get("step_m"), "cells": M.obstacles[i]["cells"]} for i in fps]}
        out["region"]["unknown_frac"] = round(out["region"]["unknown_cells"] / total, 4) if total else None
    return out


def summarize(scenes):
    """장면들을 합친 수치 — 분모를 같이 남긴다."""
    objs = [r for s in scenes for r in s["objects"]]
    det = [r for r in objs if r["detected"] is not None]
    ev = [r for r in objs if r["evaluable"]]
    stepped = [r for r in objs if r["direction_ok"] is not None]
    errs = [r["step_err_m"] for r in objs if "step_err_m" in r]
    leaks = [r["leak"] for r in objs if "leak" in r]
    chk = [x for x in leaks if x["status"] == "checked"]
    regs = [s["region"] for s in scenes if "region" in s]
    nonempty = [g for g in regs if g["cells"]]
    rep = {}
    for s in scenes:
        for r in s["objects"]:
            a = rep.setdefault(r["id"], {"attempts": 0, "steps": []})
            a["attempts"] += 1
            if "step_err_m" in r:
                a["steps"].append(abs(r["step_m"]))
    return {"scenes": len(scenes), "objects": len(objs),
            "detection": {"n": len(det), "detected": sum(r["detected"] for r in det), "merged_only": sum(bool(r["merged_into"]) for r in det)},
            "level": {"n": len(objs), "evaluable": len(ev), "unevaluable": len(objs) - len(ev),
                      "strict_ok": sum(r["level_strict"] for r in ev), "accepted_ok": sum(r["level_ok"] for r in ev),
                      "border_cases": sum(r["border"] for r in objs)},
            "over_warning_on_pass_objects": sum(1 for r in ev if r["expected"] == "pass" and LEVEL[r["got"]] >= CAUTION and not r["level_ok"]),
            "direction": {"n": len(stepped), "ok": sum(r["direction_ok"] for r in stepped)},
            "step_err_m": {"n": len(errs), "mean": round(statistics.mean(errs), 4) if errs else None,
                           "max_abs": round(max(map(abs, errs)), 4) if errs else None},
            "leak": {"objects": len(leaks), "checked": len(chk), "not_checked": {k: sum(x["status"] == k for x in leaks) for k in ("no_size", "too_small", "outside_map", "unobserved")},
                     "core_cells": sum(x["core_cells"] for x in chk), "pass_cells": sum(x["pass_cells"] for x in chk),
                     "unknown_cells": sum(x["unknown_cells"] for x in chk)},
            "false_obstacles": {"scenes_with_region": len(regs), "n": sum(len(g["false_obstacles"]) for g in regs)},
            "floor_false_risk_cells": {"n": sum(g["floor_false_risk_cells"] for g in regs), "of_floor_cells": sum(g["floor_cells"] for g in regs)},
            "unknown": {"cells": sum(g["unknown_cells"] for g in regs), "of_region_cells": sum(g["cells"] for g in regs),
                        "pooled_frac": round(sum(g["unknown_cells"] for g in regs) / sum(g["cells"] for g in regs), 4) if nonempty else None,
                        "scene_mean_frac": round(statistics.mean(g["unknown_frac"] for g in nonempty), 4) if nonempty else None,
                        "empty_regions": len(regs) - len(nonempty)},
            "repeat_step_m": {k: {"attempts": a["attempts"], "measured": len(a["steps"]), "missed": a["attempts"] - len(a["steps"]),
                                  "mean": round(statistics.mean(a["steps"]), 4) if a["steps"] else None,
                                  "sd": round(statistics.stdev(a["steps"]), 4) if len(a["steps"]) > 1 else None}
                              for k, a in rep.items() if a["attempts"] > 1}}


def shift_gt(gt, dx, dy, yaw_deg):
    """정답 가운데를 원점(시작 카메라 아래) 기준 yaw만큼 돌리고 (dx, dy) 옮긴 사본 — 사람 좌표 오차 흉내."""
    ca, sa = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    objs = [{**o, "x_m": ca * o["x_m"] - sa * o["y_m"] + dx, "y_m": sa * o["x_m"] + ca * o["y_m"] + dy} for o in gt["objects"]]
    return {**gt, "objects": objs}


def sensitivity(maps, gts, crit, d_m, yaw_deg, **kw):
    """앞·옆 {−d, 0, +d} × yaw {−θ, 0, +θ} 27조합으로 다시 평가 → 주요 합계의 최소·최대."""
    keys = {"detected": lambda s: s["detection"]["detected"], "accepted_ok": lambda s: s["level"]["accepted_ok"],
            "strict_ok": lambda s: s["level"]["strict_ok"], "leak_pass_cells": lambda s: s["leak"]["pass_cells"],
            "false_obstacles": lambda s: s["false_obstacles"]["n"], "floor_false_risk_cells": lambda s: s["floor_false_risk_cells"]["n"]}
    vals = {k: [] for k in keys}
    for dx, dy, yw in itertools.product((-d_m, 0.0, d_m), (-d_m, 0.0, d_m), (-yaw_deg, 0.0, yaw_deg)):
        sm = summarize([eval_scene(M, shift_gt(g, dx, dy, yw), crit, **kw) for M, g in zip(maps, gts)])
        for k, f in keys.items():
            vals[k].append(f(sm))
    return {"shift_m": d_m, "yaw_deg": yaw_deg, "combos": 27, **{k: [min(v), max(v)] for k, v in vals.items()}}


def gt_from_synth_truth(path, region=None):
    """synth_seq.py의 truth.json(상자 (x0, x1, y0, y1, 높이), 시작 카메라 = 월드 원점·앞 = +x) → 정답 JSON. 시험용."""
    t = json.load(open(path))
    objs = [{"id": f"box{k}_{h * 100:g}cm", "kind": "step_up", "height_m": h, "x_m": (x0 + x1) / 2, "y_m": (y0 + y1) / 2, "size_m": [x1 - x0, y1 - y0]}
            for k, (x0, x1, y0, y1, h) in enumerate(t["boxes"])]
    return {"scene": "synthetic", "objects": objs, **({"eval_region": region} if region else {})}


def main():
    ap = argparse.ArgumentParser(description="L2 결과를 줄자 실측표(정답 JSON)와 비교")
    ap.add_argument("criteria")
    ap.add_argument("pairs", nargs="+", help="<지도 폴더> <정답 json> 쌍")
    ap.add_argument("--out")
    ap.add_argument("--tol", type=float, default=0.10, help="정답 발자국을 넓혀 장애물 소유를 가르는 사람 좌표 오차(m)")
    ap.add_argument("--border", type=float, default=0.015, help="경계 허용(m): 줄자 오차 + L2 측정 여유")
    ap.add_argument("--jitter", default="0.05,3", help="좌표 오차 민감도 '앞·옆 m,yaw °'(0이면 생략)")
    ap.add_argument("--profile", help="기준값 프로필(v0.2: A_no_stop 기본 · B_stop20)")
    a = ap.parse_args()
    if len(a.pairs) % 2:
        ap.error("지도 폴더와 정답 json을 쌍으로 준다")
    crit = load_criteria(a.criteria, a.profile)
    maps = [TerrainMapNpz(m) if m.endswith(".npz") else Map(m, crit) for m in a.pairs[::2]]   # .npz = 광민 실시간 지도 재생 결과
    gts = [json.load(open(g, encoding="utf-8")) for g in a.pairs[1::2]]
    kw = {"tol_m": a.tol, "border_m": a.border}
    scenes = [eval_scene(M, g, crit, **kw) for M, g in zip(maps, gts)]
    res = {"criteria_version": crit.get("criteria_version"), "criteria_profile": crit.get("criteria_profile"),
           "criteria_sha256": crit.get("criteria_sha256"), "params": {**kw, "jitter": a.jitter}, "summary": summarize(scenes), "scenes": scenes}
    d, _, yw = a.jitter.partition(",")
    if float(d or 0) > 0 or float(yw or 0) > 0:
        res["sensitivity"] = sensitivity(maps, gts, crit, float(d or 0), float(yw or 0), **kw)
    for s in scenes:
        print(f"[{s['scene']}]")
        for r in s["objects"]:
            lk = r.get("leak", {})
            print("  %-12s %-9s h %5.3f  기대 %-7s 결과 %-7s %s%s  검출 %s  단차 %s  조각 %d  누출 %s" % (
                r["id"], r["kind"], r["height_m"], r["expected"], r["got"],
                "평가 불가" if not r["evaluable"] else ("맞음" if r["level_strict"] else "허용" if r["level_ok"] else "틀림"), "(경계)" if r["border"] else "",
                "-" if r["detected"] is None else ("예" if r["detected"] else "병합" if r["merged_into"] else "아니오"),
                "-" if r["step_m"] is None else "%+.3f(%s)" % (r["step_m"], "%+.3f" % r["step_err_m"] if "step_err_m" in r else "방향 틀림"), r["pieces"],
                "-" if not lk else ("%d/%d칸" % (lk["pass_cells"], lk["core_cells"]) if lk["status"] == "checked" else lk["status"])))
        if "region" in s:
            g = s["region"]
            print("  구역 %d칸(지도 밖 %d): unknown %s, 바닥 가짜 위험 %d/%d칸, 오검출 %d개" % (
                g["cells"], g["outside_map_cells"], "-" if g["unknown_frac"] is None else "%.1f%%" % (100 * g["unknown_frac"]),
                g["floor_false_risk_cells"], g["floor_cells"], len(g["false_obstacles"])))
    print(json.dumps(res["summary"], ensure_ascii=False))
    if "sensitivity" in res:
        print("좌표 오차 민감도:", json.dumps(res["sensitivity"], ensure_ascii=False))
    if a.out:
        json.dump(res, open(a.out, "w"), ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
